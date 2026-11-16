"""A non-critical toolhead MCU going away must end a Z move (KAN-3370, C).

feat/m1-print-start runs the toolhead (the load cell and the tap
accelerometer) as a non-critical MCU.  Two parts of the guard live in
klippy/mcu.py and are driven here with the real code:

* MCUConnectHelper.handle_non_critical_disconnect sends
  "danger:non_critical_mcu_toolhead:disconnected", and PrinterHoming, which
  finds the MCU through the real MCU.get_non_critical_disconnect_event_name,
  aborts the Z move on it.  The same happens when it arrives the way a
  toolhead "Timer too close" does: from the serial thread, holding
  SerialReader.lock, handed to the reactor (the deadlock test's setup).
  Aborting only completes the move's trigger completions, so it takes no
  lock and cannot deadlock; a thread with a timeout proves it.
* TriggerDispatch.stop stops every trsync, even when the offline MCU's
  raises, so the Z stepper's endstop is stopped, and still reports the
  failure.

pyserial, cffi and the C helper are not needed: serial, cffi and the
POSIX-only pty modules are stubbed if missing (a Windows checkout), and
chelper.get_ffi is replaced for the stop test.

    python3 test/homing_abort/test_toolhead_disconnect.py
"""
import os
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'serialqueue'))

for _name in ('serial', 'cffi', 'termios', 'fcntl', 'pty'):
    try:
        __import__(_name)
    except Exception:
        sys.modules[_name] = types.ModuleType(_name)

import fakes  # noqa: E402
from fakes import Rig, check, run  # noqa: E402
import mcu  # noqa: E402
import test_noncritical_shutdown_deadlock as dl  # noqa: E402

TOOLHEAD_MSG = "Homing aborted: the toolhead disconnected"
EVENT = "danger:non_critical_mcu_toolhead:disconnected"


def z_home(rig):
    return rig.hmove().homing_move([0., 0., -4., 0.], 15., probe_pos=True)


def wire(rig, reactor, reader):
    """The real helper and real MCU facade, on the rig's printer."""
    h = dl.make_helper(reactor, reader)
    h._printer = rig.printer
    facade = object.__new__(mcu.MCU)
    facade._conn_helper = h
    facade.is_non_critical = True
    rig.printer.objects['mcu toolhead'] = facade
    rig.printer.send_event('klippy:connect')
    rig.printer.events = []
    return h


def test_event_name_comes_from_the_real_mcu():
    reactor = dl.FakeReactor()
    rig = Rig(n_endstops=1)
    wire(rig, reactor, dl.make_reader(reactor))
    check(EVENT in rig.phoming.printer.handlers,
          "D1: PrinterHoming listens for %r" % (EVENT,))
    st = rig.phoming.get_status(0.)
    # In this tree the cap is off by default (the sibling's was 40 mm), and
    # abort_on_disconnect is a flag, with the MCU names beside it.
    check(st['max_search_distance_z'] == 0.,
          "D1: status shows the cap, off (got %r)" % (st,))
    check(st['abort_on_disconnect'] is True
          and st['abort_on_disconnect_mcus'] == ['mcu toolhead']
          and 'non_critical_disconnect: mcu toolhead' in st['abort_sources'],
          "D1: status shows the guarded MCU (got %r)" % (st,))


def test_disconnect_from_the_reactor_aborts_the_move():
    reactor = dl.FakeReactor()
    reader = dl.make_reader(reactor)
    rig = Rig(n_endstops=2)
    h = wire(rig, reactor, reader)
    rig.toolhead.on_tick = lambda mm: (
        h.handle_non_critical_disconnect() if mm == 6 else None)
    res, err = run(lambda: z_home(rig))
    check(err == TOOLHEAD_MSG, "D2: error %r (got %r)" % (TOOLHEAD_MSG, err))
    check(rig.toolhead.travelled_mm == 6, "D2: the move stopped at 6 mm")
    check(h._mcu.non_critical_disconnected, "D2: the MCU is marked offline")
    check(not reader.lock.locked(), "D2: the serial lock is free")
    check(rig.phoming.active_moves == [], "D2: no move left active")


def test_shutdown_from_the_serial_thread_aborts_without_deadlock():
    reactor = dl.FakeReactor()
    reader = dl.make_reader(reactor)
    rig = Rig(n_endstops=2)
    h = wire(rig, reactor, reader)
    reader.register_response(h._handle_shutdown, 'shutdown')
    params = {'#name': 'shutdown', 'clock': 597383684,
              'static_string_id': 'Timer too close'}
    t = threading.Thread(target=dl.deliver_like_bg_thread,
                         args=(reader, params), daemon=True)
    t.start()
    t.join(5.)
    check(not t.is_alive(), "D3: the serial thread did not deadlock")
    check(not reader.lock.locked(), "D3: the serial lock was released")
    check(not h._mcu.non_critical_disconnected,
          "D3: the disconnect has not run yet (it is on the reactor)")

    # the reactor now runs it, mid Z move, in a thread with a timeout
    rig.toolhead.on_tick = lambda mm: (
        reactor.run_async() if mm == 4 else None)
    out = {}

    def move():
        out['res'] = run(lambda: z_home(rig))
    t = threading.Thread(target=move, daemon=True)
    t.start()
    t.join(5.)
    check(not t.is_alive(), "D3: the aborted Z move did not deadlock")
    res, err = out.get('res', (None, None))
    check(err == TOOLHEAD_MSG, "D3: error %r (got %r)" % (TOOLHEAD_MSG, err))
    check(rig.toolhead.travelled_mm == 4, "D3: the move stopped at 4 mm")
    check(h._mcu.non_critical_disconnected, "D3: the MCU is offline")


class FakeTrsync:
    def __init__(self, name, offline, fail, reason=1):
        self.mcu = types.SimpleNamespace(
            non_critical_disconnected=offline,
            get_name=lambda: name)
        self.fail = fail
        self.reason = reason
        self.stopped = False

    def get_mcu(self):
        return self.mcu

    def stop(self):
        self.stopped = True
        if self.fail:
            raise RuntimeError("MCU is currently disconnected")
        return self.reason


class FakeLib:
    def __init__(self):
        self.stops = 0

    def trdispatch_stop(self, trdispatch):
        self.stops += 1


def dispatch(trsyncs):
    td = object.__new__(mcu.TriggerDispatch)
    td._trdispatch = object()
    td._trsyncs = trsyncs
    return td


def test_stop_stops_every_trsync():
    lib = FakeLib()
    real = mcu.chelper.get_ffi
    mcu.chelper.get_ffi = lambda: (None, lib)
    try:
        # offline toolhead listed FIRST: the Z MCU must still be stopped
        tool = FakeTrsync('toolhead', True, True)
        z = FakeTrsync('mcu', False, False, reason=1)
        res = dispatch([tool, z]).stop()
        check(tool.stopped and z.stopped,
              "S1: both trsyncs were stopped")
        check(res == 4, "S1: the offline MCU is a comms timeout (got %r)"
              % (res,))
        check(lib.stops == 1, "S1: trdispatch_stop ran once")
        # an online critical MCU's failure is still raised, after the rest
        bad = FakeTrsync('mcu', False, True)
        ok = FakeTrsync('mcu2', False, False)
        try:
            dispatch([bad, ok]).stop()
            raised = None
        except RuntimeError as e:
            raised = str(e)
        check(raised == "MCU is currently disconnected" and ok.stopped,
              "S2: a critical MCU's error is raised, others stopped first")
        # nothing wrong: the first trsync's reason, as before
        a = FakeTrsync('mcu', False, False, reason=1)
        b = FakeTrsync('toolhead', False, False, reason=3)
        check(dispatch([a, b]).stop() == 1,
              "S3: healthy trsyncs return the first reason")
    finally:
        mcu.chelper.get_ffi = real


def main():
    test_event_name_comes_from_the_real_mcu()
    test_disconnect_from_the_reactor_aborts_the_move()
    test_shutdown_from_the_serial_thread_aborts_without_deadlock()
    test_stop_stops_every_trsync()
    sys.stdout.flush()
    if fakes.FAILED:
        print("%d of %d checks FAILED" % (len(fakes.FAILED),
                                          len(fakes.CHECKS)))
        os._exit(1)
    print("all %d checks passed" % (len(fakes.CHECKS),))
    sys.stdout.flush()
    # a deadlocked daemon thread would otherwise keep the process alive
    os._exit(0)


if __name__ == '__main__':
    main()
