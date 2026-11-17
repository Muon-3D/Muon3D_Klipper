"""A non-critical MCU's shutdown must not deadlock the serial thread.

On sloe (2026-11-11 19:52) the toolhead MCU reported "Timer too close"
mid-print. `MCUConnectHelper._handle_shutdown` runs on the serial background
thread with `SerialReader.lock` held, and it called
`handle_non_critical_disconnect` directly. That calls `ClockSync.disconnect`,
which calls `register_response(None, 'clock')` and takes the same
non-reentrant lock. The serial thread blocked on itself, the reactor then
blocked on that lock the next time it touched the toolhead's handlers, and
Klipper stopped answering anything -- the print stopped and Moonraker's
'info' request stayed pending for 13 hours. The 17:02 and 18:38 shutdowns
that day hung the same way.

This drives the real `SerialReader` lock and the real `ClockSync.disconnect`
the way `SerialReader._bg_thread` does, with a timeout in place of a hang.

    python3 test/serialqueue/test_noncritical_shutdown_deadlock.py
"""
import os
import sys
import threading

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

import clocksync  # noqa: E402
import mcu  # noqa: E402
import serialhdl  # noqa: E402


class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.async_callbacks = []
    def register_async_callback(self, callback, waketime=NOW):
        self.async_callbacks.append(callback)
    def update_timer(self, timer, waketime):
        pass
    def run_async(self):
        callbacks, self.async_callbacks = self.async_callbacks, []
        for cb in callbacks:
            cb(1000.)


class FakePrinter:
    def __init__(self):
        self.events = []
    def send_event(self, name, *args):
        self.events.append(name)


class FakeMCU:
    non_critical_disconnected = False
    _connecting = False
    def clock32_to_clock64(self, clock32):
        return clock32


def make_reader(reactor):
    reader = object.__new__(serialhdl.SerialReader)
    reader.reactor = reactor
    reader.lock = threading.Lock()
    reader.handlers = {}
    reader.serialqueue = None
    reader.serial_dev = None
    reader.background_thread = None
    reader.pending_notifications = {}
    reader.dump_debug = lambda: "serial dump"
    return reader


def make_helper(reactor, reader):
    cs = object.__new__(clocksync.ClockSync)
    cs.reactor = reactor
    cs.serial = reader
    cs.get_clock_timer = None
    cs.queries_pending = 0
    cs.dump_debug = lambda: "clocksync dump"
    reader.register_response(lambda params: None, 'clock')

    h = object.__new__(mcu.MCUConnectHelper)
    h._name = 'toolhead'
    h._reactor = reactor
    h._printer = FakePrinter()
    h._mcu = FakeMCU()
    h._serial = reader
    h._clocksync = cs
    h._is_shutdown = False
    h._shutdown_msg = ""
    h.is_non_critical = True
    h.non_critical_recon_timer = None
    h.reconnect_interval = 2.
    h._non_critical_disconnect_event_name = \
        "danger:non_critical_mcu_toolhead:disconnected"
    return h


def deliver_like_bg_thread(reader, params):
    # SerialReader._bg_thread: `with self.lock: hdl(params)`
    hdl = reader.handlers[(params['#name'], None)]
    with reader.lock:
        hdl(params)


def test_shutdown_does_not_deadlock_serial_thread():
    reactor = FakeReactor()
    reader = make_reader(reactor)
    h = make_helper(reactor, reader)
    reader.register_response(h._handle_shutdown, 'shutdown')
    params = {'#name': 'shutdown', 'clock': 597383684,
              'static_string_id': 'Timer too close'}

    t = threading.Thread(target=deliver_like_bg_thread, args=(reader, params),
                         daemon=True)
    t.start()
    t.join(5.)
    assert not t.is_alive(), \
        "serial thread deadlocked handling a non-critical shutdown"
    assert not reader.lock.locked(), "serial lock left held"

    # The disconnect itself happens on the reactor, outside the lock.
    assert not h._mcu.non_critical_disconnected
    reactor.run_async()
    assert h._mcu.non_critical_disconnected
    assert ('clock', None) not in reader.handlers
    assert h._printer.events == [h._non_critical_disconnect_event_name]


def test_a_failing_dump_still_disconnects():
    # The debug dump is only for the log. If it raises, the disconnect must
    # still be handed to the reactor: _is_shutdown is already set, so a
    # repeat shutdown message would be ignored and the mcu would stay
    # "connected" but dead.
    reactor = FakeReactor()
    reader = make_reader(reactor)
    h = make_helper(reactor, reader)
    def boom():
        raise RuntimeError("dump failed")
    reader.dump_debug = boom
    params = {'#name': 'shutdown', 'clock': 597383684,
              'static_string_id': 'Timer too close'}
    h._handle_shutdown(params)
    assert len(reactor.async_callbacks) == 1, \
        "disconnect was not scheduled when the dump raised"
    reactor.run_async()
    assert h._mcu.non_critical_disconnected


def main():
    test_shutdown_does_not_deadlock_serial_thread()
    test_a_failing_dump_still_disconnects()
    print("ok: non-critical MCU shutdown leaves the serial thread free")
    # A deadlocked daemon thread would otherwise keep the process alive.
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
