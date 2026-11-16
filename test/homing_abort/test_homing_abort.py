"""Aborting a homing or probing move in progress (KAN-3370, guard C).

On Boxwood a load cell fault meant the Z search never triggered, ran 164 mm
and lifted the bed off its mount.  Bed removal was detected 1 s in, but
HOMING_GATE only looks at the flag before motion.  PrinterHoming now ends
a Z homing or probing move at once, from bed removal and from the toolhead
MCU disconnecting.

Drives the real HomingMove, PrinterHoming and BedRemovalDetector with
fakes for the reactor, toolhead, steppers and endstops (see fakes.py).
Constants are written out here, not imported from the code under test.

    python3 test/homing_abort/test_homing_abort.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fakes  # noqa: E402
from fakes import Rig, check, run, FakeMcuObject  # noqa: E402

from extras import bed_removal_detector  # noqa: E402

REMOVED = -83.76
BED_REMOVED_MSG = "Homing aborted: the bed was removed"
TOOLHEAD_MSG = "Homing aborted: the toolhead disconnected"
DISCONNECT_EVENT = "danger:non_critical_mcu_toolhead:disconnected"


def z_home(rig, probe_pos=False):
    return rig.hmove().homing_move([0., 0., -4., 0.], 15.,
                                   probe_pos=probe_pos)


def abort_at(rig, tick, reason="the bed was removed", **kw):
    seen = []

    def on_tick(mm):
        if mm == tick:
            seen.append(rig.phoming.abort_active_move(reason, **kw))
    rig.toolhead.on_tick = on_tick
    return seen


# --- detector scaffolding ------------------------------------------------

class FakeHeater:
    def __init__(self):
        self.temp = 60.
        self.target = 60.
        # The detector decides on fresh samples: it takes last_temp each
        # time last_temp_time moves on.
        self.last_temp_time = 0.

    @property
    def last_temp(self):
        return self.temp

    def get_temp(self, eventtime):
        return self.temp, self.target

    def set_temp(self, degrees):
        self.target = degrees


class FakeBed:
    def __init__(self, heater):
        self.heater = heater


class FakeDetectorGCode:
    error = fakes.CommandError

    def __init__(self):
        self.scripts = []

    def run_script(self, script):
        self.scripts.append(script)

    def register_command(self, name, func, desc=None):
        pass


class FakePauseResume:
    is_paused = False

    def send_pause_command(self):
        pass


class DetectorConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default=None, **kw):
        return default

    def getint(self, name, default=None, **kw):
        return default


def make_detector(printer):
    heater = FakeHeater()
    gcode = FakeDetectorGCode()
    printer.objects['heater_bed'] = FakeBed(heater)
    printer.objects['gcode'] = gcode
    printer.objects['pause_resume'] = FakePauseResume()
    det = bed_removal_detector.load_config(DetectorConfig(printer))
    return det, heater, gcode


def feed_removal(det, heater):
    heater.temp = REMOVED
    for i in range(5):
        heater.last_temp_time = 100. + 0.3 * i
        det.check_bed_temperature(100.)


# --- tests ---------------------------------------------------------------

def test_c1_abort_during_active_z_homing_move():
    for probe_pos in (False, True):
        tag = "probe_pos=%s" % (probe_pos,)
        rig = Rig(n_endstops=2)
        seen = abort_at(rig, 5)
        res, err = run(lambda: z_home(rig, probe_pos))
        check(seen == [True], "C1 %s: abort returned True" % (tag,))
        check(err == BED_REMOVED_MSG,
              "C1 %s: error is %r (got %r)" % (tag, BED_REMOVED_MSG, err))
        check(rig.toolhead.travelled_mm == 5,
              "C1 %s: drip ended after exactly 5 mm" % (tag,))
        check(len(rig.toolhead.drip_calls) == 1,
              "C1 %s: one drip move, not a retry" % (tag,))
        check([e.waited for e in rig.endstops] == [1, 1],
              "C1 %s: home_wait called once for each endstop" % (tag,))
        check(all(not e.triggered for e in rig.endstops),
              "C1 %s: never reported as a trigger" % (tag,))
        check(rig.printer.events == ["homing:homing_move_begin",
                                     "homing:homing_move_end"],
              "C1 %s: homing_move_end still fired" % (tag,))
        check(rig.toolhead.pos[2] == 155.0,
              "C1 %s: toolhead Z is where the steppers stopped, 155 (got "
              "%r)" % (tag, rig.toolhead.pos[2]))
        check(rig.phoming.active_moves == [],
              "C1 %s: move deregistered afterwards" % (tag,))


def test_c2_abort_with_no_active_move():
    rig = Rig()
    check(rig.phoming.abort_active_move("the bed was removed") is False,
          "C2: abort with no move returns False")
    check(rig.toolhead.set_positions == [],
          "C2: no position was set")
    check(rig.printer.events == [], "C2: no event fired")
    check(rig.phoming.active_moves == [], "C2: nothing registered")
    # And after a move has finished
    res, err = run(lambda: rig.hmove().homing_move(
        [0., 0., 148., 0.], 15., probe_pos=True, check_triggered=False))
    check(err is None, "C2: a plain move afterwards still works")
    check(rig.phoming.abort_active_move("late") is False,
          "C2: abort after the move ended returns False")


def test_c3_x_only_move_is_not_aborted_by_default():
    rig = Rig(axis=0, start=100., trigger_after_mm=10)
    seen = abort_at(rig, 3)
    res, err = run(lambda: rig.hmove().homing_move(
        [0., 0., 0., 0.], 60.))
    check(seen == [False], "C3: default scope abort returned False")
    check(err is None, "C3: X move completed without error (got %r)"
          % (err,))
    check(rig.endstops[0].triggered, "C3: X endstop triggered normally")
    # The same abort with Z-only switched off does reach an X move
    rig = Rig(axis=0, start=100., trigger_after_mm=50)
    seen = abort_at(rig, 3, z_only=False)
    res, err = run(lambda: rig.hmove().homing_move(
        [0., 0., 0., 0.], 60.))
    check(seen == [True] and err == BED_REMOVED_MSG,
          "C3: z_only=False aborts an X move too (got %r)" % (err,))
    check(rig.toolhead.travelled_mm == 3, "C3: X move stopped at 3 mm")


def test_c4_bed_removal_calls_the_abort():
    # a) end to end: the real detector ends a real Z move
    rig = Rig(n_endstops=1)
    det, heater, gcode = make_detector(rig.printer)
    rig.toolhead.on_tick = lambda mm: (
        feed_removal(det, heater) if mm == 4 else None)
    res, err = run(lambda: z_home(rig, True))
    check(err == BED_REMOVED_MSG,
          "C4: removal ended the move with %r (got %r)"
          % (BED_REMOVED_MSG, err))
    check(rig.toolhead.travelled_mm == 4, "C4: move stopped at 4 mm")
    check(heater.target == 0.0, "C4: bed heater still goes off")
    check("bed_removal_detector:removed" in rig.printer.events,
          "C4: the removed event still fires")
    # b) the exact call
    calls = []

    class FakeHoming:
        def abort_active_move(self, reason, z_only=True):
            calls.append((reason, z_only))
            return True
    rig2 = Rig()
    rig2.printer.objects['homing'] = FakeHoming()
    det, heater, gcode = make_detector(rig2.printer)
    feed_removal(det, heater)
    check(calls == [("the bed was removed", True)],
          "C4: abort called once with the reason (got %r)" % (calls,))

    # c) a failing abort cannot skip the rest of the removal handling
    class BrokenHoming:
        def abort_active_move(self, reason, z_only=True):
            raise RuntimeError("boom")
    rig3 = Rig()
    rig3.printer.objects['homing'] = BrokenHoming()
    det, heater, gcode = make_detector(rig3.printer)
    feed_removal(det, heater)
    check(heater.target == 0.0 and
          "bed_removal_detector:removed" in rig3.printer.events
          and det.bed_removed,
          "C4: a failing abort still lets the heater-off and event run")
    # d) no homing object at all (older configs, other tests)
    rig4 = Rig()
    del rig4.printer.objects['homing']
    det, heater, gcode = make_detector(rig4.printer)
    feed_removal(det, heater)
    check(heater.target == 0.0 and det.bed_removed,
          "C4: no homing object is not an error")


def test_c5_toolhead_disconnect_calls_the_abort():
    rig = Rig(n_endstops=2)
    rig.printer.objects['mcu'] = FakeMcuObject('mcu', False)
    rig.printer.objects['mcu toolhead'] = FakeMcuObject('toolhead', True)
    rig.printer.send_event('klippy:connect')
    rig.printer.events = []
    rig.toolhead.on_tick = lambda mm: (
        rig.printer.send_event(DISCONNECT_EVENT) if mm == 6 else None)
    res, err = run(lambda: z_home(rig, True))
    check(err == TOOLHEAD_MSG,
          "C5: disconnect ended the move with %r (got %r)"
          % (TOOLHEAD_MSG, err))
    check(rig.toolhead.travelled_mm == 6, "C5: move stopped at 6 mm")
    check([e.waited for e in rig.endstops] == [1, 1],
          "C5: home_wait called for every endstop")
    check(rig.toolhead.pos[2] == 154.0,
          "C5: toolhead Z is where the steppers stopped, 154 (got %r)"
          % (rig.toolhead.pos[2],))
    check(rig.printer.events[-1] == "homing:homing_move_end",
          "C5: homing_move_end fired")
    # The endstop MCU is gone: home_wait raises something that is not a
    # command error.  Cleanup must still finish, and the error is still ours
    gone = RuntimeError("non-critical MCU offline")
    rig = Rig(n_endstops=2, wait_errors=[gone, None])
    rig.printer.objects['mcu toolhead'] = FakeMcuObject('toolhead', True)
    rig.printer.send_event('klippy:connect')
    rig.toolhead.on_tick = lambda mm: (
        rig.printer.send_event(DISCONNECT_EVENT) if mm == 2 else None)
    res, err = run(lambda: z_home(rig, True))
    check(err == TOOLHEAD_MSG, "C5: offline endstop MCU: still %r (got %r)"
          % (TOOLHEAD_MSG, err))
    check([e.waited for e in rig.endstops] == [1, 1],
          "C5: offline endstop MCU: the other endstop was still stopped")
    check(rig.printer.events[-1] == "homing:homing_move_end",
          "C5: offline endstop MCU: homing_move_end fired")
    # A critical MCU going away is not this path's business
    rig = Rig()
    rig.printer.objects['mcu'] = FakeMcuObject('mcu', False)
    rig.printer.send_event('klippy:connect')
    check("danger:non_critical_mcu_mcu:disconnected"
          not in rig.printer.handlers,
          "C5: no handler is registered for a critical MCU")


def test_c6_probing_move_is_aborted_too():
    rig = Rig(start=10.)
    seen = abort_at(rig, 3)
    res, err = run(lambda: rig.phoming.probing_move(
        rig.endstops[0], [0., 0., -4., 0.], 5.))
    check(seen == [True], "C6: abort returned True during a probe")
    check(err == BED_REMOVED_MSG,
          "C6: probing_move raised %r (got %r)" % (BED_REMOVED_MSG, err))
    check(res is None, "C6: no probe position was returned")
    check(rig.toolhead.travelled_mm == 3, "C6: probe move stopped at 3 mm")
    check(rig.toolhead.pos[2] == 7.0,
          "C6: toolhead Z is where the steppers stopped, 7 (got %r)"
          % (rig.toolhead.pos[2],))
    check(rig.endstops[0].waited == 1, "C6: probe endstop home_wait called")


def test_abort_before_the_drip_starts():
    # Removal that lands while the move is still being set up
    rig = Rig()
    orig = rig.toolhead.flush_step_generation
    state = {'n': 0}

    def flush():
        state['n'] += 1
        if state['n'] == 1:
            rig.phoming.abort_active_move("the bed was removed")
        orig()
    rig.toolhead.flush_step_generation = flush
    res, err = run(lambda: z_home(rig, True))
    check(err == BED_REMOVED_MSG, "setup abort raised %r (got %r)"
          % (BED_REMOVED_MSG, err))
    check(rig.toolhead.drip_calls == [], "setup abort: no step was issued")
    check(rig.endstops[0].waited == 1,
          "setup abort: endstop still cleaned up")


def main():
    test_c1_abort_during_active_z_homing_move()
    test_c2_abort_with_no_active_move()
    test_c3_x_only_move_is_not_aborted_by_default()
    test_c4_bed_removal_calls_the_abort()
    test_c5_toolhead_disconnect_calls_the_abort()
    test_c6_probing_move_is_aborted_too()
    test_abort_before_the_drip_starts()
    fakes.finish()


if __name__ == '__main__':
    main()
