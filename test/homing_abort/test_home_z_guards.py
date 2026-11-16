"""The KAN-3370 guards inside LOAD_CELL_HOME_Z's retry loop (this branch only).

feat/m1-print-start homes Z with load_cell_home_z.py: a fast approach from
a soft-homed top, then a confirming tap.  A confirming tap that finds
nothing means the first touch was false, and the whole home restarts from
the top, up to `attempts` times.  Three things must hold with the guards:

* a refused tare ("The nozzle is already under load") is an error, never a
  retry: the loop must not move again after it, on any attempt, whether it
  is the fast approach or the confirming tap that refuses;
* the 40 mm search cap applies to the fast approach of EVERY attempt;
* an aborted move (toolhead disconnect, bed removal) is an error, not a
  false touch: the loop stops and does not start another approach.

Drives the real LoadCellHomeZ, the real PrinterHoming / HomingMove and the
real LoadCellProbeConfigHelper.check_tare_headroom (the refusal text is
the real one).  Only the stepper, endstop, toolhead and gcode are fakes
(see fakes.py).  The numbers are written out here.

    python3 test/homing_abort/test_home_z_guards.py

Needs numpy only because test_tare_guard.py imports it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fakes  # noqa: E402
from fakes import Rig, check, FakeMcuObject  # noqa: E402
from extras import load_cell_home_z  # noqa: E402

# test_tare_guard replaces sys.modules['extras'] with a stub when it loads
# load_cell_probe, so every real extras module is imported above this line
sys.path.insert(0, os.path.join(HERE, '..', 'load_cell_probe'))
import test_tare_guard as tg  # noqa: E402

Z_TOP, Z_MIN = 160., -4.
FAST = "PROBE_SPEED=15.000"
REFUSAL = "The nozzle is already under load"
CAP_MSG = ("Z homing stopped: no contact within 40 mm. Check the bed is"
           " fitted and nothing is wrong with the nozzle")
TOOLHEAD_MSG = "Homing aborted: the toolhead disconnected"
BED_MSG = "Homing aborted: the bed was removed"
DISCONNECT_EVENT = "danger:non_critical_mcu_toolhead:disconnected"

MOD = tg.load_module(tg._paths.patched('klippy/extras/load_cell_probe.py'))


def real_tare_check(ref, **opts):
    """check_tare_headroom of the real helper for a unit's anchor."""
    helper = tg.make_helper(MOD, ref, trigger_percent=10, **opts)

    def check_tare(tare):
        try:
            helper.check_tare_headroom(tare)
        except tg.CommandError as e:
            raise fakes.CommandError(str(e))
    return check_tare


class FakeGCmd:
    error = fakes.CommandError

    def __init__(self):
        self.info = []

    def get_float(self, name, default=None, **kw):
        return default

    def respond_info(self, msg):
        self.info.append(msg)


class Machine:
    """Z over a bed `gap` mm below the nozzle.  Interprets the script lines
    LoadCellHomeZ runs and drives real probing moves on a Rig.

    false_touches: travelled mm at which each successive fast approach
    stops on something that is not the bed (consumed one per fast probe).
    tares: the tare each successive PROBE reads; check_tare raises for a
    refused one, as _pause_and_tare does, before any motion.
    """

    def __init__(self, gap, tares, check_tare, false_touches=()):
        self.rig = Rig(axis=2, start=Z_TOP, n_endstops=1)
        self.rig.printer.objects['homing_scope'] = type(
            'S', (), {'in_scope': True})()
        self.gap = gap
        self.tares = list(tares)
        self.check_tare = check_tare
        self.false_touches = list(false_touches)
        self.relative = False
        self.last_probe_z = None
        self.script = []
        self.soft_homes = 0
        self.probes = []            # (fast, target_z, travelled_mm)
        self.lifts = 0
        self.error = fakes.CommandError

    def _set_z(self, z):
        th = self.rig.toolhead
        th.pos[2] = z
        for s in self.rig.steppers:
            s.pos = s.target_steps = int(round(z * fakes.STEPS_PER_MM))
            s.cmd_mm = z

    def probe(self, fast):
        tare = self.tares.pop(0) if self.tares else None
        if tare is not None:
            self.check_tare(tare)               # refuses before any motion
        es = self.rig.endstops[0]
        start_z = self.rig.toolhead.pos[2]
        if fast and self.false_touches:
            es.trigger_after_mm = self.false_touches.pop(0)
        else:
            es.trigger_after_mm = self.gap
        speed = 15. if fast else 5.
        target = [0., 0., Z_MIN, 0.]
        try:
            epos = self.rig.phoming.probing_move(es, target, speed)
        finally:
            moved = self.rig.toolhead.travelled_mm
            self.gap -= moved
            self.probes.append((fast, self.rig.toolhead.drip_calls[-1][0][2],
                                moved))
        self.last_probe_z = epos[2]

    def run(self, script):
        for line in script.split('\n'):
            self.script.append(line)
            w = line.split()
            if not w:
                continue
            if w[0] == 'SET_KINEMATIC_POSITION':
                self.soft_homes += 1
                self._set_z(float(w[1].split('=')[1]))
            elif w[0] == 'G91':
                self.relative = True
            elif w[0] == 'G1':
                assert self.relative
                dz = float(w[1][1:])
                self.lifts += 1
                self._set_z(self.rig.toolhead.pos[2] + dz)
                self.gap += dz
            elif w[0] == 'PROBE':
                self.probe(FAST in line)
            elif w[0] == '_HOME_Z_FROM_LAST_PROBE':
                self._set_z(self.rig.toolhead.pos[2] - self.last_probe_z)
            elif w[0] == 'RESTORE_GCODE_STATE':
                self.relative = False


class FakeGcode:
    error = fakes.CommandError

    def __init__(self, m):
        self.m = m

    def register_command(self, name, fn, desc=None):
        pass

    def run_script_from_command(self, script):
        self.m.run(script)


class FakeToolheadStatus:
    def get_status(self, et):
        return {'axis_maximum': [200., 174., Z_TOP],
                'axis_minimum': [0., -10., Z_MIN]}


class HomeZPrinter:
    def __init__(self, m):
        self.objs = {'gcode': FakeGcode(m), 'toolhead': FakeToolheadStatus()}

    def lookup_object(self, name):
        return self.objs[name]

    def get_reactor(self):
        return fakes.FakeReactor()


class Cfg:
    def __init__(self, m):
        self.p = HomeZPrinter(m)

    def get_printer(self):
        return self.p

    def getint(self, name, default=None, **kw):
        return default

    def getfloat(self, name, default=None, **kw):
        return default


def home_z(m):
    hz = load_cell_home_z.LoadCellHomeZ(Cfg(m))
    gcmd = FakeGCmd()
    try:
        hz.cmd_LOAD_CELL_HOME_Z(gcmd)
        err = None
    except fakes.CommandError as e:
        err = str(e)
    return hz, gcmd, err


UNIT_A = real_tare_check(-3010747)
SLOE = real_tare_check(-1585739)
OK_A = -1000000             # unit A, normal
OK_SLOE = 6430616           # Sloe, normal
BAD_A = -3244819            # the incident tare
MOVES = lambda m: len(m.rig.toolhead.drip_calls)


def test_refused_tare_on_the_first_fast_approach():
    m = Machine(gap=10, tares=[BAD_A], check_tare=UNIT_A)
    hz, gcmd, err = home_z(m)
    check(err is not None and err.startswith(REFUSAL),
          "T1: the home fails with the refusal (got %r)" % (err,))
    check(MOVES(m) == 0, "T1: no motion at all (drips %d)" % MOVES(m))
    check(m.lifts == 0, "T1: no lift or other move")
    check(hz.last_attempts == 1 and hz.false_touches == 0,
          "T1: one attempt, not counted as a false touch")
    check(m.relative is False, "T1: gcode state restored")


def test_refused_tare_on_a_retry_is_not_retried():
    # attempts 1 and 2 stop on a false touch at 2 mm and 3 mm (the confirm
    # then finds nothing); attempt 3's fast approach is refused
    m = Machine(gap=40, tares=[OK_A, OK_A, OK_A, OK_A, BAD_A],
                check_tare=UNIT_A, false_touches=[2, 3])
    hz, gcmd, err = home_z(m)
    check(err is not None and err.startswith(REFUSAL),
          "T2: attempt 3 fails with the refusal (got %r)" % (err,))
    check(MOVES(m) == 4,
          "T2: only attempts 1 and 2 moved (2 drips each), got %d"
          % MOVES(m))
    check(m.soft_homes == 3 and hz.last_attempts == 3,
          "T2: stopped at attempt 3 of 5 (soft homes %d, attempts %d)"
          % (m.soft_homes, hz.last_attempts))
    check(hz.false_touches == 2, "T2: two false touches counted, not three")
    check(len(gcmd.info) == 2,
          "T2: two 'Homing Z again' notices, none after the refusal")


def test_refused_tare_on_the_confirming_tap_is_not_retried():
    m = Machine(gap=10, tares=[OK_SLOE, BAD_A], check_tare=UNIT_A)
    hz, gcmd, err = home_z(m)
    check(err is not None and err.startswith(REFUSAL),
          "T3: the confirm tap's refusal ends the home (got %r)" % (err,))
    check(MOVES(m) == 1, "T3: the fast approach moved, the confirm did not")
    check(hz.last_attempts == 1 and hz.false_touches == 0,
          "T3: one attempt, not a false touch")
    check(m.soft_homes == 1, "T3: no restart from the top")


def test_cap_applies_to_every_attempt():
    # three false touches, then a fast approach that finds nothing
    m = Machine(gap=200, tares=[OK_A] * 8, check_tare=UNIT_A,
                false_touches=[2, 2, 2])
    hz, gcmd, err = home_z(m)
    fast = [p for p in m.probes if p[0]]
    check(len(fast) == 4, "T4: four fast approaches (got %d)" % len(fast))
    check([p[1] for p in fast] == [120.0] * 4,
          "T4: every fast approach was commanded to 120, i.e. 40 mm below "
          "the soft-homed 160 (got %r)" % ([p[1] for p in fast],))
    check(fast[-1][2] == 40,
          "T4: the last one travelled exactly 40 mm (got %r)" % fast[-1][2])
    check(err == CAP_MSG, "T4: ends with the cap message (got %r)" % (err,))
    check("No trigger" not in (err or ""),
          "T4: the cap message is not read as a false touch")
    check(hz.last_attempts == 4, "T4: the 4th attempt's cap is final, no "
          "5th (attempts %d)" % hz.last_attempts)
    check(max(p[2] for p in m.probes) == 40,
          "T4: no probing move travelled more than 40 mm")


def test_confirm_tap_still_retries_and_a_home_succeeds():
    # bed 10 mm down; the first fast approach stops on a false touch at 2
    m = Machine(gap=10, tares=[OK_SLOE] * 4, check_tare=SLOE,
                false_touches=[2])
    hz, gcmd, err = home_z(m)
    check(err is None, "T5: the home succeeds on attempt 2 (got %r)" % (err,))
    check(hz.last_attempts == 2 and hz.false_touches == 1,
          "T5: one false touch, two attempts")
    check([p[1] for p in m.probes if p[0]] == [120.0, 120.0],
          "T5: both fast approaches were capped at 40 mm")


def test_disconnect_and_bed_removal_are_not_false_touches():
    # (where, attempt): the fast approach of attempt 1 or 2, or attempt 1's
    # confirming tap (made to run its whole 5 mm by an earlier false touch
    # that the abort must not turn into a retry)
    for what, msg in (("disconnect", TOOLHEAD_MSG), ("bed", BED_MSG)):
        for where, attempt in (("fast", 1), ("fast", 2), ("confirm", 1)):
            tag = "T6 %s during the %s of attempt %d" % (what, where,
                                                         attempt)
            touches = [2] if (attempt == 2 or where == "confirm") else []
            m = Machine(gap=40, tares=[OK_A] * 8, check_tare=UNIT_A,
                        false_touches=touches)
            m.rig.printer.objects['mcu toolhead'] = FakeMcuObject(
                'toolhead', True)
            m.rig.printer.send_event('klippy:connect')

            def on_tick(mm, m=m, what=what, where=where, attempt=attempt):
                calls = m.rig.toolhead.drip_calls
                speed = 15. if where == "fast" else 5.
                nthis = len([c for c in calls if c[1] == speed])
                if calls[-1][1] == speed and nthis == attempt and mm == 3:
                    if what == "disconnect":
                        m.rig.printer.send_event(DISCONNECT_EVENT)
                    else:
                        m.rig.phoming.abort_active_move(
                            "the bed was removed")
            m.rig.toolhead.on_tick = on_tick
            hz, gcmd, err = home_z(m)
            check(err == msg, "%s: error is %r (got %r)" % (tag, msg, err))
            check(hz.last_attempts == attempt and hz.false_touches ==
                  (attempt - 1 if where == "fast" else 0),
                  "%s: stopped there, not retried (attempts %d)"
                  % (tag, hz.last_attempts))
            check(m.soft_homes == attempt,
                  "%s: no restart from the top (soft homes %d)"
                  % (tag, m.soft_homes))
            check(m.rig.phoming.active_moves == [],
                  "%s: no move left active" % (tag,))


def main():
    test_refused_tare_on_the_first_fast_approach()
    test_refused_tare_on_a_retry_is_not_retried()
    test_refused_tare_on_the_confirming_tap_is_not_retried()
    test_cap_applies_to_every_attempt()
    test_confirm_tap_still_retries_and_a_home_succeeds()
    test_disconnect_and_bed_removal_are_not_false_touches()
    fakes.finish()


if __name__ == '__main__':
    main()
