#!/usr/bin/env python3
"""AXIS_PROTECTED_MOVE BEYOND finds a stop behind the axis limit.

The M1's Y frame puts position_min at -0.1 (2026-11-25), which keeps prints
off the belt tension block, but the block (the Y back stop CUT_FILAMENT taps
to find) sits at about -1. A homed AXIS_PROTECTED_MOVE stopped at the axis
limit, so the tap never reached it and every cut was refused. BEYOND=<mm>
lets the move go that far past the limit. The limits stay as they are: the
move runs in a frame shifted by the overshoot, so the homing move still
ends inside them (a move is only checked where it ends), and the real
coordinate is put back after.

This drives the real AXIS_PROTECTED_MOVE handler with fakes for the
toolhead, the kinematics, the endstop and the homing move. The fake homing
move refuses an end outside the limits, as the kinematics' check_move does,
and triggers at a stop given in real coordinates.

    python3 test/landing/test_protected_move_beyond.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
KLIPPY = os.path.join(HERE, '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import advanced_homing_movement as ahm  # noqa: E402

Y_MIN, Y_MAX = -0.1, 180.1


class CommandError(Exception):
    pass


class FakeConfig:
    error = CommandError
    def __init__(self, printer):
        self.printer = printer
    def get_printer(self):
        return self.printer
    def get(self, name, default=None):
        return 'configured' if name == 'endstop_source' else default
    def getfloat(self, name, default=None, **kw):
        return default
    def getchoice(self, name, choices, default=None):
        return default


class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeReactor:
    def monotonic(self):
        return 100.


class FakeKin:
    def __init__(self, homed):
        self.homed = homed
    def get_status(self, eventtime):
        return {'homed_axes': self.homed,
                'axis_minimum': [0., Y_MIN, 0.],
                'axis_maximum': [200.1, Y_MAX, 160.]}
    def get_steppers(self):
        return []
    def clear_homing_state(self, axes):
        self.homed = self.homed.replace(axes, '')


class FakeToolhead:
    def __init__(self, kin, y):
        self.kin = kin
        self.pos = [199., y, 10., 0.]
        self.set_calls = []
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        return list(self.pos)
    def set_position(self, pos, homing_axes=""):
        self.pos = list(pos)
        self.set_calls.append((list(pos), homing_axes))
    def manual_move(self, coord, speed):
        pass


class FakePrinter:
    command_error = CommandError
    def __init__(self):
        self.objects = {'pins': object(), 'gcode': FakeGCode()}
        self.reactor = FakeReactor()
    def get_start_args(self):
        return {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def is_shutdown(self):
        return False


class FakeGcmd:
    error = CommandError
    def __init__(self, params):
        self.params = params
    def get(self, name, default=None):
        return self.params.get(name, default)
    def get_float(self, name, default=None, minval=None, maxval=None, **kw):
        if name not in self.params:
            return default
        v = float(self.params[name])
        if maxval is not None and v > maxval:
            raise CommandError("%s above %s" % (name, maxval))
        return v


def run(params, start_y=16., homed='xyz', stop_real=None):
    """Runs AXIS_PROTECTED_MOVE AXIS=Y DIR=MIN. stop_real: where the back
    stop is, in real Y. Returns (result, toolhead, homing move ends)."""
    printer = FakePrinter()
    kin = FakeKin(homed)
    toolhead = FakeToolhead(kin, start_y)
    printer.objects['toolhead'] = toolhead
    obj = ahm.AxisAdvancedHomingMovement(FakeConfig(printer))

    class Endstop:
        def add_stepper(self, stepper):
            pass
    obj._require_axis_context = (
        lambda axis, gcmd: {'mcu_endstop': Endstop(), 'source': 'configured'})
    ends = []
    real_start = start_y

    def run_homing_move(th, endstops, endpos, speed, triggered,
                        check_triggered, probe_pos):
        # check_move: a move may not end outside the limits, with no
        # tolerance (a shifted end a rounding error past -0.1 was refused
        # in the batch simulation)
        if not (Y_MIN <= endpos[1] <= Y_MAX):
            raise CommandError("Move out of range: %.3f" % endpos[1])
        ends.append(endpos[1])
        # the frame the move runs in, against the real one
        shift = th.pos[1] - (real_start if 'y' in homed else th.pos[1])
        end = list(endpos)
        if stop_real is not None and 'y' in homed:
            stop_v = stop_real + shift
            if stop_v > endpos[1]:
                end[1] = stop_v
        th.pos = end
    obj._run_homing_move = run_homing_move
    params = dict({'AXIS': 'y', 'DIR': 'min'}, **params)
    printer.objects['gcode'].commands['AXIS_PROTECTED_MOVE'](FakeGcmd(params))
    return obj.last, toolhead, ends


FAILURES = []


def check(name, cond, detail=""):
    print("%s %s" % ("ok  " if cond else "FAIL", name))
    if not cond:
        FAILURES.append(name)
        print("     %s" % (detail,))


def near(a, b):
    return a is not None and abs(a - b) < 1e-6


def test_without_beyond_the_tap_stops_at_the_limit():
    # The failure BEYOND fixes: the stop at -1.02 is never reached.
    last, th, ends = run({'DIST': '20.1'}, stop_real=-1.02)
    check("no BEYOND: the move ends at position_min",
          ends == [Y_MIN], repr(ends))
    check("no BEYOND: no stop found", last['triggered'] is False, repr(last))


def test_beyond_finds_a_stop_behind_the_limit():
    last, th, ends = run({'DIST': '20', 'BEYOND': '3.9'}, stop_real=-1.02)
    check("BEYOND: the homing move still ends inside the limits",
          all(Y_MIN - 1e-9 <= e <= Y_MAX + 1e-9 for e in ends), repr(ends))
    check("BEYOND: the stop is found", last['triggered'] is True, repr(last))
    check("BEYOND: at its real Y", near(last['position'], -1.02),
          repr(last))
    check("BEYOND: the toolhead is left at the real Y",
          near(th.pos[1], -1.02), repr(th.pos))
    check("BEYOND: Y is left homed", th.set_calls[-1][1] == 'y',
          repr(th.set_calls))


def test_beyond_without_a_stop_ends_at_its_floor():
    last, th, ends = run({'DIST': '20', 'BEYOND': '3.9'})
    check("BEYOND, no stop: not triggered", last['triggered'] is False,
          repr(last))
    check("BEYOND, no stop: the head is at the real floor (16 - 20)",
          near(th.pos[1], -4.), repr(th.pos))


def test_beyond_caps_a_longer_dist():
    last, th, ends = run({'DIST': '50', 'BEYOND': '3.9'})
    check("BEYOND caps the move at position_min - BEYOND",
          near(th.pos[1], Y_MIN - 3.9), repr(th.pos))


def test_a_move_inside_the_limits_is_unchanged():
    last, th, ends = run({'DIST': '10', 'BEYOND': '3.9'})
    check("inside the limits: no frame shift", ends == [6.], repr(ends))
    check("inside the limits: ends at 6", near(th.pos[1], 6.), repr(th.pos))


def test_unhomed_dist_may_cover_the_beyond():
    span = Y_MAX - Y_MIN
    try:
        run({'DIST': '%.3f' % (span + 3.9)}, homed='xz')
        refused = False
    except CommandError:
        refused = True
    check("unhomed, no BEYOND: a DIST past the span is refused", refused)
    last, th, ends = run({'DIST': '%.3f' % (span + 3.9), 'BEYOND': '3.9'},
                         start_y=50., homed='xz')
    check("unhomed, BEYOND: the move ends inside the limits",
          all(Y_MIN - 1e-9 <= e <= Y_MAX + 1e-9 for e in ends), repr(ends))
    check("unhomed, BEYOND: the coordinate is put back and Y left unhomed",
          near(th.pos[1], 50.) and 'y' not in th.kin.homed,
          "%r %r" % (th.pos, th.kin.homed))


def test_beyond_is_bounded():
    try:
        run({'DIST': '20', 'BEYOND': '%.1f' % (ahm.MAX_BEYOND + 1)})
        refused = False
    except CommandError:
        refused = True
    check("BEYOND above MAX_BEYOND is refused", refused)


def main():
    for t in (test_without_beyond_the_tap_stops_at_the_limit,
              test_beyond_finds_a_stop_behind_the_limit,
              test_beyond_without_a_stop_ends_at_its_floor,
              test_beyond_caps_a_longer_dist,
              test_a_move_inside_the_limits_is_unchanged,
              test_unhomed_dist_may_cover_the_beyond,
              test_beyond_is_bounded):
        t()
    if FAILURES:
        print("%d failed" % len(FAILURES))
        sys.exit(1)
    print("All AXIS_PROTECTED_MOVE BEYOND tests passed")


if __name__ == '__main__':
    main()
