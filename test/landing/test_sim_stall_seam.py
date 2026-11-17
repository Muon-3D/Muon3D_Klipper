#!/usr/bin/env python3
"""SIM_STALL_AT is a batch-test seam and must be inert on a printer.

AXIS_PROTECTED_MOVE (klippy/extras/advanced_homing_movement.py) takes a
SIM_STALL_AT=<coordinate> parameter that makes the move report a stall
there, because a simulated MCU never triggers an endstop part way. It
acts only when klippy was started with -i / --debuginput (the batch
input file), which sets start_args['debuginput'] and nothing else does
(klippy/klippy.py, main()). A printer is started without -i, so the
parameter must do nothing there: no position written, no stall made up.

This drives the real AxisAdvancedHomingMovement and its real
AXIS_PROTECTED_MOVE handler, with fakes for what is hardware: the
toolhead, the kinematics, the endstop and the homing move itself.

  1. start_args without debuginput (a printer): SIM_STALL_AT has no effect.
     The result comes only from the real move, whether it ran to its
     target or stopped short on a real trigger.
  2. start_args['debuginput'] set to None: the same.
  3. start_args['debuginput'] set (batch mode): it takes effect.
  4. Source checks: klippy.py sets start_args['debuginput'] only from the
     -i option, and SIM_STALL_AT is read in one place, guarded by it.

    python3 test/landing/test_sim_stall_seam.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
KLIPPY = os.path.join(HERE, '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import advanced_homing_movement as ahm  # noqa: E402


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class FakeConfig:
    error = ConfigError
    def __init__(self, printer):
        self.printer = printer
    def get_printer(self):
        return self.printer
    def get(self, name, default=None):
        if name == 'endstop_source':
            # No TMC context is built at config time.
            return 'configured'
        return default
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
        self.cleared = []
    def get_status(self, eventtime):
        return {'homed_axes': self.homed,
                'axis_minimum': [0., 0., 0.],
                'axis_maximum': [200., 200., 160.]}
    def get_steppers(self):
        return []
    def clear_homing_state(self, axes):
        self.cleared.append(axes)


class FakeToolhead:
    def __init__(self, kin, z):
        self.kin = kin
        self.pos = [10., 10., z, 0.]
        self.set_calls = []
        self.moves = []
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        return list(self.pos)
    def set_position(self, pos, homing_axes=""):
        self.pos = list(pos)
        self.set_calls.append((list(pos), homing_axes))
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))


class FakePrinter:
    command_error = CommandError
    def __init__(self, start_args):
        self.start_args = start_args
        self.objects = {'pins': object(), 'gcode': FakeGCode()}
        self.reactor = FakeReactor()
    def get_start_args(self):
        return self.start_args
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
    def get_float(self, name, default=None, **kw):
        if name not in self.params:
            return default
        return float(self.params[name])


def build(start_args, stops_at=None, homed='z', z=50.):
    """The real class on a fake printer. The homing move leaves the
    toolhead at its target, or at stops_at when the (fake) endstop
    triggered there."""
    printer = FakePrinter(start_args)
    kin = FakeKin(homed)
    toolhead = FakeToolhead(kin, z)
    printer.objects['toolhead'] = toolhead
    obj = ahm.AxisAdvancedHomingMovement(FakeConfig(printer))

    class Endstop:
        def add_stepper(self, stepper):
            pass
    obj._require_axis_context = (
        lambda axis, gcmd: {'mcu_endstop': Endstop(), 'source': 'configured'})

    def run_homing_move(th, endstops, endpos, speed, triggered,
                        check_triggered, probe_pos):
        end = list(endpos)
        if stops_at is not None:
            end[2] = stops_at
        th.pos = end
    obj._run_homing_move = run_homing_move
    return obj, toolhead, printer.objects['gcode']


def run(start_args, params, stops_at=None):
    obj, toolhead, gcode = build(start_args, stops_at)
    # Through the registered command, as G-code would reach it.
    gcode.commands['AXIS_PROTECTED_MOVE'](FakeGcmd(params))
    return obj.last, toolhead


FAILURES = []


def check(name, cond, detail=""):
    if not cond:
        FAILURES.append("%s %s" % (name, detail))
        print("FAIL: %s %s" % (name, detail))


def z_writes(toolhead, z):
    return [c for c, _ in toolhead.set_calls if abs(c[2] - z) < 1e-9]


def test_printer_ignores_seam():
    # axis Z, homed, from Z50 to Z60 (DIST 10 DIR max).
    base = {'AXIS': 'z', 'DIST': '10'}
    seam = dict(base, SIM_STALL_AT='55')
    for label, args in (('no debuginput', {}),
                        ('debuginput None', {'debuginput': None})):
        # The move runs to its target: no stall, and nothing at Z55.
        last, th = run(args, seam)
        check("%s, ran to target: stalled" % label,
              last['triggered'] is False, repr(last))
        check("%s, ran to target: no write at the seam coordinate" % label,
              not z_writes(th, 55.), repr(th.set_calls))
        check("%s, ran to target: ends at the target" % label,
              abs(th.pos[2] - 60.) < 1e-9, repr(th.pos))
        # Same result as without the parameter at all.
        last0, th0 = run(args, base)
        check("%s: the parameter changes nothing" % label,
              (last['triggered'], last['position'], th.set_calls)
              == (last0['triggered'], last0['position'], th0.set_calls),
              "with %r, without %r" % (th.set_calls, th0.set_calls))
        # A real stop at Z57 is reported as it is, not moved to Z55.
        last, th = run(args, seam, stops_at=57.)
        check("%s, real stop: stalled from the real move" % label,
              last['triggered'] is True, repr(last))
        check("%s, real stop: position is the real one" % label,
              last['position'] == 57., repr(last))
        check("%s, real stop: no write at the seam coordinate" % label,
              not z_writes(th, 55.), repr(th.set_calls))


def test_batch_mode_takes_effect():
    seam = {'AXIS': 'z', 'DIST': '10', 'SIM_STALL_AT': '55'}
    last, th = run({'debuginput': '/tmp/input.gcode'}, seam)
    check("batch: the move is reported stalled", last['triggered'] is True,
          repr(last))
    check("batch: position is the seam coordinate",
          last['position'] == 55., repr(last))
    check("batch: the position is written", len(z_writes(th, 55.)) == 1,
          repr(th.set_calls))
    # Without the parameter, batch mode is a normal move.
    last, th = run({'debuginput': '/tmp/input.gcode'},
                   {'AXIS': 'z', 'DIST': '10'})
    check("batch, no parameter: not stalled", last['triggered'] is False,
          repr(last))


def read(rel):
    with open(os.path.join(HERE, '..', '..', rel), encoding='utf8') as f:
        return f.read()


def test_source_facts():
    klippy_py = read('klippy/klippy.py')
    # The only write of start_args['debuginput'] is under -i / --debuginput.
    writes = re.findall(r"start_args\[['\"]debuginput['\"]\]\s*=", klippy_py)
    check("klippy.py writes start_args['debuginput'] once",
          len(writes) == 1, repr(writes))
    m = re.search(r"if options\.debuginput:\s*\n\s*start_args\['debuginput'\]"
                  r"\s*=\s*options\.debuginput", klippy_py)
    check("klippy.py sets it only from the -i option", m is not None)
    check("the -i option is the debuginput option",
          re.search(r'add_option\("-i", "--debuginput"', klippy_py)
          is not None)
    # Nothing else in klippy/ assigns it (the API server, config options).
    for root, _, names in os.walk(KLIPPY):
        for name in names:
            if not name.endswith('.py') or name == 'klippy.py':
                continue
            path = os.path.join(root, name)
            with open(path, encoding='utf8', errors='replace') as f:
                text = f.read()
            if re.search(r"\[['\"]debuginput['\"]\]\s*=[^=]", text):
                check("only klippy.py assigns debuginput", False, path)
    # SIM_STALL_AT is read in one place, with the guard beside it.
    hits = []
    for root, _, names in os.walk(KLIPPY):
        for name in names:
            if name.endswith('.py'):
                path = os.path.join(root, name)
                with open(path, encoding='utf8', errors='replace') as f:
                    text = f.read()
                if 'SIM_STALL_AT' in text:
                    hits.append(os.path.relpath(path, KLIPPY))
    check("SIM_STALL_AT appears in one klippy file",
          hits == [os.path.join('extras', 'advanced_homing_movement.py')],
          repr(hits))
    src = read('klippy/extras/advanced_homing_movement.py')
    m = re.search(r"gcmd\.get_float\('SIM_STALL_AT', None\)\s*\n"
                  r"\s*if \(sim_at is not None\s*\n"
                  r"\s*and self\.printer\.get_start_args\(\)"
                  r"\.get\('debuginput'\)\s*\n"
                  r"\s*is not None\):", src)
    check("the SIM_STALL_AT branch is guarded by start_args debuginput",
          m is not None)


def main():
    test_printer_ignores_seam()
    test_batch_mode_takes_effect()
    test_source_facts()
    if FAILURES:
        print("%d failure(s)" % len(FAILURES))
        sys.exit(1)
    print("sim_stall_seam: SIM_STALL_AT is inert without -i, active with it:"
          " all checks passed")


if __name__ == '__main__':
    main()
