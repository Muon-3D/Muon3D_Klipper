"""cold_retract: the unlock retract without FORCE_MOVE (KAN-380).

Drives the real ColdRetract with a fake force_move and checks its bounds,
then checks the shipped M1 config: FORCE_MOVE is off and nothing calls it.

    python3 test/cold_retract/test_cold_retract.py
"""
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

from extras import cold_retract  # noqa: E402

M1 = os.path.join(ROOT, 'core', 'M1')


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class FakeConfig:
    def __init__(self, printer, values=None):
        self.printer = printer
        self.values = values or {}
    def get_printer(self):
        return self.printer
    def get(self, name, default=None):
        return self.values.get(name, default)
    def getfloat(self, name, default=None, minval=None, maxval=None,
                 above=None):
        return float(self.values.get(name, default))


class FakeGCmd:
    def __init__(self, params):
        self.params = params
    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None, below=None):
        if name not in self.params:
            return default
        v = float(self.params[name])
        if above is not None and v <= above:
            raise CommandError("%s must be above %s" % (name, above))
        if maxval is not None and v > maxval:
            raise CommandError("%s must have maximum of %s" % (name, maxval))
        return v


class FakeForceMove:
    def __init__(self):
        self.steppers = {'extruder': 'extruder-stepper'}
        self.moves = []
        self.enabled = []
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise ConfigError("Unknown stepper %s" % (name,))
        return self.steppers[name]
    def _force_enable(self, stepper):
        self.enabled.append(stepper)
        return True
    def manual_move(self, stepper, dist, speed, accel=0.):
        self.moves.append((stepper, dist, speed, accel))


class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakePrinter:
    def __init__(self):
        self.gcode = FakeGCode()
        self.force_move = FakeForceMove()
        self.handlers = {}
    def lookup_object(self, name, default=None):
        return {'gcode': self.gcode}.get(name, default)
    def load_object(self, config, name):
        assert name == 'force_move'
        return self.force_move
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event):
        for cb in self.handlers.get(event, []):
            cb()


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok  " + what)


def rig(values=None):
    p = FakePrinter()
    cold_retract.load_config(FakeConfig(p, values))
    p.send_event("klippy:connect")
    return p, p.gcode.commands['COLD_RETRACT']


def test_retracts_backwards_within_bounds():
    p, cmd = rig()
    cmd(FakeGCmd({'DISTANCE': '16'}))
    check(p.force_move.moves == [('extruder-stepper', -16., 20., 1000.)],
          "16 mm backwards at 20 mm/s, 1000 mm/s^2")
    check(p.force_move.enabled == ['extruder-stepper'],
          "the stepper is enabled for the move")
    cmd(FakeGCmd({}))
    check(p.force_move.moves[-1][1] == -16., "default is max_distance")


def test_refuses_out_of_bounds():
    p, cmd = rig()
    for params, what in (({'DISTANCE': '16.5'}, "past max_distance"),
                         ({'DISTANCE': '-5'}, "a forward move"),
                         ({'DISTANCE': '0'}, "zero")):
        try:
            cmd(FakeGCmd(params))
        except CommandError:
            check(True, "refuses %s" % what)
            continue
        raise AssertionError("did not refuse %s" % what)
    check(p.force_move.moves == [], "nothing moved")


def test_unknown_stepper_is_a_config_error():
    p = FakePrinter()
    cold_retract.load_config(FakeConfig(p, {'stepper': 'stepper_z'}))
    try:
        p.send_event("klippy:connect")
    except ConfigError:
        check(True, "unknown stepper is a config error")
        return
    raise AssertionError("no config error for an unknown stepper")


def _m1_files():
    for dirpath, dirs, files in os.walk(M1):
        for f in files:
            if f.endswith('.cfg'):
                yield os.path.join(dirpath, f)


def test_m1_config_has_force_move_off():
    values, calls = [], []
    for path in _m1_files():
        section = None
        with open(path, encoding='utf-8') as f:
            for n, line in enumerate(f, 1):
                bare = line.split('#', 1)[0].split(';', 1)[0].strip()
                m = re.match(r'^\[([^\]]+)\]', bare)
                if m:
                    section = m.group(1).strip()
                    continue
                if section == 'force_move' and bare.startswith(
                        'enable_force_move'):
                    values.append((path, bare))
                if re.search(r'\bFORCE_MOVE\b', bare):
                    calls.append('%s:%d' % (path, n))
    check(len(values) == 1, "one enable_force_move in core/M1: %s" % values)
    check(re.match(r'enable_force_move\s*[:=]\s*false$', values[0][1]),
          "enable_force_move is false")
    check(calls == [], "nothing in core/M1 calls FORCE_MOVE: %s" % calls)


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d cold_retract tests passed" % len(tests))


if __name__ == '__main__':
    main()
