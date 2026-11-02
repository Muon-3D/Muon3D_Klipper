"""homing_scope: G28 as one homing scope, and a failed G28 homes nothing.

KAN-573: on Boxwood an error inside the M1's G28 macro aborted _HOME_X
after it had faked X homed, and homed_axes read xyz with X never homed.
This drives the real HomingScope with fakes, since test/klippy can only
report that a failing G28 exited with an error, not what it left behind.

    python3 test/homing_scope/test_homing_scope.py
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import homing_scope  # noqa: E402


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


class FakeReactor:
    def monotonic(self):
        return 100.


class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        if func is None:
            return self.commands.pop(name, None)
        if name in self.commands:
            raise ConfigError("gcode command %s already registered" % name)
        self.commands[name] = func
    def run(self, name, gcmd=None):
        self.commands[name](gcmd)


class FakeKinematics:
    def __init__(self):
        self.homed = set()
        self.cleared = []
    def get_status(self, eventtime):
        return {'homed_axes': ''.join(a for a in 'xyz' if a in self.homed)}
    def clear_homing_state(self, axes):
        self.cleared.append(axes)
        self.homed -= set(axes)


class FakeToolhead:
    def __init__(self):
        self.kin = FakeKinematics()
    def get_kinematics(self):
        return self.kin


class FakePrinter:
    config_error = ConfigError
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()
        self.toolhead = FakeToolhead()
        self.objects = {'gcode': self.gcode, 'toolhead': self.toolhead}
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *args):
        for cb in self.handlers.get(event, []):
            cb(*args)


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok  " + what)


def rig(g28):
    p = FakePrinter()
    p.gcode.register_command('G28', g28)
    scope = homing_scope.load_config(FakeConfig(p))
    p.send_event("klippy:ready")
    return p, scope


def test_in_scope_for_the_whole_command():
    seen = []
    def g28(gcmd):
        seen.append(scope.in_scope)
        p.toolhead.kin.homed |= set('xyz')
    p, scope = rig(g28)
    check(not scope.in_scope, "not in scope before G28")
    p.gcode.run('G28')
    check(seen == [True], "in scope while the G28 macro runs")
    check(not scope.in_scope, "out of scope after G28")
    check(p.toolhead.kin.cleared == [], "a G28 that succeeds clears nothing")
    check(scope.get_status(0.) == {'in_scope': False}, "status reports it")


def test_failed_g28_leaves_nothing_homed():
    # What _HOME_X does: fake X homed, then a move inside it raises (the
    # contact_guard refusal on Boxwood) before the fake is cleared
    def g28(gcmd):
        p.toolhead.kin.homed |= set('yz')   # Y tap-homed, Z faked
        p.toolhead.kin.homed.add('x')       # SET_HOMED=X
        raise CommandError("Nozzle is touching the plate")
    p, scope = rig(g28)
    try:
        p.gcode.run('G28')
    except CommandError as e:
        check("touching" in str(e), "the G28 error still reaches the caller")
    else:
        raise AssertionError("G28 error swallowed")
    check(p.toolhead.kin.homed == set(), "no axis is left homed")
    check(p.toolhead.kin.cleared == ["xyz"], "every axis is cleared")
    check(not scope.in_scope, "out of scope after a failed G28")


def test_nested_g28():
    # G28 X inside a G28 (a callback through HOMING_GATE): one scope
    depth = []
    def g28(gcmd):
        depth.append(scope.depth)
        if gcmd == 'outer':
            p.gcode.run('G28', 'inner')
            check(scope.in_scope, "still in scope after the inner G28")
    p, scope = rig(g28)
    p.gcode.run('G28', 'outer')
    check(depth == [1, 2], "nested G28 nests the scope")
    check(not scope.in_scope, "scope closes after the outer G28")


def test_requires_the_command():
    p = FakePrinter()
    homing_scope.load_config(FakeConfig(p))
    try:
        p.send_event("klippy:ready")
    except ConfigError as e:
        check("G28" in str(e), "config error when G28 is not registered")
        return
    raise AssertionError("no config error without the command")


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d homing_scope tests passed" % len(tests))


if __name__ == '__main__':
    main()
