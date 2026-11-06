"""G28 paths: the real homing.cfg macros inside the real HomingScope.

Bench-branch evidence for the homing fixes (#92 scope, #108 re-zero in scope,
#106 Y current + cleanup, lift flag reset).  Renders core/M1/macros/homing.cfg
with Klipper's single-brace Jinja and runs it through a small G-code
interpreter, with homing_scope.HomingScope around G28 and a homed-axes model:

  SET_KINEMATIC_POSITION ... SET_HOMED=<axes>   adds the axes
  SET_KINEMATIC_POSITION ... CLEAR_HOMED=<axes> removes them
  G28.1 <axis>                                  success homes that axis
  (coordinates alone do not change homed_axes; an assumption of this model)

Nothing here moves a motor.  contact_guard's own refusal logic is covered by
test/contact_guard, not here.

    python3 test/homing_scope/test_g28_paths.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_homing_scope as ths  # noqa: E402
import test_homing_y_current as thy  # noqa: E402
from extras import homing_scope  # noqa: E402

LIFT = 'AXIS_PROTECTED_MOVE AXIS=Z DIST=10.0'


class Reads(dict):
    """The 'mcu toolhead' status: disconnected for the first N reads."""
    def __init__(self, down_reads):
        dict.__init__(self)
        self.left = down_reads
    def __getitem__(self, key):
        if key == 'non_critical_disconnected':
            if self.left > 0:
                self.left -= 1
                return True
            return False
        return dict.__getitem__(self, key)


class PathRig(thy.Rig):
    def __init__(self, homed='', z=50., y=100., fail_on=None, down_reads=0):
        thy.Rig.__init__(self, homed=homed, fail_on=fail_on)
        self.z = z
        self.y = y
        self.mcu_toolhead = Reads(down_reads)
        self.fail_set = set([fail_on]) if fail_on else set()
        self.p = None
        self.kin = None

    def attach(self, p):
        self.p = p
        self.kin = p.toolhead.kin
        self.kin.homed = set(self.homed)

    def status(self):
        st = thy.Rig.status(self)
        st['toolhead']['homed_axes'] = ''.join(
            a for a in 'xyz' if a in self.kin.homed)
        st['toolhead']['position'] = {'x': 100., 'y': self.y, 'z': self.z}
        st['mcu toolhead'] = self.mcu_toolhead
        st['gcode_macro G28'] = self.p.macro.variables
        return st

    def run(self, text, rawparams='', params=None):
        tmpl = self.env.from_string(text)
        def raise_error(msg):
            raise thy.Failure(msg)
        out = tmpl.render(printer=self.status(), rawparams=rawparams,
                          params=params or {},
                          action_raise_error=raise_error,
                          action_respond_info=lambda m: "")
        for line in out.splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                self.command(line)

    def command(self, line):
        name, _, rest = line.partition(' ')
        rest = rest.strip()
        if name == 'G28':                    # through the scope, not the macro
            self.log.append((name, rest))
            self.p.gcode.run('G28', rest)
            return
        if name == 'HOMING_GATE':
            self.log.append((name, rest))
            cb = rest.split('CALLBACK=', 1)[1].strip().strip('"')
            self.run(self.macro_text('HOMING_GATE'), rawparams=rest,
                     params={'CALLBACK': cb})
            return
        if name in ('SET_GCODE_VARIABLE',):
            kv = dict(p.split('=', 1) for p in rest.split())
            if kv['MACRO'] == 'G28':
                v = dict(self.p.macro.variables)
                v[kv['VARIABLE']] = int(kv['VALUE'])
                self.p.macro.variables = v    # copy-and-replace, as klippy
                return
        if name == 'SET_KINEMATIC_POSITION':
            self.log.append((name, rest))
            kv = dict(p.split('=', 1) for p in rest.split() if '=' in p)
            self.kin.homed |= set(kv.get('SET_HOMED', '').lower())
            self.kin.homed -= set(kv.get('CLEAR_HOMED', '').lower())
            return
        if name == 'G28.1':
            self.log.append((name, rest))
            if line in self.fail_set:
                raise thy.Failure("%s failed" % line)
            self.kin.homed |= set(rest.lower())
            return
        self.fail_on = None
        thy.Rig.command(self, line)

    def lifts(self):
        return self.cmds().count(LIFT)

    def homed_axes(self):
        return ''.join(a for a in 'xyz' if a in self.kin.homed)


def g28_scope(rig):
    p = ths.FakePrinter()

    def g28(gcmd):
        rig.run(rig.macro_text('G28'), rawparams=gcmd or '')
    p.gcode.register_command('G28', g28)
    p.gcode.run_script_from_command = lambda text: rig.run(text)
    cleanup = thy.load_sections().get('homing_scope', 'cleanup_gcode')
    homing_scope.load_config(ths.FakeConfig(p, {'cleanup_gcode': cleanup}))
    p.send_event("klippy:ready")
    rig.attach(p)
    return p


def fails(p, arg=''):
    try:
        p.gcode.run('G28', arg)
    except thy.Failure:
        return True
    return False


def test_normal_g28_lifts_once():
    rig = PathRig(homed='')
    p = g28_scope(rig)
    ths.check(not fails(p), "a normal G28 succeeds")
    ths.check(rig.lifts() == 1, "the Z lift happens once (%d)" % rig.lifts())
    ths.check(rig.homed_axes() == 'xyz', "X, Y and Z are homed")
    ths.check(p.macro.variables['z_lifted_once'] == 0, "lift flag reset")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8}, "driver currents restored")


def test_failed_helper_leaves_nothing_homed():
    for helper, tap in (('_HOME_Y', 'G28.1 Y'), ('_HOME_X', 'G28.1 X'),
                        ('_HOME_Z', 'G28.1 Z')):
        rig = PathRig(homed='', fail_on=tap)
        p = g28_scope(rig)
        ths.check(fails(p), "G28 fails in %s (%s)" % (helper, tap))
        ths.check(any(n == helper for n, _ in rig.log),
                  "%s was reached" % helper)
        ths.check(rig.homed_axes() == '',
                  "homed_axes ends empty after a failure in %s (was %r)"
                  % (helper, rig.homed_axes()))
        ths.check(rig.driver == {'x': 0.8, 'y': 0.8},
                  "currents restored after a failure in %s" % helper)
        ths.check(not p.lookup_object('toolhead') is None, "scope closed")


def test_failure_with_axes_already_homed_clears_them():
    # a real home that fails midway must not leave the old homed state
    rig = PathRig(homed='xyz', z=50., fail_on='G28.1 Y')
    p = g28_scope(rig)
    ths.check(fails(p), "G28 fails in _HOME_Y with xyz homed")
    ths.check(rig.homed_axes() == '', "homed_axes ends empty")


def test_retry_after_failure_lifts_again():
    rig = PathRig(homed='', fail_on='G28.1 X')
    p = g28_scope(rig)
    ths.check(fails(p), "first G28 fails in _HOME_X")
    ths.check(rig.lifts() == 1, "first G28 lifted")
    ths.check(p.macro.variables['z_lifted_once'] == 1,
              "the failed G28 left the macro flag set (macro unchanged)")
    rig.fail_set.clear()
    ths.check(not fails(p), "the retry succeeds")
    ths.check(rig.lifts() == 2, "the retry lifts again (%d total)"
              % rig.lifts())
    ths.check(rig.homed_axes() == 'xyz', "retry homes everything")


def test_homing_gate_callback_lifts_once():
    # G28 sees the toolhead down (1 read), HOMING_GATE sees it back: the
    # callback runs G28 again inside the outer one (depth 2)
    depths = []
    rig = PathRig(homed='', down_reads=1)
    p = g28_scope(rig)
    orig = p.gcode.commands['G28']
    def spy(gcmd):
        depths.append(len([1 for n, _ in rig.log if n == 'G28']))
        orig(gcmd)
    p.gcode.commands['G28'] = spy
    ths.check(not fails(p), "the G28 with a HOMING_GATE callback succeeds")
    names = [n for n, _ in rig.log]
    ths.check('HOMING_GATE' in names, "HOMING_GATE ran")
    ths.check(names.count('G28') == 1, "the callback re-invoked G28 once")
    ths.check(rig.lifts() == 1, "the lift happens once (%d)" % rig.lifts())
    ths.check(rig.homed_axes() == 'xyz', "everything homed")
    ths.check(p.macro.variables['z_lifted_once'] == 0, "flag reset")


def test_home_x_y_backoff_is_plain_xy_motion():
    # Macro side only: which motion _HOME_X emits for the Y back-off.  That
    # contact_guard does not refuse it is test/contact_guard
    # (test_g28_lift_rezero_survives_home_y) and, for real, bench only.
    for y, homed, want in ((100., 'xyz', 'G1 Y-10 F4800'),
                           (5., 'xyz', 'G1 Y10 F4800'),
                           (100., '', 'G1 Y10 F4800')):
        rig = PathRig(homed=homed, y=y)
        p = g28_scope(rig)
        rig.run(rig.macro_text('_HOME_X'))
        c = rig.cmds()
        ths.check(want in c, "y=%s homed=%r emits %s" % (y, homed, want))
        ths.check(c.index(want) < c.index('G28.1 X'),
                  "the Y back-off comes before G28.1 X")
        ths.check(not any(x.startswith('AXIS_PROTECTED_MOVE') for x in c),
                  "no protected Z move in _HOME_X")


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d G28 path tests passed" % len(tests))


if __name__ == '__main__':
    main()
