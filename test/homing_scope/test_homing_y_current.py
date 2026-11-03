"""KAN-1361: the M1's Y homing current is a setting, and always put back.

Runs the real macros from core/M1/macros/homing.cfg (rendered with Klipper's
single-brace Jinja environment, as klippy/extras/gcode_macro.py builds it)
through a small G-code interpreter, and the real HomingScope around G28.
Nothing here moves a motor; it asserts the order of the commands the macros
emit around G28.1 Y/X and what is left behind when G28.1 raises.

    python3 test/homing_scope/test_homing_y_current.py
"""
import ast
import configparser
import os
import sys

import jinja2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'klippy'))
sys.path.insert(0, HERE)

from extras import homing_scope  # noqa: E402
import test_homing_scope as ths  # noqa: E402

CFG = os.path.join(ROOT, 'core', 'M1', 'macros', 'homing.cfg')
RUN_CURRENT = {'stepper_x': 0.8, 'stepper_y': 0.8}


def load_sections():
    parser = configparser.RawConfigParser(strict=False)
    with open(CFG, encoding='utf-8') as f:
        parser.read_file(f)
    return parser


class Failure(Exception):
    pass


class Rig:
    """The M1 homing macros, a printer-status stub and a command recorder."""

    def __init__(self, homed='', y_current=None, fail_on=None):
        self.parser = load_sections()
        self.env = jinja2.Environment('{%', '%}', '{', '}')
        self.log = []                 # (command, params) in execution order
        self.fail_on = fail_on        # command text that raises, e.g. 'G28.1 Y'
        self.vars = {}
        for sect in self.parser.sections():
            if not sect.startswith('gcode_macro '):
                continue
            state = {}
            for key, val in self.parser.items(sect):
                if key.startswith('variable_'):
                    state[key[len('variable_'):]] = ast.literal_eval(val)
            self.vars[sect] = state
        if y_current is not None:
            self.vars['gcode_macro _HOMING_CFG'][
                'y_homing_current'] = y_current
        self.homed = homed
        self.driver = {'x': 0.8, 'y': 0.8}

    def status(self):
        printer = {
            'toolhead': {
                'homed_axes': self.homed,
                'position': {'x': 100., 'y': 100., 'z': 50.},
                'axis_maximum': {'x': 200.1, 'y': 174.1, 'z': 250.},
            },
            'configfile': {'settings': {
                'tmc2130 stepper_x': {'run_current': RUN_CURRENT['stepper_x']},
                'tmc2130 stepper_y': {'run_current': RUN_CURRENT['stepper_y']},
            }},
            'mcu toolhead': {'non_critical_disconnected': False},
            'bed_removal_detector': {'bedRemoved': False},
        }
        printer.update(self.vars)
        return printer

    def macro_text(self, name):
        for sect in self.parser.sections():
            if sect.lower() == ('gcode_macro ' + name).lower():
                return self.parser.get(sect, 'gcode')
        return None

    def run(self, text, rawparams=''):
        tmpl = self.env.from_string(text)
        def raise_error(msg):
            raise Failure(msg)
        out = tmpl.render(printer=self.status(), rawparams=rawparams,
                          params={},
                          action_raise_error=raise_error,
                          action_respond_info=lambda m: "")
        for line in out.splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            self.command(line)

    def command(self, line):
        name, _, rest = line.partition(' ')
        rest = rest.strip()
        if line == self.fail_on:
            self.log.append((name, rest))
            raise Failure("%s failed" % line)
        macro = self.macro_text(name)
        if macro is not None and name.upper() not in ('G28.1',):
            self.log.append((name, rest))
            self.run(macro, rawparams=rest)
            return
        if name == 'SET_GCODE_VARIABLE':
            kv = dict(p.split('=', 1) for p in rest.split())
            self.vars['gcode_macro ' + kv['MACRO']][kv['VARIABLE']] = (
                ast.literal_eval(kv['VALUE']))
            return
        self.log.append((name, rest))
        if name == 'SET_TMC_CURRENT':
            kv = dict(p.split('=', 1) for p in rest.split())
            self.driver[kv['STEPPER'][-1]] = float(kv['CURRENT'])

    def cmds(self):
        return [("%s %s" % (n, r)).strip() for n, r in self.log]


def index(cmds, text, start=0):
    for i in range(start, len(cmds)):
        if cmds[i] == text:
            return i
    raise AssertionError("%r not found after %d in %s" % (text, start, cmds))


def g28_rig(rig):
    """A printer whose G28 handler runs the real G28 macro with the real
    homing_scope (and its cleanup_gcode from homing.cfg) around it."""
    p = ths.FakePrinter()

    def g28(gcmd):
        # What the G28 macro does to reach G28.1: render and run it.
        rig.run(rig.macro_text('G28'), rawparams=gcmd)
    p.gcode.register_command('G28', g28)
    p.gcode.run_script_from_command = lambda text: rig.run(text)
    cleanup = load_sections().get('homing_scope', 'cleanup_gcode')
    scope = homing_scope.load_config(
        ths.FakeConfig(p, {'cleanup_gcode': cleanup}))
    p.send_event("klippy:ready")
    return p, scope


def test_home_y_lowers_then_restores_the_current():
    rig = Rig()
    rig.run(rig.macro_text('_HOME_Y'))
    c = rig.cmds()
    cur = index(c, 'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.6')
    tap = index(c, 'G28.1 Y')
    back = index(c, 'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.8', tap)
    ths.check(cur < tap < back, "Y current, G28.1 Y, then run current back")
    ths.check(c.count('G28.1 Y') == 1, "one G28.1 Y")
    ths.check(not any('SET_TMC_FIELD' in x for x in c),
              "the driver mode is not touched (the SpreadCycle test failed)")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8}, "run currents are back")
    ths.check(rig.vars['gcode_macro _HOMING_RESTORE']['pending'] == 0,
              "restore is no longer pending")


def test_failed_tap_without_cleanup_leaves_the_current_low():
    # Why homing_scope's cleanup_gcode exists: the macro stops at G28.1
    rig = Rig(fail_on='G28.1 Y')
    try:
        rig.run(rig.macro_text('_HOME_Y'))
    except Failure:
        pass
    else:
        raise AssertionError("G28.1 Y did not raise")
    ths.check(rig.driver['y'] == 0.6, "a bare macro leaves 0.6 A behind")
    ths.check(rig.vars['gcode_macro _HOMING_RESTORE']['pending'] == 1,
              "and records that a restore is owed")


def test_failed_tap_is_restored_by_the_scope():
    rig = Rig(fail_on='G28.1 Y')
    p, scope = g28_rig(rig)
    try:
        p.gcode.run('G28', 'Y')
    except Failure as e:
        ths.check("G28.1 Y" in str(e), "the G28.1 error reaches the caller")
    else:
        raise AssertionError("G28 Y did not raise")
    c = rig.cmds()
    tap = index(c, 'G28.1 Y')
    index(c, 'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.8', tap)
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8},
              "both currents are back after the failed G28.1 Y")
    ths.check(p.toolhead.kin.cleared == ["xyz"],
              "the scope still clears the homed state")
    ths.check(not scope.in_scope, "out of scope")


def test_good_g28_y_restores_once_through_the_scope():
    rig = Rig()
    p, scope = g28_rig(rig)
    p.gcode.run('G28', 'Y')
    ths.check(rig.cmds().count(
        'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.8') == 1,
        "a good G28 Y writes the restore once (the cleanup finds it done)")


def test_current_is_parametrised_for_y_only():
    rig = Rig(y_current=0.7)
    rig.run(rig.macro_text('_HOME_Y'))
    c = rig.cmds()
    tap = index(c, 'G28.1 Y')
    ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.7' in c[:tap],
              "y_homing_current 0.7 is applied to stepper_y before the tap")
    ths.check('SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.6' in c[:tap],
              "stepper_x keeps its own 0.6 A")
    ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.6' not in c,
              "no hard-coded 0.6 A on stepper_y")
    ths.check(rig.driver['y'] == 0.8, "run current restored")


def test_default_current_is_unchanged():
    rig = Rig()
    rig.run(rig.macro_text('_HOME_Y'))
    ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.6' in rig.cmds(),
              "default y_homing_current 0.6 A, as before")


def test_home_x_uses_the_y_current():
    rig = Rig(homed='xyz', y_current=0.7)
    rig.run(rig.macro_text('_HOME_X'))
    c = rig.cmds()
    tap = index(c, 'G28.1 X')
    ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.7' in c[:tap],
              "_HOME_X sets stepper_y from y_homing_current")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8}, "restored after X")


def test_failed_x_tap_restores_currents():
    rig = Rig(homed='xyz', fail_on='G28.1 X')
    p, scope = g28_rig(rig)
    try:
        p.gcode.run('G28', 'X')
    except Failure:
        pass
    else:
        raise AssertionError("G28 X did not raise")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8},
              "currents are back after a failed X home too")


def test_home_x_uses_the_y_current():
    rig = Rig(homed='xyz', y_current=0.5)
    rig.run(rig.macro_text('_HOME_X'))
    c = rig.cmds()
    tap = index(c, 'G28.1 X')
    ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.5' in c[:tap],
              "_HOME_X sets stepper_y from y_homing_current")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8}, "restored after X")


def test_failed_x_tap_restores_currents():
    rig = Rig(homed='xyz', fail_on='G28.1 X')
    p, scope = g28_rig(rig)
    try:
        p.gcode.run('G28', 'X')
    except Failure:
        pass
    else:
        raise AssertionError("G28 X did not raise")
    ths.check(rig.driver == {'x': 0.8, 'y': 0.8},
              "currents are back after a failed X home too")


MOTION_OR_DRIVER = ('SET_TMC_CURRENT', 'G28.1', 'AXIS_PROTECTED_MOVE', 'G1',
                    'SET_KINEMATIC_POSITION')


def refused(value, macro='_HOME_Y'):
    rig = Rig(homed='xyz', y_current=value)
    try:
        rig.run(rig.macro_text(macro))
    except Failure as e:
        ths.check("0.3 A to 0.80 A" in str(e),
                  "%r refused in %s, naming the range" % (value, macro))
    else:
        raise AssertionError("%r was accepted by %s" % (value, macro))
    ths.check(not any(n in MOTION_OR_DRIVER for n, _ in rig.log),
              "nothing moved or was set before the refusal")


def test_y_homing_current_bounds():
    for ok in (0.3, 0.5, 0.8):
        for macro in ('_HOME_Y', '_HOME_X'):
            rig = Rig(homed='xyz', y_current=ok)
            rig.run(rig.macro_text(macro))
            ths.check('SET_TMC_CURRENT STEPPER=stepper_y CURRENT=%s' % ok
                      in rig.cmds(), "%s A accepted by %s" % (ok, macro))
    for bad in (0.29, 0.0, -0.5, 0.81, 1.5, 'abc', '0.5', float('nan'),
                float('inf'), True):
        refused(bad)
    refused(0.9, '_HOME_X')
    refused('abc', '_HOME_X')


def test_g28_refuses_a_bad_current_before_the_z_lift():
    for bad in (0.9, 'abc'):
        for arg in ('Y', 'X', ''):
            rig = Rig(homed='', y_current=bad)
            p, scope = g28_rig(rig)
            try:
                p.gcode.run('G28', arg)
            except Failure as e:
                ths.check("allowed range" in str(e),
                          "G28 %s refuses %r" % (arg, bad))
            else:
                raise AssertionError("G28 accepted %r" % (bad,))
            ths.check(not any(n in MOTION_OR_DRIVER for n, _ in rig.log),
                      "no Z lift or motion before the refusal")


def test_z_only_g28_does_not_check_the_y_current():
    rig = Rig(homed='xy', y_current='abc')
    rig.run(rig.macro_text('G28'), rawparams='Z')
    ths.check('_HOMING_CHECK' not in rig.cmds(), "G28 Z skips the Y check")


def test_config_wires_the_cleanup():
    parser = load_sections()
    ths.check(parser.get('homing_scope', 'cleanup_gcode') == '_HOMING_RESTORE',
              "[homing_scope] cleanup_gcode names _HOMING_RESTORE")
    cfg = parser['gcode_macro _HOMING_CFG']
    ths.check('variable_y_homing_spreadcycle' not in cfg,
              "no SpreadCycle switch")
    ths.check(cfg['variable_y_homing_current'] == '0.6',
              "y_homing_current defaults to 0.6")


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d homing y-current tests passed" % len(tests))


if __name__ == '__main__':
    main()
