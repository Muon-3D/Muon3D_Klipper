"""fan_modes: the print-mode contract Control v4 relies on, and M106/M107.

Control v4 (Muon3D_Fluidd src/services/control/print-mode.ts) sends
SET_PRINT_MODE MODE=quiet|normal|turbo and reads printer.fan_modes back.
This drives the real FanModes with the real [fan_modes] section of
core/M1/core.cfg, and renders the real M106 / M107 / TOOLHEAD_DISCONNECT
macros with Klipper's single-brace Jinja, with and without [fan_modes].
Nothing here moves a motor.

    python3 test/fan_modes/test_print_modes.py
"""
import configparser
import os
import re
import sys
import unittest

import jinja2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, HERE)

from test_fan_modes import ConfigError, FakeGcmd, build, DUTY  # noqa: E402
from extras import fan_modes  # noqa: E402

CORE_CFG = os.path.join(ROOT, 'core', 'M1', 'core.cfg')
MACRO_DIR = os.path.join(ROOT, 'core', 'M1', 'macros')


def read_cfg(paths):
    # Klipper drops whole-line comments and cuts a line at its first '#'.
    cp = configparser.RawConfigParser(strict=False)
    for path in paths:
        with open(path, encoding='utf-8') as f:
            lines = [re.sub(r'\s*#.*$', '', ln) if not ln.lstrip()
                     .startswith('{') else ln
                     for ln in f.read().splitlines()]
        lines = ['' if ln.lstrip().startswith('#') else ln for ln in lines]
        cp.read_string('\n'.join(lines))
    return cp


def core_fan_modes():
    with open(CORE_CFG, encoding='utf-8') as f:
        text = f.read()
    m = re.search(r'^\[fan_modes\]\n.*?(?=^\[)', text, re.S | re.M)
    cp = configparser.RawConfigParser(strict=False)
    cp.read_string('\n'.join(re.sub(r'#.*$', '', ln)
                             for ln in m.group(0).splitlines()))
    return dict(cp['fan_modes'])


CORE = core_fan_modes()


class PrintModes(unittest.TestCase):
    def setUp(self):
        self.fm, self.printer = build(CORE)
        self.fan = self.printer.objects['fan'].fan

    def cmd(self, name, **params):
        gcmd = FakeGcmd(params)
        self.printer.gcode.commands[name](gcmd)
        return gcmd

    def test_core_cfg_offers_the_three_modes(self):
        self.assertEqual(self.fm.get_status(0.)['modes'],
                         ['normal', 'quiet', 'turbo'])
        self.assertEqual(self.fm.get_status(0.)['mode'], 'normal')

    def test_each_mode_sets_the_part_fan(self):
        # duty for a 100% and a 50% command under each mode, from the
        # core.cfg speed_table (ADR 0056)
        expect = {'normal': (0.3020, 0.1551), 'quiet': (0.2000, 0.1264),
                  'turbo': (1.0000, 0.2771)}
        for mode, (full, half) in sorted(expect.items()):
            self.cmd('SET_PRINT_MODE', MODE=mode)
            self.cmd('SET_PART_FAN', SPEED=1.)
            self.assertAlmostEqual(self.fan.speed, full, places=4, msg=mode)
            self.cmd('SET_PART_FAN', SPEED=0.5)
            self.assertAlmostEqual(self.fan.speed, half, places=4, msg=mode)
            self.cmd('SET_PART_FAN', SPEED=0.)
            self.assertEqual(self.fan.speed, 0., msg=mode)

    def test_switching_mode_reapplies_the_command(self):
        self.cmd('SET_PART_FAN', SPEED=1.)
        for mode, duty in (('quiet', 0.2), ('turbo', 1.0), ('normal', 0.302)):
            self.cmd('SET_PRINT_MODE', MODE=mode)
            self.assertAlmostEqual(self.fan.speed, duty, places=4, msg=mode)
            self.assertEqual(self.fm.get_status(0.)['mode'], mode)

    def test_status_carries_the_contract_keys(self):
        self.cmd('SET_PRINT_MODE', MODE='quiet')
        self.cmd('SET_PART_FAN', SPEED=1.)
        st = self.fm.get_status(0.)
        self.assertEqual(sorted(st), ['commanded_speed', 'mode', 'modes',
                                      'output_speed', 'target_rpm'])
        self.assertEqual(st['mode'], 'quiet')
        self.assertIsInstance(st['mode'], str)
        self.assertEqual(st['modes'], ['normal', 'quiet', 'turbo'])
        self.assertEqual(st['commanded_speed'], 1.)
        self.assertAlmostEqual(st['target_rpm'], 13680.)
        self.assertAlmostEqual(st['output_speed'], 0.2)

    def test_status_example_without_a_speed_table(self):
        # The example in the Control v4 brief.
        fm, printer = build(DUTY)
        printer.gcode.commands['SET_PART_FAN'](FakeGcmd({'SPEED': 0.6}))
        st = fm.get_status(0.)
        self.assertEqual(st['mode'], 'normal')
        self.assertEqual(st['modes'], ['normal', 'quiet', 'turbo'])
        self.assertEqual(st['commanded_speed'], 0.6)
        self.assertIsNone(st['target_rpm'])
        self.assertAlmostEqual(st['output_speed'], 0.18)

    def test_unknown_mode_is_an_error(self):
        self.cmd('SET_PART_FAN', SPEED=1.)
        for mode in ('fast', 'normal2', ''):
            with self.assertRaises(ConfigError):
                self.cmd('SET_PRINT_MODE', MODE=mode)
            self.assertEqual(self.fm.mode, 'normal')
            self.assertAlmostEqual(self.fan.speed, 0.302, places=4)

    def test_bare_set_print_mode_reports(self):
        gcmd = self.cmd('SET_PRINT_MODE')
        self.assertIn('normal', gcmd.info[0])
        self.assertEqual(self.fm.mode, 'normal')

    def test_restart_starts_in_default_mode(self):
        self.cmd('SET_PRINT_MODE', MODE='turbo')
        self.cmd('SET_PART_FAN', SPEED=1.)
        self.assertEqual(self.fm.mode, 'turbo')
        # A Klipper restart builds the module again from the same config.
        fm, printer = build(CORE)
        st = fm.get_status(0.)
        self.assertEqual(CORE['default_mode'], 'normal')
        self.assertEqual(st['mode'], 'normal')
        self.assertEqual(st['commanded_speed'], 0.)
        self.assertEqual(printer.objects['fan'].fan.set, [])
        # and the mode is kept nowhere that outlives the process
        with open(fan_modes.__file__, encoding='utf-8') as f:
            src = f.read()
        for word in ('save_variables', 'SAVE_VARIABLE', 'open('):
            self.assertNotIn(word, src)


MACROS = read_cfg([os.path.join(MACRO_DIR, 'fan.cfg'),
                   os.path.join(MACRO_DIR, 'toolhead.cfg')])
ENV = jinja2.Environment('{%', '%}', '{', '}')


def render(name, rawparams='', fan_modes=None, disconnected=False,
           fan_speed=0.):
    params = {}
    for tok in rawparams.split():
        k, _, v = tok.partition('=')
        if not v and k[:1].isalpha():
            k, v = k[0], k[1:]
        params[k.upper()] = v
    printer = {'mcu toolhead': {'non_critical_disconnected': disconnected},
               'fan': {'speed': fan_speed},
               'print_stats': {'state': 'standby'}}
    if fan_modes is not None:
        printer['fan_modes'] = fan_modes
    out = ENV.from_string(MACROS['gcode_macro ' + name]['gcode']).render(
        printer=printer, params=params, rawparams=rawparams)
    return [ln.strip() for ln in out.splitlines() if ln.strip()]


MODES = {'mode': 'normal', 'modes': ['normal', 'quiet', 'turbo'],
         'commanded_speed': 0.5, 'target_rpm': None, 'output_speed': 0.15}


class MacrosWithoutFanModes(unittest.TestCase):
    # What master's M106 sends, and Klipper's own M107: the raw fan.
    def test_m106_is_the_raw_fan(self):
        self.assertEqual(render('M106', 'S128'), ['M106.1 S128'])
        self.assertEqual(render('M106', 'S0'), ['M106.1 S0'])
        self.assertEqual(render('M106', ''), ['M106.1'])

    def test_m107_is_klippers_own(self):
        self.assertEqual(render('M107'), ['M107.1'])

    def test_disconnected_m106_saves_and_holds_off(self):
        out = render('M106', 'S128', disconnected=True)
        self.assertEqual(out[1:], [
            'SET_GCODE_VARIABLE MACRO=TOOLHEAD_DISCONNECT '
            'VARIABLE=last_partfan_s VALUE=128', 'M106.1 S0'])
        self.assertTrue(out[0].startswith('RESPOND MSG="Toolhead'))
        # master's macro defaulted a bare S to 0 here
        self.assertIn('VALUE=0', render('M106', '', disconnected=True)[1])

    def test_disconnect_saves_the_fan_speed(self):
        out = [ln for ln in render('TOOLHEAD_DISCONNECT', fan_speed=0.5)
               if 'last_partfan_s' in ln]
        self.assertEqual(out, ['SET_GCODE_VARIABLE MACRO=TOOLHEAD_DISCONNECT '
                               'VARIABLE=last_partfan_s VALUE=127'])


class MacrosWithFanModes(unittest.TestCase):
    def test_m106_goes_through_the_mode(self):
        self.assertEqual(render('M106', 'S255', MODES),
                         ['SET_PART_FAN SPEED=1.0'])
        self.assertEqual(render('M106', '', MODES),
                         ['SET_PART_FAN SPEED=1.0'])
        self.assertEqual(render('M106', 'S0', MODES),
                         ['SET_PART_FAN SPEED=0.0'])

    def test_m107_clears_the_command(self):
        self.assertEqual(render('M107', '', MODES), ['SET_PART_FAN SPEED=0'])

    def test_disconnect_saves_the_commanded_speed(self):
        out = [ln for ln in render('TOOLHEAD_DISCONNECT', '', MODES,
                                   fan_speed=0.15)
               if 'last_partfan_s' in ln]
        self.assertEqual(out, ['SET_GCODE_VARIABLE MACRO=TOOLHEAD_DISCONNECT '
                               'VARIABLE=last_partfan_s VALUE=128'])


if __name__ == '__main__':
    unittest.main()
