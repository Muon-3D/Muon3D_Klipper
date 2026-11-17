#!/usr/bin/env python3
"""Home All stows the bed (KAN-3811); every internal G28 does not.

A full user home (bare G28, which is what the panel's and Control's Home
All send) ends with the bed raised to Z_PARK's stow position. Only that:
G28 X / Y / Z / O, PARK=0 and STOW=0 do not, and every G28 that another
macro runs passes STOW=0 so those flows move exactly as before. This
renders the real G28 macro of core/M1 and core/M1-B0 with jinja2, no
klippy and no hardware (so it also runs on Windows).

    python3 test/home_stow/test_home_all_stow.py
"""
import os
import re
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import check_macro_config as cmc  # noqa: E402

BOARDS = ('M1', 'M1-B0')


def read(path):
    with open(path, 'rb') as f:
        return f.read().decode('utf-8')


def macros_of(board):
    """{section header: {option: value}} for every cfg of the board."""
    out = {}
    base = os.path.join(ROOT, 'core', board)
    for path in cmc.config_files(base):
        sections, _ = cmc.parse(read(path))
        for _n, header, options in sections:
            out.setdefault(header, {}).update(
                {k: cmc._join_value(v) for _l, k, v in options})
    return out


def render(board, rawparams, gate_open=True):
    macro = macros_of(board)['gcode_macro G28']['gcode']
    params = {}
    for tok in rawparams.split():
        if '=' in tok:
            # G28 is a traditional G-code: Klipper keeps the "=" in the value
            k, v = tok.split('=', 1)
            params[k.upper()] = '=' + v
    printer = {
        'toolhead': {'homed_axes': '', 'position': {'z': 0.},
                     'axis_maximum': {'x': 200.1, 'y': 174.1}},
        'z_power': {'z': 'locked' if gate_open else 'unlocked',
                    'xy_home_allowed': True},
        'mcu toolhead': {'non_critical_disconnected': False},
        'bed_removal_detector': {'bedRemoved': not gate_open},
        'gcode_macro G28': {'z_lifted_once': 0},
    }
    text = cmc._jinja_env().from_string(macro).render(
        rawparams=rawparams, params=params, printer=printer)
    return [ln.strip() for ln in text.split('\n') if ln.strip()]


class StowRender(unittest.TestCase):
    def stows(self, board, raw, gate_open=True):
        lines = render(board, raw, gate_open)
        return any(ln.startswith('Z_PARK') for ln in lines), lines

    def test_full_home_stows_after_the_z_home(self):
        for b in BOARDS:
            for raw in ('', 'STOW=1', 'PARK=1', 'PARK=1 STOW=1'):
                ok, lines = self.stows(b, raw)
                self.assertTrue(ok, (b, raw, lines))
                self.assertEqual(lines[-1], 'Z_PARK', (b, raw))
                self.assertTrue(any(ln.startswith('_HOME_Z') for ln in
                                    lines[:-1]), (b, raw))
                self.assertEqual(sum(ln.startswith('Z_PARK') for ln in
                                     lines), 1, (b, raw))

    def test_everything_else_does_not_stow(self):
        for b in BOARDS:
            for raw in ('X', 'Y', 'Z', 'X Y', 'X Y Z', 'O', 'O STOW=1',
                        'STOW=0', 'PARK=0', 'PARK=0 STOW=1', 'STOW=0 PARK=1',
                        'Z PARK=0 X_OFFSET=-5'):
                ok, lines = self.stows(b, raw)
                self.assertFalse(ok, (b, raw, lines))

    def test_the_stow_flag_alone_homes_everything(self):
        # "STOW=0" has no axis word, so it is still a home of X, Y and Z.
        for b in BOARDS:
            lines = render(b, 'STOW=0')
            self.assertTrue(any(ln.startswith('_HOME_Z') for ln in lines))
            self.assertTrue(any(ln.startswith('_HOME_X') for ln in lines))

    def test_a_refused_gate_stows_nothing_and_keeps_the_params(self):
        # The gate's CONTINUE button re-runs G28 with the same params, which
        # then stows; the refused run itself must not move.
        for b in ('M1', 'M1-B0'):
            lines = render(b, 'STOW=1', gate_open=False)
            self.assertFalse(any(ln.startswith('Z_PARK') for ln in lines))

    def test_o_is_not_a_stow_even_when_nothing_is_homed(self):
        for b in BOARDS:
            ok, lines = self.stows(b, 'O')
            self.assertFalse(ok, lines)
            self.assertTrue(any(ln.startswith('_HOME_Z') for ln in lines))


class StowTarget(unittest.TestCase):
    def test_stow_position_is_inside_the_z_travel(self):
        for b in BOARDS:
            m = macros_of(b)
            z = float(re.search(r'G0 Z([\d.]+)', m['gcode_macro Z_PARK']
                                ['gcode']).group(1))
            self.assertEqual(z, 140.0, b)
            self.assertLessEqual(z, float(m['stepper_z']['position_max']), b)


class InternalCallers(unittest.TestCase):
    def callers(self, board):
        """(file, line, text) of every macro line that starts a G28."""
        base = os.path.join(ROOT, 'core', board, 'macros')
        out = []
        for path in cmc.config_files(base):
            for n, ln in enumerate(cmc._klipper_lines(read(path)), 1):
                if re.match(r'^\s*G28(\s|$)', ln):
                    out.append((os.path.relpath(path, ROOT), n, ln.strip()))
        return out

    def test_every_internal_g28_passes_stow_0(self):
        for b in BOARDS:
            found = self.callers(b)
            self.assertGreater(len(found), 10, b)
            for f, n, ln in found:
                self.assertRegex(ln, r'\bSTOW=0\b', '%s:%d %s' % (f, n, ln))

    def test_no_other_g28_source_in_macros_restarts_a_full_home(self):
        # Quoted callbacks are the user's own params coming back
        # (HOMING_GATE CALLBACK="G28 {rawparams}") and must keep stowing.
        for b in BOARDS:
            for path in cmc.config_files(os.path.join(ROOT, 'core', b,
                                                      'macros')):
                for ln in cmc._klipper_lines(read(path)):
                    if 'CALLBACK=' in ln and 'G28' in ln:
                        self.assertNotIn('STOW=0', ln)

    def test_clients_may_send_the_flag(self):
        for b in BOARDS:
            self.assertRegex(read(os.path.join(ROOT, 'core', b, 'core.cfg')),
                             r'(?m)^\s*G28 [^\n]*\bSTOW=0\.\.1\b')


if __name__ == '__main__':
    unittest.main()
