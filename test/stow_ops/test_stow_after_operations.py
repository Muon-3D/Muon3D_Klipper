#!/usr/bin/env python3
"""The bed rests at the top after a user operation (KAN-4298).

Z_PARK raises the bed to the stow position (Z140). After a full user home
(KAN-3811) and after a print it already did; this covers the user-facing
filament operations that end idle: UNLOAD_FILAMENT, the insert flow's Quick
Load and Full Purge. They stow only when no print is running or paused (a
RESUME depends on where the head is left) and, but for the purge's legacy
unhomed park, only with Z homed. A plain jog is not a macro and never
stows, and nothing on the print path gained a stow. This renders the real
macros of core/M1 and core/M1-B0 with jinja2: no klippy, no hardware.

    python3 test/stow_ops/test_stow_after_operations.py
"""
import os
import re
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import check_macro_config as cmc  # noqa: E402

BOARDS = ('M1', 'M1-B0')
STATES = (  # (print_stats.state, is_paused, sd active, label)
    ('standby', False, False, 'idle'),
    ('complete', False, False, 'complete'),
    ('cancelled', False, False, 'cancelled'),
    ('error', False, False, 'error'),
    ('printing', False, False, 'printing'),
    ('printing', False, True, 'printing from file'),
    ('paused', True, False, 'paused'),
    ('standby', True, False, 'pause flag alone'),
    ('standby', False, True, 'file active alone'),
)
JOB = ('printing', 'printing from file', 'paused', 'pause flag alone',
       'file active alone')
PAUSED = STATES[6]


def read(path):
    with open(path, 'rb') as f:
        return f.read().decode('utf-8')


_CACHE = {}


def macros_of(board):
    """{section header: {option: value}} for every cfg of the board."""
    if board not in _CACHE:
        out = {}
        base = os.path.join(ROOT, 'core', board)
        for path in cmc.config_files(base):
            sections, _ = cmc.parse(read(path))
            for _n, header, options in sections:
                out.setdefault(header, {}).update(
                    {k: cmc._join_value(v) for _l, k, v in options})
        _CACHE[board] = out
    return _CACHE[board]


def printer_for(state, paused, sd, homed):
    return {
        'toolhead': {'homed_axes': homed, 'position': {'z': 20.},
                     'axis_maximum': {'x': 200.1, 'y': 174.1, 'z': 160.}},
        'print_stats': {'state': state},
        'pause_resume': {'is_paused': paused},
        'virtual_sdcard': {'is_active': sd},
        'extruder': {'target': 0., 'can_extrude': 'true'},
    }


def render(board, name, st, homed='xyz'):
    macro = macros_of(board)['gcode_macro ' + name]['gcode']
    text = cmc._jinja_env().from_string(macro).render(
        rawparams='', params={}, printer=printer_for(*st[:3], homed))
    return [ln.strip() for ln in text.split('\n')
            if ln.strip() and not ln.strip().startswith('#')]


def zparks(board, name, st, homed='xyz'):
    """How many Z_PARK run when the macro (and the helper it calls) does."""
    n = 0
    for ln in render(board, name, st, homed):
        if ln.startswith('Z_PARK'):
            n += 1
        elif ln.startswith('_STOW_IF_IDLE'):
            n += sum(x.startswith('Z_PARK') for x in
                     render(board, '_STOW_IF_IDLE', st, homed))
    return n


class Helper(unittest.TestCase):
    def test_stows_only_when_idle_and_z_homed(self):
        for b in BOARDS:
            for st in STATES:
                for homed in ('xyz', 'z', 'xy', ''):
                    want = (st[3] not in JOB) and 'z' in homed
                    self.assertEqual(
                        zparks(b, '_STOW_IF_IDLE', st, homed),
                        1 if want else 0, (b, st, homed))

    def test_never_runs_the_unhomed_parking_routine(self):
        # No AXIS_HOME_MOVE, no Z_PARK: nothing at all with Z unhomed.
        for b in BOARDS:
            for st in STATES:
                self.assertEqual(render(b, '_STOW_IF_IDLE', st, 'xy'), [],
                                 (b, st))


class Operations(unittest.TestCase):
    def test_unload_and_quick_load_stow_standalone_only(self):
        for b in BOARDS:
            for name in ('UNLOAD_FILAMENT', 'FILAMENT_INSERT_QUICK_LOAD'):
                for st in STATES:
                    self.assertEqual(zparks(b, name, st),
                                     0 if st[3] in JOB else 1, (b, name, st))
                    self.assertEqual(zparks(b, name, st, 'xy'), 0,
                                     (b, name, st, 'z unhomed'))

    def test_unload_stows_after_the_unload_itself(self):
        for b in BOARDS:
            lines = render(b, 'UNLOAD_FILAMENT', STATES[0])
            stow = lines.index('_STOW_IF_IDLE')
            m400 = max(n for n, ln in enumerate(lines)
                       if ln.startswith('M400'))
            self.assertGreater(stow, m400)
            self.assertGreater(stow, lines.index(
                'RESTORE_GCODE_STATE NAME=UNLOAD_FILAMENT'))

    def test_full_purge_stows_when_not_in_a_print(self):
        for b in BOARDS:
            for st in STATES:
                for homed in ('xyz', 'xy'):
                    # Z unhomed: the purge sets Z only to move X and Y and
                    # its Z_PARK (the parking routine) is the old behaviour.
                    self.assertEqual(
                        zparks(b, 'FILAMENT_INSERT_FULL_PURGE', st, homed),
                        0 if st[3] in JOB else 1, (b, st, homed))

    def test_the_purge_stow_is_last_before_restoring_the_state(self):
        for b in BOARDS:
            lines = render(b, 'FILAMENT_INSERT_FULL_PURGE', STATES[0])
            i = max(n for n, ln in enumerate(lines) if ln == 'Z_PARK')
            self.assertEqual(lines[i + 1], 'M400', b)
            self.assertTrue(lines[i + 2].startswith('RESTORE_GCODE_STATE'), b)


class PrintPathAndPositioning(unittest.TestCase):
    def text(self, b, name):
        return macros_of(b)['gcode_macro ' + name]['gcode']

    def test_print_path_callers_do_not_stow(self):
        # They end the print elsewhere (PRINT_END's park) or hand back to a
        # RESUME; none calls the stow helper.
        for b in BOARDS:
            for name in ('PAUSE', 'RESUME', 'M600', '_CANCEL_M600',
                         '_DO_UNLOAD_M600', '_RUNOUT_UNLOAD',
                         '_FILAMENT_RUNOUT', 'INSERT_FILAMENT_SENSOR',
                         'FILAMENT_INSERT_FLOW', 'FILAMENT_INSERT_CANCEL',
                         'PRINT_START', 'CANCEL_PRINT', 'FINETUNE_RESONANCE',
                         '_BED_MESH_ENSURE', 'PURGE_LINE'):
                if 'gcode_macro ' + name in macros_of(b):
                    self.assertNotIn('_STOW_IF_IDLE', self.text(b, name),
                                     (b, name))

    def test_a_paused_print_flow_does_not_stow(self):
        # A runout pause: unload, insert, purge, then RESUME.
        for b in BOARDS:
            for name in ('UNLOAD_FILAMENT', 'FILAMENT_INSERT_QUICK_LOAD',
                         'FILAMENT_INSERT_FULL_PURGE'):
                self.assertEqual(zparks(b, name, PAUSED), 0, (b, name))

    def test_moves_and_jogs_do_not_stow(self):
        # A jog is G0/G1 from the Move screen, not a macro. The positioning
        # macros are moves of the same kind: the user put the bed there.
        for b in BOARDS:
            for name in ('CENTER', 'FRONT', 'ANTI_OOZE_NOZZLE_DOCK'):
                t = self.text(b, name)
                self.assertNotIn('Z_PARK', t, (b, name))
                self.assertNotIn('_STOW_IF_IDLE', t, (b, name))

    def test_only_the_listed_macros_stow(self):
        pat = r'(\{%[^}]*%\}\s*)?NAME\b'
        for b in BOARDS:
            helper, direct = set(), set()
            for header, opts in macros_of(b).items():
                for key in ('gcode', 'safe_gcode'):
                    lines = [ln.strip() for ln in
                             opts.get(key, '').split('\n')]
                    if any(re.match(pat.replace('NAME', '_STOW_IF_IDLE'), ln)
                           for ln in lines):
                        helper.add(header)
                    if any(re.match(pat.replace('NAME', 'Z_PARK'), ln)
                           for ln in lines):
                        direct.add(header)
            self.assertEqual(
                helper, {'gcode_macro UNLOAD_FILAMENT',
                         'gcode_macro FILAMENT_INSERT_QUICK_LOAD'}, b)
            expect = {'gcode_macro _STOW_IF_IDLE',
                      'gcode_macro PARK_PRINTER', 'gcode_macro G28',
                      'gcode_macro FILAMENT_INSERT_FULL_PURGE'}
            if b == 'M1':  # B0 has no safe park
                expect.add('print_end_park')
            self.assertEqual(direct, expect, b)


class PrintEndStillStows(unittest.TestCase):
    """A print ends at Z140, normally and by cancel (KAN-4298 confirms)."""

    def test_print_end_park_chain_reaches_z_park(self):
        m = macros_of('M1')
        end = m['gcode_macro PRINT_END']['gcode']
        lines = [ln.strip() for ln in end.split('\n')]
        self.assertLess(lines.index('_PRINT_END_PARK'),
                        lines.index('_PRINT_END_PARK_SAFE'))
        park = m['print_end_park']
        self.assertRegex(park['gcode'].rstrip(), r'CUT_FILAMENT$')
        # the cut ends with the park, whose first move is Z_PARK
        self.assertRegex(m['gcode_macro _CUT_FILAMENT_CUT']['gcode'],
                         r'(?m)^\s*PARK_PRINTER\s*$')
        self.assertRegex(m['gcode_macro PARK_PRINTER']['gcode'],
                         r'(?m)^\s*Z_PARK\s*$')
        # a refused park still stows when Z is homed
        self.assertRegex(park['safe_gcode'],
                         r"if 'z' in HA %\}\s*\n\s*Z_PARK\b")

    def test_park_printer_stows_whatever_the_state(self):
        # It runs inside PRINT_END, where the print is still "printing".
        for b in BOARDS:
            for st in STATES:
                self.assertEqual(render(b, 'PARK_PRINTER', st)[0], 'Z_PARK',
                                 (b, st))

    def test_cancel_print_ends_through_print_end(self):
        cancel = macros_of('M1')['gcode_macro CANCEL_PRINT']['gcode']
        self.assertRegex(cancel, r'(?m)^\s*PRINT_END\s*$')

    def test_the_stow_position_is_140(self):
        for b in BOARDS:
            z = re.search(r'G0 Z([\d.]+)',
                          macros_of(b)['gcode_macro Z_PARK']['gcode'])
            self.assertEqual(float(z.group(1)), 140.0, b)


if __name__ == '__main__':
    unittest.main()
