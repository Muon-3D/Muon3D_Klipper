#!/usr/bin/env python3
"""The rise to the stow position: its speed and its company (KAN-4683/4684).

KAN-4684: Z_PARK raises the bed at 10 mm/s. With the plate height known to
the contact guard it runs at Z_PARK's known_speed, never above [printer]
max_z_velocity.

This renders the real macros of core/M1 with jinja2, no klippy and no
hardware (so it runs on Windows).

    python3 test/home_stow/test_stow_rise.py
"""
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_home_all_stow as ths  # noqa: E402

cmc = ths.cmc
MAX_Z_VELOCITY = 15.0


def render(macro, params=None, z=10., x=100., y=60., homed='xyz',
           guard='known', max_z_velocity=MAX_Z_VELOCITY):
    text = ths.macros_of('M1')['gcode_macro ' + macro]['gcode']
    printer = {
        'toolhead': {'homed_axes': homed,
                     'position': {'x': x, 'y': y, 'z': z},
                     'axis_maximum': {'x': 200.1, 'y': 174.1}},
        'configfile': {'settings': {'printer': {
            'max_z_velocity': max_z_velocity}}},
        'gcode_macro Z_PARK': {'known_speed': known_speed()},
    }
    if guard == 'known':
        printer['contact_guard'] = {'z_known': True}
    elif guard == 'unknown':
        printer['contact_guard'] = {'z_known': False}
    elif guard == 'no_key':
        printer['contact_guard'] = {'move_guard': 'load_cell'}
    out = cmc._jinja_env().from_string(text).render(
        params=params or {}, printer=printer, **macro_vars(macro))
    return [ln.strip() for ln in out.split(chr(10)) if ln.strip()
            and not ln.strip().startswith((';', '#'))]


def macro_vars(macro):
    # a macro's own variable_* options are bare names in its template
    opts = ths.macros_of('M1')['gcode_macro ' + macro]
    out = {}
    for k, v in opts.items():
        if k.startswith('variable_'):
            out[k[len('variable_'):]] = float(v)
    return out


def known_speed():
    return macro_vars('Z_PARK')['known_speed']


def feed_of(line):
    return float(re.search(r'\bF([\d.]+)', line).group(1))


class ZParkSpeed(unittest.TestCase):
    def rise(self, **kw):
        lines = render('Z_PARK', **kw)
        move = [ln for ln in lines if ln.startswith('G0 Z140')]
        self.assertEqual(len(move), 1, lines)
        return feed_of(move[0]) / 60.

    def test_a_known_height_rises_at_the_known_speed(self):
        self.assertEqual(self.rise(), 15.0)

    def test_an_unknown_height_rises_at_ten(self):
        for guard in ('unknown', 'none', 'no_key'):
            self.assertEqual(self.rise(guard=guard), 10.0, guard)

    def test_never_above_max_z_velocity(self):
        self.assertEqual(self.rise(max_z_velocity=12.), 12.0)
        self.assertEqual(self.rise(max_z_velocity=10.), 10.0)
        self.assertLessEqual(self.rise(max_z_velocity=40.), known_speed())

    def test_the_known_speed_is_inside_the_configured_ceiling(self):
        cfg = ths.macros_of('M1')['printer']
        ceiling = float(cfg['max_z_velocity'])
        self.assertEqual(ceiling, MAX_Z_VELOCITY)
        self.assertLessEqual(known_speed(), ceiling)
        # and the acceleration the rise starts with is not touched here
        self.assertEqual(float(cfg['max_z_accel']), 1000.)

    def test_z_speed_still_overrides(self):
        self.assertEqual(self.rise(params={'Z_SPEED': '300'}), 5.0)

    def test_unhomed_z_keeps_the_parking_routine(self):
        lines = render('Z_PARK', homed='xy')
        self.assertEqual(lines, ['AXIS_HOME_MOVE AXIS=Z'])


if __name__ == '__main__':
    unittest.main()
