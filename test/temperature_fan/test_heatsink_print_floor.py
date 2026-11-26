#!/usr/bin/env python3
"""KAN-7031: the toolhead heatsink fan runs at 100% while printing.

PRINT_START raises the fan's min_speed to 1.0 (_HEATSINK_FAN_PRINT ON=1) and
PRINT_END, CANCEL_PRINT and _ON_PRINT_ERROR put the configured value back.
temperature_fan must hold that raised floor even where the curve says 0 (a
thermistor reading low), and keep upstream's meaning for the configured one.
Also checks the M1 [load_cell_home_z] wrong-way defaults stay inside the cap.

    python3 test/temperature_fan/test_heatsink_print_floor.py
"""
import configparser
import os
import sys
import unittest

import jinja2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

from extras import temperature_fan  # noqa: E402

M1 = os.path.join(ROOT, 'core', 'M1')
PRINT_CFG = os.path.join(M1, 'macros', 'print.cfg')
CORE_CFG = os.path.join(M1, 'core.cfg')


class FakeFan:
    def __init__(self):
        self.speeds = []

    def set_speed(self, value, print_time):
        self.speeds.append(value)


def make_tf(min_speed_conf=0., target=65.):
    tf = temperature_fan.TemperatureFan.__new__(
        temperature_fan.TemperatureFan)
    tf.fan = FakeFan()
    tf.min_speed_conf = tf.min_speed = min_speed_conf
    tf.max_speed_conf = tf.max_speed = 1.
    tf.target_temp = target
    tf.speed_delay = 0.
    tf.next_speed_time = 0.
    tf.last_speed_value = 0.
    return tf


class SpeedFloor(unittest.TestCase):
    def test_configured_zero_curve_zero_is_off(self):
        tf = make_tf()
        tf.set_tf_speed(1., 0.)
        self.assertEqual(tf.last_speed_value, 0.)

    def test_raised_floor_holds_where_curve_is_zero(self):
        tf = make_tf()
        tf.set_min_speed(1.)
        tf.set_tf_speed(1., 0.)
        self.assertEqual(tf.fan.speeds, [1.])

    def test_raised_floor_lifts_partial_curve(self):
        tf = make_tf()
        tf.set_min_speed(1.)
        tf.set_tf_speed(1., 0.3)
        self.assertEqual(tf.fan.speeds, [1.])

    def test_restore_returns_to_curve(self):
        tf = make_tf()
        tf.set_min_speed(1.)
        tf.set_tf_speed(1., 0.)
        tf.set_min_speed(tf.min_speed_conf)
        tf.set_tf_speed(2., 0.)
        self.assertEqual(tf.fan.speeds, [1., 0.])

    def test_target_zero_still_parks(self):
        tf = make_tf(target=0.)
        tf.set_min_speed(1.)
        tf.set_tf_speed(1., 0.)
        self.assertEqual(tf.last_speed_value, 0.)

    def test_configured_min_speed_keeps_upstream_meaning(self):
        tf = make_tf(min_speed_conf=0.2)
        tf.set_tf_speed(1., 0.)
        self.assertEqual(tf.last_speed_value, 0.)
        tf.set_tf_speed(2., 0.1)
        self.assertEqual(tf.fan.speeds, [0.2])


def macro_gcode(section):
    with open(PRINT_CFG) as f:
        lines = f.read().splitlines()
    start = lines.index('[gcode_macro %s]' % (section,)) + 1
    out, in_gcode = [], False
    for line in lines[start:]:
        if line.startswith('['):
            break
        if line.startswith('gcode:'):
            in_gcode = True
            continue
        if in_gcode and line.strip() and not line.strip().startswith('#'):
            out.append(line.strip())
    return out


class Macros(unittest.TestCase):
    def render(self, params, printer):
        src = '\n'.join(macro_gcode('_HEATSINK_FAN_PRINT'))
        env = jinja2.Environment('{%', '%}', '{', '}')
        text = env.from_string(src).render(params=params, printer=printer)
        return [l.strip() for l in text.splitlines() if l.strip()]

    def printer(self, min_speed=0.):
        name = 'temperature_fan toolhead_heatsink'
        return {name: {}, 'configfile': {'settings': {
            name: {'min_speed': min_speed, 'max_speed': 1.0}}}}

    def test_on_sets_full(self):
        self.assertEqual(self.render({'ON': '1'}, self.printer()), [
            'SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=toolhead_heatsink'
            ' MIN_SPEED=1.0 MAX_SPEED=1.0'])

    def test_off_restores_configured(self):
        self.assertEqual(self.render({'ON': '0'}, self.printer(0.25)), [
            'SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=toolhead_heatsink'
            ' MIN_SPEED=0.25 MAX_SPEED=1.0'])

    def test_no_fan_no_command(self):
        self.assertEqual(self.render({'ON': '1'}, {}), [])

    def test_print_start_raises_after_guard(self):
        g = macro_gcode('PRINT_START')
        self.assertEqual(g[0], '_HOTEND_FAN_GUARD WHAT=Print')
        self.assertEqual(g[1], '_HEATSINK_FAN_PRINT ON=1')

    def test_exit_paths_restore(self):
        for section in ('PRINT_END', 'CANCEL_PRINT', '_ON_PRINT_ERROR'):
            self.assertIn('_HEATSINK_FAN_PRINT ON=0', macro_gcode(section),
                          section)

    def test_print_end_restores_after_heaters_off(self):
        g = macro_gcode('PRINT_END')
        self.assertLess(g.index('TURN_OFF_HEATERS'),
                        g.index('_HEATSINK_FAN_PRINT ON=0'))

    def test_m1_fan_and_heatsink_sensor_unchanged(self):
        cp = configparser.RawConfigParser(strict=False,
                                          inline_comment_prefixes=('#',))
        cp.read(CORE_CFG)
        fan = cp['temperature_fan toolhead_heatsink']
        self.assertEqual(float(fan['min_speed']), 0.)
        self.assertEqual(float(fan['shutdown_speed']), 1.)
        with open(CORE_CFG) as f:
            self.assertIn('heatsink_sensor: temperature_fan toolhead_heatsink',
                          f.read())
        self.assertEqual(cp['load_cell_home_z']['wrong_way_retries'], '10')
        self.assertEqual(float(cp['load_cell_home_z']['wrong_way_settle']),
                         1.0)


if __name__ == '__main__':
    unittest.main()
