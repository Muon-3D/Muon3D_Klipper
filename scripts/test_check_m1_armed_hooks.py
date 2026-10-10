#!/usr/bin/env python3
# Tests for check_m1_armed_hooks.py. Offline, no hardware, no klippy import.
#
#   python3 scripts/test_check_m1_armed_hooks.py
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_m1_armed_hooks as cah  # noqa: E402

SENSOR = (
    "[filament_switch_sensor filament_sensor_toolhead]\r\n"
    "pause_on_runout: False\r\n"
    "%s\r\n"
    "switch_pin: !toolhead:gpio3\r\n")
ARMED = "insert_gcode: INSERT_FILAMENT_SENSOR"
MACRO = (
    "[gcode_macro INSERT_FILAMENT_SENSOR]\r\n"
    "gcode:\r\n"
    "    {%% if not printing %%}\r\n"
    "        %s\r\n"
    "    {%% endif %%}\r\n")


def _check(files):
    with tempfile.TemporaryDirectory() as tmp:
        for name, text in files.items():
            path = os.path.join(tmp, name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', newline='') as handle:
                handle.write(text)
        return cah.check(tmp)


def _tree(insert=ARMED, flow='FILAMENT_INSERT_FLOW', extra=''):
    return {'core.cfg': SENSOR % insert + extra,
            'macros/filament.cfg': MACRO % flow}


class CheckTest(unittest.TestCase):
    def test_armed_tree_passes(self):
        self.assertEqual(_check(_tree()), [])

    def test_commented_out_insert_gcode_fails(self):
        # The KAN-232 shape: the line is still there, behind a '#'.
        failures = _check(_tree(insert='#' + ARMED))
        self.assertEqual(len(failures), 1)
        self.assertIn('no insert_gcode', failures[0])

    def test_empty_insert_gcode_fails(self):
        failures = _check(_tree(insert='insert_gcode:'))
        self.assertEqual(len(failures), 1)
        self.assertIn("insert_gcode is ''", failures[0])

    def test_repointed_insert_gcode_fails(self):
        failures = _check(_tree(insert='insert_gcode: RESPOND MSG=noop'))
        self.assertEqual(len(failures), 1)
        self.assertIn('not INSERT_FILAMENT_SENSOR', failures[0])

    def test_missing_sensor_section_fails(self):
        failures = _check({'macros/filament.cfg':
                           MACRO % 'FILAMENT_INSERT_FLOW'})
        self.assertEqual(failures,
                         ['[filament_switch_sensor filament_sensor_toolhead]'
                          ' is missing'])

    def test_later_section_disarming_it_fails(self):
        # Klipper merges a repeated section across includes and keeps the
        # last value, so a second empty insert_gcode anywhere wins.
        tree = _tree()
        tree['zz_override.cfg'] = SENSOR % 'insert_gcode:'
        self.assertEqual(len(_check(tree)), 1)

    def test_gutted_macro_fails(self):
        failures = _check(_tree(flow='RESPOND MSG="disabled"'))
        self.assertEqual(len(failures), 1)
        self.assertIn('no longer runs FILAMENT_INSERT_FLOW', failures[0])

    def test_commented_out_flow_call_fails(self):
        failures = _check(_tree(flow='# FILAMENT_INSERT_FLOW'))
        self.assertEqual(len(failures), 1)

    def test_longer_macro_name_is_not_the_flow(self):
        failures = _check(_tree(flow='FILAMENT_INSERT_FLOW_DISABLED'))
        self.assertEqual(len(failures), 1)

    def test_missing_macro_fails(self):
        failures = _check({'core.cfg': SENSOR % ARMED})
        self.assertEqual(failures,
                         ['[gcode_macro INSERT_FILAMENT_SENSOR] is missing'])

    def test_sensor_disabled_from_a_template_fails(self):
        extra = ("[delayed_gcode quiet_sensor]\r\n"
                 "initial_duration: 1\r\n"
                 "gcode:\r\n"
                 "    SET_FILAMENT_SENSOR"
                 " SENSOR=filament_sensor_toolhead ENABLE=0\r\n")
        failures = _check(_tree(extra=extra))
        self.assertEqual(len(failures), 1)
        self.assertIn('turns a filament sensor off', failures[0])

    def test_sensor_enabled_from_a_template_passes(self):
        extra = ("[gcode_macro WAKE]\r\n"
                 "gcode:\r\n"
                 "    SET_FILAMENT_SENSOR"
                 " SENSOR=filament_sensor_toolhead ENABLE=1\r\n")
        self.assertEqual(_check(_tree(extra=extra)), [])

    def test_shipped_m1_config_is_armed(self):
        self.assertEqual(cah.check(os.path.join(cah.ROOT, 'core', 'M1')), [])


if __name__ == '__main__':
    unittest.main()
