#!/usr/bin/env python3
# Tests for check_macro_config.py. Offline, no hardware, no klippy import.
#
#   python3 scripts/test_check_macro_config.py
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_macro_config as cmc  # noqa: E402


def _check(text, name='m.cfg'):
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, name), 'w', newline='') as handle:
            handle.write(text)
        return cmc.check(tmp)


class ParseTest(unittest.TestCase):
    def test_indented_header_is_a_continuation_line(self):
        # The shape of core/M1/macros/debug.cfg before KAN-297.
        sections, indented = cmc.parse(
            "[gcode_macro WEAR_TEST]\r\n"
            "description: wear\r\n"
            "gcode:\r\n"
            "  G28\r\n"
            "\r\n"
            "    [gcode_macro TEST_SPEED]\r\n"
            "description: speed\r\n"
            "gcode:\r\n"
            "  G1 X0\r\n")
        self.assertEqual([h for _l, h, _o in sections],
                         ['gcode_macro WEAR_TEST'])
        self.assertEqual(indented, [(6, 'gcode_macro TEST_SPEED')])
        keys = [k for _l, k, _v in sections[0][2]]
        self.assertEqual(keys, ['description', 'gcode', 'description',
                                'gcode'])

    def test_bracketed_gcode_argument_is_not_a_header(self):
        _sections, indented = cmc.parse(
            "[gcode_macro X]\ngcode:\n  M117 [Done]\n  [1, 2]\n")
        self.assertEqual(indented, [])

    def test_comments_are_cut_at_hash(self):
        sections, _indented = cmc.parse(
            "[stepper_x] # comment\nrotation_distance: 40 # 34t\n")
        self.assertEqual(sections[0][1], 'stepper_x')
        self.assertEqual(cmc._join_value(sections[0][2][0][2]), '40')


class CheckTest(unittest.TestCase):
    def test_clean_config_passes(self):
        self.assertEqual(_check(
            "[gcode_macro G28]\n"
            "rename_existing: G28.1\n"
            "variable_z_lifted_once: 0\n"
            "gcode:\n"
            "  {% set ha = printer.toolhead.homed_axes %}\n"
            "  {% if 'z' in ha %}\n"
            "    {% set ok = not printer['mcu toolhead'].disconnected %}\n"
            "    {% if ok and z_lifted_once == 0 %}\n"
            "      G28.1 {rawparams}\n"
            "    {% endif %}\n"
            "  {% endif %}\n"
            "  {% for ax in params %}{ loop.index }{% endfor %}\n"
            "  {% set ns = namespace(n=0) %}{% set ns.n = 1 %}\n"
            "  {action_respond_info(ns.n|string)}\n"
            "\n"
            "[delayed_gcode start]\n"
            "initial_duration: 1\n"
            "gcode:\n"
            "  {% if printer.idle_timeout.state == 'Idle' %}M117 hi{% endif %}\n"
        ), [])

    def test_indented_header_fails(self):
        failures = _check(
            "[gcode_macro A]\ngcode:\n  G28\n\n"
            "    [gcode_macro B]\ngcode:\n  G1 X0\n")
        self.assertEqual(len(failures), 2, failures)
        self.assertIn(":5: [gcode_macro B] is indented", failures[0])
        self.assertIn(":6: [gcode_macro A] sets gcode again (first at line 2)",
                      failures[1])

    def test_duplicate_key_fails(self):
        failures = _check(
            "[stepper_y]\nrotation_distance: 40\nrotation_distance: 40\n")
        self.assertEqual(len(failures), 1)
        self.assertIn(":3: [stepper_y] sets rotation_distance again",
                      failures[0])

    def test_bare_undefined_name_fails(self):
        # KAN-79: is_paused was a variable_ on another macro.
        failures = _check(
            "[gcode_macro PAUSE]\nvariable_is_paused: 0\ngcode:\n  M117\n\n"
            "[gcode_macro RESUME]\ngcode:\n"
            "  {% if not is_paused %}\n    M117 not paused\n  {% endif %}\n")
        self.assertEqual(len(failures), 1, failures)
        self.assertIn(":8: [gcode_macro RESUME] gcode reads 'is_paused'",
                      failures[0])

    def test_macro_only_context_is_not_given_to_other_templates(self):
        failures = _check(
            "[delayed_gcode d]\ngcode:\n  M117 {params.X}\n")
        self.assertEqual(len(failures), 1)
        self.assertIn("reads 'params'", failures[0])

    def test_syntax_error_fails(self):
        failures = _check(
            "[gcode_macro A]\ngcode:\n  M117\n  {% if x %}\n")
        self.assertEqual(len(failures), 1)
        self.assertIn("does not parse as a template", failures[0])

    def test_missing_tree_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            failures = cmc.check(tmp)
        self.assertEqual(len(failures), 1)
        self.assertIn("found no .cfg files", failures[0])

    def test_shipped_config_is_clean(self):
        self.assertEqual(cmc.check(os.path.join(cmc.ROOT, 'core')), [])


if __name__ == '__main__':
    unittest.main()
