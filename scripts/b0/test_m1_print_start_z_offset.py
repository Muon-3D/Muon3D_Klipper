#!/usr/bin/env python3
# KAN-1730: PRINT_START and a saved probe z_offset past the safe range.
#
# The probe ignores such a value at start (0.0 stands in) and publishes
# z_offset_invalid. PRINT_START used to apply the RAW saved value from
# printer.configfile, and muon_guard's gate does not run in dev mode, so a
# saved -5 was applied there. Now PRINT_START
#
# * refuses to start while z_offset_invalid is set, in dev mode too, unless
#   IGNORE_Z_OFFSET=1 is given;
# * applies the probe's LIVE effective offset (the stand-in), never the raw
#   saved value.
#
#   python3 scripts/test_m1_print_start_z_offset.py   (or: pytest <this file>)
#
# Offline, no hardware. PRINT_START from the shipped print.cfg through
# klippy's GCodeDispatch and GCodeMacro, as test_m1_purge_line.py does.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402
from test_m1_led_modes import Status  # noqa: E402
from test_m1_purge_line import RECORDED  # noqa: E402
import gcode  # noqa: E402

START = 'PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA'
WANT = ("Saved Z offset -5.000 mm is outside the safe range +-2 mm. "
        "Recalibrate Z (PROBE_CALIBRATE or the panel's Z calibration) "
        "before printing.")


class PrintStartZOffset(_M1TestCase):
    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        self.status = {'z_offset': 0.0, 'z_offset_invalid': False,
                       'z_offset_saved': None}
        self.m1.printer.add_object('load_cell_probe', Status(self.status))
        # what the raw saved value would have been
        settings = self.m1.printer.lookup_object('configfile') \
            .get_status(0)['settings']
        settings['load_cell_probe']['z_offset'] = -5.0
        self.expected_error = False

    def tearDown(self):
        if self.expected_error:
            # the refusal is an error response and a command error
            self.m1.responses[:] = [r for r in self.m1.responses
                                    if not r.startswith('!!')]
            self.log.records.clear()
        self.m1.responses[:] = [r for r in self.m1.responses
                                if 'PURGE_LINE: no purge lane' not in r]
        super().tearDown()

    def offsets(self):
        return [l for l in self.m1.timeline
                if l.split()[0] == 'SET_GCODE_OFFSET']

    def invalid(self, saved=-5.0):
        self.status.update({'z_offset': 0.0, 'z_offset_invalid': True,
                            'z_offset_saved': saved})

    def test_healthy_probe_is_applied_as_before(self):
        self.status['z_offset'] = 0.35
        self.m1.timeline.clear()
        self.m1.run(START)
        self.assertEqual(self.offsets(), ['SET_GCODE_OFFSET Z=0.35'])

    def test_invalid_offset_refuses_even_in_dev_mode(self):
        # There is no guard here at all (dev mode): the macro is the check.
        self.invalid()
        self.m1.timeline.clear()
        self.expected_error = True
        with self.assertRaises(gcode.CommandError) as ctx:
            self.m1.run(START)
        self.assertIn(WANT, str(ctx.exception))
        self.assertEqual(self.offsets(), [])
        # nothing heated, homed or moved before the refusal
        self.assertEqual(
            [n for n in self.m1.names()
             if n in ('M104', 'M140', 'G28', 'G0', 'G1')], [])

    def test_the_raw_saved_value_is_never_applied(self):
        self.invalid()
        self.m1.timeline.clear()
        self.m1.run(START + ' IGNORE_Z_OFFSET=1')
        self.assertEqual(self.offsets(), ['SET_GCODE_OFFSET Z=0.0'])
        self.assertNotIn('-5', ' '.join(self.offsets()))

    def test_override_must_be_explicit(self):
        self.invalid()
        for extra in ('', ' IGNORE_Z_OFFSET=0'):
            self.m1.timeline.clear()
            self.expected_error = True
            with self.assertRaises(gcode.CommandError):
                self.m1.run(START + extra)
            self.m1.responses.clear()
            self.log.records.clear()

    def test_recalibrated_into_range_starts(self):
        self.invalid()
        self.status.update({'z_offset': 0.4, 'z_offset_invalid': False,
                            'z_offset_saved': None})
        self.m1.timeline.clear()
        self.m1.run(START)
        self.assertEqual(self.offsets(), ['SET_GCODE_OFFSET Z=0.4'])


if __name__ == '__main__':
    unittest.main()
