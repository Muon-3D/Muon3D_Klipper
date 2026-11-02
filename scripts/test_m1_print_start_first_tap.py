#!/usr/bin/env python3
# KAN-234: PRINT_START's first G28 taps the nozzle on the bed before the
# wipe has cleaned it. Pre-heated straight to the wipe's 200 C, the nozzle
# was hot enough to ooze by that tap and pressed the blob into the surface
# the load cell measures. This runs PRINT_START from the shipped print.cfg
# through klippy's GCodeDispatch and GCodeMacro (test_m1_heater_before_leds's
# M1, which stubs motion and heating) and checks the hot-end targets around
# that first G28:
#
# * no target set before it is above the no-ooze floor (135 C);
# * the climb to wipe_temp is set after it, and before NOZZLE_WIPE_SMART.
#
#   python3 scripts/test_m1_print_start_first_tap.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402
from test_m1_purge_line import RECORDED  # noqa: E402

NO_OOZE = 135.0     # PRINT_START's PREHEAT_EXTRUDER_TEMP
WIPE_TEMP = 200.0   # core/M1/macros/nozzle_wipe.cfg [nozzle_wipe_smart]


def target(line):
    for word in line.split()[1:]:
        if word.startswith('S'):
            return float(word[1:])
    return None


class FirstTap(_M1TestCase):
    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        self.settings = self.m1.printer.lookup_object('configfile') \
            .get_status(0)['settings']

    def print_start(self):
        self.m1.timeline.clear()
        self.m1.run('PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA')
        names = [line.split()[0] for line in self.m1.timeline]
        first_g28 = names.index('G28')
        wipe = names.index('NOZZLE_WIPE_SMART')
        before = [target(line) for line in self.m1.timeline[:first_g28]
                  if line.split()[0] == 'M104']
        between = [target(line)
                   for line in self.m1.timeline[first_g28:wipe]
                   if line.split()[0] == 'M104']
        return before, between

    def test_first_tap_is_below_ooze(self):
        self.settings['nozzle_wipe_smart'] = {'wipe_temp': WIPE_TEMP}
        before, between = self.print_start()
        self.assertTrue(before, "no pre-heat overlaps G28 (KAN-221)")
        self.assertTrue(all(t <= NO_OOZE for t in before),
                        "a target above %s before the first tap: %s"
                        % (NO_OOZE, before))
        self.assertEqual(between, [WIPE_TEMP],
                         "the climb to wipe_temp belongs after the tap")

    def test_low_wipe_temp_is_not_raised(self):
        # A wipe below the floor pre-heats to it and sets nothing more
        self.settings['nozzle_wipe_smart'] = {'wipe_temp': 120.0}
        before, between = self.print_start()
        self.assertEqual(before, [120.0])
        self.assertEqual(between, [])

    def test_no_wipe_section_no_preheat(self):
        self.settings.pop('nozzle_wipe_smart', None)
        before, between = self.print_start()
        self.assertEqual(before, [])
        self.assertEqual(between, [])


if __name__ == '__main__':
    unittest.main()
