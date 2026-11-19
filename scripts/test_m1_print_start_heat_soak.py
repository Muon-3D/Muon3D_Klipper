#!/usr/bin/env python3
# D-178 / KAN-4478: PRINT_START waits for the hot end, then soaks, before
# every Z home or probe (defence in depth for KAN-4364, load-cell probe drift
# while the hot end is changing temperature).
#
#   python3 scripts/test_m1_print_start_heat_soak.py   (or: pytest <this file>)
#
# Offline, no hardware: PRINT_START from the shipped print.cfg through
# klippy's GCodeDispatch, as test_m1_print_start_z_offset.py does.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402
from test_m1_led_modes import Status  # noqa: E402
from test_m1_purge_line import RECORDED  # noqa: E402

START = 'PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA'
# core/M1/macros/nozzle_wipe.cfg [nozzle_wipe_smart] stop_temp.
OOZE = 135.0


def is_z_probe(line):
    w = line.split()
    if w[0] == 'G28':
        # a bare G28 homes Z too; "G28 PARK=0 ..." has no axis letter
        axes = [a for a in w[1:] if '=' not in a]
        return not axes or 'Z' in axes
    return w[0] in ('BED_MESH_CALIBRATE', '_PRINT_START_MESH', 'PROBE',
                    'PROBE_CALIBRATE', 'Z_TAP')


class PrintStartHeatSoak(_M1TestCase):
    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        self.m1.gcode.register_command('_PRINT_START_MESH', None)
        self.m1.gcode.register_command('_PRINT_START_MESH', self.m1._record)
        self.m1.printer.add_object('load_cell_probe', Status(
            {'z_offset': 0.0, 'z_offset_invalid': False,
             'z_offset_saved': None}))
        self.m1.printer.lookup_object(
            'filament_switch_sensor filament_sensor_toolhead') \
            .get_status(0)['filament_detected'] = True

    def tearDown(self):
        self.m1.responses[:] = [r for r in self.m1.responses
                                if 'PURGE_LINE: no purge lane' not in r]
        super().tearDown()

    def timeline(self):
        self.m1.timeline.clear()
        self.m1.run(START)
        return list(self.m1.timeline)

    def test_every_z_home_or_probe_follows_wait_then_soak(self):
        tl = self.timeline()
        probes = [i for i, l in enumerate(tl) if is_z_probe(l)]
        # rough home, precise Z home, mesh
        self.assertEqual(len(probes), 3, tl)
        for i in probes:
            before = tl[:i]
            waits = [j for j, l in enumerate(before)
                     if l.startswith('MACHINE_HEAT_WAIT HEATER=extruder')
                     and 'MINIMUM=%s' % (OOZE - 0.5) in l]
            self.assertTrue(waits, 'no hot end wait before %s' % tl[i])
            # the soak is the G4 straight after the wait, nothing between
            # the soak and the probe but comments-free bookkeeping
            soak = [j for j, l in enumerate(before)
                    if l == 'G4 P20000' and j > waits[-1]]
            self.assertTrue(soak, 'no 20 s soak before %s' % tl[i])
            self.assertLess(waits[-1], soak[-1])

    def test_first_z_home_is_preceded_by_wait_and_soak(self):
        tl = self.timeline()
        first = next(i for i, l in enumerate(tl) if is_z_probe(l))
        self.assertEqual(tl[first - 1], 'G4 P20000')
        self.assertTrue(tl[first - 3].startswith(
            'MACHINE_HEAT_WAIT HEATER=extruder MINIMUM=134.5'))
        self.assertEqual(tl[first - 2], 'TEMPERATURE_WAIT SENSOR=extruder '
                         'MAXIMUM=140.0')

    def test_wait_is_for_the_anti_ooze_temperature_not_the_print_one(self):
        tl = self.timeline()
        first = next(i for i, l in enumerate(tl) if is_z_probe(l))
        for l in tl[:first]:
            if l.startswith('MACHINE_HEAT_WAIT HEATER=extruder'):
                self.assertNotIn('214.5', l)

    def test_soak_is_the_macro_variable(self):
        self.m1.printer.lookup_object('gcode_macro PRINT_START') \
            .variables['z_soak_s'] = 7
        tl = self.timeline()
        self.assertIn('G4 P7000', tl)
        self.assertNotIn('G4 P20000', tl)


if __name__ == '__main__':
    unittest.main()
