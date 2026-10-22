#!/usr/bin/env python3
# KAN-742 / D-048: the umbilical unlock press clears the contact_guard
# offset its filament retract leaves, with a 1.2 mm Z lift and return (the
# guard's steady-over-Z rule), last, after the heater and LED steps. With
# no room to lift, or Z unhomed, it falls back to REZERO TRY=1, never FORCE.
# Not while printing, and not on an unarmed press or without the guard.
#
#   python3 scripts/test_m1_unlock_guard_offset.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only: the shipped cfg files run through
# klippy's own GCodeDispatch and GCodeMacro, with the M1 fakes of
# test_m1_heater_before_leds.py.  What the lift does to the guard is in
# test/contact_guard/test_contact_guard.py (test_unlock_retract_then_lift).
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import M1, _M1TestCase  # noqa: E402
from test_m1_led_modes import Status  # noqa: E402

LIFT = ['G4 P300', 'SAVE_GCODE_STATE NAME=_UNLOCK_GUARD', 'G91',
        'G1 Z1.2 F300', 'G1 Z-1.2 F300',
        'RESTORE_GCODE_STATE NAME=_UNLOCK_GUARD']


class UnlockClearsGuardOffset(_M1TestCase):
    def setUp(self):
        super().setUp()
        self.m1.printer.add_object('contact_guard', Status({
            'contact': True, 'load': 717295, 'active': False,
            'enabled': True}))
        self.m1.gcode.register_command('SET_CONTACT_GUARD', self.m1._record)
        self.toolhead = self.m1.printer.lookup_object('toolhead').status
        self.m1.print_stats['state'] = 'paused'   # the KAN-742 incident

    def press(self):
        self.arm_unlock()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        return [line.strip() for line in self.m1.timeline]

    def test_lift_and_return_last(self):
        lines = self.press()
        start = lines.index(LIFT[0])
        self.assertEqual(lines[start:start + len(LIFT)], LIFT)
        names = self.m1.names()
        self.assertLess(names.index('FORCE_MOVE'), start,
                        'the lift comes after the retract')
        self.assertLess(self.m1.heater_off_index(), start)
        self.assertLess(self.m1.first_led(), start,
                        'the lift comes after LED_CONNECTED')
        self.assertNotIn('SET_CONTACT_GUARD', names)

    def test_idle_lifts_too(self):
        self.m1.print_stats['state'] = 'standby'
        self.assertIn('G1 Z1.2 F300', self.press())

    def test_no_room_falls_back_to_rezero_try(self):
        self.toolhead['position']['z'] = 179.5    # axis_maximum 180
        lines = self.press()
        self.assertNotIn('G1 Z1.2 F300', lines)
        rezero = [l for l in lines if l.startswith('SET_CONTACT_GUARD')]
        self.assertEqual(rezero, ['SET_CONTACT_GUARD REZERO=1 TRY=1'])

    def test_z_unhomed_falls_back_to_rezero_try(self):
        self.toolhead['homed_axes'] = 'xy'
        lines = self.press()
        self.assertNotIn('G1 Z1.2 F300', lines)
        self.assertIn('SET_CONTACT_GUARD REZERO=1 TRY=1', lines)

    def test_never_force(self):
        for state in ('paused', 'standby'):
            for z, homed in ((40., 'xyz'), (179.5, 'xyz'), (40., 'xy')):
                self.m1.timeline.clear()
                self.m1.print_stats['state'] = state
                self.toolhead['position']['z'] = z
                self.toolhead['homed_axes'] = homed
                lines = self.press()
                self.assertFalse(any('FORCE=' in l for l in lines
                                     if 'CONTACT_GUARD' in l), lines)

    def test_not_while_printing(self):
        self.m1.print_stats['state'] = 'printing'
        lines = self.press()
        self.assertNotIn('G1 Z1.2 F300', lines)
        self.assertNotIn('SET_CONTACT_GUARD', self.m1.names())

    def test_not_when_unarmed(self):
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertNotIn('G1 Z1.2 F300',
                         [l.strip() for l in self.m1.timeline])


class UnlockWithoutGuard(_M1TestCase):
    def test_nothing_without_contact_guard(self):
        self.m1.print_stats['state'] = 'paused'
        self.arm_unlock()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertNotIn('G1 Z1.2 F300',
                         [l.strip() for l in self.m1.timeline])


if __name__ == '__main__':
    unittest.main()
