#!/usr/bin/env python3
# KAN-742 / D-048: the umbilical unlock press leaves a contact_guard offset
# (filament tension after its retract). Review #86 R5-2: the press must not
# move the machine to clear it, because a hand is on the toolhead. It tells
# the user, and tries a REZERO that is never FORCE (TRY=1, so a refusal
# does not stop the macro). A 1 mm Z lift clears the offset: a jog,
# PRINT_END's lift, or G28 (test/contact_guard: test_unlock_retract_then_lift,
# test_print_end_lift_clears_unlock_offset). It runs last, after the heater
# and LED steps, and not on an unarmed press or without the guard.
#
#   python3 scripts/test_m1_unlock_guard_offset.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only: the shipped cfg files run through
# klippy's own GCodeDispatch and GCodeMacro, with the M1 fakes of
# test_m1_heater_before_leds.py.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import M1, _M1TestCase  # noqa: E402
from test_m1_led_modes import Status  # noqa: E402

MOTION = ('G0', 'G1', 'G4', 'G91', 'SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE')
MESSAGE = 'lift Z 1 mm (or home) before moving X or Y'


class UnlockReportsGuardOffset(_M1TestCase):
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

    def after_retract(self):
        names = self.m1.names()
        return names[names.index('COLD_RETRACT') + 1:]

    def test_no_motion_in_any_state(self):
        for state in ('paused', 'standby', 'complete'):
            for z, homed in ((40., 'xyz'), (179.5, 'xyz'), (40., 'xy')):
                self.m1.timeline.clear()
                self.m1.print_stats['state'] = state
                self.toolhead['position']['z'] = z
                self.toolhead['homed_axes'] = homed
                self.press()
                moved = [n for n in self.after_retract() if n in MOTION]
                self.assertEqual(moved, [], (state, z, homed,
                                             self.m1.timeline))

    def test_rezero_try_never_force_last(self):
        lines = self.press()
        rezero = [l for l in lines if l.startswith('SET_CONTACT_GUARD')]
        self.assertEqual(rezero, ['SET_CONTACT_GUARD REZERO=1 TRY=1'])
        names = self.m1.names()
        at = names.index('SET_CONTACT_GUARD')
        self.assertLess(names.index('COLD_RETRACT'), at)
        self.assertLess(self.m1.heater_off_index(), at)
        self.assertLess(self.m1.first_led(), at)

    def test_tells_the_user(self):
        self.press()
        self.assertTrue(any(MESSAGE in r for r in self.m1.responses),
                        self.m1.responses)

    def test_printing_press_touches_nothing(self):
        # A print is running: COLD_RETRACT refuses, so the macro returns
        # before the heater, the retract or any stepper. The release must
        # then have nothing to restore, and the user is told to pause.
        self.m1.print_stats['state'] = 'printing'
        for name, value in (('e_was_enabled', 1), ('changed_hotend_target', 1)):
            self.m1.run('SET_GCODE_VARIABLE MACRO='
                        'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED '
                        'VARIABLE=%s VALUE=%d' % (name, value))
        self.m1.timeline.clear()
        self.m1.extruder['target'] = 215.
        self.press()
        names = self.m1.names()
        for n in ('M104', 'COLD_RETRACT', 'SET_STEPPER_ENABLE',
                  'SET_CONTACT_GUARD', 'PAUSE') + MOTION:
            self.assertNotIn(n, names, self.m1.timeline)
        self.assertIn('action:prompt_text Pause the print before unlocking '
                      'the toolhead', self.m1.prompt_lines())
        self.assertEqual(self.m1.prompt_lines()[-1], 'action:prompt_show')
        v = self.m1.macro_vars('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(int(v['e_was_enabled']), 0)
        self.assertEqual(int(v['changed_hotend_target']), 0)
        self.assertEqual(self.m1.extruder['target'], 215.)
        self.assertFalse(any(MESSAGE in r for r in self.m1.responses))

    def test_printing_press_then_release_does_nothing(self):
        self.m1.print_stats['state'] = 'printing'
        self.press()
        self.m1.timeline.clear()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_RELEASED')
        names = self.m1.names()
        self.assertNotIn('M104', names)
        self.assertNotIn('SET_STEPPER_ENABLE', names)

    def test_paused_and_standby_press_do_the_full_sequence(self):
        for state in ('paused', 'standby'):
            self.m1.timeline.clear()
            self.m1.print_stats['state'] = state
            self.m1.extruder['target'] = 215.
            self.press()
            names = self.m1.names()
            self.assertIn('COLD_RETRACT', names, state)
            self.assertIn('M104', names, state)
            self.assertLess(self.m1.heater_off_index(),
                            names.index('COLD_RETRACT'), state)
            self.assertIn('SET_STEPPER_ENABLE', names, state)
            self.assertFalse(any('Pause the print' in l
                                 for l in self.m1.prompt_lines()), state)

    def test_not_when_unarmed(self):
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertNotIn('SET_CONTACT_GUARD', self.m1.names())


class UnlockWithoutGuard(_M1TestCase):
    def test_nothing_without_contact_guard(self):
        self.m1.print_stats['state'] = 'paused'
        self.arm_unlock()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertNotIn('SET_CONTACT_GUARD', self.m1.names())
        self.assertFalse(any(MESSAGE in r for r in self.m1.responses))


if __name__ == '__main__':
    unittest.main()
