#!/usr/bin/env python3
# KAN-569: on the M1's fault paths the hot end turns off first, before
# any LED command or anything else that could raise, and the pause still
# happens when an LED macro raises. A fan stall turns the hot end off in
# any state, paused included, and RESUME refuses to reheat while the
# hot-end fan check has tripped. Covers TOOLHEAD_CONNECT,
# TOOLHEAD_DISCONNECT and the umbilical unlock press (toolhead.cfg),
# FAN_STOPPAGE_ROUTINE (fan.cfg), BED_REMOVED (bed.cfg), and PAUSE and
# RESUME (print.cfg), including RESUME's two stale-target guards.
#
#   python3 scripts/test_m1_heater_before_leds.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only. Like test_m1_led_modes.py, whose fake
# reactor, printer and config it reuses: the shipped cfg files are read by
# klippy's ConfigFileReader and run through klippy's own GCodeDispatch,
# GCodeMacro and DelayedGcode. PAUSE, RESUME, CANCEL_PRINT and M106 are
# renamed by GCodeMacro's own rename_existing path. Stubbed: the led_effect
# objects, and the motion, heater, fan, prompt and calibration commands,
# any of which a test can make raise. All are recorded in one timeline
# with the LED commands, so the tests can check what ran first. The clock
# is never advanced.
import logging
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_led_modes import (  # noqa: E402
    MACROS, FakeConfig, FakePrinter, FakeReactor, Status, _ErrorLog,
    _parse_leds)
import configfile  # noqa: E402
import gcode  # noqa: E402
from extras import delayed_gcode, gcode_macro  # noqa: E402

INCLUDES = ('led.cfg', 'led_modes.cfg', 'print.cfg', 'fan.cfg',
            'toolhead.cfg', 'bed.cfg')

# Commands the macros under test send that these files don't define, plus
# CUT_FILAMENT and LOAD_FILAMENT (filament.cfg isn't loaded).
STUBS = ('G0', 'G1', 'G90', 'G91', 'G92', 'M84', 'M107', 'M400', 'M117',
         'M118', 'RESPOND', 'FORCE_MOVE', 'SET_STEPPER_ENABLE',
         'TEMPERATURE_WAIT', 'RESET_CALIBRATION', '_BED_MESH_INVALIDATE',
         'SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE', 'TURN_OFF_HEATERS',
         'CLEAR_PAUSE', 'CUT_FILAMENT', 'LOAD_FILAMENT')
# Builtins the macros rename on klippy:connect.
RENAMED = ('PAUSE', 'RESUME', 'CANCEL_PRINT', 'M106')
LED_COMMANDS = ('SET_LED_EFFECT', 'STOP_LED_EFFECTS')


class FailingLedEffect:
    """A led_effect that records SET_LED_EFFECT in the timeline, and can
    be made to raise."""

    def __init__(self, config, effects, timeline):
        self.name = config.get_name().split()[1]
        self.leds = _parse_leds(config.get('leds'))
        self.effects = effects
        self.timeline = timeline
        self.enabled = False
        self.fail = False
        effects.append(self)
        config.get_printer().lookup_object('gcode').register_mux_command(
            'SET_LED_EFFECT', 'EFFECT', self.name, self.cmd_SET_LED_EFFECT)

    def cmd_SET_LED_EFFECT(self, gcmd):
        self.timeline.append('SET_LED_EFFECT ' + self.name)
        if self.fail:
            raise gcmd.error("led_effect %s: failing on purpose" % self.name)
        if gcmd.get_int('REPLACE', 0) >= 1:
            for other in self.effects:
                if other is not self and other.leds & self.leds:
                    other.enabled = False
        self.enabled = gcmd.get_int('STOP', 0) < 1

    def get_status(self, eventtime):
        return {'enabled': self.enabled}


class M1:
    def __init__(self):
        self.reactor = FakeReactor()
        self.printer = printer = FakePrinter(self.reactor)
        self.gcode = gcode.GCodeDispatch(printer)
        printer.add_object('gcode', self.gcode)
        self.responses = []
        self.gcode.register_output_handler(self.responses.append)
        self.macro = gcode_macro.PrinterGCodeMacro(FakeConfig(
            printer, None, 'gcode_macro'))
        printer.add_object('gcode_macro', self.macro)
        printer.add_object('toolhead', Status({
            'position': {'x': 100., 'y': 90., 'z': 40., 'e': 0.},
            'axis_maximum': {'x': 200., 'y': 180., 'z': 180., 'e': 0.},
            'homed_axes': 'xyz', 'estimated_print_time': 0.}))
        self.extruder = {'can_extrude': True, 'target': 215.,
                         'temperature': 215.}
        printer.add_object('extruder', Status(self.extruder))
        self.print_stats = {'state': 'printing'}
        printer.add_object('print_stats', Status(self.print_stats))
        self.pause_resume = {'is_paused': False}
        printer.add_object('pause_resume', Status(self.pause_resume))
        printer.add_object('fan', Status({'speed': 0.5}))
        printer.add_object('stepper_enable', Status(
            {'steppers': {'extruder': True}}))
        printer.add_object('heaters', Status({}))
        printer.add_object('configfile', Status(
            {'settings': {'extruder': {'max_temp': 265.0}}}))
        printer.add_object('idle_timeout', Status({'state': 'Printing'}))
        printer.add_object('filament_switch_sensor filament_sensor_toolhead',
                           Status({'filament_detected': False}))
        # Names of stubbed commands a test wants to raise.
        self.raising = set()

        self.timeline = []
        for name in STUBS + RENAMED:
            self.gcode.register_command(name, self._record)
        self.gcode.register_command('M104', self._m104)
        self.gcode.register_command('SET_HEATER_TEMPERATURE', self._set_heater)
        self.gcode.register_command('STOP_LED_EFFECTS', self._stop_leds)

        top = os.path.join(MACROS, '_test_m1_heater_before_leds.cfg')
        data = ''.join('[include %s]\n' % (name,) for name in INCLUDES)
        fileconfig = configfile.ConfigFileReader() \
            .build_fileconfig_with_includes(data, top)
        self.effects = []
        for section in fileconfig.sections():
            config = FakeConfig(printer, fileconfig, section)
            kind, _, name = section.partition(' ')
            if kind == 'gcode_macro':
                if name.upper() in STUBS:
                    continue
                obj = gcode_macro.GCodeMacro(config)
            elif kind == 'delayed_gcode':
                obj = delayed_gcode.DelayedGcode(config)
            elif kind == 'led_effect':
                obj = FailingLedEffect(config, self.effects, self.timeline)
            else:
                continue
            printer.add_object(section, obj)
        printer.send_event('klippy:connect')
        printer.send_event('klippy:ready')
        self.responses.clear()

    def _record(self, gcmd):
        line = gcmd.get_commandline()
        self.timeline.append(line)
        name = line.split()[0].upper()
        if name in self.raising:
            raise gcmd.error("%s: failing on purpose" % (name,))
        # As pause_resume and virtual_sdcard do: a pause while idle sets
        # is_paused and leaves print_stats alone.
        if name == 'PAUSE_BASE' and not self.pause_resume['is_paused']:
            self.pause_resume['is_paused'] = True
            if self.print_stats['state'] == 'printing':
                self.print_stats['state'] = 'paused'
        elif name == 'RESUME_BASE' and self.pause_resume['is_paused']:
            self.pause_resume['is_paused'] = False
            self.print_stats['state'] = 'printing'
        elif name == 'CANCEL_PRINT_BASE':
            self.print_stats['state'] = 'cancelled'
        elif name == 'CLEAR_PAUSE':
            self.pause_resume['is_paused'] = False

    def _m104(self, gcmd):
        self.timeline.append(gcmd.get_commandline())
        if 'M104' in self.raising:
            raise gcmd.error("M104: failing on purpose")
        self.extruder['target'] = gcmd.get_float('S', 0.)

    def _set_heater(self, gcmd):
        self.timeline.append(gcmd.get_commandline())
        if gcmd.get('HEATER') == 'extruder':
            self.extruder['target'] = gcmd.get_float('TARGET', 0.)

    def _stop_leds(self, gcmd):
        self.timeline.append('STOP_LED_EFFECTS')
        for effect in self.effects:
            effect.enabled = False

    def run(self, script):
        self.gcode.run_script(script)

    def effect(self, name):
        return self.printer.lookup_object('led_effect ' + name)

    def macro_vars(self, name):
        return self.printer.lookup_object('gcode_macro ' + name).variables

    def names(self):
        return [line.split()[0].upper() for line in self.timeline]

    def first_led(self):
        names = self.names()
        led = [i for i, n in enumerate(names) if n in LED_COMMANDS]
        return led[0] if led else None

    def index(self, name):
        names = self.names()
        return names.index(name) if name in names else None

    def heater_off_index(self):
        """Where the hot end's target was set to 0, or None."""
        for i, line in enumerate(self.timeline):
            words = line.upper().split()
            if words[0] == 'M104' and 'S0' in words:
                return i
            if (words[0] == 'SET_HEATER_TEMPERATURE'
                    and 'HEATER=EXTRUDER' in words and 'TARGET=0' in words):
                return i
        return None


class _M1TestCase(unittest.TestCase):
    def setUp(self):
        self.log = _ErrorLog()
        logging.getLogger().addHandler(self.log)
        self.addCleanup(logging.getLogger().removeHandler, self.log)
        self.m1 = M1()

    def tearDown(self):
        self.assertEqual([r for r in self.m1.responses if r.startswith('!!')
                          or 'Unknown command' in r], [])
        self.assertEqual(self.log.records, [])
        self.assertEqual(self.m1.printer.shutdowns, [])

    def assert_heater_off_first(self):
        off = self.m1.heater_off_index()
        self.assertIsNotNone(off, 'hot end never turned off: %s'
                             % (self.m1.timeline,))
        self.assertEqual(self.m1.extruder['target'], 0.)
        led = self.m1.first_led()
        if led is not None:
            self.assertLess(off, led, 'an LED command ran before the hot '
                            'end turned off: %s' % (self.m1.timeline,))

    def run_failing(self, effect, script):
        self.m1.effect(effect).fail = True
        with self.assertRaises(gcode.CommandError):
            self.m1.run(script)
        self.m1.responses.clear()
        self.log.records.clear()

    def arm_unlock(self):
        self.m1.run('SET_GCODE_VARIABLE MACRO=TOOLHEAD_UMBILICAL_UNLOCK_STATE'
                    ' VARIABLE=armed VALUE=1')



class HeaterOffBeforeLeds(_M1TestCase):
    """The hot end turns off before any LED command, and still does when
    the LED effect raises."""

    # TOOLHEAD_DISCONNECT

    def test_disconnect_mid_print(self):
        self.m1.run('TOOLHEAD_DISCONNECT')
        self.assert_heater_off_first()
        self.assertIn('PAUSE_BASE', self.m1.names())
        self.assertIn('SET_LED_EFFECT', self.m1.names())

    def test_disconnect_with_a_raising_led(self):
        self.run_failing('connected', 'TOOLHEAD_DISCONNECT')
        self.assert_heater_off_first()
        self.assertIn('PAUSE_BASE', self.m1.names())

    # FAN_STOPPAGE_ROUTINE

    def test_fan_stall_mid_print(self):
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assert_heater_off_first()
        self.assertIn('PAUSE_BASE', self.m1.names())
        self.assertIn('SET_LED_EFFECT', self.m1.names())
        # PAUSE still records the target the hot end had, for RESUME.
        self.assertEqual(self.m1.macro_vars('PAUSE')['saved_hotend_target'],
                         215.)

    def test_fan_stall_with_a_raising_led(self):
        self.run_failing('info', 'FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assert_heater_off_first()
        self.assertIn('PAUSE_BASE', self.m1.names())
        self.assertTrue(self.m1.pause_resume['is_paused'])

    def test_resume_after_a_fan_stall_reheats(self):
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)
        self.assertIn('RESUME_BASE', self.m1.names())

    def test_fan_stall_when_idle(self):
        self.m1.print_stats['state'] = 'standby'
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assert_heater_off_first()
        self.assertNotIn('PAUSE_BASE', self.m1.names())

    # The umbilical unlock press

    def test_unlock_press(self):
        self.arm_unlock()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assert_heater_off_first()
        self.assertIn('SET_LED_EFFECT', self.m1.names())

    def test_unlock_press_with_a_raising_led(self):
        self.arm_unlock()
        self.run_failing('connected',
                         'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assert_heater_off_first()

    def test_unarmed_unlock_press_still_pulses(self):
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(self.m1.extruder['target'], 215.)
        self.assertTrue(self.m1.effect('connected').enabled)


class FanStallInAnyState(_M1TestCase):
    """#73 review B1: a fan stall turns the hot end off while paused too."""

    def test_fan_stall_while_paused_by_hand(self):
        self.m1.run('PAUSE')
        self.assertEqual(self.m1.extruder['target'], 215.)
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assertEqual(self.m1.extruder['target'], 0.)
        # The manual pause's target is still what RESUME would put back.
        self.assertEqual(self.m1.macro_vars('PAUSE')['saved_hotend_target'],
                         215.)

    def test_fan_stall_after_m600_while_idle(self):
        self.m1.print_stats['state'] = 'standby'
        self.m1.run('PAUSE')
        self.assertTrue(self.m1.pause_resume['is_paused'])
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assertEqual(self.m1.extruder['target'], 0.)


class ResumeRefusedWithADeadFan(_M1TestCase):
    """#73 review B2: RESUME doesn't reheat while the hot-end fan check
    has tripped, and the print stays paused."""

    def stall(self, **counts):
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assertTrue(self.m1.pause_resume['is_paused'])
        for name, value in counts.items():
            self.m1.run('SET_GCODE_VARIABLE MACRO=HOTEND_FAN_CHECK '
                        'VARIABLE=%s VALUE=%d' % (name, value))
        self.m1.timeline.clear()

    def assert_refused(self):
        self.run_failing_script('RESUME')
        self.assertNotIn('M104', self.m1.names())
        self.assertNotIn('RESUME_BASE', self.m1.names())
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.assertTrue(self.m1.pause_resume['is_paused'])

    def run_failing_script(self, script):
        with self.assertRaises(gcode.CommandError):
            self.m1.run(script)
        self.assertTrue(any('RESUME refused' in r
                            for r in self.m1.responses))
        self.m1.responses.clear()
        self.log.records.clear()

    def test_refused_after_a_fan_stall(self):
        self.stall(hotend_stop_count=3)
        self.assert_refused()

    def test_refused_after_a_sensor_fault(self):
        self.stall(sensor_bad_count=3)
        self.assert_refused()

    def test_refused_after_a_stall_while_paused_by_hand(self):
        self.m1.run('PAUSE')
        self.stall(hotend_stop_count=3)
        self.assert_refused()

    def test_allowed_once_the_fault_clears(self):
        self.stall(hotend_stop_count=2)
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)
        self.assertIn('RESUME_BASE', self.m1.names())


class HeaterOffBeforeAnything(_M1TestCase):
    """#73 review B3 and S2: the hot end turns off before any other call
    that could raise, on a disconnect, a reconnect and an unlock press."""

    def test_disconnect_turns_the_heater_off_first(self):
        self.m1.run('TOOLHEAD_DISCONNECT')
        self.assertEqual(self.m1.heater_off_index(), 0, self.m1.timeline)

    def test_disconnect_when_calibration_reset_raises(self):
        self.m1.raising.add('RESET_CALIBRATION')
        self.run_failing_stub('TOOLHEAD_DISCONNECT')
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_disconnect_when_the_fan_command_raises(self):
        self.m1.raising.add('M106.1')
        self.run_failing_stub('TOOLHEAD_DISCONNECT')
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_unlock_press_when_the_retract_raises(self):
        self.arm_unlock()
        self.m1.raising.add('FORCE_MOVE')
        self.run_failing_stub('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(self.m1.extruder['target'], 0.)
        # And RELEASE still knows to put the heat back.
        pressed = self.m1.macro_vars('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(pressed['changed_hotend_target'], 1)

    def test_reconnect_turns_the_heater_off_first(self):
        self.m1.run('TOOLHEAD_CONNECT')
        self.assertEqual(self.m1.heater_off_index(), 0, self.m1.timeline)

    def test_reconnect_with_a_raising_led(self):
        self.run_failing('connected', 'TOOLHEAD_CONNECT')
        self.assertEqual(self.m1.extruder['target'], 0.)

    def run_failing_stub(self, script):
        with self.assertRaises(gcode.CommandError):
            self.m1.run(script)
        self.m1.responses.clear()
        self.log.records.clear()


class PauseHappensDespiteLeds(_M1TestCase):
    """#73 review S1: no LED call stands in front of PAUSE_BASE."""

    def assert_paused_before_leds(self):
        paused = self.m1.index('PAUSE_BASE')
        self.assertIsNotNone(paused, self.m1.timeline)
        led = self.m1.first_led()
        if led is not None:
            self.assertLess(paused, led, self.m1.timeline)

    def test_pause_with_a_raising_led(self):
        self.run_failing('info', 'PAUSE')
        self.assert_paused_before_leds()
        self.assertTrue(self.m1.pause_resume['is_paused'])

    def test_pause_order(self):
        for script in ('PAUSE', 'FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"',
                       'TOOLHEAD_DISCONNECT', 'BED_REMOVED'):
            with self.subTest(script=script):
                self.m1.pause_resume['is_paused'] = False
                self.m1.print_stats['state'] = 'printing'
                self.m1.timeline.clear()
                self.m1.run(script)
                self.assert_paused_before_leds()
                self.assertIn('SET_LED_EFFECT', self.m1.names())

    def test_bed_removed_with_a_raising_led(self):
        self.run_failing('connected', 'BED_REMOVED')
        self.assert_paused_before_leds()


class StaleTargetGuards(_M1TestCase):
    """#73 review S3: the two guards that keep RESUME from putting back a
    stale target (print.cfg PAUSE and _RESUME_RESTORE_HOTEND)."""

    def test_a_second_pause_keeps_the_recorded_target(self):
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.m1.run('PAUSE')
        self.assertEqual(self.m1.macro_vars('PAUSE')['saved_hotend_target'],
                         215.)
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)

    def test_a_stray_resume_after_a_cancel_does_not_heat(self):
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Hotend"')
        self.m1.run('CANCEL_PRINT')
        self.assertFalse(self.m1.pause_resume['is_paused'])
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertNotIn('M104', self.m1.names())
        self.assertEqual(self.m1.extruder['target'], 0.)


class PauseTargetIsChecked(_M1TestCase):
    """#73 review N1: PAUSE's HOTEND_TARGET is a number from 0 to the
    extruder's max_temp (265), or 0."""

    def test_values(self):
        cases = (('200', 200.), ('265', 265.), ('0', 0.), ('999', 0.),
                 ('265.5', 0.), ('-5', 0.), ('abc', 0.), ('nan', 0.),
                 ('inf', 0.))
        for value, want in cases:
            with self.subTest(value=value):
                self.m1.pause_resume['is_paused'] = False
                self.m1.run('PAUSE HOTEND_TARGET=%s' % (value,))
                self.assertEqual(
                    self.m1.macro_vars('PAUSE')['saved_hotend_target'], want)

    def test_without_the_parameter(self):
        self.m1.run('PAUSE')
        self.assertEqual(self.m1.macro_vars('PAUSE')['saved_hotend_target'],
                         215.)


if __name__ == '__main__':
    unittest.main()
