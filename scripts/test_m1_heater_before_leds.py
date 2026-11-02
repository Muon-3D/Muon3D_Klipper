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
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_led_modes import (  # noqa: E402
    MACROS, FakeConfig, FakePrinter, FakeReactor, Status, _ErrorLog,
    _parse_leds)
import configfile  # noqa: E402
import gcode  # noqa: E402
from extras import (delayed_gcode, gcode_macro, hotend_fan_latch,
                    print_end_park, save_variables)  # noqa: E402

INCLUDES = ('led.cfg', 'led_modes.cfg', 'print.cfg', 'fan.cfg',
            'toolhead.cfg', 'bed.cfg', 'filament.cfg', 'calibration.cfg')

# Commands the macros under test send that these files don't define, plus
# CUT_FILAMENT and LOAD_FILAMENT (filament.cfg's, which only move).
STUBS = ('G0', 'G1', 'G90', 'G91', 'G92', 'M84', 'M107', 'M400', 'M117',
         'M118', 'RESPOND', 'FORCE_MOVE', 'SET_STEPPER_ENABLE',
         'TEMPERATURE_WAIT', 'RESET_CALIBRATION', '_BED_MESH_INVALIDATE',
         'SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE', 'TURN_OFF_HEATERS',
         'CLEAR_PAUSE', 'CUT_FILAMENT', 'LOAD_FILAMENT', 'M190', 'M83',
         'G4', 'M300', 'G28', 'SET_KINEMATIC_POSITION',
         'SET_BED_REMOVAL_DETECTOR', 'SAVE_CONFIG')
# Builtins the macros rename on klippy:connect.
RENAMED = ('PAUSE', 'RESUME', 'CANCEL_PRINT', 'M106', 'M109',
           'PID_CALIBRATE', 'PROBE_CALIBRATE')
LED_COMMANDS = ('SET_LED_EFFECT', 'STOP_LED_EFFECTS')

# The prompt a hot-end fan fault puts on the panel and in Fluidd.
FAULT_PROMPT = [
    'action:prompt_begin Hot-end fan fault',
    'action:prompt_text Hot-end fan fault - check the fan, then confirm. '
    'Heating and resume are blocked until you do.',
    'action:prompt_footer_button Fan checked|HOTEND_FAN_FAULT_CLEAR|warning',
    'action:prompt_show']


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
    def __init__(self, variables_file, restore_startup=True, latched=False):
        self.reactor = FakeReactor()
        self.printer = printer = FakePrinter(self.reactor)
        self.gcode = gcode.GCodeDispatch(printer)
        printer.add_object('gcode', self.gcode)
        self.responses = []
        self.gcode.register_output_handler(self.responses.append)
        self.macro = gcode_macro.PrinterGCodeMacro(FakeConfig(
            printer, None, 'gcode_macro'))
        printer.add_object('gcode_macro', self.macro)
        self.saved = save_variables.SaveVariables(FakeConfig(
            printer, None, 'save_variables', {'filename': variables_file}))
        printer.add_object('save_variables', self.saved)
        self.latch_status = {
            'latched': bool(latched or self.saved.allVariables.get(
                'hotend_fan_fault', 0))}
        printer.add_object('hotend_fan_latch', Status(self.latch_status))
        self.gcode.register_command('HOTEND_FAN_LATCH_SET',
                                    self._latch_set)
        self.gcode.register_command('_HOTEND_FAN_LATCH_CONFIRM',
                                    self._latch_confirm)
        # Put a recording/failure-injection shim around Klipper's real
        # SAVE_VARIABLE command.  Successful calls still write and reload the
        # actual file, which lets tests create a second M1 as a restart.
        self.gcode.register_command('SAVE_VARIABLE', None)
        self.gcode.register_command('SAVE_VARIABLE', self._save_variable)
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
        # PRINT_START reads these while it renders, before its first line.
        printer.add_object('configfile', Status({'settings': {
            'extruder': {'max_temp': 265.0},
            'load_cell_probe': {'z_offset': 0.0},
            'bed_mesh': {'mesh_min': [10.0, 7.0],
                         'mesh_max': [190.0, 170.0]}}}))
        printer.add_object('idle_timeout', Status({'state': 'Printing'}))
        printer.add_object('filament_switch_sensor filament_sensor_toolhead',
                           Status({'filament_detected': False}))
        # What HOTEND_FAN_CHECK reads: a healthy, idle heatsink fan.
        self.heatsink = {'temperature': 40.0, 'speed': 0.3, 'rpm': 0.0}
        printer.add_object('temperature_fan toolhead_heatsink',
                           Status(self.heatsink))
        self.fan_air = {'temperature': 35.0}
        printer.add_object('temperature_sensor toolhead_fan_air',
                           Status(self.fan_air))
        printer.add_object('mcu toolhead',
                           Status({'non_critical_disconnected': False}))
        # removedWhilePrinting: bed_removal_detector's own decision that the
        # latest removal came while a file was printing (KAN-272).
        self.bed_detector = {'bedRemoved': False, 'restoreTarget': 60.0,
                             'removedWhilePrinting': True,
                             'printEnding': False}
        printer.add_object('bed_removal_detector', Status(self.bed_detector))
        self.sdcard = {'is_active': False}
        printer.add_object('virtual_sdcard', Status(self.sdcard))
        # Names of stubbed commands a test wants to raise.
        self.raising = set()
        self.raising_lines = set()

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
            elif kind == 'print_end_park':
                obj = print_end_park.load_config(config)
            elif kind == 'delayed_gcode':
                obj = delayed_gcode.DelayedGcode(config)
            elif kind == 'led_effect':
                obj = FailingLedEffect(config, self.effects, self.timeline)
            else:
                continue
            printer.add_object(section, obj)
        printer.send_event('klippy:connect')
        printer.send_event('klippy:ready')
        if restore_startup:
            self.restore_startup_fault()
        self.responses.clear()

    def restore_startup_fault(self):
        # CHECK_ALL_FANS also starts at one second, but this focused harness
        # intentionally has no CPU/MD/PSU fan objects. Leave it parked while
        # firing the two-second persistent-fault restore timer.
        for timer in self.reactor.timers:
            if timer[0] == 1.:
                timer[0] = self.reactor.NEVER
        self.reactor.advance(2.)

    def _latch_set(self, gcmd):
        self.latch_status['latched'] = True

    def _latch_confirm(self, gcmd):
        temp = self.heatsink['temperature']
        if temp >= 51. or temp <= -10.:
            raise gcmd.error('Fan fault not cleared: heatsink sensor %.1fC'
                             % (temp,))
        self.latch_status['latched'] = False

    def _record(self, gcmd):
        line = gcmd.get_commandline()
        self.timeline.append(line)
        name = line.split()[0].upper()
        if (name in self.raising or any(
                line.startswith(prefix) for prefix in self.raising_lines)):
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

    def _save_variable(self, gcmd):
        self.timeline.append(gcmd.get_commandline())
        if 'SAVE_VARIABLE' in self.raising:
            raise gcmd.error("SAVE_VARIABLE: failing on purpose")
        self.saved.cmd_SAVE_VARIABLE(gcmd)

    def _m104(self, gcmd):
        line = gcmd.get_commandline()
        self.timeline.append(line.replace('M104.1', 'M104', 1))
        if 'M104' in self.raising:
            raise gcmd.error("M104: failing on purpose")
        self.extruder['target'] = gcmd.get_float('S', 0.)

    def _set_heater(self, gcmd):
        line = gcmd.get_commandline()
        self.timeline.append(line.replace('SET_HEATER_TEMPERATURE_BASE',
                                          'SET_HEATER_TEMPERATURE', 1))
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

    def prompt_lines(self):
        """The action:prompt lines since the last prompt_begin."""
        lines = [line.split('MSG=', 1)[1].strip('"')
                 for line in self.timeline
                 if line.upper().startswith('RESPOND') and 'action:prompt'
                 in line]
        begins = [i for i, line in enumerate(lines)
                  if line.startswith('action:prompt_begin')]
        return lines[begins[-1]:] if begins else []

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
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.variables_file = os.path.join(self.tempdir.name, 'variables.cfg')
        with open(self.variables_file, 'w') as f:
            f.write('[Variables]\nhotend_fan_fault = 0\n')
        self.m1 = M1(self.variables_file)

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

    def assert_refused(self, script='RESUME'):
        self.run_failing_script(script)
        self.assertNotIn('M104', self.m1.names())
        self.assertNotIn('RESUME_BASE', self.m1.names())
        # Nothing ran before the refusal but the fault prompt: no LEDs
        # (LED_PRINTING), no bed reheat.
        self.assertNotIn('SET_LED_EFFECT', self.m1.names())
        self.assertNotIn('M190', self.m1.names())
        self.assertEqual(self.m1.prompt_lines(), FAULT_PROMPT)
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.assertTrue(self.m1.pause_resume['is_paused'])

    def run_failing_script(self, script):
        with self.assertRaises(gcode.CommandError):
            self.m1.run(script)
        self.assertTrue(any('Resume refused' in r and 'Fan checked' in r
                            and 'HOTEND_FAN_FAULT_CLEAR' in r
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

    def test_bed_resume_button_refused_before_reheating_the_bed(self):
        self.stall(hotend_stop_count=3)
        self.assert_refused('_BED_RESUME_NOW')
        # The guard comes before _BED_RESUME_NOW's own prompt_end: the only
        # one is the fault prompt's.
        ends = [line for line in self.m1.timeline
                if 'action:prompt_end' in line]
        self.assertEqual(len(ends), 1, self.m1.timeline)

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


class _FanCheckTestCase(_M1TestCase):
    """Runs the real HOTEND_FAN_CHECK against a heatsink the test sets."""

    def check(self, times=1):
        for _ in range(times):
            self.m1.run('HOTEND_FAN_CHECK')

    def counts(self):
        v = self.m1.macro_vars('HOTEND_FAN_CHECK')
        return v['hotend_stop_count'], v['sensor_bad_count']

    def fault(self):
        return self.m1.macro_vars('HOTEND_FAN_CHECK')['fault_latched']

    def fan_dead(self):
        # Flat out and losing: past over_temp_c (51 C at a 215 C nozzle)
        # and 21 C over its air.
        self.m1.heatsink.update(temperature=56.0, speed=1.0)

    def sensor_open(self):
        # Open circuit reads about -35 C; the curve then commands 0.
        self.m1.heatsink.update(temperature=-35.0, speed=0.0)

    def trip(self):
        self.check(3)
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.assertTrue(self.m1.pause_resume['is_paused'])

    def resume_refused(self):
        self.m1.timeline.clear()
        with self.assertRaises(gcode.CommandError):
            self.m1.run('RESUME')
        self.assertNotIn('M104', self.m1.names())
        self.m1.responses.clear()
        self.log.records.clear()


class SensorLatchHolds(_FanCheckTestCase):
    """#73 c9adca81 review B: an open heatsink sensor stays latched after
    the nozzle cools, so RESUME stays refused."""

    def setUp(self):
        super().setUp()
        self.sensor_open()
        self.trip()
        self.assertEqual(self.counts()[1], 3)

    def test_stays_latched_after_the_nozzle_cools(self):
        self.m1.extruder['temperature'] = 95.
        self.check(5)
        self.assertEqual(self.counts()[1], 3)
        self.resume_refused()

    def test_a_plausible_reading_resets_the_counter_not_the_fault(self):
        self.m1.extruder['temperature'] = 95.
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.check()
        self.assertEqual(self.counts()[1], 0)
        self.assertEqual(self.fault(), 1)
        self.resume_refused()
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)

    def test_a_preheat_with_the_sensor_still_open_is_cut(self):
        self.m1.extruder['temperature'] = 95.
        self.check()
        with self.assertRaises(gcode.CommandError):
            self.m1.run('M104 S215')
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.m1.responses.clear()
        self.log.records.clear()


class LatchedFaultCutsHeat(_FanCheckTestCase):
    """#73 c9adca81 review S-2: while the fan check is latched and the fault
    still holds, any heat request is cut at the next check. Each entry point
    heats with M104 or SET_HEATER_TEMPERATURE (print.cfg, filament.cfg);
    each test issues the command that entry point sends, except the unlock
    release, which runs for real."""

    def setUp(self):
        super().setUp()
        self.fan_dead()
        self.trip()

    def assert_cut(self):
        self.check()
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_print_start_after_a_cancel(self):
        self.m1.run('CANCEL_PRINT')
        self.assertFalse(self.m1.pause_resume['is_paused'])
        # A new print: PRINT_START's M104 S{PREHEAT_EXTRUDER_TEMP}.
        self.m1.print_stats['state'] = 'printing'
        self.m1.timeline.clear()
        self.refused('M104 S215', 'M104')
        # The direct request is rejected before it can heat or pause.
        self.assertNotIn('PAUSE_BASE', self.m1.names())

    def refused(self, script, what):
        self.m1.timeline.clear()
        with self.assertRaises(gcode.CommandError):
            self.m1.run(script)
        self.assertNotIn('M104', self.m1.names())
        self.assertNotIn('M109.1', self.m1.names())
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.assertTrue(any(what + ' refused' in r
                            for r in self.m1.responses), self.m1.responses)
        self.m1.responses.clear()
        self.log.records.clear()

    def test_unload_from_the_m600_prompt(self):
        # Heats to 250 and waits holding the gcode lock: refused up front.
        self.refused('UNLOAD_FILAMENT TEMP=250', 'Unload')

    def test_load_filament_purge(self):
        self.m1.print_stats['state'] = 'standby'
        self.refused('FILAMENT_INSERT_FULL_PURGE', 'Purge')
        # The guard comes before the purge's own prompt_end: the only one
        # is the fault prompt's.
        ends = [line for line in self.m1.timeline
                if 'action:prompt_end' in line]
        self.assertEqual(len(ends), 1, self.m1.timeline)

    def test_hotend_heating_calibration_refused(self):
        self.refused('CALIBRATE_HEATING_HOTEND', 'Calibration')
        self.assertNotIn('PID_CALIBRATE_BASE', self.m1.names())

    def test_direct_hotend_pid_calibration_refused(self):
        self.refused('PID_CALIBRATE HEATER=extruder TARGET=220',
                     'PID_CALIBRATE')

    def test_hotend_heating_calibration_runs_once_cleared(self):
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.m1.timeline.clear()
        self.m1.run('CALIBRATE_HEATING_HOTEND')
        self.assertIn('PID_CALIBRATE_BASE HEATER=extruder TARGET=220',
                      self.m1.timeline)

    def test_m109_refused(self):
        self.refused('M109 S215', 'M109')
        self.refused('M109 R215', 'M109')

    def test_m109_to_zero_and_after_a_clear(self):
        self.m1.run('M109 S0')
        self.assertIn('M109.1 S0', self.m1.timeline)
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.m1.run('M109 S215')
        self.assertIn('M109.1 S215', self.m1.timeline)

    def test_console_preheat(self):
        for command in ('M104 S215',
                        'SET_HEATER_TEMPERATURE HEATER=extruder TARGET=215'):
            with self.subTest(command=command):
                self.m1.extruder['target'] = 0.
                self.refused(command, command.split()[0])

    def test_direct_heater_zero_always_works(self):
        self.m1.extruder['target'] = 215.
        self.m1.run('M104 S0')
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.m1.extruder['target'] = 215.
        self.m1.run('SET_HEATER_TEMPERATURE HEATER=extruder TARGET=0')
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_unlock_release(self):
        # The press saved 215 and the fault came during the hold. The
        # release itself no longer reheats while the fault stands, and the
        # check would cut it if it did.
        self.m1.run('SET_GCODE_VARIABLE MACRO=TOOLHEAD_UMBILICAL_UNLOCK_BUTTON'
                    '_PRESSED VARIABLE=saved_hotend_target VALUE=215.0')
        self.m1.run('SET_GCODE_VARIABLE MACRO=TOOLHEAD_UMBILICAL_UNLOCK_BUTTON'
                    '_PRESSED VARIABLE=changed_hotend_target VALUE=1')
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_RELEASED')
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.assertTrue(any('NOT reheated' in r for r in self.m1.responses))
        self.m1.responses.clear()
        self.assert_cut()

    def test_one_normal_reading_does_not_clear_the_fault(self):
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.check(5)
        self.assertEqual(self.counts()[0], 0)
        self.assertEqual(self.fault(), 1)
        self.refused('M104 S215', 'M104')

    def test_heat_is_not_cut_once_the_fault_is_cleared(self):
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.check()
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.m1.run('M104 S215')
        self.check()
        self.assertEqual(self.m1.extruder['target'], 215.)


class StickyFanFault(_FanCheckTestCase):
    """Jack, 17 Oct: a hot-end fan trip latches until someone confirms on
    the panel (HOTEND_FAN_FAULT_CLEAR, the Fan checked button)."""

    def test_an_over_temperature_trip_latches_and_asks(self):
        self.fan_dead()
        self.trip()
        self.assertEqual(self.fault(), 1)
        self.assertEqual(self.m1.prompt_lines(), FAULT_PROMPT)

    def test_a_sensor_trip_latches_and_asks(self):
        self.sensor_open()
        self.trip()
        self.assertEqual(self.fault(), 1)
        self.assertEqual(self.m1.prompt_lines(), FAULT_PROMPT)

    def test_clear_refused_while_the_heatsink_is_hot(self):
        self.fan_dead()
        self.trip()
        self.refuse_clear('51C or more')

    def test_clear_refused_while_the_sensor_is_open(self):
        self.sensor_open()
        self.trip()
        self.m1.extruder['temperature'] = 95.
        self.check()
        self.refuse_clear('not plausible')

    def test_clear_resets_the_counters(self):
        # Cleared before any check has seen the heatsink cool: the counter
        # still stands at 3 and RESUME would refuse on it alone.
        self.fan_dead()
        self.trip()
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.assertEqual(self.counts()[0], 3)
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertEqual(self.counts(), (0, 0))
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)

    def test_clear_once_checked(self):
        self.fan_dead()
        self.trip()
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.check()
        self.m1.timeline.clear()
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertEqual(self.fault(), 0)
        self.assertEqual(self.counts(), (0, 0))
        self.assertIn('RESPOND TYPE=command MSG="action:prompt_end"',
                      self.m1.timeline)
        self.m1.timeline.clear()
        self.m1.run('RESUME')
        self.assertIn('M104 S215.0', self.m1.timeline)
        # Still dead: the check trips again once the heat is back.
        self.fan_dead()
        self.trip()
        self.assertEqual(self.fault(), 1)

    def test_clear_with_nothing_latched(self):
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertEqual(self.fault(), 0)
        self.assertTrue(any('No hot-end fan fault' in r
                            for r in self.m1.responses))
        self.m1.responses.clear()

    def test_print_start_refused(self):
        self.fan_dead()
        self.trip()
        self.m1.heatsink.update(temperature=40.0, speed=0.3)
        self.check()
        self.m1.run('CANCEL_PRINT')
        self.m1.print_stats['state'] = 'printing'
        self.m1.timeline.clear()
        with self.assertRaises(gcode.CommandError):
            self.m1.run('PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA')
        self.assertTrue(any('Print refused' in r and 'Fan checked' in r
                            for r in self.m1.responses))
        self.assertNotIn('M104', self.m1.names())
        self.assertEqual(self.m1.prompt_lines(), FAULT_PROMPT)
        self.m1.responses.clear()
        self.log.records.clear()

    def refuse_clear(self, why):
        with self.assertRaises(gcode.CommandError):
            self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertEqual(self.fault(), 1)
        self.assertTrue(any(why in r for r in self.m1.responses),
                        self.m1.responses)
        self.m1.responses.clear()
        self.log.records.clear()


class Heatsink(Status):
    """temperature_fan as the latch reads it: a status, and the print time
    of its last real sample, which stays 0. until the sensor first reports
    (KAN-992)."""

    def __init__(self, status, last_temp_time=100.):
        Status.__init__(self, status)
        self.last_temp_time = last_temp_time


class FakeMCU:
    """Print time runs 100s ahead of the fake reactor, whose clock starts
    at 0., so a Heatsink sampled at 100. is fresh."""

    def estimated_print_time(self, eventtime):
        return eventtime + 100.


class PythonHeaterLatch(unittest.TestCase):
    """The Python gate, unlike macro renames/variables, is unavoidable."""

    class Heater:
        def __init__(self):
            self.target_temp = 0.

        def set_temp(self, value):
            self.target_temp = value

        def alter_target(self, value):
            self.target_temp = value

    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.state_file = os.path.join(self.tempdir.name, 'latch.state')
        self.variables_file = os.path.join(self.tempdir.name, 'variables.cfg')
        with open(self.variables_file, 'w') as f:
            f.write('[Variables]\nmuon_led = \'party\'\nmesh_valid = 1\n')
        self.printer = FakePrinter(FakeReactor())
        self.gcode = gcode.GCodeDispatch(self.printer)
        self.printer.add_object('gcode', self.gcode)
        self.heater = self.Heater()
        self.printer.add_object('extruder', self.heater)
        self.heatsink = Heatsink({'temperature': 40.})
        self.printer.add_object('temperature_fan toolhead_heatsink',
                                self.heatsink)
        self.printer.add_object('mcu', FakeMCU())
        self.latch = hotend_fan_latch.HotendFanLatch(FakeConfig(
            self.printer, None, 'hotend_fan_latch',
            {'state_file': self.state_file}))
        self.printer.add_object('hotend_fan_latch', self.latch)
        self.printer.send_event('klippy:connect')
        self.printer.send_event('klippy:ready')

    def trip(self):
        # D-018: a fresh latch with no state file is clear, so tests of the
        # latched behaviour trip it first, as a fan fault would.
        self.gcode.run_script('HOTEND_FAN_LATCH_SET')
        self.assertTrue(self.latch.latched)

    def start_latch(self):
        """A fresh klippy start reading self.state_file; returns the latch,
        its heater and the lines it sent to the console."""
        printer = FakePrinter(FakeReactor())
        dispatch = gcode.GCodeDispatch(printer)
        printer.add_object('gcode', dispatch)
        heater = self.Heater()
        printer.add_object('extruder', heater)
        printer.add_object('temperature_fan toolhead_heatsink',
                           Heatsink({'temperature': 40.}))
        printer.add_object('mcu', FakeMCU())
        console = []
        dispatch.respond_info = lambda msg, log=True: (
            console.append(msg) if 'fan fault' in msg else None)
        latch = hotend_fan_latch.HotendFanLatch(FakeConfig(
            printer, None, 'hotend_fan_latch',
            {'state_file': self.state_file}))
        printer.send_event('klippy:connect')
        printer.send_event('klippy:ready')
        return latch, heater, console

    def assert_refused(self, setter):
        with self.assertRaises(gcode.CommandError):
            setter(215.)
        self.assertEqual(self.heater.target_temp, 0.)

    def assert_restart_is_latched(self):
        printer = FakePrinter(FakeReactor())
        dispatch = gcode.GCodeDispatch(printer)
        printer.add_object('gcode', dispatch)
        heater = self.Heater()
        printer.add_object('extruder', heater)
        printer.add_object('temperature_fan toolhead_heatsink',
                           Heatsink({'temperature': 40.}))
        printer.add_object('mcu', FakeMCU())
        latch = hotend_fan_latch.HotendFanLatch(FakeConfig(
            printer, None, 'hotend_fan_latch',
            {'state_file': self.state_file}))
        printer.send_event('klippy:connect')
        self.assertTrue(latch.latched)
        with self.assertRaises(gcode.CommandError):
            heater.set_temp(215.)

    def trip_with_failure_then_assert_restart_latched(self, patcher):
        with open(self.state_file, 'w') as f:
            f.write('unlatched\n')
        marker = self.state_file + '.trip'
        if os.path.exists(marker):
            os.unlink(marker)
        with patcher:
            # The latch closes in memory and the call still returns, so the
            # macro can go on to show the prompt; the failure is reported.
            self.gcode.run_script('HOTEND_FAN_LATCH_SET')
        self.assertTrue(self.latch.latched)
        self.assert_restart_is_latched()

    def test_all_renamed_command_bypasses_reach_the_python_gate(self):
        # These setters model M104.1/M109.1, SET_HEATER_TEMPERATURE_BASE,
        # PID_CALIBRATE_BASE and remote/API callers respectively: all of them
        # ultimately reach one of the heater's two target-setting methods.
        self.trip()
        routes = {
            'M104.1': self.heater.set_temp,
            'M109.1': self.heater.set_temp,
            'SET_HEATER_TEMPERATURE_BASE': self.heater.set_temp,
            'PID_CALIBRATE_BASE': self.heater.set_temp,
            'remote set_temp': self.heater.set_temp,
            'internal alter_target': self.heater.alter_target,
        }
        for name, setter in routes.items():
            with self.subTest(route=name):
                self.assert_refused(setter)

    def test_macro_and_saved_variable_edits_cannot_clear_latch(self):
        # SET_GCODE_VARIABLE and SAVE_VARIABLE can mutate these decoys, but
        # the Python-owned state remains closed.
        # Real mutations: rewrite the saved-variables file to say no fault,
        # delete it, and restart.  The latch must come back closed each time.
        self.trip()
        for content in ('[Variables]\nhotend_fan_fault = 0\n', None):
            if content is None:
                os.unlink(self.variables_file)
            else:
                with open(self.variables_file, 'w') as f:
                    f.write(content)
            self.assert_refused(self.heater.set_temp)
            self.assert_restart_is_latched()
        self.gcode.run_script('HOTEND_FAN_LATCH_SET')
        self.assert_refused(self.heater.alter_target)

    def test_zero_and_cooling_targets_always_work(self):
        self.heater.set_temp(0.)
        self.heater.alter_target(0.)
        self.assertEqual(self.heater.target_temp, 0.)

    def test_confirm_is_the_only_open_path_and_checks_sensor(self):
        self.trip()
        self.heatsink.status['temperature'] = 55.
        with self.assertRaises(gcode.CommandError):
            self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assert_refused(self.heater.set_temp)
        self.heatsink.status['temperature'] = 40.
        self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.heater.set_temp(215.)
        self.assertEqual(self.heater.target_temp, 215.)

    def assert_clear_refused(self, message):
        with self.assertRaises(gcode.CommandError) as caught:
            self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assertIn(message, str(caught.exception))
        self.assertTrue(self.latch.latched)
        self.assert_refused(self.heater.set_temp)

    def test_clear_waits_for_the_first_heatsink_sample(self):
        # KAN-992, as found on the bench: Klipper restarted and Fan checked
        # sent about 1s after ready, while temperature_fan still held its
        # 0.0C placeholder.  0.0 is inside the clear window (-10 to 51), so
        # the clear went through on no reading at all.
        self.trip()
        self.heatsink.status['temperature'] = 0.
        self.heatsink.last_temp_time = 0.
        self.assert_clear_refused('not available yet')
        # The first real sample arrives, and the clear works as before.
        self.heatsink.status['temperature'] = 31.
        self.heatsink.last_temp_time = 100.
        self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assertFalse(self.latch.latched)
        self.heater.set_temp(215.)
        self.assertEqual(self.heater.target_temp, 215.)

    def test_a_real_zero_degree_reading_still_clears(self):
        # A cold room is not the placeholder: what matters is the sample.
        self.trip()
        self.heatsink.status['temperature'] = 0.
        self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assertFalse(self.latch.latched)

    def test_clear_refuses_a_stale_sample(self):
        self.trip()
        self.heatsink.last_temp_time = 100. - 6.
        self.assert_clear_refused('last reported 6s ago')
        self.heatsink.last_temp_time = 100. - 4.
        self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assertFalse(self.latch.latched)

    def test_clear_refuses_a_status_without_a_temperature(self):
        # This used to read as 0.0 through .get('temperature', 0.).
        self.trip()
        del self.heatsink.status['temperature']
        self.assert_clear_refused('not available')

    def test_clear_refuses_a_sensor_that_cannot_say_when_it_read(self):
        self.trip()
        self.printer.add_object('temperature_fan toolhead_heatsink',
                                Status({'temperature': 40.}))
        self.assert_clear_refused('does not report when it last read')

    def test_restart_then_immediate_clear_is_refused(self):
        # The bench sequence end to end: a corrupt state file latches the
        # fresh start, and the heatsink has not sampled yet.
        with open(self.state_file, 'w') as f:
            f.write('garbage')
        latch, heater, _ = self.start_latch()
        self.assertTrue(latch.latched)
        heatsink = latch.printer.lookup_object(
            'temperature_fan toolhead_heatsink')
        heatsink.status['temperature'] = 0.
        heatsink.last_temp_time = 0.
        dispatch = latch.printer.lookup_object('gcode')
        with self.assertRaises(gcode.CommandError):
            dispatch.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        self.assertTrue(latch.latched)
        with self.assertRaises(gcode.CommandError):
            heater.set_temp(215.)

    def test_temperature_fan_records_when_it_sampled(self):
        # The field the latch reads.  Upstream declares it and never sets it.
        # Imported here: temperature_fan needs numpy, which CI installs
        # (klippy-requirements.txt) and an offline run of the rest of this
        # file does not.
        try:
            from extras import temperature_fan
        except ImportError as e:
            self.skipTest('temperature_fan needs %s' % (e.name,))
        tf = temperature_fan.TemperatureFan.__new__(
            temperature_fan.TemperatureFan)
        tf.last_temp = tf.last_temp_time = 0.
        tf.control = mock.Mock()
        tf.temperature_callback(12.5, 38.)
        self.assertEqual((tf.last_temp, tf.last_temp_time), (38., 12.5))
        tf.control.temperature_callback.assert_called_once_with(12.5, 38.)

    def test_public_python_confirm_command_is_not_registered(self):
        self.assertNotIn('HOTEND_FAN_LATCH_CONFIRM',
                         self.gcode.ready_gcode_handlers)

    def test_marker_survives_marker_write_failure_and_restart(self):
        self.trip_with_failure_then_assert_restart_latched(
            mock.patch.object(os, 'write', side_effect=OSError('write')))

    def test_marker_survives_state_replace_failure_and_restart(self):
        self.trip_with_failure_then_assert_restart_latched(
            mock.patch.object(os, 'replace', side_effect=OSError('replace')))

    def test_marker_survives_fsync_failure_and_restart(self):
        self.trip_with_failure_then_assert_restart_latched(
            mock.patch.object(os, 'fsync', side_effect=OSError('fsync')))

    def test_corrupt_latch_fails_closed_without_touching_saved_variables(self):
        with open(self.state_file, 'w') as f:
            f.write('not a latch state')
        with open(self.variables_file) as f:
            before = f.read()
        self.assertTrue(self.latch._read_state())
        with open(self.variables_file) as f:
            self.assertEqual(f.read(), before)


    # D-018 (Jack, 20 Oct 2026): absent file = clear, corrupt file = latched.
    def test_d018_absent_file_is_clear_and_heating_is_allowed(self):
        self.assertFalse(os.path.exists(self.state_file))
        latch, heater, console = self.start_latch()
        self.assertFalse(latch.latched)
        heater.set_temp(215.)
        self.assertEqual(heater.target_temp, 215.)
        self.assertEqual(console, [])

    def test_exists_only_treats_enoent_as_absent(self):
        import errno
        self.assertFalse(self.latch._exists(self.state_file))
        for code in (errno.EACCES, errno.ENOTDIR, errno.EIO):
            with self.subTest(errno=code):
                with mock.patch.object(os, 'lstat',
                                       side_effect=OSError(code, 'x')):
                    with self.assertRaises(OSError):
                        self.latch._exists(self.state_file)
                    # ...and a restart on that error is latched, not clear.
                    latch, heater, console = self.start_latch()
                    self.assertTrue(latch.latched)

    def test_marker_is_fsynced(self):
        opened, synced = [], []
        real_open, real_fsync = os.open, os.fsync

        def spy_open(path, *a, **k):
            fd = real_open(path, *a, **k)
            if str(path).endswith('.trip'):
                opened.append(fd)
            return fd

        def spy_fsync(fd):
            synced.append(fd)
            return real_fsync(fd)
        with mock.patch.object(os, 'open', spy_open), \
                mock.patch.object(os, 'fsync', spy_fsync):
            self.latch._create_trip_marker()
        self.assertEqual(len(opened), 1)
        self.assertIn(opened[0], synced)

    def test_missing_state_directory_is_created_and_clear(self):
        self.state_file = os.path.join(self.tempdir.name, 'cal', 'latch.state')
        latch, heater, console = self.start_latch()
        self.assertTrue(os.path.isdir(os.path.dirname(self.state_file)))
        self.assertFalse(latch.latched)
        heater.set_temp(215.)
        # A trip now persists, so a restart is latched.
        latch.cmd_latch(mock.Mock())
        latch2, heater2, console2 = self.start_latch()
        self.assertTrue(latch2.latched)

    def test_uncreatable_state_directory_is_latched_with_a_message(self):
        blocker = os.path.join(self.tempdir.name, 'afile')
        with open(blocker, 'w') as f:
            f.write('x')
        self.state_file = os.path.join(blocker, 'latch.state')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        self.assertEqual(len(console), 1)
        self.assertIn('cannot be created', console[0])

    def test_persistence_failure_still_latches_and_warns(self):
        cmd = mock.Mock()
        with mock.patch.object(os, 'replace', side_effect=OSError('replace')), \
                mock.patch.object(os, 'write', side_effect=OSError('write')):
            self.latch.cmd_latch(cmd)
        self.assertTrue(self.latch.latched)
        self.assert_refused(self.heater.set_temp)
        self.assertIn('could NOT be saved', cmd.respond_info.call_args[0][0])

    def test_d018_absent_file_but_trip_marker_is_latched(self):
        with open(self.state_file + '.trip', 'w') as f:
            f.write('latched\n')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        with self.assertRaises(gcode.CommandError):
            heater.set_temp(215.)

    def test_d018_file_without_the_fault_key_is_corrupt(self):
        # Not a latch file at all (e.g. a variables.cfg pointed at by
        # mistake): present but unparseable, so latched.
        with open(self.state_file, 'w') as f:
            f.write('[Variables]\nmesh_valid = 1\n')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)

    def test_d018_corrupt_files_are_latched_and_say_why(self):
        for name, content in (('garbage', 'not a latch state'),
                              ('empty', ''), ('binary', '\x00\x01\x02')):
            with self.subTest(content=name):
                with open(self.state_file, 'w') as f:
                    f.write(content)
                latch, heater, console = self.start_latch()
                self.assertTrue(latch.latched)
                with self.assertRaises(gcode.CommandError):
                    heater.set_temp(215.)
                self.assertEqual(heater.target_temp, 0.)
                self.assertEqual(len(console), 1)
                self.assertIn('corrupt or unreadable', console[0])
                self.assertIn('Fan checked', console[0])

    def test_d018_unreadable_file_is_latched_not_absent(self):
        with open(self.state_file, 'w') as f:
            f.write('unlatched\n')
        real_open = open

        def deny(path, *args, **kwargs):
            if path == self.state_file:
                raise PermissionError(13, 'Permission denied')
            return real_open(path, *args, **kwargs)
        with mock.patch('builtins.open', side_effect=deny):
            latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        self.assertEqual(len(console), 1)

    def test_d018_directory_or_dangling_symlink_is_latched(self):
        os.mkdir(self.state_file)
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        os.rmdir(self.state_file)
        try:
            os.symlink(os.path.join(self.tempdir.name, 'nowhere'),
                       self.state_file)
        except (OSError, NotImplementedError):
            self.skipTest('symlinks unavailable here')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)

    def test_d018_valid_file_fault_1_is_latched(self):
        with open(self.state_file, 'w') as f:
            f.write('latched\n')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        with self.assertRaises(gcode.CommandError):
            heater.set_temp(215.)
        self.assertEqual(console, [])

    def test_d018_valid_file_fault_0_is_clear(self):
        with open(self.state_file, 'w') as f:
            f.write('unlatched\n')
        latch, heater, console = self.start_latch()
        self.assertFalse(latch.latched)
        heater.set_temp(215.)
        self.assertEqual(heater.target_temp, 215.)

    def test_d018_trip_then_restart_is_latched_until_cleared(self):
        self.gcode.run_script('HOTEND_FAN_LATCH_SET')
        latch, heater, console = self.start_latch()
        self.assertTrue(latch.latched)
        self.gcode.run_script('_HOTEND_FAN_LATCH_CONFIRM')
        latch, heater, console = self.start_latch()
        self.assertFalse(latch.latched)


class ClearAfterRestart(_FanCheckTestCase):
    """The macro variable fault_latched is 0 after every restart, the Python
    latch is not: Fan checked must still reach _HOTEND_FAN_LATCH_CONFIRM."""

    def restart_latched(self):
        self.m1 = M1(self.variables_file, latched=True)
        self.assertEqual(self.m1.macro_vars('HOTEND_FAN_CHECK')
                         ['fault_latched'], 0)

    def test_fan_checked_clears_after_a_restart(self):
        self.restart_latched()
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertFalse(self.m1.latch_status['latched'])
        self.assertFalse([r for r in self.m1.responses
                          if 'No hot-end fan fault' in r])

    def test_fan_checked_still_refuses_while_the_heatsink_is_hot(self):
        self.restart_latched()
        self.m1.heatsink['temperature'] = 55.
        self.m1.latch_status['latched'] = True
        with self.assertRaises(gcode.CommandError):
            self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertTrue(self.m1.latch_status['latched'])
        self.assertTrue(any('51C or more' in r for r in self.m1.responses))
        self.m1.responses.clear()
        self.log.records.clear()

    def test_nothing_latched_anywhere_says_so(self):
        self.m1.run('HOTEND_FAN_FAULT_CLEAR')
        self.assertTrue([r for r in self.m1.responses
                         if 'No hot-end fan fault' in r])


class UnlockReleaseRestoresOnce(_FanCheckTestCase):
    """Jack, 17 Oct: the unlock release turns the heater back on, but only
    once, only while no fan fault stands, and not after a stall, a cancel
    or a pulled toolhead during the hold."""

    def setUp(self):
        super().setUp()
        self.arm_unlock()

    def press(self):
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(self.m1.extruder['target'], 0.)

    def release(self):
        self.m1.timeline.clear()
        self.m1.run('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_RELEASED')
        return [line for line in self.m1.timeline
                if line.split()[0] in ('M104', 'SET_STEPPER_ENABLE')]

    def test_a_normal_press_and_release(self):
        self.press()
        self.assertEqual(self.release(), [
            'SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1', 'M104 S215.0'])
        self.assertEqual(self.m1.extruder['target'], 215.)

    def test_p3_a_stall_during_the_hold(self):
        self.press()
        self.fan_dead()
        self.check(3)
        self.assertEqual(self.fault(), 1)
        self.assertNotIn('M104 S215.0', self.release())
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_p4_a_cancel_during_the_hold(self):
        self.press()
        self.m1.run('CANCEL_PRINT')
        self.assertEqual(self.release(), [])
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_no_reheat_while_the_fault_is_latched(self):
        # Counters below 3, the sticky fault alone standing.
        self.press()
        self.m1.run('HOTEND_FAN_LATCH_SET')
        self.m1.run('SET_GCODE_VARIABLE MACRO=HOTEND_FAN_CHECK '
                    'VARIABLE=fault_latched VALUE=0')
        self.assertNotIn('M104 S215.0', self.release())
        self.assertEqual(self.m1.extruder['target'], 0.)
        self.m1.responses.clear()

    def test_a_disconnect_during_the_hold(self):
        self.press()
        self.m1.run('TOOLHEAD_DISCONNECT')
        self.assertEqual(self.release(), [])

    def test_a_fan_stop_during_the_hold(self):
        # The routine alone, with no latch: it forgets the press.
        self.press()
        self.m1.run('FAN_STOPPAGE_ROUTINE FAN_NAME="Motor Drivers"')
        self.assertEqual(self.release(), [])

    def test_p6_a_second_release(self):
        self.press()
        self.release()
        self.m1.run('M104 S0')
        self.assertEqual(self.release(), [])
        self.assertEqual(self.m1.extruder['target'], 0.)

    def test_p5_toolhead_pulled_while_held(self):
        # klippy's own button debouncing (extras/buttons.py), driven the
        # way the toolhead MCU does: press, the toolhead is pulled while the
        # button is held, it is reattached, then pressed and released.
        events = []
        presses = ButtonSim(events)
        presses.report(0)
        self.assertEqual(events, ['PRESS'])
        self.press()
        presses.disconnect()
        self.m1.run('TOOLHEAD_DISCONNECT')
        presses.reconnect()
        self.m1.run('TOOLHEAD_CONNECT')
        self.arm_unlock()
        self.m1.run('CANCEL_PRINT')
        self.m1.print_stats['state'] = 'standby'
        presses.report(0)
        presses.report(1)
        # The press after reattach is swallowed; only its release fires.
        self.assertEqual(events, ['PRESS', 'RELEASE'])
        self.assertEqual(self.release(), [])
        self.assertEqual(self.m1.extruder['target'], 0.)


class ButtonSim:
    """klippy's MCU_buttons and DebounceButton for one inverted pin, with
    the non-critical toolhead disconnect and reconnect events."""

    def __init__(self, events):
        from extras import buttons

        class Reactor:
            def register_callback(self, callback, waketime):
                callback(waketime)

            def register_async_callback(self, callback):
                callback(0.)

        class Printer:
            def __init__(self):
                self.handlers = {}
                self.reactor = Reactor()

            def get_reactor(self):
                return self.reactor

            def register_event_handler(self, name, callback):
                self.handlers.setdefault(name, []).append(callback)

            def send(self, name):
                for callback in self.handlers.get(name, []):
                    callback()

        class Mcu:
            is_non_critical = True

            def register_config_callback(self, callback):
                pass

            def get_non_critical_disconnect_event_name(self):
                return 'toolhead:disconnect'

            def get_non_critical_reconnect_event_name(self):
                return 'toolhead:reconnect'

        class Config:
            def __init__(self, printer):
                self.printer = printer

            def get_printer(self):
                return self.printer

            def getfloat(self, name, default, minval=None):
                return 0.05

        class Ack:
            def send(self, args):
                pass

        self.printer = Printer()
        self.mcu_buttons = buttons.MCU_buttons(self.printer, Mcu())
        debounce = buttons.DebounceButton(
            Config(self.printer),
            lambda t, state: events.append('PRESS' if state else 'RELEASE'))
        self.mcu_buttons.setup_buttons(
            [{'pin': 'gpio5', 'invert': 1, 'pullup': 0}],
            debounce.button_handler)
        self.mcu_buttons.ack_cmd = Ack()
        self.mcu_buttons.oid = 0
        self.time = 0.

    def report(self, raw):
        self.time += 1.
        self.mcu_buttons.handle_buttons_state({
            'ack_count': self.mcu_buttons.ack_count & 0xff,
            'state': bytes([raw]), '#receive_time': self.time})

    def disconnect(self):
        self.printer.send('toolhead:disconnect')

    def reconnect(self):
        self.printer.send('toolhead:reconnect')



class CancelParkTests(_M1TestCase):
    # A refused XY park used to abort PRINT_END before state/LED/unlock
    # cleanup. Exercise the actual CANCEL_PRINT -> PRINT_END macro chain.
    def test_cancel_continues_after_park_refusal(self):
        m = self.m1
        unlock = m.macro_vars('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        unlock.update(e_was_enabled=1, changed_hotend_target=1)
        m.raising_lines.add('G0 X195 Y10')
        m.run('CANCEL_PRINT')
        names = m.names()
        self.assertEqual(m.print_stats['state'], 'cancelled')
        self.assertFalse(m.pause_resume['is_paused'])
        self.assertIn('M84', names)
        self.assertIn('RESTORE_GCODE_STATE NAME=STATE_PRINT_END', m.timeline)
        self.assertIsNotNone(m.first_led())
        self.assertLess(names.index('TURN_OFF_HEATERS'), names.index('G0'))
        self.assertNotIn('CUT_FILAMENT', names)
        self.assertEqual(names.count('G0'), 2)  # lift, then refused XY
        self.assertEqual([r for r in m.responses if r.startswith('!!')],
                         ['!! G0: failing on purpose'])
        m.responses[:] = [r for r in m.responses if not r.startswith('!!')]
        self.assertTrue(any('parking skipped' in r for r in m.responses),
                        m.responses)
        unlock = m.macro_vars('TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        self.assertEqual(unlock['e_was_enabled'], 0)
        self.assertEqual(unlock['changed_hotend_target'], 0)



class BedRemovedOutsideThePrintPhase(_M1TestCase):
    """KAN-272: BED_REMOVED pauses only on bed_removal_detector's word that
    a file was printing, whatever idle_timeout and print_stats read."""

    def removed(self, while_printing):
        m = self.m1
        m.bed_detector.update(bedRemoved=True,
                              removedWhilePrinting=while_printing)
        m.timeline.clear()
        m.run('BED_REMOVED')
        return m.names()

    def test_idle_with_startup_moves_does_not_pause(self):
        # walnut, 2026-10-01: idle_timeout "Printing", nothing loaded.
        m = self.m1
        m.print_stats['state'] = 'standby'
        names = self.removed(False)
        self.assertNotIn('PAUSE_BASE', names)
        self.assertFalse(m.pause_resume['is_paused'])
        self.assertEqual(m.prompt_lines(), [])
        # The non-pause actions still happen.
        self.assertIn('_BED_MESH_INVALIDATE', names)
        self.assertIn('RESPOND TYPE=command MSG="action:prompt_end"',
                      m.timeline)
        self.assertIn('SET_LED_EFFECT', names)

    def test_print_end_tail_does_not_pause(self):
        # print_stats still "printing" and the file still being read:
        # PRINT_END is a line of it.
        m = self.m1
        m.print_stats['state'] = 'printing'
        m.sdcard['is_active'] = True
        m.bed_detector['printEnding'] = True
        names = self.removed(False)
        self.assertNotIn('PAUSE_BASE', names)
        self.assertEqual(m.print_stats['state'], 'printing')
        self.assertEqual(m.prompt_lines(), [])

    def test_mid_print_pauses_and_says_so(self):
        # KAN-79, unchanged. print_stats may already read "paused" by the
        # time BED_REMOVED renders, because the detector paused the file.
        m = self.m1
        m.print_stats['state'] = 'paused'
        names = self.removed(True)
        self.assertIn('PAUSE_BASE', names)
        self.assertTrue(m.pause_resume['is_paused'])
        self.assertEqual(m.prompt_lines()[0],
                         'action:prompt_begin Bed Detached')

    def test_resumed_before_this_rendered_pauses(self):
        # Review of #KAN-272: plate pulled during RESUME's hot-end reheat
        # wait. The print read "paused", so the detector sent nothing, and
        # RESUME then restarted the file before BED_REMOVED got the mutex.
        m = self.m1
        m.print_stats['state'] = 'printing'
        m.sdcard['is_active'] = True
        names = self.removed(False)
        self.assertIn('PAUSE_BASE', names)
        self.assertTrue(m.pause_resume['is_paused'])
        self.assertEqual(m.prompt_lines()[0],
                         'action:prompt_begin Bed Detached')

    def test_cancelled_before_this_rendered_does_not_pause(self):
        # The detector paused a printing file, but a CANCEL_PRINT already
        # queued for the mutex ran first and ended the print.
        m = self.m1
        m.print_stats['state'] = 'cancelled'
        names = self.removed(True)
        self.assertNotIn('PAUSE_BASE', names)
        self.assertFalse(m.pause_resume['is_paused'])
        self.assertEqual(m.prompt_lines(), [])

    def test_print_end_marks_the_tail_after_the_heaters(self):
        m = self.m1
        m.timeline.clear()
        m.run('PRINT_END')
        names = m.names()
        mark = m.timeline.index('SET_BED_REMOVAL_DETECTOR PRINT_ENDING=1')
        self.assertLess(names.index('TURN_OFF_HEATERS'), mark)
        self.assertLess(names.index('M107'), mark)
        # Before the park, which is the window the ticket is about.
        self.assertLess(mark, names.index('G0'))


if __name__ == '__main__':
    unittest.main()
