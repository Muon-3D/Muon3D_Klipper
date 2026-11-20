#!/usr/bin/env python3
# Behaviour tests for the M1 LED state macros and the user LED mode
# (KAN-480, KAN-568): core/M1/macros/led.cfg and led_modes.cfg, and the
# LED calls in print.cfg (PRINT_END, CANCEL_PRINT, PAUSE, RESUME,
# _ON_PRINT_ERROR) and filament.cfg (UNLOAD_FILAMENT, the load prompts).
#
#   python3 scripts/test_m1_led_modes.py      (or: pytest <this file>)
#
# Offline, no hardware, Linux only (klippy imports pty and fcntl). The
# shipped cfg files are read by klippy's own ConfigFileReader, and the
# macros run through klippy's own GCodeDispatch, GCodeMacro, DelayedGcode
# and SaveVariables. That means Klipper's single-brace Jinja environment,
# its render-then-run order, macro recursion limits and literal_eval'd
# variables are the real ones, not a copy. The reactor is a fake whose
# clock the test moves, so delayed_gcode timers fire on demand. Stubbed:
# the led_effect objects (enabled flag and REPLACE overlap, nothing
# rendered to a pixel), the motion, heater and prompt commands the macros
# emit, and the builtins CANCEL_PRINT, PAUSE and RESUME, which the macros
# rename to *_BASE through GCodeMacro's own rename_existing path.
import contextlib
import logging
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# MUON_M1_CONFIG_DIR points every test_m1_*.py at another config tree
# (core/M1-B0, relative to the repo root or absolute). Default: core/M1.
CONFIG_DIR = os.path.join(ROOT, os.environ.get('MUON_M1_CONFIG_DIR')
                          or os.path.join('core', 'M1'))
MACROS = os.path.join(CONFIG_DIR, 'macros')
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

import configfile  # noqa: E402
import gcode  # noqa: E402
from extras import (delayed_gcode, gcode_macro, print_end_park,
                    save_variables)  # noqa: E402

# The files under test, as core/M1/macros/macros.cfg includes them.
INCLUDES = ('led.cfg', 'led_modes.cfg', 'print.cfg', 'filament.cfg')

# Commands the macros send that the files above don't define, plus
# CUT_FILAMENT, which they do but which only moves, and
# _SAVE_CONFIG_IF_PENDING (calibration.cfg's, which CANCEL_PRINT runs after
# PRINT_END since KAN-382). Recorded, otherwise ignored. Anything else
# unknown is an error.
STUB_COMMANDS = ('SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE', 'M400', 'G92',
                 'G1', 'G0', 'G90', 'G91', 'M107', 'M84', 'TURN_OFF_HEATERS',
                 'CUT_FILAMENT', '_STOW_IF_IDLE', '_STOW_AFTER_PRINT_ERROR',
                 'M104', 'M83', 'G4', 'M300',
                 'M117',
                 'TEMPERATURE_WAIT', 'RESPOND', 'CLEAR_PAUSE',
                 '_SAVE_CONFIG_IF_PENDING', 'M106', 'M140', 'BED_MESH_CLEAR',
                 'MACHINE_WAIT_READY', 'MACHINE_REQUIRE',
                 'MACHINE_HEAT_WAIT', 'MACHINE_PAUSED',
                 'MACHINE_PAUSE_REASON', 'MACHINE_RESUMED',
                 'MACHINE_PLATE_CLEAR')

# What PRINT_END must always reach: heaters, part fan, motors off.
SHUTDOWN = ('TURN_OFF_HEATERS', 'M107', 'M84')

# The M1 neopixel chains are 40 LEDs each. A bare chain name in a
# led_effect's leds: covers all of them.
CHAIN_LENGTH = 40

PROGRESS = {'progress_left', 'progress_left_mirror', 'progress_right',
            'progress_right_mirror'}
SETUP = {'setup_left', 'setup_left_mirror', 'setup_right',
         'setup_right_mirror'}

BOTH_MODE = frozenset({'muon_mode_left', 'muon_mode_right'})


def idle_set(idle):
    """A test's idle look: 'ready', or BOTH_MODE for a user mode."""
    return set(idle) if isinstance(idle, (set, frozenset)) else {idle}


_MISSING = object()


class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 0.
        self.timers = []

    def monotonic(self):
        return self.now

    def mutex(self):
        return contextlib.nullcontext()

    def assert_no_pause(self):
        return contextlib.nullcontext()

    def register_timer(self, callback, waketime=NEVER):
        timer = [waketime, callback]
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        timer[0] = waketime

    def advance(self, seconds):
        end = self.now + seconds
        while True:
            due = [t for t in self.timers if t[0] <= end]
            if not due:
                break
            timer = min(due, key=lambda t: t[0])
            self.now = max(self.now, timer[0])
            timer[0] = timer[1](self.now)
        self.now = end


class Status:
    def __init__(self, status):
        self.status = status

    def get_status(self, eventtime):
        return self.status


class FakePrinter:
    config_error = configfile.error
    command_error = gcode.CommandError

    def __init__(self, reactor):
        self.reactor = reactor
        self.objects = {}
        self.handlers = {}
        self.shutdowns = []

    def get_start_args(self):
        return {}

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *args):
        return [cb(*args) for cb in self.handlers.get(event, [])]

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=_MISSING):
        if name in self.objects:
            return self.objects[name]
        if default is _MISSING:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default

    def lookup_objects(self, module=None):
        return list(self.objects.items())

    def load_object(self, config, section):
        return self.objects[section]

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

    def request_exit(self, result):
        pass


class FakeConfig:
    error = configfile.error

    def __init__(self, printer, fileconfig, section, overrides=None):
        self.printer = printer
        self.fileconfig = fileconfig
        self.section = section
        self.overrides = overrides or {}

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.section

    def get(self, option, default=_MISSING):
        if option in self.overrides:
            return self.overrides[option]
        if (self.fileconfig is not None
                and self.fileconfig.has_option(self.section, option)):
            return self.fileconfig.get(self.section, option)
        if default is _MISSING:
            raise self.error("Option '%s' in section '%s' must be specified"
                             % (option, self.section))
        return default

    def getint(self, option, default=_MISSING, minval=None, maxval=None):
        value = self.get(option, default)
        return value if value is None else int(value)

    def getfloat(self, option, default=_MISSING, minval=None, maxval=None,
                 above=None, below=None):
        value = self.get(option, default)
        return value if value is None else float(value)

    def getboolean(self, option, default=_MISSING):
        value = self.get(option, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'yes', 'true', 'on')

    def get_prefix_options(self, prefix):
        return [o for o in self.fileconfig.options(self.section)
                if o.startswith(prefix)]


def _parse_leds(text):
    leds = set()
    for line in text.split('\n'):
        line = line.strip()
        if not line:
            continue
        chain, _, rng = line.partition(' ')
        rng = rng.strip().strip('()')
        if not rng:
            indices = range(1, CHAIN_LENGTH + 1)
        else:
            first, _, last = rng.partition('-')
            lo, hi = sorted((int(first), int(last or first)))
            indices = range(lo, hi + 1)
        leds.update((chain, i) for i in indices)
    return leds


class FakeLedEffect:
    """The parts of klippy/extras/led_effect.py the macros can observe:
    the enabled flag, REPLACE turning off overlapping effects, and the
    layers template rendered with SET_LED_EFFECT's params."""

    def __init__(self, config, effects, macro, timeline):
        self.name = config.get_name().split()[1]
        self.effects = effects
        self.timeline = timeline
        self.auto_start = config.getboolean('autostart', False)
        self.recalculate = config.get('recalculate', False)
        self.leds = _parse_leds(config.get('leds'))
        self.template = macro.load_template(config, 'layers')
        self.enabled = False
        self.layers = None
        self.params = None
        # Set by a test to make SET_LED_EFFECT raise, as a broken effect
        # or template would.
        self.fail = False
        effects.append(self)
        printer = config.get_printer()
        printer.register_event_handler('klippy:ready', self._handle_ready)
        printer.lookup_object('gcode').register_mux_command(
            'SET_LED_EFFECT', 'EFFECT', self.name, self.cmd_SET_LED_EFFECT)

    def _handle_ready(self):
        # led_effect.py _generateLayers: empty params at klippy:ready.
        context = self.template.create_template_context()
        context.update({'params': {}, 'rawparams': ''})
        self.layers = self.template.render(context)
        self.enabled = self.auto_start

    def cmd_SET_LED_EFFECT(self, gcmd):
        self.timeline.append('SET_LED_EFFECT ' + self.name)
        if self.fail:
            raise gcmd.error("led_effect %s: failing on purpose" % self.name)
        if gcmd.get_int('STOP', 0) >= 1:
            self.enabled = False
            return
        if self.recalculate:
            context = self.template.create_template_context()
            context['params'] = gcmd.get_command_parameters()
            context['rawparams'] = gcmd.get_raw_command_parameters()
            self.layers = self.template.render(context)
            self.params = dict(gcmd.get_command_parameters())
        if gcmd.get_int('REPLACE', 0) >= 1:
            for other in self.effects:
                if other is not self and other.leds & self.leds:
                    other.enabled = False
        self.enabled = True

    def get_status(self, eventtime):
        return {'enabled': self.enabled}


class M1Leds:
    """The LED macros loaded into a klippy that has reached ready."""

    def __init__(self, variables_file, includes=INCLUDES):
        self.reactor = FakeReactor()
        self.printer = printer = FakePrinter(self.reactor)
        self.gcode = gcode.GCodeDispatch(printer)
        printer.add_object('gcode', self.gcode)
        self.responses = []
        self.gcode.register_output_handler(self.responses.append)
        self.macro = gcode_macro.PrinterGCodeMacro(FakeConfig(
            printer, None, 'gcode_macro'))
        printer.add_object('gcode_macro', self.macro)
        printer.add_object('save_variables', save_variables.SaveVariables(
            FakeConfig(printer, None, 'save_variables',
                       {'filename': variables_file})))
        printer.add_object('toolhead', Status({
            'position': {'x': 100., 'y': 90., 'z': 40., 'e': 0.},
            'axis_maximum': {'x': 200., 'y': 180., 'z': 180., 'e': 0.},
            'axis_minimum': {'x': 0., 'y': -10., 'z': -4., 'e': 0.},
            'homed_axes': 'xyz', 'estimated_print_time': 0.}))
        printer.add_object('extruder', Status({
            'can_extrude': True, 'target': 0., 'temperature': 215.}))
        printer.add_object('heaters', Status({}))
        self.print_stats = {'state': 'standby'}
        printer.add_object('print_stats', Status(self.print_stats))
        self.pause_resume = {'is_paused': False}
        printer.add_object('pause_resume', Status(self.pause_resume))
        # B1: RESUME reads the Z column; locked.
        printer.add_object('z_power', Status({
            'z': 'locked', 'state': 'ready', 'reason': '',
            'xy_allowed': True, 'xy_home_allowed': True}))
        printer.add_object('m1_machine', Status({
            'job_targets': {'bed': 0.0, 'hotend': 0.0}, 'saved_fan': 0.0}))
        printer.add_object('heater_bed', Status({'target': 0.0}))

        self.stubbed = []
        for name in STUB_COMMANDS:
            self.gcode.register_command(name, self._stub)
        self.gcode.register_command('STOP_LED_EFFECTS',
                                    self.cmd_STOP_LED_EFFECTS)
        # The builtins print.cfg renames on klippy:connect.
        self.gcode.register_command('CANCEL_PRINT', self._base_cancel)
        self.gcode.register_command('PAUSE', self._base_pause)
        self.gcode.register_command('RESUME', self._base_resume)

        top = os.path.join(MACROS, '_test_m1_led_modes.cfg')
        data = ''.join('[include %s]\n' % (name,) for name in includes)
        reader = configfile.ConfigFileReader()
        fileconfig = reader.build_fileconfig_with_includes(data, top)
        self.effects = []
        for section in fileconfig.sections():
            config = FakeConfig(printer, fileconfig, section)
            kind, _, name = section.partition(' ')
            if kind == 'gcode_macro':
                if name.upper() in STUB_COMMANDS:
                    continue
                obj = gcode_macro.GCodeMacro(config)
            elif kind == 'print_end_park':
                obj = print_end_park.load_config(config)
            elif kind == 'delayed_gcode':
                obj = delayed_gcode.DelayedGcode(config)
            elif kind == 'led_effect':
                obj = FakeLedEffect(config, self.effects, self.macro,
                                    self.stubbed)
            else:
                continue
            printer.add_object(section, obj)
        printer.send_event('klippy:connect')
        printer.send_event('klippy:ready')
        self.responses.clear()

    def _stub(self, gcmd):
        self.stubbed.append(gcmd.get_commandline())

    def _base_cancel(self, gcmd):
        self.stubbed.append('CANCEL_PRINT_BASE')
        self.print_stats['state'] = 'cancelled'

    def _base_pause(self, gcmd):
        self.stubbed.append('PAUSE_BASE')
        if not self.pause_resume['is_paused']:
            self.pause_resume['is_paused'] = True
            self.print_stats['state'] = 'paused'

    def _base_resume(self, gcmd):
        self.stubbed.append('RESUME_BASE')
        if self.pause_resume['is_paused']:
            self.pause_resume['is_paused'] = False
            self.print_stats['state'] = 'printing'

    def job(self, state):
        """Set print_stats.state, as virtual_sdcard would."""
        self.print_stats['state'] = state

    def commands(self):
        """The stubbed and LED commands run so far, by name, in order."""
        return [c.split()[0].upper() for c in self.stubbed if c.strip()]

    def assert_before_leds(self, test, names):
        """Every name in names ran, and before any LED command."""
        done = self.commands()
        led = [i for i, c in enumerate(done)
               if c in ('SET_LED_EFFECT', 'STOP_LED_EFFECTS')]
        for name in names:
            test.assertIn(name, done)
            if led:
                test.assertLess(done.index(name), led[0],
                                '%s ran after an LED command: %s'
                                % (name, done))

    def effect(self, name):
        return self.printer.lookup_object('led_effect ' + name)

    def cmd_STOP_LED_EFFECTS(self, gcmd):
        self.stubbed.append('STOP_LED_EFFECTS')
        # The M1 macros never pass LEDS=, so every effect stops.
        if gcmd.get('LEDS', ''):
            raise gcmd.error('LEDS= is not modelled by this harness')
        for effect in self.effects:
            effect.enabled = False

    def run(self, script):
        self.gcode.run_script(script)

    def wait(self, seconds):
        self.reactor.advance(seconds)

    def lit(self):
        return {e.name for e in self.effects if e.enabled}

    def macro_vars(self, name):
        return self.printer.lookup_object('gcode_macro ' + name).variables

    def laststate(self):
        return self.macro_vars('LED_TIMEOUT_STATE')['laststate']

    def saved(self):
        status = self.printer.lookup_object('save_variables').get_status(0.)
        return status['variables']

    def mode_params(self):
        """SET_LED_EFFECT's params on the last muon_mode_left render."""
        return self.printer.lookup_object('led_effect muon_mode_left').params

    def strip_params(self, strip):
        return self.printer.lookup_object(
            'led_effect muon_mode_' + strip).params

    def forget_per_strip_saves(self):
        """Back to a variables file from before KAN-4899: only muon_led."""
        saved = self.printer.lookup_object('save_variables').allVariables
        saved.pop('muon_led_left', None)
        saved.pop('muon_led_right', None)

    def errors(self):
        # Klippy answers an unknown command with a plain '//' line, not
        # an error, so a misspelt macro name would otherwise pass.
        return [r for r in self.responses
                if r.startswith('!!') or 'Unknown command' in r]


class _ErrorLog(logging.Handler):
    def __init__(self):
        super().__init__(logging.ERROR)
        self.records = []

    def emit(self, record):
        self.records.append(self.format(record))


class LedTestCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.log = _ErrorLog()
        logging.getLogger().addHandler(self.log)
        self.addCleanup(logging.getLogger().removeHandler, self.log)
        self.leds = M1Leds(os.path.join(tmp.name, 'variables.cfg'))
        # Past the boot-time _LED_MODE_RESTORE (initial_duration: 2).
        self.leds.wait(3)
        self.assertEqual(self.leds.lit(), {'ready'})

    def tearDown(self):
        # A delayed_gcode swallows its errors into the log, so a broken
        # template in a timer would otherwise pass silently.
        self.assertEqual(self.leds.errors(), [])
        self.assertEqual(self.log.records, [])
        self.assertEqual(self.leds.printer.shutdowns, [])

    def print_end(self):
        """PRINT_END as a print file runs it, then the file ends."""
        self.leds.run('PRINT_END')
        for name in SHUTDOWN:
            self.assertIn(name, self.leds.commands())
        if self.leds.print_stats['state'] == 'printing':
            self.leds.job('complete')

    def expect_error(self, script):
        """Run script, which must raise; forget the error it reported."""
        with self.assertRaises(gcode.CommandError):
            self.leds.run(script)
        self.assertTrue(self.leds.errors())
        self.leds.responses.clear()
        self.log.records.clear()


class HarnessCanFail(LedTestCase):
    """The harness itself: a check that could not fail proves nothing."""

    def test_single_brace_variables(self):
        # gcode_macro.py's environment, not stock Jinja's '{{ }}'.
        template = self.leds.macro.env.from_string('S{1 + 1}')
        self.assertEqual(template.render(), 'S2')

    def test_unknown_command_is_an_error(self):
        self.leds.run('LED_NO_SUCH_MACRO')
        self.assertTrue(self.leds.errors())
        self.leds.responses.clear()

    def test_a_raising_timer_is_seen(self):
        # DelayedGcode logs and swallows what its script raises, so
        # tearDown checks the responses and the log.
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.effect('muon_mode_left').fail = True
        self.leds.run('SET_GCODE_VARIABLE MACRO=LED_TIMEOUT_STATE '
                      'VARIABLE=laststate VALUE="\'LED_COMPLETE\'"')
        self.leds.run('UPDATE_DELAYED_GCODE ID=_LED_MODE_COMPLETE_DONE '
                      'DURATION=1')
        self.leds.wait(2)
        self.assertTrue(self.leds.errors())
        records, self.log.records = self.log.records, []
        self.assertTrue(records)
        self.leds.responses.clear()

    def test_an_effect_can_be_made_to_fail(self):
        self.leds.effect('info').fail = True
        self.expect_error('LED_INFO')


class StatusModeUnchanged(LedTestCase):
    """The default mode, 'status', behaves as it did before KAN-480."""

    def test_ready(self):
        self.leds.run('LED_INFO')
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_print_flow_ends_on_complete_and_stays(self):
        self.leds.run('LED_SETUP_PROGRESS')
        self.assertEqual(self.leds.lit(), SETUP)
        self.leds.run('LED_PRINTING')
        self.assertEqual(self.leds.lit(), PROGRESS)
        self.print_end()
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(24 * 3600)
        self.assertEqual(self.leds.lit(), {'complete'})
        self.assertEqual(self.leds.laststate(), 'LED_COMPLETE')

    def test_timeout(self):
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.leds.wait(119)
        self.assertEqual(self.leds.lit(), {'ready'})
        self.leds.wait(2)
        self.assertEqual(self.leds.lit(), set())
        self.leds.run('LED_TIMEOUT_SET M=0')
        self.assertEqual(self.leds.lit(), set())
        self.leds.run('LED_TIMEOUT_SET M=-1')
        self.assertEqual(self.leds.lit(), {'ready'})


class ModeChangeWhileBusy(LedTestCase):
    """#61 finding 1: LED_MODE during a print's own effect saves, and
    defers."""

    def setUp(self):
        super().setUp()
        self.leds.job('printing')

    def test_mid_print_keeps_the_progress_display(self):
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('LED_PRINTING')
        for mode in ('solid', 'off', 'rainbow', 'status'):
            with self.subTest(mode=mode):
                self.leds.run('LED_MODE MODE=%s H=30' % (mode,))
                self.assertEqual(self.leds.lit(), PROGRESS)
                self.assertEqual(self.leds.laststate(), 'LED_PRINTING')
                self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'],
                                 mode)
                saved = self.leds.saved()['muon_led_left']
                self.assertEqual(saved['mode'], mode)

    def test_every_active_state_is_kept(self):
        states = {'LED_SETUP_PROGRESS': SETUP, 'LED_PRINTING': PROGRESS,
                  'LED_INFO': {'info'}, 'LED_WARNING': {'warning'},
                  'LED_ERROR': {'error'}}
        for state, effects in sorted(states.items()):
            with self.subTest(state=state):
                self.leds.run(state)
                self.leds.run('LED_MODE MODE=chase H=0 H2=180')
                self.assertEqual(self.leds.lit(), effects)
                self.assertEqual(self.leds.laststate(), state)

    def test_deferred_mode_shows_at_the_next_ready(self):
        self.leds.run('LED_PRINTING')
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), PROGRESS)
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'], 'solid')

    def test_idle_mode_change_shows_now(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.run('LED_MODE MODE=off')
        self.assertEqual(self.leds.lit(), set())
        self.leds.run('LED_MODE MODE=status')
        self.assertEqual(self.leds.lit(), {'ready'})

    def test_restore_does_not_replace_an_active_state(self):
        self.leds.run('LED_MODE MODE=breathe H=90')
        self.leds.run('LED_PRINTING')
        self.leds.run('UPDATE_DELAYED_GCODE ID=_LED_MODE_RESTORE DURATION=1')
        self.leds.wait(2)
        self.assertEqual(self.leds.lit(), PROGRESS)


class ModeAfterPrintEnd(LedTestCase):
    """#61 finding 2: the selected mode comes back after a print."""

    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE MODE=gradient H=30 H2=210')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('LED_PRINTING')

    def test_complete_then_the_selected_mode(self):
        self.print_end()
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(30)
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.laststate(), 'LED_READY')
        self.assertEqual(self.leds.mode_params()['MODE'], 'gradient')

    def test_a_short_timeout_still_ends_dark(self):
        self.leds.run('LED_TIMEOUT_SET M=1')
        self.print_end()
        self.leds.wait(61)
        self.assertEqual(self.leds.lit(), set())
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), set())

    def test_a_long_timeout_applies_to_the_mode_after(self):
        self.leds.run('LED_TIMEOUT_SET M=30')
        self.print_end()
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.wait(30 * 60 + 1)
        self.assertEqual(self.leds.lit(), set())

    def test_timeout_zero_stays_dark(self):
        self.leds.run('LED_TIMEOUT_SET M=0')
        self.print_end()
        self.assertEqual(self.leds.lit(), set())
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), set())

    def test_a_new_print_is_not_interrupted(self):
        self.print_end()
        self.leds.wait(10)
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), SETUP)

    def test_a_mode_change_after_the_print_shows_now(self):
        self.print_end()
        self.leds.run('LED_MODE MODE=status')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), {'ready'})


class CustomReadyRecordsState(LedTestCase):
    """#61 finding 3: LED_READY records READY whatever the idle look is."""

    def test_status_after_custom_ready_does_not_resume_info(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.run('LED_INFO')
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.laststate(), 'LED_READY')
        self.leds.run('LED_MODE MODE=status')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_timeout_set_resumes_the_custom_mode_not_complete(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.run('LED_COMPLETE')
        self.leds.run('LED_READY')
        self.leds.run('LED_TIMEOUT_SET M=-1')
        self.assertEqual(self.leds.lit(), BOTH_MODE)


class CustomModeTimeout(LedTestCase):
    """#61 finding 4: the LED timeout applies to a custom idle mode."""

    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_timeout_zero_turns_it_off(self):
        self.leds.run('LED_TIMEOUT_SET M=0')
        self.assertEqual(self.leds.lit(), set())

    def test_positive_timeout_turns_it_off_later(self):
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.wait(119)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.wait(2)
        self.assertEqual(self.leds.lit(), set())

    def test_showing_the_mode_restarts_the_timer(self):
        self.leds.run('LED_MODE MODE=status')
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.leds.wait(100)
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.wait(30)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.leds.wait(91)
        self.assertEqual(self.leds.lit(), set())

    def test_no_timeout_cancels_a_pending_one(self):
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.leds.run('LED_TIMEOUT_SET M=-1')
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)


class LedsNeverBlockShutdown(LedTestCase):
    """KAN-568 S1: no LED macro stands between the end of a print and the
    heaters, part fan and motors turning off."""

    def setUp(self):
        super().setUp()
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('LED_PRINTING')
        self.leds.stubbed.clear()

    def assert_shut_down(self):
        # Heaters, part fan and motors off, all before any LED command.
        self.leds.assert_before_leds(self, SHUTDOWN)

    def test_a_negative_complete_hold_cannot_raise(self):
        for mode in ('solid', 'status'):
            with self.subTest(mode=mode):
                self.leds.stubbed.clear()
                self.leds.run('LED_MODE MODE=%s' % (mode,))
                self.leds.run('SET_GCODE_VARIABLE MACRO=LED_MODE '
                              'VARIABLE=complete_hold VALUE=-1')
                self.leds.stubbed.clear()
                self.leds.run('PRINT_END')
                self.assert_shut_down()
                self.assertEqual(self.leds.lit(), {'complete'})
                # Clamped to 0: no return, so the glow stays.
                self.leds.wait(3600)
                self.assertEqual(self.leds.lit(), {'complete'})

    def test_print_end_shuts_down_when_an_led_macro_raises(self):
        self.leds.effect('complete').fail = True
        self.expect_error('PRINT_END')
        self.assert_shut_down()
        self.assertIn('RESTORE_GCODE_STATE', self.leds.commands())

    def test_cancel_print_shuts_down_when_an_led_macro_raises(self):
        self.leds.effect('complete').fail = True
        self.expect_error('CANCEL_PRINT')
        self.assertIn('CANCEL_PRINT_BASE', self.leds.commands())
        self.assert_shut_down()

    def test_cancel_print_ends_on_the_glow(self):
        self.leds.run('CANCEL_PRINT')
        self.assert_shut_down()
        self.assertEqual(self.leds.lit(), {'complete'})
        self.assertEqual(self.leds.print_stats['state'], 'cancelled')

    def test_a_print_error_turns_the_heaters_off_first(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.effect('warning').fail = True
        self.expect_error('_ON_PRINT_ERROR')
        self.leds.assert_before_leds(self, ['TURN_OFF_HEATERS'])
        self.assertIn('SET_LED_EFFECT', self.leds.commands())

    def test_the_order_holds_without_a_failure(self):
        for script in ('PRINT_END', 'CANCEL_PRINT'):
            with self.subTest(script=script):
                self.leds.job('printing')
                self.leds.stubbed.clear()
                self.leds.run(script)
                self.assert_shut_down()
                self.assertIn('SET_LED_EFFECT', self.leds.commands())


class ConnectPulseRestoresState(LedTestCase):
    """KAN-568 S2: the LED_CONNECTED pulse goes back to the state it
    interrupted, and never records READY on its own."""

    def start_print(self):
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('LED_PRINTING')

    def test_connect_mid_print(self):
        for mode in ('status', 'solid'):
            with self.subTest(mode=mode):
                self.leds.job('standby')
                self.leds.run('LED_READY')
                self.leds.run('LED_MODE MODE=%s H=30' % (mode,))
                self.start_print()
                self.leds.run('LED_CONNECTED')
                self.assertEqual(self.leds.lit(), {'connected'})
                self.leds.wait(1)
                self.assertEqual(self.leds.lit(), PROGRESS)
                self.assertEqual(self.leds.laststate(), 'LED_PRINTING')
                # So a mode chosen now still waits.
                self.leds.run('LED_MODE MODE=rainbow')
                self.assertEqual(self.leds.lit(), PROGRESS)

    def test_disconnect_mid_print_shows_the_pause(self):
        # TOOLHEAD_DISCONNECT: LED_CONNECTED, then PAUSE while printing.
        for mode in ('status', 'solid'):
            with self.subTest(mode=mode):
                self.leds.job('standby')
                self.leds.run('LED_READY')
                self.leds.run('LED_MODE MODE=%s H=30' % (mode,))
                self.start_print()
                self.leds.run('LED_CONNECTED')
                self.leds.run('PAUSE')
                self.leds.wait(1)
                self.assertEqual(self.leds.lit(), {'info'})
                self.assertEqual(self.leds.laststate(), 'LED_INFO')
                self.leds.run('RESUME')
                self.assertEqual(self.leds.lit(), PROGRESS)
                self.assertEqual(self.leds.print_stats['state'], 'printing')

    def test_connect_when_idle(self):
        self.leds.run('LED_CONNECTED')
        self.leds.wait(1)
        self.assertEqual(self.leds.lit(), {'ready'})
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.run('LED_CONNECTED')
        self.leds.wait(1)
        self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_connect_during_the_glow_after_a_print(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.start_print()
        self.print_end()
        self.leds.wait(10)
        self.leds.run('LED_CONNECTED')
        self.leds.wait(1)
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)


class ModeShowsWhenNoPrint(LedTestCase):
    """KAN-568 S3: with no print running or paused, a chosen mode shows
    whatever state an unload, a prompt or a failed print left."""

    def test_after_an_unload(self):
        self.leds.run('UNLOAD_FILAMENT TEMP=250')
        self.assertEqual(self.leds.laststate(), 'LED_INFO')
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_after_a_dismissed_filament_prompt(self):
        for prompt in ('FILAMENT_INSERT_MODE_PROMPT',
                       'FILAMENT_INSERT_LOAD_FAILED_PROMPT'):
            with self.subTest(prompt=prompt):
                self.leds.run('LED_MODE MODE=status')
                self.leds.run(prompt)
                self.assertEqual(self.leds.lit(), {'info'})
                self.leds.run('LED_MODE MODE=chase H=0 H2=180')
                self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_after_a_failed_print(self):
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('_ON_PRINT_ERROR')
        self.assertIn('TURN_OFF_HEATERS', self.leds.commands())
        # 'status' keeps what the failed print left lit, as before.
        self.assertEqual(self.leds.lit(), SETUP)
        self.leds.job('error')
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_a_mode_chosen_during_a_print_that_fails(self):
        self.leds.job('printing')
        self.leds.run('LED_PRINTING')
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), PROGRESS)
        self.leds.run('_ON_PRINT_ERROR')
        self.assertIn('TURN_OFF_HEATERS', self.leds.commands())
        self.leds.job('error')
        self.assertEqual(self.leds.lit(), {'warning'})
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_a_paused_print_still_defers(self):
        self.leds.job('printing')
        self.leds.run('LED_PRINTING')
        self.leds.run('PAUSE')
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), {'info'})
        self.leds.run('RESUME')
        self.assertEqual(self.leds.lit(), PROGRESS)


class SavedModeAndStandalone(LedTestCase):
    """KAN-568 nits: a bad saved mode, and led.cfg on its own."""

    def restore(self, value):
        self.leds.forget_per_strip_saves()
        self.leds.run('SAVE_VARIABLE VARIABLE=muon_led VALUE="%s"' % (value,))
        self.leds.run('UPDATE_DELAYED_GCODE ID=_LED_MODE_RESTORE DURATION=1')
        self.leds.wait(2)

    def test_an_unknown_saved_mode_is_status(self):
        self.restore("{'mode': 'neon', 'h': 10, 'h2': 20, 'bright': 50, "
                     "'speed': 50}")
        self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'], 'status')
        self.assertEqual(self.leds.lit(), {'ready'})

    def test_missing_saved_numbers_keep_the_defaults(self):
        self.restore("{'mode': 'solid'}")
        self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'], 'solid')
        self.assertEqual(self.leds.macro_vars('LED_MODE')['h'], 0.)
        self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_a_good_saved_mode_is_restored(self):
        self.restore("{'mode': 'breathe', 'h': 90, 'h2': 180, "
                     "'bright': 40, 'speed': 30}")
        self.assertEqual(self.leds.mode_params()['MODE'], 'breathe')
        self.assertEqual(self.leds.macro_vars('LED_MODE')['bright'], 40.)


class LedCfgWithoutModes(unittest.TestCase):
    """KAN-568 nit: led.cfg (and print.cfg) work without led_modes.cfg."""

    def test_status_behaviour_without_led_modes(self):
        log = _ErrorLog()
        logging.getLogger().addHandler(log)
        self.addCleanup(logging.getLogger().removeHandler, log)
        with tempfile.TemporaryDirectory() as tmp:
            leds = M1Leds(os.path.join(tmp, 'variables.cfg'),
                          includes=('led.cfg', 'print.cfg'))
            leds.run('LED_INFO')
            leds.run('LED_READY')
            self.assertEqual(leds.lit(), {'ready'})
            leds.job('printing')
            leds.run('LED_PRINTING')
            leds.run('LED_CONNECTED')
            leds.wait(1)
            self.assertEqual(leds.lit(), PROGRESS)
            leds.run('PRINT_END')
            self.assertEqual(leds.lit(), {'complete'})
            leds.run('_ON_PRINT_ERROR')
            self.assertEqual(leds.errors(), [])
        self.assertEqual(log.records, [])


class FailedPrintWarns(LedTestCase):
    """#72 review S1: in a user mode a failed print shows the warning for
    complete_hold, then the mode; it never looks like idle at once."""

    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.stubbed.clear()

    def fail_print(self):
        # virtual_sdcard runs on_error_gcode, then notes the error.
        self.leds.run('_ON_PRINT_ERROR')
        self.leds.job('error')

    def test_warning_then_the_mode(self):
        self.fail_print()
        self.leds.assert_before_leds(self, ['TURN_OFF_HEATERS'])
        self.assertEqual(self.leds.lit(), {'warning'})
        self.assertEqual(self.leds.laststate(), 'LED_WARNING')
        self.leds.wait(30)
        self.assertEqual(self.leds.lit(), {'warning'})
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_status_keeps_what_was_lit(self):
        self.leds.run('LED_MODE MODE=status')
        self.leds.run('LED_SETUP_PROGRESS')
        self.fail_print()
        self.assertEqual(self.leds.lit(), SETUP)
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), SETUP)

    def test_a_short_timeout_still_ends_dark(self):
        self.leds.run('SET_GCODE_VARIABLE MACRO=LED_TIMEOUT_STATE '
                      'VARIABLE=minutes VALUE=1')
        self.fail_print()
        self.assertEqual(self.leds.lit(), {'warning'})
        self.leds.wait(61)
        self.assertEqual(self.leds.lit(), set())
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), set())

    def test_a_new_print_is_not_interrupted(self):
        self.fail_print()
        self.leds.wait(10)
        self.leds.job('printing')
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), SETUP)

    def test_choosing_a_mode_ends_the_warning(self):
        self.fail_print()
        self.leds.run('LED_MODE MODE=rainbow')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.mode_params()['MODE'], 'rainbow')


class PromptAfterConnect(LedTestCase):
    """#72 review S2: TOOLHEAD_CONNECT pulses, then (KAN-232) always runs
    LOAD_FILAMENT, which ends on a prompt. The prompt's info glow wins
    over the idle look, in 'status' and in a user mode."""

    def connect_with_prompt(self):
        self.leds.run('LED_CONNECTED')
        self.leds.run('FILAMENT_INSERT_MODE_PROMPT')

    def test_connect_then_prompt(self):
        for mode, idle in (('status', 'ready'), ('solid', BOTH_MODE)):
            with self.subTest(mode=mode):
                self.leds.run('LED_MODE MODE=%s H=30' % (mode,))
                self.assertEqual(self.leds.lit(), idle_set(idle))
                self.connect_with_prompt()
                self.leds.wait(1)
                self.assertEqual(self.leds.lit(), {'info'})
                self.assertEqual(self.leds.laststate(), 'LED_INFO')
                # A button on the prompt ends it, and the idle look returns.
                self.leds.run('FILAMENT_INSERT_QUICK_LOAD')
                self.assertEqual(self.leds.lit(), idle_set(idle))


class BootOrder(unittest.TestCase):
    """#72 review S2: at boot _LED_MODE_RESTORE (2 s) races the toolhead
    reconnect (about 2.12 s), which opens a filament prompt. Either way
    round, the prompt's info glow is what stays lit."""

    def boot(self, mode):
        log = _ErrorLog()
        logging.getLogger().addHandler(log)
        self.addCleanup(logging.getLogger().removeHandler, log)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, 'variables.cfg')
        with open(path, 'w') as handle:
            handle.write("[Variables]\nmuon_led = {'mode': '%s', 'h': 30.0, "
                         "'h2': 180.0, 'bright': 60.0, 'speed': 50.0}\n"
                         % (mode,))
        leds = M1Leds(path)
        self.addCleanup(lambda: self.assertEqual(leds.errors(), []))
        self.addCleanup(lambda: self.assertEqual(log.records, []))
        return leds

    def connect_with_prompt(self, leds):
        leds.run('LED_CONNECTED')
        leds.run('FILAMENT_INSERT_MODE_PROMPT')

    def test_restore_first(self):
        for mode, idle in (('status', 'ready'), ('solid', BOTH_MODE)):
            with self.subTest(mode=mode):
                leds = self.boot(mode)
                leds.wait(2.05)
                self.assertEqual(leds.lit(), idle_set(idle))
                leds.wait(0.07)
                self.connect_with_prompt(leds)
                leds.wait(3)
                self.assertEqual(leds.lit(), {'info'})
                self.assertEqual(leds.macro_vars('LED_MODE')['mode'], mode)
                leds.run('FILAMENT_INSERT_QUICK_LOAD')
                self.assertEqual(leds.lit(), idle_set(idle))

    def test_reconnect_first(self):
        for mode, idle in (('status', 'ready'), ('solid', BOTH_MODE)):
            with self.subTest(mode=mode):
                leds = self.boot(mode)
                leds.wait(1.9)
                self.connect_with_prompt(leds)
                leds.wait(3)
                self.assertEqual(leds.lit(), {'info'})
                self.assertEqual(leds.laststate(), 'LED_INFO')
                self.assertEqual(leds.macro_vars('LED_MODE')['mode'], mode)
                leds.run('FILAMENT_INSERT_QUICK_LOAD')
                self.assertEqual(leds.lit(), idle_set(idle))


class ValidModesAreFixed(LedTestCase):
    """#72 review N2: the valid modes can't be widened at runtime, and
    LED_MODE and the boot restore accept the same ones."""

    def test_no_runtime_variable(self):
        self.expect_error('SET_GCODE_VARIABLE MACRO=LED_MODE VARIABLE=modes '
                          'VALUE="[\'neon\']"')
        self.leds.run('LED_MODE MODE=neon')
        self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'], 'status')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.leds.responses.clear()

    def test_led_mode_and_restore_accept_the_same_modes(self):
        names = ('solid', 'gradient', 'rainbow', 'breathe', 'chase',
                 'status', 'off', 'neon', 'Solid', 'statuss', 'of')
        for name in names:
            with self.subTest(name=name):
                self.leds.run('LED_MODE MODE=status')
                if name.lower() in ('solid', 'gradient', 'rainbow',
                                    'breathe', 'chase', 'status', 'off'):
                    self.leds.run('LED_MODE MODE=%s' % (name,))
                else:
                    # A bad MODE is an info line and changes nothing.
                    self.leds.run('LED_MODE MODE=%s' % (name,))
                by_command = self.leds.macro_vars('LED_MODE')['mode']
                self.leds.run('LED_MODE MODE=status')
                self.leds.forget_per_strip_saves()
                self.leds.run('SAVE_VARIABLE VARIABLE=muon_led '
                              'VALUE="{\'mode\': \'%s\'}"' % (name,))
                self.leds.run('UPDATE_DELAYED_GCODE ID=_LED_MODE_RESTORE '
                              'DURATION=1')
                self.leds.wait(2)
                by_restore = self.leds.macro_vars('LED_MODE')['mode']
                self.assertEqual(by_command, by_restore)
                self.assertEqual(by_command != 'status',
                                 name.lower() in ('solid', 'gradient',
                                                  'rainbow', 'breathe',
                                                  'chase', 'off'))
        self.leds.responses.clear()


# ---- KAN-4899: LED_MODE STRIP= and SAT= ------------------------------------

def old_rgb(hue, bright):
    """The colour LED_MODE rendered before KAN-4899 (HSL 85%/55%)."""
    hf = (hue % 360) / 60.0
    c = (1 - abs(2 * 0.55 - 1)) * 0.85
    x = c * (1 - abs((hf % 2) - 1))
    lp = 0.55 - c / 2
    rgb = [(c, x, 0), (x, c, 0), (0, c, x), (0, x, c), (x, 0, c),
           (c, 0, x)][min(int(hf), 5)]
    return [(v + lp) * bright / 100 for v in rgb]


def rgb_of(params, suffix=''):
    return [float(params[k + suffix]) for k in 'RGB']


class StripAndSatContract(LedTestCase):
    """What Control reads, from boot, before any LED_MODE call."""

    def test_variables_exist_at_boot(self):
        v = self.leds.macro_vars('LED_MODE')
        self.assertIn('sat', v)
        self.assertEqual(v['sat'], 100)
        for strip in ('left', 'right'):
            self.assertEqual(sorted(v[strip]),
                             ['bright', 'h', 'h2', 'mode', 'sat', 'speed'])
            self.assertEqual(v[strip]['mode'], 'status')
            self.assertEqual(v[strip]['sat'], 100)

    def test_the_per_strip_effects_differ_only_in_leds(self):
        left = self.leds.effect('muon_mode_left')
        right = self.leds.effect('muon_mode_right')
        self.assertEqual(left.leds, {('neopixel:Left', i)
                                     for i in range(1, CHAIN_LENGTH + 1)})
        self.assertEqual(right.leds, {('neopixel:Right', i)
                                      for i in range(1, CHAIN_LENGTH + 1)})
        self.assertEqual(left.layers, right.layers)
        # Control: the effect list really is the split one.
        names = {e.name for e in self.leds.effects}
        self.assertNotIn('muon_mode', names)

    def test_live_state_follows_every_call(self):
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30 SAT=40')
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual(v['left'], {'mode': 'solid', 'h': 30.0, 'h2': 180.0,
                                     'bright': 60.0, 'speed': 50.0,
                                     'sat': 40.0})
        self.assertEqual(v['right']['mode'], 'status')
        self.assertEqual(v['sat'], 40.0)
        # variable_sat is the left strip's when the two differ.
        self.leds.run('LED_MODE STRIP=right SAT=10')
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual(v['right']['sat'], 10.0)
        self.assertEqual(v['sat'], 40.0)
        # ... and the shared value after a STRIP=both call.
        self.leds.run('LED_MODE SAT=70')
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual((v['left']['sat'], v['right']['sat'], v['sat']),
                         (70.0, 70.0, 70.0))


class StripSelectsTheEffects(LedTestCase):
    def sets(self):
        return [c for c in self.leds.stubbed if c.startswith('SET_LED_EFFECT')]

    def test_left_only(self):
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30')
        self.assertEqual(self.leds.lit(), {'muon_mode_left',
                                           'muon_status_right'})
        self.assertNotIn('SET_LED_EFFECT muon_mode_right', self.leds.stubbed)
        self.assertEqual(self.leds.macro_vars('LED_MODE')['right']['mode'],
                         'status')
        self.assertEqual(self.leds.saved()['muon_led_left']['mode'], 'solid')
        self.assertNotIn('muon_led_right', self.leds.saved())

    def test_right_only(self):
        self.leds.run('LED_MODE STRIP=right MODE=solid H=200')
        self.assertEqual(self.leds.lit(), {'muon_mode_right',
                                           'muon_status_left'})
        self.assertNotIn('SET_LED_EFFECT muon_mode_left', self.leds.stubbed)
        self.assertEqual(self.leds.saved()['muon_led_right']['mode'], 'solid')
        self.assertNotIn('muon_led_left', self.leds.saved())

    def test_both_is_the_default_and_equal_to_explicit_both(self):
        self.leds.run('LED_MODE MODE=solid H=120')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        by_default = {s: self.leds.strip_params(s) for s in ('left', 'right')}
        self.leds.run('LED_MODE STRIP=BOTH MODE=solid H=120')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(by_default['left'], self.leds.strip_params('left'))
        self.assertEqual(by_default['right'], self.leds.strip_params('right'))

    def test_the_other_strip_is_left_alone(self):
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30')
        self.leds.stubbed.clear()
        self.leds.run('LED_MODE STRIP=right MODE=breathe H=200')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.sets(), ['SET_LED_EFFECT muon_mode_right'])
        self.assertEqual(self.leds.strip_params('left')['MODE'], 'solid')
        self.assertEqual(self.leds.strip_params('right')['MODE'], 'breathe')
        # Control: a both call does redraw both.
        self.leds.stubbed.clear()
        self.leds.run('LED_MODE MODE=solid')
        self.assertEqual(sorted(self.sets()),
                         ['SET_LED_EFFECT muon_mode_left',
                          'SET_LED_EFFECT muon_mode_right'])

    def test_off_stops_only_that_strip(self):
        self.leds.run('LED_MODE MODE=solid')
        self.leds.run('LED_MODE STRIP=left MODE=off')
        self.assertEqual(self.leds.lit(), {'muon_mode_right'})
        self.leds.run('LED_MODE STRIP=right MODE=off')
        self.assertEqual(self.leds.lit(), set())
        # Control: one strip off never used the stop-everything command;
        # the old both-strips off still does.
        self.assertNotIn('STOP_LED_EFFECTS', self.leds.commands())
        self.leds.run('LED_MODE MODE=solid')
        self.leds.run('LED_MODE MODE=off')
        self.assertIn('STOP_LED_EFFECTS', self.leds.commands())
        self.assertEqual(self.leds.lit(), set())

    def test_status_on_one_strip_keeps_the_other_mode(self):
        self.leds.run('LED_MODE MODE=solid')
        self.leds.run('LED_MODE STRIP=right MODE=status')
        self.assertEqual(self.leds.lit(), {'muon_mode_left',
                                           'muon_status_right'})
        # Both back to status: the plain 'ready' look.
        self.leds.run('LED_MODE STRIP=left MODE=status')
        self.assertEqual(self.leds.lit(), {'ready'})

    def test_a_strip_change_after_a_print_redraws_both(self):
        # The glow covers both strips, so one strip's redraw would leave
        # the other on it.
        self.leds.run('LED_MODE MODE=solid')
        self.leds.run('LED_COMPLETE')
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.run('LED_MODE STRIP=left MODE=breathe')
        self.assertEqual(self.leds.lit(), BOTH_MODE)


class SaturationAndWhite(LedTestCase):
    def test_sat_zero_is_white_at_bright(self):
        self.leds.run('LED_MODE MODE=solid H=30 BRIGHT=60 SAT=0')
        for strip in ('left', 'right'):
            r, g, b = rgb_of(self.leds.strip_params(strip))
            self.assertAlmostEqual(r, 0.6)
            self.assertAlmostEqual(g, 0.6)
            self.assertAlmostEqual(b, 0.6)
        # Control: with colour the channels differ.
        self.leds.run('LED_MODE MODE=solid H=30 BRIGHT=60 SAT=100')
        r, g, b = rgb_of(self.leds.strip_params('left'))
        self.assertNotAlmostEqual(r, b)

    def test_sat_zero_gradient_both_stops_white(self):
        self.leds.run('LED_MODE MODE=gradient H=0 H2=200 BRIGHT=50 SAT=0')
        p = self.leds.strip_params('left')
        self.assertEqual(rgb_of(p), rgb_of(p, '2'))
        self.assertAlmostEqual(rgb_of(p)[0], 0.5)

    def test_sat_zero_rainbow_is_white(self):
        self.leds.run('LED_MODE STRIP=left MODE=rainbow BRIGHT=50 SAT=0')
        layers = self.leds.effect('muon_mode_left').layers
        stops = [tuple(float(v) for v in t.strip('()').split(','))
                 for t in layers.split('add ')[1].split('\n')[0].split('),(')]
        self.assertEqual(len(stops), 6)
        for stop in stops:
            self.assertAlmostEqual(stop[0], 0.5)
            self.assertAlmostEqual(stop[1], 0.5)
            self.assertAlmostEqual(stop[2], 0.5)
        # Control: SAT=100 keeps the six colours apart.
        self.leds.run('LED_MODE STRIP=left MODE=rainbow BRIGHT=50 SAT=100')
        layers = self.leds.effect('muon_mode_left').layers
        self.assertIn('(0.5,0.035', layers)

    def test_half_saturation_is_between(self):
        self.leds.run('LED_MODE MODE=solid H=0 BRIGHT=100 SAT=100')
        full = rgb_of(self.leds.strip_params('left'))
        self.leds.run('LED_MODE MODE=solid H=0 BRIGHT=100 SAT=50')
        half = rgb_of(self.leds.strip_params('left'))
        for f, h in zip(full, half):
            self.assertAlmostEqual(h, (f + 1) / 2)

    def test_sat_is_clamped_not_rejected(self):
        self.leds.run('LED_MODE MODE=solid SAT=150')
        self.assertEqual(self.leds.macro_vars('LED_MODE')['sat'], 100.0)
        self.leds.run('LED_MODE MODE=solid SAT=-5')
        self.assertEqual(self.leds.macro_vars('LED_MODE')['sat'], 0.0)


class OldCallersUnchanged(LedTestCase):
    def test_the_panel_call(self):
        self.leds.run('LED_MODE MODE=solid H=30 H2=200 BRIGHT=70 SPEED=40')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        want = old_rgb(30, 70)
        want2 = old_rgb(200, 70)
        for strip in ('left', 'right'):
            p = self.leds.strip_params(strip)
            for got, w in zip(rgb_of(p), want):
                self.assertEqual(got, w)
            for got, w in zip(rgb_of(p, '2'), want2):
                self.assertEqual(got, w)
            self.assertEqual(p['MODE'], 'solid')
            self.assertEqual(float(p['SPEED']), 40.0)
            self.assertEqual(float(p['BRIGHT']), 70.0)
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual((v['mode'], v['h'], v['h2'], v['bright'], v['speed']),
                         ('solid', 30.0, 200.0, 70.0, 40.0))
        self.assertEqual(v['left'], v['right'])
        self.assertEqual(self.leds.saved()['muon_led_left'],
                         self.leds.saved()['muon_led_right'])
        # Control: a different hue gives a different colour.
        self.assertNotEqual(want, old_rgb(90, 70))

    def test_old_ranges_still_clamp(self):
        self.leds.run('LED_MODE MODE=solid H=400 BRIGHT=250 SPEED=1')
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual((v['h'], v['bright'], v['speed']), (40.0, 100.0, 5.0))

    def test_rainbow_at_full_sat_is_the_old_palette(self):
        self.leds.run('LED_MODE MODE=rainbow BRIGHT=60 SPEED=50')
        layers = self.leds.effect('muon_mode_left').layers
        k = 60.0 / 100.0
        pal = ([1.0, .07, .07], [1.0, .65, .07], [.07, 1.0, .07],
               [.07, .65, 1.0], [.07, .07, 1.0], [.65, .07, 1.0])
        old = 'gradient 0.275 1.0 add ' + ','.join(
            '(%s,%s,%s)' % tuple(v * k for v in rgb) for rgb in pal)
        self.assertIn(old, layers)


class MigrationFromMuonLed(unittest.TestCase):
    def boot(self, text):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, 'variables.cfg')
        with open(path, 'w') as handle:
            handle.write('[Variables]\n' + text)
        leds = M1Leds(path)
        self.addCleanup(lambda: self.assertEqual(leds.errors(), []))
        return leds

    def test_old_muon_led_becomes_both_strips(self):
        leds = self.boot("muon_led = {'mode': 'chase', 'h': 30.0, "
                         "'h2': 90.0, 'bright': 55.0, 'speed': 20.0}\n")
        leds.wait(3)
        v = leds.macro_vars('LED_MODE')
        want = {'mode': 'chase', 'h': 30.0, 'h2': 90.0, 'bright': 55.0,
                'speed': 20.0, 'sat': 100.0}
        self.assertEqual(v['left'], want)
        self.assertEqual(v['right'], want)
        self.assertEqual(leds.lit(), BOTH_MODE)
        saved = leds.saved()
        self.assertEqual(saved['muon_led_left'], want)
        self.assertEqual(saved['muon_led_right'], want)
        # The old key stays as the fallback.
        self.assertEqual(saved['muon_led']['mode'], 'chase')

    def test_a_second_boot_reads_the_new_keys(self):
        leds = self.boot("muon_led = {'mode': 'solid', 'h': 30.0}\n"
                         "muon_led_left = {'mode': 'breathe', 'h': 10.0, "
                         "'h2': 20.0, 'bright': 30.0, 'speed': 40.0, "
                         "'sat': 25.0}\n"
                         "muon_led_right = {'mode': 'off'}\n")
        leds.wait(3)
        v = leds.macro_vars('LED_MODE')
        self.assertEqual(v['left']['mode'], 'breathe')
        self.assertEqual(v['left']['sat'], 25.0)
        self.assertEqual(v['right']['mode'], 'off')
        # Control: muon_led was not used for either strip.
        self.assertNotEqual(v['left']['h'], 30.0)
        self.assertEqual(leds.saved()['muon_led_right'], {'mode': 'off'})

    def test_a_strip_missing_its_key_falls_back_to_muon_led(self):
        leds = self.boot("muon_led = {'mode': 'solid', 'h': 30.0}\n"
                         "muon_led_left = {'mode': 'breathe'}\n")
        leds.wait(3)
        v = leds.macro_vars('LED_MODE')
        self.assertEqual(v['left']['mode'], 'breathe')
        self.assertEqual(v['right']['mode'], 'solid')
        self.assertEqual(v['right']['h'], 30.0)
        self.assertEqual(leds.saved()['muon_led_left'], {'mode': 'breathe'})

    def test_nothing_saved_stays_on_status(self):
        leds = self.boot('')
        leds.wait(3)
        self.assertEqual(leds.lit(), {'ready'})
        self.assertNotIn('muon_led_left', leds.saved())

    def test_a_restart_keeps_each_strip(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, 'variables.cfg')
        first = M1Leds(path)
        first.wait(3)
        first.run('LED_MODE STRIP=left MODE=solid H=30 SAT=0')
        first.run('LED_MODE STRIP=right MODE=breathe H=200 SPEED=80')
        want = (first.macro_vars('LED_MODE')['left'],
                first.macro_vars('LED_MODE')['right'])
        second = M1Leds(path)
        self.addCleanup(lambda: self.assertEqual(second.errors(), []))
        second.wait(3)
        self.assertEqual((second.macro_vars('LED_MODE')['left'],
                          second.macro_vars('LED_MODE')['right']), want)
        self.assertEqual(second.lit(), BOTH_MODE)
        self.assertEqual(second.strip_params('left')['MODE'], 'solid')
        self.assertEqual(second.strip_params('right')['MODE'], 'breathe')
        # Control: the two strips really differ.
        self.assertNotEqual(want[0], want[1])


class FaultsAndPrintsOverrideBothStrips(LedTestCase):
    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30')
        self.leds.run('LED_MODE STRIP=right MODE=breathe H=200 SAT=30')
        self.before = dict(self.leds.macro_vars('LED_MODE'))
        self.assertEqual(self.leds.lit(), BOTH_MODE)

    def assert_choice_kept(self):
        v = self.leds.macro_vars('LED_MODE')
        self.assertEqual(v['left'], self.before['left'])
        self.assertEqual(v['right'], self.before['right'])
        self.assertEqual(v['sat'], self.before['sat'])
        self.assertEqual(self.leds.saved()['muon_led_left']['mode'], 'solid')
        self.assertEqual(self.leds.saved()['muon_led_right']['mode'],
                         'breathe')

    def test_each_fault_effect_replaces_both(self):
        for macro, effect in (('LED_ERROR', 'error'),
                              ('LED_WARNING', 'warning'),
                              ('LED_INFO', 'info'),
                              ('LED_COMPLETE', 'complete')):
            with self.subTest(macro=macro):
                self.leds.run(macro)
                self.assertEqual(self.leds.lit(), {effect})
                self.leds.run('LED_READY')
                self.assertEqual(self.leds.lit(), BOTH_MODE)

    def test_a_fault_then_clear_restores_both_strips(self):
        self.leds.run('LED_ERROR')
        self.assertEqual(self.leds.lit(), {'error'})
        self.assert_choice_kept()
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assertEqual(self.leds.strip_params('left')['MODE'], 'solid')
        self.assertEqual(self.leds.strip_params('right')['MODE'], 'breathe')
        self.assert_choice_kept()

    def test_print_progress_overrides_both_then_restores(self):
        self.leds.run('LED_SETUP_PROGRESS')
        self.assertEqual(self.leds.lit(), SETUP)
        self.leds.run('LED_PRINTING')
        self.assertEqual(self.leds.lit(), PROGRESS)
        self.assert_choice_kept()
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), BOTH_MODE)
        self.assert_choice_kept()

    def test_a_mixed_state_is_overridden_and_restored_too(self):
        self.leds.run('LED_MODE STRIP=right MODE=status')
        self.assertEqual(self.leds.lit(), {'muon_mode_left',
                                           'muon_status_right'})
        self.leds.run('LED_PRINTING')
        self.assertEqual(self.leds.lit(), PROGRESS)
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), {'muon_mode_left',
                                           'muon_status_right'})

    def test_the_override_depends_on_the_leds_overlapping(self):
        # Negative control: were the right effect not on the right LEDs,
        # a fault would leave it lit. That is what the split must not do.
        self.leds.effect('muon_mode_right').leds = set()
        self.leds.run('LED_ERROR')
        self.assertEqual(self.leds.lit(), {'error', 'muon_mode_right'})


BAD_CALLS = ('LED_MODE STRIP=top MODE=solid',
             'LED_MODE STRIP= MODE=solid',
             'LED_MODE STRIP=left SAT=abc',
             'LED_MODE SAT=nan',
             'LED_MODE SAT=inf',
             'LED_MODE MODE=neon',
             'LED_MODE H=red',
             'LED_MODE BRIGHT=x',
             'LED_MODE SPEED=fast',
             # A good STRIP and MODE with one bad number: the good parts
             # must not apply either.
             'LED_MODE STRIP=right MODE=breathe H=x')


class BadParametersAreNonFatalAndChangeNothing(LedTestCase):
    """LED_MODE runs inside print start, print end and fault macros, so by
    default a bad value is an info line, never an error (lead-prn review).
    STRICT=1 is the clients' opt-in to a raised error."""

    def snapshot(self):
        return (dict(self.leds.macro_vars('LED_MODE')),
                dict(self.leds.saved()), self.leds.lit(),
                list(self.leds.stubbed))

    def test_each_bad_call_is_an_info_line_and_changes_nothing(self):
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30')
        for call in BAD_CALLS:
            with self.subTest(call=call):
                before = self.snapshot()
                self.leds.responses.clear()
                self.leds.run(call)
                self.assertEqual(self.snapshot(), before)
                self.assertEqual(self.leds.errors(), [])
                self.assertTrue(any('LED_MODE' in r
                                    for r in self.leds.responses),
                                self.leds.responses)

    def test_the_same_calls_with_strict_raise_and_change_nothing(self):
        self.leds.run('LED_MODE STRIP=left MODE=solid H=30')
        for call in BAD_CALLS:
            with self.subTest(call=call):
                before = self.snapshot()
                self.expect_error(call + ' STRICT=1')
                self.assertEqual(self.snapshot(), before)

    def test_the_message_names_the_problem(self):
        self.leds.run('LED_MODE STRIP=top')
        self.assertTrue(any('STRIP' in r and 'top' in r
                            for r in self.leds.responses))
        with self.assertRaises(gcode.CommandError) as ctx:
            self.leds.run('LED_MODE STRIP=top STRICT=1')
        self.assertIn('STRIP', str(ctx.exception))
        self.leds.responses.clear()
        self.log.records.clear()

    def test_good_calls_are_silent_and_strict_does_not_raise(self):
        # Negative control for the lists above: a good call applies, with
        # or without STRICT, and says nothing.
        self.leds.responses.clear()
        for call in ('LED_MODE STRIP=left MODE=solid SAT=50',
                     'LED_MODE STRIP=right MODE=Rainbow STRICT=1',
                     'LED_MODE STRIP=BOTH SAT=0 STRICT=1',
                     'LED_MODE SAT=100.0'):
            with self.subTest(call=call):
                self.leds.run(call)
        self.assertEqual(self.leds.responses, [])
        self.assertEqual(self.leds.macro_vars('LED_MODE')['sat'], 100.0)

    def test_a_bad_mode_in_a_calling_macro_does_not_abort_it(self):
        import configparser
        fileconfig = configparser.RawConfigParser()
        section = 'gcode_macro TEST_PRINT_START'
        fileconfig.add_section(section)
        fileconfig.set(section, 'gcode',
                       'LED_MODE MODE=neon\nTEST_MARKER\n'
                       'LED_MODE STRIP=top\nTEST_MARKER')
        strict = 'gcode_macro TEST_PRINT_START_STRICT'
        fileconfig.add_section(strict)
        fileconfig.set(strict, 'gcode',
                       'LED_MODE MODE=neon STRICT=1\nTEST_MARKER')
        for sec in (section, strict):
            self.leds.printer.add_object(sec, gcode_macro.GCodeMacro(
                FakeConfig(self.leds.printer, fileconfig, sec)))
        self.leds.gcode.register_command('TEST_MARKER', self.leds._stub)
        self.leds.stubbed.clear()
        self.leds.run('TEST_PRINT_START')
        self.assertEqual(self.leds.commands().count('TEST_MARKER'), 2)
        self.assertEqual(self.leds.errors(), [])
        # Control: with STRICT=1 the caller is aborted before its marker.
        self.leds.stubbed.clear()
        self.expect_error('TEST_PRINT_START_STRICT')
        self.assertEqual(self.leds.commands().count('TEST_MARKER'), 0)


if __name__ == '__main__':
    unittest.main()
