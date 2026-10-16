#!/usr/bin/env python3
# Behaviour tests for the M1 LED state macros and the user LED mode
# (KAN-480): core/M1/macros/led.cfg, led_modes.cfg, and the LED call in
# print.cfg's PRINT_END.
#
#   python3 scripts/test_m1_led_modes.py      (or: pytest <this file>)
#
# Offline, no hardware, Linux only (klippy imports pty and fcntl). The
# shipped cfg files are read by klippy's own ConfigFileReader, and the
# macros run through klippy's own GCodeDispatch, GCodeMacro, DelayedGcode
# and SaveVariables. That means Klipper's single-brace Jinja environment,
# its render-then-run order, macro recursion limits and literal_eval'd
# variables are the real ones, not a copy. The reactor is a fake whose
# clock the test moves, so delayed_gcode timers fire on demand. Only the
# led_effect objects (enabled flag and REPLACE overlap, nothing rendered
# to a pixel) and the motion and heater commands PRINT_END emits are
# stubbed.
import contextlib
import logging
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MACROS = os.path.join(ROOT, 'core', 'M1', 'macros')
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

import configfile  # noqa: E402
import gcode  # noqa: E402
from extras import delayed_gcode, gcode_macro, save_variables  # noqa: E402

# The files under test, as core/M1/macros/macros.cfg includes them.
INCLUDES = ('led.cfg', 'led_modes.cfg', 'print.cfg')

# Commands PRINT_END sends that are not defined by the files above.
# Recorded, otherwise ignored. Anything else unknown is an error.
STUB_COMMANDS = ('SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE', 'M400', 'G92',
                 'G1', 'G0', 'G90', 'G91', 'M107', 'M84', 'TURN_OFF_HEATERS',
                 'CUT_FILAMENT')

# The M1 neopixel chains are 40 LEDs each. A bare chain name in a
# led_effect's leds: covers all of them.
CHAIN_LENGTH = 40

PROGRESS = {'progress_left', 'progress_left_mirror', 'progress_right',
            'progress_right_mirror'}
SETUP = {'setup_left', 'setup_left_mirror', 'setup_right',
         'setup_right_mirror'}

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

    def __init__(self, config, effects, macro):
        self.name = config.get_name().split()[1]
        self.effects = effects
        self.auto_start = config.getboolean('autostart', False)
        self.recalculate = config.get('recalculate', False)
        self.leds = _parse_leds(config.get('leds'))
        self.template = macro.load_template(config, 'layers')
        self.enabled = False
        self.layers = None
        self.params = None
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

    def __init__(self, variables_file):
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
            'homed_axes': 'xyz', 'estimated_print_time': 0.}))
        printer.add_object('extruder', Status({
            'can_extrude': True, 'target': 0., 'temperature': 215.}))

        self.stubbed = []
        for name in STUB_COMMANDS:
            self.gcode.register_command(name, self._stub)
        self.gcode.register_command('STOP_LED_EFFECTS',
                                    self.cmd_STOP_LED_EFFECTS)

        top = os.path.join(MACROS, '_test_m1_led_modes.cfg')
        data = ''.join('[include %s]\n' % (name,) for name in INCLUDES)
        reader = configfile.ConfigFileReader()
        fileconfig = reader.build_fileconfig_with_includes(data, top)
        self.effects = []
        for section in fileconfig.sections():
            config = FakeConfig(printer, fileconfig, section)
            kind, _, name = section.partition(' ')
            if kind == 'gcode_macro':
                if (fileconfig.has_option(section, 'rename_existing')
                        or name.upper() in STUB_COMMANDS):
                    continue
                obj = gcode_macro.GCodeMacro(config)
            elif kind == 'delayed_gcode':
                obj = delayed_gcode.DelayedGcode(config)
            elif kind == 'led_effect':
                obj = FakeLedEffect(config, self.effects, self.macro)
            else:
                continue
            printer.add_object(section, obj)
        printer.send_event('klippy:ready')
        self.responses.clear()

    def _stub(self, gcmd):
        self.stubbed.append(gcmd.get_commandline())

    def cmd_STOP_LED_EFFECTS(self, gcmd):
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
        """SET_LED_EFFECT's params on the last muon_mode render."""
        return self.printer.lookup_object('led_effect muon_mode').params

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
        self.leds.run('PRINT_END')
        self.assertIn('TURN_OFF_HEATERS', self.leds.stubbed)


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
        # DelayedGcode logs and swallows a template that raises, with no
        # '!!' response, so tearDown checks the log as well.
        self.leds.run('SAVE_VARIABLE VARIABLE=muon_led VALUE="{\'mode\': 1}"')
        self.leds.run('UPDATE_DELAYED_GCODE ID=_LED_MODE_RESTORE DURATION=1')
        self.leds.wait(2)
        records, self.log.records = self.log.records, []
        self.assertTrue(any('_LED_MODE_RESTORE' in r for r in records))


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
    """Finding 1: LED_MODE during a status effect saves, and defers."""

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
                self.assertEqual(self.leds.saved()['muon_led']['mode'], mode)

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
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.assertEqual(self.leds.macro_vars('LED_MODE')['mode'], 'solid')

    def test_idle_mode_change_shows_now(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), {'muon_mode'})
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
    """Finding 2: the selected mode comes back after a print."""

    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE MODE=gradient H=30 H2=210')
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.leds.run('LED_SETUP_PROGRESS')
        self.leds.run('LED_PRINTING')

    def test_complete_then_the_selected_mode(self):
        self.print_end()
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(30)
        self.assertEqual(self.leds.lit(), {'complete'})
        self.leds.wait(600)
        self.assertEqual(self.leds.lit(), {'muon_mode'})
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
        self.assertEqual(self.leds.lit(), {'muon_mode'})
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
    """Finding 3: LED_READY records READY whatever the idle look is."""

    def test_status_after_custom_ready_does_not_resume_info(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.run('LED_INFO')
        self.leds.run('LED_READY')
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.assertEqual(self.leds.laststate(), 'LED_READY')
        self.leds.run('LED_MODE MODE=status')
        self.assertEqual(self.leds.lit(), {'ready'})
        self.assertEqual(self.leds.laststate(), 'LED_READY')

    def test_timeout_set_resumes_the_custom_mode_not_complete(self):
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.run('LED_COMPLETE')
        self.leds.run('LED_READY')
        self.leds.run('LED_TIMEOUT_SET M=-1')
        self.assertEqual(self.leds.lit(), {'muon_mode'})


class CustomModeTimeout(LedTestCase):
    """Finding 4: the LED timeout applies to a custom idle mode."""

    def setUp(self):
        super().setUp()
        self.leds.run('LED_MODE MODE=solid H=30')
        self.assertEqual(self.leds.lit(), {'muon_mode'})

    def test_timeout_zero_turns_it_off(self):
        self.leds.run('LED_TIMEOUT_SET M=0')
        self.assertEqual(self.leds.lit(), set())

    def test_positive_timeout_turns_it_off_later(self):
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.leds.wait(119)
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.leds.wait(2)
        self.assertEqual(self.leds.lit(), set())

    def test_showing_the_mode_restarts_the_timer(self):
        self.leds.run('LED_MODE MODE=status')
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.leds.wait(100)
        self.leds.run('LED_MODE MODE=solid H=30')
        self.leds.wait(30)
        self.assertEqual(self.leds.lit(), {'muon_mode'})
        self.leds.wait(91)
        self.assertEqual(self.leds.lit(), set())

    def test_no_timeout_cancels_a_pending_one(self):
        self.leds.run('LED_TIMEOUT_SET M=2')
        self.leds.run('LED_TIMEOUT_SET M=-1')
        self.leds.wait(3600)
        self.assertEqual(self.leds.lit(), {'muon_mode'})


if __name__ == '__main__':
    unittest.main()
