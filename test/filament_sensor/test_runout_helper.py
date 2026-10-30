"""Host tests for the filament switch event policy.

The M1's toolhead switch uses ``insert_gcode`` to start automatic loading.
These tests drive the real RunoutHelper without an MCU and pin down the
arming, print-state, pause ordering, and debounce rules around that hook.
"""
import os
import sys
import unittest


KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import filament_switch_sensor  # noqa: E402


class FakeTemplate:
    def __init__(self, script):
        self.script = script

    def render(self):
        return self.script


class FakeMacro:
    def load_template(self, config, name, default=None):
        return FakeTemplate(config.values.get(name, default))


class FakeGCode:
    def __init__(self):
        self.scripts = []
        self.commands = []

    def register_mux_command(self, *args, **kwargs):
        self.commands.append(args)

    def run_script(self, script):
        self.scripts.append(script)


class FakeReactor:
    NEVER = 1.e30

    def __init__(self):
        self.now = 100.
        self.callbacks = []
        self.pauses = []

    def monotonic(self):
        return self.now

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def pause(self, waketime):
        self.pauses.append(waketime)
        self.now = waketime

    def run_callbacks(self):
        callbacks, self.callbacks = self.callbacks, []
        for callback in callbacks:
            callback(self.now)


class FakeIdleTimeout:
    def __init__(self):
        self.state = 'Ready'

    def get_status(self, eventtime):
        return {'state': self.state}


class FakePauseResume:
    def __init__(self, events):
        self.events = events

    def send_pause_command(self):
        self.events.append('pause')


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()
        self.idle = FakeIdleTimeout()
        self.events = []
        self.pause_resume = FakePauseResume(self.events)
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        return {'gcode': self.gcode, 'idle_timeout': self.idle,
                'pause_resume': self.pause_resume}[name]

    def load_object(self, config, name):
        if name == 'gcode_macro':
            return FakeMacro()
        if name == 'pause_resume':
            return self.pause_resume
        raise AssertionError(name)

    def register_event_handler(self, name, callback):
        self.handlers[name] = callback


class FakeConfig:
    def __init__(self, printer, **values):
        self.printer = printer
        self.values = values

    def get_name(self):
        return 'filament_switch_sensor filament_sensor_toolhead'

    def get_printer(self):
        return self.printer

    def get(self, name, default=None):
        return self.values.get(name, default)

    def getboolean(self, name, default=None):
        return bool(self.values.get(name, default))

    def getfloat(self, name, default=None, above=None, minval=None):
        return float(self.values.get(name, default))


def make_helper(**values):
    defaults = {'pause_on_runout': True, 'runout_gcode': 'RUNOUT_ACTION',
                'insert_gcode': 'INSERT_FILAMENT_SENSOR',
                'pause_delay': .5, 'event_delay': 3.}
    defaults.update(values)
    printer = FakePrinter()
    return printer, filament_switch_sensor.RunoutHelper(
        FakeConfig(printer, **defaults))


class RunoutHelperTest(unittest.TestCase):
    def test_idle_insert_runs_armed_hook_once_and_debounces(self):
        printer, helper = make_helper()
        helper._handle_ready()
        printer.reactor.now = 102.01

        helper.note_filament_present(102.01, True)
        self.assertEqual(1, len(printer.reactor.callbacks))
        self.assertEqual(printer.reactor.NEVER, helper.min_event_systime)
        printer.reactor.run_callbacks()
        self.assertEqual(['INSERT_FILAMENT_SENSOR\nM400'],
                         printer.gcode.scripts)
        self.assertEqual(105.01, helper.min_event_systime)

        # A bounce updates the switch state, but cannot schedule another load.
        helper.note_filament_present(103., False)
        helper.note_filament_present(103.01, True)
        self.assertEqual([], printer.reactor.callbacks)
        self.assertEqual(1, len(printer.gcode.scripts))

    def test_insert_is_suppressed_during_print(self):
        printer, helper = make_helper()
        helper.min_event_systime = 0.
        printer.idle.state = 'Printing'

        helper.note_filament_present(100., True)

        self.assertTrue(helper.filament_present)
        self.assertEqual([], printer.reactor.callbacks)
        self.assertEqual([], printer.gcode.scripts)

    def test_print_runout_pauses_before_running_script(self):
        printer, helper = make_helper()
        helper.min_event_systime = 0.
        helper.filament_present = True
        printer.idle.state = 'Printing'

        helper.note_filament_present(100., False)
        printer.reactor.run_callbacks()

        self.assertEqual(['pause'], printer.events)
        self.assertEqual([100.5], printer.reactor.pauses)
        self.assertEqual(['PAUSE\nRUNOUT_ACTION\nM400'],
                         printer.gcode.scripts)
        self.assertEqual(103.5, helper.min_event_systime)

    def test_disabled_sensor_tracks_state_without_actions(self):
        printer, helper = make_helper()
        helper.min_event_systime = 0.
        helper.sensor_enabled = False

        helper.note_filament_present(100., True)

        status = helper.get_status(100.)
        self.assertIs(True, status['filament_detected'])
        self.assertIs(False, status['enabled'])
        self.assertEqual([], printer.reactor.callbacks)

    def test_initialization_grace_blocks_insert(self):
        printer, helper = make_helper()
        helper._handle_ready()

        helper.note_filament_present(101.99, True)

        self.assertTrue(helper.filament_present)
        self.assertEqual([], printer.reactor.callbacks)


if __name__ == '__main__':
    unittest.main()
