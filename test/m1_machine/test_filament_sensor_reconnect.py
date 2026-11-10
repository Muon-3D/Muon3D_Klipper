# Offline tests for a filament switch on a non-critical MCU (the M1 toolhead).
#
# Harry, 2026-11-05: the plug-in auto-load ran with no filament. Filament
# pulled out while the toolhead was unplugged stayed "detected": nothing read
# the pin while the MCU was away, and the reconnected MCU only reports the
# pin when it differs from the released state.
#
#   python3 test/m1_machine/test_filament_sensor_reconnect.py
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import filament_switch_sensor  # noqa: E402


class FakeReactor:
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 100.
        self.callbacks = []

    def monotonic(self):
        return self.now

    def register_callback(self, cb, waketime=None):
        self.callbacks.append((waketime or self.now, cb))

    def run_until(self, t):
        self.now = t
        while True:
            due = [c for c in self.callbacks if c[0] <= t]
            if not due:
                return
            for c in due:
                self.callbacks.remove(c)
                c[1](c[0])


class FakeMcu:
    is_non_critical = True

    def get_non_critical_disconnect_event_name(self):
        return 'toolhead:disconnect'

    def get_non_critical_reconnect_event_name(self):
        return 'toolhead:reconnect'


class FakePins:
    def __init__(self, mcu):
        self.mcu = mcu

    def parse_pin(self, desc, can_invert=False, can_pullup=False):
        return {'chip': self.mcu}


class FakeButtons:
    def register_buttons(self, pins, callback):
        self.callback = callback


class FakeGcode:
    def __init__(self):
        self.scripts = []

    def register_mux_command(self, *a, **kw):
        pass

    def run_script(self, script):
        self.scripts.append(script)


class FakeTemplate:
    def __init__(self, text):
        self.text = text

    def render(self):
        return self.text


class FakeMacro:
    def load_template(self, config, name, default=None):
        return FakeTemplate(config.values[name])


class FakeM1:
    printing = False

    def file_printing(self, eventtime):
        return self.printing


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.handlers = {}
        self.buttons = FakeButtons()
        self.gcode = FakeGcode()
        self.objects = {'gcode': self.gcode,
                        'pins': FakePins(FakeMcu()),
                        'm1_machine': FakeM1()}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def load_object(self, config, name):
        return {'buttons': self.buttons,
                'gcode_macro': FakeMacro()}.get(name)

    def register_event_handler(self, name, cb):
        self.handlers.setdefault(name, []).append(cb)

    def send_event(self, name):
        for cb in self.handlers.get(name, []):
            cb()


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer
        self.values = {'switch_pin': '!toolhead:gpio5',
                       'insert_gcode': 'INSERT', 'runout_gcode': 'RUNOUT',
                       'debounce_delay': 0.2}

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'filament_switch_sensor filament_sensor_toolhead'

    def get(self, name, default=None):
        return self.values.get(name, default)

    def getboolean(self, name, default=None):
        return False if name == 'pause_on_runout' else default

    def getfloat(self, name, default=None, **kw):
        return self.values.get(name, default)


class Sensor:
    def __init__(self):
        self.printer = FakePrinter()
        self.sensor = filament_switch_sensor.SwitchSensor(
            FakeConfig(self.printer))
        self.reactor = self.printer.reactor
        self.printer.send_event('klippy:ready')
        self.reactor.run_until(self.reactor.now + 3.)

    def pin(self, state):
        """The MCU reports the (inverted) switch state."""
        self.printer.buttons.callback(self.reactor.now, state)
        self.reactor.run_until(self.reactor.now + 0.3)
        self.reactor.run_until(self.reactor.now + 0.01)

    def detected(self):
        return self.sensor.get_status(0)['filament_detected']

    def scripts(self):
        return self.printer.gcode.scripts


class TestReconnect(unittest.TestCase):
    def test_an_insert_on_a_connected_toolhead_still_fires(self):
        s = Sensor()
        s.pin(1)
        self.assertTrue(s.detected())
        self.assertEqual(len(s.scripts()), 1)
        self.assertIn('INSERT', s.scripts()[0])

    def test_filament_pulled_while_unplugged_reads_absent_after(self):
        s = Sensor()
        s.pin(1)
        s.printer.send_event('toolhead:disconnect')
        self.assertFalse(s.detected())
        s.printer.send_event('toolhead:reconnect')
        # The MCU sends nothing: the pin is at its released state.
        s.reactor.run_until(s.reactor.now + 2.)
        self.assertFalse(s.detected())

    def test_the_drop_itself_fires_nothing(self):
        # No runout, no insert: nobody touched the filament.
        s = Sensor()
        s.pin(1)
        del s.scripts()[:]
        s.printer.send_event('toolhead:disconnect')
        s.reactor.run_until(s.reactor.now + 2.)
        self.assertEqual(s.scripts(), [])

    def test_plugged_in_with_filament_reads_present_without_an_insert(self):
        # m1_machine's plug-in auto-load does the loading, so the sensor's
        # own insert must not run as well.
        s = Sensor()
        s.printer.send_event('toolhead:disconnect')
        s.printer.send_event('toolhead:reconnect')
        s.pin(1)
        self.assertTrue(s.detected())
        self.assertEqual(s.scripts(), [])

    def test_an_insert_after_it_settles_fires(self):
        s = Sensor()
        s.printer.send_event('toolhead:disconnect')
        s.printer.send_event('toolhead:reconnect')
        s.reactor.run_until(s.reactor.now + 2.)
        s.pin(1)
        self.assertTrue(s.detected())
        self.assertEqual(len(s.scripts()), 1)


if __name__ == '__main__':
    unittest.main()
