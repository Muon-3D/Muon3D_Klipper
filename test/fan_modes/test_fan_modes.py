"""fan_modes: the command -> duty mapping, with and without a speed_table.

test/klippy/fan_modes*.test drive the module inside klippy, and test_klippy
reports only whether klippy exited with an error. This drives the real
FanModes with fakes and asserts the numbers: the duty each command gives,
the floor of the speed table, and the configs that must be refused.

    python3 test/fan_modes/test_fan_modes.py
"""
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import fan_modes  # noqa: E402

# The B1 values from core/M1/core.cfg, written out: they are what is tested.
TABLE = ("3408:0.0824, 5577:0.0980, 7729:0.1176, 9436:0.1373,"
         " 10906:0.1569, 12274:0.1765, 13680:0.2000, 14900:0.2235,"
         " 16181:0.2510, 17210:0.2745, 18277:0.3020, 23600:0.5020,"
         " 28400:0.7490, 31500:1.0")
B1 = {
    'modes': 'normal, quiet, turbo',
    'speed_table': TABLE,
    'normal_curve': '0:0, 0.01:3408, 1:18277',
    'quiet_curve': '0:0, 0.01:3408, 1:13680',
    'turbo_curve': '0:0, 0.01:3408, 1:31500',
}
DUTY = {
    'modes': 'normal, quiet, turbo',
    'normal_curve': '0:0, 1:0.30',
    'quiet_curve': '0:0, 0.5:0.05, 1:0.20',
    'turbo_curve': '0:0, 1:1.0',
}


class ConfigError(Exception):
    pass


class FakeFan:
    def __init__(self):
        self.speed = 0.
        self.set = []
    def set_speed_from_command(self, value):
        self.speed = value
        self.set.append(value)
    def get_status(self, eventtime):
        return {'speed': self.speed}


class FakePrinterFan:
    def __init__(self):
        self.fan = FakeFan()


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.scripts = []
    def register_command(self, name, func, desc=None):
        self.commands[name] = func
    def run_script_from_command(self, script):
        self.scripts.append(script)


class FakeReactor:
    def monotonic(self):
        return 0.


class FakePrinter:
    def __init__(self):
        self.gcode = FakeGcode()
        self.objects = {'gcode': self.gcode, 'fan': FakePrinterFan()}
        self.handlers = {}
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, event, cb):
        self.handlers[event] = cb
    def get_reactor(self):
        return FakeReactor()
    def config_error(self, msg):
        return ConfigError(msg)


class FakeConfig:
    error = ConfigError
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_printer(self):
        return self.printer
    def get(self, name, default=None):
        return self.values.get(name, default)
    def getlist(self, name, default=None):
        if name not in self.values:
            return default
        return [v.strip() for v in self.values[name].split(',')]
    def getfloat(self, name, default=None, above=None):
        return float(self.values[name]) if name in self.values else default


class FakeGcmd:
    def __init__(self, params):
        self.params = params
        self.info = []
    def get_float(self, name, default=None, minval=None, maxval=None):
        return float(self.params.get(name, default))
    def get(self, name, default=None):
        return self.params.get(name, default)
    def respond_info(self, msg):
        self.info.append(msg)
    def error(self, msg):
        return ConfigError(msg)


def build(values):
    printer = FakePrinter()
    fm = fan_modes.load_config(FakeConfig(printer, dict(values)))
    printer.handlers['klippy:connect']()
    return fm, printer


class TestSpeedTable(unittest.TestCase):
    def setUp(self):
        self.fm, self.printer = build(B1)
        self.fan = self.printer.objects['fan'].fan

    def part_fan(self, speed):
        self.printer.gcode.commands['SET_PART_FAN'](
            FakeGcmd({'SPEED': speed}))
        return self.fan.speed

    def test_zero_is_off(self):
        self.assertEqual(self.part_fan(0.), 0.)
        self.assertEqual(self.fm.target_rpm, 0.)

    def test_one_percent_starts_the_blower(self):
        # The point of ADR 0056: a small command gives air, not 0.3% duty.
        self.assertAlmostEqual(self.part_fan(0.01), 0.0824, places=4)
        self.assertAlmostEqual(self.fm.target_rpm, 3408.)

    def test_below_one_percent_is_held_at_the_table_floor(self):
        self.assertAlmostEqual(self.part_fan(0.002), 0.0824, places=4)
        self.assertLess(self.fm.target_rpm, 3408.)

    def test_mode_tops_keep_the_old_ceilings(self):
        for mode, duty in (('normal', 0.302), ('quiet', 0.200),
                           ('turbo', 1.0)):
            self.assertAlmostEqual(self.fm.map_duty(1., mode), duty,
                                   places=3, msg=mode)

    def test_table_points_are_hit_exactly(self):
        for rpm, duty in ((7729., 0.1176), (13680., 0.2), (23600., 0.502)):
            self.assertAlmostEqual(self.fm.duty_for_rpm(rpm), duty, places=4)

    def test_rpm_is_linear_in_the_command(self):
        # normal: 1% -> 3408, 100% -> 18277, so 50.5% is the midpoint.
        self.part_fan(0.505)
        self.assertAlmostEqual(self.fm.target_rpm, (3408. + 18277.) / 2.,
                               places=3)

    def test_duty_increases_with_the_command(self):
        last = -1.
        for i in range(0, 101):
            duty = self.fm.map_duty(i / 100.)
            self.assertGreaterEqual(duty, last)
            last = duty

    def test_mode_change_reapplies_and_reports_rpm(self):
        self.part_fan(1.)
        gcmd = FakeGcmd({'MODE': 'turbo'})
        self.printer.gcode.commands['SET_PRINT_MODE'](gcmd)
        self.assertAlmostEqual(self.fan.speed, 1.0)
        self.assertIn('31500 rpm', gcmd.info[0])

    def test_status_carries_target_rpm(self):
        self.part_fan(1.)
        status = self.fm.get_status(0.)
        self.assertAlmostEqual(status['target_rpm'], 18277.)
        self.assertAlmostEqual(status['output_speed'], 0.302)


class TestDutyCurvesUnchanged(unittest.TestCase):
    def test_duty_curve_without_a_table(self):
        fm, printer = build(DUTY)
        self.assertAlmostEqual(fm.map_duty(1.), 0.30)
        self.assertAlmostEqual(fm.map_duty(0.5, 'quiet'), 0.05)
        self.assertAlmostEqual(fm.map_duty(0.75, 'quiet'), 0.125)
        self.assertIsNone(fm.get_status(0.)['target_rpm'])


class TestRefusedConfigs(unittest.TestCase):
    def refused(self, **changes):
        values = dict(B1)
        values.update(changes)
        with self.assertRaises(ConfigError):
            build(values)

    def test_curve_above_the_table_top(self):
        self.refused(turbo_curve='0:0, 1:40000')

    def test_rpm_curve_without_a_table(self):
        values = dict(B1)
        del values['speed_table']
        with self.assertRaises(ConfigError):
            build(values)

    def test_table_not_increasing(self):
        self.refused(speed_table='3408:0.08, 3000:0.10')
        self.refused(speed_table='3408:0.08, 5000:0.07')

    def test_table_with_one_point(self):
        self.refused(speed_table='3408:0.08')

    def test_table_duty_out_of_range(self):
        self.refused(speed_table='3408:0.08, 31500:1.2')

    def test_table_entry_malformed(self):
        self.refused(speed_table='3408:0.08, 31500')


if __name__ == '__main__':
    unittest.main()
