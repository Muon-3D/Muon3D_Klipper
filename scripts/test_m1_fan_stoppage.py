#!/usr/bin/env python3
# KAN-205: a bad CPU, motor-driver or PSU fan trips FAN_STOPPAGE_ROUTINE
# once per episode. Their stop counters were uncapped, so a fan that stayed
# bad ran on to (21/3), re-invoking the routine -- a pause and the hot end
# zeroed -- and its warning every 3 s. HOTEND_FAN_CHECK was already capped.
# This runs the shipped fan.cfg checks through klippy's GCodeDispatch and
# GCodeMacro (test_m1_heater_before_leds's M1) with the routine recorded.
#
#   python3 scripts/test_m1_fan_stoppage.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase, Status  # noqa: E402

# check macro, its counter, the printer object it reads, the routine's name
FANS = (
    ('CPU_FAN_CHECK', 'cpu_stop_count', 'temperature_fan CPU', 'CPU'),
    ('MD_FAN_CHECK', 'md_stop_count', 'temperature_fan Motor_Drivers',
     'Motor Drivers'),
    ('PSU_FAN_CHECK', 'ps_stop_count', 'power_fan Power_Supply',
     'Power Supply'),
)
MAX_STOPS = 3


class FanTripsOnce(_M1TestCase):
    def setUp(self):
        super().setUp()
        self.m1.gcode.register_command('FAN_STOPPAGE_ROUTINE', None)
        self.m1.gcode.register_command('FAN_STOPPAGE_ROUTINE',
                                       self.m1._record)
        self.fans = {}
        for macro, counter, obj, name in FANS:
            self.fans[macro] = status = {'speed': 0.5, 'rpm': 0.0}
            self.m1.printer.add_object(obj, Status(status))

    def trips(self, name):
        return [line for line in self.m1.timeline
                if line.startswith('FAN_STOPPAGE_ROUTINE')
                and name in line]

    def warnings(self, macro):
        word = {'CPU_FAN_CHECK': 'CPU fan', 'MD_FAN_CHECK': 'Motor-drivers',
                'PSU_FAN_CHECK': 'PSU fan'}[macro]
        # M118 is a recorded stub in M1, so the warnings are in the timeline
        return [line for line in self.m1.timeline
                if line.startswith('M118') and 'WARNING' in line
                and word in line]

    def check_one(self, macro, counter, name):
        status = self.fans[macro]
        status.update(speed=0.5, rpm=0.0)
        self.m1.timeline.clear()
        self.m1.responses.clear()
        for i in range(12):     # 36 s of a dead fan at the 3 s cadence
            self.m1.run(macro)
        self.assertEqual(len(self.trips(name)), 1,
                         "%s tripped %d times" % (macro,
                                                  len(self.trips(name))))
        self.assertEqual(self.m1.macro_vars(macro)[counter], MAX_STOPS)
        self.assertEqual(len(self.warnings(macro)), MAX_STOPS,
                         self.m1.timeline)
        # Healthy again: the counter re-arms, and a new episode trips again
        status['rpm'] = 4000.0
        self.m1.run(macro)
        self.assertEqual(self.m1.macro_vars(macro)[counter], 0)
        status['rpm'] = 0.0
        for i in range(12):
            self.m1.run(macro)
        self.assertEqual(len(self.trips(name)), 2)

    def test_cpu_fan_trips_once(self):
        self.check_one('CPU_FAN_CHECK', 'cpu_stop_count', 'CPU')

    def test_motor_driver_fan_trips_once(self):
        self.check_one('MD_FAN_CHECK', 'md_stop_count', 'Motor Drivers')

    def test_psu_fan_trips_once(self):
        self.check_one('PSU_FAN_CHECK', 'ps_stop_count', 'Power Supply')


if __name__ == '__main__':
    unittest.main()
