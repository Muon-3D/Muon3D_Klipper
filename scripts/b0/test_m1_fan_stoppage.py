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
# Pass (1-based) of a fan dead from the start on which the routine first
# fires: pass 1 only records prev_speed, so CPU/MD judge from pass 2 and trip
# on pass 4; the PSU fan also waits warmup_passes (3), so it trips on pass 6.
CPU_FIRST = 4
MD_FIRST = 4
PSU_FIRST = 6


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

    def run_pass(self, macro, name):
        # FAN_STOPPAGE_ROUTINE is only recorded here; the real one pauses a
        # running print, so do the same when it fires.
        before = len(self.trips(name))
        self.m1.run(macro)
        if len(self.trips(name)) > before:
            self.m1.print_stats['state'] = 'paused'

    def passes_to_first_trip(self, macro, name, limit=20):
        # 1-based index of the pass on which the routine first fires
        for i in range(1, limit + 1):
            self.run_pass(macro, name)
            if self.trips(name):
                return i
        self.fail("%s never tripped in %d passes" % (macro, limit))

    def check_one(self, macro, counter, name, first_trip):
        status = self.fans[macro]
        stats = self.m1.print_stats
        stats['state'] = 'printing'
        status.update(speed=0.5, rpm=0.0)
        self.m1.timeline.clear()
        self.m1.responses.clear()
        # The pass index pins each fan's warm-up: the PSU fan has 3 extra
        # passes, so changing warmup_passes moves this and fails the test.
        self.assertEqual(self.passes_to_first_trip(macro, name), first_trip)
        for i in range(12):     # 36 s more of a dead fan at the 3 s cadence
            self.run_pass(macro, name)
        self.assertEqual(len(self.trips(name)), 1,
                         "%s tripped %d times" % (macro,
                                                  len(self.trips(name))))
        self.assertEqual(self.m1.macro_vars(macro)[counter], MAX_STOPS)
        self.assertEqual(len(self.warnings(macro)), MAX_STOPS,
                         self.m1.timeline)
        # The routine paused the print. Still dead and still paused: quiet.
        stats['state'] = 'paused'
        for i in range(12):
            self.m1.run(macro)
        self.assertEqual(len(self.trips(name)), 1)
        self.assertEqual(len(self.warnings(macro)), MAX_STOPS)
        # RESUME onto the still-dead fan: the next pass trips again, with no
        # new warning, and every pass while it runs on does the same.
        stats['state'] = 'printing'
        self.run_pass(macro, name)
        self.assertEqual(stats['state'], 'paused')
        self.assertEqual(len(self.trips(name)), 2)
        self.assertEqual(len(self.warnings(macro)), MAX_STOPS)
        self.assertEqual(self.m1.macro_vars(macro)[counter], MAX_STOPS)
        # Paused again (the routine's doing): quiet once more
        stats['state'] = 'paused'
        self.m1.run(macro)
        self.assertEqual(len(self.trips(name)), 2)
        # Healthy again: the counter re-arms, and a new episode trips again
        status['rpm'] = 4000.0
        self.m1.run(macro)
        self.assertEqual(self.m1.macro_vars(macro)[counter], 0)
        # A healthy fan in a running print never trips
        stats['state'] = 'printing'
        for i in range(5):
            self.m1.run(macro)
        self.assertEqual(len(self.trips(name)), 2)
        status['rpm'] = 0.0
        for i in range(2):
            self.run_pass(macro, name)
        self.assertEqual(len(self.trips(name)), 2)    # 2 of 3 passes
        self.run_pass(macro, name)
        self.assertEqual(len(self.trips(name)), 3)    # 3rd bad pass trips

    def test_cpu_fan_trips_once(self):
        self.check_one('CPU_FAN_CHECK', 'cpu_stop_count', 'CPU', CPU_FIRST)

    def test_motor_driver_fan_trips_once(self):
        self.check_one('MD_FAN_CHECK', 'md_stop_count', 'Motor Drivers',
                       MD_FIRST)

    def test_psu_fan_trips_once(self):
        self.check_one('PSU_FAN_CHECK', 'ps_stop_count', 'Power Supply',
                       PSU_FIRST)


if __name__ == '__main__':
    unittest.main()
