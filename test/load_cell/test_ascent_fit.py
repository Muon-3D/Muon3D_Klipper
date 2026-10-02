#!/usr/bin/env python3
"""Regression for raw-count M1 probes without gram calibration.

Uses real load-cell conversion and ascent-fit code. Only the physical MCU,
stepper-history lookup and printer error interface are substituted.
"""
import math
import os
import sys
import types
import unittest
from unittest import mock

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))
from extras import load_cell, load_cell_probe


class CommandError(Exception):
    pass


class AscentFitTest(unittest.TestCase):
    def setUp(self):
        self.printer = types.SimpleNamespace(command_error=CommandError)
        self.tap = object.__new__(load_cell_probe.TappingMove)
        self.tap._printer = self.printer
        self.tap._best_fit = load_cell_probe.LCBestFit(self.printer)
        self.tap._load_cell_probing_move = types.SimpleNamespace(
            _mcu=types.SimpleNamespace(is_fileoutput=lambda: False))
        # Hand-derived contact at Z=0.12 with six free-air samples above it.
        self.z = [0., .02, .04, .06, .08, .10, .12,
                  .14, .16, .18, .20, .22, .24]
        self.raw = [-1080000, -1100000, -1120000, -1140000,
                    -1160000, -1180000, -1200000, -1200000,
                    -1200000, -1200000, -1200000, -1200000, -1200000]
        self.times = [1. + i * .01 for i in range(len(self.z))]
        self.positions = dict(zip(self.times, self.z))

    def samples(self, calibrated=False, raw=None):
        cell = object.__new__(load_cell.LoadCell)
        cell.counts_per_gram = 1500. if calibrated else None
        cell.reference_tare_counts = -1200000 if calibrated else None
        cell.tare_counts = -1200000
        cell.invert = 1.
        captured = []
        cell.clients = types.SimpleNamespace(send=captured.append)
        data = list(zip(self.times, self.raw if raw is None else raw))
        cell._sensor_data_event({'data': data, 'errors': 0, 'overflows': 0})
        return captured[0]['data']

    def fit(self, samples):
        with mock.patch.object(load_cell_probe, '_lookup_z_pos',
                               side_effect=lambda toolhead, t:
                               self.positions[t]):
            return self.tap._analyze_ascent(None, samples, 1., None, .1)

    def test_calibrated_grams_contact_is_unchanged(self):
        self.assertAlmostEqual(self.fit(self.samples(True)), .12, places=4)

    def test_uncalibrated_preloaded_counts_fit_contact(self):
        self.assertAlmostEqual(self.fit(self.samples()), .12, places=4)

    def test_raw_fit_handles_opposite_polarity_and_large_offset(self):
        raw = [8000000, 7980000, 7960000, 7940000, 7920000,
               7900000, 7880000, 7880000, 7880000, 7880000,
               7880000, 7880000, 7880000]
        for values in (raw, [-v for v in raw]):
            with self.subTest(polarity=values[0]):
                self.assertAlmostEqual(self.fit(self.samples(raw=values)),
                                       .12, places=4)

    def test_calibration_changed_mid_window_is_command_error(self):
        samples = self.samples()
        samples[4][1] = 26.6666666667
        with self.assertRaises(CommandError):
            self.fit(samples)

    def test_invalid_raw_values_are_command_errors(self):
        for bad in (None, math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad):
                samples = self.samples()
                samples[4][2] = bad
                with self.assertRaises(CommandError):
                    self.fit(samples)

    def test_invalid_gram_values_are_command_errors(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad):
                samples = self.samples(True)
                samples[4][1] = bad
                with self.assertRaises(CommandError):
                    self.fit(samples)

    def test_insufficient_window_still_refuses_fit(self):
        with self.assertRaises(CommandError):
            self.fit(self.samples()[:5])

    def test_invalid_stepper_history_is_command_error(self):
        for bad in (None, math.nan, math.inf):
            with self.subTest(bad=bad):
                self.positions[self.times[4]] = bad
                with self.assertRaises(CommandError):
                    self.fit(self.samples())

    def test_missing_values_outside_ascent_window_do_not_change_fit(self):
        samples = self.samples(True)
        samples.insert(0, [.9, None, None, None])
        samples.append([1.31, None, None, None])
        self.assertAlmostEqual(self.fit(samples), .12, places=4)


if __name__ == '__main__':
    unittest.main()
