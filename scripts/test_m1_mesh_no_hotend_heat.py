#!/usr/bin/env python3
# KAN-1689: a bed mesh must never heat the hot end. Probing is by the load
# cell at the nozzle, which needs no warm nozzle (nothing in
# core/M1/macros asks for one; the hot end is only held at the anti-ooze
# temperature during PRINT_START so a nozzle that is over the bed does not
# drip). So:
#
# * the macros a standalone mesh runs, _BED_MESH_PLAN and _BED_MESH_ENSURE,
#   and the script the panel's Tools mesh sends (G28 STOW=0, then
#   BED_MESH_CALIBRATE) issue no hot-end heating command and leave the
#   hot-end target where it was;
# * PRINT_START is unchanged: it holds the hot end at the anti-ooze
#   temperature (never above it) while the mesh runs.
#
#   python3 scripts/test_m1_mesh_no_hotend_heat.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only. Reuses test_m1_heater_before_leds.py's
# M1, which stubs motion and heating; BED_MESH_CALIBRATE is recorded.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402

RECORDED = ('BED_MESH_CALIBRATE', 'BED_MESH_PROFILE', '_BED_MESH_RECORD',
            'SET_PRINT_STATS_INFO', 'M73', 'NOZZLE_WIPE_SMART',
            'FINETUNE_RESONANCE', 'ANTI_OOZE_NOZZLE_DOCK',
            '_SAVE_CONFIG_IF_PENDING', 'SET_GCODE_OFFSET')
# Anything that can raise the hot end's target or wait on it.
HEATING = ('M104', 'M109', 'SET_HEATER_TEMPERATURE', 'TEMPERATURE_WAIT',
           'MACHINE_HEAT_WAIT', 'PID_CALIBRATE', 'PID_CALIBRATE_BASE')
OOZE_TEMP = 135.0


def hotend_heating(timeline):
    """Timeline lines that heat, or wait on, the hot end."""
    out = []
    for line in timeline:
        words = line.split()
        name = words[0].upper()
        if name not in HEATING:
            continue
        if name == 'MACHINE_HEAT_WAIT' and 'HEATER=heater_bed' in words:
            continue
        if name in ('SET_HEATER_TEMPERATURE', 'TEMPERATURE_WAIT') and (
                'HEATER=heater_bed' in words
                or 'SENSOR=heater_bed' in words):
            continue
        out.append(line)
    return out


class _Base(_M1TestCase):
    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        self.m1.printer.lookup_object(
            'filament_switch_sensor filament_sensor_toolhead') \
            .get_status(0)['filament_detected'] = True
        # A cold hot end, as a user finds it when they open Tools.
        self.m1.extruder['target'] = 0.0
        self.m1.extruder['temperature'] = 25.0
        self.m1.timeline.clear()


class StandaloneMesh(_Base):
    def assert_cold(self, script):
        self.m1.timeline.clear()
        self.m1.run(script)
        self.assertIn('BED_MESH_CALIBRATE', self.m1.names(),
                      self.m1.timeline)
        self.assertEqual(hotend_heating(self.m1.timeline), [])
        self.assertEqual(self.m1.extruder['target'], 0.0)

    def test_panel_tools_mesh_script(self):
        # MuonUI's meshCalibScript: G28 STOW=0 when not homed, then the mesh.
        self.assert_cold('G28 STOW=0\nBED_MESH_CALIBRATE\n'
                         '_BED_MESH_RECORD BED=60\n_SAVE_CONFIG_IF_PENDING')

    def test_plan_and_ensure_whole_bed(self):
        self.assert_cold('_BED_MESH_PLAN BED=60\n_BED_MESH_ENSURE BED=60')

    def test_plan_and_ensure_region(self):
        region = 'MESH_MIN=40,40 MESH_MAX=120,120 PROBE_COUNT=4,4'
        self.assert_cold('_BED_MESH_PLAN BED=60 %s\n'
                         '_BED_MESH_ENSURE BED=60 %s ALGORITHM=bicubic'
                         % (region, region))

    def test_ensure_leaves_a_warm_hot_end_alone(self):
        # Whatever the hot end is at, the mesh neither raises nor drops it.
        self.m1.extruder['target'] = 80.0
        self.m1.run('_BED_MESH_PLAN BED=60\n_BED_MESH_ENSURE BED=60')
        self.assertEqual(hotend_heating(self.m1.timeline), [])
        self.assertEqual(self.m1.extruder['target'], 80.0)


class PrintStartUnchanged(_Base):
    def test_hot_end_held_at_ooze_temp_over_the_mesh(self):
        self.m1.run('PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA')
        names = self.m1.names()
        mesh = names.index('BED_MESH_CALIBRATE')
        # Every hot-end command before the mesh asks for the anti-ooze
        # temperature, and there is at least one.
        before = [line for line in self.m1.timeline[:mesh]
                  if line.split()[0].upper() == 'M104']
        self.assertTrue(before, self.m1.timeline)
        for line in before:
            self.assertEqual(float(line.split('S')[1]), OOZE_TEMP, line)
        self.assertEqual(hotend_heating(self.m1.timeline[:mesh]), before)
        # The print temperature is only asked for after the mesh.
        after = [line for line in self.m1.timeline[mesh:]
                 if line.split()[0].upper() == 'M104']
        self.assertTrue(any(float(l.split('S')[1]) > OOZE_TEMP
                            for l in after), after)


if __name__ == '__main__':
    unittest.main()
