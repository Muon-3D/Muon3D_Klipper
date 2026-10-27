#!/usr/bin/env python3
# KAN-990: the start purge goes where the bed mesh probed. The slicer's old
# purge ran along Y -1.5 to 4.5 at Z 0.15, outside every adaptive mesh, so
# the nozzle met the unlevelled front edge and contact_guard aborted the
# print. PRINT_START now plans a purge lane in front of (or behind) the
# slicer's region, off the bed notch, and grows the mesh region to take it
# in; PURGE_LINE draws along it. This is the dry run: PRINT_START and
# PURGE_LINE from the shipped print.cfg, through klippy's own GCodeDispatch
# and GCodeMacro, for the bench's part, the placements that move the lane,
# and a sweep of the whole bed. For every drawn purge it checks:
#
# * every move at purge height is inside the region handed to the mesh;
# * none touches the notch ([bed_mesh] no_go_region_1);
# * none crosses the region the print occupies;
#
# and that no lane means no purge, and a lane is drawn once.
#
#   python3 scripts/test_m1_purge_line.py   (or: pytest <this file>)
#
# Offline, no hardware, Linux only. Reuses test_m1_heater_before_leds.py's
# M1, which stubs motion and heating; the mesh macros and the rest of what
# PRINT_START calls are recorded here instead of run.
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402

# What PRINT_START calls that M1 doesn't stub. The mesh macros are recorded
# so the test can read the region they are handed.
RECORDED = ('_BED_MESH_PLAN', '_BED_MESH_ENSURE', '_HOTEND_RAMP_UNDER_MESH',
            'BED_MESH_PROFILE', 'SET_PRINT_STATS_INFO', 'M73', 'M140',
            'NOZZLE_WIPE_SMART', 'FINETUNE_RESONANCE',
            'IF_BED_TEMP_ANTI_OOZE_NOZZLE_DOCK', 'ANTI_OOZE_NOZZLE_DOCK',
            '_SAVE_CONFIG_IF_PENDING', 'SET_GCODE_OFFSET')

# core/M1/core.cfg [bed_mesh].
BED = (10.0, 7.0, 190.0, 170.0)
NOTCH = (75.0, 0.0, 125.0, 34.0)
# Below this the nozzle is purging, or about to.
PURGE_HEIGHT = 1.0
SKIPPED = 'PURGE_LINE: no purge lane was planned'


def params(line):
    return dict(word.split('=', 1) for word in line.split()[1:]
                if '=' in word)


def inside(point, box, eps=1e-6):
    x, y = point
    x0, y0, x1, y1 = box
    return x0 - eps <= x <= x1 + eps and y0 - eps <= y <= y1 + eps


def strictly_inside(point, box):
    x, y = point
    x0, y0, x1, y1 = box
    return x0 < x < x1 and y0 < y < y1


def along(start, end, step=0.25):
    (ax, ay), (bx, by) = start, end
    n = max(1, int(max(abs(bx - ax), abs(by - ay)) / step))
    return [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n)
            for i in range(n + 1)]


class PurgeLine(_M1TestCase):
    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        settings = self.m1.printer.lookup_object('configfile') \
            .get_status(0)['settings']['bed_mesh']
        settings['no_go_region_1_min'] = [NOTCH[0], NOTCH[1]]
        settings['no_go_region_1_max'] = [NOTCH[2], NOTCH[3]]

    def tearDown(self):
        self.m1.responses[:] = [r for r in self.m1.responses
                                if SKIPPED not in r]
        super().tearDown()

    def print_start(self, mesh_min=None, mesh_max=None):
        """Run PRINT_START; return the region both mesh macros got, or
        None when they got none."""
        self.m1.timeline.clear()
        script = 'PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA'
        if mesh_min is not None:
            script += ' MESH_MIN=%s,%s MESH_MAX=%s,%s PROBE_COUNT=4,4' % (
                mesh_min + mesh_max)
        self.m1.run(script)
        regions = []
        for line in self.m1.timeline:
            if line.split()[0] in ('_BED_MESH_PLAN', '_BED_MESH_ENSURE'):
                p = params(line)
                if 'MESH_MIN' not in p:
                    regions.append(None)
                    continue
                regions.append(tuple(
                    float(v) for v in (p['MESH_MIN'] + ',' + p['MESH_MAX'])
                    .split(',')))
        self.assertEqual(len(regions), 2, self.m1.timeline)
        self.assertEqual(regions[0], regions[1])
        return regions[0]

    def purge(self):
        """Run PURGE_LINE; return its moves as (start, end, z, e), with
        start None until the first XY position is known."""
        self.m1.timeline.clear()
        self.m1.responses.clear()
        self.m1.run('PURGE_LINE')
        pos = {'X': None, 'Y': None, 'Z': None}
        moves = []
        for line in self.m1.timeline:
            if line.split()[0] not in ('G0', 'G1'):
                continue
            p = dict((w[0], float(w[1:])) for w in line.split()[1:])
            start = (pos['X'], pos['Y']) if pos['X'] is not None else None
            for axis in 'XYZ':
                if axis in p:
                    pos[axis] = p[axis]
            moves.append((start, (pos['X'], pos['Y']), pos['Z'],
                          p.get('E', 0.0)))
        return moves

    def skipped(self):
        return any(SKIPPED in r for r in self.m1.responses)

    def assert_purge_probed(self, region, part):
        moves = self.purge()
        self.assertFalse(self.skipped(), self.m1.responses)
        low = [m for m in moves if m[2] is not None and m[2] < PURGE_HEIGHT]
        self.assertTrue(low, moves)
        # Nothing extrudes before the nozzle is down at purge height.
        for start, end, z, e in moves:
            if e > 0:
                self.assertIsNotNone(z)
                self.assertLess(z, PURGE_HEIGHT)
        pushed = sum(m[3] for m in low)
        self.assertGreater(pushed, 15.0, moves)
        for start, end, z, e in low:
            self.assertIsNotNone(start, moves)
            for point in along(start, end):
                self.assertTrue(inside(point, region),
                                '%s outside the probed region %s'
                                % (point, region))
                self.assertFalse(strictly_inside(point, NOTCH),
                                 '%s in the notch' % (point,))
                self.assertFalse(inside(point, part),
                                 '%s on the print %s' % (point, part))
        return low

    def lane(self):
        return self.m1.macro_vars('PURGE_LINE')['lane']

    def run_part(self, mesh_min, mesh_max):
        region = self.print_start(mesh_min, mesh_max)
        part = (max(mesh_min[0], BED[0]), max(mesh_min[1], BED[1]),
                min(mesh_max[0], BED[2]), min(mesh_max[1], BED[3]))
        return region, part

    # --- placements -------------------------------------------------------

    def test_centred_part(self):
        # The bench print: a 20 mm cube centred, plus the slicer's margin.
        region, part = self.run_part((87, 77), (113, 103))
        self.assertEqual(self.lane(), [87.0, 147.0, 71.0, 1])
        self.assertEqual(region, (87.0, 71.0, 147.0, 103.0))
        low = self.assert_purge_probed(region, part)
        # The long line is on the region's front row, which the mesh probes.
        self.assertTrue(any(start[1] == end[1] == 71.0
                            and abs(end[0] - start[0]) > 40
                            for start, end, z, e in low))

    def test_probe_counts_follow_the_grown_region(self):
        self.run_part((87, 77), (113, 103))
        plan = [line for line in self.m1.timeline
                if line.startswith('_BED_MESH_PLAN')][0]
        # 60 x 32 mm at PRINT_START's 37.9 mm step: 3 x 3, as before.
        self.assertEqual(params(plan)['PROBE_COUNT'], '3,3')

    def test_lane_steps_left_of_the_notch(self):
        # 6 mm in front of this part is y 34, the notch's back edge.
        region, part = self.run_part((60, 40), (100, 80))
        x0, x1, y, toward = self.lane()
        self.assertEqual(y, 34.0)
        self.assertLessEqual(x1, NOTCH[0])
        self.assert_purge_probed(region, part)

    def test_lane_steps_right_of_the_notch(self):
        region, part = self.run_part((110, 40), (150, 80))
        x0, x1, y, toward = self.lane()
        self.assertEqual(y, 34.0)
        self.assertGreaterEqual(x0, NOTCH[2])
        self.assertLessEqual(x1, BED[2])
        self.assert_purge_probed(region, part)

    def test_part_at_the_front_edge_purges_behind_it(self):
        region, part = self.run_part((20, 8), (60, 60))
        x0, x1, y, toward = self.lane()
        self.assertEqual((y, toward), (66.0, -1))
        self.assert_purge_probed(region, part)

    def test_part_off_the_bed_is_clamped_first(self):
        # Orca's adaptive bounds include its margin and can leave the bed.
        region, part = self.run_part((-5, -5), (40, 40))
        self.assertGreaterEqual(region[0], BED[0])
        self.assertGreaterEqual(region[1], BED[1])
        self.assert_purge_probed(region, part)

    def test_full_depth_part_has_no_lane(self):
        region, part = self.run_part((20, 7), (180, 170))
        self.assertIsNone(self.lane())
        # The region is the slicer's own, not grown.
        self.assertEqual(region, (20.0, 7.0, 180.0, 170.0))
        self.assertEqual(self.purge(), [])
        self.assertTrue(self.skipped())

    def test_no_room_beside_the_notch_has_no_lane(self):
        self.m1.run('SET_GCODE_VARIABLE MACRO=PURGE_LINE '
                    'VARIABLE=min_length VALUE=70.0')
        region, part = self.run_part((60, 40), (100, 80))
        self.assertIsNone(self.lane())
        self.assertEqual(region, (60.0, 40.0, 100.0, 80.0))
        self.assertEqual(self.purge(), [])

    def test_no_slicer_region_has_no_lane(self):
        self.assertIsNone(self.print_start())
        self.assertIsNone(self.lane())
        self.assertEqual(self.purge(), [])
        self.assertTrue(self.skipped())

    # --- lifetime ---------------------------------------------------------

    def test_a_lane_is_drawn_once(self):
        self.run_part((87, 77), (113, 103))
        self.assertTrue(self.purge())
        self.assertIsNone(self.lane())
        self.assertEqual(self.purge(), [])
        self.assertTrue(self.skipped())

    def test_a_print_without_a_region_drops_an_old_lane(self):
        self.run_part((87, 77), (113, 103))
        self.assertIsNotNone(self.lane())
        self.print_start()
        self.assertIsNone(self.lane())

    # --- the whole bed ----------------------------------------------------

    def test_sweep(self):
        drawn = skipped = 0
        for w in (10, 40, 100, 170):
            for h in (10, 40, 100, 160):
                for x in range(0, 200 - w + 1, 15):
                    for y in range(0, 175 - h + 1, 15):
                        with self.subTest(x=x, y=y, w=w, h=h):
                            region, part = self.run_part((x, y),
                                                         (x + w, y + h))
                            self.assertTrue(inside(region[:2], BED))
                            self.assertTrue(inside(region[2:], BED))
                            self.assertTrue(inside(part[:2], region))
                            self.assertTrue(inside(part[2:], region))
                            if self.lane() is None:
                                self.assertEqual(self.purge(), [])
                                skipped += 1
                            else:
                                self.assert_purge_probed(region, part)
                                drawn += 1
        # Only parts that fill the bed's depth go without a purge.
        self.assertGreater(drawn, 10 * skipped, (drawn, skipped))


if __name__ == '__main__':
    unittest.main()
