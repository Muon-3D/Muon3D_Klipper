"""KAN-468: a no-go snapped point keeps the mesh on its grid.

bed_mesh snaps a generated point that falls inside a no_go_region to the
nearest in-bounds edge of the region.  It used to do that by overwriting the
point in base_points, which everything that finds rows walks, so a snap in y
split its row and BED_MESH_CALIBRATE failed with "Invalid y-axis table
length".  The grid now stays in base_points and only the probe goes to the
snapped coordinate.

test/klippy/bed_mesh_no_go_snap.test covers the standard probe path end to
end.  The rapid-scan path only runs with an eddy probe, which the simulator
cannot place, so it is checked here against the real ProbeManager:

* base_points is the grid, the standard path is snapped
* the rapid-scan lead-in is on the snapped point's row, so the first sample
  is approached straight along x, not diagonally from the grid row

    python3 test/bed_mesh/test_no_go_snap.py
"""
import collections
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import bed_mesh  # noqa: E402

# The M1's notch (core/M1/core.cfg [bed_mesh]).
NOTCH = ([75., 0.], [125., 34.])
TOLERANCE = 10.
OVERSHOOT = 4.


class FakeProbeHelper:
    def __init__(self):
        self.points = None

    def update_probe_points(self, points, min_points):
        self.points = points


def make_probe_manager():
    # Only the state generate_points() and the path iterators read; __init__
    # needs a printer and a config to build its probe helpers.
    pm = bed_mesh.ProbeManager.__new__(bed_mesh.ProbeManager)
    pm.zero_ref_pos = None
    pm.zref_mode = bed_mesh.ZrefMode.DISABLED
    pm.no_go_regions = [NOTCH]
    pm.no_go_boundary_tolerance = TOLERANCE
    pm.faulty_regions = []
    pm.cfg_overshoot = OVERSHOOT
    pm.overshoot = OVERSHOOT
    pm.orig_config = {"mesh_min": (10., 7.), "mesh_max": (190., 170.),
                      "radius": None, "origin": None}
    pm.base_points = []
    pm.canceled_points = set()
    pm.adjusted_points = {}
    pm.substitutes = collections.OrderedDict()
    pm.is_round = False
    pm.probe_helper = FakeProbeHelper()
    return pm


class NoGoSnapTest(unittest.TestCase):
    def setUp(self):
        # 4 x 3 from (100, 26) to (190, 100): a 30 x 37 mm grid. The first
        # point is inside the notch, 8 mm under its top edge and 25 mm or
        # more from every other edge, so it is snapped up to (100, 34).
        self.pm = make_probe_manager()
        self.pm.generate_points({"x_count": 4, "y_count": 3},
                                (100., 26.), (190., 100.), None, None)

    def test_grid_is_kept_and_probe_is_snapped(self):
        self.assertEqual(self.pm.base_points[0], (100., 26.))
        self.assertEqual(self.pm.adjusted_points, {0: (100., 34.)})
        self.assertEqual(self.pm.canceled_points, set())
        path = self.pm.get_std_path()
        self.assertEqual(path[0], (100., 34.))
        self.assertEqual(path[1:4], [(130., 26.), (160., 26.), (190., 26.)])
        self.assertEqual(self.pm.probe_helper.points, path)

    def test_rapid_lead_in_is_on_the_snapped_row(self):
        path = list(self.pm.iter_rapid_path())
        (lead_in, lead_sampled), (first, first_sampled) = path[0], path[1]
        self.assertFalse(lead_sampled)
        self.assertTrue(first_sampled)
        self.assertEqual(first, (100., 34.))
        self.assertEqual(lead_in, (100. - OVERSHOOT, 34.))


if __name__ == '__main__':
    unittest.main()
