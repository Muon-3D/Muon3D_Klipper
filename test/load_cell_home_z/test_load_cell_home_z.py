# Offline tests for klippy/extras/load_cell_home_z.py: a false first touch
# (the confirming tap finds no bed) is retried from the top, not fatal.
import os, sys, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'klippy'))
from extras import load_cell_home_z  # noqa: E402


class CommandError(Exception):
    pass


Z_TOP, Z_MIN = 160., -4.


class Machine:
    """A Z axis over a bed at real height 0. `z` is the commanded position,
    `off` the commanded minus the real height. `touches` lists heights (real)
    where the next fast approaches stop early, as a false touch."""
    def __init__(self, real_start=150., touches=()):
        self.real = real_start
        self.z = 0.
        self.touches = list(touches)
        self.relative = False
        self.last_probe_z = None
        self.log = []
        self.dwells = []

    def off(self):
        return self.z - self.real

    def probe(self, fast):
        floor = self.real - (self.z - Z_MIN)   # lowest real height reachable
        if fast and self.touches:
            t = self.touches.pop(0)
            if t <= self.real:
                self.z -= self.real - t
                self.real = t
                self.last_probe_z = self.z
                return
        if floor > 0.:
            self.z = Z_MIN
            self.real = floor
            raise CommandError("No trigger on probe after full movement")
        self.z -= self.real
        self.real = 0.
        self.last_probe_z = self.z

    def run(self, script):
        for line in script.split('\n'):
            self.log.append(line)
            w = line.split()
            if not w:
                continue
            if w[0] == 'SET_KINEMATIC_POSITION':
                self.z = float(w[1].split('=')[1])
            elif w[0] == 'G4':
                self.dwells.append(float(w[1][1:]) / 1000.)
            elif w[0] == 'G91':
                self.relative = True
            elif w[0] == 'G1':
                dz = float(w[1][1:])
                assert self.relative
                self.z += dz
                self.real += dz
            elif w[0] == 'PROBE':
                self.probe('PROBE_SPEED=15.000' in line)
            elif w[0] == '_HOME_Z_FROM_LAST_PROBE':
                self.z -= self.last_probe_z
            elif w[0] == 'RESTORE_GCODE_STATE':
                self.relative = False


class FakeGcode:
    error = CommandError
    def __init__(self, m):
        self.m = m
        self.cmds = {}
    def register_command(self, name, fn, desc=None):
        self.cmds[name] = fn
    def run_script_from_command(self, script):
        self.m.run(script)


class FakeToolhead:
    def __init__(self, m):
        self.m = m
    def get_status(self, et):
        return {'axis_maximum': [200., 174., Z_TOP],
                'axis_minimum': [0., -10., Z_MIN]}
    def wait_moves(self):
        pass


class FakeReactor:
    def monotonic(self):
        return 0.


class FakePrinter:
    def __init__(self, m):
        self.objs = {'gcode': FakeGcode(m), 'toolhead': FakeToolhead(m)}
    def lookup_object(self, name):
        return self.objs[name]
    def get_reactor(self):
        return FakeReactor()


class FakeConfig:
    def __init__(self, printer, **opts):
        self.p, self.opts = printer, opts
    def get_printer(self):
        return self.p
    def getint(self, k, d, **kw):
        return self.opts.get(k, d)
    def getfloat(self, k, d, **kw):
        return self.opts.get(k, d)


class FakeGcmd:
    error = CommandError
    def __init__(self, **params):
        self.params = params
        self.info = []
    def get_float(self, k, d, **kw):
        return self.params.get(k, d)
    def respond_info(self, msg):
        self.info.append(msg)


def make(m, **opts):
    p = FakePrinter(m)
    obj = load_cell_home_z.load_config(FakeConfig(p, **opts))
    return obj, p.objs['gcode'].cmds['LOAD_CELL_HOME_Z']


class Test(unittest.TestCase):
    def check_homed(self, m):
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertAlmostEqual(m.real, 10.)
        self.assertFalse(m.relative)

    def test_clean_home(self):
        m = Machine()
        obj, cmd = make(m)
        g = FakeGcmd()
        cmd(g)
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 0)
        self.assertEqual(m.dwells, [])
        self.assertEqual(g.info, [])

    def test_false_touch_is_retried(self):
        # The sloe case: a touch ~159 mm up, the bed far below.
        m = Machine(real_start=159.5, touches=[159.])
        obj, cmd = make(m)
        g = FakeGcmd()
        cmd(g)
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 1)
        self.assertEqual(obj.last_attempts, 2)
        self.assertEqual(m.dwells, [2.])
        self.assertIn('first touch was not the bed', g.info[0])
        self.assertEqual(sum(1 for l in m.log
                             if l.startswith('SET_KINEMATIC_POSITION Z=160')),
                         2)

    def test_false_touch_near_the_bed_needs_no_retry(self):
        # A false touch within the confirm's reach: the confirm finds the
        # bed and homes from it, as before.
        m = Machine(touches=[3.])
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 0)

    def test_gives_up_after_attempts(self):
        m = Machine(touches=[100., 90., 80., 70., 60.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('gave up', str(cm.exception))
        self.assertEqual(obj.false_touches, 5)
        self.assertFalse(m.relative)   # gcode state restored on error

    def test_repeated_early_touches_walk_down(self):
        # The plain-G28 case on sloe: head ~20 mm up, the fast approach stops
        # 1.5-2.5 mm in, three times running.
        m = Machine(real_start=20., touches=[18.6, 11.1, 3.6])
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 2)
        self.assertEqual(obj.last_attempts, 3)

    def test_other_confirm_errors_still_raise(self):
        m = Machine()
        obj, cmd = make(m)
        real_probe = m.probe
        def probe(fast):
            if not fast:
                raise CommandError("Probe samples exceed samples_tolerance")
            real_probe(fast)
        m.probe = probe
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('samples_tolerance', str(cm.exception))
        self.assertEqual(obj.false_touches, 0)

    def test_tolerance_passed_to_confirm(self):
        m = Machine()
        obj, cmd = make(m)
        cmd(FakeGcmd(SAMPLES_TOLERANCE=1.))
        self.assertIn('PROBE SAMPLES_TOLERANCE=1.000', m.log)


if __name__ == '__main__':
    unittest.main()
