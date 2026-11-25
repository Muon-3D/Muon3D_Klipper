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
    def __init__(self, real_start=150., touches=(), known=False):
        self.real = real_start
        # known: Z is homed and the guard vouches for it, so the commanded
        # frame is the real one (the plate at 0)
        self.known = known
        self.z = real_start if known else 0.
        self.touches = list(touches)
        self.relative = False
        self.last_probe_z = None
        self.log = []
        self.dwells = []
        # (fast, real height at the start, at the end) of every PROBE, and
        # the highest real height the head reached
        self.probes = []
        self.max_real = real_start

    def off(self):
        return self.z - self.real

    def probe(self, fast):
        start = self.real
        try:
            self._probe(fast)
        finally:
            self.probes.append((fast, start, self.real))

    def _probe(self, fast):
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
                self.max_real = max(self.max_real, self.real)
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
                'axis_minimum': [0., -10., Z_MIN],
                'homed_axes': 'xyz' if self.m.known else '',
                'position': [100., 60., self.m.z, 0.]}
    def wait_moves(self):
        pass


class FakeReactor:
    def monotonic(self):
        return 0.


class FakeGuard:
    def __init__(self, known):
        self.known = known
    def get_status(self, et):
        return {'z_known': self.known}


class FakePrinter:
    def __init__(self, m, guard='from machine'):
        self.objs = {'gcode': FakeGcode(m), 'toolhead': FakeToolhead(m)}
        if guard == 'from machine':
            guard = FakeGuard(m.known)
        if guard is not None:
            self.objs['contact_guard'] = guard
    def lookup_object(self, name, *default):
        if name in self.objs:
            return self.objs[name]
        if default:
            return default[0]
        raise KeyError(name)
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
    def getboolean(self, k, d, **kw):
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


def make(m, guard='from machine', **opts):
    p = FakePrinter(m, guard)
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
        # A false touch 12 mm over the bed: the confirm ends 7 mm over it,
        # and the bounded repeat (10 mm up, 20 mm down) reaches it.
        m = Machine(real_start=30., touches=[12.])
        obj, cmd = make(m)
        g = FakeGcmd()
        cmd(g)
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 1)
        self.assertEqual(obj.last_attempts, 2)
        self.assertEqual(m.dwells, [])   # retries at once by default
        self.assertIn('first touch was not the bed', g.info[0])
        # only the first attempt soft-homes at the top
        self.assertEqual(sum(1 for l in m.log
                             if l.startswith('SET_KINEMATIC_POSITION Z=160')),
                         1)

    def test_false_touch_near_the_bed_needs_no_retry(self):
        # A false touch within the confirm's reach: the confirm finds the
        # bed and homes from it, as before.
        m = Machine(touches=[3.])
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.check_homed(m)
        self.assertEqual(obj.false_touches, 0)

    def test_gives_up_after_attempts(self):
        # every repeat false-touches again inside its bounded search
        m = Machine(real_start=60., touches=[50., 44., 38., 32., 26.])
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

    # KAN-3370: the safety stops are errors the home must not swallow.
    # Only "No trigger" on the confirming tap means a false first touch.
    GUARD_ERRORS = [
        "Homing aborted: the bed was removed",
        "Homing aborted: the toolhead disconnected",
        "Homing aborted: the load on the nozzle exceeded the safety ceiling"
        " during the Z move",
        "Homing aborted: the load cell moved the wrong way during the Z"
        " move",
        "The nozzle is already under load: check nothing is touching it"
        " (tare -3244819, max-load reference -3010747, headroom -7.8%,"
        " need 20.0%)",
        "Z homing stopped: no contact within 10 mm. Check the bed is"
        " fitted and nothing is wrong with the nozzle",
    ]

    def test_guard_errors_on_the_confirming_tap_are_not_retried(self):
        for msg in self.GUARD_ERRORS:
            m = Machine()
            obj, cmd = make(m)
            real_probe = m.probe
            calls = []

            def probe(fast, msg=msg, real_probe=real_probe, calls=calls):
                calls.append(fast)
                if not fast:
                    raise CommandError(msg)
                real_probe(fast)
            m.probe = probe
            with self.assertRaises(CommandError) as cm:
                cmd(FakeGcmd())
            self.assertEqual(str(cm.exception), msg)
            # one fast approach, one confirming tap, no second attempt
            self.assertEqual(calls, [True, False], msg)
            self.assertEqual(obj.false_touches, 0, msg)
            self.assertEqual(obj.last_attempts, 1, msg)
            self.assertEqual(
                sum(1 for l in m.log
                    if l.startswith('SET_KINEMATIC_POSITION')), 1, msg)
            self.assertFalse(m.relative)   # gcode state restored

    def test_guard_errors_on_the_fast_approach_are_not_retried(self):
        for msg in self.GUARD_ERRORS:
            m = Machine()
            obj, cmd = make(m)
            calls = []

            def probe(fast, msg=msg, calls=calls):
                calls.append(fast)
                raise CommandError(msg)
            m.probe = probe
            with self.assertRaises(CommandError) as cm:
                cmd(FakeGcmd())
            self.assertEqual(str(cm.exception), msg)
            self.assertEqual(calls, [True], msg)
            self.assertEqual(obj.last_attempts, 1, msg)

    def test_no_trigger_still_retries_next_to_a_guard_error(self):
        # the retry still works, and a guard error on the retry's
        # confirming tap ends the home there
        m = Machine(real_start=30., touches=[12.])
        obj, cmd = make(m)
        real_probe = m.probe
        confirms = []

        def probe(fast):
            if not fast:
                confirms.append(1)
                if len(confirms) == 2:
                    raise CommandError(self.GUARD_ERRORS[0])
            real_probe(fast)
        m.probe = probe
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertEqual(str(cm.exception), self.GUARD_ERRORS[0])
        self.assertEqual(obj.false_touches, 1)
        self.assertEqual(obj.last_attempts, 2)

    def test_retry_settle_dwells_before_the_retry(self):
        m = Machine(real_start=30., touches=[12.])
        obj, cmd = make(m, retry_settle=1.5)
        cmd(FakeGcmd())
        self.check_homed(m)
        self.assertEqual(m.dwells, [1.5])

    def test_tolerance_passed_to_confirm(self):
        m = Machine()
        obj, cmd = make(m)
        cmd(FakeGcmd(SAMPLES_TOLERANCE=1.))
        self.assertIn('PROBE SAMPLES_TOLERANCE=1.000', m.log)


class KnownZ(unittest.TestCase):
    # KAN-4682: a home from a Z the contact guard vouches for keeps its
    # frame and descends from where it is, with FAST_ABOVE
    def soft_homes(self, m):
        return sum(1 for l in m.log
                   if l.startswith('SET_KINEMATIC_POSITION Z=160'))

    def probes(self, m):
        return [l for l in m.log if l.startswith('PROBE')]

    def test_a_known_z_is_not_soft_homed(self):
        m = Machine(real_start=140., known=True)
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertAlmostEqual(m.real, 10.)
        self.assertEqual(self.soft_homes(m), 0)
        fast = self.probes(m)[0]
        self.assertIn('PROBE_SPEED=15.000', fast)
        self.assertIn('FAST_ABOVE=12.000', fast)
        self.assertNotIn('FAST_ABOVE', self.probes(m)[1])   # the confirm
        self.assertEqual(obj.fast_homes, 1)
        self.assertEqual(obj.last_fast, 'known')

    def test_the_margin_is_a_config_value(self):
        m = Machine(real_start=140., known=True)
        obj, cmd = make(m, fast_margin=10.)
        cmd(FakeGcmd())
        self.assertIn('FAST_ABOVE=10.000', self.probes(m)[0])

    def legacy(self, m, obj, why):
        self.assertEqual(self.soft_homes(m), 1, why)
        self.assertNotIn('FAST_ABOVE', ' '.join(self.probes(m)), why)
        self.assertEqual(obj.fast_homes, 0, why)

    def test_what_is_not_known_is_homed_as_before(self):
        for why, kw in (('guard says no', dict(guard=FakeGuard(False))),
                        ('no guard', dict(guard=None)),
                        ('fast_approach off', dict(fast_approach=False))):
            m = Machine(real_start=140., known=True)
            obj, cmd = make(m, **kw)
            cmd(FakeGcmd())
            self.legacy(m, obj, why)
            self.assertAlmostEqual(m.off(), 0., places=6)
        # Z not homed at all (the guard may still say anything)
        m = Machine(real_start=150.)
        obj, cmd = make(m, guard=FakeGuard(True))
        cmd(FakeGcmd())
        self.legacy(m, obj, 'Z not homed')
        self.assertEqual(obj.last_fast, 'Z not homed')

    def test_a_head_already_in_the_margin_is_homed_as_before(self):
        m = Machine(real_start=13., known=True)
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.legacy(m, obj, 'inside the margin')
        self.assertEqual(obj.last_fast, 'already within the margin')

    def test_no_contact_where_it_was_known_is_an_error_not_a_blind_retry(self):
        # the plate is 30 mm lower than believed: the head reaches Z-4
        # without a trigger
        m = Machine(real_start=140., known=True)
        m.real = 140. + 30.
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('not where it was believed', str(cm.exception))
        self.assertEqual(self.soft_homes(m), 0)
        self.assertEqual(obj.last_attempts, 1)
        self.assertFalse(m.relative)   # gcode state restored on error

    def test_guard_errors_on_the_fast_approach_are_raised_as_they_are(self):
        for msg in KnownZ_GUARD_ERRORS:
            m = Machine(real_start=140., known=True)
            obj, cmd = make(m)

            def probe(fast, msg=msg):
                raise CommandError(msg)
            m.probe = probe
            with self.assertRaises(CommandError) as cm:
                cmd(FakeGcmd())
            self.assertEqual(str(cm.exception), msg)
            self.assertEqual(obj.last_attempts, 1)

    def test_a_false_touch_on_the_known_path_repeats_bounded(self):
        # a stop 80 mm up on the known path: the confirm finds nothing, and
        # the repeat is the bounded one (not soft-homed at the top, as it was
        # until 2026-11-25): with the bed 76 mm under the confirm's end it
        # finds nothing within 20 mm and stops with an error
        m = Machine(real_start=140., known=True, touches=[80.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('repeat 2 of 5 found no contact', str(cm.exception))
        self.assertEqual(obj.false_touches, 1)
        self.assertEqual(obj.last_attempts, 2)
        self.assertEqual(self.soft_homes(m), 0)
        self.assertAlmostEqual(m.real, 66.)
        fasts = [p for p in self.probes(m) if 'FAST_ABOVE' in p]
        self.assertEqual(len(fasts), 1)     # only the first attempt

    def test_a_false_touch_on_the_known_path_near_the_bed_homes(self):
        m = Machine(real_start=140., known=True, touches=[14.])
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertAlmostEqual(m.real, 10.)
        self.assertEqual(obj.false_touches, 1)
        self.assertEqual(self.soft_homes(m), 0)

    def heaters(self, *pairs):
        class H:
            def __init__(self, t, g):
                self.t, self.g = t, g
            def get_status(self, et):
                return {'temperature': self.t, 'target': self.g}

        class Hs:
            heaters = {'h%d' % i: H(t, g) for i, (t, g) in enumerate(pairs)}
        return Hs()

    def run_with_heaters(self, *pairs):
        m = Machine(real_start=140., known=True)
        p = FakePrinter(m)
        p.objs['heaters'] = self.heaters(*pairs)
        obj = load_cell_home_z.load_config(FakeConfig(p))
        p.objs['gcode'].cmds['LOAD_CELL_HOME_Z'](FakeGcmd())
        return m, obj

    def test_a_heater_still_settling_takes_the_old_path(self):
        # print start: the hot end is heating; its drift rises inside a leg
        for pairs in (((120., 215.),), ((215., 215.), (30., 60.))):
            m, obj = self.run_with_heaters(*pairs)
            self.legacy(m, obj, pairs)
            self.assertEqual(obj.last_fast, 'heater settling')

    def test_heaters_at_target_or_off_do_not_stop_it(self):
        for pairs in (((214., 215.), (60., 60.)), ((25., 0.),), ()):
            m, obj = self.run_with_heaters(*pairs)
            self.assertEqual(obj.fast_homes, 1, pairs)
            self.assertEqual(self.soft_homes(m), 0, pairs)

    def test_the_status_says_what_the_last_home_did(self):
        m = Machine(real_start=140., known=True)
        obj, cmd = make(m)
        cmd(FakeGcmd())
        st = obj.get_status(0.)
        self.assertEqual(st['fast_homes'], 1)
        self.assertEqual(st['last_fast'], 'known')


KnownZ_GUARD_ERRORS = Test.GUARD_ERRORS


class BoundedRetry(unittest.TestCase):
    """2026-11-25, sloe: a repeat after a false touch soft-homed at the top
    and searched 164 mm from wherever the head was; with no contact it drove
    the carriage into the bottom stop.  A repeat now searches at most
    retry_lift + retry_reach (20 mm) and stops with an error."""

    def repeats(self, m):
        return [p for p in m.probes if p[0]][1:]

    def test_max_travel_bounds_the_first_attempt(self):
        # the bench check: 60 mm over the plate (or no plate), a 20 mm
        # search stops 40 mm over it with an error
        m = Machine(real_start=60.)
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd(MAX_TRAVEL=20.))
        self.assertIn('no contact within MAX_TRAVEL 20.0 mm', str(cm.exception))
        self.assertEqual([p[1:] for p in m.probes], [(60., 40.)])
        self.assertEqual(obj.last_attempts, 1)
        self.assertFalse(m.relative)

    def test_max_travel_that_reaches_the_plate_homes(self):
        m = Machine(real_start=15.)
        obj, cmd = make(m)
        cmd(FakeGcmd(MAX_TRAVEL=20.))
        self.assertAlmostEqual(m.real, 10.)


    def test_the_sloe_case_stops_within_the_bound(self):
        # a false touch ~159 mm over the bed, the bed far below: the repeat
        # finds nothing and must stop, not search the whole travel
        m = Machine(real_start=159.5, touches=[159.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        msg = str(cm.exception)
        self.assertIn('repeat 2 of 5 found no contact', msg)
        self.assertIn('Z stopped there', msg)
        self.assertEqual(obj.last_attempts, 2)
        (fast, top, bottom), = self.repeats(m)
        # the repeat lifts only as far as the attempt had come down (to
        # 159.5, where it began, not 10 mm), then searches 14.5 mm
        self.assertAlmostEqual(top, 159.5)
        self.assertAlmostEqual(bottom, 145.)
        self.assertAlmostEqual(obj.last_retry_travel, 14.5)
        # the repeat never lifted the head above where it started (the
        # confirm's 1 mm lift after a touch 0.5 mm down is the old path)
        self.assertLessEqual(m.max_real, 160. + 1e-9)
        self.assertFalse(m.relative)   # gcode state restored on error

    def test_no_contact_retry_from_mid_travel(self):
        # plate missing under a false touch at 80 mm: the confirm ends at
        # 76, the repeat lifts to 86, searches 20 mm and stops at 66
        m = Machine(real_start=120., touches=[80.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('within 20.0 mm', str(cm.exception))
        (fast, top, bottom), = self.repeats(m)
        self.assertAlmostEqual(top, 86.)
        self.assertAlmostEqual(bottom, 66.)
        # net, the head ends 14 mm under the first touch: the confirm's 4
        # plus the repeat's 10 of reach
        self.assertAlmostEqual(80. - m.real, 14.)

    def test_every_repeat_is_bounded(self):
        for real_start, touches in ((159.5, [159.]), (60., [55., 50.]),
                                    (30., [12.]), (20., [18.6, 11.1, 3.6]),
                                    (60., [50., 44., 38., 32., 26.])):
            m = Machine(real_start=real_start, touches=touches)
            obj, cmd = make(m)
            try:
                cmd(FakeGcmd())
            except CommandError:
                pass
            for fast, top, bottom in self.repeats(m):
                self.assertLessEqual(top - bottom, 20. + 1e-9,
                                     (real_start, touches))
            # never above the start, except the confirm's own 1 mm lift
            self.assertLessEqual(m.max_real,
                                 max(real_start, touches[0] + 1.) + 1e-9)

    def test_bound_is_configurable(self):
        m = Machine(real_start=120., touches=[80.])
        obj, cmd = make(m, retry_lift=2., retry_reach=3.)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('within 5.0 mm', str(cm.exception))
        (fast, top, bottom), = self.repeats(m)
        self.assertAlmostEqual(top - bottom, 5.)

    def test_a_repeat_never_soft_homes_at_the_top(self):
        m = Machine(real_start=60., touches=[50., 44., 38., 32., 26.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError):
            cmd(FakeGcmd())
        frames = [l for l in m.log if l.startswith('SET_KINEMATIC_POSITION')]
        self.assertEqual(frames[0], 'SET_KINEMATIC_POSITION Z=160.000')
        self.assertEqual(sum(1 for l in frames if 'Z=160' in l), 1)

    def test_guard_errors_on_a_repeat_are_raised_as_they_are(self):
        m = Machine(real_start=30., touches=[12.])
        obj, cmd = make(m)
        real_probe = m.probe
        fasts = []

        def probe(fast):
            if fast:
                fasts.append(1)
                if len(fasts) == 2:
                    raise CommandError("Homing aborted: the bed was removed")
            real_probe(fast)
        m.probe = probe
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertEqual(str(cm.exception),
                         "Homing aborted: the bed was removed")


if __name__ == '__main__':
    unittest.main()
