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
        self.cleared = False

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
                if w[1] == 'CLEAR_HOMED=Z':
                    self.cleared = True
                else:
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
                # the fast approach names its speed; the confirm does not
                self.probe('PROBE_SPEED=' in line)
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


class FakeProbe:
    def __init__(self, m):
        self.m = m
        self.ceilings = []
    def set_lift_ceiling(self, z):
        # (the ceiling, the real height it stands for)
        self.ceilings.append(
            None if z is None else (z, z - self.m.off()))


class FakePrinter:
    def __init__(self, m, guard='from machine'):
        self.objs = {'gcode': FakeGcode(m), 'toolhead': FakeToolhead(m),
                     'probe': FakeProbe(m)}
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

    def get(self, k, d=None):
        return self.opts.get(k, d)

    def deprecate(self, option, value=None):
        if option in self.opts:
            self.deprecated = getattr(self, 'deprecated', []) + [option]


class FakeGcmd:
    error = CommandError
    def __init__(self, **params):
        self.params = params
        self.info = []
    def get_float(self, k, d, **kw):
        return self.params.get(k, d)
    def get_int(self, k, d, **kw):
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

    # KAN-3370: the safety stops are errors the home must not swallow
    # (a wrong-way stop is retried a bounded number of times: WrongWay).
    # Only "No trigger" on the confirming tap means a false first touch.
    GUARD_ERRORS = [
        "Homing aborted: the bed was removed",
        "Homing aborted: the toolhead disconnected",
        "Homing aborted: the load on the nozzle exceeded the safety ceiling"
        " during the Z move",
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
                    if l.startswith('SET_KINEMATIC_POSITION Z=')), 1, msg)
            self.assertTrue(m.cleared, msg)   # failed: Z not left homed
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

    def test_trigger_counts_reaches_both_taps(self):
        # A TRIGGER_COUNTS= on the home reaches the fast approach and the
        # confirming tap both.
        m = Machine()
        obj, cmd = make(m)
        cmd(FakeGcmd(TRIGGER_COUNTS=150000, SAMPLES_TOLERANCE=1.))
        self.check_homed(m)
        probes = [l for l in m.log if l.startswith('PROBE')]
        self.assertEqual(len(probes), 2)
        self.assertTrue(probes[0].endswith('SAMPLES=1 TRIGGER_COUNTS=150000'))
        self.assertEqual(probes[1],
                         'PROBE TRIGGER_COUNTS=150000 SAMPLES_TOLERANCE=1.000')

    def test_no_trigger_counts_leaves_the_probe_default(self):
        m = Machine()
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertFalse(any('TRIGGER_COUNTS' in l for l in m.log))


class OneDescent(unittest.TestCase):
    # 2026-11-24: the KAN-4682 fast approach was removed with the Z legs.
    # Every home soft-homes at the top and makes one descent at fast_speed.
    def soft_homes(self, m):
        return sum(1 for l in m.log
                   if l.startswith('SET_KINEMATIC_POSITION Z=160'))

    def probes(self, m):
        return [l for l in m.log if l.startswith('PROBE')]

    def test_a_known_z_is_soft_homed_like_any_other(self):
        m = Machine(real_start=140., known=True)
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertAlmostEqual(m.real, 10.)
        self.assertEqual(self.soft_homes(m), 1)
        fast = self.probes(m)[0]
        self.assertIn('PROBE_SPEED=10.000', fast)
        self.assertNotIn('FAST_ABOVE', ' '.join(self.probes(m)))

    def test_fast_speed_default_is_10(self):
        obj, cmd = make(Machine())
        self.assertEqual(obj.fast_speed, 10.)

    def test_removed_options_load_and_are_ignored(self):
        # a dev-mode config written before the removal still loads
        m = Machine(real_start=140., known=True)
        p = FakePrinter(m)
        cfg = FakeConfig(p, fast_approach=True, fast_margin=12.,
                         fast_heater_settle=3.)
        load_cell_home_z.load_config(cfg)
        self.assertEqual(sorted(cfg.deprecated),
                         ['fast_approach', 'fast_heater_settle',
                          'fast_margin'])
        p.objs['gcode'].cmds['LOAD_CELL_HOME_Z'](FakeGcmd())
        self.assertAlmostEqual(m.real, 10.)
        self.assertNotIn('FAST_ABOVE',
                         ' '.join(l for l in m.log if l.startswith('PROBE')))

    def test_the_status_has_no_fast_keys(self):
        obj, cmd = make(Machine())
        cmd(FakeGcmd())
        self.assertEqual(sorted(obj.get_status(0.)),
                         ['false_touches', 'last_attempts',
                          'last_retry_travel', 'last_wrong_way_retries',
                          'wrong_way_soft_retries'])




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
        self.assertTrue(m.cleared)     # Z is not left homed in that frame

    def test_max_travel_does_not_repeat_after_a_false_touch(self):
        m = Machine(real_start=60., touches=[45.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd(MAX_TRAVEL=20.))
        self.assertIn('gave up', str(cm.exception))
        self.assertEqual(len([p for p in m.probes if p[0]]), 1)
        self.assertTrue(m.cleared)

    def test_a_confirm_never_goes_below_max_travel(self):
        # a false touch 19 mm into a 20 mm search: the confirm stops at the
        # search's floor (40), not 4 mm under the touch (37)
        m = Machine(real_start=60., touches=[41.])
        obj, cmd = make(m)
        with self.assertRaises(CommandError):
            cmd(FakeGcmd(MAX_TRAVEL=20.))
        self.assertAlmostEqual(min(p[2] for p in m.probes), 40.)
        self.assertTrue(m.cleared)

    def test_a_confirm_never_goes_below_a_repeats_floor(self):
        # repeat 2 searches 150 -> 134 and false-touches at 135: its confirm
        # stops at 134, not at 131
        m = Machine(real_start=150., touches=[148., 135.])
        obj, cmd = make(m, attempts=2)
        with self.assertRaises(CommandError):
            cmd(FakeGcmd())
        (fast, top, bottom), = self.repeats(m)
        self.assertAlmostEqual(top, 150.)
        self.assertAlmostEqual(m.probes[-1][2], 134.)

    def test_the_plate_near_the_floor_still_homes(self):
        # the plate 2 mm over the floor: the kept frame still homes on it
        m = Machine(real_start=15.)
        obj, cmd = make(m)
        cmd(FakeGcmd(MAX_TRAVEL=17.))
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertAlmostEqual(m.real, 10.)
        self.assertFalse(m.cleared)

    def test_early_trigger_lifts_are_capped_at_the_attempts_start(self):
        # sloe: every attempt's ceiling is where it began (159.5 real), and
        # the probe is left uncapped afterwards
        m = Machine(real_start=159.5, touches=[159.])
        obj, cmd = make(m)
        p = obj.printer.lookup_object('probe')
        with self.assertRaises(CommandError):
            cmd(FakeGcmd())
        self.assertIsNone(p.ceilings[-1])
        for z, real in p.ceilings[:-1]:
            self.assertAlmostEqual(real, 159.5)

    def test_a_confirm_lift_never_passes_the_start(self):
        # the repeat lifts back to 150, where the home began, and touches at
        # 149.5: its confirm lifts 0.5 mm, not 1, so not above 150
        m = Machine(real_start=150., touches=[148., 149.5])
        obj, cmd = make(m, attempts=2)
        with self.assertRaises(CommandError):
            cmd(FakeGcmd())
        (fast, top, bottom), = self.repeats(m)
        self.assertAlmostEqual(top, 150.)
        self.assertLessEqual(m.max_real, 150. + 1e-9)

    def test_a_good_home_leaves_z_homed(self):
        m = Machine(real_start=30., touches=[12.])
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertFalse(m.cleared)

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
        # 159.5, where it began), then searches 14.5 mm
        self.assertAlmostEqual(top, 159.5)
        self.assertAlmostEqual(bottom, 145.)
        self.assertAlmostEqual(obj.last_retry_travel, 14.5)
        self.assertTrue(m.cleared)     # Z is not left homed
        # the head never went above where it started, not even for the
        # confirm's lift after the touch 0.5 mm down
        self.assertLessEqual(m.max_real, 159.5 + 1e-9)
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



WRONG_WAY = ("Homing aborted: the load cell moved the wrong way during the Z"
             " move")


class WrongWay(unittest.TestCase):
    """2026-11-25, smew: every PRINT_START's first descent was stopped for a
    wrong-way load about 3 s in.  The probe is re-tared and run again in the
    same frame, so it ends at the same floor, at most wrong_way_retries
    times per home."""

    def drift(self, m, which, travel=30.):
        # the probes numbered in `which` (0 = the first PROBE) go down
        # `travel` (never past their floor) and are stopped wrong-way
        real = m._probe
        n = [0]

        def probe(fast):
            i = n[0]
            n[0] += 1
            if i in which:
                floor = max(0., m.real - (m.z - Z_MIN))
                d = min(travel, m.real - floor)
                m.z -= d
                m.real -= d
                raise CommandError(WRONG_WAY)
            real(fast)
        m._probe = probe

    def floors(self, m):
        return [l for l in m.log if l.startswith('SET_KINEMATIC_POSITION Z=')]

    def test_a_wrong_way_stop_is_retried_and_homes(self):
        m = Machine(real_start=150.)
        self.drift(m, {0})
        obj, cmd = make(m)
        g = FakeGcmd()
        cmd(g)
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertAlmostEqual(m.real, 10.)
        self.assertFalse(m.cleared)
        self.assertEqual(obj.last_wrong_way_retries, 1)
        self.assertEqual(obj.wrong_way_soft_retries, 1)
        self.assertEqual(obj.last_attempts, 1)
        self.assertEqual(obj.false_touches, 0)
        # one info line, no error; settled then re-tared before the retry
        self.assertEqual(len([i for i in g.info if 'wrong way' in i]), 1)
        self.assertFalse(any(i.startswith('!!') for i in g.info))
        i = m.log.index('G4 P500')
        self.assertEqual(m.log[i + 1:i + 3], ['M400', 'LOAD_CELL_TARE'])
        # the same frame: Z declared once, at the top, and the retry went
        # on from where the stopped probe ended
        self.assertEqual(self.floors(m), ['SET_KINEMATIC_POSITION Z=160.000'])
        self.assertEqual([p[1:] for p in m.probes[:2]],
                         [(150., 120.), (120., 0.)])

    def test_past_the_limit_the_home_stops_with_an_error(self):
        m = Machine(real_start=150.)
        self.drift(m, set(range(10)), travel=5.)
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('moved the wrong way 4 times in this home',
                      str(cm.exception))
        self.assertEqual(len(m.probes), 4)       # the first and 3 retries
        self.assertEqual(obj.last_wrong_way_retries, 3)
        self.assertTrue(m.cleared)
        self.assertFalse(m.relative)

    def test_zero_retries_stops_at_once(self):
        m = Machine(real_start=150.)
        self.drift(m, {0})
        obj, cmd = make(m, wrong_way_retries=0)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('moved the wrong way 1 times', str(cm.exception))
        self.assertEqual(len(m.probes), 1)
        self.assertTrue(m.cleared)

    def test_the_retries_stay_within_max_travel(self):
        # no plate: 4 probes stopped wrong-way after 4 mm each, then no
        # contact.  Together they go 20 mm, MAX_TRAVEL, and no further
        m = Machine(real_start=60.)
        self.drift(m, {0, 1, 2}, travel=4.)
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd(MAX_TRAVEL=20.))
        self.assertIn('no contact within MAX_TRAVEL 20.0 mm',
                      str(cm.exception))
        self.assertAlmostEqual(min(p[2] for p in m.probes), 40.)
        self.assertEqual(len(self.floors(m)), 1)
        self.assertTrue(m.cleared)

    def test_the_retries_stay_within_a_repeats_floor(self):
        # a false touch at 148 (its confirm ends at 144), then the bounded
        # repeat (back up to 150, floor 134) is stopped wrong-way twice and
        # finds no bed: it ends at 134
        m = Machine(real_start=150., touches=[148.])
        self.drift(m, {2, 3}, travel=6.)
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('repeat 2 of 5 found no contact', str(cm.exception))
        self.assertAlmostEqual(min(p[2] for p in m.probes), 134.)
        self.assertEqual(obj.last_wrong_way_retries, 2)
        self.assertLessEqual(m.max_real, 150. + 1e-9)

    def test_a_wrong_way_confirm_is_retried(self):
        m = Machine(real_start=150.)
        self.drift(m, {1}, travel=0.5)
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertEqual(obj.last_wrong_way_retries, 1)
        self.assertEqual(obj.false_touches, 0)

    def test_one_limit_per_home_and_reset_on_the_next(self):
        m = Machine(real_start=150.)
        self.drift(m, {0, 3})
        obj, cmd = make(m)
        cmd(FakeGcmd())
        self.assertEqual(obj.last_wrong_way_retries, 1)
        m.real = 150.
        cmd(FakeGcmd())
        self.assertAlmostEqual(m.off(), 0., places=6)
        self.assertEqual(obj.last_wrong_way_retries, 1)
        self.assertEqual(obj.wrong_way_soft_retries, 2)

    def test_other_guard_stops_still_end_the_home(self):
        m = Machine(real_start=150.)
        def probe(fast):
            raise CommandError("Homing aborted: the load on the nozzle"
                               " exceeded the safety ceiling during the Z"
                               " move")
        m._probe = probe
        obj, cmd = make(m)
        with self.assertRaises(CommandError) as cm:
            cmd(FakeGcmd())
        self.assertIn('safety ceiling', str(cm.exception))
        self.assertEqual(len(m.probes), 1)
        self.assertEqual(obj.last_wrong_way_retries, 0)

    def test_status_reports_the_retries(self):
        m = Machine(real_start=150.)
        self.drift(m, {0})
        obj, cmd = make(m)
        cmd(FakeGcmd())
        st = obj.get_status(0.)
        self.assertEqual(st['last_wrong_way_retries'], 1)
        self.assertEqual(st['wrong_way_soft_retries'], 1)

if __name__ == '__main__':
    unittest.main()
