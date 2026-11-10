# Offline tests for klippy/extras/motor_tune.py: register reads happen in
# the constant-speed part of each pass, fields decode, TSTEP gives speed.
import json, os, sys, tempfile, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'klippy'))
from extras import motor_tune, tmc, tmc2241  # noqa: E402


class CommandError(Exception):
    pass


class FakeReactor:
    def __init__(self):
        self.now = 100.
    def monotonic(self):
        return self.now
    def pause(self, t):
        self.now = max(self.now, t)
        return self.now


class FakeMcu:
    """Print time = monotonic - 90."""
    def __init__(self, reactor):
        self.r = reactor
    def estimated_print_time(self, now):
        return now - 90.


class FakeToolhead:
    def __init__(self, reactor):
        self.r = reactor
        self.last_move_time = 10.
        self.queued_end = None
        self.status = {'homed_axes': 'xyz', 'max_velocity': 800.,
                       'max_accel': 20000., 'minimum_cruise_ratio': 0.5}
    def get_status(self, et):
        return dict(self.status)
    def get_last_move_time(self):
        return self.last_move_time
    def wait_moves(self):
        self.r.now = max(self.r.now, self.last_move_time + 90.)


class FakeGcode:
    def __init__(self, toolhead, reactor):
        self.th, self.r = toolhead, reactor
        self.scripts = []
        self.out = []
        self.accel = 20000.
    def register_command(self, *a, **k):
        pass
    def run_script_from_command(self, script):
        self.scripts.append(script)
        for line in script.split('\n'):
            if line.startswith('SET_VELOCITY_LIMIT'):
                for w in line.split()[1:]:
                    k, v = w.split('=')
                    if k == 'ACCEL':
                        self.accel = float(v)
            if line.startswith('G1') and ' F' in line:
                # Queue: start at max(now, last), with the planner's timing.
                f = float(line.split(' F')[1]) / 60.
                start = max(self.th.last_move_time, self.r.now - 90.)
                d = 140. if 'X170' in line or 'X30' in line else 110.
                if f >= 100.:
                    a = self.accel
                    dur = 2 * f / a + (d - f * f / a) / f
                else:
                    dur = 1.
                self.th.last_move_time = start + dur
            if line == 'M400':
                self.th.wait_moves()


class FakeMcuTmc:
    def __init__(self, reactor, log):
        self.fields = tmc.FieldHelper(tmc2241.Fields, tmc2241.SignedFields,
                                      tmc2241.FieldFormatters)
        self.fields.set_field('mres', 2)    # 64 microsteps
        self.r = reactor
        self.log = log
    def get_fields(self):
        return self.fields
    def get_tmc_frequency(self):
        return 12500000.
    def get_register(self, reg):
        self.log.append((reg, self.r.now - 90.))
        f = self.fields
        if reg == 'PWM_SCALE':
            return f.set_field('pwm_scale_sum', 180, 0)
        if reg == 'DRV_STATUS':
            v = f.set_field('cs_actual', 25, 0)
            return f.set_field('sg_result', 300, v)
        if reg == 'PWM_AUTO':
            return f.set_field('pwm_ofs_auto', 30, 0)
        if reg == 'SG4_RESULT':
            return f.set_field('sg4_result', 120, 0)
        if reg == 'TSTEP':
            # 600 mm/s: 0.003125 mm per 1/64 step -> 1/256 is 0.00078125.
            return int(round(0.00078125 * 12500000. / 600.))
        return 0


class FakeDriver:
    def __init__(self, mcu_tmc):
        self.mcu_tmc = mcu_tmc


class FakeStepper:
    def get_step_dist(self):
        return 40. / (200 * 64)


class FakePrinter:
    command_error = CommandError
    def __init__(self, objects, reactor):
        self.objects, self.r = objects, reactor
        self.handlers = {}
    def get_reactor(self):
        return self.r
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, ev, fn):
        self.handlers[ev] = fn


class FakeConfig:
    def __init__(self, printer, log_path):
        self.p, self.lp = printer, log_path
    def get_printer(self):
        return self.p
    def get(self, k, default=None):
        return self.lp if k == 'log_path' else default
    def getlist(self, k, default=None):
        return default


class FakeGcmd:
    def __init__(self, params):
        self.p = params
        self.out = []
    def get(self, k, default=None):
        return self.p.get(k, default)
    def get_float(self, k, default=None, **kw):
        return float(self.p.get(k, default))
    def get_int(self, k, default=None, **kw):
        return int(self.p.get(k, default))
    def error(self, msg):
        return CommandError(msg)
    def respond_info(self, msg):
        self.out.append(msg)


def make():
    r = FakeReactor()
    th = FakeToolhead(r)
    gc = FakeGcode(th, r)
    reads = []
    objs = {'gcode': gc, 'toolhead': th, 'mcu': FakeMcu(r),
            'tmc2241 stepper_x': FakeDriver(FakeMcuTmc(r, reads)),
            'tmc2241 stepper_y': FakeDriver(FakeMcuTmc(r, reads)),
            'stepper_x': FakeStepper(), 'stepper_y': FakeStepper()}
    p = FakePrinter(objs, r)
    fd, path = tempfile.mkstemp(suffix='.jsonl')
    os.close(fd)
    mt = motor_tune.MotorTune(FakeConfig(p, path))
    p.handlers['klippy:connect']()
    return mt, gc, th, reads, path


class TestMotorTune(unittest.TestCase):
    def test_reads_land_in_the_cruise_and_decode(self):
        mt, gc, th, reads, path = make()
        g = FakeGcmd({'AXIS': 'X', 'SPEED': '600', 'ACCEL': '10000',
                      'PASSES': '2', 'SAMPLES': '2', 'LABEL': 't',
                      'TAG': '{"current": 0.8}'})
        mt.cmd_SAMPLE(g)
        with open(path) as f:
            recs = [json.loads(l) for l in f]
        os.unlink(path)
        # 2 passes x 2 samples x 2 drivers
        self.assertEqual(len(recs), 8)
        r = recs[0]
        self.assertEqual(r['pwm_scale_sum'], 180)
        self.assertEqual(r['cs_actual'], 25)
        self.assertEqual(r['sg_result'], 300)
        self.assertEqual(r['sg4_result'], 120)
        self.assertEqual(r['current'], 0.8)
        self.assertAlmostEqual(r['speed_measured'], 600., delta=40.)
        # 140 mm at 600 mm/s, 10k: ramps 0.06 s each, cruise 0.173 s.
        self.assertAlmostEqual(r['cruise_s'], (140. - 36.) / 600., places=3)
        self.assertIn('MINIMUM_CRUISE_RATIO=0', '\n'.join(gc.scripts))
        self.assertIn('MINIMUM_CRUISE_RATIO=0.500', gc.scripts[-1])
        self.assertIn('pwm_scale_sum 180', g.out[-1])

    def test_no_cruise_reads_nothing(self):
        mt, gc, th, reads, path = make()
        g = FakeGcmd({'AXIS': 'Y', 'SPEED': '800', 'ACCEL': '3000',
                      'LABEL': 't'})
        mt.cmd_SAMPLE(g)
        os.unlink(path)
        self.assertEqual(reads, [])
        self.assertIn('no constant-speed part', g.out[-1])

    def test_unhomed_is_refused(self):
        mt, gc, th, reads, path = make()
        th.status['homed_axes'] = 'z'
        with self.assertRaises(CommandError):
            mt.cmd_SAMPLE(FakeGcmd({'AXIS': 'X', 'SPEED': '100'}))
        os.unlink(path)


if __name__ == '__main__':
    unittest.main()
