"""KAN-1361: the M1 slows the Y confirm taps to 10 mm/s.

Two things are checked, both against the REAL adxl345_probe.py (exec'd with
the klippy modules around it stubbed) and the REAL core/M1/core.cfg:

1. The shipped M1 config resolves the Y confirm speed to 10 mm/s and leaves
   everything else alone: X confirm speed unset (falls back to its
   homing_speed), the first tap still runs at stepper_y homing_speed (60),
   confirm retract speed unset (falls back to the rail's retract speed, fast),
   y samples 2, retries 2, seek_retries 1, retract 8 mm.
2. The confirm loop itself, driven through _confirm_axis_home with a
   simulated wall: a first tap at 60 mm/s that slipped one electrical cycle
   (0.8 mm late) is out-voted by two agreeing slow taps, every re-approach
   runs at the configured confirm speed, and a missed slow tap falls back to
   the fast seek and then confirms slowly again.
"""
import configparser
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths

CORE_CFG = _paths.patched('core/M1/core.cfg')
STEP_DIST = 0.00625  # mm per step, 40 mm / (200 * 32)... any value works


def read_cfg(path):
    cp = configparser.RawConfigParser(strict=False, interpolation=None,
                                      inline_comment_prefixes=None)
    cp.optionxform = str
    cp.read(path)
    return cp


class Cfg:
    # Just enough of klippy's ConfigWrapper for _load_confirm_profile
    def __init__(self, cp, section):
        self.sec = dict(cp.items(section))

    def _get(self, conv, name, default):
        if name not in self.sec:
            return default
        return conv(self.sec[name].split('#')[0].strip())

    def getint(self, name, default=None, **kw):
        return self._get(int, name, default)

    def getfloat(self, name, default=None, **kw):
        return self._get(float, name, default)

    def get(self, name, default=None, **kw):
        return self._get(str, name, default)


class HomingMove:
    # Simulated wall; records the speed of every homing move.
    sim = None

    def __init__(self, printer, endstops):
        self.sim = HomingMove.sim
        self.stepper_positions = []

    def homing_move(self, homepos, speed, check_triggered=True):
        sim = self.sim
        sim.speeds.append(speed)
        slip = sim.script.pop(0)
        if slip is None:  # missed tap: nothing within reach
            sim.wrapper.last_trigger_time = 0.
            return
        # the head really is sim.dist_to_wall from the stop; the tap reports
        # `slip` mm late (0 = at contact)
        d = sim.direction
        start = sim.th.get_position()[1]
        for n in sim.step_names:
            sp = types.SimpleNamespace(
                stepper_name=n, start_pos=int(round(start / STEP_DIST)),
                trig_pos=int(round((174. + d * slip) / STEP_DIST)))
            self.stepper_positions.append(sp)
        sim.wrapper.last_trigger_time = 1.
        sim.dist_to_wall = -slip  # head now sits slip mm into the stop


class Sim:
    def __init__(self, mod, script, cfg, direction=1.):
        self.script = list(script)
        self.speeds = []
        self.moves = []
        self.direction = direction
        self.dist_to_wall = 0.
        self.step_names = ['stepper_y']
        sim = self
        pos_endstop = 174.

        class TH:
            pos = [100., pos_endstop, 0., 0.]

            def get_position(self):
                return list(self.pos)

            def set_position(self, p):
                self.pos = list(p)

            def move(self, p, speed):
                sim.moves.append(speed)
                sim.dist_to_wall = max(0., abs(pos_endstop - p[1]))
                self.pos = list(p)
        self.th = TH()
        self.wrapper = types.SimpleNamespace(
            last_trigger_time=0., get_axis=lambda: 'y',
            get_steppers=lambda: [types.SimpleNamespace(
                get_name=lambda: 'stepper_y',
                get_step_dist=lambda: STEP_DIST)])
        hi = types.SimpleNamespace(positive_dir=direction > 0,
                                   position_endstop=pos_endstop,
                                   speed=60., retract_speed=60.)
        self.rail = types.SimpleNamespace(
            get_homing_info=lambda: hi, get_range=lambda: (-10., 174.1),
            get_endstops=lambda: [])
        self.cfg = cfg
        printer = types.SimpleNamespace(
            lookup_object=lambda n: self.th,
            command_error=RuntimeError,
            get_start_args=lambda: {})
        probe = object.__new__(mod.ADXL345Probe)
        probe.printer = printer
        probe.axis_homing_arm_delay = {'x': 0.1, 'y': 0.1, 'z': 0.1}
        self.probe = probe
        HomingMove.sim = self

    def run(self, first_tap_slip=0.):
        # The first tap (at the homing speed) has already happened.
        self.dist_to_wall = 0.
        start = int(round((174. + self.direction * first_tap_slip)
                          / STEP_DIST))
        state = types.SimpleNamespace(trigger_mcu_pos={'stepper_y': start})
        self.probe._confirm_axis_home(state, self.rail, self.wrapper,
                                      self.cfg)
        return state.trigger_mcu_pos['stepper_y']


def load_probe():
    pkg = types.ModuleType('extras')
    pkg.__path__ = []
    sys.modules['extras'] = pkg
    for name, attrs in (('probe', {}), ('adxl345', {'FREEFALL_ACCEL': 9806.65}),
                        ('homing', {'HomingMove': HomingMove})):
        m = types.ModuleType('extras.' + name)
        m.__dict__.update(attrs)
        sys.modules['extras.' + name] = m
        setattr(pkg, name, m)
    sys.modules['pins'] = types.ModuleType('pins')
    path = _paths.patched('klippy/extras/adxl345_probe.py')
    mod = types.ModuleType('extras.adxl345_probe')
    mod.__dict__['__name__'] = 'extras.adxl345_probe'
    mod.__dict__['__package__'] = 'extras'
    exec(compile(open(path, 'r', newline='').read(), path, 'exec'),
         mod.__dict__)
    return mod


bad = 0


def check(name, ok):
    global bad
    print('  %-70s %s' % (name, 'PASS' if ok else 'FAIL'))
    bad += (not ok)


mod = load_probe()
cp = read_cfg(CORE_CFG)
probe = object.__new__(mod.ADXL345Probe)
cfg = Cfg(cp, 'adxl345_probe')
yprof = probe._load_confirm_profile(cfg, 'y')
xprof = probe._load_confirm_profile(cfg, 'x')

print('M1 config resolution')
check('Y confirm speed resolves to 10 mm/s', yprof['speed'] == 10.)
check('X confirm speed unset (follows X homing_speed)', xprof['speed'] is None)
check('first tap unchanged: stepper_y homing_speed 60',
      cp.get('stepper_y', 'homing_speed') == '60')
check('first tap unchanged: stepper_x homing_speed 60',
      cp.get('stepper_x', 'homing_speed') == '60')
check('confirm retract speed unset (stays fast: rail retract speed)',
      yprof['retract_speed'] is None)
check('Y samples 2, retries 2, seek_retries 1, retract 8 mm',
      (yprof['samples'], yprof['retries'], yprof['seek_retries'],
       yprof['retract_dist']) == (2, 2, 1, 8.))
check('X stays single-tap', xprof['samples'] == 1)
check('arm delay still fits: retract 8 > 10 mm/s * 0.1 s',
      yprof['retract_dist'] > (yprof['speed'] or 60.) * 0.1)

print('confirm loop at the shipped speed')
# slipped fast first tap (+0.8 mm), then two agreeing slow taps
sim = Sim(mod, [0., 0.], yprof)
res = sim.run(first_tap_slip=0.8)
check('slipped first tap is out-voted by two agreeing slow taps',
      abs(res - 174. / STEP_DIST) <= 1)
check('every confirm re-approach ran at 10 mm/s', sim.speeds == [10., 10.])
check('retracts stayed at the rail retract speed (60)',
      sim.moves == [60., 60.])

# clean first tap: one slow confirm tap is enough
sim = Sim(mod, [0.], yprof)
sim.run()
check('good home: a single slow confirm tap', sim.speeds == [10.])

# missed slow tap: fast seek, then confirm slowly again
sim = Sim(mod, [None, 0., 0.], yprof)
sim.run()
check('missed slow tap: seek at 60, then slow confirm at 10',
      sim.speeds == [10., 60., 10.])

# two missed slow taps: refuses, does not loop
sim = Sim(mod, [None, 0., None], yprof)
try:
    sim.run()
    check('second miss after the seek refuses', False)
except RuntimeError as e:
    check('second miss after the seek refuses', 'not confirmed' in str(e))

sys.exit(1 if bad else 0)
