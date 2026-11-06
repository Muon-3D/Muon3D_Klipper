"""KAN-1361: adxl345_probe logs both tap trigger positions for every
confirmation comparison.  The module is loaded with its klippy imports
stubbed (as the other tests here do), then the real helper is called.

    python3 test/tap_path/test_tap_compare_log.py
"""
import importlib.util
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402


def load_module():
    pkg = types.ModuleType('stubpkg')
    pkg.__path__ = []
    sys.modules['stubpkg'] = pkg
    for name in ('probe', 'adxl345', 'homing'):
        mod = types.ModuleType('stubpkg.' + name)
        sys.modules['stubpkg.' + name] = mod
        setattr(pkg, name, mod)
    sys.modules['stubpkg.adxl345'].FREEFALL_ACCEL = 9806.65
    sys.modules['pins'] = types.ModuleType('pins')
    path = _paths.patched('klippy/extras/adxl345_probe.py')
    spec = importlib.util.spec_from_file_location(
        'stubpkg.adxl345_probe', path)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as e:
        sys.exit("cannot load adxl345_probe with stubs: %r" % (e,))
    return mod


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok  " + what)


def main():
    mod = load_module()
    fmt = mod.format_tap_comparison
    sd = {'stepper_y': 0.003125}
    # Two taps one electrical cycle (256 microsteps = 0.800 mm) apart
    line = fmt('y', {'stepper_y': 55360}, {'stepper_y': 55616}, sd, 0.8,
               60., 16., 15.876)
    print(line)
    check('\n' not in line, "a single log line")
    check('y tap compare' in line, "names the axis")
    check('prev stepper_y=173.000 mm' in line, "first trigger position, mm")
    check('new stepper_y=173.800 mm' in line, "second trigger position, mm")
    check('spread 0.800mm' in line, "the spread")
    check('approach 60.0mm/s over 16.00mm, travelled 15.876mm' in line,
          "approach speed, distance offered and distance travelled")
    # The sign is recoverable: a tap on the other side of the first
    back = fmt('y', {'stepper_y': 55616}, {'stepper_y': 55360}, sd, 0.8)
    check('prev stepper_y=173.800 mm' in back
          and 'new stepper_y=173.000 mm' in back, "order shows the direction")
    # Agreeing taps are logged too (the caller logs every comparison)
    ok = fmt('y', {'stepper_y': 55360}, {'stepper_y': 55362}, sd, 0.006)
    check('spread 0.006mm' in ok, "an agreeing pair formats the same way")
    # Two steppers on one axis: every stepper appears, sorted
    dual = fmt('x', {'stepper_x': 100, 'stepper_x1': 90},
               {'stepper_x': 104, 'stepper_x1': 94},
               {'stepper_x': 0.0125, 'stepper_x1': 0.0125}, 0.05)
    check('stepper_x=1.250' in dual and 'stepper_x1=1.125' in dual,
          "each stepper's position is logged")
    check(dual.index('stepper_x=') < dual.index('stepper_x1='),
          "steppers in a stable order")
    test_set_accel_probe_confirm_knobs(mod)
    print("tap compare log tests passed")


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self.params = params
        self.out = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_command_parameters(self):
        return {k: str(v) for k, v in self.params.items()}

    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None):
        v = float(self.params[name])
        bad = ((minval is not None and v < minval)
               or (above is not None and v <= above)
               or (maxval is not None and v > maxval))
        if bad:
            raise self.error("%s out of range" % name)
        return v

    def respond_info(self, msg):
        self.out.append(msg)


def test_set_accel_probe_confirm_knobs(mod):
    cls = mod.ADXL345Probe if hasattr(mod, 'ADXL345Probe') else None
    if cls is None:
        names = [n for n, v in vars(mod).items()
                 if isinstance(v, type) and hasattr(v, 'cmd_SET_ACCEL_PROBE')]
        cls = vars(mod)[names[0]]
    probe = object.__new__(cls)
    probe._tap_mode = None
    prof = {'tap_thresh': 15., 'tap_dur': 0.03, 'tap_axes_mask': 2}
    probe.tap_profiles = {'x': dict(prof), 'y': dict(prof), 'z': dict(prof)}
    probe.axis_homing_arm_delay = {'x': 0., 'y': 0.1, 'z': 0.}
    probe._mask_to_axes = lambda m: 'y'
    probe._homing_speed = lambda a: 60.
    conf = {'samples': 2, 'tolerance': 0.2, 'retries': 2,
            'retract_dist': 8., 'retract_speed': None, 'speed': None,
            'seek_retries': 1, 'max_seek_dist': None}
    probe.tap_confirm = {'x': dict(conf), 'y': dict(conf)}
    g = FakeGcmd(AXIS='Y', CONFIRM_SPEED=30, CONFIRM_RETRACT_DIST=12,
                 CONFIRM_RETRACT_SPEED=45)
    probe.cmd_SET_ACCEL_PROBE(g)
    y = probe.tap_confirm['y']
    check((y['speed'], y['retract_dist'], y['retract_speed']) ==
          (30., 12., 45.), "CONFIRM_* set the Y confirm speed and retract")
    check(probe.tap_confirm['x']['retract_dist'] == 8.,
          "AXIS=Y leaves X alone")
    check('speed=30' in g.out[0] and 'retract_dist=12.00' in g.out[0],
          "the command reports the confirm settings")
    probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', CONFIRM_SPEED=0))
    check(probe.tap_confirm['y']['speed'] is None,
          "CONFIRM_SPEED=0 goes back to the homing speed")
    probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', TAP_DUR=0.01))
    check(probe.tap_confirm['y']['retract_dist'] == 12.,
          "other SET_ACCEL_PROBE calls keep the confirm settings")
    probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', CONFIRM_SPEED=60,
                                       CONFIRM_RETRACT_SPEED=60,
                                       CONFIRM_RETRACT_DIST=20))
    y = probe.tap_confirm['y']
    check((y['speed'], y['retract_speed'], y['retract_dist']) ==
          (60., 60., 20.), "the limits themselves are accepted")
    for kw, what in (({'CONFIRM_SPEED': 60.1}, "speed above homing_speed"),
                     ({'CONFIRM_RETRACT_SPEED': 60.1},
                      "retract speed above homing_speed"),
                     ({'CONFIRM_RETRACT_DIST': 20.01},
                      "retract distance above 20 mm"),
                     ({'CONFIRM_RETRACT_DIST': 0},
                      "retract distance of 0"),
                     ({'CONFIRM_SPEED': -1}, "negative speed")):
        before = dict(probe.tap_confirm['y'])
        try:
            probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', **kw))
        except RuntimeError:
            check(probe.tap_confirm['y'] == before,
                  "%s refused and nothing changed" % what)
        else:
            raise AssertionError("%s accepted" % what)
    try:
        probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='ALL', CONFIRM_SPEED=61))
    except RuntimeError:
        check(probe.tap_confirm['x']['speed'] is None,
              "AXIS=ALL checks every axis before changing any")
    else:
        raise AssertionError("AXIS=ALL accepted 61 mm/s")
    for param in ('CONFIRM_SPEED', 'CONFIRM_RETRACT_SPEED',
                  'CONFIRM_RETRACT_DIST'):
        for bad in ('nan', '-nan', 'inf', '-inf'):
            before = dict(probe.tap_confirm['y'])
            try:
                probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', **{param: bad}))
            except RuntimeError as e:
                check(("finite" in str(e) or "range" in str(e)) and
                      probe.tap_confirm['y'] == before,
                      "%s=%s refused, nothing changed" % (param, bad))
            else:
                raise AssertionError("%s=%s accepted" % (param, bad))
    for kw in ({'CONFIRM_SPEED': 30}, {'CONFIRM_RETRACT_DIST': 5}):
        try:
            probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Z', **kw))
        except RuntimeError as e:
            check("AXIS=Z" in str(e), "AXIS=Z with CONFIRM_* is an error")
        else:
            raise AssertionError("AXIS=Z silently ignored %r" % (kw,))
    probe._tap_mode = 'axis_homing'
    try:
        probe.cmd_SET_ACCEL_PROBE(FakeGcmd(AXIS='Y', CONFIRM_SPEED=10))
    except RuntimeError:
        check(True, "refused while a homing move is active")
    else:
        raise AssertionError("changed the confirm speed mid-home")


if __name__ == '__main__':
    main()
