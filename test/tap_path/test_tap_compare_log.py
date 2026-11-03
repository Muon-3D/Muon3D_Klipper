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
    line = fmt('y', {'stepper_y': 55360}, {'stepper_y': 55616}, sd, 0.8)
    print(line)
    check('\n' not in line, "a single log line")
    check('y tap compare' in line, "names the axis")
    check('prev stepper_y=173.000 mm' in line, "first trigger position, mm")
    check('new stepper_y=173.800 mm' in line, "second trigger position, mm")
    check('spread 0.800mm' in line, "the spread")
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
    print("tap compare log tests passed")


if __name__ == '__main__':
    main()
