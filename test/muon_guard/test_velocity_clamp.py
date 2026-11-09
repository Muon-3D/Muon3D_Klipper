"""KAN-1733: outside dev mode a CLIENT's SET_VELOCITY_LIMIT or M204 cannot
raise the toolhead's limits above the printer config (max_velocity,
max_accel, square_corner_velocity). It is clamped, not refused: slicers
send these on every layer, and a refusal would stop the print. Macros and
dev mode are not clamped.

Drives the real GCodeDispatch, the real ToolHeadCommandHelper (toolhead.py)
and muon_guard through test_muon_guard's Rig.

    PYTHONUTF8=1 python3 test/muon_guard/test_velocity_clamp.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_muon_guard as tmg  # noqa: E402
import toolhead as toolhead_mod  # noqa: E402

FAILURES = tmg.FAILURES
check = tmg.check

# core/M1/core.cfg [printer]
MAX_V, MAX_A, SCV = 800., 20000., 7.


class Toolhead:
    """toolhead.ToolHead.set_max_velocities, values only."""
    def __init__(self):
        self.v, self.a, self.scv, self.mcr = MAX_V, MAX_A, SCV, .5

    def set_max_velocities(self, v, a, scv, mcr):
        if v is not None:
            self.v = v
        if a is not None:
            self.a = a
        if scv is not None:
            self.scv = scv
        if mcr is not None:
            self.mcr = mcr
        return self.v, self.a, self.scv, self.mcr


class HelperConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer


def rig(dev=False):
    r = tmg.Rig(dev=dev)
    r.cfg.settings['printer'] = {'max_velocity': MAX_V, 'max_accel': MAX_A,
                                 'square_corner_velocity': SCV}
    th = Toolhead()
    r.printer.add_object('toolhead', th)
    r.printer.set_rollover_info = lambda *a, **k: None
    for name in ('G4', 'M400', 'SET_VELOCITY_LIMIT', 'M204'):
        r.gcode.register_command(name, None)
    toolhead_mod.ToolHeadCommandHelper(HelperConfig(r.printer))
    getattr(r.guard, '_handle_connect', lambda: None)()
    r.gcode.gcode_handlers = r.gcode.ready_gcode_handlers
    r.th = th
    return r


def limits(r):
    return (r.th.v, r.th.a, r.th.scv)


def test_client_set_velocity_limit_is_clamped():
    r = rig()
    err = r.client('SET_VELOCITY_LIMIT VELOCITY=5000 ACCEL=100000 '
                   'SQUARE_CORNER_VELOCITY=50')
    check(err is None, "an over-limit SET_VELOCITY_LIMIT is not refused: %r"
          % (err,))
    check(limits(r) == (MAX_V, MAX_A, SCV), "clamped to the config maxima:"
          " %r" % (limits(r),))
    said = '\n'.join(r.out)
    check('max_velocity' in said and '800' in said,
          "the client is told it was clamped: %r" % (said,))


def test_each_limit_on_its_own():
    r = rig()
    r.client('SET_VELOCITY_LIMIT VELOCITY=900')
    check(r.th.v == MAX_V, "VELOCITY clamped: %r" % (r.th.v,))
    r.client('SET_VELOCITY_LIMIT ACCEL=20001')
    check(r.th.a == MAX_A, "ACCEL clamped: %r" % (r.th.a,))
    r.client('SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=7.5')
    check(r.th.scv == SCV, "SQUARE_CORNER_VELOCITY clamped: %r"
          % (r.th.scv,))
    r.client('set_velocity_limit velocity=1200')
    check(r.th.v == MAX_V, "lower case is clamped too: %r" % (r.th.v,))


def test_lower_values_pass_unchanged():
    r = rig()
    r.client('SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=12000 '
             'SQUARE_CORNER_VELOCITY=5 MINIMUM_CRUISE_RATIO=0.3')
    check((r.th.v, r.th.a, r.th.scv, r.th.mcr) == (300., 12000., 5., .3),
          "values under the maxima are applied as sent: %r"
          % ((r.th.v, r.th.a, r.th.scv, r.th.mcr),))
    check(not any('clamp' in o.lower() for o in r.out),
          "nothing said when nothing was clamped: %r" % (r.out,))
    # what the slicer fixtures send on each layer
    r.client('SET_VELOCITY_LIMIT ACCEL=12000 ACCEL_TO_DECEL=6000 '
             'SQUARE_CORNER_VELOCITY=7', 'file')
    check((r.th.a, r.th.scv) == (12000., 7.), "slicer line applied: %r"
          % ((r.th.a, r.th.scv),))


def test_m204():
    r = rig()
    r.client('M204 S50000')
    check(r.th.a == MAX_A, "M204 S clamped: %r" % (r.th.a,))
    r.client('M204 S5000')
    check(r.th.a == 5000., "M204 under the max applied: %r" % (r.th.a,))
    r.client('M204 P90000 T80000')
    check(r.th.a == MAX_A, "M204 P/T clamped: %r" % (r.th.a,))
    r.client('M204 P3000 T90000')
    check(r.th.a == 3000., "M204 P/T uses the lower, as toolhead.py: %r"
          % (r.th.a,))


def test_every_client_door():
    for door in ('webhooks', 'pty', 'file'):
        r = rig()
        r.client('SET_VELOCITY_LIMIT VELOCITY=2000', door)
        check(r.th.v == MAX_V, "clamped via %s: %r" % (door, r.th.v))


def test_follows_the_config():
    r = rig()
    r.cfg.settings['printer']['max_velocity'] = 500.
    r.client('SET_VELOCITY_LIMIT VELOCITY=600')
    check(r.th.v == 500., "the live config max is used: %r" % (r.th.v,))


def test_macros_and_internal_are_not_clamped():
    r = rig()
    r.from_macro('SET_VELOCITY_LIMIT VELOCITY=1000 ACCEL=30000')
    check((r.th.v, r.th.a) == (1000., 30000.), "a macro's call is its own:"
          " %r" % ((r.th.v, r.th.a),))
    r.internal('M204 S30000')
    check(r.th.a == 30000., "internal M204 not clamped: %r" % (r.th.a,))
    # a macro started by a client, calling SET_VELOCITY_LIMIT inside
    r.gcode.register_command(
        'MY_MACRO', lambda gcmd: r.gcode.run_script_from_command(
            'SET_VELOCITY_LIMIT VELOCITY=1100'))
    r.guard.rules['MY_MACRO'] = r.guard.rules['SET_VELOCITY_LIMIT']
    check(r.client('MY_MACRO') is None, "client starts the macro")
    check(r.th.v == 1100., "inside a client's macro: not clamped: %r"
          % (r.th.v,))
    # and the next client call is clamped again
    r.client('SET_VELOCITY_LIMIT VELOCITY=1100')
    check(r.th.v == MAX_V, "next client call clamped: %r" % (r.th.v,))


def test_dev_mode_is_not_clamped():
    r = rig(dev=True)
    r.client('SET_VELOCITY_LIMIT VELOCITY=1500 ACCEL=40000')
    check((r.th.v, r.th.a) == (1500., 40000.), "dev mode: as sent: %r"
          % ((r.th.v, r.th.a),))


def test_bad_values_still_error():
    r = rig()
    err = r.client('SET_VELOCITY_LIMIT VELOCITY=abc')
    check(err is not None, "a bad value is still an error: %r" % (err,))
    check(r.th.v == MAX_V, "and changes nothing")


def test_query_still_reports():
    r = rig()
    r.client('SET_VELOCITY_LIMIT')
    check(any('max_velocity: 800' in o for o in r.out),
          "a bare SET_VELOCITY_LIMIT still reports: %r" % (r.out,))


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        try:
            t()
        except Exception as e:
            import traceback
            traceback.print_exc()
            FAILURES.append("%s raised %s: %s" % (t.__name__,
                                                  type(e).__name__, e))
    if FAILURES:
        print("\n%d FAILURE(S)" % (len(FAILURES),))
        for f in FAILURES:
            print(" - " + f)
        return 1
    print("ok: %d tests" % (len(tests),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
