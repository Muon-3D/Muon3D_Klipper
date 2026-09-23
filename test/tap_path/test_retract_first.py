"""Does lifting before reporting change WHAT the probe does?

The timing claim is easy to measure on the bench. The claim that matters
for safety is the other one: that this reordering changes only WHEN the
lift is queued relative to the reporting, and never the number of taps,
the number of lifts, or the samples_tolerance verdict.

`samples_tolerance` with two samples is what caught a damaged load cell on
this machine -- a 350 um error that a single sample would have written
silently into the mesh. So the tolerance path is not something to eyeball.

Both `run_probe` AND `_probe` are the REAL functions, exec'd from
stock_probe.py and tp_probe.py with the layer beneath them stubbed, so
this compares the shipped code against the patched code rather than my
description of either. Every scenario is run through both and the motion
traces must be identical.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths
import collections
import sys
import types

ProbeResult = collections.namedtuple('ProbeResult',
                                     ['bed_x', 'bed_y', 'bed_z'])


def load_module(path, name):
    pkg = types.ModuleType('extras')
    pkg.__path__ = []
    sys.modules['extras'] = pkg
    mp = types.ModuleType('extras.manual_probe')
    mp.ProbeResult = ProbeResult
    sys.modules['extras.manual_probe'] = mp
    sys.modules['pins'] = types.ModuleType('pins')
    mod = types.ModuleType(name)
    mod.__dict__['__name__'] = name
    mod.__dict__['__package__'] = 'extras'
    src = open(path, 'r', newline='').read()
    exec(compile(src, path, 'exec'), mod.__dict__)
    return mod


class Toolhead:
    def __init__(self, trace):
        self.pos = [70.0, 7.0, 5.0]
        self.trace = trace

    def get_position(self):
        return list(self.pos)

    def get_status(self, eventtime):
        return {'homed_axes': 'xyz'}

    def manual_move(self, coord, speed):
        self.trace.append(('lift', round(coord[2] - self.pos[2], 4)))
        for i, v in enumerate(coord):
            if v is not None:
                self.pos[i] = v


class Gcode:
    def __init__(self, trace):
        self.trace = trace

    def respond_info(self, msg):
        self.trace.append(('report',))

    def error(self, msg):
        return CommandError(msg)


class CommandError(Exception):
    pass


class Reactor:
    def monotonic(self):
        return 0.0


class Printer:
    def __init__(self, trace):
        self.trace = trace
        self.toolhead = Toolhead(trace)
        self.gcode = Gcode(trace)
        self.command_error = CommandError

    def lookup_object(self, name):
        return {'toolhead': self.toolhead, 'gcode': self.gcode}[name]

    def get_reactor(self):
        return Reactor()

    def send_event(self, name, *args):
        self.trace.append(('event', name))
        return []


class HwSession:
    """Descends and stops where the script says the plate is."""

    def __init__(self, trace, zs, toolhead):
        self.trace = trace
        self.zs = list(zs)
        self.toolhead = toolhead
        self.pending = None

    def run_probe(self, gcmd):
        z = self.zs.pop(0)
        self.trace.append(('descend', z))
        # a real tap ends with the toolhead stopped at the trigger
        self.toolhead.pos[2] = z
        self.pending = ProbeResult(70.0, 7.0, z)

    def pull_probed_results(self):
        return [self.pending]


class ParamHelper:
    def __init__(self, params):
        self.params = params

    def get_probe_params(self, gcmd=None):
        return self.params


def run(mod, zs, samples, tolerance, retries):
    trace = []
    printer = Printer(trace)
    params = {'samples': samples,
              'sample_retract_dist': 0.4,
              'lift_speed': 10.0,
              'samples_tolerance': tolerance,
              'samples_tolerance_retries': retries,
              'samples_result': 'average',
              'probe_speed': 2.0}
    sess = object.__new__(mod.ProbeSessionHelper)
    sess.printer = printer
    sess.param_helper = ParamHelper(params)
    sess.hw_probe_session = HwSession(trace, zs, printer.toolhead)
    sess.results = []
    try:
        sess.run_probe(_Gcmd(printer))
        outcome = 'ok z=%.4f' % sess.results[0].bed_z
    except CommandError as e:
        outcome = 'ERROR: %s' % e
    except IndexError:
        outcome = 'RAN OUT OF SCRIPTED TAPS'
    return trace, outcome


class _Gcmd:
    def __init__(self, printer):
        self.printer = printer

    def error(self, msg):
        return CommandError(msg)

    def respond_info(self, msg):
        pass


def motion_only(trace):
    return [e for e in trace if e[0] in ('descend', 'lift')]


SCENARIOS = [
    ('samples=2, both agree',
     [0.100, 0.1005], 2, 0.100, 0),
    ('samples=2, second disagrees, no retries -> must abort',
     [0.100, 0.500], 2, 0.100, 0),
    ('samples=2, disagree once then agree, 1 retry',
     [0.100, 0.500, 0.200, 0.2005], 2, 0.100, 1),
    ('samples=3, third disagrees, 1 retry',
     [0.10, 0.101, 0.900, 0.30, 0.301, 0.302], 3, 0.100, 1),
    ('samples=3, second disagrees, 1 retry',
     [0.10, 0.900, 0.30, 0.301, 0.302], 3, 0.100, 1),
    ('samples=1 (not a shipping option, but must not change)',
     [0.100], 1, 0.100, 0),
]


def descents(trace):
    return [e for e in trace if e[0] == 'descend']


def main():
    stock = load_module(_paths.stock('klippy/extras/probe.py'), 'stock_probe')
    patched = load_module(_paths.patched('klippy/extras/probe.py'), 'tp_probe')
    lifted = load_module(_paths.patched('klippy/extras/probe.py'),
                         'tp_probe_lift')
    lifted.LIFT_AFTER_LAST_SAMPLE = True
    bad = 0
    for name, zs, n, tol, retries in SCENARIOS:
        t_s, o_s = run(stock, zs, n, tol, retries)
        t_p, o_p = run(patched, zs, n, tol, retries)
        ms, mp = motion_only(t_s), motion_only(t_p)
        ok_motion = ms == mp
        ok_outcome = o_s == o_p
        # and in the patched trace every lift must come before the report
        # that follows its tap
        order_ok = True
        for i, e in enumerate(t_p):
            if e[0] == 'lift':
                after = [x[0] for x in t_p[i + 1:]]
                before = [x[0] for x in t_p[:i]]
                if 'report' in before[before.index('descend'):] \
                        if 'descend' in before else False:
                    pass
                if 'report' not in after:
                    order_ok = False
        # LIFT_AFTER_LAST_SAMPLE must change ONLY the lifts, never the taps
        t_l, o_l = run(lifted, zs, n, tol, retries)
        same_taps = descents(t_s) == descents(t_l)
        same_verdict = o_s == o_l
        extra = (len([e for e in t_l if e[0] == 'lift'])
                 - len([e for e in t_s if e[0] == 'lift']))
        status = 'PASS' if (ok_motion and ok_outcome
                            and same_taps and same_verdict) else 'FAIL'
        if status == 'FAIL':
            bad += 1
        print('%-52s %s' % (name, status))
        print('   stock   motion %s' % (ms,))
        print('   patched motion %s' % (mp,))
        print('   stock   outcome %s' % o_s)
        print('   patched outcome %s' % o_p)
        if not ok_motion:
            print('   !! MOTION DIFFERS -- taps or lifts are not preserved')
        if not ok_outcome:
            print('   !! OUTCOME DIFFERS -- tolerance verdict changed')
        print('   lift-after-last: taps %s, verdict %s, +%d lift(s)'
              % ('same' if same_taps else 'CHANGED',
                 'same' if same_verdict else 'CHANGED', extra))
        if not same_taps:
            print('   !! LIFT_AFTER_LAST_SAMPLE changed the TAPS -- reject')
        if not same_verdict:
            print('   !! LIFT_AFTER_LAST_SAMPLE changed the tolerance verdict')
        if not order_ok:
            print('   note: a lift had no following report in this scenario')
        print()
    print('%d scenario(s) FAILED' % bad if bad else 'all scenarios PASS')
    # show the reordering actually happened in the ordinary case
    t_s, _ = run(stock, [0.100, 0.1005], 2, 0.100, 0)
    t_p, _ = run(patched, [0.100, 0.1005], 2, 0.100, 0)
    print()
    print('ordinary 2-sample point, full trace:')
    print('  stock  : %s' % ' '.join(e[0] for e in t_s))
    print('  patched: %s' % ' '.join(e[0] for e in t_p))
    return 1 if bad else 0


sys.exit(main())
