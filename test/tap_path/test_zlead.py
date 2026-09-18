"""ProbePointsHelper with ZLEAD: (1) with ZLEAD off the move trace is identical
to stock; (2) with ZLEAD on, Z reaches horizontal_move_z within the first
ZLEAD mm of every travel leg and is never below the post-tap clearance while
XY moves; (3) legs the no-go callback handles are left to it; (4) a leg
shorter than the lead is one segment; (5) ZLEAD without LIFTLAST is refused."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths
import collections
import sys
import types

ProbeResult = collections.namedtuple('ProbeResult', ['bed_x', 'bed_y', 'bed_z'])


def load(path, name):
    pkg = types.ModuleType('extras')
    pkg.__path__ = []
    sys.modules['extras'] = pkg
    mp = types.ModuleType('extras.manual_probe')
    mp.ProbeResult = ProbeResult
    mp.verify_no_manual_probe = lambda p: None
    sys.modules['extras.manual_probe'] = mp
    sys.modules['pins'] = types.ModuleType('pins')
    mod = types.ModuleType(name)
    mod.__dict__['__package__'] = 'extras'
    src = open(path, 'r', newline='').read()
    exec(compile(src, path, 'exec'), mod.__dict__)
    return mod


class Err(Exception):
    pass


class Gcmd:
    def __init__(self, **p):
        self.p = p

    def get(self, k, d=None):
        return self.p.get(k, d)

    def get_float(self, k, d=None, minval=None, **kw):
        v = self.p.get(k, d)
        return d if v is None else float(v)

    def get_int(self, k, d=None, **kw):
        return int(self.p.get(k, d))

    def error(self, m):
        return Err(m)


class Toolhead:
    def __init__(self):
        self.pos = [100., 60., 10.]
        self.trace = []

    def get_position(self):
        return list(self.pos)

    def get_last_move_time(self):
        return 0.

    def manual_move(self, coord, speed):
        coord = list(coord) + [None] * (3 - len(coord))
        new = [c if c is not None else p for c, p in zip(coord, self.pos)]
        self.trace.append(('move', tuple(round(v, 4) for v in new), speed))
        self.pos = new


class Session:
    def __init__(self, th, liftlast, plate):
        self.th, self.liftlast, self.plate, self.res = th, liftlast, plate, []

    def run_probe(self, gcmd):
        x, y = self.th.pos[0], self.th.pos[1]
        z = self.plate(x, y)
        self.th.trace.append(('tap', (x, y, z)))
        self.th.pos[2] = z + (0.4 if self.liftlast else 0.)
        self.res.append(ProbeResult(x, y, z))

    def pull_probed_results(self):
        return self.res

    def end_probe_session(self):
        pass


class Probe:
    def __init__(self, th, liftlast, plate):
        self.th, self.liftlast, self.plate = th, liftlast, plate

    def get_probe_params(self, gcmd):
        return {'lift_speed': 10.}

    def get_offsets(self, gcmd):
        return (0., 0., 0.)

    def start_probe_session(self, gcmd):
        return Session(self.th, self.liftlast, self.plate)


class Printer:
    config_error = Exception

    def __init__(self, th, probe):
        self.th, self.probe = th, probe

    def lookup_object(self, n, default=None):
        return {'toolhead': self.th, 'probe': self.probe, 'gcode': None}[n]


class Config:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'bed_mesh'

    def get(self, k, d=None):
        return d

    def getfloat(self, k, d=None, **kw):
        return d

    def getlists(self, *a, **kw):
        return None


def notch_callback(th):
    # emulate bed_mesh._move_to_next_probe for a no-go box x75-125, y0-34
    def cb(curpos, nextpos, speed, hmz, lift_speed):
        x0, y0 = curpos[0], curpos[1]
        x1, y1 = nextpos[0], nextpos[1]
        crosses = min(y0, y1) < 34 and max(x0, x1) > 75 and min(x0, x1) < 125
        if not crosses:
            return False
        if curpos[2] < 6 - 1e-6:
            th.manual_move([None, None, 6.], lift_speed)
        th.manual_move(nextpos, speed)
        th.manual_move([None, None, hmz], lift_speed)
        return True
    return cb


POINTS = [(10., 88.5), (70., 88.5), (130., 88.5), (190., 88.5),
          (190., 20.), (60., 20.), (60., 25.)]


def plate(x, y):
    return -0.2 - 0.005 * (x - 10) - 0.004 * y   # gentle slope


def run(mod, liftlast, **params):
    th = Toolhead()
    probe = Probe(th, liftlast, plate)
    pr = Printer(th, probe)
    h = mod.ProbePointsHelper(Config(pr), lambda results: None,
                              default_points=POINTS)
    h.horizontal_move_z = 2.
    h.set_travel_callback(notch_callback(th))
    g = Gcmd(HORIZONTAL_MOVE_Z=2., LIFTLAST=1 if liftlast else 0, **params)
    h.start_probe(g)
    return th.trace


stock = load(_paths.stock('klippy/extras/probe.py'), 's')
zl = load(_paths.patched('klippy/extras/probe.py'), 'z')
bad = 0


def check(name, ok):
    global bad
    print('  %-70s %s' % (name, 'PASS' if ok else 'FAIL'))
    bad += (not ok)


def taps(t):
    return [e for e in t if e[0] == 'tap']


check('ZLEAD off: trace identical to stock',
      run(stock, False) == run(zl, False))
check('ZLEAD off + LIFTLAST: same taps as stock',
      taps(run(zl, True)) == taps(run(stock, False)))
t = run(zl, True, ZLEAD=10.)
check('ZLEAD on: same taps as stock', taps(t) == taps(run(stock, False)))
# safety envelope: every XY move keeps >= 0.4 mm above the plate along its
# whole length, and Z never descends while XY moves
ok = True
prev = None
combined = notch = 0
for e in t:
    if e[0] == 'move' and prev is not None:
        if prev[0] == 'tap':
            # the leg starts where the tap left the nozzle: 0.4 above contact
            x0, y0, z0 = prev[1][0], prev[1][1], prev[1][2] + 0.4
        else:
            x0, y0, z0 = prev[1]
        x1, y1, z1 = e[1]
        if (x0, y0) != (x1, y1):
            if z1 != z0:
                combined += 1
            for i in range(21):
                s = i / 20.
                x = x0 + (x1 - x0) * s
                y = y0 + (y1 - y0) * s
                z = z0 + (z1 - z0) * s
                if z < plate(x, y) + 0.4 - 1e-9:
                    ok = False
            if z1 < z0 - 1e-9:
                ok = False
            if max(z0, z1) >= 5.9:
                notch += 1
    prev = e
check('every XY leg keeps >= 0.4 mm clearance along its whole length', ok)
check('Z reaches 2 mm within the lead on ordinary legs (combined segments)',
      combined >= 4)
check('notch legs still lifted to 6 by the callback', notch >= 1)
last_tap = max(i for i, e in enumerate(t) if e[0] == 'tap')
short = [e for e in t[:last_tap] if e[0] == 'move' and e[1][:2] == (60., 25.)]
check('leg shorter than ZLEAD is one segment at hz',
      len(short) == 1 and short[0][1][2] == 2.0)
try:
    run(zl, False, ZLEAD=10.)
    check('ZLEAD without LIFTLAST refused', False)
except Err as ex:
    check('ZLEAD without LIFTLAST refused (%s)' % ex, True)
print('stock  :', ' '.join(('T' if e[0] == 'tap' else 'M')
                           for e in run(stock, False)))
print('zlead  :', ' '.join(('T' if e[0] == 'tap' else 'M') for e in t))
for e in t[-10:]:
    print('   ', e)
print('all PASS' if not bad else '%d FAIL' % bad)
sys.exit(bad)
