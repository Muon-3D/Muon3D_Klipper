"""muon_jog: a live drag that reverses within a known distance.

Drives the real klippy/extras/muon_jog.py against a simulated toolhead whose
committed batches run as trapezoids ending at rest (what toolhead
get_last_move_time does to the lookahead; checked against the real
LookAheadQueue below), with the MCU clock equal to the reactor clock.

    PYTHONUTF8=1 python3 test/muon_jog/test_muon_jog.py

Proves: the queued path never exceeds max_ahead, so a reversal, STOP, a lost
client or a refusal stops the motion within it; axis limits, z_min, the notch
below its height and the contact guard hold; it refuses while printing,
unhomed, or for mixed axes; another move or homing ends it. Expected values
are written out here, not imported.
"""
import configparser
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

from extras import muon_jog  # noqa: E402
from extras import muon_guard  # noqa: E402
# The real ContactGuard and its fakes (test/contact_guard)
sys.path.insert(0, os.path.join(ROOT, 'test', 'contact_guard'))
import test_height_guard as hg  # noqa: E402
try:
    import serial  # noqa: F401
except ImportError:
    # toolhead imports mcu imports pyserial; nothing here opens a port
    import types
    sys.modules['serial'] = types.ModuleType('serial')
import toolhead as toolhead_mod  # noqa: E402

CORE_CFG = os.path.join(ROOT, 'core', 'M1', 'core.cfg')
NEVER = 9999999999999999.
BUFFER_TIME_START = 0.250   # toolhead.py
LIMITS = ((0., -10., -4.), (200.1, 174.1, 160.))   # core/M1/core.cfg
NOTCH = (75., 0., 125., 34.)
TRAVERSE_Z = 6.


class CommandError(Exception):
    pass


class Reactor:
    NOW = 0.
    NEVER = NEVER

    def __init__(self):
        self.now = 100.
        self.timers = []

    def monotonic(self):
        return self.now

    def register_timer(self, cb, waketime=NEVER):
        t = [cb, waketime]
        self.timers.append(t)
        return t

    def update_timer(self, t, waketime):
        t[1] = waketime

    def run_until(self, end):
        while True:
            t = min(self.timers, key=lambda t: t[1])
            wake = max(t[1], self.now)
            if wake > end:
                self.now = end
                return
            self.now = wake
            t[1] = t[0](self.now)


class Mutex:
    locked = False

    def test(self):
        return self.locked

    def __enter__(self):
        assert not self.locked
        self.locked = True

    def __exit__(self, *a):
        self.locked = False


class Gcode:
    def __init__(self):
        self.mutex = Mutex()
        self.commands = {}

    def register_command(self, name, fn, desc=None):
        self.commands[name] = fn

    def get_mutex(self):
        return self.mutex


class Kin:
    max_z_velocity = 15.

    homed = 'xyz'

    def get_status(self, eventtime):
        return {'axis_minimum': LIMITS[0] + (0.,),
                'axis_maximum': LIMITS[1] + (0.,), 'homed_axes': self.homed}


class GuardKin(Kin):
    """homed_axes is the real guard rig's, so what a test does to the
    guard's toolhead is what the jog sees too"""
    def __init__(self, rig):
        self.rig = rig

    @property
    def homed(self):
        return self.rig.printer.toolhead.homed_axes

    @homed.setter
    def homed(self, v):
        self.rig.printer.toolhead.homed_axes = v


class Toolhead:
    """Commanded position plus a list of committed batches, each a
    trapezoid from rest to rest (the acceleration is the Z one if it
    moves Z)."""
    def __init__(self, pos):
        self.kin = Kin()
        self.commanded = list(pos) + [0.]
        self.pending = []
        self.batches = []
        self.print_time = 0.
        self.clock = None

    def get_kinematics(self):
        return self.kin

    def get_max_velocity(self):
        return 800., 20000.

    def get_position(self):
        return list(self.commanded)

    def move(self, newpos, speed):
        self.pending.append((list(self.commanded[:3]), list(newpos[:3]),
                             speed))
        self.commanded = list(newpos[:3]) + [0.]

    def get_last_move_time(self):
        if self.pending:
            p0, p1 = self.pending[0][0], self.pending[-1][1]
            speed = self.pending[0][2]
            d = math.dist(p0, p1)
            accel = 1000. if abs(p1[2] - p0[2]) > 1e-9 else 20000.
            v = min(speed, math.sqrt(d * accel))
            dur = 2. * v / accel + max(0., d - v * v / accel) / v
            start = max(self.print_time, self.clock() + BUFFER_TIME_START)
            self.batches.append((start, dur, p0, p1, v, accel))
            self.print_time = start + dur
            self.pending = []
        return self.print_time

    def position_at(self, t):
        pos = None
        for start, dur, p0, p1, v, a in self.batches:
            if t < start:
                break
            d = math.dist(p0, p1)
            tt = min(t - start, dur)
            ta = v / a
            if tt < ta:
                s = .5 * a * tt * tt
            elif tt < dur - ta:
                s = .5 * v * ta + v * (tt - ta)
            else:
                td = dur - tt
                s = d - .5 * a * td * td
            f = s / d if d else 1.
            pos = [p0[i] + (p1[i] - p0[i]) * f for i in range(3)]
        return pos


class GcodeMove:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.last_position = toolhead.get_position()
        self.base_position = [0., 0., 0., 0.]
        self.refuse = None

    def move_with_transform(self, newpos, speed):
        if self.refuse:
            raise CommandError(self.refuse)
        self.toolhead.move(newpos, speed)

    def reset_last_position(self):
        self.last_position = self.toolhead.get_position()


class PrintStats:
    state = 'standby'

    def get_status(self, eventtime):
        return {'state': self.state}


class MCU:
    def __init__(self, reactor):
        self.reactor = reactor

    def estimated_print_time(self, eventtime):
        return eventtime


class Printer:
    command_error = CommandError

    def __init__(self, pos):
        self.reactor = Reactor()
        self.gcode = Gcode()
        self.toolhead = Toolhead(pos)
        self.toolhead.clock = self.reactor.monotonic
        self.gcode_move = GcodeMove(self.toolhead)
        self.print_stats = PrintStats()
        self.objects = {'gcode': self.gcode, 'toolhead': self.toolhead,
                        'gcode_move': self.gcode_move,
                        'mcu': MCU(self.reactor),
                        'print_stats': self.print_stats}
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        for cb in self.handlers.get(event, []):
            cb(*args)


class Section:
    def __init__(self, printer, values, sections):
        self.printer, self.values, self.sections = printer, values, sections

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default=None, **kw):
        if name in self.values:
            return float(self.values[name])
        if default is None:
            raise KeyError(name)
        return default

    def getfloatlist(self, name, default=None, count=None):
        if name not in self.values:
            return default
        return [float(v) for v in self.values[name].split(',')]

    def has_section(self, name):
        return name in self.sections

    def getsection(self, name):
        return Section(self.printer, self.sections[name], self.sections)


def core_cfg():
    cp = configparser.RawConfigParser(strict=False)
    with open(CORE_CFG, encoding='utf-8') as f:
        cp.read_file(f)
    return cp


def make(pos=(100., 100., 20.), notch=True, rig=None):
    printer = Printer(pos)
    if rig is not None:
        printer.toolhead.kin = GuardKin(rig)
        printer.objects['contact_guard'] = rig.guard
    cp = core_cfg()
    sections = {'muon_jog': dict(cp.items('muon_jog'))}
    if notch:
        sections['bed_mesh'] = {k: v for k, v in cp.items('bed_mesh')
                                if k.startswith('no_go_')}
    jog = muon_jog.MuonJog(Section(printer, sections['muon_jog'], sections))
    printer.send_event('klippy:connect')
    return printer, jog


class Gcmd:
    def __init__(self, params):
        self.params = params

    def error(self, msg):
        return CommandError(msg)

    def get_float(self, name, default=None, above=None):
        return float(self.params[name]) if name in self.params else default

    def get_int(self, name, default=0, minval=None, maxval=None):
        return int(self.params[name]) if name in self.params else default


def jog(printer, **params):
    printer.gcode.commands['MUON_JOG'](Gcmd(params))


class Watch:
    """Runs the reactor in small steps, recording the physical position
    and the worst queued path (commanded minus physical)."""
    def __init__(self, printer, axes):
        self.p, self.idx = printer, axes
        self.trace = []
        self.worst_ahead = 0.

    def pos(self):
        t = self.p.reactor.now
        got = self.p.toolhead.position_at(t)
        return got if got is not None else self.p.toolhead.commanded[:3]

    def run(self, dt, refresh=None, step=0.01):
        end = self.p.reactor.now + dt
        next_refresh = self.p.reactor.now
        while self.p.reactor.now < end - 1e-9:
            if refresh is not None and self.p.reactor.now >= next_refresh:
                refresh()
                next_refresh += 0.1
            self.p.reactor.run_until(self.p.reactor.now + step)
            here = self.pos()
            ahead = math.sqrt(sum((self.p.toolhead.commanded[i] - here[i])
                                  ** 2 for i in self.idx))
            self.worst_ahead = max(self.worst_ahead, ahead)
            self.trace.append(here)


FAILS = []


def check(cond, what):
    print(('ok   ' if cond else 'FAIL ') + what)
    if not cond:
        FAILS.append(what)


def test_reversal_z():
    p, j = make((100., 100., 20.))
    w = Watch(p, (2,))
    w.run(2., refresh=lambda: jog(p, Z='150', SPEED='10'))
    at = w.pos()[2]
    check(at > 30., "Z drag is moving up at full speed (%.1f mm)" % at)
    n = len(w.trace)
    w.run(2., refresh=lambda: jog(p, Z='0', SPEED='10'))
    peak = max(t[2] for t in w.trace[n:])
    check(peak - at <= 4. + 1e-6,
          "Z reversal overshoots %.2f mm <= max_ahead_z 4" % (peak - at))
    back = next(i for i, t in enumerate(w.trace[n:]) if t[2] < at) * 0.01
    check(back <= 1.0,
          "Z is back below the reversal point %.2f s later (<= 1 s)" % back)
    check(w.worst_ahead <= 4. + 1e-6,
          "Z queued path never above 4 mm (worst %.2f)" % w.worst_ahead)


def test_reversal_xy():
    p, j = make((20., 60., 20.))
    w = Watch(p, (0, 1))
    w.run(1., refresh=lambda: jog(p, X='180', Y='60', SPEED='100'))
    at = w.pos()
    n = len(w.trace)
    w.run(1., refresh=lambda: jog(p, X='20', Y='60', SPEED='100'))
    peak = max(t[0] for t in w.trace[n:])
    check(peak - at[0] <= 15. + 1e-6,
          "XY reversal overshoots %.2f mm <= max_ahead_xy 15"
          % (peak - at[0]))
    check(w.worst_ahead <= 15. + 1e-6,
          "XY queued path never above 15 mm (worst %.2f)" % w.worst_ahead)


def test_speed_held_to_bound():
    p, j = make()
    jog(p, Z='150', SPEED='1000')
    s = j.session['speed']
    check(abs(s - 4. / 0.4) < 1e-9 and s <= 15.,
          "a Z speed the bound cannot hold is lowered (%.1f mm/s)" % s)


def test_axis_limits():
    p, j = make((100., 100., 150.))
    w = Watch(p, (2,))
    w.run(3., refresh=lambda: jog(p, Z='500', SPEED='10'))
    check(abs(w.pos()[2] - 160.) < 1e-6 and max(t[2] for t in w.trace)
          <= 160. + 1e-6, "Z stops at axis maximum 160")
    check('limit' in j.session['clamp'], "the clamp says why")
    p, j = make((100., 100., 5.), notch=False)
    w = Watch(p, (2,))
    w.run(3., refresh=lambda: jog(p, Z='-50', SPEED='10'))
    check(min(t[2] for t in w.trace) >= -1e-6,
          "without a [contact_guard] Z stops at z_min 0, not -4")
    p, j = make((190., 100., 20.))
    w = Watch(p, (0, 1))
    w.run(2., refresh=lambda: jog(p, X='500', Y='-80', SPEED='100'))
    check(max(t[0] for t in w.trace) <= 200.1 + 1e-6
          and min(t[1] for t in w.trace) >= -10. - 1e-6,
          "XY stops at X max 200.1 and Y min -10")


def test_notch():
    p, j = make((50., 10., 3.))
    w = Watch(p, (0, 1))
    w.run(2., refresh=lambda: jog(p, X='150', Y='10', SPEED='100'))
    check(max(t[0] for t in w.trace) <= NOTCH[0] + 1e-6,
          "below the notch height XY stops at the notch edge (x %.2f)"
          % max(t[0] for t in w.trace))
    check('notch' in j.session['clamp'], "and says so")
    p, j = make((50., 10., 8.))
    w = Watch(p, (0, 1))
    w.run(4., refresh=lambda: jog(p, X='150', Y='10', SPEED='100'))
    check(abs(w.pos()[0] - 150.) < 1e-6, "above Z6 XY crosses the notch")
    p, j = make((100., 10., 12.))
    w = Watch(p, (2,))
    w.run(2., refresh=lambda: jog(p, Z='0', SPEED='10'))
    check(abs(min(t[2] for t in w.trace) - TRAVERSE_Z) < 1e-6,
          "over the notch Z stops at Z6")
    p, j = make((100., 10., 3.))
    jog(p, X='100', Y='60')
    w = Watch(p, (0, 1))
    w.run(1.)
    check(abs(w.pos()[1] - 10.) < 1e-6, "inside the notch below Z6: no XY")


def test_deadman_and_stop():
    p, j = make()
    jog(p, Z='150', SPEED='10')
    w = Watch(p, (2,))
    w.run(0.4)
    check(j.session is not None, "alive within keepalive")
    w.run(0.2)
    check(j.session is None and 'no update' in j.reason,
          "no update for 0.5 s ends the jog: %r" % j.reason)
    at = w.pos()[2]
    w.run(1.)
    check(w.pos()[2] - at <= 4. + 1e-6, "and stops within 4 mm")
    p, j = make()
    w = Watch(p, (2,))
    w.run(1., refresh=lambda: jog(p, Z='150', SPEED='10'))
    jog(p, STOP='1')
    at = w.pos()[2]
    w.run(1.)
    check(j.session is None and j.reason == 'stopped'
          and w.pos()[2] - at <= 4. + 1e-6, "STOP=1 stops within 4 mm")


def test_final_arrives():
    p, j = make()
    jog(p, Z='25', SPEED='10', FINAL='1')
    w = Watch(p, (2,))
    w.run(2.)
    check(abs(w.pos()[2] - 25.) < 1e-6 and j.reason == 'arrived',
          "FINAL=1 goes to the target without keepalive and ends")


def test_refusals():
    p, j = make()
    p.gcode_move.refuse = ("Nozzle is touching the plate: lift Z before "
                           "moving X or Y")
    jog(p, X='120', Y='100')
    p.reactor.run_until(p.reactor.now + 0.1)
    check(j.session is None and 'Nozzle is touching' in j.reason,
          "a contact-guard refusal ends the jog with its text")
    check(p.gcode_move.last_position == p.toolhead.get_position(),
          "and resets gcode_move to the toolhead")
    for setup, params, word in (
            (lambda p: setattr(p.print_stats, 'state', 'printing'),
             {'Z': '30'}, 'printing'),
            (lambda p: setattr(p.toolhead.kin, 'homed', 'xy'),
             {'Z': '30'}, 'Home XYZ'),
            (lambda p: setattr(p.toolhead.kin, 'homed', 'z'),
             {'Z': '30'}, 'Home XYZ'),
            (lambda p: setattr(p.toolhead.kin, 'homed', 'xz'),
             {'Z': '30'}, 'Home XYZ'),
            (lambda p: setattr(p.toolhead.kin, 'homed', 'xy'),
             {'X': '30'}, None),
            (lambda p: setattr(p.toolhead.kin, 'homed', 'z'),
             {'X': '30'}, 'Home XY'),
            (lambda p: None, {'X': '1', 'Z': '30'}, 'not both'),
            (lambda p: None, {}, 'needs a target')):
        p, j = make()
        setup(p)
        try:
            jog(p, **params)
            got = None
        except CommandError as e:
            got = str(e)
        if word is None:
            check(got is None and j.session is not None,
                  "accepted %r" % (params,))
            continue
        check(got is not None and word in got and j.session is None,
              "refused %r: %r" % (params, got))


def test_takeover_and_homing():
    p, j = make()
    jog(p, Z='150', SPEED='10')
    p.reactor.run_until(p.reactor.now + 0.2)
    p.gcode_move.last_position[0] += 5.
    p.reactor.run_until(p.reactor.now + 0.1)
    check(j.session is None and 'another move' in j.reason,
          "a G1 from elsewhere ends the jog")
    p, j = make()
    jog(p, Z='150', SPEED='10')
    p.send_event('homing:home_rails_begin', None, [])
    check(j.session is None and 'homing' in j.reason, "homing ends the jog")
    p, j = make()
    jog(p, Z='150', SPEED='10')
    p.gcode.mutex.locked = True
    n = len(p.toolhead.batches)
    p.reactor.run_until(p.reactor.now + 0.3)
    check(len(p.toolhead.batches) == n,
          "nothing is queued while another command holds the mutex")


def test_real_lookahead_ends_at_rest():
    # The premise: committing the lookahead without lazy plans the last
    # move to rest, so nothing runs past the committed batches.
    th = type('T', (), {'max_accel': 1000., 'junction_deviation': 0.01,
                        'max_velocity': 800., 'mcr_pseudo_accel': 500.,
                        'extra_axes': []})()
    q = toolhead_mod.LookAheadQueue()
    pos = (0., 0., 10., 0.)
    for i in range(4):
        end = (0., 0., 10. + i + 1., 0.)
        q.add_move(toolhead_mod.Move(th, pos, end, 10.))
        pos = end
    moves = q.flush(lazy=False)
    check(len(moves) == 4 and moves[-1].end_v == 0.,
          "real LookAheadQueue.flush(lazy=False) ends the batch at rest")


def guarded(pos, mesh=False, homed=True):
    rig = hg.Rig(z=pos[2], homed=homed, mesh=mesh)
    p, j = make(pos, notch=False, rig=rig)
    return rig, p, j


def refused(p, **params):
    try:
        jog(p, **params)
    except CommandError as e:
        return str(e)
    return None


def test_guard_z_floor():
    # No bed mesh: contact_z 0.1 + unmeshed_margin 2.5
    rig, p, j = guarded((100., 100., 20.))
    w = Watch(p, (2,))
    w.run(4., refresh=lambda: jog(p, Z='-50', SPEED='10'))
    low = min(t[2] for t in w.trace)
    check(abs(low - 2.6) < 1e-6 and abs(w.pos()[2] - 2.6) < 1e-6,
          "no mesh: Z stops at the guard floor 2.6 (low %.3f)" % low)
    check('Z2.6' in j.session['clamp'], "and says why: %r"
          % j.session['clamp'])
    # A probed mesh: contact_z 0.1
    rig, p, j = guarded((100., 100., 20.), mesh=True)
    w = Watch(p, (2,))
    w.run(4., refresh=lambda: jog(p, Z='-50', SPEED='10'))
    low = min(t[2] for t in w.trace)
    check(abs(low - 0.1) < 1e-6 and abs(w.pos()[2] - 0.1) < 1e-6,
          "with a mesh: Z stops at 0.1 (low %.3f)" % low)
    # The floor never holds the plate back from rising
    rig, p, j = guarded((100., 100., 1.0))
    w = Watch(p, (2,))
    w.run(1., refresh=lambda: jog(p, Z='-50', SPEED='10'))
    check(min(t[2] for t in w.trace) >= 1.0 - 1e-6
          and abs(w.pos()[2] - 1.0) < 1e-6,
          "started below the floor: down goes nowhere")
    w.run(3., refresh=lambda: jog(p, Z='10', SPEED='10'))
    check(abs(w.pos()[2] - 10.) < 1e-6,
          "started below the floor: up is allowed (Z %.2f)" % w.pos()[2])
    rig, p, j = guarded((100., 100., 0.05), mesh=True)
    w = Watch(p, (2,))
    w.run(2., refresh=lambda: jog(p, Z='5', SPEED='10'))
    check(abs(w.pos()[2] - 5.) < 1e-6,
          "started below the mesh floor 0.1: up is allowed")


def test_guard_start_needs_the_height():
    rig, p, j = guarded((100., 100., 20.), homed=False)
    # Z homed flag set but never trusted (a fake home)
    rig.printer.toolhead.homed_axes = 'xyz'
    for params in ({'Z': '30'}, {'X': '120', 'Y': '100'}):
        got = refused(p, **params)
        check(got == hg.HOME_FIRST and j.session is None,
              "height unknown refuses %r with the guard text" % (params,))
    # A lift is not enough for a live drag
    rig, p, j = guarded((100., 100., 20.), homed=False)
    hg.g28_xy(rig)
    rig.printer.toolhead.homed_axes = 'xyz'
    st = rig.status()
    check(st['lift_clear'] and not st['z_known'],
          "the guard is in the lift-only state")
    for params in ({'Z': '30'}, {'X': '120', 'Y': '100'}):
        got = refused(p, **params)
        check(got == hg.HOME_FIRST and j.session is None,
              "lift-only refuses %r too" % (params,))
    # Known: starts
    rig, p, j = guarded((100., 100., 20.))
    check(refused(p, Z='30') is None and j.session is not None,
          "height known: the jog starts")


def test_guard_height_lost_ends_the_jog():
    for axes, params in (('Z', {'Z': '150'}),
                         ('XY', {'X': '180', 'Y': '100'})):
        rig, p, j = guarded((100., 100., 20.))
        w = Watch(p, (2,) if axes == 'Z' else (0, 1))
        w.run(0.5, refresh=lambda: jog(p, SPEED='10', **params))
        check(j.session is not None, "%s jog is running" % axes)
        rig.printer.send_event("bed_removal_detector:removed")
        w.run(0.1)
        check(j.session is None and j.reason == hg.HOME_FIRST,
              "%s jog ends when the height is lost: %r" % (axes, j.reason))
        n = len(p.toolhead.batches)
        w.run(0.5)
        check(len(p.toolhead.batches) == n, "and queues nothing more")


def test_guard_z_jog_needs_xyz_homed():
    for homed in ('z', 'xz', 'yz'):
        rig, p, j = guarded((100., 100., 20.))
        rig.printer.toolhead.homed_axes = homed
        got = refused(p, Z='30')
        check(got is not None and 'Home XYZ' in got and j.session is None,
              "Z jog refused with only %r homed: %r" % (homed, got))
    rig, p, j = guarded((100., 100., 20.))
    jog(p, Z='30', SPEED='10')
    w = Watch(p, (2,))
    w.run(0.2)
    rig.printer.toolhead.homed_axes = 'z'
    w.run(0.2)
    check(j.session is None and 'homing was lost' in j.reason,
          "a running Z jog ends when X or Y is no longer homed")


def test_guard_floor_follows_the_mesh_mid_session():
    # Another client clears the bed mesh during the drag: the floor goes
    # from 0.1 to 2.6. The feed re-clamps every tick, so FINAL=1 (nothing
    # else ever updates the target) is held too.
    for final in (False, True):
        rig, p, j = guarded((100., 100., 20.), mesh=True)
        w = Watch(p, (2,))
        extra = {'FINAL': '1'} if final else {}
        if final:
            jog(p, Z='-50', SPEED='10', **extra)
            w.run(0.5)
        else:
            w.run(0.5, refresh=lambda: jog(p, Z='-50', SPEED='10'))
        check(w.pos()[2] > 2.6 + 1. and j.session['target'][2] < 0.2,
              "FINAL=%s: heading for 0.1 over the mesh (Z %.2f)"
              % (final, w.pos()[2]))
        rig.bed_mesh.mesh = None
        n = len(w.trace)
        w.run(4., refresh=None if final else
              (lambda: jog(p, Z='-50', SPEED='10')))
        low = min(t[2] for t in w.trace[n:])
        check(low >= 2.6 - 1e-6 and abs(w.pos()[2] - 2.6) < 1e-6,
              "FINAL=%s: mesh cleared mid-drag, Z stops at 2.6 (low %.3f)"
              % (final, low))
        check('Z2.6' in j.session['clamp'] if j.session else True,
              "FINAL=%s: and says why" % (final,))
    # Already below the new floor: holds where it is, is not driven up
    for final in (False, True):
        rig, p, j = guarded((100., 100., 2.0), mesh=True)
        w = Watch(p, (2,))
        jog(p, Z='-50', SPEED='1', **({'FINAL': '1'} if final else {}))
        w.run(0.5, refresh=None if final else
              (lambda: jog(p, Z='-50', SPEED='1')))
        held = j.session['own'][2]
        check(0.1 < held < 2.0, "FINAL=%s: descending below 2.6 on a mesh "
              "(queued to Z %.2f)" % (final, held))
        rig.bed_mesh.mesh = None
        n = len(w.trace)
        w.run(4., refresh=None if final else
              (lambda: jog(p, Z='-50', SPEED='1')))
        check(min(t[2] for t in w.trace[n:]) >= held - 1e-6
              and abs(w.pos()[2] - held) < 1e-6
              and abs(p.toolhead.commanded[2] - held) < 1e-6,
              "FINAL=%s: below the new floor it holds at Z%.2f, not driven "
              "up (Z %.3f)" % (final, held, w.pos()[2]))
        if final:
            check(j.session is None and 'arrived' in j.reason,
                  "FINAL=%s: and the jog ends: %r" % (final, j.reason))


class GuardedGcodeMove(GcodeMove):
    """move_with_transform goes through the real ContactGuard.move, as
    gcode_move does (the guard is a move transform); reset_last_position
    reads the position back through it. The guard's own fake toolhead
    stands in for the rest of the chain."""
    def __init__(self, toolhead, guard):
        GcodeMove.__init__(self, toolhead)
        self.guard = guard

    def move_with_transform(self, newpos, speed):
        self.guard.move(newpos, speed)
        self.toolhead.move(newpos, speed)

    def reset_last_position(self):
        self.last_position = self.guard.get_position()


def test_guard_refusal_of_an_xy_batch_ends_the_jog():
    # Plate at Z1.0, below the guard's 2.6 minimum (no mesh): an XY jog is
    # refused by the real guard. A non-zero G92-style base makes the gcode
    # X/Y/Z differ from last_position, which is what the guard sees.
    rig, p, j = guarded((100., 100., 1.0))
    rig.printer.toolhead.position[:2] = [100., 100.]
    rig.guard.get_position()
    p.command_error = hg.base.CommandError
    gm = GuardedGcodeMove(p.toolhead, rig.guard)
    gm.base_position = [5., -3., 0.5, 0.]
    p.gcode_move = p.objects['gcode_move'] = j.gcode_move = gm
    gm.last_position = list(p.toolhead.get_position())
    check(rig.guard.last_position[:3] == [100., 100., 1.0],
          "the guard and the jog start in step")
    jog(p, X='125', Y='97', SPEED='10')
    check(j.session['target'][:2] == [130., 94.],
          "the target is in last_position coordinates (base added)")
    w = Watch(p, (0, 1))
    w.run(0.3)
    check(j.session is None and 'refused' in j.reason
          and 'Z 1.00' in j.reason and '2.60' in j.reason,
          "the guard refuses the batch and the jog ends: %r" % (j.reason,))
    check(rig.printer.toolhead.moves == [] and not p.toolhead.batches
          and gm.last_position[:3] == [100., 100., 1.0]
          and gm.last_position[:3] == rig.guard.last_position[:3],
          "nothing moved and gcode_move.last_position is back in sync: %r"
          % (gm.last_position,))
    n = len(p.toolhead.batches)
    w.run(0.3)
    check(len(p.toolhead.batches) == n, "and nothing more is queued")


def test_allowlisted():
    cp = core_cfg()
    entries = cp.get('muon_guard', 'client_allow').split('\n')
    rules = muon_guard.parse_allowlist(entries, CommandError)
    ok = all(any(muon_guard.rule_matches(r, params)
                 for r in rules.get('MUON_JOG', []))
             for params in ({'Z': '12.5', 'SPEED': '10'},
                            {'X': '100', 'Y': '-5', 'FINAL': '1'},
                            {'STOP': '1'}))
    check(ok, "MUON_JOG is on the client allowlist with its parameters")
    check(not any(muon_guard.rule_matches(r, {'ACCEL': '1'})
                  for r in rules.get('MUON_JOG', [])),
          "and nothing else gets through it")


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
    for f in FAILS:
        print('FAILED: ' + f)
    print('%d failed' % len(FAILS))
    sys.exit(1 if FAILS else 0)
