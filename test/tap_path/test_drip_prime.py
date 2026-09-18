"""short_prime_move must (a) leave the move for the normal priming path,
(b) apply the DRIP lead when the lookahead is flushed, (c) consume the flag,
(d) not touch print_time when the queue is already ahead, (e) reset the
flag when the move was a no-op."""
import contextlib
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402

sys.argv = sys.argv[:1]
MIN_KIN_TIME = 0.100
KFD = 0.001
STUBS = ('mcu', 'chelper', 'kinematics', 'kinematics.extruder', 'msgproto',
         'clocksync', 'serialhdl', 'pins')


def load(path):
    for n in STUBS:
        sys.modules.setdefault(n, types.ModuleType(n))
    sys.modules['kinematics'].extruder = sys.modules['kinematics.extruder']
    mod = types.ModuleType('th')
    src = open(path, 'r', newline='').read()
    exec(compile(src, path, 'exec'), mod.__dict__)
    return mod


class Sim:
    # Stands in for reactor, mcu, motion_queuing, lookahead and printer
    def __init__(self, mod):
        self.now = 1000.0
        self.moves = []
        self.last_step_gen_time = 0.
        self.kin_flush_delay = KFD
        self.added = []
        th = object.__new__(mod.ToolHead)
        th.reactor = th.mcu = th.motion_queuing = self
        th.lookahead = th.printer = self
        th.print_time = 0.
        th.special_queuing_state = 'NeedPrime'
        th.need_check_pause = 0.
        th.check_stall_time = 0.
        th.trapq = None
        th.extra_axes = []
        th.commanded_pos = [0., 0., 0., 0.]
        th.trapq_append = lambda *a: self.added.append(a)
        # manual_move builds Move objects; stub at the move() level instead
        th.move = self._move
        self.th = th

    def monotonic(self):
        return self.now

    def assert_no_pause(self):
        return contextlib.nullcontext()

    def estimated_print_time(self, t):
        return t

    def calc_step_gen_restart(self, est):
        return (max(est + MIN_KIN_TIME, self.last_step_gen_time)
                + self.kin_flush_delay)

    def get_kin_flush_delay(self):
        return self.kin_flush_delay

    def flush(self, lazy=False):
        m, self.moves = self.moves, []
        return m

    def set_flush_time(self, t):
        pass

    def is_empty(self):
        return not self.moves

    def send_event(self, *a):
        return []

    def note_mcu_movequeue_activity(self, t):
        pass

    def _move(self, coord, speed):
        # a 0.05 s kinematic move sitting in the lookahead until flushed
        m = types.SimpleNamespace(
            is_kinematic_move=True, accel_t=0.01, cruise_t=0.03,
            decel_t=0.01, start_pos=[0, 0, 0], axes_r=[0, 0, 1],
            start_v=0., cruise_v=10., accel=1000., axes_d=[0, 0, 0.4, 0],
            timing_callbacks=[])
        self.moves.append(m)


mod = load(_paths.patched('klippy/toolhead.py'))
bad = 0


def check(name, ok):
    global bad
    print('  %-64s %s' % (name, 'PASS' if ok else 'FAIL'))
    bad += (not ok)


s = Sim(mod)
th = s.th
th.manual_move = lambda coord, speed: th.move(coord, speed)
th.short_prime_move([0, 0, 0.4], 10.)
check('move left in lookahead, not flushed at call',
      len(s.moves) == 1 and not s.added)
check('flag set', th.drip_prime_next is True)
check('priming state untouched (still NeedPrime)',
      th.special_queuing_state == 'NeedPrime')
s.now += 0.02
th._flush_lookahead()
check('flushed with DRIP lead: start == est+0.101',
      abs(s.added[0][1] - (s.now + 0.100 + KFD)) < 1e-9)
check('flag consumed', th.drip_prime_next is False)

# already-ahead queue: no change to print_time
s2 = Sim(mod)
th2 = s2.th
th2.manual_move = lambda c, sp: th2.move(c, sp)
th2.special_queuing_state = ''
th2.print_time = s2.now + 0.5
th2.short_prime_move([0, 0, 0.4], 10.)
th2._flush_lookahead()
check('queue already ahead: move appended at existing print_time',
      abs(s2.added[0][1] - (s2.now + 0.5)) < 1e-9)

# no-op move resets the flag
s3 = Sim(mod)
th3 = s3.th
th3.manual_move = lambda c, sp: None
th3.short_prime_move([0, 0, 0], 10.)
check('no-op move: flag reset', th3.drip_prime_next is False)

# exception path
s4 = Sim(mod)
th4 = s4.th


def boom(c, sp):
    raise RuntimeError('x')


th4.manual_move = boom
try:
    th4.short_prime_move([0, 0, 0.4], 10.)
except RuntimeError:
    pass
check('exception: flag reset', th4.drip_prime_next is False)
print('all PASS' if not bad else '%d FAIL' % bad)
sys.exit(bad)
