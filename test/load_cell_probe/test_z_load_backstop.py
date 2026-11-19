"""Z move load backstop (KAN-3370, guard D), run without a printer.

On the M1 "Boxwood" the nozzle was loaded at tare, the probe's trigger
never fired and the Z homing search (a PROBE from a soft-homed Z=160 at
15 mm/s) pushed the bed up for 164 mm.  Guards A-C stop that inside the
probe, the search cap and the abort hook; guard D is independent of them
all: ZLoadBackstop watches the load cell samples on the host during a Z
probing move and calls PrinterHoming.abort_active_move() when the load
goes the wrong way by a trigger delta, or sits past a ceiling on either
side, for 20 ms of consecutive samples.

This drives the real ZLoadBackstop, the real LoadCellProbingMove
(.probing_move, ._pause_and_tare) and, for the last test, the real
PrinterHoming / HomingMove with the fakes in test/homing_abort/fakes.py.
Numbers are written out literally, not imported from the code under test.

    python3 test/load_cell_probe/test_z_load_backstop.py

Needs numpy.  Negative controls (performed by hand for KAN-3370, see the
report): the watcher disabled (arm() returns at once) fails test 1; the
consecutive-sample filter removed (_need forced to 1) fails test 4.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'homing_abort'))
sys.path.insert(0, HERE)

import fakes  # noqa: E402  (loads the real extras.homing first)
import test_tare_guard as tg  # noqa: E402
from fakes import Rig  # noqa: E402

# Anchors (reference_max_load_counts) and tares
BOX_REF = -3010747
SLOE_REF = -1585739
INCIDENT_TARE = -3244819
SPEED = 15.             # mm/s, the Z homing search speed
SPS = 2000              # nominal; the M1 really runs 1977 (test 9)

# Trigger deltas (counts) at trigger_percent 10 of |anchor - tare|
DELTA_BOX_578 = 243275      # span 2432747
DELTA_BOX_800 = 221075      # span 2210747
DELTA_BOX_1750 = 126075     # span 1260747
DELTA_SLOE_200 = 178574     # span 1785739
DELTA_INCIDENT = 23407      # span 234072: the tare is PAST the anchor

CEILING_REASON = ("the load on the nozzle exceeded the safety ceiling "
                  "during the Z move")
WRONG_WAY_REASON = "the load cell moved the wrong way during the Z move"
FAILED_REASON = "the load watch failed during the Z move"


# --- fakes ---------------------------------------------------------------

class Sensor(tg.FakeSensor):
    def __init__(self, sps):
        self.sps = sps

    def get_samples_per_second(self):
        return self.sps


class Cell(tg.FakeLoadCell):
    """A load cell whose clients can be fed sample batches."""

    def __init__(self, sps=SPS, counts_per_gram=None, tare_counts=0):
        tg.FakeLoadCell.__init__(self, counts_per_gram, tare_counts)
        self.sensor = Sensor(sps)
        self.clients = []

    def add_client(self, cb):
        self.clients.append(cb)

    def feed(self, rows):
        msg = {'data': rows, 'errors': 0, 'overflows': 0}
        for cb in list(self.clients):
            if not cb(msg):
                self.clients.remove(cb)


class Homing(object):
    """Records abort requests, like PrinterHoming.abort_active_move."""

    def __init__(self, accept=True, raises=None):
        self.calls = []
        self.accept = accept
        self.raises = raises

    def abort_active_move(self, reason, z_only=True):
        self.calls.append(reason)
        if self.raises is not None:
            raise self.raises
        return self.accept


class Reactor(object):
    def monotonic(self):
        return 1000.


class Printer(object):
    command_error = tg.CommandError

    def __init__(self, homing):
        self.objects = {'homing': homing}

    def get_reactor(self):
        return Reactor()

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def make_watch(mod, homing=None, sps=SPS, **opts):
    homing = homing if homing is not None else Homing()
    printer = Printer(homing)
    cell = Cell(sps)
    cfg = tg.FakeConfig(printer, opts)
    return mod.ZLoadBackstop(cfg, printer, cell), cell, homing


def rows(t0, counts, sps=SPS):
    return [[t0 + i / float(sps), None, c, None]
            for i, c in enumerate(counts)]


def feed_until_abort(cell, homing, t0, counts, sps=SPS, batch=40):
    """Feed counts in 20 ms batches; returns the index of the sample the
    abort was requested on (counts are consumed in order), or None."""
    r = rows(t0, counts, sps)
    for i in range(0, len(r), batch):
        before = len(homing.calls)
        cell.feed(r[i:i + batch])
        if len(homing.calls) > before:
            return i, r[i:i + batch]
    return None, None


def first_abort_sample(mod, tare, direction, delta, counts, t0=10.,
                       sps=SPS, **opts):
    """Sample index at which the watcher asks for the abort."""
    w, cell, homing = make_watch(mod, sps=sps, **opts)
    seen = []
    orig = w._judge

    def judge(c, t=None):
        seen.append(c)
        return orig(c, t)
    w._judge = judge
    w.arm(t0, tare, delta, direction)
    feed_until_abort(cell, homing, t0, counts, sps)
    return w, homing, len(seen) - 1 if homing.calls else None


def run_incident(mod, stiffness_per_mm, contact_s=1.0, seconds=3.0):
    tare = INCIDENT_TARE
    per_sample = stiffness_per_mm * SPEED / float(SPS)
    n = int(seconds * SPS)
    n_rest = int(contact_s * SPS)
    counts = [tare] * n_rest
    counts += [tare - int(per_sample * i) for i in range(n - n_rest)]
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_INCIDENT,
                                        counts)
    return w, homing, idx, n_rest


# --- tests ---------------------------------------------------------------

def test_1_incident_watcher_alone(mod):
    # The incident: tare -3244819 (past the anchor -3010747), the probe's
    # trigger never fires (nothing here ever triggers), samples ramp
    # negative as the bed is pushed.  Delta 23407 is what trigger_percent
    # 10 arms from that tare; the ceiling is int(2.5 * 23407) = 58517.
    for name, stiff, want_idx, want_travel in (
            ('stiff 100000 counts/mm', 100000, 118, 0.885),
            ('slow ramp 10000 counts/mm', 10000, 820, 6.15)):
        w, homing, idx, n_rest = run_incident(mod, stiff)
        assert homing.calls == [CEILING_REASON], (
            '%s: abort not requested: %r' % (name, homing.calls))
        assert w.get_limits() == (58517, 23407), w.get_limits()
        assert idx is not None, name
        after_contact = idx - n_rest
        assert after_contact == want_idx, (name, after_contact)
        travel = after_contact / float(SPS) * SPEED
        assert abs(travel - want_travel) < 0.01, (name, travel)
        assert travel < 7., (name, travel)
        assert w.last_fired == CEILING_REASON
        st = w.get_status()
        assert st['z_backstop_ceiling_counts'] == 58517, st
        assert st['z_backstop_fired'] == CEILING_REASON, st
        print('ok  1 incident replay, %s: abort on the ceiling at sample '
              '%d after contact = %.3f s = %.2f mm at 15 mm/s'
              % (name, after_contact, after_contact / float(SPS), travel))


def test_2_wrong_way_and_idle_wander(mod):
    # Normal Boxwood tare -800000, delta 221075.  Counts drift UP (the
    # wrong way: pressing lowers them) by 100 per sample.
    tare = -800000
    counts = [tare + 100 * i for i in range(4000)]
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                        counts)
    assert w.get_limits() == (552687, 221075), w.get_limits()
    # first sample over 221075 is i=2211 (221100); 40 in a row -> 2250
    # (KAN-4364: nothing is followed by default; legs, test_z_legs.py)
    assert homing.calls == [WRONG_WAY_REASON], homing.calls
    assert idx == 2250, idx
    assert w.last_fired == WRONG_WAY_REASON
    # the same drift on the press side is not the wrong way: the ceiling
    # (552687) is what ends it, at i=5527 + 39
    counts = [tare - 100 * i for i in range(8000)]
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                        counts)
    assert homing.calls == [CEILING_REASON] and idx == 5566, (
        homing.calls, idx)
    # Sloe: tare +200000, delta 178574; idle wander -4000..+7000 for 3 s
    tare = 200000
    seed = 12345
    counts = []
    for _ in range(6000):
        seed = (seed * 1103515245 + 12345) % (1 << 31)
        counts.append(tare - 4000 + seed % 11001)
    assert min(counts) >= tare - 4000 and max(counts) <= tare + 7000
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_SLOE_200,
                                        counts)
    assert homing.calls == [] and w.last_fired is None, homing.calls
    assert w.get_limits() == (446435, 178574), w.get_limits()
    print('ok  2 wrong way aborts at sample 2250 (1.125 s); idle wander '
          '-4000..+7000 on Sloe never does')


def ring(delta, n):
    """A normal tap: rest, ramp in the press direction to delta (the MCU
    trigger fires there), keep going 40 samples (20 ms) at 1.5x the slope
    as the move stops, then ring down."""
    out = [0] * 2000
    steps = 400
    out += [int(delta * (i + 1) / float(steps)) for i in range(steps)]
    over = int(delta * 1.5 / steps)
    out += [delta + over * (i + 1) for i in range(40)]
    peak = out[-1]
    out += [int(peak * (1. - i / 100.)) * (1 if i % 2 else -1)
            for i in range(100)]
    return out


def test_3_normal_probes_never_abort(mod):
    cases = (('Boxwood -578000', BOX_REF, -578000, DELTA_BOX_578),
             ('Boxwood -1750000', BOX_REF, -1750000, DELTA_BOX_1750),
             ('Sloe +200000', SLOE_REF, 200000, DELTA_SLOE_200))
    for name, ref, tare, want_delta in cases:
        # the delta really is what the probe arms (trigger_percent 10)
        h = tg.make_helper(mod, ref, trigger_percent=10)
        lo, hi = h.get_safety_range(tare)
        ts = h.get_trigger_setup(tare, lo, hi)
        assert ts['trigger_delta_counts'] == want_delta, (name, ts)
        assert ts['press_direction'] == -1., (name, ts)
        counts = [tare - d for d in ring(want_delta, 0)]
        w, homing, idx = first_abort_sample(mod, tare, -1., want_delta,
                                            counts)
        assert homing.calls == [] and w.last_fired is None, (
            name, homing.calls)
        peak = max(abs(c - tare) for c in counts)
        assert peak < w.get_limits()[0], (name, peak)
        print('ok  3 normal probe %s: delta %d, peak %d (%.2f x delta) vs '
              'ceiling %d, no abort' % (name, want_delta, peak,
                                        peak / float(want_delta),
                                        w.get_limits()[0]))


def test_4_single_spike_and_consecutive(mod):
    tare = -800000
    ceiling = 552687
    rest = [tare] * 200
    # one sample far over the ceiling, press side and the other side
    for spike in (tare - 900000, tare + 900000):
        counts = rest + [spike] + rest
        w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                            counts)
        assert w._need == 40 and homing.calls == [], (spike, homing.calls)
    over = tare - ceiling - 1
    # 39 in a row, then one under: still no abort
    counts = rest + [over] * 39 + [tare] + rest
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                        counts)
    assert homing.calls == [], homing.calls
    # an alternating ring over/under never gets to 40
    counts = rest + [over, tare] * 200 + rest
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                        counts)
    assert homing.calls == [], homing.calls
    # 40 in a row: the abort comes on the 40th, and only once
    counts = rest + [over] * 100
    w, homing, idx = first_abort_sample(mod, tare, -1., DELTA_BOX_800,
                                        counts)
    assert homing.calls == [CEILING_REASON], homing.calls
    assert idx == 200 + 39, idx
    # the filter length follows the sample rate: 20 ms of samples, min 3
    for sps, need in ((2000, 40), (1000, 20), (660, 14), (100, 3)):
        w, cell, homing = make_watch(mod, sps=sps)
        w.arm(10., tare, DELTA_BOX_800, -1.)
        assert w._need == need, (sps, w._need)
    print('ok  4 one spike is ignored, 39 in a row is ignored, 40 in a row '
          'aborts at the 40th (20 ms at 2000 sps)')


def test_5_start_time_and_disarm(mod):
    tare = -800000
    over = tare - 600000
    # samples stamped before the descent start are ignored, the one
    # stamped exactly at the start counts: 100 old over-ceiling samples,
    # then 40 from t=10.0 abort on the 40th (not the 1st)
    w, cell, homing = make_watch(mod)
    w.arm(10., tare, DELTA_BOX_800, -1.)
    cell.feed(rows(9.9, [over] * 100))
    assert homing.calls == []
    r = rows(10., [over] * 40)
    cell.feed(r[:39])
    assert homing.calls == []
    cell.feed(r[39:])
    assert homing.calls == [CEILING_REASON]
    # once fired the client is dropped and nothing more is requested
    cell.feed(rows(10.1, [over] * 200))
    assert homing.calls == [CEILING_REASON] and cell.clients == []
    # a LoadCellProbingMove that ends three ways leaves nothing armed
    for ending in ('success', 'no trigger', 'aborted'):
        m, cell, homing, watch = make_move(mod, ending)
        err = None
        try:
            m.probing_move(None)
        except tg.CommandError as e:
            err = str(e)
        if ending == 'success':
            assert err is None, err
        elif ending == 'no trigger':
            assert err == "No trigger on probe after full movement", err
        else:
            assert err == "Homing aborted: " + CEILING_REASON, err
        assert not watch.is_armed(), ending
        n = len(homing.calls)
        cell.feed(rows(11., [over] * 300))
        cell.feed(rows(11.2, [tare + 700000] * 300))
        assert len(homing.calls) == n, (ending, homing.calls)
        assert cell.clients == [], ending
        assert homing.calls == ([CEILING_REASON] if ending == 'aborted'
                                else []), (ending, homing.calls)
    print('ok  5 samples before the start are ignored (at the start '
          'counts); disarmed after success, "No trigger" and an abort')


def make_move(mod, ending, mcutare=False, tare=-800000, ref=BOX_REF,
              cell=None, **opts):
    """A LoadCellProbingMove with the real probing_move/_pause_and_tare,
    driven by a fake homing whose probing_move ends as `ending`."""
    homing = Homing()
    printer = Printer(homing)
    cell = cell or Cell(SPS, counts_per_gram=1000., tare_counts=tare)
    helper = tg.make_helper(mod, ref, trigger_percent=10)
    helper._load_cell = cell
    m = tg.make_move(mod, helper, tare)
    m._load_cell = cell
    m._printer = printer
    m._z_min_position = -4.
    m._param_helper = ParamHelper()
    m._nozzle_is_clear = lambda: True
    m._start_collector = lambda: Collector(m)
    m._z_watch = mod.ZLoadBackstop(tg.FakeConfig(printer, opts), printer,
                                   cell)
    printer.objects['toolhead'] = Toolhead()
    over = tare - 600000

    class FakePhoming(Homing):
        def probing_move(self, endstop, pos, speed):
            if ending == 'aborted':
                # the watcher fires while the move is in progress, and the
                # move then raises as HomingMove does
                cell.feed(rows(10.1, [over] * 60))
                raise tg.CommandError("Homing aborted: " + homing.calls[-1])
            if ending == 'no trigger':
                raise tg.CommandError(
                    "No trigger on probe after full movement")
            return [0., 0., 1.2, 0.]
    fake = FakePhoming()
    fake.calls = homing.calls
    printer.objects['homing'] = fake
    m._mcu_tare = lambda gcmd: m._pause_and_tare(gcmd)
    return m, cell, fake, m._z_watch


class ParamHelper(object):
    def get_probe_params(self, gcmd):
        return {'probe_speed': 15.}


class Toolhead(object):
    def get_position(self):
        return [100., 100., 160., 0.]

    def dwell(self, t):
        pass

    def get_last_move_time(self):
        return 10.


class Collector(object):
    # the tare collector (quiet samples at the cell's tare) and the
    # descent collector, as _start_collector hands them out
    def __init__(self, m):
        m._collect_start_time = 10.
        self.counts = m._load_cell.tare_counts

    def start_collecting(self, min_time=None):
        pass

    def collect_min(self, n=1):
        return [[10. + i / 2000., 0., self.counts]
                for i in range(max(n, 1))], None

    def stop_collecting(self):
        pass


class Gcmd(object):
    def __init__(self, mcutare):
        self.mcutare = mcutare

    def get_int(self, name, default=None, *args, **kw):
        return self.mcutare if name == 'MCUTARE' else default

    def get_float(self, name, default=None, *args, **kw):
        return default


def test_6_mcutare_and_directionless(mod):
    # MCUTARE: the MCU zeroes itself, the host tare is approximate, so
    # rule (a) is off and only the ceiling stays.  The probe arms from
    # the approximate tare through the real _mcu_tare.
    tare = -800000
    cell = Cell(SPS, counts_per_gram=1000., tare_counts=tare)
    seen = {}

    class Ph(Homing):
        def probing_move(self, endstop, pos, speed):
            seen['limits'] = m._z_watch.get_limits()
            seen['armed'] = m._z_watch.is_armed()
            return [0., 0., 1., 0.]
    printer = Printer(Ph())
    printer.objects['toolhead'] = Toolhead()
    helper = tg.make_helper(mod, BOX_REF, trigger_percent=10)
    helper._load_cell = cell
    m = tg.make_move(mod, helper, tare)
    m._load_cell = cell
    m._printer = printer
    m._z_min_position = -4.
    m._param_helper = ParamHelper()
    m._nozzle_is_clear = lambda: True
    m._start_collector = lambda: Collector(m)
    m._z_watch = mod.ZLoadBackstop(tg.FakeConfig(printer, {}), printer, cell)

    class Fresh(object):
        def start_collecting(self, min_time=None):
            pass

        def collect_min(self, n=1):
            return [[10., 0., tare]], None
    cell.get_collector = lambda: Fresh()
    m._approx_tare = tare
    m.probing_move(Gcmd(1))
    assert seen['armed'] and seen['limits'] == (552687, 0), seen
    assert m._trigger_delta == DELTA_BOX_800
    w = m._z_watch
    # without MCUTARE the same descent arms rule (a) as well
    m.probing_move(Gcmd(0))
    assert seen['limits'] == (552687, 221075), seen
    # the approximate tare may be wrong by MAX_BASELINE_DRIFT_COUNTS
    # (100000) in the worst direction: a quiet descent is still quiet,
    # and a +300000 wrong-way excursion is left to the MCU; the ceiling
    # still stops +700000
    w.arm(10., tare + 100000, DELTA_BOX_800, -1., mcutare=True)
    assert w.get_limits() == (552687, 0)
    h = Homing()
    w._printer.objects['homing'] = h
    cell.feed(rows(10., [tare] * 400))
    cell.feed(rows(10.2, [tare + 300000] * 400))
    assert h.calls == [], h.calls
    cell.feed(rows(10.4, [tare + 700000] * 100))
    assert h.calls == [CEILING_REASON], h.calls
    # without MCUTARE the same +300000 excursion IS the wrong way
    w2, cell2, h2 = make_watch(mod)
    w2.arm(10., tare, DELTA_BOX_800, -1.)
    cell2.feed(rows(10., [tare + 300000] * 100))
    assert h2.calls == [WRONG_WAY_REASON], h2.calls
    # a directionless trigger (abs_ge: grams mode, reference_tare): no
    # wrong side, the ceiling watches both sides
    for sign in (1, -1):
        w3, cell3, h3 = make_watch(mod)
        w3.arm(10., 5000, 30000, None)
        assert w3.get_limits() == (75000, 0), w3.get_limits()
        cell3.feed(rows(10., [5000 + sign * 60000] * 200))
        assert h3.calls == [], (sign, h3.calls)
        cell3.feed(rows(10.2, [5000 + sign * 90000] * 100))
        assert h3.calls == [CEILING_REASON], (sign, h3.calls)
    # the trigger setup reports no press side for grams mode
    h = tg.make_helper(mod, BOX_REF, counts_per_gram=1000., trigger_force=75)
    lo, hi = h.get_safety_range(-800000)
    ts = h.get_trigger_setup(-800000, lo, hi)
    assert ts['trigger_type'] == 'abs_ge' and ts['press_direction'] is None
    assert ts['trigger_delta_counts'] == 75000, ts
    # options: an explicit counts ceiling wins; 0 turns each rule off
    w4, cell4, h4 = make_watch(mod, z_move_abort_ceiling_counts=300000)
    w4.arm(10., -800000, DELTA_BOX_800, -1.)
    assert w4.get_limits() == (300000, 221075)
    w5, cell5, h5 = make_watch(mod, z_move_abort_ceiling_factor=0,
                               z_move_wrong_way_factor=0)
    w5.arm(10., -800000, DELTA_BOX_800, -1.)
    assert w5.get_limits() == (0, 0) and cell5.clients == []
    for bad in ({'z_move_abort_ceiling_factor': 1.0},
                {'z_move_wrong_way_factor': 0.2}):
        try:
            make_watch(mod, **bad)
        except tg.ConfigError:
            pass
        else:
            raise AssertionError('accepted %r' % (bad,))
    print('ok  6 MCUTARE: wrong-way rule off, ceiling kept; abs_ge: '
          'ceiling both sides; options and 0 = off')


def test_7_watcher_failure_fails_safe(mod):
    tare = -800000
    for bad in ([[10., None, None, None]],            # no counts
                [[10., None, 'x', None]],             # not a number
                ):
        w, cell, homing = make_watch(mod)
        w.arm(10., tare, DELTA_BOX_800, -1.)
        cell.feed(bad)          # must not raise
        assert homing.calls == [FAILED_REASON], homing.calls
        assert cell.clients == [] and not w.is_armed()
    # a batch with no data at all
    w, cell, homing = make_watch(mod)
    w.arm(10., tare, DELTA_BOX_800, -1.)
    assert w._on_batch({}) is False
    assert homing.calls == [FAILED_REASON], homing.calls
    # the abort request itself failing is logged, never raised
    w, cell, homing = make_watch(mod, homing=Homing(raises=ValueError('x')))
    w.arm(10., tare, DELTA_BOX_800, -1.)
    cell.feed(rows(10., [tare - 600000] * 100))
    assert homing.calls == [CEILING_REASON] and w.last_fired is None
    # nothing to abort (the move already ended): not counted as fired
    w, cell, homing = make_watch(mod, homing=Homing(accept=False))
    w.arm(10., tare, DELTA_BOX_800, -1.)
    cell.feed(rows(10., [tare - 600000] * 100))
    assert homing.calls == [CEILING_REASON] and w.last_fired is None
    print('ok  7 a failing watcher asks for the abort with its own reason '
          'and raises nothing')


class Stream(object):
    """Feeds a 1500 sps load cell from the fake toolhead's 1 mm ticks:
    100 samples per mm at 15 mm/s."""

    def __init__(self, cell, tare, contact_mm, per_sample, extra=0):
        self.cell = cell
        self.tare = tare
        self.contact = contact_mm
        self.per_sample = per_sample
        self.extra = extra
        self.last = None

    def __call__(self, mm):
        out = []
        for j in range(100):
            idx = (mm - self.contact - 1) * 100 + j
            out.append(self.tare - (self.per_sample * idx
                                    if idx >= 0 else 0))
        # samples after the last one of the tick are the move stopping
        base = out[-1] if out else self.tare
        if mm == self.contact + 1:
            out += [base - self.per_sample * (k + 1) * 5
                    for k in range(self.extra)]
        t0 = 10. + (mm - 1) / SPEED
        self.cell.feed(rows(t0, out, 1500))


def test_8_integration_with_real_homing(mod):
    # (a) the incident, through the real PrinterHoming/HomingMove: the
    # endstop never triggers, contact at 20 mm of travel, 100000 counts/mm
    rig = Rig(start=160.)
    cell = Cell(1500, counts_per_gram=1000., tare_counts=INCIDENT_TARE)
    m = integ_move(mod, rig, cell, INCIDENT_TARE, DELTA_INCIDENT)
    rig.toolhead.on_tick = Stream(cell, INCIDENT_TARE, 20, 1000)
    err = None
    try:
        m.probing_move(None)
    except fakes.CommandError as e:
        err = str(e)
    want = "Homing aborted: " + CEILING_REASON
    assert err == want, err
    assert rig.toolhead.travelled_mm == 21, rig.toolhead.travelled_mm
    assert rig.phoming.active_moves == [] and not m._z_watch.is_armed()
    assert m._z_watch.last_fired == CEILING_REASON
    assert rig.toolhead.pos[2] == 139., rig.toolhead.pos
    print('ok  8a real HomingMove: "%s" after 21 mm (1 mm past contact at '
          '20)' % (err,))
    # (b) a normal trigger at 21 mm, then physically impossible samples
    # over the ceiling in the same tick: the move is a success, no abort
    rig = Rig(start=160., trigger_after_mm=21)
    cell = Cell(1500, counts_per_gram=1000., tare_counts=-800000)
    m = integ_move(mod, rig, cell, -800000, DELTA_BOX_800)
    rig.toolhead.on_tick = Stream(cell, -800000, 20, 5000, extra=60)
    res, err = fakes.run(lambda: m.probing_move(None))
    assert err is None and res is not None, err
    assert rig.endstops[0].triggered and rig.toolhead.travelled_mm == 21
    assert m._z_watch.last_fired is None
    assert not m._z_watch.is_armed()
    print('ok  8b a probe that triggered is never reported as an abort')
    # (c) the abort is refused by the move once it has triggered, and a
    # second request after an abort is refused too
    rig = Rig(start=160., trigger_after_mm=5)
    hmove = rig.hmove()
    got = []

    def on_tick(mm):
        if mm == 5:
            got.append(rig.phoming.abort_active_move("late"))
    rig.toolhead.on_tick = on_tick
    res, err = fakes.run(lambda: hmove.homing_move([0., 0., -4., 0.], 15.))
    assert got == [False] and err is None, (got, err)
    print('ok  8c abort_active_move after the trigger returns False')


def test_9_muon_ads1220_cadence(mod):
    # [muon_ads1220 toolhead] runs the ADS1220 at 2000 SPS turbo: a
    # conversion is 2072 tCLK of 4.096 MHz = 0.5059 ms, so sps = 1977.
    # The host polls every 20 ms (about 40 samples per batch).  Slots of a
    # temperature read are fillers the host drops, so the stream has holes
    # but the timestamps stay on the fixed grid.  Rows are LoadCell's
    # [time, grams, counts, tare], as in the other tests.
    sps = 1977
    tare = -800000
    over = tare - 552688       # past the ceiling, 2.5 x DELTA_BOX_800
    period = 1. / sps
    t0 = 10.
    first = t0 + 100 * period  # the first over-ceiling slot

    def stream(keep, flips=()):
        counts = [tare] * 100 + [over] * 400
        for i in flips:
            counts[i] = tare
        return [[t0 + i * period, None, c, None]
                for i, c in enumerate(counts) if keep(i)]

    def run(rs):
        """(abort reasons, time of the sample that confirmed it, run
        length then, first sample time of that run)"""
        w, cell, homing = make_watch(mod, sps=sps)
        w.arm(t0, tare, DELTA_BOX_800, -1.)
        assert w._need == 40, w._need   # ceil(0.020 x 1977)
        fired = []
        orig = w._judge

        def judge(c, t=None):
            r = orig(c, t)
            if r is not None:
                fired.append((t, w._over_run, w._over_t0))
            return r
        w._judge = judge
        for i in range(0, len(rs), 40):
            cell.feed(rs[i:i + 40])
        return homing.calls, (fired[0] if fired else None)
    # (a) a clean stream: the 40th over-ceiling sample, 19.7 ms after the
    # first, confirms by count (the 20 ms span is one period further on)
    calls, f = run(stream(lambda i: True))
    assert calls == [CEILING_REASON], calls
    assert f[1] == 40 and abs(f[0] - (first + 39 * period)) < 1e-9, f
    # (b) every other slot missing (988 effective sps): the 20 ms span
    # confirms on the 21st kept sample, long before 40 samples (40.5 ms)
    calls, f = run(stream(lambda i: i < 100 or i % 2 == 0))
    assert calls == [CEILING_REASON], calls
    assert f[1] < 40 and 0.020 <= f[0] - first < 0.0225, f
    # (c) a 5 ms hole (a blind temperature read: 10 slots) inside the run
    # does not restart it
    calls, f = run(stream(lambda i: not 130 <= i < 140))
    assert calls == [CEILING_REASON], calls
    assert f[1] < 40 and 0.020 <= f[0] - first < 0.0215, f
    assert abs(f[2] - first) < 1e-9, f
    # (d) one good sample inside the run restarts it
    calls, f = run(stream(lambda i: True, flips=(130,)))
    assert calls == [CEILING_REASON], calls
    assert abs(f[2] - (t0 + 131 * period)) < 1e-9, f
    # (e) a long hole alone never confirms: the time rule still needs
    # MIN_CONSECUTIVE samples
    w, cell, homing = make_watch(mod, sps=sps)
    w.arm(t0, tare, DELTA_BOX_800, -1.)
    cell.feed([[t0, None, over, None], [t0 + 0.05, None, over, None]])
    assert homing.calls == [], homing.calls
    cell.feed([[t0 + 0.0501, None, over, None]])
    assert homing.calls == [CEILING_REASON], homing.calls
    print('ok  9 muon_ads1220 1977 sps / 20 ms batches: 40 samples, holes '
          'and half-rate streams confirm on the 20 ms span')


def integ_move(mod, rig, cell, tare, delta):
    m = tg.make_move(mod, tg.make_helper(mod, BOX_REF, trigger_percent=10),
                     tare)
    m._load_cell = cell
    m._printer = rig.printer
    m._z_min_position = -4.
    m._param_helper = ParamHelper()
    m._mcu_trigger_analog = rig.endstops[0]
    m._start_collector = lambda: Collector(m)

    def tare_fn(gcmd):
        m._tare_counts = tare
        m._trigger_delta = delta
        m._trigger_press_dir = -1.
    m._pause_and_tare = tare_fn
    # one descent, as the incident was (legs: test_z_legs.py)
    m._z_watch = mod.ZLoadBackstop(
        tg.FakeConfig(rig.printer, {'z_move_leg_time': 0.}), rig.printer,
        cell)
    return m


def test_10_configured_limits_are_in_status_and_the_log(mod):
    """KAN-3387 aid: the armed limits can be read before any probe ran."""
    import logging
    import types
    w, cell, homing = make_watch(mod)
    st = w.get_status()
    want = {'z_backstop_ceiling_counts': 0, 'z_backstop_wrong_way_counts': 0,
            'z_backstop_fired': None, 'z_backstop_armed_rules': 'none',
            'z_backstop_ceiling_factor': 2.5,
            'z_backstop_ceiling_counts_config': 0,
            'z_backstop_wrong_way_factor': 1.0,
            'z_backstop_confirm_time': 0.02,
            'z_backstop_drift_rate_counts': 0.,
            'z_backstop_leg_time': 1.0,
            'z_backstop_leg_rate_counts': 50000.,
            'z_backstop_drift_total_factor': 4.0,
            'z_backstop_press_total_factor': 1.0,
            'z_backstop_fast_leg_max': 4.0,
            'z_backstop_fast_drift_factor': 0.5,
            'z_backstop_fast_rest_time': 0.5,
            'z_backstop_fast_temp_margin': 0.3,
            'z_backstop_legs': 0,
            'z_backstop_leg_last_change': 0,
            'z_backstop_leg_total_change': 0,
            'z_backstop_leg_wrong_peak': 0,
            'z_backstop_leg_press_peak': 0}
    assert st == want, st
    w2, cell2, homing2 = make_watch(
        mod, z_move_abort_ceiling_factor=3.0, z_move_wrong_way_factor=0.75,
        z_move_abort_confirm_time=0.05, z_move_abort_ceiling_counts=70000)
    st = w2.get_status()
    assert st['z_backstop_ceiling_factor'] == 3.0, st
    assert st['z_backstop_wrong_way_factor'] == 0.75, st
    assert st['z_backstop_confirm_time'] == 0.05, st
    assert st['z_backstop_ceiling_counts_config'] == 70000, st
    assert st['z_backstop_ceiling_counts'] == 0, st
    # arming changes the descent's counts, never the configured values
    w2.arm(0., -800000, 221075, -1.)
    st = w2.get_status()
    assert st['z_backstop_ceiling_counts'] == 70000, st
    assert st['z_backstop_wrong_way_counts'] == int(0.75 * 221075), st
    assert st['z_backstop_ceiling_factor'] == 3.0, st

    class Capture(logging.Handler):
        def __init__(self):
            logging.Handler.__init__(self)
            self.lines = []

        def emit(self, record):
            self.lines.append(record.getMessage())
    cap = Capture()
    root = logging.getLogger()
    old = root.level
    root.addHandler(cap)
    root.setLevel(logging.INFO)
    try:
        p = object.__new__(mod.LoadCellPrinterProbe)
        p._config_helper = tg.make_helper(mod, BOX_REF,
                                          trigger_counts=20000)
        p._load_cell_probing_move = types.SimpleNamespace(_z_watch=w2)
        p._log_guard_settings()
    finally:
        root.removeHandler(cap)
        root.setLevel(old)
    line = [l for l in cap.lines if l.startswith('load_cell_probe:')]
    assert len(line) == 1, cap.lines
    for part in ('press_direction -1', 'min_tare_headroom 20%',
                 'ceiling 3 x delta (fixed 70000 counts',
                 'wrong way 0.75 x delta', 'confirm 0.05 s'):
        assert part in line[0], (part, line[0])
    print('ok  10 configured factors and confirm time in status before any '
          'probe; one log line states them')


class LogCapture(object):
    """Collects (level, message) of the root logger while in a with."""

    def __enter__(self):
        import logging

        class Handler(logging.Handler):
            def __init__(h, out):
                logging.Handler.__init__(h)
                h.out = out

            def emit(h, record):
                h.out.append((record.levelname, record.getMessage()))
        self.lines = []
        self.handler = Handler(self.lines)
        self.root = logging.getLogger()
        self.old = self.root.level
        self.root.addHandler(self.handler)
        self.root.setLevel(logging.INFO)
        return self

    def __exit__(self, *exc):
        self.root.removeHandler(self.handler)
        self.root.setLevel(self.old)

    def of(self, level):
        return [m for lv, m in self.lines
                if lv == level and m.startswith('load_cell_probe:')
                and 'Z load backstop' in m]


def test_11_arming_is_logged_and_deduped(mod):
    """The watch must say in klippy.log when it is not armed, and which
    rules are armed; a bed mesh must not repeat either line."""
    # Both rules: one info note, then silence while nothing changes
    w, cell, homing = make_watch(mod)
    with LogCapture() as cap:
        for _ in range(5):
            w.arm(0., -800000, 221075, -1.)
    notes = cap.of('INFO')
    assert len(notes) == 1, cap.lines
    assert 'ceiling 552687 counts' in notes[0], notes[0]
    assert 'wrong way 221075 counts' in notes[0], notes[0]
    assert not cap.of('WARNING'), cap.lines
    assert w.get_status()['z_backstop_armed_rules'] == 'ceiling+wrong_way'
    # a different limit (the tare moves every tap of a mesh) is not a
    # change of rules: no new note, and the new limits are in status
    with LogCapture() as cap:
        w.arm(0., -1750000, 126075, -1.)
        w.arm(0., -1750000, 126075, -1.)
    assert not cap.of('INFO'), cap.lines
    assert w.get_limits() == (315187, 126075), w.get_limits()
    # MCUTARE: ceiling only, the wrong-way rule off and said so
    w, cell, homing = make_watch(mod)
    with LogCapture() as cap:
        w.arm(0., -800000, 221075, -1., mcutare=True)
    notes = cap.of('INFO')
    assert len(notes) == 1, cap.lines
    assert 'ceiling 552687 counts' in notes[0], notes[0]
    assert 'wrong way off (MCUTARE' in notes[0], notes[0]
    assert w.get_status()['z_backstop_armed_rules'] == 'ceiling'
    assert w.get_limits() == (552687, 0)
    # Not armed: no trigger delta at all
    w, cell, homing = make_watch(mod)
    with LogCapture() as cap:
        for _ in range(50):
            w.arm(0., -800000, None, -1.)
    warns = cap.of('WARNING')
    assert len(warns) == 1, cap.lines
    assert 'NOT armed' in warns[0], warns[0]
    assert 'no trigger delta (None)' in warns[0], warns[0]
    assert not w.is_armed()
    assert w.get_status()['z_backstop_armed_rules'] == 'none'
    # a different reason (delta 0 is not None) warns again, once
    with LogCapture() as cap:
        for _ in range(5):
            w.arm(0., -800000, 0, -1.)
    assert len(cap.of('WARNING')) == 1, cap.lines
    assert 'no trigger delta (0)' in cap.of('WARNING')[0], cap.lines
    # armed in between: the state changed, so the same reason warns again
    with LogCapture() as cap:
        w.arm(0., -800000, 221075, -1.)
        w.arm(0., -800000, 0, -1.)
        w.arm(0., -800000, 0, -1.)
    assert len(cap.of('WARNING')) == 1, cap.lines
    assert len(cap.of('INFO')) == 1, cap.lines
    # Both factors 0: not armed, and the reason names the factors
    w, cell, homing = make_watch(mod, z_move_abort_ceiling_factor=0.,
                                 z_move_wrong_way_factor=0.)
    with LogCapture() as cap:
        w.arm(0., -800000, 221075, -1.)
        w.arm(0., -800000, 221075, -1.)
    warns = cap.of('WARNING')
    assert len(warns) == 1, cap.lines
    assert 'z_move_abort_ceiling_factor is 0' in warns[0], warns[0]
    assert 'z_move_wrong_way_factor is 0' in warns[0], warns[0]
    assert w.get_status()['z_backstop_armed_rules'] == 'none'
    assert w.get_limits() == (0, 0)
    # wrong-way only (ceiling off) reports itself
    w, cell, homing = make_watch(mod, z_move_abort_ceiling_factor=0.)
    with LogCapture() as cap:
        w.arm(0., -800000, 221075, -1.)
    assert 'ceiling off (' in cap.of('INFO')[0], cap.lines
    assert w.get_status()['z_backstop_armed_rules'] == 'wrong_way'
    print('ok  11 not-armed warns once per reason, armed note only on a '
          'change, armed rules in status')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_incident_watcher_alone, test_2_wrong_way_and_idle_wander,
             test_3_normal_probes_never_abort,
             test_4_single_spike_and_consecutive,
             test_5_start_time_and_disarm, test_6_mcutare_and_directionless,
             test_7_watcher_failure_fails_safe,
             test_8_integration_with_real_homing,
             test_9_muon_ads1220_cadence,
             test_10_configured_limits_are_in_status_and_the_log,
             test_11_arming_is_logged_and_deduped]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d Z load backstop tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d Z load backstop tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
