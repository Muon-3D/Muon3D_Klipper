"""Exercise the real confirm loop and config loader without hardware."""
import ast
import logging
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(__file__))
import _paths
sys.path.insert(0, _paths.patched('klippy/extras'))
import tap_confirm


def method(name, env):
    with open(_paths.patched('klippy/extras/adxl345_probe.py')) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'ADXL345Probe')
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name == name)
    module = ast.Module(body=[node], type_ignores=[])
    exec(compile(module, '<probe>', 'exec'), env)
    return env[name]


class Section:
    def __init__(self, values):
        self.values = values

    def get(self, name, default=None, **kw):
        return self.values.get(name, default)

    getfloat = getint = get


class Config:
    error = ValueError

    def __init__(self, values, sections=None):
        self.values = values
        self.sections = ({'stepper_y': {'rotation_distance': 40.}}
                         if sections is None else sections)

    def getfloat(self, name, default, **kw):
        return self.values.get(name, default)

    getint = getfloat

    def has_section(self, name):
        return name in self.sections

    def getsection(self, name):
        return Section(self.sections[name])


# M1 Y: rotation_distance 40 / 200 full steps.  A blocked rotor slips 2 full
# steps = 0.4mm = 128 microsteps of 0.003125mm.
FULL_STEP = 0.2
SLIP_STEPS = 128


def profile(values, sections=None):
    # The REAL _full_step_dist, reading a fake [stepper_y] section.
    values = dict(values)
    if any('overtravel' in k for k in values):
        # A capped config must keep tolerance under the cap.
        values.setdefault('y_tap_confirm_tolerance', 0.1)
    owner = NS(_default_stall_dist=lambda *a: 0.8)
    owner._full_step_dist = lambda *a: method('_full_step_dist', {})(
        owner, *a)
    return method('_load_confirm_profile', {})(
        owner, Config(values, sections), 'y')


def test_validation_and_axis_override():
    for value in (-1, 0, 0.149, 8, 9, 0.21, 0.4, 0.45):
        try:
            profile({'tap_confirm_retract_dist': 8,
                     'y_tap_confirm_overtravel': value})
        except ValueError as e:
            assert 'y_tap_confirm_overtravel' in str(e)
            if value < 0.15:
                assert 'deceleration distance' in str(e)
            if 0.21 <= value < 8:
                assert 'at most 1 full step (0.200mm)' in str(e)
        else:
            raise AssertionError(value)
    assert profile({'tap_confirm_overtravel': 0.2})['overtravel'] == 0.2
    assert profile({'tap_confirm_overtravel': 0.15,
                    'y_tap_confirm_overtravel': 0.2})['overtravel'] == 0.2
    assert profile({'tap_confirm_overtravel': 0.15})['overtravel'] == 0.15
    assert profile({})['overtravel'] is None


def run(cap, triggered=True, seek_retries=0, offsets=None, samples=2,
        retries=2, physical=None):
    cfg = profile({'tap_confirm_retract_dist': 8,
                   'y_tap_confirm_overtravel': cap,
                   'y_tap_confirm_samples': samples,
                   'tap_confirm_retries': retries,
                   'tap_confirm_seek_retries': seek_retries})
    pos = [0., 174., 0., 0.]
    moves = []
    wrapper = NS(get_axis=lambda: 'y', last_trigger_time=0.,
                 get_steppers=lambda: [NS(get_name=lambda: 'stepper_y',
                                         get_step_dist=lambda: 0.003125)])
    distance = [0.]

    def move(target, speed):
        moves.append(('retract', pos[1], target[1]))
        distance[0] += target[1] - pos[1]
        pos[:] = target

    tool = NS(get_position=lambda: list(pos),
              set_position=lambda p: pos.__setitem__(slice(None), p),
              move=move)
    triggers = (None if isinstance(triggered, (bool, str))
                else iter(triggered))

    class HMove:
        def __init__(self, *a):
            pass

        def homing_move(self, target, speed, check_triggered):
            assert not check_triggered and speed == 60
            length = target[1] - pos[1]
            moves.append(('approach', pos[1], target[1]))
            contact = triggered if triggers is None else next(triggers)
            wrapper.last_trigger_time = 1. if contact else 0.
            distance[0] += length
            if physical is not None:
                physical.append(distance[0])
            # A prompt contact is 8mm after retract, independent of rebase.
            trig_pos = 0
            if contact == 'end':
                # Triggers where the commanded steps end.
                trig_pos = -2560 + round(length / 0.003125)
            elif contact == 'near':
                trig_pos = -2560 + round(length / 0.003125) - 1
            self.stepper_positions = [NS(stepper_name='stepper_y',
                start_pos=-2560, trig_pos=trig_pos, halt_pos=trig_pos)]
            pos[:] = target

    printer = NS(command_error=ValueError, lookup_object=lambda n: tool,
                 get_start_args=lambda: {'debuginput': 'test'}
                 if offsets is not None else {})
    owner = NS(printer=printer, axis_homing_arm_delay={},
               _sim_taps={} if offsets is None else {'y': list(offsets)},
               _trace_phase=None)
    hi = NS(positive_dir=True, position_endstop=174., speed=60.,
            retract_speed=60.)
    rail = NS(get_homing_info=lambda: hi, get_range=lambda: (-10., 174.1),
              get_endstops=lambda: [])
    state = NS(trigger_mcu_pos={'stepper_y': 0})
    error = None
    try:
        method('_confirm_axis_home', {'tap_confirm': tap_confirm,
            'homing': NS(HomingMove=HMove), 'logging': logging})(
                owner, state, rail, wrapper, cfg)
    except ValueError as e:
        error = str(e)
    return moves, state, error


def test_capped_and_legacy_length_and_trigger_mapping():
    for cap, length in ((0.2, 8.2), (None, 16.)):
        moves, state, error = run(cap)
        assert error is None, error
        assert moves[0] == ('retract', 174., 166.)
        assert abs(moves[1][2] - moves[1][1] - length) < 1e-9
        assert state.trigger_mcu_pos == {'stepper_y': 0}


def test_missed_tap_is_capped_not_a_cycle_and_is_not_accepted():
    physical = []
    moves, state, error = run(0.2, triggered=False, seek_retries=1,
                              physical=physical)
    length = moves[1][2] - moves[1][1]
    # Commanded steps beyond the known contact must stay under the 2 full
    # steps (128 microsteps) a blocked rotor slips.
    assert round((length - 8.) / 0.003125) == 64 < SLIP_STEPS
    assert state.trigger_mcu_pos == {'stepper_y': 0}
    assert len(moves) == 6, moves
    assert all(abs(end - start) <= 8.2 + 1e-9
               for _, start, end in moves), moves
    assert all(abs(end - 0.2) < 1e-9 for end in physical), physical
    assert ('y home aborted: tap not detected within 0.200mm of the'
            ' previous tap 3 times; either the tap threshold is too high'
            ' or the previous tap was spurious. Seek not resumed, to avoid'
            ' driving into the stop.') in error
    assert error.count('missed halt') == 3


def test_core_cfg_cap_is_accepted_and_stays_under_the_slip():
    assert profile({'y_tap_confirm_overtravel': 0.15})['overtravel'] == 0.15
    # Mirror the shipped config so the two cannot drift apart.
    cfg = os.path.join(os.path.dirname(__file__), '..', '..', 'core', 'M1',
                       'core.cfg')
    with open(cfg) as f:
        lines = [l.strip() for l in f if l.startswith(
            ('y_tap_confirm_overtravel:', 'y_tap_confirm_tolerance:'))]
    assert lines == ['y_tap_confirm_overtravel: 0.15',
                     'y_tap_confirm_tolerance: 0.1'], lines
    cap = float(lines[0].split(':')[1])
    tolerance = float(lines[1].split(':')[1])
    assert tolerance < cap, (tolerance, cap)
    assert profile({'y_tap_confirm_overtravel': cap,
                    'y_tap_confirm_tolerance': tolerance})
    physical = []
    moves, _, error = run(0.15, triggered=False, seek_retries=1,
                          physical=physical)
    steps = round((moves[1][2] - moves[1][1] - 8.) / 0.003125)
    assert steps == 48 < SLIP_STEPS, steps
    assert all(abs(end - 0.15) < 1e-9 for end in physical), physical
    assert 'within 0.150mm' in error, error


def test_cap_above_one_full_step_is_rejected():
    for value in (0.2, 0.2 - 0.001):
        assert profile({'tap_confirm_retract_dist': 8,
                        'y_tap_confirm_overtravel': value}
                       )['overtravel'] == value
    for value in (0.21, 0.4, 0.5, 1.):
        try:
            profile({'tap_confirm_retract_dist': 8,
                     'y_tap_confirm_overtravel': value})
        except ValueError as e:
            assert 'at most 1 full step' in str(e), e
        else:
            raise AssertionError(value)


def test_real_full_step_dist_from_a_fake_section():
    fsd = method('_full_step_dist', {})
    owner = NS()
    assert fsd(owner, Config({}), 'y') == 0.2
    sec = {'stepper_y': {'rotation_distance': 40.,
                         'full_steps_per_rotation': 400}}
    assert fsd(owner, Config({}, sec), 'y') == 0.1
    for sections, text in (
            ({'stepper_y': {'rotation_distance': 40., 'gear_ratio': '3:1'}},
             'gear_ratio'),
            ({}, 'needs a [stepper_y] section'),
            ({'stepper_y': {}}, 'needs rotation_distance')):
        try:
            profile({'y_tap_confirm_overtravel': 0.15}, sections)
        except ValueError as e:
            assert text in str(e), e
        else:
            raise AssertionError(text)
    # No cap configured: no stepper lookup, no error.
    assert profile({}, {})['overtravel'] is None
    # A 0.1 mm full step (400 full steps) rejects 0.15.
    try:
        profile({'y_tap_confirm_overtravel': 0.15}, sec)
    except ValueError as e:
        assert 'at most 1 full step (0.100mm)' in str(e), e
    else:
        raise AssertionError('0.15 accepted on a 0.1mm full step')


def test_tolerance_must_be_under_the_cap():
    for tol in (0.15, 0.2, 0.3):
        try:
            profile({'y_tap_confirm_overtravel': 0.15,
                     'y_tap_confirm_tolerance': tol})
        except ValueError as e:
            assert ('y_tap_confirm_tolerance must be less than'
                    ' y_tap_confirm_overtravel: otherwise every capped'
                    ' tap agrees') in str(e), e
        else:
            raise AssertionError(tol)
    assert profile({'y_tap_confirm_overtravel': 0.15,
                    'y_tap_confirm_tolerance': 0.149})['tolerance'] == 0.149


def test_capped_tap_at_the_commanded_end_is_late_and_never_accepted():
    physical = []
    moves, state, error = run(0.15, triggered='end', seek_retries=1,
                              physical=physical)
    assert state.trigger_mcu_pos == {'stepper_y': 0}
    assert len(moves) == 6, moves  # retract+approach x3, no resumed seek
    assert all(abs(end - 0.15) < 1e-9 for end in physical), physical
    assert error.count('late halt') == 3 and 'missed halt' not in error
    assert 'y home aborted: tap not detected within 0.150mm' in error
    assert 'Seek not resumed' in error
    # One step short of the end is a real measurement, but 0.147mm past the
    # anchor is over the 0.1 tolerance: a stall, not a late tap.
    moves, state, error = run(0.15, triggered='near', retries=0)
    assert 'late halt' not in error and 'stall' in error, error
    # Uncapped axes keep accepting a trigger at the end of the approach.
    moves, state, error = run(None, triggered='end')
    assert error is None, error


def test_late_tap_shares_the_budget_with_disagreement():
    moves, _, error = run(0.15, triggered=['end', True], retries=1)
    assert error is None and len(moves) == 4, (moves, error)
    # The late tap used the only retry; a second miss aborts.
    moves, _, error = run(0.15, triggered=['end', 'end'], retries=1)
    assert len(moves) == 4 and 'previous tap 2 times' in error, error
    moves, _, error = run(0.15, triggered=['end'], retries=0)
    assert len(moves) == 2 and 'previous tap 1 times' in error


def test_capped_tolerance_makes_a_cap_end_tap_a_stall_not_agree():
    got = {}
    for off in (0.05, 0.12):
        logs = []
        handler = logging.Handler()
        handler.emit = lambda r, logs=logs: logs.append(r.getMessage())
        root = logging.getLogger()
        root.addHandler(handler)
        level = root.level
        root.setLevel(logging.INFO)
        try:
            moves, _, error = run(0.15, offsets=[off], retries=0)
        finally:
            root.removeHandler(handler)
            root.setLevel(level)
        got[off] = (error, [m for m in logs if 'tap compare' in m])
    assert got[0.05][0] is None, got[0.05]
    assert '(agree)' in got[0.05][1][0], got[0.05]
    error, compare = got[0.12]
    assert '(stall)' in compare[0], compare
    assert error is not None and 'taps disagree' in error
    assert '1 taps were driven into the stop' in error, error


def test_uncapped_no_trigger_still_resumes_seek():
    moves, _, error = run(None, triggered=False, seek_retries=1)
    assert len(moves) == 3
    assert moves[-1][2] - moves[-1][1] > 16.
    assert 'no tap within' in error and 'resumed seek' in error
    _, _, error = run(None, triggered=False)
    assert 'capped' not in error


def test_missed_and_disagree_share_retry_budget():
    moves, _, error = run(0.2, triggered=[False, True], offsets=[-0.5],
                           retries=1)
    assert len(moves) == 4
    assert 'taps disagree' in error and 'missed halt' in error
    moves, _, error = run(0.2, triggered=False, retries=0)
    assert len(moves) == 2 and 'previous tap 1 times' in error


if __name__ == '__main__':
    for name, fn in sorted(list(globals().items())):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
