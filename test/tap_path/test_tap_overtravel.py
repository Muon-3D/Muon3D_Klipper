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


class Config:
    error = ValueError

    def __init__(self, values):
        self.values = values

    def getfloat(self, name, default, **kw):
        return self.values.get(name, default)

    getint = getfloat


def profile(values):
    return method('_load_confirm_profile', {})(NS(
        _default_stall_dist=lambda *a: 0.8), Config(values), 'y')


def test_validation_and_axis_override():
    for value in (-1, 0, 0.149, 8, 9):
        try:
            profile({'tap_confirm_retract_dist': 8,
                     'y_tap_confirm_overtravel': value})
        except ValueError as e:
            assert 'y_tap_confirm_overtravel' in str(e)
            if value < 0.15:
                assert 'deceleration distance' in str(e)
        else:
            raise AssertionError(value)
    assert profile({'tap_confirm_overtravel': 0.2})['overtravel'] == 0.2
    assert profile({'tap_confirm_overtravel': 0.2,
                    'y_tap_confirm_overtravel': 0.3})['overtravel'] == 0.3
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
    triggers = None if isinstance(triggered, bool) else iter(triggered)

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
            self.stepper_positions = [NS(stepper_name='stepper_y',
                start_pos=-2560, trig_pos=0, halt_pos=0)]
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
    for cap, length in ((0.3, 8.3), (None, 16.)):
        moves, state, error = run(cap)
        assert error is None, error
        assert moves[0] == ('retract', 174., 166.)
        assert abs(moves[1][2] - moves[1][1] - length) < 1e-9
        assert state.trigger_mcu_pos == {'stepper_y': 0}


def test_missed_tap_is_capped_not_a_cycle_and_is_not_accepted():
    physical = []
    moves, state, error = run(0.3, triggered=False, seek_retries=1,
                              physical=physical)
    length = moves[1][2] - moves[1][1]
    # Commanded steps beyond the known contact cannot span a 256-step cycle.
    assert round((length - 8.) / 0.003125) == 96 < 256
    assert state.trigger_mcu_pos == {'stepper_y': 0}
    assert len(moves) == 6, moves
    assert all(abs(end - start) <= 8.3 + 1e-9
               for _, start, end in moves), moves
    assert all(abs(end - 0.3) < 1e-9 for end in physical), physical
    assert ('y home aborted: tap not detected within 0.300mm of the'
            ' previous tap 3 times; either the tap threshold is too high'
            ' or the previous tap was spurious. Seek not resumed, to avoid'
            ' driving into the stop.') in error
    assert error.count('missed halt') == 3


def test_uncapped_no_trigger_still_resumes_seek():
    moves, _, error = run(None, triggered=False, seek_retries=1)
    assert len(moves) == 3
    assert moves[-1][2] - moves[-1][1] > 16.
    assert 'no tap within' in error and 'resumed seek' in error
    _, _, error = run(None, triggered=False)
    assert 'capped' not in error


def test_missed_and_disagree_share_retry_budget():
    moves, _, error = run(0.3, triggered=[False, True], offsets=[-0.5],
                           retries=1)
    assert len(moves) == 4
    assert 'taps disagree' in error and 'missed halt' in error
    moves, _, error = run(0.3, triggered=False, retries=0)
    assert len(moves) == 2 and 'previous tap 1 times' in error


if __name__ == '__main__':
    for name, fn in sorted(list(globals().items())):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
