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
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<probe>', 'exec'), env)
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


def run(cap, triggered=True, seek_retries=0):
    cfg = profile({'tap_confirm_retract_dist': 8,
                   'y_tap_confirm_overtravel': cap,
                   'y_tap_confirm_samples': 2,
                   'tap_confirm_seek_retries': seek_retries})
    pos = [0., 174., 0., 0.]
    moves = []
    wrapper = NS(get_axis=lambda: 'y', last_trigger_time=0.,
                 get_steppers=lambda: [NS(get_name=lambda: 'stepper_y',
                                         get_step_dist=lambda: 0.003125)])
    tool = NS(get_position=lambda: list(pos),
              set_position=lambda p: pos.__setitem__(slice(None), p),
              move=lambda p, speed: (moves.append(('retract', pos[1], p[1])),
                                     pos.__setitem__(slice(None), p)))

    class HMove:
        def __init__(self, *a):
            pass

        def homing_move(self, target, speed, check_triggered):
            assert not check_triggered and speed == 60
            length = target[1] - pos[1]
            moves.append(('approach', pos[1], target[1]))
            wrapper.last_trigger_time = 1. if triggered else 0.
            # A prompt contact is 8mm after retract, independent of rebase.
            self.stepper_positions = [NS(stepper_name='stepper_y',
                start_pos=-2560, trig_pos=0, halt_pos=0)]
            pos[:] = target

    printer = NS(command_error=ValueError, lookup_object=lambda n: tool,
                 get_start_args=lambda: {})
    owner = NS(printer=printer, axis_homing_arm_delay={},
               _sim_taps={}, _trace_phase=None)
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
    moves, state, error = run(0.3, triggered=False)
    length = moves[1][2] - moves[1][1]
    # Commanded steps beyond the known contact cannot span a 256-step cycle.
    assert round((length - 8.) / 0.003125) == 96 < 256
    assert state.trigger_mcu_pos == {'stepper_y': 0}
    assert 'approach capped at 0.300mm' in error
    assert 'previous tap spurious or stop further on' in error
    moves, state, error = run(0.3, triggered=False, seek_retries=1)
    assert len(moves) == 3
    assert 'resumed seek' in error and 'approach capped at 0.300mm' in error
    _, _, error = run(None, triggered=False)
    assert 'capped' not in error


if __name__ == '__main__':
    for name, fn in sorted(list(globals().items())):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
