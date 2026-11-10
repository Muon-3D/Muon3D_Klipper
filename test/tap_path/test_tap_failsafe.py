"""Fail closed on a live stream and report inconsistent tap runs."""
import ast
import logging
import os
from types import SimpleNamespace as NS

from test_tap_overtravel import method, run


def timeout_probe():
    clock = [0.]
    client = NS(handle_batch=lambda msg: False)
    bulk = NS(client_cbs=[client.handle_batch], is_started=True)
    p = NS(_onset_client=client, adxl345=NS(batch_bulk=bulk),
           printer=NS(command_error=ValueError),
           reactor=NS(monotonic=lambda: clock[0],
                      pause=lambda t: clock.__setitem__(0, t)),
           _active_axis_homing_wrappers=[NS(get_axis=lambda: 'y')])
    p._release_onset_client = lambda: method('_release_onset_client',
                                            {'logging': logging})(p)
    return p


def test_stream_timeout_aborts_and_blocks_rearming_until_stopped():
    p = timeout_probe()
    try:
        p._release_onset_client()
    except ValueError as e:
        text = str(e)
        assert 'y home aborted' in text and 'within 1.0s' in text
        assert 'Re-run G28' in text and 'other ADXL clients' in text
    else:
        raise AssertionError('stream timeout did not abort')
    assert p._onset_client.is_finished
    try:
        method('_start_tap', {})(p, {}, 'axis_homing')
    except ValueError as e:
        assert str(e) == text
    else:
        raise AssertionError('armed with a live stream')
    # A later G28 can recover only once the old client has really stopped.
    p.adxl345.batch_bulk.client_cbs = []
    p.adxl345.batch_bulk.is_started = False
    p._release_onset_client()
    assert p._onset_client is None and p._stream_stop_error is None


def test_finally_timeout_propagates_and_resets_after_int_disabled():
    p = timeout_probe()
    calls = []
    wrapper = p._active_axis_homing_wrappers[0]
    p._stop_tap = lambda mode: calls.append('INT disabled')
    p._apply_tap_onset = lambda *a: p._release_onset_client()
    p._reset_axis_homing_state = lambda: calls.append('reset')
    p._y_tap_log = NS(record=lambda *a: None)
    try:
        method('_handle_homing_move_end', {})(
            p, NS(get_mcu_endstops=lambda: [wrapper], stepper_positions=[]))
    except ValueError as e:
        assert 'accelerometer stream' in str(e)
    else:
        raise AssertionError('finally swallowed timeout')
    assert calls == ['INT disabled', 'reset'], calls


def test_acceptance_uses_first_tap_not_predecessor():
    _, _, error = run(0.3, offsets=[0.15 * n for n in range(1, 30)],
                      samples=3)
    assert error is not None, 'a drifting run was accepted'
    assert 'y home taps disagree' in error and 'vs anchor' in error
    for offsets in ([0.15, 0.1], [0.15, 0.3, 0.3, 0.3]):
        _, _, error = run(0.3, offsets=offsets, samples=3)
        assert error is None, error


def test_other_adxl_client_cannot_keep_the_stream_alive():
    p = timeout_probe()
    p.adxl345.batch_bulk.client_cbs = [lambda msg: True]
    try:
        p._release_onset_client()
    except ValueError as e:
        assert 'other ADXL clients' in str(e)
    else:
        raise AssertionError('another client kept the stream alive')


def test_homing_preserves_timeout_alongside_an_earlier_error():
    path = os.path.join(os.path.dirname(__file__), '..', '..',
                        'klippy', 'extras', 'homing.py')
    with open(path) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'HomingMove')
    move = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name == '_homing_move')
    end = next(i for i, n in enumerate(move.body)
               if isinstance(n, ast.Try) and any(
                   isinstance(c, ast.Constant)
                   and c.value == 'homing:homing_move_end'
                   for c in ast.walk(n)))

    def fail(*args):
        raise ValueError('accelerometer stream did not stop within 1.0s')

    env = {'self': NS(printer=NS(command_error=ValueError, send_event=fail)),
           'error': 'earlier move failure'}
    module = ast.Module(body=move.body[end:-1], type_ignores=[])
    try:
        exec(compile(module, '<homing-end>', 'exec'), env)
    except ValueError as e:
        assert 'earlier move failure' in str(e)
        assert 'accelerometer stream did not stop within 1.0s' in str(e)
    else:
        raise AssertionError('timeout hidden by earlier error')


def test_exhausted_errors_include_positions_deltas_and_classes():
    for delta, kind in ((0.8, 'stall'), (-0.5, 'disagree')):
        _, _, error = run(0.3, offsets=[delta * n for n in range(1, 30)])
        assert 'y home taps disagree' in error
        assert 'initial [stepper_y=0(0.00000mm)]' in error
        assert 'delta ' in error and kind in error and 'vs anchor [' in error
    _, _, error = run(0.3, triggered=False)
    assert 'y home tap not confirmed' in error
    assert 'initial [stepper_y=0(0.00000mm)]' in error
    assert 'no_trigger halt [stepper_y=' in error
    assert 'trigger delta unavailable' in error


if __name__ == '__main__':
    for name, fn in sorted(list(globals().items())):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
