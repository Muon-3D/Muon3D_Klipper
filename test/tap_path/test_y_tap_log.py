"""Always-on stopped-state diagnostics and hypothesis parser tests."""
import ast
import io
import logging
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(__file__))
import _paths
sys.path.insert(0, _paths.patched('klippy'))
sys.path.insert(0, _paths.patched('scripts'))
from extras import y_tap_log as yt
import y_tap_log_summary as summary


def fixture(fail=False):
    reads = []

    def read(reg):
        reads.append(reg)
        if fail:
            raise ValueError('SPI unavailable')
        return 123

    drv = NS(mcu_tmc=NS(get_register=read), fields=NS(
        get_field=lambda name, raw: 42 if name == 'mscnt' else 0,
        lookup_register=lambda *a: 'MSCNT'))
    steppers = [NS(get_name=lambda n=n: n, get_step_dist=lambda: 0.003125,
                   get_mcu_position=lambda: 0)
                for n in yt.STEPPERS]
    owner = NS(_lookup_tmc=lambda n: drv,
               _axis_wrappers={'y': NS(last_trigger_time=1.)},
               tap_confirm={'y': {'tolerance': 0.2, 'stall_dist': 0.8}})
    move = NS(toolhead=NS(get_position=lambda: [0., 158., 0., 0.],
              get_kinematics=lambda: NS(get_steppers=lambda: steppers)),
              command_end=[0., 174., 0., 0.],
              stepper_positions=[NS(stepper_name=n, start_pos=-sign * 2560,
                  trig_pos=sign * 256, halt_pos=sign * 260)
                  for n, sign in zip(yt.STEPPERS, (1, -1))])
    return owner, move, reads


def capture(fail=False, no_trigger=False, aborted=False):
    owner, move, reads = fixture(fail)
    if no_trigger:
        owner._axis_wrappers['y'].last_trigger_time = 0.
    stream = io.StringIO()
    handler = logging.StreamHandler(stream)
    root = logging.getLogger()
    level = root.level
    root.setLevel(logging.INFO)
    root.addHandler(handler)
    try:
        log = yt.YTapLog(owner)
        log.begin()
        log.kind = 'confirm'
        log.anchor = dict.fromkeys(yt.STEPPERS, 0)
        log.start(move)
        log.record(move, 'stream did not stop' if aborted else None,
                   halted=not aborted)
        # Repeated cleanup must not duplicate the attempt or summary.
        log.record(move)
        short = log.finish('failed' if aborted or no_trigger else 'accepted')
        assert 'y tap summary:' in short
        assert log.finish('failed') == ''
    finally:
        root.removeHandler(handler)
        root.setLevel(level)
    return stream.getvalue().splitlines(), reads


def test_line_format_decodes_fields_and_positions_without_streaming():
    lines, reads = capture()
    assert len(lines) == 2
    row = summary.parse_line(lines[0], 'y tap attempt:')
    assert row['attempt'] == 1 and row['kind'] == 'confirm'
    assert row['classification'] == 'stall'
    assert row['command_start'] == 158 and row['command_end'] == 174
    assert row['trigger_mm'] == 0.8 and row['halt_mm'] == 0.8125
    assert row['delta'] == 0.8
    for name in yt.STEPPERS:
        assert row['drv_status'][name]['mscnt'] == 42
        assert set(row['drv_status'][name]) == set(yt.FIELDS) | {'mscnt'}
        assert name in row['start_steps'] and name in row['trigger_steps']
        assert name in row['halt_steps']
    assert reads == ['DRV_STATUS', 'MSCNT'] * 2


def test_failed_driver_reads_never_propagate():
    lines, reads = capture(fail=True)
    row = summary.parse_line(lines[0], 'y tap attempt:')
    assert reads == ['DRV_STATUS'] * 2
    assert all(v == 'unavailable(SPI unavailable)'
               for v in row['drv_status'].values())
    assert row['classification'] == 'stall'


def test_no_trigger_and_unverified_abort_are_recorded_once():
    lines, _ = capture(no_trigger=True)
    row = summary.parse_line(lines[0], 'y tap attempt:')
    assert row['classification'] == 'no_trigger'
    assert row['trigger_mm'] is None
    lines, reads = capture(aborted=True)
    row = summary.parse_line(lines[0], 'y tap attempt:')
    assert row['classification'] == 'aborted' and reads == []
    assert len(lines) == 2


def synthetic(delta=0., sg=100, halt=0., **kw):
    row = dict(home=1, attempt=1, kind='confirm', classification='agree',
               trigger_mm=delta, halt_mm=halt, delta=delta,
               drv_status={'stepper_x': {'sg_result': sg, 'cs_actual': 20}},
               start_steps={'stepper_x': -2560, 'stepper_y': 2560},
               trigger_steps={'stepper_x': 0, 'stepper_y': 0},
               halt_steps={'stepper_x': 1, 'stepper_y': -1})
    row.update(kw)
    return row


def hypothesis(rows, error=None):
    lines = [yt.line('y tap attempt:', r) for r in rows]
    lines.append(yt.line('y tap summary:', {
        'home': 1, 'result': 'failed' if error else 'accepted',
        'error': error}))
    return summary.verdict(summary.read_homes(lines)[1])


def test_parser_hypotheses_and_empty_input():
    assert summary.read_homes([]) == {}
    assert summary.read_homes(['y tap attempt: broken']) == {}
    assert summary.read_homes(['y tap attempt: attempt=1']) == {}
    assert hypothesis([synthetic()]) == 'ok'
    assert hypothesis([synthetic(0.8, sg=0)]) == 'pole_slip'
    assert hypothesis([synthetic(-0.4)]) == 'early_trigger'
    assert hypothesis([synthetic(), synthetic(0.35, halt=0.35)]) == 'binding'
    assert hypothesis([synthetic()],
                      'accelerometer stream did not stop') == 'sensor_race'
    row = synthetic(halt_steps={'stepper_x': -2560, 'stepper_y': 2560})
    assert hypothesis([row]) == 'sensor_race'
    assert hypothesis([synthetic(command_start=158, command_end=174,
                      halt_travel_mm=16., halt_follow_mm=8.)]) == 'sensor_race'
    assert hypothesis([synthetic(0.8)]) == 'ok'
    assert hypothesis([synthetic(), synthetic(0.8, drv_status={
        'stepper_x': {'sg_result': 100, 'cs_actual': 10}})]) == 'pole_slip'
    assert hypothesis([synthetic(-0.4, step_anomaly=True)]) == 'ok'


def test_failed_home_summary_is_appended_to_command_error():
    with open(_paths.patched('klippy/extras/homing.py')) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'HomingMove')
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name == 'homing_move')
    env = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<homing>', 'exec'),
         env)

    def fail(*a):
        raise ValueError('no trigger')

    def event(name, move, error):
        assert name == 'homing:homing_move_error'
        assert str(error) == 'no trigger'
        return ['y tap summary: home=1 result="failed" attempts=1']

    move = NS(printer=NS(command_error=ValueError, send_event=event),
              _homing_move=fail)
    try:
        env['homing_move'](move, [0., 174., 0., 0.], 60.)
    except ValueError as e:
        assert 'no trigger; y tap summary:' in str(e)
    else:
        raise AssertionError('failed home did not propagate')


if __name__ == '__main__':
    for name, fn in sorted(list(globals().items())):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
