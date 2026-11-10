"""Stopped-state Y homing diagnostics, independent of ADXL streaming."""
import json
import logging

from . import tap_confirm

FIELDS = ('sg_result', 'stallguard', 'stst', 'ola', 'olb', 's2ga',
          's2gb', 'otpw', 'ot', 'cs_actual')
STEPPERS = ('stepper_x', 'stepper_y')


def line(prefix, values):
    return prefix + ' ' + ' '.join(
        '%s=%s' % (k, json.dumps(v, separators=(',', ':')))
        for k, v in values.items())


def driver_status(owner, name):
    try:
        driver = owner._lookup_tmc(name)
        if driver is None:
            raise ValueError('driver not configured')
        raw = driver.mcu_tmc.get_register('DRV_STATUS')
        data = {name: driver.fields.get_field(name, raw) for name in FIELDS}
        reg = driver.fields.lookup_register('mscnt', None)
        data['mscnt'] = driver.fields.get_field(
            'mscnt', driver.mcu_tmc.get_register(reg))
        return data
    except Exception as e:
        return 'unavailable(%s)' % str(e)


class YTapLog:
    def __init__(self, owner):
        self.owner = owner
        self.home = 0
        self.active = False
        self.pending = None

    def begin(self):
        self.home += 1
        self.active = True
        self.attempt = 0
        self.anchor = None
        self.counts = {}
        self.pending = None
        self.kind = 'seek'

    def start(self, hmove):
        if not self.active:
            self.begin()
        self.attempt += 1
        self.pending = {
            'home': self.home, 'attempt': self.attempt, 'kind': self.kind,
            'command_start': hmove.toolhead.get_position()[1],
            'command_end': hmove.command_end[1]}
        self.pending['start_steps'] = {
            s.get_name(): s.get_mcu_position()
            for s in hmove.toolhead.get_kinematics().get_steppers()
            if s.get_name() in STEPPERS}

    def record(self, hmove, error=None, halted=True):
        if self.pending is None:
            return
        row, self.pending = self.pending, None
        owner = self.owner
        cfg = owner.tap_confirm['y']
        steppers = {s.get_name(): s for s in
                    hmove.toolhead.get_kinematics().get_steppers()}
        sps = {sp.stepper_name: sp for sp in hmove.stepper_positions}
        triggered = (halted and
                     owner._axis_wrappers['y'].last_trigger_time > 0.)
        for stage, attr in (('start', 'start_pos'), ('trigger', 'trig_pos'),
                            ('halt', 'halt_pos')):
            raw = getattr(hmove, 'y_tap_raw_steps', None)
            before = row.get(stage + '_steps', {})
            row[stage + '_steps'] = (dict(raw[stage]) if raw else {
                n: getattr(sps.get(n), attr, before.get(n)) for n in STEPPERS})
            if stage == 'trigger' and not triggered:
                row[stage + '_steps'] = dict.fromkeys(STEPPERS)

        def y_pos(stage):
            steps = row[stage + '_steps']
            if any(steps[n] is None or n not in steppers for n in STEPPERS):
                return None
            return (steps['stepper_x'] * steppers['stepper_x'].get_step_dist()
                    - steps['stepper_y']
                    * steppers['stepper_y'].get_step_dist()) * 0.5

        row['trigger_mm'] = y_pos('trigger')
        row['halt_mm'] = y_pos('halt')
        start_mm = y_pos('start')
        row['halt_travel_mm'] = (row['halt_mm'] - start_mm
                                 if row['halt_mm'] is not None
                                 and start_mm is not None else None)
        row['halt_follow_mm'] = (row['halt_mm'] - row['trigger_mm']
                                 if row['halt_mm'] is not None
                                 and row['trigger_mm'] is not None else None)
        row['step_anomaly'] = False
        if triggered and all(row['start_steps'][n] is not None
                             and row['trigger_steps'][n] is not None
                             for n in STEPPERS):
            x_drift = sum((row['trigger_steps'][n] - row['start_steps'][n])
                          * steppers[n].get_step_dist() for n in STEPPERS) / 2.
            row['step_anomaly'] = abs(x_drift) > cfg['tolerance']
        row['delta'] = None
        row['classification'] = 'aborted' if error else 'no_trigger'
        trig = row['trigger_steps']
        if triggered and all(trig[n] is not None for n in STEPPERS):
            if self.anchor is None or row['kind'] != 'confirm':
                row['classification'] = 'contact' if not error else 'aborted'
                self.anchor = dict(trig)
            else:
                dirs = tap_confirm.tap_dirs(row['start_steps'], trig)
                row['delta'] = tap_confirm.signed_delta(
                    self.anchor, trig, dirs,
                    {n: steppers[n].get_step_dist() for n in STEPPERS})
                kind = tap_confirm.classify(row['delta'], cfg['tolerance'],
                                            cfg['stall_dist'])
                row['classification'] = 'aborted' if error else kind
                if kind != tap_confirm.AGREE:
                    self.anchor = dict(trig)
        row['tolerance'] = cfg['tolerance']
        row['cycle'] = cfg['stall_dist']
        row['error'] = str(error) if error else None
        row['drv_status'] = {
            n: driver_status(owner, n) if halted
            else 'unavailable(attempt aborted before halt was verified)'
            for n in STEPPERS}
        kind = row['classification']
        self.counts[kind] = self.counts.get(kind, 0) + 1
        logging.info('%s', line('y tap attempt:', row))

    def finish(self, result, error=None):
        if not self.active:
            return ''
        self.active = False
        text = line('y tap summary:', {
            'home': self.home, 'result': result, 'attempts': self.attempt,
            'classes': self.counts, 'error': str(error) if error else None})
        logging.info('%s', text)
        return line('y tap summary:', {
            'home': self.home, 'result': result, 'attempts': self.attempt,
            'classes': self.counts})
