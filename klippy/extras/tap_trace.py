# Per-tap trace of ADXL345 X/Y tap homing (KAN-1361 diagnostic).
#
# TAP_TRACE ENABLE=1 (or G28 ... TAP_TRACE=1) makes adxl345_probe record,
# for every tap of a traced axis, the raw accelerometer samples around the
# chip's INT edge, the INT time, the sample onset time (tap_onset) and the
# stepper positions at each, and append them to a CSV in the logs dir.
# Nothing here changes homing: it only writes what it is handed.
#
# One row per sample; the tap's own fields repeat on each of its rows, so
# the file loads straight into a spreadsheet or pandas. A tap with no
# samples (no INT, or the stream did not arrive) still gets one row with
# the sample fields empty. scripts/tap_trace_summary.py reads it back.
#
# Positions: *_pos_mm is the axis position computed from the raw MCU step
# counts (steps * step_dist through the kinematics), so it is comparable
# across the taps of one Klipper run even though homing re-bases the
# toolhead position between them. *_steps lists each stepper's raw count.
# Pure module: no klippy imports, so it can be unit tested anywhere.
import os
import time

FIELDS = [
    'home', 'tap', 'phase', 'axis', 'speed', 'triggered', 'applied',
    'int_time', 'onset_time', 'int_minus_onset_ms',
    'start_pos_mm', 'onset_pos_mm', 'int_pos_mm', 'halt_pos_mm',
    'onset_steps', 'int_steps', 'halt_steps',
    'thresh', 'tap_axes',
    'sample_time', 't_rel_int_ms', 'ax', 'ay', 'az', 'tap_mag',
    'over_thresh',
]
SAMPLE_FIELDS = ('sample_time', 't_rel_int_ms', 'ax', 'ay', 'az',
                 'tap_mag', 'over_thresh')


def format_steps(steps):
    if not steps:
        return ''
    return ';'.join('%s:%d' % (n, steps[n]) for n in sorted(steps))


def parse_steps(text):
    out = {}
    for part in (text or '').split(';'):
        if part:
            name, val = part.rsplit(':', 1)
            out[name] = int(val)
    return out


def _f(value, fmt):
    if value is None:
        return ''
    return fmt % (value,)


def window_samples(samples, int_time, pre, post):
    """The (t, x, y, z) samples from int_time - pre to int_time + post."""
    if int_time is None:
        return []
    lo, hi = int_time - pre, int_time + post
    return [s for s in samples if lo <= s[0] <= hi]


def tap_magnitude(sample, axes):
    use = ['xyz'.index(a) + 1 for a in axes]
    return max(abs(sample[i]) for i in use) if use else 0.


def tap_rows(tap, samples, pre, post):
    """CSV rows (dicts keyed by FIELDS) for one tap.

    tap: dict with home, tap, phase, axis, speed, int_time (None if no
    INT), onset_time, applied ('int' or 'onset'), thresh, tap_axes, and
    *_pos_mm / *_steps for start, onset, int and halt (any may be None).
    """
    int_time = tap.get('int_time')
    onset = tap.get('onset_time')
    axes = tap.get('tap_axes') or tap.get('axis') or ''
    thresh = tap.get('thresh')
    delay = None
    if int_time is not None and onset is not None:
        delay = (int_time - onset) * 1000.
    base = {
        'home': tap.get('home', ''),
        'tap': tap.get('tap', ''),
        'phase': tap.get('phase', ''),
        'axis': tap.get('axis', ''),
        'speed': _f(tap.get('speed'), '%.3f'),
        'triggered': 1 if int_time is not None else 0,
        'applied': tap.get('applied', 'int'),
        'int_time': _f(int_time, '%.6f'),
        'onset_time': _f(onset, '%.6f'),
        'int_minus_onset_ms': _f(delay, '%.3f'),
        'thresh': _f(thresh, '%.1f'),
        'tap_axes': axes,
    }
    for what in ('start', 'onset', 'int', 'halt'):
        base[what + '_pos_mm'] = _f(tap.get(what + '_pos_mm'), '%.5f')
        if what != 'start':
            base[what + '_steps'] = format_steps(tap.get(what + '_steps'))
    rows = []
    for s in window_samples(samples, int_time, pre, post):
        mag = tap_magnitude(s, axes)
        row = dict(base)
        row.update({
            'sample_time': '%.6f' % (s[0],),
            't_rel_int_ms': '%.3f' % ((s[0] - int_time) * 1000.,),
            'ax': '%.1f' % (s[1],), 'ay': '%.1f' % (s[2],),
            'az': '%.1f' % (s[3],), 'tap_mag': '%.1f' % (mag,),
            'over_thresh': '' if thresh is None else int(mag >= thresh),
        })
        rows.append(row)
    if not rows:
        row = dict(base)
        row.update({k: '' for k in SAMPLE_FIELDS})
        rows.append(row)
    return rows


def _csv_line(values):
    out = []
    for v in values:
        v = str(v)
        if any(c in v for c in ',"\n\r'):
            v = '"%s"' % (v.replace('"', '""'),)
        out.append(v)
    return ','.join(out) + '\n'


def trace_path(directory, now=None):
    """A new file name in `directory`, never an existing one."""
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now))
    base = os.path.join(directory, 'tap_trace-%s' % (stamp,))
    path = base + '.csv'
    n = 1
    while os.path.exists(path):
        path = '%s-%d.csv' % (base, n)
        n += 1
    return path


class TapTraceWriter:
    """Appends tap rows to one CSV; flushed after each tap so a klippy
    crash or a failed home still leaves every tap before it on disk."""

    def __init__(self, path, pre, post):
        self.path = path
        self.pre = pre
        self.post = post
        self.taps = 0
        self.rows = 0
        self._fh = open(path, 'w')
        self._fh.write(_csv_line(FIELDS))
        self._fh.flush()

    def write_tap(self, tap, samples):
        rows = tap_rows(tap, samples, self.pre, self.post)
        self._fh.write(''.join(_csv_line([r[k] for k in FIELDS])
                               for r in rows))
        self._fh.flush()
        self.taps += 1
        self.rows += len(rows)
        return len(rows)

    def close(self):
        fh, self._fh = self._fh, None
        if fh is not None:
            fh.close()

    @property
    def closed(self):
        return self._fh is None
