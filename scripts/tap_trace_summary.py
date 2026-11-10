#!/usr/bin/env python3
"""Summarise (and optionally plot) a TAP_TRACE CSV (KAN-1361).

    python3 scripts/tap_trace_summary.py tap_trace-20261109-130000.csv
    python3 scripts/tap_trace_summary.py trace.csv --plot trace.png

The CSV comes from adxl345_probe's TAP_TRACE (klippy/extras/tap_trace.py):
one row per accelerometer sample, the tap's own fields repeated on each.

Per tap it prints the INT - onset delay, the position at the onset and at
INT, how far each moved from the previous tap of the same home, and the
overrun from INT to the halt.  Per home it prints the spread of the
positions and whether consecutive taps moved one way only (the ratchet
seen on Boxwood).  --plot (needs matplotlib) draws the tap signal around
INT for every tap, and the onset and INT positions against tap number.
Standard library only, apart from the optional plot.
"""
import argparse
import csv
import sys

TAP_KEYS = ('home', 'tap', 'phase', 'axis', 'speed', 'triggered', 'applied',
            'int_time', 'onset_time', 'int_minus_onset_ms', 'start_pos_mm',
            'onset_pos_mm', 'int_pos_mm', 'halt_pos_mm', 'thresh')


def _num(text):
    if text is None or text == '':
        return None
    return float(text)


def read_taps(path):
    """[{tap fields..., 'samples': [(t_rel_int_ms, tap_mag, over), ...]}]
    in file order."""
    taps = []
    index = {}
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            key = (row['home'], row['tap'])
            tap = index.get(key)
            if tap is None:
                tap = {k: row.get(k, '') for k in TAP_KEYS}
                for k in ('home', 'tap', 'triggered'):
                    tap[k] = int(tap[k] or 0)
                for k in TAP_KEYS[4:]:
                    if k not in ('triggered', 'applied'):
                        tap[k] = _num(tap[k])
                tap['samples'] = []
                index[key] = tap
                taps.append(tap)
            if row.get('t_rel_int_ms'):
                tap['samples'].append((float(row['t_rel_int_ms']),
                                       float(row['tap_mag']),
                                       row.get('over_thresh') == '1'))
    return taps


def _diff(a, b):
    if a is None or b is None:
        return None
    return a - b


def summarise(taps):
    """(per_tap, per_home). per_tap rows add d_onset_mm / d_int_mm (vs the
    previous tap of the same home), overrun_mm (halt - INT, signed along
    the move), peak_mag and first_over_ms (first sample over threshold,
    relative to INT)."""
    per_tap = []
    prev = {}
    for tap in taps:
        row = dict((k, tap[k]) for k in TAP_KEYS)
        p = prev.get(tap['home'])
        row['d_onset_mm'] = _diff(tap['onset_pos_mm'],
                                  p and p['onset_pos_mm'])
        row['d_int_mm'] = _diff(tap['int_pos_mm'], p and p['int_pos_mm'])
        row['overrun_mm'] = _diff(tap['halt_pos_mm'], tap['int_pos_mm'])
        mags = [m for _, m, _ in tap['samples']]
        row['peak_mag'] = max(mags) if mags else None
        over = [t for t, _, o in tap['samples'] if o]
        row['first_over_ms'] = over[0] if over else None
        row['n_samples'] = len(tap['samples'])
        per_tap.append(row)
        if tap['triggered']:
            prev[tap['home']] = tap
    per_home = []
    for home in sorted(set(t['home'] for t in per_tap)):
        rows = [t for t in per_tap if t['home'] == home]
        out = {'home': home, 'taps': len(rows),
               'missed': sum(1 for t in rows if not t['triggered'])}
        for what in ('onset', 'int'):
            vals = [t[what + '_pos_mm'] for t in rows
                    if t[what + '_pos_mm'] is not None]
            out[what + '_spread_mm'] = (max(vals) - min(vals)
                                        if vals else None)
            steps = [t['d_%s_mm' % what] for t in rows
                     if t['d_%s_mm' % what] is not None]
            out[what + '_one_way'] = one_way(steps)
        delays = [t['int_minus_onset_ms'] for t in rows
                  if t['int_minus_onset_ms'] is not None]
        out['delay_ms_min'] = min(delays) if delays else None
        out['delay_ms_max'] = max(delays) if delays else None
        per_home.append(out)
    return per_tap, per_home


def one_way(steps, eps=0.01):
    """'+' or '-' when every step past eps mm moves the same way (at least
    two of them), '' otherwise."""
    big = [s for s in steps if abs(s) > eps]
    if len(big) < 2:
        return ''
    if all(s > 0 for s in big):
        return '+'
    if all(s < 0 for s in big):
        return '-'
    return ''


def _fmt(v, f='%.3f'):
    if v is None or v == '':
        return '-'
    if isinstance(v, float):
        return f % (v,)
    return str(v)


def print_summary(per_tap, per_home, out=sys.stdout):
    cols = [('home', 'home', None), ('tap', 'tap', None),
            ('phase', 'phase', None), ('applied', 'used', None),
            ('speed', 'mm/s', '%.1f'),
            ('int_minus_onset_ms', 'INT-onset ms', '%.2f'),
            ('onset_pos_mm', 'onset mm', '%.4f'),
            ('int_pos_mm', 'INT mm', '%.4f'),
            ('d_onset_mm', 'd onset', '%+.4f'),
            ('d_int_mm', 'd INT', '%+.4f'),
            ('overrun_mm', 'overrun', '%+.4f'),
            ('peak_mag', 'peak', '%.0f'),
            ('first_over_ms', 'over@ms', '%.2f'),
            ('n_samples', 'n', None)]
    table = [[h for _, h, _ in cols]]
    for t in per_tap:
        table.append([_fmt(t[k], f or '%s') for k, _, f in cols])
    widths = [max(len(r[i]) for r in table) for i in range(len(cols))]
    for r in table:
        out.write('  '.join(c.rjust(w) for c, w in zip(r, widths)) + '\n')
    out.write('\n')
    for h in per_home:
        out.write(
            "home %d: %d taps (%d without INT); onset spread %s mm %s; INT"
            " spread %s mm %s; INT-onset %s..%s ms\n" % (
                h['home'], h['taps'], h['missed'],
                _fmt(h['onset_spread_mm'], '%.4f'),
                ('one-way ' + h['onset_one_way']) if h['onset_one_way']
                else '',
                _fmt(h['int_spread_mm'], '%.4f'),
                ('one-way ' + h['int_one_way']) if h['int_one_way'] else '',
                _fmt(h['delay_ms_min'], '%.2f'),
                _fmt(h['delay_ms_max'], '%.2f')))


def plot(taps, per_tap, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 9))
    for tap in taps:
        if not tap['samples']:
            continue
        ts = [s[0] for s in tap['samples']]
        ms = [s[1] for s in tap['samples']]
        a1.plot(ts, ms, lw=0.8, label='h%d t%d %s' % (
            tap['home'], tap['tap'], tap['phase']))
        if tap['int_minus_onset_ms'] is not None:
            a1.axvline(-tap['int_minus_onset_ms'], lw=0.4, ls=':')
    thresh = [t['thresh'] for t in taps if t['thresh'] is not None]
    if thresh:
        a1.axhline(thresh[0], color='k', ls='--', lw=0.8, label='tap_thresh')
    a1.axvline(0., color='r', lw=0.8)
    a1.set_xlabel('ms from INT')
    a1.set_ylabel('|accel| on the tap axes (mm/s^2)')
    a1.set_title('Tap signal around INT (dotted: onset)')
    if len(taps) <= 20:
        a1.legend(fontsize=6, ncol=2)
    xs = list(range(1, len(per_tap) + 1))
    for key, label in (('onset_pos_mm', 'onset'), ('int_pos_mm', 'INT')):
        a2.plot(xs, [t[key] for t in per_tap], marker='o', label=label)
    for i, t in enumerate(per_tap):
        if t['tap'] == 1:
            a2.axvline(i + 0.5, color='grey', lw=0.5)
    a2.set_xlabel('tap (grey lines: a new home)')
    a2.set_ylabel('axis position from MCU steps (mm)')
    a2.set_title('Position per tap')
    a2.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv')
    ap.add_argument('--plot', metavar='PNG')
    args = ap.parse_args(argv)
    taps = read_taps(args.csv)
    if not taps:
        sys.stderr.write('%s has no taps\n' % (args.csv,))
        return 1
    per_tap, per_home = summarise(taps)
    print_summary(per_tap, per_home)
    if args.plot:
        try:
            plot(taps, per_tap, args.plot)
        except ImportError:
            sys.stderr.write('--plot needs matplotlib\n')
            return 1
        print('plot written to %s' % (args.plot,))
    return 0


if __name__ == '__main__':
    sys.exit(main())
