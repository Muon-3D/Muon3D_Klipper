#!/usr/bin/env python3
# Turn a motor_tune_suite.py run into a Markdown report.
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
#   python3 motor_tune_report.py <out dir> [samples.jsonl] > report.md
#
# <out dir> holds results.jsonl from the suite; samples.jsonl is
# [motor_tune]'s register log (default <out dir>/motor_tune.jsonl, else
# /home/printer_data/logs/motor_tune.jsonl).
import collections, json, os, re, sys


def load(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def latest(results):
    """The last record per id (a resumed run appends)."""
    by = collections.OrderedDict()
    for r in results:
        if r.get('id'):
            by[r['id']] = r
    return list(by.values())


def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def fmt(v, f='%.0f'):
    return '-' if v is None else f % v


def section_noise(res, w):
    w("## Homing repeat (noise floor)\n")
    for tag in ('noise', 'closing_noise'):
        rows = [r for r in res if r['id'].startswith(tag + '_')
                and 'motor_a_mm' in r]
        if not rows:
            continue
        vals = [abs(r[k]) for r in rows for k in ('motor_a_mm', 'motor_b_mm')]
        w("- %s: %d runs, |shift| max %.3f mm, mean %.3f mm\n"
          % (tag, len(rows), max(vals), mean(vals)))
    w("\nA STEPLOSS run passes when both motors' shift is within"
      " max(0.1 mm, 1.5x the max above). A full step is 0.2 mm, a pole slip 0.8 mm.\n\n")


SHAPER_RE = re.compile(r"Fitted shaper '(\w+)' frequency = ([\d.]+) Hz"
                       r" \(vibrations = ([\d.]+)%, smoothing ~= ([\d.]+)\)")
ACCEL_RE = re.compile(r"with '(\w+)', suggested max_accel <= (\d+)")
RECO_RE = re.compile(r"Recommended shaper_type_(\w) = (\w+),"
                     r" shaper_freq_\w = ([\d.]+)")


def section_resonance(res, w):
    w("## Resonance (SHAPER_CALIBRATE, 20-130 Hz)\n\n"
      "max accel is Klipper's suggestion to keep the shaper's smoothing"
      " down (print quality), not a step-loss limit.\n")
    for r in res:
        if r.get('kind') != 'resonance':
            continue
        w("\n### %s\n" % (r['id'],))
        if r.get('error'):
            w("Error: %s\n" % (r['error'],))
            continue
        # The console window can start inside the other axis's output:
        # take only the lines after this axis's own command.
        lines = r.get('console', [])
        cmd = 'SHAPER_CALIBRATE AXIS=%s' % (r.get('axis', '').upper(),)
        starts = [i for i, l in enumerate(lines) if l.startswith(cmd)]
        if starts:
            lines = lines[starts[-1]:]
        fitted, accel = [], {}
        for line in lines:
            m = SHAPER_RE.search(line)
            if m:
                fitted.append(m.groups())
            a = ACCEL_RE.search(line)
            if a:
                accel[a.group(1)] = a.group(2)
        w("| shaper | freq Hz | vibrations % | smoothing | max accel |\n"
          "|---|---|---|---|---|\n")
        for name, freq, vib, smooth in fitted:
            w("| %s | %s | %s | %s | %s |\n" % (name, freq, vib, smooth,
                                                accel.get(name, '-')))
        for m in RECO_RE.finditer('\n'.join(lines)):
            w("\nRecommended: %s at %s Hz\n" % (m.group(2), m.group(3)))
        if r.get('csv'):
            w("\nData: `%s`\n" % (r['csv'],))
    w("\n")


def section_stealth(samples, res, w):
    w("## StealthChop headroom (PWM_SCALE_SUM vs speed)\n\n"
      "StealthChop holds the current only while PWM_SCALE_SUM (0-1023 on the TMC2241) has room;"
      " where it flattens there is no voltage left. Switch to SpreadCycle"
      " well below that (about 70-80 %).\n\n")
    rows = [s for s in samples if s.get('label') == 'stealth_sweep']
    table = collections.defaultdict(lambda: collections.defaultdict(list))
    for s in rows:
        key = (s.get('current'), s['axis'], s['stepper'])
        table[key][s['speed']].append(s)
    speeds = sorted({s['speed'] for s in rows})
    if not speeds:
        w("No samples.\n\n")
        return
    w("| current A | axis | motor | " + " | ".join(
        "%d" % v for v in speeds) + " |\n")
    w("|---|---|---|" + "---|" * len(speeds) + "\n")
    for key in sorted(table, key=lambda k: (k[0] or 0, k[1], k[2])):
        cells = []
        for v in speeds:
            m = mean([x.get('pwm_scale_sum') for x in table[key].get(v, [])])
            cells.append(fmt(m))
        w("| %s | %s | %s | %s |\n" % (key[0], key[1].upper(), key[2],
                                       " | ".join(cells)))
    sat = [r for r in res if r.get('kind') == 'stealth']
    if sat:
        w("\nFlat (saturated) from: " + "; ".join(
            "%.1f A %s" % (r['current'], r.get('saturated_at') or 'none')
            for r in sat) + "\n")
    w("\n### StealthChop auto-tune (PWM_AUTO at standstill after tuning)\n\n"
      "| label | motor | pwm_ofs_auto | pwm_grad_auto |\n|---|---|---|---|\n")
    for s in samples:
        if s.get('kind') == 'read' and str(s.get('label', '')).startswith(
                'autotune'):
            w("| %s | %s | %s | %s |\n" % (s['label'], s['stepper'],
                                          s.get('pwm_ofs_auto'),
                                          s.get('pwm_grad_auto')))
    w("\nSave the converged values as driver_PWM_OFS / driver_PWM_GRAD.\n"
      "\n### StallGuard4 in StealthChop (load margin, lower is more load)\n\n")
    w("| current A | axis | motor | " + " | ".join(
        "%d" % v for v in speeds) + " |\n")
    w("|---|---|---|" + "---|" * len(speeds) + "\n")
    for key in sorted(table, key=lambda k: (k[0] or 0, k[1], k[2])):
        cells = [fmt(mean([x.get('sg4_result')
                           for x in table[key].get(v, [])]))
                 for v in speeds]
        w("| %s | %s | %s | %s |\n" % (key[0], key[1].upper(), key[2],
                                       " | ".join(cells)))
    w("\n")


def section_spread(samples, w):
    w("## SpreadCycle load (StallGuard2 SG_RESULT vs speed)\n\n"
      "Lower is more load; 0 is a stall. A falling trend with speed is the"
      " torque margin shrinking.\n\n")
    rows = [s for s in samples if s.get('label') == 'spread_sweep']
    speeds = sorted({s['speed'] for s in rows})
    if not speeds:
        w("No samples.\n\n")
        return
    table = collections.defaultdict(lambda: collections.defaultdict(list))
    for s in rows:
        table[(s.get('current'), s['axis'], s['stepper'])][s['speed']].append(
            s)
    w("| current A | axis | motor | " + " | ".join(
        "%d" % v for v in speeds) + " |\n")
    w("|---|---|---|" + "---|" * len(speeds) + "\n")
    for key in sorted(table, key=lambda k: (k[0] or 0, k[1], k[2])):
        cells = [fmt(mean([x.get('sg_result')
                           for x in table[key].get(v, [])]))
                 for v in speeds]
        w("| %s | %s | %s | %s |\n" % (key[0], key[1].upper(), key[2],
                                       " | ".join(cells)))
    flags = collections.Counter()
    for s in rows:
        for f in ('ot', 'otpw', 's2ga', 's2gb', 'ola', 'olb', 's2vsa',
                  's2vsb'):
            if s.get(f):
                flags[f] += 1
    if flags:
        w("\nDriver flags seen: %s\n" % (dict(flags),))
    w("\n")


def section_ladders(res, w):
    w("## Step-loss ladders (highest accel with no lost steps)\n\n"
      "Each ladder raises ACCEL (5k..30k) at a fixed SPEED until a fail"
      " that repeats; the best pass is re-run with 4x the moves"
      " (confirmed).\n\n")
    lad = [r for r in res if r.get('kind') == 'ladder'
           and r.get('ladder_kind') == 'accel']
    if not lad:
        w("No ladders.\n\n")
        return
    speeds = sorted({r['speed'] for r in lad})
    keys = sorted({(r['mode'], r['current']) for r in lad},
                  key=lambda k: (k[1], k[0]))
    w("| mode | current A | " + " | ".join("%d mm/s" % v for v in speeds)
      + " |\n|---|---|" + "---|" * len(speeds) + "\n")
    for mode, c in keys:
        cells = []
        for v in speeds:
            r = [x for x in lad if x['mode'] == mode and x['current'] == c
                 and x['speed'] == v]
            if not r:
                cells.append('-')
                continue
            r = r[-1]
            best = r.get('best')
            cell = 'none' if not best else '%dk' % (best[1] // 1000)
            if r.get('confirmed') is False:
                cell += ' (confirm failed)'
            cells.append(cell)
        w("| %s | %s | %s |\n" % (mode, c, " | ".join(cells)))
    fails = [r for r in res if r.get('kind') == 'steploss'
             and r.get('passed') is False and 'motor_a_mm' in r]
    if fails:
        w("\nFails (shift A / B mm): " + "; ".join(
            "%s %+.2f/%+.2f" % (r['id'], r['motor_a_mm'], r['motor_b_mm'])
            for r in fails[:40]) + "\n")
    errs = [r for r in res if r.get('kind') == 'steploss' and r.get('error')]
    if errs:
        w("\nErrors: " + "; ".join("%s: %s" % (r['id'], r['error'])
                                   for r in errs[:20]) + "\n")
    w("\n")


def section_overspeed(res, w):
    w("## Top speed (SpreadCycle, 20k mm/s^2)\n\n")
    lad = [r for r in res if r.get('kind') == 'ladder'
           and r.get('ladder_kind') == 'speed']
    for r in lad:
        w("- %.1f A: best %s%s\n" % (
            r['current'], r.get('best'),
            '' if r.get('confirmed') is not False else ' (confirm failed)'))
    if not lad:
        w("No speed ladders.\n")
    w("\n")


def section_thermal(res, w):
    w("## Driver temperatures under sustained stress (600 mm/s, 20k)\n\n")
    for r in res:
        if r.get('kind') != 'thermal':
            continue
        s = r.get('series', [])
        if not s:
            continue
        peak = {k: max(x.get(k) or 0 for x in s) for k in s[0]
                if k != 't'}
        w("- %.1f A over %.0f min: peak %s\n" % (
            r['current'], (s[-1]['t'] - s[0]['t']) / 60.,
            ", ".join("%s %.0f" % (k, v) for k, v in sorted(peak.items()))))
    w("\n")


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "usage: motor_tune_report.py <out dir>")
        return 2
    out = sys.argv[1]
    spath = sys.argv[2] if len(sys.argv) > 2 else None
    if spath is None:
        cand = os.path.join(out, 'motor_tune.jsonl')
        spath = cand if os.path.exists(cand) else \
            '/home/printer_data/logs/motor_tune.jsonl'
    res = latest(load(os.path.join(out, 'results.jsonl')))
    samples = load(spath)
    parts = []
    w = parts.append
    w("# M1 X/Y motor tuning report\n\n")
    pre = [r for r in res if r.get('kind') == 'preflight']
    if pre and pre[-1].get('config'):
        c = pre[-1]['config']
        w("Config at the start: max_velocity %s, max_accel %s; X/Y"
          " run_current %s A, stealthchop_threshold %s.\n\n" % (
              (c.get('toolhead') or {}).get('max_velocity'),
              (c.get('toolhead') or {}).get('max_accel'),
              (c.get('tmc2241 stepper_x') or {}).get('run_current'),
              (c.get('tmc2241 stepper_x') or {}).get(
                  'stealthchop_threshold')))
    section_noise(res, w)
    section_resonance(res, w)
    section_stealth(samples, res, w)
    section_spread(samples, w)
    section_ladders(res, w)
    section_overspeed(res, w)
    section_thermal(res, w)
    sys.stdout.write(''.join(parts))
    return 0


if __name__ == '__main__':
    sys.exit(main())
