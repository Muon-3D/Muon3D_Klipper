#!/usr/bin/env python3
"""Summarize always-on Y tap diagnostics: y_tap_log_summary.py klippy.log.

Verdicts are hypotheses, not rotor measurements. DRV_STATUS is sampled
after halt, not on approach; low load before contact cannot be established
from these records alone.
"""
import argparse
import json
import math

# To calibrate on M1.
CYCLE_MM = 0.8
CYCLE_WINDOW_MM = 0.1
TOLERANCE_MM = 0.2
SG_LOW = 10
CS_DROP = 2
POSITION_EPS_MM = 0.003125


def parse_line(text, prefix):
    if prefix not in text:
        return None
    text = text.split(prefix, 1)[1].strip()
    data = {}
    decoder = json.JSONDecoder()
    while text:
        key, text = text.split('=', 1)
        value, end = decoder.raw_decode(text)
        data[key] = value
        text = text[end:].lstrip()
    return data


def read_homes(lines):
    homes = {}
    for text in lines:
        for prefix, key in (('y tap attempt:', 'attempts'),
                            ('y tap summary:', 'summaries')):
            try:
                row = parse_line(text, prefix)
            except (ValueError, TypeError):
                continue
            if row is not None:
                if not isinstance(row.get('home'), int):
                    continue
                home = homes.setdefault(row['home'],
                                        {'attempts': [], 'summaries': []})
                # G-code errors can echo the short summary; prefer the full one.
                if key == 'attempts' or 'error' in row:
                    home[key].append(row)
    return homes


def near_cycle(delta):
    cycles = round(abs(delta) / CYCLE_MM)
    return (cycles >= 1
            and abs(abs(delta) - cycles * CYCLE_MM) <= CYCLE_WINDOW_MM)


def verdict(home):
    rows = home['attempts']
    errors = ' '.join(str(r.get('error', '')) for r in
                      rows + home['summaries'])
    if 'stream' in errors and 'did not stop' in errors:
        return 'sensor_race'
    previous = {}
    for row in rows:
        start, halt = row.get('start_steps', {}), row.get('halt_steps', {})
        trig = row.get('trigger_steps', {})
        if (row.get('trigger_mm') is not None and start and halt == start
                and any(trig.get(n) != start.get(n) for n in start)):
            return 'sensor_race'
        travel, follow = row.get('halt_travel_mm'), row.get('halt_follow_mm')
        if (travel is not None and follow is not None
                and 'command_start' in row and 'command_end' in row
                and abs(abs(travel) - abs(row['command_end']
                        - row['command_start'])) <= POSITION_EPS_MM
                and abs(follow) > row.get('tolerance', TOLERANCE_MM)):
            return 'sensor_race'
        delta = row.get('delta')
        drivers = row.get('drv_status', {})
        evidence = False
        for name, data in drivers.items():
            if not isinstance(data, dict):
                continue
            cs = data.get('cs_actual')
            old_cs = previous.get(name)
            evidence |= (bool(data.get('stallguard'))
                         or data.get('sg_result', math.inf) <= SG_LOW
                         or (cs is not None and old_cs is not None
                             and old_cs - cs >= CS_DROP))
            previous[name] = cs
        if delta is not None and near_cycle(delta) and evidence:
            return 'pole_slip'
    for row in rows:
        delta = row.get('delta')
        if (row.get('kind') == 'confirm' and delta is not None
                and delta < -row.get('tolerance', TOLERANCE_MM)
                and not row.get('step_anomaly', False)):
            return 'early_trigger'
    halts = [r['halt_mm'] for r in rows if r.get('halt_mm') is not None]
    if halts:
        for halt in halts[1:]:
            diff = abs(halt - halts[0])
            if diff > TOLERANCE_MM and not near_cycle(diff):
                return 'binding'
    return 'ok'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log')
    args = ap.parse_args()
    with open(args.log, errors='replace') as f:
        homes = read_homes(f)
    for number, home in homes.items():
        result = (home['summaries'][-1].get('result', 'unknown')
                  if home['summaries'] else 'incomplete')
        print('home %s result=%s' % (number, result))
        print('attempt kind classification trigger_mm halt_mm delta drivers')
        for r in home['attempts']:
            print('%s %s %s %s %s %s %s' % (
                r['attempt'], r['kind'], r['classification'],
                r.get('trigger_mm'), r.get('halt_mm'), r.get('delta'),
                json.dumps(r.get('drv_status', {}), separators=(',', ':'))))
        print('hypothesis=%s (post-halt evidence; calibrate on M1)' %
              verdict(home))


if __name__ == '__main__':
    main()
