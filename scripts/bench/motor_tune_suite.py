#!/usr/bin/env python3
# Overnight X/Y motor and kinematics tuning suite for an M1 on the bench.
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Runs ON the printer (python3, standard library only) and talks to Klipper
# through Moonraker on localhost, so a dropped SSH session does not stop it:
#
#   nohup setsid python3 motor_tune_suite.py > /dev/null 2>&1 &
#
# Needs the bench config: debug.cfg included (STEPLOSS_TEST, [motor_tune]).
# The bed must be clear: the suite homes Z once (a tap at X100 Y60) and then
# keeps the nozzle at Z60, except the resonance sweeps (Z20 at X100 Y100).
# Heaters stay off throughout.
#
# Phases (each test is skipped if results.jsonl already has it, so a stopped
# run resumes where it left off):
#   preflight  heaters off, G28, Z60, driver registers and config snapshot
#   noise      STEPLOSS_TEST with no moves, x5: the homing repeat, which is
#              the floor any shift has to clear
#   resonance  SHAPER_CALIBRATE X and Y over 20-130 Hz
#   stealth    per current 0.6/0.8/1.0 A: StealthChop's auto-tune (PWM_AUTO)
#              and its headroom (PWM_SCALE_SUM) from 50 to 800 mm/s
#   spread     per current: SpreadCycle load (StallGuard) from 100 to 1000 mm/s
#   ladders    STEPLOSS_TEST accel ladders (5k..30k) per driver mode
#              (SpreadCycle; StealthChop below 100/150/200/300 mm/s), current
#              (0.8, 1.0 A; 0.6 A for two modes) and speed (600, 800 mm/s),
#              stopping at the first confirmed fail; the best pass confirmed
#              with 4x the moves
#
# About 5-6 hours if every ladder passes to 30k; fails end ladders early.
#   overspeed  SpreadCycle speed ladder 800..1200 mm/s at 20k
#   thermal    10-minute stress at 1.0 and 0.8 A with driver temperatures
#   closing    noise and resonance again, for drift over the night
#
# Stop it: touch <out>/STOP (it stops after the current test).
# Results: <out>/results.jsonl, <out>/run.log; register samples in
# /home/printer_data/logs/motor_tune.jsonl. scripts/bench/motor_tune_report.py
# turns them into a report.
import argparse, glob, json, os, shutil, sys, threading, time
import urllib.error, urllib.parse, urllib.request

MOONRAKER = 'http://127.0.0.1:7125'
SAMPLES_LOG = '/home/printer_data/logs/motor_tune.jsonl'
TEMP_OBJECTS = ('temperature_sensor driver_x', 'temperature_sensor driver_y',
                'temperature_sensor driver_z', 'temperature_fan Motor_Drivers')
MODES = (('spread', 0), ('stealth300', 300), ('stealth150', 150),
         ('stealth100', 100), ('stealth200', 200))
ACCELS = (5000, 10000, 15000, 20000, 25000, 30000)
HOT_C = 95.          # wait for the drivers to cool above this
COOL_C = 75.
BIG_SLIP_MM = 5.     # a slip this big is a crash risk: re-home, count it
MAX_BIG_SLIPS = 3


class Stop(Exception):
    pass


class GcodeError(Exception):
    pass


class Suite:
    def __init__(self, out, dry_run=False, quick=False):
        self.out = out
        self.dry = dry_run
        self.quick = quick
        os.makedirs(out, exist_ok=True)
        self.results_path = os.path.join(out, 'results.jsonl')
        self.log_path = os.path.join(out, 'run.log')
        self.done = set()
        if os.path.exists(self.results_path):
            with open(self.results_path) as f:
                for line in f:
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        continue
                    if rec.get('id') and rec.get('final', True):
                        self.done.add(rec['id'])
        self.noise_mm = None
        self.big_slips = 0
        self.t_start = time.time()

    # --- Moonraker -------------------------------------------------------
    def _req(self, path, data=None, timeout=30.):
        url = MOONRAKER + path
        req = urllib.request.Request(url, data=data,
                                     method='POST' if data is not None
                                     else 'GET')
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors='replace')
            try:
                msg = json.loads(body)['error']['message']
            except Exception:
                msg = body[:300]
            raise GcodeError(msg)

    def gcode(self, script, timeout=3600.):
        self.log("gcode: %s" % (script.replace('\n', ' / '),))
        if self.dry:
            return
        q = urllib.parse.urlencode({'script': script})
        self._req('/printer/gcode/script?' + q, data=b'', timeout=timeout)

    def query(self, objects):
        if self.dry:
            return {}
        q = '&'.join(urllib.parse.quote(o) for o in objects)
        return self._req('/printer/objects/query?' + q)['result']['status']

    def state(self):
        if self.dry:
            return 'ready'
        return self._req('/printer/info')['result']['state']

    def console_since(self, t0):
        if self.dry:
            return []
        res = self._req('/server/gcode_store?count=1000')['result']
        return [g['message'] for g in res['gcode_store'] if g['time'] >= t0]

    # --- Bookkeeping -----------------------------------------------------
    def log(self, msg):
        line = "%s +%5.1f min  %s" % (time.strftime('%H:%M:%S'),
                                      (time.time() - self.t_start) / 60., msg)
        print(line, flush=True)
        with open(self.log_path, 'a') as f:
            f.write(line + '\n')

    def record(self, rec):
        rec.setdefault('ts', round(time.time(), 1))
        with open(self.results_path, 'a') as f:
            f.write(json.dumps(rec, sort_keys=True) + '\n')
        if rec.get('id') and rec.get('final', True):
            self.done.add(rec['id'])

    def temps(self):
        st = self.query(TEMP_OBJECTS)
        out = {}
        for name, s in st.items():
            key = name.split()[-1]
            out[key] = s.get('temperature')
            if 'speed' in s:
                out[key + '_fan'] = s.get('speed')
        return out

    def check(self):
        """Between tests: stop file, Klipper state, driver heat."""
        if os.path.exists(os.path.join(self.out, 'STOP')):
            raise Stop("STOP file")
        st = self.state()
        if st != 'ready':
            raise Stop("Klipper is %s" % (st,))
        if self.dry:
            return
        ps = self.query(['print_stats'])['print_stats']['state']
        if ps == 'printing':
            raise Stop("a print is running")
        t = self.temps()
        hot = max(v for k, v in t.items()
                  if k.startswith('driver') and not k.endswith('_fan')
                  and v is not None)
        if hot > HOT_C:
            self.log("drivers at %.0f C: cooling to %.0f C" % (hot, COOL_C))
            deadline = time.time() + 1200
            while hot > COOL_C and time.time() < deadline:
                time.sleep(15)
                if os.path.exists(os.path.join(self.out, 'STOP')):
                    raise Stop("STOP file")
                t = self.temps()
                hot = max(v for k, v in t.items()
                          if k.startswith('driver') and not k.endswith('_fan')
                          and v is not None)

    # --- Building blocks -------------------------------------------------
    def home_xy(self):
        self.gcode("G28 X Y")

    def set_mode(self, stealth, current):
        """stealth: 0 SpreadCycle when moving, >0 StealthChop below that
        motor speed (mm/s). current: X/Y run current (A)."""
        lines = []
        for s in ('stepper_x', 'stepper_y'):
            lines.append("SET_TMC_FIELD STEPPER=%s FIELD=tpwmthrs VELOCITY=%s"
                         % (s, stealth))
            lines.append("SET_TMC_FIELD STEPPER=%s FIELD=en_pwm_mode VALUE=1"
                         % (s,))
            lines.append("SET_TMC_CURRENT STEPPER=%s CURRENT=%s"
                         % (s, current))
        self.gcode("M400\n" + "\n".join(lines))

    def restore_mode(self):
        self.gcode("_STEPLOSS_MODE RESTORE=1\n"
                   "SET_TMC_FIELD STEPPER=stepper_x FIELD=tcoolthrs VALUE=0\n"
                   "SET_TMC_FIELD STEPPER=stepper_y FIELD=tcoolthrs VALUE=0")

    def steploss(self, tid, speed, accel, count, stealth=-1, current=0,
                 overspeed=False, extra=None):
        """One STEPLOSS_TEST; returns (passed, record)."""
        if tid in self.done:
            rec = self.lookup(tid)
            return rec.get('passed', False), rec
        self.check()
        t0 = time.time()
        temps_before = self.temps()
        script = ("STEPLOSS_TEST SPEED=%d ACCEL=%d COUNT=%d STEALTH=%s"
                  " CURRENT=%s" % (speed, accel, count, stealth, current))
        if overspeed:
            script += " OVERSPEED=1"
        rec = {'id': tid, 'kind': 'steploss', 'speed': speed,
               'accel': accel, 'count': count, 'stealth': stealth,
               'current': current, 'temps_before': temps_before}
        rec.update(extra or {})
        try:
            self.gcode(script)
        except GcodeError as e:
            rec.update(error=str(e), passed=False, final=True,
                       seconds=round(time.time() - t0, 1))
            self.log("%s: ERROR %s" % (tid, e))
            self.record(rec)
            self.recover()
            return False, rec
        if self.dry:
            rec.update(passed=True, final=True)
            self.record(rec)
            return True, rec
        m = self.query(['motor_tune'])['motor_tune']['measures']
        bef, aft = m.get('before') or {}, m.get('after') or {}
        if None in (bef.get('x'), bef.get('y'), aft.get('x'), aft.get('y')):
            rec.update(error="no tap in a measuring home: before %s,"
                       " after %s" % (bef, aft), passed=False, final=True,
                       seconds=round(time.time() - t0, 1))
            self.log("%s: ERROR %s" % (tid, rec['error']))
            self.record(rec)
            return False, rec
        dx, dy = aft['x'] - bef['x'], aft['y'] - bef['y']
        da, db = dx + dy, dx - dy
        # The pass limit: 0.1 mm, or above the taps' own repeat (the noise
        # runs) with margin. A pole slip is 0.8 mm.
        limit = max(0.1, 1.5 * (self.noise_mm or 0.))
        passed = max(abs(da), abs(db)) <= limit
        rec.update(motor_a_mm=round(da, 4), motor_b_mm=round(db, 4),
                   x_mm=round(dx, 4), y_mm=round(dy, 4),
                   before=bef, after=aft, limit_mm=round(limit, 4),
                   passed=passed, final=True,
                   seconds=round(time.time() - t0, 1),
                   temps_after=self.temps())
        self.record(rec)
        self.log("%s: %s A %+.3f B %+.3f mm (%.0f s)" % (
            tid, 'PASS' if passed else 'FAIL', da, db, rec['seconds']))
        if max(abs(da), abs(db)) > BIG_SLIP_MM:
            self.big_slips += 1
            self.log("big slip (%d so far)" % (self.big_slips,))
            if self.big_slips >= MAX_BIG_SLIPS:
                raise Stop("%d slips over %.0f mm" % (self.big_slips,
                                                      BIG_SLIP_MM))
        return passed, rec

    def lookup(self, tid):
        with open(self.results_path) as f:
            last = {}
            for line in f:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if rec.get('id') == tid:
                    last = rec
        return last

    def recover(self):
        """After a refused command: Klipper must be ready, then re-home."""
        st = self.state()
        if st != 'ready':
            raise Stop("Klipper is %s after an error" % (st,))
        try:
            self.restore_mode()
            self.home_xy()
        except GcodeError as e:
            raise Stop("cannot re-home after an error: %s" % (e,))

    # --- Phases ----------------------------------------------------------
    def phase_preflight(self):
        tid = 'preflight'
        self.check()
        self.gcode("TURN_OFF_HEATERS\nM107")
        # Straight after a Klipper restart HOMING_GATE can still see the
        # toolhead and the Z column as missing and put up a prompt instead
        # of homing, so G28 returns with nothing homed. Retry until Z homes.
        for attempt in range(6):
            self.gcode("G28")
            if self.dry or 'z' in self.query(['toolhead'])['toolhead'][
                    'homed_axes']:
                break
            self.log("G28 did not home Z (homing gate?), retrying in 10 s")
            time.sleep(10)
        else:
            raise Stop("G28 does not home Z")
        self.gcode("G90\nG1 Z60 F600\nM400")
        self.gcode("MOTOR_TUNE_READ LABEL=preflight")
        snap = {}
        if not self.dry:
            st = self.query(['configfile', 'toolhead'])
            s = st['configfile']['settings']
            snap = {k: s.get(k) for k in (
                'printer', 'stepper_x', 'stepper_y', 'tmc2241 stepper_x',
                'tmc2241 stepper_y', 'input_shaper')}
            snap['toolhead'] = {k: st['toolhead'].get(k) for k in (
                'max_velocity', 'max_accel', 'minimum_cruise_ratio',
                'square_corner_velocity')}
        self.record({'id': tid, 'kind': 'preflight', 'config': snap,
                     'temps': self.temps() if not self.dry else {}})

    def phase_noise(self, tag='noise', n=5):
        shifts = []
        for i in range(n):
            ok, rec = self.steploss('%s_%d' % (tag, i), 300, 5000, 0)
            if 'motor_a_mm' in rec:
                shifts += [abs(rec['motor_a_mm']), abs(rec['motor_b_mm'])]
        if shifts and tag == 'noise':
            self.noise_mm = max(shifts)
            self.log("homing repeat: max %.3f mm, mean %.3f mm" % (
                max(shifts), sum(shifts) / len(shifts)))

    def phase_resonance(self, tag='resonance'):
        for axis in ('x', 'y'):
            tid = '%s_%s' % (tag, axis)
            if tid in self.done:
                continue
            self.check()
            t0 = time.time()
            try:
                self.gcode("SHAPER_CALIBRATE AXIS=%s FREQ_START=20"
                           " FREQ_END=130" % (axis.upper(),))
            except GcodeError as e:
                self.record({'id': tid, 'kind': 'resonance', 'axis': axis,
                             'error': str(e), 'final': True})
                self.recover()
                continue
            lines = [m for m in self.console_since(t0 - 1)
                     if 'shaper' in m.lower() or 'max_accel' in m]
            files = sorted(glob.glob('/tmp/calibration_data_%s_*.csv'
                                     % (axis,)), key=os.path.getmtime)
            saved = None
            if files and os.path.getmtime(files[-1]) >= t0 - 5:
                os.makedirs(os.path.join(self.out, 'resonance'),
                            exist_ok=True)
                saved = os.path.join(self.out, 'resonance', '%s_%s'
                                     % (tag, os.path.basename(files[-1])))
                shutil.copy(files[-1], saved)
            self.record({'id': tid, 'kind': 'resonance', 'axis': axis,
                         'console': lines, 'csv': saved, 'final': True,
                         'seconds': round(time.time() - t0, 1)})
            self.log("%s done (%.0f s)" % (tid, time.time() - t0))
        if not self.dry:
            self.gcode("G90\nG1 Z60 F600\nM400")

    def _last_samples(self, label, since):
        out = []
        if self.dry or not os.path.exists(SAMPLES_LOG):
            return out
        with open(SAMPLES_LOG) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get('label') == label and r.get('ts', 0) >= since:
                    out.append(r)
        return out

    def phase_stealth(self):
        currents = (0.8,) if self.quick else (0.6, 0.8, 1.0)
        speeds = (50, 150, 300) if self.quick else range(50, 801, 50)
        for c in currents:
            tid = 'stealth_c%.1f' % (c,)
            if tid in self.done:
                continue
            self.check()
            t0 = time.time()
            self.home_xy()
            self.set_mode(2000, c)      # StealthChop at every speed tested
            tag = json.dumps({'current': c, 'mode': 'stealth_all'})
            # AT#2 wants steady motion at moderate speed, many passes.
            for ax in ('X', 'Y'):
                self.gcode("MOTOR_TUNE_SAMPLE AXIS=%s SPEED=120 ACCEL=3000"
                           " PASSES=%d SAMPLES=1 LABEL=autotune TAG='%s'"
                           % (ax, 4 if self.quick else 16, tag))
            self.gcode("MOTOR_TUNE_READ LABEL=autotune_c%.1f TAG='%s'"
                       % (c, tag))
            saturated = {}
            for ax in ('X', 'Y'):
                prev = []
                for v in speeds:
                    label = 'stealth_sweep'
                    since = time.time() - 1
                    try:
                        self.gcode("MOTOR_TUNE_SAMPLE AXIS=%s SPEED=%d"
                                   " ACCEL=10000 PASSES=2 SAMPLES=2"
                                   " LABEL=%s TAG='%s'" % (ax, v, label, tag))
                    except GcodeError as e:
                        self.log("%s %s %d: %s" % (tid, ax, v, e))
                        self.recover()
                        self.set_mode(2000, c)
                        break
                    recs = self._last_samples(label, since)
                    vals = [r['pwm_scale_sum'] for r in recs
                            if 'pwm_scale_sum' in r]
                    if vals:
                        prev.append(sum(vals) / len(vals))
                    # Plateau: no rise over the last two speeds above
                    # 300 mm/s means no voltage left; beyond that it only
                    # risks a stall.
                    if (len(prev) >= 3 and v >= 300
                            and prev[-1] <= prev[-3] * 1.01):
                        saturated[ax] = v
                        self.log("%s %s: PWM_SCALE_SUM flat at %.0f from"
                                 " %d mm/s" % (tid, ax, prev[-1], v))
                        break
                self.home_xy()
                self.set_mode(2000, c)
            self.restore_mode()
            self.record({'id': tid, 'kind': 'stealth', 'current': c,
                         'saturated_at': saturated, 'final': True,
                         'seconds': round(time.time() - t0, 1)})

    def phase_spread(self):
        currents = (0.8,) if self.quick else (0.6, 0.8, 1.0)
        speeds = (100, 400) if self.quick else (100, 200, 300, 400, 500,
                                                600, 700, 800, 900, 1000)
        for c in currents:
            tid = 'spread_c%.1f' % (c,)
            if tid in self.done:
                continue
            self.check()
            t0 = time.time()
            self.home_xy()
            self.set_mode(0, c)
            # StallGuard2 only reads above TCOOLTHRS; CoolStep stays off
            # (semin 0).
            self.gcode("SET_TMC_FIELD STEPPER=stepper_x FIELD=tcoolthrs"
                       " VELOCITY=1\nSET_TMC_FIELD STEPPER=stepper_y"
                       " FIELD=tcoolthrs VELOCITY=1")
            tag = json.dumps({'current': c, 'mode': 'spread'})
            for ax in ('X', 'Y'):
                for v in speeds:
                    try:
                        self.gcode("MOTOR_TUNE_SAMPLE AXIS=%s SPEED=%d"
                                   " ACCEL=10000 PASSES=2 SAMPLES=2"
                                   " LABEL=spread_sweep TAG='%s'"
                                   % (ax, v, tag))
                    except GcodeError as e:
                        self.log("%s %s %d: %s" % (tid, ax, v, e))
                        self.recover()
                        self.set_mode(0, c)
                        break
            self.restore_mode()
            self.home_xy()
            self.record({'id': tid, 'kind': 'spread', 'current': c,
                         'final': True,
                         'seconds': round(time.time() - t0, 1)})

    def ladder(self, mode, stealth, current, speed, accels=ACCELS,
               overspeed_speeds=None):
        """Accel ladder (or a speed ladder when overspeed_speeds is given):
        up until a fail that repeats, then confirm the best pass with 4x
        the moves."""
        count = 4 if self.quick else 10
        best = None
        steps = ([(s, accels[0]) for s in overspeed_speeds]
                 if overspeed_speeds else [(speed, a) for a in accels])
        kind = 'speed' if overspeed_speeds else 'accel'
        base = 'ladder_%s_c%.1f_v%d' % (mode, current, speed)
        if overspeed_speeds:
            base = 'overspeed_%s_c%.1f_a%d' % (mode, current, accels[0])
        for v, a in steps:
            tid = '%s_%s%d' % (base, 'v' if overspeed_speeds else 'a',
                               v if overspeed_speeds else a)
            extra = {'mode': mode, 'ladder': base}
            ok, _ = self.steploss(tid, v, a, count, stealth, current,
                                  overspeed=bool(overspeed_speeds),
                                  extra=extra)
            if not ok:
                ok, _ = self.steploss(tid + '_retry', v, a, count, stealth,
                                      current,
                                      overspeed=bool(overspeed_speeds),
                                      extra=extra)
                if not ok:
                    break
            best = (v, a)
        summary = {'id': base, 'kind': 'ladder', 'ladder_kind': kind,
                   'mode': mode, 'stealth': stealth, 'current': current,
                   'speed': speed, 'best': best, 'final': True}
        if best and not self.quick:
            ok, _ = self.steploss(base + '_confirm', best[0], best[1],
                                  count * 4, stealth, current,
                                  overspeed=bool(overspeed_speeds),
                                  extra={'mode': mode, 'ladder': base})
            summary['confirmed'] = ok
        if base not in self.done:
            self.record(summary)
        self.log("%s: best %s" % (base, best))

    def phase_ladders(self):
        if self.quick:
            plan = [('spread', 0, 0.8, 600), ('stealth300', 300, 0.8, 600)]
            for mode, st, c, v in plan:
                self.ladder(mode, st, c, v, accels=(5000, 10000))
            return
        plan = []
        for c in (0.8, 1.0):
            for mode, st in MODES:
                for v in (600, 800):
                    plan.append((mode, st, c, v))
        for mode, st in (('spread', 0), ('stealth150', 150)):
            for v in (600, 800):
                plan.append((mode, st, 0.6, v))
        for mode, st, c, v in plan:
            self.ladder(mode, st, c, v)

    def phase_overspeed(self):
        speeds = (800, 900) if self.quick else (800, 900, 1000, 1100, 1200)
        for c in ((0.8,) if self.quick else (1.0, 0.8)):
            self.ladder('spread', 0, c, 0, accels=(20000,),
                        overspeed_speeds=speeds)

    def phase_thermal(self):
        for c in ((0.8,) if self.quick else (1.0, 0.8)):
            tid = 'thermal_c%.1f' % (c,)
            if tid in self.done:
                continue
            self.check()
            stop = threading.Event()
            series = []

            def poll():
                while not stop.wait(10.):
                    try:
                        series.append(dict(t=round(time.time(), 1),
                                           **self.temps()))
                    except Exception:
                        pass
            th = threading.Thread(target=poll, daemon=True)
            th.start()
            try:
                count = 10 if self.quick else 120
                self.steploss(tid + '_run', 600, 20000, count, 0, c,
                              extra={'mode': 'spread', 'thermal': True})
            finally:
                stop.set()
                th.join()
            self.record({'id': tid, 'kind': 'thermal', 'current': c,
                         'series': series, 'final': True})
            peak = max((max(v for k, v in s.items() if k.startswith('driver')
                            and not k.endswith('_fan') and v is not None)
                        for s in series), default=None)
            self.log("%s: peak driver %s C" % (tid, peak))

    def phase_closing(self):
        self.phase_noise('closing_noise', 3)
        self.phase_resonance('closing_resonance')

    PHASES = ('preflight', 'noise', 'resonance', 'stealth', 'spread',
              'ladders', 'overspeed', 'thermal', 'closing')

    def run(self, phases):
        self.log("motor tune suite: phases %s%s%s" % (
            ",".join(phases), " (dry run)" if self.dry else "",
            " (quick)" if self.quick else ""))
        # The noise floor sets the pass limit; reuse it on a resumed run.
        for tid in ('noise_%d' % i for i in range(5)):
            if tid in self.done:
                rec = self.lookup(tid)
                if 'motor_a_mm' in rec:
                    self.noise_mm = max(self.noise_mm or 0.,
                                        abs(rec['motor_a_mm']),
                                        abs(rec['motor_b_mm']))
        try:
            for p in phases:
                self.log("=== phase %s" % (p,))
                getattr(self, 'phase_' + p)()
        except Stop as e:
            self.log("STOPPED: %s" % (e,))
            try:
                if self.state() == 'ready':
                    self.restore_mode()
            except Exception:
                pass
            return 1
        finally:
            if not self.dry:
                try:
                    self.restore_mode()
                    self.gcode("M84")
                except Exception:
                    pass
        self.log("suite finished in %.1f h" % (
            (time.time() - self.t_start) / 3600.,))
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/home/printer_admin/motor_tune')
    ap.add_argument('--phases', default=','.join(Suite.PHASES))
    ap.add_argument('--quick', action='store_true',
                    help='a few minutes of each phase, for a smoke test')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the plan, touch nothing')
    args = ap.parse_args()
    phases = [p.strip() for p in args.phases.split(',') if p.strip()]
    bad = [p for p in phases if p not in Suite.PHASES]
    if bad:
        ap.error("unknown phase(s): %s" % (", ".join(bad),))
    return Suite(args.out, dry_run=args.dry_run, quick=args.quick).run(phases)


if __name__ == '__main__':
    sys.exit(main())
