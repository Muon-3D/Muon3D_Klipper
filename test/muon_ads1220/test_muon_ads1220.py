"""Offline tests for the Muon B1 shared load cell / hotend ADS1220.

No hardware and no klippy reactor. Covers:

* the mcu's no-DRDY sampling logic (src/sensor_muon_ads1220.h), compiled
  on the host with sim_sampler.c against a modelled chip
* hotend type detection from PT1000-mode readings
* the mcu-side temperature range check limits for both hotend types
* the heater hold that keeps the heater off through a probe tap and
  refuses heating until the hotend is known
* the detection state machine end to end, through to the applied profile

    python3 test/muon_ads1220/test_muon_ads1220.py
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
KLIPPY = os.path.join(HERE, '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import heaters, muon_ads1220 as m  # noqa: E402

R_REF = 4700.
REF_NOM = 100e-6 * R_REF / (3.3 / 4.) * (1 << 23)


######################################################################
# Fakes
######################################################################

class CommandError(Exception):
    pass


class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.now = 100.
        self.timers = {}
    def monotonic(self):
        return self.now
    def register_timer(self, callback, waketime=NEVER):
        self.timers[callback] = waketime
        return callback
    def update_timer(self, timer, waketime):
        self.timers[timer] = waketime
    def register_async_callback(self, cb):
        cb(self.now)


class FakeMcu:
    def estimated_print_time(self, eventtime):
        # print_time tracks monotonic one-for-one in these tests
        return eventtime


class FakePwm:
    def __init__(self):
        self.events = []
    def set_pwm(self, print_time, value):
        if self.events:
            assert print_time >= self.events[-1][0], "pwm scheduled backwards"
        self.events.append((print_time, value))


class FakeGcode:
    def __init__(self):
        self.messages = []
    def register_mux_command(self, *args, **kw):
        pass
    def respond_info(self, msg):
        self.messages.append(msg)


class FakeHeatsink:
    def __init__(self, temp):
        self.temp = temp
    def get_temp(self, eventtime):
        return self.temp, 0.


class FakePrinter:
    command_error = CommandError
    def __init__(self, chip, heater=None):
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.chip = chip
        self.objects = {'gcode': self.gcode}
        self.heater = heater
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        if name == 'heaters':
            return self
        return self.objects.get(name, default)
    def lookup_heater(self, name):
        return self.heater
    def load_object(self, config, name):
        return self.chip
    def add_object(self, name, obj):
        self.objects[name] = obj


class FakeChip:
    def __init__(self):
        self.modes = []
        self.ranges = []
        self.hotends = []
        self.mcu = FakeMcu()
    def get_reference_resistor(self):
        return R_REF
    def get_ref_nominal(self):
        return REF_NOM
    def get_temperature_interval(self):
        return 0.2
    def get_mcu(self):
        return self.mcu
    def register_hotend(self, hotend):
        self.hotends.append(hotend)
    def set_mode(self, mode):
        self.modes.append(mode)
    def set_range(self, *args):
        self.ranges.append(args)


class ConfigError(Exception):
    pass


class FakeConfig:
    error = ConfigError
    def __init__(self, printer, options):
        self.printer = printer
        self.options = options
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'extruder'
    def get(self, name, default=None):
        return self.options.get(name, default)
    def getfloat(self, name, default=None, **kw):
        v = self.options.get(name, default)
        return None if v is None else float(v)
    def getint(self, name, default=None, **kw):
        return int(self.options.get(name, default))
    def getchoice(self, name, choices, default=None):
        return choices[self.options.get(name, default)]


def make_heater(printer):
    h = object.__new__(heaters.Heater)
    import threading
    h.printer = printer
    h.lock = threading.Lock()
    h.name = h.short_name = 'extruder'
    h.pwm_holds = {}
    h.last_pwm_value = h.last_pwm_time = h.next_pwm_time = 0.
    h.pwm_delay = 0.2
    h.target_temp = 0.
    h.min_temp, h.max_temp = -70., 350.
    h.verify_mainthread_time = 1e9
    h.mcu_pwm = FakePwm()
    h.max_power = 1.
    c = object.__new__(heaters.ControlPID)
    c.heater = h
    c.heater_max_power = 1.
    c.Kp = c.Ki = c.Kd = 1.
    c.temp_integ_max = 1.
    h.control = c
    return h


def make_hotend(options=None, heatsink=None):
    chip = FakeChip()
    printer = FakePrinter(chip)
    heater = make_heater(printer)
    printer.heater = heater
    if heatsink is not None:
        printer.objects['temperature_fan heatsink'] = FakeHeatsink(heatsink)
    opts = {'muon_ads1220': 'toolhead', 'max_temp': 350.,
            'heatsink_sensor': ('temperature_fan heatsink'
                                if heatsink is not None else None),
            'ntc_pid_Kp': 20., 'ntc_pid_Ki': 2., 'ntc_pid_Kd': 40.}
    opts.update(options or {})
    hotend = m.MuonHotend(FakeConfig(printer, opts))
    hotend.setup_minmax(-70., 350.)
    temps = []
    hotend.setup_callback(lambda t, temp: temps.append((t, temp)))
    return hotend, chip, printer, heater, temps


######################################################################
# Sampler simulation (C)
######################################################################

def test_sampler_simulation():
    cc = shutil.which('cc') or shutil.which('gcc')
    if cc is None:
        print("skip test_sampler_simulation: no C compiler")
        return
    out_dir = tempfile.mkdtemp()
    exe = os.path.join(out_dir, 'sim_sampler')
    subprocess.check_call([cc, '-O2', '-Wall', '-Werror', '-o', exe,
                           os.path.join(HERE, 'sim_sampler.c'), '-lm'])
    res = subprocess.run([exe], stdout=subprocess.PIPE,
                         universal_newlines=True)
    sys.stdout.write(res.stdout)
    shutil.rmtree(out_dir, ignore_errors=True)
    assert res.returncode == 0, "sampler simulation failed"


######################################################################
# Detection
######################################################################

def ntc_model():
    hotend = make_hotend()[0]
    return hotend.ntc_temp, hotend

def ntc_resistance(hotend, temp):
    adc = hotend.ntc.calc_adc(temp)
    return R_REF * adc / (1. - adc)

def pt_reading(resistance):
    # sense, ref counts for a PT1000-mode reading of 'resistance'
    ref = REF_NOM
    return int(ref * resistance / R_REF), int(ref)

def points_for(resistances, dt=0.2, t0=0.):
    pts = []
    for i, r in enumerate(resistances):
        sense, ref = pt_reading(r)
        kind, res = m.classify_pt_reading(sense, ref, REF_NOM, R_REF)
        pts.append((t0 + i * dt, kind, res))
    return pts

def detect(resistances, heatsink=None):
    ntc_temp, _ = ntc_model()
    return m.detect_hotend(points_for(resistances), heatsink,
                           m.DetectParams(), ntc_temp)

def test_detect_cold_ntc_hits_compliance():
    # 100k at 25C: the IDAC cannot push 100uA, R43 reads ~22% of nominal
    pts = [(i * 0.2, ) + m.classify_pt_reading(m.ADC_MAX, int(REF_NOM * 0.22),
                                               REF_NOM, R_REF)
           for i in range(3)]
    ntc_temp, _ = ntc_model()
    v, reason = m.detect_hotend(pts, 25., m.DetectParams(), ntc_temp)
    assert v == 'ntc', (v, reason)

def test_detect_open_sensor():
    pts = [(i * 0.2, ) + m.classify_pt_reading(m.ADC_MAX, 30, REF_NOM, R_REF)
           for i in range(3)]
    ntc_temp, _ = ntc_model()
    v, reason = m.detect_hotend(pts, 25., m.DetectParams(), ntc_temp)
    assert v == 'fault' and 'open' in reason, (v, reason)

def test_detect_short_sensor():
    v, reason = detect([5., 5., 5.])
    assert v == 'fault' and 'short' in reason, (v, reason)

def test_detect_hot_ntc_below_band():
    _, hotend = ntc_model()
    r = ntc_resistance(hotend, 220.)
    assert r < 800.
    v, reason = detect([r] * 3, heatsink=60.)
    assert v == 'ntc', (v, reason)

def test_detect_warm_ntc_above_band():
    _, hotend = ntc_model()
    r = ntc_resistance(hotend, 90.)
    assert 3000. < r < 17000.
    v, reason = detect([r] * 3, heatsink=40.)
    assert v == 'ntc', (v, reason)

def test_detect_hot_ntc_cooling():
    # NTC at 150C cooling at 1C/s: resistance rises
    _, hotend = ntc_model()
    rs = [ntc_resistance(hotend, 150. - 0.2 * i) for i in range(20)]
    assert 800. < rs[0] < 3000.
    v, reason = detect(rs[:4], heatsink=45.)
    assert v is None, "decided before watching: %s %s" % (v, reason)
    v, reason = detect(rs, heatsink=45.)
    assert v == 'ntc' and 'rising' in reason, (v, reason)

def test_detect_hot_pt1000_cooling():
    rs = [m.pt1000_resistance(200. - 0.2 * i) for i in range(20)]
    v, reason = detect(rs, heatsink=50.)
    assert v == 'pt1000' and 'falling' in reason, (v, reason)

def test_detect_pt1000_at_ambient():
    rs = [m.pt1000_resistance(24.) + 0.05 * (i % 2) for i in range(20)]
    v, reason = detect(rs, heatsink=26.)
    assert v == 'pt1000' and 'steady' in reason, (v, reason)

def test_detect_pt1000_ambient_without_heatsink():
    rs = [m.pt1000_resistance(22.)] * 20
    v, reason = detect(rs)
    assert v == 'pt1000', (v, reason)

def test_detect_steady_but_inconsistent_waits():
    # Steady 1.6k is 160C as a PT1000; a 30C heatsink makes that
    # impossible with the heater off, so keep watching
    v, reason = detect([1600.] * 20, heatsink=30.)
    assert v is None, (v, reason)

def test_detect_cold_ntc_inconsistent_with_heatsink():
    # 5k as an NTC is ~100C; a 140C heatsink contradicts it. (Above
    # ~8.25k the PT1000-mode input saturates and reads 'high' instead.)
    v, reason = detect([5000.] * 3, heatsink=140.)
    assert v == 'fault', (v, reason)


######################################################################
# MCU range check limits
######################################################################

def mcu_range_ok(mode, rng, sense, ref):
    # Python copy of check_range() in src/sensor_muon_ads1220.c
    lo, hi, ref_lo, ref_hi = rng
    if mode == m.MODE_PT1000:
        if ref < ref_lo or ref > ref_hi or sense < 0:
            return False
        s, r = sense << 4, ref >> 8
        return lo * r <= s <= hi * r
    return lo <= sense <= hi

def test_range_pt1000():
    hotend = make_hotend()[0]
    rng = hotend._mcu_range('pt1000', 350.)
    assert all(abs(v) < (1 << 31) for v in rng)
    # products in the mcu stay inside 32 bits
    assert rng[1] * (rng[3] >> 8) < (1 << 32)
    def ok(temp, ref_scale=1.):
        sense, ref = pt_reading(m.pt1000_resistance(temp))
        return mcu_range_ok(m.MODE_PT1000, rng, int(sense * ref_scale),
                            int(ref * ref_scale))
    assert ok(25.) and ok(345.) and ok(-60.)
    assert ok(25., ref_scale=0.9) and ok(25., ref_scale=1.1)
    assert not ok(356.)
    assert not mcu_range_ok(m.MODE_PT1000, rng, 0, int(REF_NOM))  # short
    assert not mcu_range_ok(m.MODE_PT1000, rng, m.ADC_MAX, 40)    # open

def test_range_ntc():
    hotend = make_hotend()[0]
    rng = hotend._mcu_range('ntc', 265.)
    def counts(temp):
        r = ntc_resistance(hotend, temp)
        return int(R_REF / (r + R_REF) * (1 << 23))
    def ok(c):
        return mcu_range_ok(m.MODE_NTC, rng, c, 0)
    assert ok(counts(25.)) and ok(counts(260.)) and ok(counts(-60.))
    assert not ok(counts(270.))
    assert not ok(0) and not ok(-200)            # open: no current
    assert not ok(m.ADC_MAX)                     # short: full scale


######################################################################
# Heater hold
######################################################################

def test_heater_hold_and_refuse():
    printer = FakePrinter(FakeChip())
    h = make_heater(printer)
    h.target_temp = 200.
    h.set_pwm(10., 0.6)
    assert h.mcu_pwm.events == [(10.2, 0.6)]
    # A tap hold turns it off at once (not after the next reading) ...
    h.hold_pwm('tap', 10.05)
    assert h.mcu_pwm.events[-1] == (10.2, 0.), h.mcu_pwm.events
    # ... and keeps control from turning it back on
    h.set_pwm(10.2, 0.8)
    assert h.mcu_pwm.events[-1][1] == 0.
    assert h.target_temp == 200.
    h.release_pwm('tap')
    h.set_pwm(10.4, 0.8)
    assert h.mcu_pwm.events[-1] == (10.6, 0.8)
    # A refusing hold blocks a new target
    h.hold_pwm('detect', 11., refuse_msg="extruder: not detected")
    try:
        h.set_temp(210.)
    except CommandError as e:
        assert 'not detected' in str(e)
    else:
        raise AssertionError("set_temp was not refused")
    h.set_temp(0.)  # turning it off is always allowed
    h.release_pwm('detect')
    h.set_temp(210.)
    assert h.target_temp == 210.


######################################################################
# Detection state machine end to end
######################################################################

def feed(hotend, printer, t, mode, sense, ref):
    hotend.handle_reading(t, sense, ref, mode)

def run_detection(hotend, printer, reading_for, seconds):
    reactor = printer.reactor
    t_end = reactor.now + seconds
    wake = reactor.timers[hotend.detect_timer]
    while reactor.now < t_end:
        reactor.now = round(reactor.now + 0.05, 6)
        if int(round(reactor.now * 20)) % 4 == 0:
            mode = (hotend.chip.modes[-1] if hotend.chip.modes
                    else m.MODE_PT1000)
            sense, ref = reading_for(reactor.now, mode)
            feed(hotend, printer, reactor.now, mode, sense, ref)
        wake = reactor.timers[hotend.detect_timer]
        if wake <= reactor.now:
            reactor.timers[hotend.detect_timer] = hotend._detect_step(
                reactor.now)

def test_state_machine_pt1000():
    hotend, chip, printer, heater, temps = make_hotend(heatsink=24.)
    hotend.note_chip_started()
    assert HOLD_REFUSES(heater)
    def reading(t, mode):
        return pt_reading(m.pt1000_resistance(23.))
    run_detection(hotend, printer, reading, 8.)
    assert hotend.hotend_type == 'pt1000', (hotend.state, hotend.message)
    assert chip.modes == [], chip.modes      # never left PT1000 mode
    assert heater.max_temp == 350.
    assert not heater.pwm_holds
    assert chip.ranges and chip.ranges[-1][4] == 4
    assert temps and abs(temps[-1][1] - 23.) < 0.5, temps[-1:]

def test_state_machine_ntc():
    hotend, chip, printer, heater, temps = make_hotend(heatsink=25.)
    hotend.note_chip_started()
    r_ntc = ntc_resistance(hotend, 25.)
    def reading(t, mode):
        if mode == m.MODE_PT1000:
            return m.ADC_MAX, int(REF_NOM * 0.2)   # IDAC at compliance
        return int(R_REF / (r_ntc + R_REF) * (1 << 23)), 0
    run_detection(hotend, printer, reading, 8.)
    assert chip.modes == [m.MODE_NTC], chip.modes
    assert hotend.hotend_type == 'ntc', (hotend.state, hotend.message)
    assert heater.max_temp == 265.
    assert abs(heater.control.Kp * heaters.PID_PARAM_BASE - 20.) < 1e-9
    assert not heater.pwm_holds
    assert temps and abs(temps[-1][1] - 25.) < 0.5, temps[-1:]

def test_state_machine_reconnect_relatches():
    hotend, chip, printer, heater, temps = make_hotend(heatsink=24.)
    hotend.note_chip_started()
    run_detection(hotend, printer,
                  lambda t, mode: pt_reading(m.pt1000_resistance(23.)), 8.)
    assert hotend.hotend_type == 'pt1000'
    hotend.note_chip_lost()
    assert hotend.hotend_type is None and HOLD_REFUSES(heater)
    hotend.note_chip_started()
    assert hotend.state == 'settling' and HOLD_REFUSES(heater)

def test_tap_hold_follows_probe_window():
    hotend, chip, printer, heater, temps = make_hotend(heatsink=24.)
    hotend.note_chip_started()
    run_detection(hotend, printer,
                  lambda t, mode: pt_reading(m.pt1000_resistance(23.)), 8.)
    heater.target_temp = 200.
    heater.set_pwm(printer.reactor.now, 0.5)
    hotend.hold_for_tap()
    assert m.HOLD_TAP in heater.pwm_holds
    assert heater.mcu_pwm.events[-1][1] == 0.
    hotend.release_tap_hold()
    assert not heater.pwm_holds

def HOLD_REFUSES(heater):
    return bool(heater.pwm_holds.get(m.HOLD_DETECT))


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    failed = 0
    for test in tests:
        try:
            test()
            print("ok   %s" % test.__name__)
        except Exception as e:
            failed += 1
            import traceback
            traceback.print_exc()
            print("FAIL %s: %s" % (test.__name__, e))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
