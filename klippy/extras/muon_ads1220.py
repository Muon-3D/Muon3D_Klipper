# Muon B1 toolhead ADS1220: load cell and hotend temperature on one chip
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# One ADS1220 (DRDY not connected) measures both the load cell bridge and
# the hotend temperature sensor, which is either a 100k NTC or a PT1000:
#
#   [muon_ads1220 NAME]      the chip: SPI, timing, excitation pin
#   sensor_type: muon_ads1220  in [load_cell_probe] / [load_cell]
#   sensor_type: muon_hotend   in [extruder] (heater sensor)
#
# The mcu side (src/sensor_muon_ads1220.c) streams the load cell, slices in
# a temperature conversion every temperature_interval, pauses those slices
# while a probe trigger is armed, and range checks the temperature itself.
# This side detects which hotend is fitted on every toolhead connect and
# applies that hotend's sensor math, max_temp and PID.
import collections, logging, math, threading
from . import bulk_sensor, bus, heaters, thermistor

######################################################################
# Chip constants (TI SBAS501D)
######################################################################

TCLK = 1. / 4096000.
# Data rate: (DR bits, conversion tCLK continuous, single-shot tCLK)
TURBO_RATES = {
    '40': (0, 102384, 102434), '90': (1, 45560, 45618),
    '180': (2, 23064, 23122), '350': (3, 11832, 11890),
    '660': (4, 6232, 6290), '1200': (5, 3448, 3506),
    '2000': (6, 2072, 2130),
}
# Conversion start after a START/SYNC (or register write) in turbo mode
TURBO_START_TCLK = 114
OSC_TOLERANCE = 0.03   # internal oscillator is +/-2%; keep a margin
READ_RATE = 2.2        # RDATA reads per nominal conversion
SETTLE_MARGIN = 20e-6  # SPI and task latency on top of the conversion
GAINS = {'1': 0, '2': 1, '4': 2, '8': 3, '16': 4, '32': 5, '64': 6,
         '128': 7}
MUX = {'AIN0_AIN1': 0b0000, 'AIN0_AIN2': 0b0001, 'AIN0_AIN3': 0b0010,
       'AIN1_AIN2': 0b0011, 'AIN1_AIN3': 0b0100, 'AIN2_AIN3': 0b0101,
       'AIN1_AIN0': 0b0110, 'AIN3_AIN2': 0b0111}
MUX_AIN2_AIN3 = 0b0101
MUX_AIN3_AVSS = 0b1011
VREF_AVDD = 0b11
IDAC_CODES = {'50': 0b010, '100': 0b011, '250': 0b100}
I1MUX_REFP0 = 0b101
ADC_MAX = 0x7fffff
ADC_MIN = -0x800000
ADC_FULL_SCALE = float(1 << 23)
FILLER = -0x80000000  # MADS_FILLER: a slot that holds no measurement

MODE_OFF, MODE_PT1000, MODE_NTC = 0, 1, 2
MODE_NAMES = {MODE_PT1000: 'pt1000', MODE_NTC: 'ntc'}

UPDATE_INTERVAL = 0.02
BYTES_PER_SAMPLE = 4
SENSOR_ERRORS = {0: "ADS1220 stopped converting (frozen reading)"}

def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))

######################################################################
# Temperature sensor math
######################################################################

KELVIN_TO_CELSIUS = -273.15
# Callendar-Van Dusen, T >= 0 (the C term below 0C is far inside the
# accuracy a range check needs)
PT_A, PT_B = 3.9083e-3, -5.775e-7
PT_R0 = 1000.

def pt1000_resistance(temp):
    return PT_R0 * (1. + PT_A * temp + PT_B * temp * temp)

def pt1000_temp(resistance):
    disc = PT_A * PT_A - 4. * PT_B * (1. - resistance / PT_R0)
    if disc <= 0.:
        return 1000.
    return (-PT_A + math.sqrt(disc)) / (2. * PT_B)

# The B1 NTC is on the high side: 3.3V (AVDD) -> NTC -> AIN3 -> R43 -> GND,
# so AIN3 against AVSS with the AVDD reference reads R43 / (R43 + R_ntc).
def ntc_fraction_to_resistance(fraction, r_ref):
    if fraction <= 0.:
        return float('inf')
    return r_ref * (1. / fraction - 1.)

def slope(points):
    # Least squares slope of [(x, y), ...]
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    if sxx <= 0.:
        return 0.
    return sum((p[0] - mx) * (p[1] - my) for p in points) / sxx

######################################################################
# Hotend type detection (pure logic, unit tested)
######################################################################

class DetectParams:
    def __init__(self, ambiguous_min=800., ambiguous_max=3000.,
                 steady_rate=1.0, watch_time=3.0, heatsink_tolerance=15.,
                 short_resistance=50.):
        self.ambiguous_min = ambiguous_min
        self.ambiguous_max = ambiguous_max
        self.steady_rate = steady_rate
        self.watch_time = watch_time
        self.heatsink_tolerance = heatsink_tolerance
        self.short_resistance = short_resistance

# One PT1000-mode reading: (kind, resistance). kind is 'open' (no current
# reaches R43), 'high' (the IDAC is at compliance or the sensor reading
# saturates: far above any PT1000), or 'ok'.
def classify_pt_reading(sense, ref, ref_nominal, r_ref):
    if ref < 0.10 * ref_nominal:
        return 'open', None
    if sense >= ADC_MAX or ref < 0.85 * ref_nominal:
        return 'high', None
    if ref <= 0:
        return 'open', None
    return 'ok', r_ref * float(sense) / float(ref)

# Decide the hotend type from PT1000-mode readings taken with the heater
# off. points: [(time, kind, resistance), ...] oldest first. Returns
# (verdict, reason) with verdict 'ntc', 'pt1000' or 'fault', or
# (None, reason) when more readings are needed.
def detect_hotend(points, heatsink_temp, params, ntc_temp):
    if len(points) < 3:
        return None, "collecting readings"
    recent = points[-3:]
    kinds = set(k for _, k, _ in recent)
    if kinds == set(['open']):
        return 'fault', "hotend sensor open circuit (no excitation current)"
    if kinds == set(['high']):
        return 'ntc', ("resistance above any PT1000 reading (IDAC at"
                       " compliance)")
    if kinds != set(['ok']):
        return None, "readings unsettled"
    r = sorted(p[2] for p in recent)[1]
    def ntc_checked(reason):
        if heatsink_temp is not None:
            t = ntc_temp(r)
            if t < heatsink_temp - params.heatsink_tolerance:
                return 'fault', ("%.0f ohm reads as an NTC at %.0fC, colder"
                                 " than the heatsink (%.0fC)"
                                 % (r, t, heatsink_temp))
        return 'ntc', reason
    if r < params.short_resistance:
        return 'fault', "hotend sensor short circuit (%.0f ohm)" % (r,)
    if r < params.ambiguous_min:
        return ntc_checked("%.0f ohm is below any PT1000 reading: hot NTC"
                           % (r,))
    if r > params.ambiguous_max:
        return ntc_checked("%.0f ohm is above any PT1000 reading: NTC" % (r,))
    # Ambiguous band: a PT1000 near ambient or an NTC at ~125-170C. With
    # the heater off, a hot hotend can only be cooling, and cooling moves
    # the two in opposite directions.
    end = points[-1][0]
    window = [(t, res) for t, k, res in points
              if k == 'ok' and t >= end - params.watch_time]
    span = window[-1][0] - window[0][0] if window else 0.
    if len(window) < 5 or span < 0.9 * params.watch_time:
        return None, "watching %.0f ohm for a trend" % (r,)
    rate = slope(window)
    if rate > params.steady_rate:
        return ntc_checked("%.0f ohm rising %.1f ohm/s: hot NTC cooling"
                           % (r, rate))
    if rate < -params.steady_rate:
        return 'pt1000', ("%.0f ohm falling %.1f ohm/s: hot PT1000 cooling"
                          % (r, -rate))
    t_pt = pt1000_temp(r)
    if heatsink_temp is None:
        if 0. <= t_pt <= 60.:
            return 'pt1000', ("%.0f ohm steady: PT1000 at %.0fC" % (r, t_pt))
        return None, "%.0f ohm steady and no heatsink to compare" % (r,)
    if abs(t_pt - heatsink_temp) <= params.heatsink_tolerance:
        return 'pt1000', ("%.0f ohm steady: PT1000 at %.0fC, heatsink %.0fC"
                          % (r, t_pt, heatsink_temp))
    return None, ("%.0f ohm steady but %.0fC as a PT1000 against a %.0fC"
                  " heatsink" % (r, t_pt, heatsink_temp))

######################################################################
# The chip
######################################################################

class MuonADS1220:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.name = config.get_name().split()[-1]
        # Load cell channel
        self.gain = config.getchoice('gain', GAINS, default='64')
        rate = config.getchoice('sample_rate', {k: k for k in TURBO_RATES},
                                default='2000')
        self.dr_code, conv_tclk, single_tclk = TURBO_RATES[rate]
        self.mux = config.getchoice('input_mux', MUX, default='AIN0_AIN1')
        self.pga_bypass = config.getboolean('pga_bypass', False)
        if self.pga_bypass and self.gain > GAINS['4']:
            raise config.error("muon_ads1220: pga_bypass needs gain <= 4")
        # Hotend channel
        self.r_ref = config.getfloat('reference_resistor', 4700., above=0.)
        self.idac_ua = config.getchoice('pt1000_idac_current',
                                        {k: k for k in IDAC_CODES},
                                        default='100')
        self.pt_gain = config.getchoice('pt1000_gain',
                                        {'1': 0, '2': 1, '4': 2}, default='4')
        self.avdd = config.getfloat('analog_supply_voltage', 3.3, above=0.)
        self.temp_interval = config.getfloat('temperature_interval', 0.200,
                                             minval=0.100, maxval=0.250)
        settle_convs = config.getint('temperature_settle_conversions', 2,
                                     minval=1, maxval=4)
        max_pause = config.getfloat('max_temperature_pause', 4.0,
                                    minval=0.5, maxval=6.0)
        stall_convs = config.getint('stall_conversions', 64, minval=8,
                                    maxval=10000)
        # SPI and pins
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=4000000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        ppins = printer.lookup_object('pins')
        hs = ppins.lookup_pin(config.get('high_side_pin'), can_invert=True)
        if hs['chip'] is not mcu:
            raise config.error("muon_ads1220: high_side_pin must be on the"
                               " same mcu as the chip")
        # Timing
        self.conv_time = conv_tclk * TCLK
        self.sps = int(round(1. / self.conv_time))
        first = (TURBO_START_TCLK + single_tclk) * TCLK
        conv = self.conv_time
        self.timing = {
            'read': conv / READ_RATE,
            'conv': conv,
            'conv_min': conv * (1. - OSC_TOLERANCE),
            'conv_max': conv * (1. + OSC_TOLERANCE),
            'settle': ((first + (settle_convs - 1) * conv)
                       * (1. + OSC_TOLERANCE) + SETTLE_MARGIN),
            'lc_first': first,
            'lc_settle': first * (1. + OSC_TOLERANCE) + SETTLE_MARGIN,
            'temp': self.temp_interval,
            'max_pause': max_pause,
        }
        # Registers
        idac = IDAC_CODES[self.idac_ua]
        self.regs = {
            'lc_reg0': (self.mux << 4) | (self.gain << 1) | self.pga_bypass,
            # turbo mode, continuous conversion
            'reg1': (self.dr_code << 5) | (0b10 << 3) | (1 << 2),
            'pt_reg2': (VREF_AVDD << 6) | idac,
            'pt_reg3': I1MUX_REFP0 << 5,
            'pt_sense_reg0': (MUX_AIN2_AIN3 << 4) | (self.pt_gain << 1) | 1,
            'pt_ref_reg0': (MUX_AIN3_AVSS << 4) | (self.pt_gain << 1) | 1,
            'ntc_reg2': VREF_AVDD << 6,
            'ntc_reg3': 0,
            'ntc_sense_reg0': (MUX_AIN3_AVSS << 4) | (0 << 1) | 1,
        }
        # Expected reading of the 4.7k reference in PT1000 mode
        pt_fs = self.avdd / (1 << self.pt_gain)
        self.ref_nominal = (float(self.idac_ua) * 1e-6 * self.r_ref
                            / pt_fs * ADC_FULL_SCALE)
        # Load cell stream
        chip_smooth = self.sps * 0.10 * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # State
        self.mode = MODE_OFF
        self.load_cells = []
        self.hotends = []
        self.blind_count = 0
        self.last_error = None
        self.start_cmd = self.range_cmd = self.lc_cmd = None
        self.stats_cmd = None
        self.last_stats = {}
        # MCU configuration
        tick = mcu.seconds_to_clock
        t = self.timing
        mcu.add_config_cmd(
            "config_muon_ads1220 oid=%d spi_oid=%d high_side_pin=%s"
            " high_side_off_value=%d" % (self.oid, self.spi.get_oid(),
                                          hs['pin'], hs['invert']))
        mcu.add_config_cmd(
            "muon_ads1220_set_regs oid=%d lc_reg0=%d reg1=%d pt_reg2=%d"
            " pt_reg3=%d pt_sense_reg0=%d pt_ref_reg0=%d ntc_reg2=%d"
            " ntc_reg3=%d ntc_sense_reg0=%d" % (
                self.oid, self.regs['lc_reg0'], self.regs['reg1'],
                self.regs['pt_reg2'], self.regs['pt_reg3'],
                self.regs['pt_sense_reg0'], self.regs['pt_ref_reg0'],
                self.regs['ntc_reg2'], self.regs['ntc_reg3'],
                self.regs['ntc_sense_reg0']))
        self._clock_cfg = (t, tick, stall_convs)
        mcu.add_config_cmd("muon_ads1220_start oid=%d mode=0" % (self.oid,),
                           on_restart=True)
        mcu.add_config_cmd("muon_ads1220_lc_enable oid=%d enable=0"
                           % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        mcu.register_response(self._handle_temp, "muon_ads1220_temp",
                              self.oid)
        mcu.register_response(self._handle_error, "muon_ads1220_error",
                              self.oid)
        mcu.register_response(self._handle_blind, "muon_ads1220_blind",
                              self.oid)
        mcu.register_response(self._handle_stats, "muon_ads1220_stats",
                              self.oid)
        # Events
        printer.register_event_handler("klippy:connect", self._handle_connect)
        if getattr(mcu, "is_non_critical", False):
            printer.register_event_handler(
                mcu.get_non_critical_reconnect_event_name(),
                self._handle_connect)
            printer.register_event_handler(
                mcu.get_non_critical_disconnect_event_name(),
                self._handle_disconnect)
        printer.register_event_handler("load_cell_probe:tap_begin",
                                       self._handle_tap_begin)
        printer.register_event_handler("load_cell_probe:tap_end",
                                       self._handle_tap_end)
        gcode = printer.lookup_object('gcode')
        gcode.register_mux_command("MUON_ADS1220_STATS", "CHIP", self.name,
                                   self.cmd_MUON_ADS1220_STATS,
                                   desc=self.cmd_MUON_ADS1220_STATS_help)

    def _build_config(self):
        t, tick, stall_convs = self._clock_cfg
        # Ticks depend on the mcu frequency, known only now
        self.mcu.add_config_cmd(
            "muon_ads1220_set_timing oid=%d read_ticks=%d conv_ticks=%d"
            " conv_min_ticks=%d conv_max_ticks=%d" % (
                self.oid, tick(t['read']), tick(t['conv']),
                tick(t['conv_min']), tick(t['conv_max'])))
        self.mcu.add_config_cmd(
            "muon_ads1220_set_schedule oid=%d settle_ticks=%d"
            " lc_first_ticks=%d lc_settle_ticks=%d temp_ticks=%d"
            " max_pause_ticks=%d stall_count=%d" % (
                self.oid, tick(t['settle']), tick(t['lc_first']),
                tick(t['lc_settle']), tick(t['temp']), tick(t['max_pause']),
                stall_convs))
        cmdqueue = self.spi.get_command_queue()
        self.start_cmd = self.mcu.lookup_command(
            "muon_ads1220_start oid=%c mode=%c", cq=cmdqueue)
        self.range_cmd = self.mcu.lookup_command(
            "muon_ads1220_set_range oid=%c min_value=%i max_value=%i"
            " ref_min=%i ref_max=%i fault_count=%c", cq=cmdqueue)
        self.lc_cmd = self.mcu.lookup_command(
            "muon_ads1220_lc_enable oid=%c enable=%c", cq=cmdqueue)
        self.stats_cmd = self.mcu.lookup_query_command(
            "muon_ads1220_query_stats oid=%c",
            "muon_ads1220_stats2 oid=%c temp=%u blind=%u overrun=%u"
            " stall=%u mode=%c", oid=self.oid, cq=cmdqueue)
        self.ffreader.setup_query_command("muon_ads1220_query_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)

    # Interfaces for the load cell adapter and the hotend sensor
    def get_mcu(self):
        return self.mcu
    def get_reference_resistor(self):
        return self.r_ref
    def get_ref_nominal(self):
        return self.ref_nominal
    def get_temperature_interval(self):
        return self.temp_interval
    def get_samples_per_second(self):
        return self.sps
    def register_load_cell(self, adapter):
        self.load_cells.append(adapter)
    def register_hotend(self, hotend):
        self.hotends.append(hotend)
    def setup_trigger_analog(self, trigger_analog_oid):
        self.mcu.add_config_cmd(
            "muon_ads1220_attach_trigger_analog oid=%d trigger_analog_oid=%d"
            % (self.oid, trigger_analog_oid), is_init=True)
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    # Hotend excitation. The sensor's 1.1uF input filter takes hundreds of
    # ms to settle after a change, so this is only for detection, never
    # while the heater is in use.
    def set_mode(self, mode):
        self.mode = mode
        self.start_cmd.send([self.oid, mode])
    def get_mode(self):
        return self.mode
    def set_range(self, min_value, max_value, ref_min, ref_max, count):
        self.range_cmd.send([self.oid, int(min_value), int(max_value),
                             int(ref_min), int(ref_max), count])

    # Connection lifecycle
    def _handle_connect(self):
        if getattr(self.mcu, "non_critical_disconnected", False):
            # Started by the reconnect event instead
            return
        self.last_error = None
        self.set_mode(MODE_PT1000)
        for hotend in self.hotends:
            hotend.note_chip_started()
    def _handle_disconnect(self):
        self.mode = MODE_OFF
        for hotend in self.hotends:
            hotend.note_chip_lost()

    # MCU messages (these arrive on the serial thread)
    def _handle_temp(self, params):
        clock = self.mcu.clock32_to_clock64(params['clock'])
        read_time = self.mcu.clock_to_print_time(clock)
        for hotend in self.hotends:
            hotend.handle_reading(read_time, params['sense'], params['ref'],
                                  params['mode'])
    def _handle_error(self, params):
        self.last_error = ("ADS1220 '%s' did not accept its configuration"
                           " (wiring or chip fault)" % (self.name,))
        logging.error(self.last_error)
        self.mode = MODE_OFF
        for hotend in self.hotends:
            self.reactor.register_async_callback(
                (lambda e, h=hotend: h.note_chip_error(self.last_error)))
    def _handle_blind(self, params):
        self.blind_count += 1
        logging.warning("ADS1220 '%s': probe armed past max_temperature_pause;"
                        " a temperature read briefly took the load cell"
                        " (%d so far)", self.name, self.blind_count)
    def _handle_stats(self, params):
        self.last_stats = dict(params)

    # Probe windows: hold the heater off
    def _handle_tap_begin(self, load_cell):
        if load_cell.get_sensor() in self.load_cells:
            for hotend in self.hotends:
                hotend.hold_for_tap()
    def _handle_tap_end(self, load_cell):
        if load_cell.get_sensor() in self.load_cells:
            for hotend in self.hotends:
                hotend.release_tap_hold()

    # Load cell stream
    def _start_measurements(self):
        self.lc_cmd.send([self.oid, 1])
        logging.info("ADS1220 '%s' starting load cell measurements",
                     self.name)
        self.ffreader.note_start()
    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.lc_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS1220 '%s' finished load cell measurements",
                     self.name)
    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        adc_factor = 1. / ADC_FULL_SCALE
        out = [(round(ptime, 6), val, round(val * adc_factor, 9))
               for ptime, val in samples if val != FILLER]
        return {'data': out, 'errors': 0,
                'overflows': self.ffreader.get_last_overflows()}

    cmd_MUON_ADS1220_STATS_help = "Report ADS1220 sampling statistics"
    def cmd_MUON_ADS1220_STATS(self, gcmd):
        params = self.stats_cmd.send([self.oid])
        s = self.last_stats
        gcmd.respond_info(
            "ADS1220 %s: new=%s repeat=%s hold=%s filler=%s late=%s"
            " temp=%d blind=%d overrun=%d stall=%d mode=%s" % (
                self.name, s.get('new'), s.get('dup'), s.get('hold'),
                s.get('filler'), s.get('late'), params['temp'],
                params['blind'], params['overrun'], params['stall'],
                MODE_NAMES.get(params['mode'], 'off')))

######################################################################
# Load cell sensor (sensor_type: muon_ads1220)
######################################################################

class MuonADS1220LoadCell:
    def __init__(self, config):
        printer = config.get_printer()
        chip_name = config.get('muon_ads1220')
        self.chip = printer.load_object(config, 'muon_ads1220 ' + chip_name)
        self.chip.register_load_cell(self)
    def get_mcu(self):
        return self.chip.get_mcu()
    def get_samples_per_second(self):
        return self.chip.get_samples_per_second()
    def get_range(self):
        return ADC_MIN, ADC_MAX
    def lookup_sensor_error(self, error_code):
        return SENSOR_ERRORS.get(error_code,
                                 "Unknown ADS1220 error %d" % (error_code,))
    def setup_trigger_analog(self, trigger_analog_oid):
        self.chip.setup_trigger_analog(trigger_analog_oid)
    def add_client(self, callback):
        self.chip.add_client(callback)

MUON_ADS1220_SENSOR_TYPE = {"muon_ads1220": MuonADS1220LoadCell}

######################################################################
# Hotend temperature sensor (sensor_type: muon_hotend)
######################################################################

HOLD_DETECT = "muon_hotend detect"
HOLD_TAP = "muon_hotend tap"
PIN_MIN_TIME = 0.100
DETECT_INTERVAL = 0.25
NTC_104NT = ((25., 100000.), (160., 1074.), (300., 82.78))

class HotendProfile:
    def __init__(self, config, prefix, max_temp, heater_max):
        self.name = prefix
        self.max_temp = config.getfloat(prefix + '_max_temp', max_temp,
                                        above=0., maxval=heater_max)
        pid = [config.getfloat(prefix + '_pid_' + k, None)
               for k in ('Kp', 'Ki', 'Kd')]
        if any(v is not None for v in pid) and None in pid:
            raise config.error("%s: set all of %s_pid_Kp, _Ki and _Kd or"
                               " none" % (config.get_name(), prefix))
        self.pid = pid if pid[0] is not None else None

class MuonHotend:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.heater_name = config.get_name().split()[-1]
        chip_name = config.get('muon_ads1220')
        self.chip = printer.load_object(config, 'muon_ads1220 ' + chip_name)
        self.r_ref = self.chip.get_reference_resistor()
        self.heatsink_name = config.get('heatsink_sensor', None)
        self.forced_type = config.getchoice(
            'hotend_type', {'auto': None, 'ntc': 'ntc', 'pt1000': 'pt1000'},
            default='auto')
        # NTC model (default Semitec 104NT-4-R025H42G)
        pts = []
        for i, (t, r) in enumerate(NTC_104NT):
            pts.append(config.getfloat('ntc_temperature%d' % (i + 1,), t))
            pts.append(config.getfloat('ntc_resistance%d' % (i + 1,), r,
                                       above=0.))
        self.ntc = thermistor.Thermistor(self.r_ref, 0.)
        self.ntc.setup_coefficients(*pts, name=config.get_name())
        heater_max = config.getfloat('max_temp')
        self.profiles = {
            'ntc': HotendProfile(config, 'ntc', min(265., heater_max),
                                 heater_max),
            'pt1000': HotendProfile(config, 'pt1000', min(350., heater_max),
                                    heater_max),
        }
        self.range_check_count = config.getint('range_check_count', 4,
                                               minval=1, maxval=20)
        self.params = DetectParams(
            ambiguous_min=config.getfloat('detect_min_resistance', 800.,
                                          above=0.),
            ambiguous_max=config.getfloat('detect_max_resistance', 3000.,
                                          above=0.),
            steady_rate=config.getfloat('detect_steady_rate', 1.0, above=0.),
            watch_time=config.getfloat('detect_watch_time', 3.0, minval=1.),
            heatsink_tolerance=config.getfloat('detect_heatsink_tolerance',
                                               15., above=0.))
        self.settle_time = config.getfloat('detect_settle_time', 1.5,
                                           minval=0.5)
        self.ntc_settle_time = config.getfloat('ntc_settle_time', 1.0,
                                               minval=0.2)
        self.detect_timeout = config.getfloat('detect_timeout', 30.,
                                              minval=5.)
        # Heater interface
        self.min_temp = self.max_temp = 0.
        self.callback = None
        self.heater = None
        # Detection state
        self.lock = threading.Lock()
        self.readings = collections.deque(maxlen=400)
        self.state = 'disconnected'
        self.hotend_type = None
        self.active_mode = MODE_OFF
        self.message = "toolhead not connected"
        self.last_resistance = None
        self.state_time = self.detect_from = 0.
        self.detect_reason = ''
        self.detect_timer = self.reactor.register_timer(self._detect_step)
        self.chip.register_hotend(self)
        printer.add_object('muon_hotend ' + self.heater_name, self)
        gcode = printer.lookup_object('gcode')
        gcode.register_mux_command("MUON_HOTEND_DETECT", "HEATER",
                                   self.heater_name,
                                   self.cmd_MUON_HOTEND_DETECT,
                                   desc=self.cmd_MUON_HOTEND_DETECT_help)
        gcode.register_mux_command("MUON_HOTEND_STATUS", "HEATER",
                                   self.heater_name,
                                   self.cmd_MUON_HOTEND_STATUS,
                                   desc=self.cmd_MUON_HOTEND_STATUS_help)

    # Heater sensor interface
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_callback(self, cb):
        self.callback = cb
    def get_report_time_delta(self):
        return self.chip.get_temperature_interval()

    def _get_heater(self):
        if self.heater is None:
            pheaters = self.printer.lookup_object('heaters')
            self.heater = pheaters.lookup_heater(self.heater_name)
        return self.heater
    def _print_time(self, lead=PIN_MIN_TIME):
        mcu = self.chip.get_mcu()
        return mcu.estimated_print_time(self.reactor.monotonic()) + lead
    def _refuse(self, msg, send=True):
        heater = self._get_heater()
        heater.hold_pwm(HOLD_DETECT, self._print_time() if send else None,
                        refuse_msg="%s: %s" % (self.heater_name, msg))

    # Conversions
    def _resistance(self, sense, ref, mode):
        if mode == MODE_PT1000:
            if ref <= 0:
                return float('inf')
            return self.r_ref * float(sense) / float(ref)
        return ntc_fraction_to_resistance(sense / ADC_FULL_SCALE, self.r_ref)
    def _temperature(self, sense, ref, mode):
        if mode == MODE_PT1000:
            return pt1000_temp(self._resistance(sense, ref, mode))
        # thermistor.py expects R / (R + pullup)
        return self.ntc.calc_temp(1. - sense / ADC_FULL_SCALE)
    def ntc_temp(self, resistance):
        return self.ntc.calc_temp(resistance / (resistance + self.r_ref))

    # Readings arrive on the serial thread
    def handle_reading(self, read_time, sense, ref, mode):
        with self.lock:
            self.readings.append((read_time, sense, ref, mode))
            active = (self.active_mode == mode
                      and self.state in ('ntc', 'pt1000'))
        if mode == MODE_PT1000 or mode == MODE_NTC:
            self.last_resistance = self._resistance(sense, ref, mode)
        if active and self.callback is not None:
            self.callback(read_time, self._temperature(sense, ref, mode))

    # Chip lifecycle
    def note_chip_started(self):
        self.hotend_type = None
        self.active_mode = MODE_OFF
        self._refuse("hotend type not yet detected")
        self._set_state('settling', "measuring the hotend sensor")
        with self.lock:
            self.readings.clear()
        self.reactor.update_timer(self.detect_timer, self.reactor.NOW)
    def note_chip_lost(self):
        self.hotend_type = None
        self.active_mode = MODE_OFF
        self._refuse("toolhead not connected", send=False)
        self.heater.release_pwm(HOLD_TAP)
        self._set_state('disconnected', "toolhead not connected")
        self.reactor.update_timer(self.detect_timer, self.reactor.NEVER)
    def note_chip_error(self, msg):
        self.hotend_type = None
        self.active_mode = MODE_OFF
        self._refuse(msg)
        self._set_state('failed', msg)
        self.reactor.update_timer(self.detect_timer, self.reactor.NEVER)

    def _set_state(self, state, message):
        changed = state != self.state or message != self.message
        self.state = state
        self.message = message
        self.state_time = self.reactor.monotonic()
        if changed:
            logging.info("muon_hotend %s: %s (%s)", self.heater_name, state,
                         message)

    def _respond(self, msg):
        try:
            self.printer.lookup_object('gcode').respond_info(msg)
        except Exception:
            logging.exception("muon_hotend respond")

    def _heatsink_temp(self, eventtime):
        if self.heatsink_name is None:
            return None
        obj = self.printer.lookup_object(self.heatsink_name, None)
        if obj is None:
            return None
        temp = obj.get_temp(eventtime)[0]
        # temperature_fan reports 0 until its sensor has a reading
        return temp if temp else None

    def _pt_points(self, since):
        with self.lock:
            readings = [r for r in self.readings
                        if r[3] == MODE_PT1000 and r[0] >= since]
        ref_nom = self.chip.get_ref_nominal()
        points = []
        for t, sense, ref, mode in readings:
            kind, res = classify_pt_reading(sense, ref, ref_nom, self.r_ref)
            points.append((t, kind, res))
        return points

    # Detection, run from a reactor timer with the heater held off
    def _detect_step(self, eventtime):
        state = self.state
        elapsed = eventtime - self.state_time
        if state == 'settling':
            if elapsed < self.settle_time:
                return eventtime + DETECT_INTERVAL
            if self.forced_type is not None:
                return self._decide(eventtime, self.forced_type,
                                    "set by hotend_type")
            self.detect_from = self.chip.get_mcu().estimated_print_time(
                eventtime)
            self._set_state('measuring', "classifying the hotend sensor")
            return eventtime + DETECT_INTERVAL
        if state in ('measuring', 'failed'):
            points = self._pt_points(self.detect_from)
            verdict, reason = detect_hotend(
                points, self._heatsink_temp(eventtime), self.params,
                self.ntc_temp)
            if verdict in ('ntc', 'pt1000'):
                return self._decide(eventtime, verdict, reason)
            if verdict == 'fault':
                if state != 'failed' or reason != self.message:
                    self._respond("Hotend sensor fault: %s. Heating is"
                                  " refused." % (reason,))
                    self._refuse(reason)
                self._set_state('failed', reason)
                # Keep looking: a sensor plugged back in recovers
                self.detect_from = self.chip.get_mcu().estimated_print_time(
                    eventtime)
                return eventtime + 1.
            if state == 'measuring' and elapsed > self.detect_timeout:
                msg = "could not determine the hotend type (%s)" % (reason,)
                self._respond("Hotend detection: %s. Heating is refused."
                              % (msg,))
                self._refuse(msg)
                self._set_state('failed', msg)
                return eventtime + 1.
            if state == 'measuring':
                self.message = reason
            return eventtime + DETECT_INTERVAL
        if state == 'switching':
            if elapsed < self.ntc_settle_time:
                return eventtime + DETECT_INTERVAL
            since = self.chip.get_mcu().estimated_print_time(eventtime) - 0.6
            with self.lock:
                ntc = [r for r in self.readings
                       if r[3] == MODE_NTC and r[0] >= since]
            if len(ntc) < 2:
                if elapsed > self.ntc_settle_time + 2.:
                    msg = "no readings after switching to NTC excitation"
                    self._refuse(msg)
                    self._set_state('failed', msg)
                    return self.reactor.NEVER
                return eventtime + DETECT_INTERVAL
            temp = self._temperature(ntc[-1][1], ntc[-1][2], MODE_NTC)
            prof = self.profiles['ntc']
            if not (self.min_temp < temp < prof.max_temp):
                msg = "NTC reads %.1fC after switching, outside %.0f:%.0f" % (
                    temp, self.min_temp, prof.max_temp)
                self._respond("Hotend sensor fault: %s. Heating is refused."
                              % (msg,))
                self._refuse(msg)
                self._set_state('failed', msg)
                return self.reactor.NEVER
            self._apply('ntc')
            return self.reactor.NEVER
        return self.reactor.NEVER

    def _decide(self, eventtime, verdict, reason):
        self.detect_reason = reason
        if verdict == 'ntc':
            self.chip.set_mode(MODE_NTC)
            self._set_state('switching', "NTC: %s" % (reason,))
            return eventtime + DETECT_INTERVAL
        self._apply('pt1000')
        return self.reactor.NEVER

    def _mcu_range(self, hotend_type, max_temp):
        if hotend_type == 'pt1000':
            lo = pt1000_resistance(self.min_temp) / self.r_ref * 4096.
            hi = pt1000_resistance(max_temp) / self.r_ref * 4096.
            ref_nom = self.chip.get_ref_nominal()
            return (math.floor(lo), math.ceil(hi), int(ref_nom * 0.75),
                    int(ref_nom * 1.25))
        # NTC: the reading rises with temperature
        lo = (1. - self.ntc.calc_adc(self.min_temp)) * ADC_FULL_SCALE
        hi = (1. - self.ntc.calc_adc(max_temp)) * ADC_FULL_SCALE
        return math.floor(lo), min(ADC_MAX, math.ceil(hi)), 0, 0

    def _apply(self, hotend_type):
        prof = self.profiles[hotend_type]
        heater = self._get_heater()
        mode = MODE_NTC if hotend_type == 'ntc' else MODE_PT1000
        max_temp = min(prof.max_temp, self.max_temp)
        lo, hi, ref_lo, ref_hi = self._mcu_range(hotend_type, max_temp)
        self.chip.set_range(lo, hi, ref_lo, ref_hi, self.range_check_count)
        heater.max_temp = max_temp
        if prof.pid is not None:
            control = heater.control
            if isinstance(control, heaters.ControlPID):
                kp, ki, kd = prof.pid
                control.Kp = kp / heaters.PID_PARAM_BASE
                control.Ki = ki / heaters.PID_PARAM_BASE
                control.Kd = kd / heaters.PID_PARAM_BASE
                control.temp_integ_max = (control.heater_max_power
                                          / control.Ki if control.Ki else 0.)
            else:
                logging.warning("muon_hotend %s: %s_pid_* ignored, the heater"
                                " control is not pid", self.heater_name,
                                hotend_type)
        if heater.target_temp > max_temp:
            heater.target_temp = 0.
            self._respond("Hotend target above the %s limit of %.0fC was"
                          " cleared" % (hotend_type, max_temp))
        self.hotend_type = hotend_type
        with self.lock:
            self.active_mode = mode
        self._set_state(hotend_type, self.detect_reason)
        heater.release_pwm(HOLD_DETECT)
        self._respond("Hotend detected: %s (max_temp %.0fC). %s" % (
            'NTC' if hotend_type == 'ntc' else 'PT1000', max_temp,
            self.message))

    # Probe windows
    def hold_for_tap(self):
        if self.heater is None or self.state == 'disconnected':
            return
        self.heater.hold_pwm(HOLD_TAP, self._print_time())
    def release_tap_hold(self):
        if self.heater is not None:
            self.heater.release_pwm(HOLD_TAP)

    def get_status(self, eventtime):
        return {
            'hotend_type': self.hotend_type,
            'state': self.state,
            'message': self.message,
            'resistance': (None if self.last_resistance is None
                           or math.isinf(self.last_resistance)
                           else round(self.last_resistance, 1)),
            'max_temp': (self.profiles[self.hotend_type].max_temp
                         if self.hotend_type else None),
        }

    cmd_MUON_HOTEND_DETECT_help = "Re-run hotend type detection"
    def cmd_MUON_HOTEND_DETECT(self, gcmd):
        heater = self._get_heater()
        if heater.target_temp or self.state == 'disconnected':
            raise gcmd.error("Hotend detection needs the toolhead connected"
                             " and the heater off")
        self.chip.set_mode(MODE_PT1000)
        self.note_chip_started()
        gcmd.respond_info("Hotend detection restarted")
    cmd_MUON_HOTEND_STATUS_help = "Report the detected hotend"
    def cmd_MUON_HOTEND_STATUS(self, gcmd):
        st = self.get_status(self.reactor.monotonic())
        gcmd.respond_info("Hotend %s: type=%s state=%s resistance=%s (%s)" % (
            self.heater_name, st['hotend_type'], st['state'],
            st['resistance'], st['message']))

######################################################################
# Registration
######################################################################

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("muon_hotend", MuonHotend)

def load_config_prefix(config):
    return MuonADS1220(config)
