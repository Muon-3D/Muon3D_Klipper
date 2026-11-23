# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, time
import mcu
from . import probe, trigger_analog, load_cell, hx71x, ads1220, muon_ads1220

np = None  # delay NumPy import until configuration time

# KAN-1730: largest probe z_offset the M1 stages or uses (mm)
MAX_Z_OFFSET = 2.0

# MCU SOS filter scaled to "fractional grams" for consistent sensor precision
FRAC_GRAMS_CONV = 32768.0


class TapAnalysis:
    def __init__(self, samples):
        nd_samples = np.asarray(samples, dtype=np.float64)
        self.time = nd_samples[:, 0]
        self.force = nd_samples[:, 1]

    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            'time': self.time.tolist(), 'force': self.force.tolist(),
            'is_valid': True,
        }


# Access a parameter from config or GCode command via a consistent interface
# stores name and constraints to keep things DRY
class ParamHelper:
    def __init__(self, config, name, type_name, default=None, minval=None,
            maxval=None, above=None, below=None, max_len=None):
        self._config_section = config.get_name()
        self._config_error = config.error
        self.name = name
        self._type_name = type_name
        self.value = default
        self.minval = minval
        self.maxval = maxval
        self.above = above
        self.below = below
        self.max_len = max_len
        # read from config once
        self.value = self.get(config=config)

    def _get_name(self, gcmd):
        return self.name.upper() if gcmd else self.name

    def _validate_float(self, description, error, value, above, below):
        above = above or self.above
        if above is not None and value <= above:
            raise error("%s must be above %s" % (description, above))
        below = below or self.below
        if below is not None and value >= below:
            raise error("%s must be below %s" % (description, below))

    # support for validating individual options in a list of floats
    def _validate_float_list(self, gcmd, values, above, below):
        if gcmd:
            description = ("Error on '%s': %s" % (
                gcmd.get_commandline(), self._get_name(gcmd)))
            error = gcmd.error
        else:
            description = ("Option '%s' in section '%s'" % (
                self._get_name(gcmd), self._config_section))
            error = self._config_error
        if self.max_len is not None and len(values) > self.max_len:
            raise error(
                "%s has maximum length %s" % (description, self.max_len))
        for value in values:
            self._validate_float(description, error, value, above, below)

    def _get_int(self, config, gcmd, minval, maxval):
        get = gcmd.get_int if gcmd else config.getint
        return get(self._get_name(gcmd), self.value, minval or self.minval,
            maxval or self.maxval)

    def _get_float(self, config, gcmd, minval, maxval, above, below):
        get = gcmd.get_float if gcmd else config.getfloat
        return get(self._get_name(gcmd), self.value, minval or self.minval,
            maxval or self.maxval, above or self.above, below or self.below)

    def _get_float_list(self, config, gcmd, above, below):
        # this code defaults to the empty list, never return None
        default = (self.value or [])
        if gcmd:
            # if the parameter isn't part of the command, return the default
            if not self._get_name(gcmd) in gcmd.get_command_parameters():
                return default
            # parameter exists, always prefer whatever is in the command
            value = gcmd.get(self._get_name(gcmd), default='')
            # Return an empty list for empty value
            if len(value.strip()) == 0:
                return []
            try:
                float_list = [float(p.strip()) for p in value.split(',')]
            except:
                raise gcmd.error("Error on '%s': unable to parse %s" % (
                    gcmd.get_commandline(), value))
        else:
            float_list = config.getfloatlist(self._get_name(gcmd),
                default=default)
        if float_list:
            self._validate_float_list(gcmd, float_list, above, below)
        return float_list

    def get(self, gcmd=None, minval=None, maxval=None, above=None, below=None,
            config=None):
        if config is None and gcmd is None:
            return self.value
        if self._type_name == 'int':
            return self._get_int(config, gcmd, minval, maxval)
        elif self._type_name == 'float':
            return self._get_float(config, gcmd, minval, maxval, above, below)
        else:
            return self._get_float_list(config, gcmd, above, below)


def intParamHelper(config, name, default=None, minval=None, maxval=None):
    return ParamHelper(config, name, 'int', default, minval=minval,
        maxval=maxval)


def floatParamHelper(config, name, default=None, minval=None, maxval=None,
        above=None, below=None):
    return ParamHelper(config, name, 'float', default, minval=minval,
        maxval=maxval, above=above, below=below)


def floatListParamHelper(config, name, default=None, above=None, below=None,
        max_len=None):
    return ParamHelper(config, name, 'float_list', default, above=above,
        below=below, max_len=max_len)


# container for filter parameters
# allows different filter configurations to be compared
class ContinuousTareFilter:
    def __init__(self, sps=None, drift=None, drift_delay=None, buzz=None,
            buzz_delay=None, notches=None, notch_quality=None):
        self.sps = sps
        self.drift = drift
        self.drift_delay = drift_delay
        self.buzz = buzz
        self.buzz_delay = buzz_delay
        self.notches = notches
        self.notch_quality = notch_quality

    def __eq__(self, other):
        if not isinstance(other, ContinuousTareFilter):
            return False
        return (
                self.sps == other.sps and self.drift == other.drift and
                self.drift_delay == other.drift_delay and self.buzz ==
                other.buzz and self.buzz_delay == other.buzz_delay and
                self.notches == other.notches and self.notch_quality ==
                other.notch_quality)

    # create a filter design from the parameters
    def design_filter(self, error_func):
        return trigger_analog.DigitalFilter(self.sps, error_func, self.drift,
            self.drift_delay, self.buzz, self.buzz_delay, self.notches,
            self.notch_quality)


# Combine ContinuousTareFilter and SosFilter into an easy-to-use class
class ContinuousTareFilterHelper:
    def __init__(self, config, sensor, sos_filter):
        self._sensor = sensor
        self._sos_filter = sos_filter
        self._sps = self._sensor.get_samples_per_second()
        max_filter_frequency = math.floor(self._sps / 2.)
        # setup filter parameters
        self._drift_param = floatParamHelper(config,
            "drift_filter_cutoff_frequency", default=None, minval=0.1,
            maxval=20.0)
        self._drift_delay_param = intParamHelper(config, "drift_filter_delay",
            default=2, minval=1, maxval=2)
        self._buzz_param = floatParamHelper(config,
            "buzz_filter_cutoff_frequency", default=None,
            above=min(80.0, max_filter_frequency - 1.0),
            below=max_filter_frequency)
        self._buzz_delay_param = intParamHelper(config, "buzz_filter_delay",
            default=2, minval=1, maxval=2)
        self._notches_param = floatListParamHelper(config,
            "notch_filter_frequencies", default=[], above=0.,
            below=max_filter_frequency, max_len=2)
        self._notch_quality_param = floatParamHelper(config,
            "notch_filter_quality", default=2.0, minval=0.5, maxval=6.0)
        # filter design specified in the config file, used for defaults
        self._config_design = ContinuousTareFilter()  # empty filter
        self._config_design = self._build_filter()
        # filter design currently inside the MCU
        self._active_design = self._config_design
        design = self._active_design.design_filter(config.error)
        self._sos_filter.set_filter_design(design)

    def _build_filter(self, gcmd=None):
        drift = self._drift_param.get(gcmd)
        drift_delay = self._drift_delay_param.get(gcmd)
        buzz = self._buzz_param.get(gcmd)
        buzz_delay = self._buzz_delay_param.get(gcmd)
        # notches must be between drift and buzz:
        notches = self._notches_param.get(gcmd, above=drift, below=buzz)
        notch_quality = self._notch_quality_param.get(gcmd)
        return ContinuousTareFilter(self._sps, drift, drift_delay, buzz,
            buzz_delay, notches, notch_quality)

    def update_from_command(self, gcmd, cq=None):
        gcmd_filter = self._build_filter(gcmd)
        # if filters are identical, no change required
        if self._active_design == gcmd_filter:
            return
        # update MCU filter from GCode command
        design = self._active_design.design_filter(gcmd.error)
        self._sos_filter.set_filter_design(design)


# MUON: tell sensors sharing the load cell's ADC that a tap starts or ends
def send_tap_event(printer, event, load_cell_inst):
    printer.send_event(event, load_cell_inst)


# check results from the collector for errors and raise an exception is found
def check_sensor_errors(results, printer):
    samples, errors = results
    if errors:
        raise printer.command_error("Load cell sensor reported errors while"
                                    " probing: %i errors, %i overflows" % (
                                        errors[0], errors[1]))
    return samples


class LoadCellProbeConfigHelper:
    SAFETY_MODEL_REFERENCE_TARE = "reference_tare"
    SAFETY_MODEL_PRELOADED_MAX = "preloaded_max"

    def __init__(self, config, load_cell_inst):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        self._sensor = load_cell_inst.get_sensor()
        self._config_name = config.get_name()
        # Collect 4 x 60hz power cycles of data to average across power noise
        self._tare_time_param = floatParamHelper(config, 'tare_time',
            default=4. / 60., minval=0.01, maxval=1.0)
        self._safety_model = config.getchoice('safety_model', {
            self.SAFETY_MODEL_REFERENCE_TARE:
                self.SAFETY_MODEL_REFERENCE_TARE,
            self.SAFETY_MODEL_PRELOADED_MAX:
                self.SAFETY_MODEL_PRELOADED_MAX,
        }, default=self.SAFETY_MODEL_REFERENCE_TARE)
        self._reference_max_load_counts = config.getint(
            'reference_max_load_counts', default=None)
        self._max_load_safety_margin_counts_param = intParamHelper(config,
            'max_load_safety_margin_counts', default=0, minval=0,
            maxval=(1 << 30))
        self._max_load_safety_margin_percent_param = floatParamHelper(config,
            'max_load_safety_margin_percent', default=5.0, minval=0.,
            below=100.0)
        # triggering options
        self._trigger_force_param = intParamHelper(config, 'trigger_force',
            default=75, minval=10, maxval=250)
        self._trigger_counts_param = intParamHelper(config, 'trigger_counts',
            default=0, minval=0, maxval=(1 << 30))
        self._trigger_percent_param = floatParamHelper(config,
            'trigger_percent', default=None, above=0., maxval=100.0)
        # KAN-3370: the side of the tare the press moves toward is a fact
        # about the unit, never inferred from the tare.
        self._press_direction = config.getint('press_direction',
                                              default=None)
        if self._press_direction not in (None, -1, 1):
            raise config.error("press_direction must be -1 or 1")
        self._min_tare_headroom_percent = config.getfloat(
            'min_tare_headroom_percent', 20., minval=0., below=100.)
        self._force_safety_limit_param = intParamHelper(config,
            'force_safety_limit', minval=100, maxval=5000, default=2000)

    def get_tare_samples(self, gcmd=None):
        tare_time = self._tare_time_param.get(gcmd)
        sps = self._sensor.get_samples_per_second()
        return max(2, math.ceil(tare_time * sps))

    def get_safety_model(self):
        return self._safety_model

    def get_reference_max_load_counts(self):
        return self._reference_max_load_counts

    def set_reference_max_load_counts(self, max_counts):
        self._reference_max_load_counts = int(max_counts)
        cfg = self._printer.lookup_object('configfile')
        cfg.set(self._config_name, 'reference_max_load_counts',
                "%i" % (self._reference_max_load_counts,))

    # Class defaults so a helper built without __init__ (tests) is safe
    _press_direction = None
    _min_tare_headroom_percent = 20.

    def get_min_tare_headroom_percent(self):
        return self._min_tare_headroom_percent

    # The direction (-1. or 1.) that pressing on the nozzle moves the raw
    # counts.  Only for safety_model=preloaded_max.  Configured, else the
    # sign of reference_max_load_counts: the anchor is the fully loaded
    # reading, so it lies on the press side of ADC zero.
    def get_press_direction(self):
        if self._press_direction is not None:
            return float(self._press_direction)
        ref_max = self._reference_max_load_counts
        if ref_max is None:
            raise self._printer.command_error(
                "Missing reference_max_load_counts. Run "
                "LOAD_CELL_CALIBRATE_MAX first.")
        if ref_max == 0:
            raise self._printer.command_error(
                "reference_max_load_counts is 0, so the press direction "
                "is unknown: set press_direction to -1 or 1")
        return 1. if ref_max > 0 else -1.

    # Counts between the tare and the anchor, positive while the tare is
    # on the unloaded side of the anchor.  Returns
    # (press_direction, headroom_counts, headroom_percent_of_anchor)
    def get_tare_headroom(self, tare_counts):
        direction = self.get_press_direction()
        ref_max = self._reference_max_load_counts
        headroom = int(direction * (ref_max - tare_counts))
        percent = 100. * headroom / max(1, abs(ref_max))
        return direction, headroom, percent

    # KAN-3370: refuse before any motion when the nozzle is already
    # loaded.  Independent of the trigger mode and of counts_per_gram.
    # 20% default: the smallest normal headroom seen is one unit's worst
    # tare of -1750000 against an anchor of -3010747, which leaves
    # 1260747 = 42% of the anchor; another unit's normal tares leave about
    # 113% (about 75% after its known tare shift); the incident was -7.8%.
    # 20% is half the smallest normal headroom, so normal tares keep a 2x
    # margin, and a nozzle preloaded to within a fifth of the calibrated
    # maximum is refused.  The percent margin below shrinks to nothing as
    # the tare nears the anchor, hence the floor relative to abs(anchor).
    def check_tare_headroom(self, tare_counts, gcmd=None):
        if self._safety_model != self.SAFETY_MODEL_PRELOADED_MAX:
            return
        direction, headroom, percent = self.get_tare_headroom(tare_counts)
        ref_max = self._reference_max_load_counts
        margin = self.get_max_load_safety_margin_counts(tare_counts, gcmd)
        floor = int(self._min_tare_headroom_percent / 100. * abs(ref_max))
        required = max(margin, floor)
        if headroom <= required:
            raise self._printer.command_error(
                "The nozzle is already under load: check nothing is "
                "touching it (tare %i, max-load reference %i, headroom "
                "%.1f%%, need %.1f%%)"
                % (tare_counts, ref_max, percent,
                   100. * required / max(1, abs(ref_max))))

    def get_max_load_safety_margin_counts(self, tare_counts, gcmd=None):
        # Keep explicit raw-count override for compatibility and direct control.
        margin_counts = self._max_load_safety_margin_counts_param.get(gcmd)
        if margin_counts > 0:
            return margin_counts
        # Default behavior uses a percentage of tare->reference_max span.
        margin_percent = self._max_load_safety_margin_percent_param.get(gcmd)
        if margin_percent is None or margin_percent <= 0.:
            return 0
        ref_max = self._reference_max_load_counts
        if ref_max is None:
            return 0
        span_counts = abs(int(ref_max - tare_counts))
        return int(span_counts * (margin_percent / 100.0))

    def get_trigger_force_grams(self, gcmd=None):
        return self._trigger_force_param.get(gcmd)

    def get_trigger_counts(self, gcmd=None):
        trigger_counts = self._trigger_counts_param.get(gcmd)
        return trigger_counts if trigger_counts > 0 else None

    def get_trigger_percent(self, gcmd=None):
        trigger_percent = self._trigger_percent_param.get(gcmd)
        if trigger_percent is None:
            return None
        return float(trigger_percent)

    def get_safety_limit_grams(self, gcmd=None):
        return self._force_safety_limit_param.get(gcmd)

    def validate_probe_setup(self, gcmd=None):
        cmd_err = self._printer.command_error
        if self._safety_model == self.SAFETY_MODEL_REFERENCE_TARE:
            if self._load_cell.get_counts_per_gram() is None:
                raise cmd_err(
                    "Load cell probe requires counts_per_gram calibration "
                    "for safety_model=reference_tare")
            if self._load_cell.get_reference_tare_counts() is None:
                raise cmd_err(
                    "Load cell probe requires reference_tare_counts "
                    "for safety_model=reference_tare")
        elif self._safety_model == self.SAFETY_MODEL_PRELOADED_MAX:
            if self._reference_max_load_counts is None:
                raise cmd_err(
                    "Load cell probe requires reference_max_load_counts for "
                    "safety_model=preloaded_max. Set it in config or run "
                    "LOAD_CELL_CALIBRATE_MAX.")
        else:
            raise cmd_err("Unsupported load cell probe safety_model '%s'"
                          % (self._safety_model,))
        if (self.get_trigger_counts(gcmd) is None
                and self.get_trigger_percent(gcmd) is None
                and self._load_cell.get_counts_per_gram() is None):
            raise cmd_err(
                "Load cell probe requires counts_per_gram unless "
                "TRIGGER_COUNTS/TRIGGER_PERCENT "
                "(or trigger_counts/trigger_percent) is configured")

    def _get_safety_range_reference_tare(self, gcmd=None):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # calculate the safety band
        zero = self._load_cell.get_reference_tare_counts()
        safety_counts = int(counts_per_gram * self.get_safety_limit_grams(gcmd))
        safety_min = int(zero - safety_counts)
        safety_max = int(zero + safety_counts)
        return safety_min, safety_max

    def _get_safety_range_preloaded_max(self, tare_counts, gcmd=None):
        cmd_err = self._printer.command_error
        ref_max = self._reference_max_load_counts
        if ref_max is None:
            raise cmd_err(
                "Missing reference_max_load_counts. Run "
                "LOAD_CELL_CALIBRATE_MAX first.")
        self.check_tare_headroom(tare_counts, gcmd)
        margin = self.get_max_load_safety_margin_counts(tare_counts, gcmd)
        sensor_min, sensor_max = self._load_cell.get_sensor().get_range()
        if self.get_press_direction() > 0:
            # Pressing raises the counts: limit travel toward the anchor
            # and allow unrestricted drift on the other side.
            safety_min = sensor_min + 1
            safety_max = ref_max - margin
        else:
            safety_min = ref_max + margin
            safety_max = sensor_max - 1
        return int(safety_min), int(safety_max)

    def get_safety_range(self, tare_counts, gcmd=None):
        if self._safety_model == self.SAFETY_MODEL_REFERENCE_TARE:
            safety_min, safety_max = self._get_safety_range_reference_tare(gcmd)
        else:
            safety_min, safety_max = self._get_safety_range_preloaded_max(
                tare_counts, gcmd)
        # don't allow a safety range outside the sensor's real range
        sensor_min, sensor_max = self._load_cell.get_sensor().get_range()
        if safety_min <= sensor_min or safety_max >= sensor_max:
            cmd_err = self._printer.command_error
            if self._safety_model == self.SAFETY_MODEL_REFERENCE_TARE:
                raise cmd_err(
                    "Load cell force_safety_limit exceeds sensor range!")
            raise cmd_err("Load cell preloaded_max safety range exceeds "
                          "sensor range!")
        if safety_min >= safety_max:
            cmd_err = self._printer.command_error
            raise cmd_err("Load cell computed an invalid safety range")
        if tare_counts < safety_min or tare_counts > safety_max:
            cmd_err = self._printer.command_error
            raise cmd_err("Load cell tare is outside the computed safety range")
        return safety_min, safety_max

    def _get_max_delta_before_raw_range(self, tare_counts, safety_min, safety_max,
                                        trigger_type, direction=None):
        if trigger_type == "gt":
            # Directed trigger where positive values move toward safety limit.
            if direction is not None and direction < 0:
                return tare_counts - safety_min
            return safety_max - tare_counts
        # abs_ge triggers in either direction, so use the nearest limit.
        return min(tare_counts - safety_min, safety_max - tare_counts)

    def get_trigger_setup(self, tare_counts, safety_min, safety_max, gcmd=None):
        trigger_counts = self.get_trigger_counts(gcmd)
        if trigger_counts is not None:
            scale = 1.
            scale_frac_bits = 0
            trigger_type = "abs_ge"
            direction = None
            if self._safety_model == self.SAFETY_MODEL_PRELOADED_MAX:
                direction = self.get_press_direction()
                scale = direction
                trigger_type = "gt"
            max_delta = self._get_max_delta_before_raw_range(
                tare_counts, safety_min, safety_max, trigger_type, direction)
            if trigger_counts >= max_delta:
                cmd_err = self._printer.command_error
                raise cmd_err("trigger_counts (%i) is too large for the active "
                              "safety range (%i available counts)"
                              % (trigger_counts, max_delta))
            return {
                'scale': scale,
                'scale_frac_bits': scale_frac_bits,
                'trigger_type': trigger_type,
                'trigger_value': int(trigger_counts),
                'trigger_mode': 'counts',
                # for the Z load backstop: the armed delta in counts, and
                # the press direction when the trigger is directed
                'trigger_delta_counts': int(trigger_counts),
                'press_direction': direction,
            }
        trigger_percent = self.get_trigger_percent(gcmd)
        if trigger_percent is not None:
            if self._safety_model != self.SAFETY_MODEL_PRELOADED_MAX:
                cmd_err = self._printer.command_error
                raise cmd_err("trigger_percent is only supported with "
                              "safety_model=preloaded_max")
            ref_max = self._reference_max_load_counts
            direction = self.get_press_direction()
            span_counts = abs(int(ref_max - tare_counts))
            if span_counts <= 0:
                cmd_err = self._printer.command_error
                raise cmd_err("trigger_percent has zero span between "
                              "tare_counts and reference_max_load_counts")
            trigger_counts = max(
                1, int(round(span_counts * (trigger_percent / 100.0))))
            max_delta = self._get_max_delta_before_raw_range(
                tare_counts, safety_min, safety_max, "gt", direction)
            if trigger_counts >= max_delta:
                cmd_err = self._printer.command_error
                raise cmd_err("trigger_percent (%.3f%% -> %i counts) is too "
                              "large for the active safety range "
                              "(%i available counts)"
                              % (trigger_percent, trigger_counts, max_delta))
            return {
                'scale': direction,
                'scale_frac_bits': 0,
                'trigger_type': "gt",
                'trigger_value': int(trigger_counts),
                'trigger_mode': 'percent',
                'trigger_delta_counts': int(trigger_counts),
                'press_direction': direction,
            }
        # Fall back to grams-based trigger logic
        trigger_val = self.get_trigger_force_grams(gcmd)
        gpc = self.get_grams_per_count() * FRAC_GRAMS_CONV
        trigger_frac_grams = trigger_val * FRAC_GRAMS_CONV
        max_delta = self._get_max_delta_before_raw_range(
            tare_counts, safety_min, safety_max, "abs_ge")
        counts_per_gram = self._load_cell.get_counts_per_gram()
        trigger_delta_counts = int(trigger_val * counts_per_gram)
        if trigger_delta_counts >= max_delta:
            cmd_err = self._printer.command_error
            raise cmd_err("trigger_force (%ig) is too large for the active "
                          "safety range (%i available counts)"
                          % (trigger_val, max_delta))
        return {
            'scale': gpc,
            'scale_frac_bits': 14,
            'trigger_type': "abs_ge",
            'trigger_value': int(trigger_frac_grams),
            'trigger_mode': 'grams',
            # abs_ge fires in either direction, so there is no press side
            'trigger_delta_counts': abs(trigger_delta_counts),
            'press_direction': None,
        }

    # calculate 1/counts_per_gram
    def get_grams_per_count(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # The counts_per_gram could be so large that it becomes 0.0 when
        # sent to the mcu. This would mean the ADC range only measures
        # a few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= (1<<29):
            raise OverflowError("counts_per_gram value is too large to filter")
        return 1. / counts_per_gram


# KAN-3370 guard D: host-side load backstop for Z probing moves.
#
# The M1's Z homing and probe descents are blind searches: if the MCU
# trigger never fires (a wrong tare, a wrong trigger direction) the move
# runs on and pushes the bed up.  This watches the load cell samples on the
# host for the length of the descent and ends the move through
# PrinterHoming.abort_active_move() on either of two signs, whatever the
# MCU thinks:
#   (a) WRONG WAY: the load moved away from the tare on the side opposite
#       the press by more than z_move_wrong_way_factor x the trigger delta
#       (the count delta the probe's own trigger is armed with).  Only for
#       a directed trigger ("gt"); an abs_ge trigger (grams mode, the
#       reference_tare model) already fires on either side, so there is no
#       wrong side to watch.
#   (b) CEILING: |load - tare| above the ceiling on either side.  In the
#       press direction it is a backstop behind the MCU trigger and the raw
#       range; in the other it covers a sensor that has gone the wrong way.
# Either needs z_move_abort_confirm_time (20 ms) of consecutive samples, so
# one noisy sample never aborts a home.  20 ms is longer than the half
# period of the cell's 48 Hz ring (contact_guard uses the same reasoning).
# "20 ms" is both a count (ceil(20 ms x sps) samples in a row) and a span:
# the watch also fires when the run of bad samples has lasted 20 ms by
# the samples' own timestamps, so a stream with holes in it (the
# muon_ads1220 drops the slots of a temperature read) or a rate below the
# nominal one confirms as soon as it would on a clean stream.
#
# SIZING (counts; trigger_percent 10 of |anchor - tare| is the MCU trigger):
#   Boxwood, anchor -3010747: tare -578000 -> delta 243275;
#     tare -1750000 -> delta 126075.   Sloe, anchor -1585739: tare +200000
#     -> delta 178574.
#   The ceiling is 2.5 x delta: 608000 / 315000 / 446000.  The trigger
#   stops the steppers within about a millisecond of the sample that arms
#   it, so a normal tap overshoots by a small fraction of one delta and
#   never reaches 2.5 x; the MCU trigger wins by a wide margin.
#   The ceiling is relative to the armed delta, not to the anchor, on
#   purpose.  A fixed share of abs(anchor) cannot tell the 16 Nov Boxwood
#   incident (contact loads of 197994 to 289639 counts) from a normal tap
#   (243275 at tare -578000).  Relative to the delta it can: the incident
#   tare -3244819 is PAST the anchor -3010747, so its delta is 10% of the
#   234072 counts between them = 23407, and the ceiling is 58517, under a
#   third of the smallest contact load that was recorded.
#   KAN-4364 (Sloe, B1, hot): the load cell drifts 1.7-2.4M counts over a
#   heat-up (KAN-3818), and Z descents tripped the wrong-way rule at 146400
#   and 259190 counts (tares -121734 and 1006163).  The one long descent is
#   G28's first approach: up to 164 mm at 15 mm/s from the soft-homed top,
#   ~11 s.  A follower of the drift cannot tell it from a slow press, so
#   nothing is followed (z_move_drift_rate_counts defaults to 0; if set,
#   what it absorbs is clamped at 0.5 x delta).  Instead the descent runs
#   as LEGS of at most z_move_leg_time (1.0 s): each leg ends at rest,
#   tares afresh and arms the MCU trigger and these rules on that fixed
#   tare, absorbing nothing.  Sloe's worst drift is 48000 counts in 1 s,
#   0.33 of its smallest delta, so (a) heat never trips a leg, (c) the
#   trigger fires at no more than delta + 48000, and (b) a wrong-way push
#   faster than ~50000 counts/s is caught inside one leg by ~1.33 x delta.
#   Between legs check_leg() bounds what a new tare may take in:
#     - a press-side jump over z_move_leg_rate_counts x dt + 0.1 x delta
#       keeps the previous tare (light contact at a leg's end is not
#       zeroed away); two such jumps in a row end the move ("load on the
#       nozzle between Z legs");
#     - the wrong-side change since leg 1 may total at most
#       z_move_drift_total_factor x delta (4): a slower wrong-way push is
#       caught there, at worst about 4 + 1 delta;
#     - the press-side change since leg 1 may total at most
#       z_move_press_total_factor x delta (1): a soft object pressed a
#       little more each leg (under the per-leg jump limit) ends the move
#       ("load on the nozzle between Z legs") by about 2 x delta.  The
#       price: a cell drifting TOWARD the press (cooling) by more than one
#       delta over a single move ends it the same way.
#   An abort (bed removal, z_power, the column) that comes while the
#   toolhead is paused between legs is latched by a stand-in move
#   (_LegsAbortLatch) and raised before the next leg.
#   Wrong way: 1.0 x delta.  Sloe's idle wander is -4000..+7000 counts
#   against a delta of 178574, so noise is under 4% of the limit.
# MCUTARE: the MCU zeroes itself on the first descent sample and the host
#   only has _approx_tare, which note_baseline() lets move by at most
#   MAX_BASELINE_DRIFT_COUNTS (100000) a tap.  That error is most of a
#   126075-count delta, so rule (a) is OFF under MCUTARE; the ceiling stays
#   on, because 2.5 x delta minus 100000 is still 1.7 x the smallest delta.
class ZLoadBackstop:
    REASON_WRONG_WAY = "the load cell moved the wrong way during the Z move"
    REASON_CEILING = ("the load on the nozzle exceeded the safety ceiling "
                      "during the Z move")
    REASON_FAILED = "the load watch failed during the Z move"
    REASON_LEG_LOAD = "load on the nozzle between Z legs"
    MIN_CONSECUTIVE = 3
    # check_leg: a press-side tare jump may exceed rate x dt by this many
    # deltas (noise, a slightly longer pause) before it is held.
    LEG_JUMP_SLACK = 0.1
    # what a follower (z_move_drift_rate_counts > 0) may absorb
    DRIFT_CLAMP = 0.5

    def __init__(self, config, printer, load_cell):
        self._printer = printer
        self._load_cell = load_cell
        self._ceiling_factor = config.getfloat(
            'z_move_abort_ceiling_factor', 2.5, minval=0.)
        if 0. < self._ceiling_factor < 1.5:
            raise config.error("z_move_abort_ceiling_factor must be 0 (off)"
                               " or at least 1.5, so it stays above the "
                               "trigger")
        self._ceiling_counts_param = config.getint(
            'z_move_abort_ceiling_counts', 0, minval=0)
        self._wrong_way_factor = config.getfloat(
            'z_move_wrong_way_factor', 1., minval=0.)
        if 0. < self._wrong_way_factor < 0.5:
            raise config.error("z_move_wrong_way_factor must be 0 (off) or "
                               "at least 0.5")
        self._confirm_time = config.getfloat(
            'z_move_abort_confirm_time', 0.020, minval=0.002, maxval=0.2)
        # KAN-4364: the in-move follower of #147, off by default; if set
        # it absorbs at most DRIFT_CLAMP x delta
        self._drift_rate = config.getfloat(
            'z_move_drift_rate_counts', 0., minval=0.)
        # KAN-4364 legs: see above
        self._leg_time = config.getfloat('z_move_leg_time', 1.0, minval=0.)
        self._leg_rate = config.getfloat(
            'z_move_leg_rate_counts', 50000., minval=0.)
        self._total_factor = config.getfloat(
            'z_move_drift_total_factor', 4.0, minval=1.)
        self._press_total_factor = config.getfloat(
            'z_move_press_total_factor', 1.0, minval=0.2)
        # KAN-4682: legs of a home that starts from a KNOWN Z, above the
        # last few mm, may run longer than z_move_leg_time (see
        # fast_leg_time).  0 turns it off: every leg is z_move_leg_time.
        self._fast_leg_max = config.getfloat(
            'z_move_fast_leg_max', 4.0, minval=0.)
        if 0. < self._fast_leg_max < self._leg_time:
            raise config.error("z_move_fast_leg_max must be 0 (off) or at "
                               "least z_move_leg_time")
        self._fast_growth = config.getfloat(
            'z_move_fast_leg_growth', 3.0, minval=1.)
        # the share of a trigger delta a long leg may absorb as drift
        self._fast_budget = config.getfloat(
            'z_move_fast_drift_factor', 0.5, above=0., maxval=0.75)
        # the measured rate is multiplied by this (it may rise mid-leg)
        self._fast_safety = config.getfloat(
            'z_move_fast_rate_safety', 2.0, minval=1.)
        # the spread of one tare (Sloe's idle wander is -4000..+7000)
        self._fast_noise = config.getfloat(
            'z_move_fast_noise_counts', 11000., minval=0.)
        # KAN-4682 option B: before the descent the nozzle holds still this
        # long, and the drift is measured over it (0 = off, as #150)
        self._fast_rest_time = config.getfloat(
            'z_move_fast_rest_time', 0., minval=0.)
        # a long leg ends this many seconds before the ADS1220's
        # max_temperature_pause (see _temp_cap)
        self._fast_temp_margin = config.getfloat(
            'z_move_fast_temp_margin', 0.5, minval=0.)
        # KAN-4835: let the long legs above FAST_ABOVE run past the ADS1220's
        # temperature read, which the mcu then hands over with the trigger
        # blanked (a hard-capped few ms).  Off = the cap above stands.  It
        # takes effect only if the mcu firmware supports it.
        self._fast_temp_handover = config.getboolean(
            'z_move_fast_temp_handover', False)
        if self._fast_temp_handover:
            ask = getattr(self._sensor(), 'request_temp_handover', None)
            if ask is not None:
                ask()
        self._blank_seen_t = None   # the last sample time judged
        self._blank_skipped = 0     # samples ignored inside a blank window
        self._blank_resets = 0      # runs cut by a blank window
        self._rest = None           # the rest window's numbers (note_rest)
        self._rate_ref = None       # (tare, time) the drift is measured from
        self._noise_used = None     # the tare spread the plan allows for
        self._leg_first_t = None
        self._fast_prev = (0, None)     # the tare before this one, and when
        self._fast_prev_raw = None      # its change per second, no spread
        self._leg_last_time = 0.
        self._leg_times = []        # seconds each leg was planned for
        self._leg_pauses = []       # seconds between one leg and the next
        self.last_fast_plan = None
        self._leg_plans = []        # KAN-4682: each leg's fast_leg_time plan
        self._fast_above_note = 'not asked'
        self._leg_press_total = 0
        self._leg_wrong_peak = 0
        self._leg_press_peak = 0
        self._drift_clamp = 0.
        self._legs = 0
        self._leg_first_tare = None
        self._leg_first_delta = 0
        self._leg_jumps = 0
        self._leg_last_change = 0
        self._leg_total = 0
        self._armed = False
        self._registered = False    # our client is on the load cell
        self._start_time = 0.
        self._tare = 0
        self._direction = None
        self._ceiling = 0
        self._wrong = 0
        self._need = self.MIN_CONSECUTIVE
        self._over_run = 0
        self._wrong_run = 0
        self._over_t0 = None    # timestamp of the first sample of the run
        self._wrong_t0 = None
        self._drift = 0.        # wrong-side drift absorbed so far, counts
        self._drift_t = None
        self.last_fired = None
        self._armed_rules = 'none'      # of the last descent
        self._last_not_armed = None     # logging dedupe, see _note_arming
        self._last_armed_key = None

    def is_armed(self):
        return self._armed

    def get_limits(self):
        return self._ceiling, self._wrong

    # Start watching a descent.  Only samples stamped at or after
    # start_time (print_time, as the sample timestamps are) are judged.
    def arm(self, start_time, tare_counts, trigger_delta, press_direction,
            mcutare=False):
        self.disarm()
        self.last_fired = None
        sps = self._load_cell.get_sensor().get_samples_per_second()
        need = max(self.MIN_CONSECUTIVE,
                   int(math.ceil(self._confirm_time * sps)))
        ceiling = 0
        if self._ceiling_counts_param > 0:
            ceiling = self._ceiling_counts_param
        elif self._ceiling_factor > 0. and trigger_delta:
            ceiling = int(self._ceiling_factor * abs(trigger_delta))
        wrong = 0
        if (press_direction in (-1, 1) and trigger_delta
                and self._wrong_way_factor > 0. and not mcutare):
            wrong = int(self._wrong_way_factor * abs(trigger_delta))
        self._start_time = start_time
        self._tare = tare_counts
        self._direction = press_direction
        self._ceiling = ceiling
        self._wrong = wrong
        self._need = need
        self._over_run = self._wrong_run = 0
        self._over_t0 = self._wrong_t0 = None
        self._drift = 0.
        self._drift_t = None
        self._blank_seen_t = None
        self._drift_clamp = self.DRIFT_CLAMP * abs(trigger_delta or 0)
        self._note_arming(ceiling, wrong, trigger_delta, press_direction,
                          mcutare)
        if not ceiling and not wrong:
            return
        self._armed = True
        if not self._registered:
            # One client for good: the load cell drops it (False) at the
            # first batch after the watch ends, and arm() adds it again.
            self._load_cell.add_client(self._on_batch)
            self._registered = True

    def disarm(self):
        self._armed = False

    # Say in klippy.log what the descent that just started is watched for.
    # Logging only: it never changes what arms.  "Not armed" is warned once
    # per distinct reason until a descent arms again; the "armed" note is
    # written only when the rule set or a limit differs from the previous
    # descent's, so a bed mesh does not repeat either line per point.
    def _note_arming(self, ceiling, wrong, trigger_delta, press_direction,
                     mcutare):
        rules = [n for n, v in (('ceiling', ceiling),
                                ('wrong_way', wrong)) if v]
        self._armed_rules = '+'.join(rules) or 'none'
        no_ceiling = []
        no_wrong = []
        if not ceiling:
            if self._ceiling_counts_param <= 0 and not self._ceiling_factor:
                no_ceiling.append("z_move_abort_ceiling_factor is 0 and "
                                  "z_move_abort_ceiling_counts is 0")
            elif not trigger_delta:
                no_ceiling.append("no trigger delta (%r)" % (trigger_delta,))
        if not wrong:
            if mcutare:
                no_wrong.append("MCUTARE (the host tare is approximate)")
            if not self._wrong_way_factor:
                no_wrong.append("z_move_wrong_way_factor is 0")
            if press_direction not in (-1, 1):
                no_wrong.append("no press direction (%r)"
                                % (press_direction,))
            if not trigger_delta and not no_ceiling:
                no_wrong.append("no trigger delta (%r)" % (trigger_delta,))
        if not rules:
            why = "no ceiling: %s; no wrong-way: %s" % (
                ", ".join(no_ceiling) or "unknown",
                ", ".join(no_wrong) or "unknown")
            self._last_armed_key = None
            if why != self._last_not_armed:
                self._last_not_armed = why
                logging.warning("load_cell_probe: the Z load backstop is NOT"
                                " armed for this descent (%s)", why)
            return
        self._last_not_armed = None
        # The rule set only: the limits follow every tare, so keying on
        # them would log each tap of a bed mesh.  The limits of the last
        # descent are in get_status.
        key = self._armed_rules
        if key == self._last_armed_key:
            return
        self._last_armed_key = key
        logging.info(
            "load_cell_probe: Z load backstop armed (%s): ceiling %s, wrong"
            " way %s", self._armed_rules,
            "%i counts" % ceiling if ceiling else "off (%s)" % (
                ", ".join(no_ceiling) or "unknown"),
            "%i counts" % wrong if wrong else "off (%s)" % (
                ", ".join(no_wrong) or "unknown"))

    def _on_batch(self, msg):
        # Runs in the sensor's batch timer: it never raises.  False drops
        # the client from the load cell.
        if not self._armed:
            self._registered = False
            return False
        try:
            windows = self._blank_windows()
            for sample in msg['data']:
                if sample[0] < self._start_time:
                    continue
                if windows and self._in_blank(windows, sample[0]):
                    continue
                reason = self._judge(sample[2], sample[0])
                if reason is not None:
                    self._fire(reason)
                    self._registered = False
                    return False
            return True
        except Exception:
            logging.exception("load_cell_probe: the Z load watch failed")
            self._fire(self.REASON_FAILED)
            self._registered = False
            return False

    # KAN-4835: the windows the mcu blanked the trigger for (a temperature
    # read taken while armed).  No sample in one counts as wrong-way or
    # press load, and a run of samples is not carried across one: the
    # judging restarts from the first sample after it.  A real contact in
    # the window is caught by the samples that follow it.
    def _blank_windows(self):
        if not self._fast_temp_handover:
            return None
        fn = getattr(self._sensor(), 'get_blank_windows', None)
        try:
            return fn() if fn is not None else None
        except Exception:
            return None

    def _in_blank(self, windows, t):
        for w0, w1, _n in windows:
            if w0 <= t <= w1:
                self._blank_skipped += 1
                return True
        last, self._blank_seen_t = self._blank_seen_t, t
        for w0, w1, _n in windows:
            if last is not None and last < w0 and w1 < t:
                # a window lies between the last judged sample and this
                self._over_run = self._wrong_run = 0
                self._over_t0 = self._wrong_t0 = None
                self._blank_resets += 1
                break
        return False

    def _confirmed(self, run, t0, t):
        if run >= self._need:
            return True
        return (t is not None and t0 is not None
                and run >= self.MIN_CONSECUTIVE
                and t - t0 >= self._confirm_time)

    def _judge(self, counts, t=None):
        dev = counts - self._tare
        # KAN-4364: the wrong side (opposite the press) is judged against
        # the tare plus the slow drift seen so far, so a hot load cell
        # drifting for the length of a long descent never trips it.  The
        # drift is followed at no more than z_move_drift_rate_counts a
        # second and dropped at once when the load comes back, so a step
        # (a real wrong-way event) is judged at its full size.  The press
        # side keeps the raw tare: that is the push protection.
        if self._direction in (-1, 1):
            wrong_side = max(-dev * self._direction, 0.)
            if (self._drift_rate > 0. and t is not None
                    and self._drift_t is not None):
                grow = self._drift_rate * max(0., t - self._drift_t)
                self._drift = min(self._drift + grow, wrong_side,
                                  self._drift_clamp)
            else:
                self._drift = min(self._drift, wrong_side)
            if t is not None:
                self._drift_t = t
            dev += self._drift * self._direction    # 0 on the press side
        if self._ceiling and abs(dev) > self._ceiling:
            if not self._over_run:
                self._over_t0 = t
            self._over_run += 1
        else:
            self._over_run = 0
        if self._wrong and dev * self._direction < -self._wrong:
            if not self._wrong_run:
                self._wrong_t0 = t
            self._wrong_run += 1
        else:
            self._wrong_run = 0
        if self._confirmed(self._over_run, self._over_t0, t):
            return self.REASON_CEILING
        if self._confirmed(self._wrong_run, self._wrong_t0, t):
            return self.REASON_WRONG_WAY
        return None

    def get_leg_time(self):
        return self._leg_time

    # KAN-4364: a descent of legs starts at leg 1's tare and delta; the
    # cumulative limit stays in leg 1's delta (a later tare drifting the
    # wrong way widens the span to the anchor, and so the delta)
    def start_legs(self, tare_counts, trigger_delta=None, tare_time=None):
        self._legs = 1
        self._leg_first_tare = tare_counts
        self._leg_first_t = tare_time
        self._fast_prev = (tare_counts, tare_time)
        self._fast_prev_raw = None
        self._rest = None
        self._rate_ref = None
        self._noise_used = None
        self._leg_last_time = self._leg_time
        self._leg_times = []
        self._leg_pauses = []
        self.last_fast_plan = None
        self._leg_plans = []
        self._leg_first_delta = abs(trigger_delta or 0)
        self._leg_press_total = 0
        self._leg_wrong_peak = 0
        self._leg_press_peak = 0
        self._leg_jumps = 0
        self._leg_last_change = 0
        self._leg_total = 0

    # Between legs: tare is the new leg's measured tare, prev_tare the
    # last leg's, dt the seconds between them.  Returns the tare the new
    # leg uses, or raises a command error (see KAN-4364 above).
    def check_leg(self, tare, dt, prev_tare, trigger_delta, press_direction):
        self._legs += 1
        if press_direction not in (-1, 1) or not trigger_delta:
            return tare
        delta = abs(trigger_delta)
        step = tare - prev_tare
        self._leg_last_change = step
        use = tare
        allow = self._leg_rate * max(0., dt) + self.LEG_JUMP_SLACK * delta
        if step * press_direction > allow:
            self._leg_jumps += 1
            if self._leg_jumps >= 2:
                self.last_fired = self.REASON_LEG_LOAD
                logging.warning("load_cell_probe: Z move stopped: %s (leg"
                                " %d, tare moved %i toward the press, %i"
                                " allowed)", self.REASON_LEG_LOAD,
                                self._legs, step, allow)
                raise self._printer.command_error(
                    "Homing aborted: %s (the tare moved %i counts toward"
                    " the press twice in a row)"
                    % (self.REASON_LEG_LOAD, step))
            logging.info("load_cell_probe: leg %d tare %i moved %i toward"
                         " the press (%i allowed): keeping %i", self._legs,
                         tare, step, allow, prev_tare)
            use = prev_tare
        else:
            self._leg_jumps = 0
        first_delta = self._leg_first_delta or delta
        total = (use - self._leg_first_tare) * -press_direction
        self._leg_total = total
        self._leg_press_total = -total
        self._leg_wrong_peak = max(self._leg_wrong_peak, total)
        self._leg_press_peak = max(self._leg_press_peak, -total)
        press_limit = self._press_total_factor * first_delta
        if -total > press_limit:
            self.last_fired = self.REASON_LEG_LOAD
            logging.warning("load_cell_probe: Z move stopped: %s (%i counts"
                            " toward the press since the first leg, limit"
                            " %i)", self.REASON_LEG_LOAD, -total,
                            press_limit)
            raise self._printer.command_error(
                "Homing aborted: %s (%i counts toward the press since the"
                " first leg, limit %i)" % (self.REASON_LEG_LOAD, -total,
                                           press_limit))
        limit = self._total_factor * first_delta
        if total > limit:
            self.last_fired = self.REASON_WRONG_WAY
            logging.warning("load_cell_probe: Z move stopped: %s (%i counts"
                            " the wrong way since the first leg, limit %i)",
                            self.REASON_WRONG_WAY, total, limit)
            raise self._printer.command_error(
                "Homing aborted: %s (%i counts since the first leg, limit"
                " %i)" % (self.REASON_WRONG_WAY, total, limit))
        return use

    # KAN-4682: how long the next leg may run.  Only for a descent that
    # starts from a known Z, and only above the last few mm (the caller
    # decides both).  Never less than z_move_leg_time, so a hot cell keeps
    # #148's 1 s legs exactly.  The drift rate is what the tares measured
    # since leg 1: (change + one tare's spread) / elapsed, times a safety
    # factor because it may rise within the leg; the leg then absorbs at
    # most z_move_fast_drift_factor x delta of it.  A tare that was held
    # (a press-side jump) or a cell with no usable timestamps gets no
    # growth.  tare is the MEASURED tare of the leg about to start, t its
    # time.  Beyond this, the rules above stay armed: a rate that rises
    # past the plan trips the wrong-way rule (1 x delta) or fires the
    # trigger early, both of which stop the move.
    def fast_leg_time(self, tare, t, trigger_delta=None):
        base = self._leg_time
        plan = {'leg': self._legs, 'time': base, 'why': 'off'}
        self.last_fast_plan = plan
        self._leg_plans.append(plan)
        if self._fast_leg_max <= base or base <= 0.:
            return base
        delta = self._leg_first_delta or abs(trigger_delta or 0)
        if self._leg_first_t is None or t is None or not delta:
            plan['why'] = 'no first tare time'
            return base
        if self._leg_jumps:
            plan['why'] = 'press-side jump held'
            return base
        if self._rest is not None and self._rest['blocked']:
            plan['why'] = self._rest['why']
            return base
        ref_tare, ref_t = self._leg_first_tare, self._leg_first_t
        noise = self._fast_noise
        if self._rest is not None:
            # measured from the start of the rest: a longer baseline
            ref_tare, ref_t = self._rate_ref
            noise = self._noise_used
        elapsed = t - ref_t
        prev_tare, prev_t = self._fast_prev
        self._fast_prev = (tare, t)
        if elapsed < base * 0.5:
            plan['why'] = 'too soon after leg 1'
            return base
        moved = abs(tare - ref_tare)
        # the rate since leg 1 and the rate over the last leg: a cell whose
        # drift is speeding up (a hot end heating) shows in the second, and
        # the larger of the two is planned for.  A rate that has risen since
        # the leg before (1.5x and more than a tare's spread) gets no growth.
        rate = (moved + noise) / elapsed
        dt = (t - prev_t) if prev_t is not None else 0.
        if dt > 0.:
            raw = abs(tare - prev_tare) / dt
            rate = max(rate, raw + noise / dt)
            before, self._fast_prev_raw = self._fast_prev_raw, raw
            if (before is not None
                    and raw > 1.5 * before + noise / dt):
                plan['why'] = 'drift rising'
                plan['rate'] = rate
                return base
        plan['rate'] = rate
        allowed = self._fast_budget * delta / (self._fast_safety * rate)
        grown = self._leg_last_time * self._fast_growth
        use = max(base, min(self._fast_leg_max, allowed, grown))
        plan['why'] = 'drift' if use == allowed else 'cap'
        use = self._temp_limit(use, plan)
        plan['time'] = use
        return use

    def fast_enabled(self):
        return self._fast_leg_max > self._leg_time > 0.

    # KAN-4682 option B: the longest an armed leg may run so the ADS1220's
    # temperature read (muon_ads1220 max_temperature_pause, 4 s) never falls
    # inside it: that read takes the load cell, and a probe armed past it
    # can fire in mid-air.  max_temperature_pause less z_move_fast_temp_margin.
    # None when the load cell is not a muon_ads1220 (no cap).
    def _sensor(self):
        return getattr(self._load_cell, 'sensor', None)

    # KAN-4835: True when legs may run past the temperature read: asked
    # for in the config AND the mcu firmware reported it can blank
    def temp_handover(self):
        if not self._fast_temp_handover:
            return False
        fn = getattr(self._sensor(), 'get_temp_handover', None)
        try:
            return bool(fn()) if fn is not None else False
        except Exception:
            return False

    def _temp_cap(self):
        sensor = self._sensor()
        fn = getattr(sensor, 'get_max_temperature_pause', None)
        if fn is None:
            return None
        if self.temp_handover():
            return None
        try:
            pause = float(fn())
        except Exception:
            return None
        return max(0., pause - self._fast_temp_margin)

    # never shorter than z_move_leg_time (#148's legs stand)
    def _temp_limit(self, use, plan):
        if self.temp_handover():
            plan['handover'] = True
        cap = self._temp_cap()
        if cap is not None and use > cap:
            use = max(self._leg_time, cap)
            plan['why'] = 'temp cap'
            plan['cap'] = cap
        return use

    def get_rest_time(self):
        return self._fast_rest_time if self.fast_enabled() else 0.

    # Least-squares slope (counts/s) of the points, or None
    @staticmethod
    def _slope(ts, ms):
        if len(ts) < 2 or ts[-1] <= ts[0]:
            return None
        if len(ts) == 2:
            return (ms[-1] - ms[0]) / (ts[-1] - ts[0])
        return float(np.polyfit(np.asarray(ts) - ts[0], ms, 1)[0])

    # The nozzle held still for the rest: tare1 (taken at t1) began it and
    # times/counts are the samples after it.  The samples are cut into
    # tare-sized blocks (n each, counted from the end so the last block is
    # the tare leg 1 arms on) and their means, with tare1, are the points.
    # What it measures: the drift as a fitted slope, the slope over the
    # last half (a rise shows there), and the noise: how far a tare sits
    # from the fitted line.  The noise the plan allows for is twice that,
    # never above z_move_fast_noise_counts and never below a quarter of it
    # (a quiet half second does not prove the cell is quiet).
    NOISE_FLOOR = 0.25
    REST_RISE = 1.25
    def analyse_rest(self, t1, tare1, times, counts, n, trigger_delta=None,
                     press_direction=None):
        times = np.asarray(times, dtype=float)
        counts = np.asarray(counts, dtype=float)
        nb = len(counts) // max(1, n)
        if nb < 2:
            return None
        means, bt = [], []
        for k in range(nb):
            hi = len(counts) - k * n
            means.append(float(np.mean(counts[hi - n:hi])))
            bt.append(float(times[hi - 1]))
        means.reverse()
        bt.reverse()
        pts_t = [float(t1)] + bt
        pts_m = [float(tare1)] + means
        span = pts_t[-1] - pts_t[0]
        if span <= 0.:
            return None
        slope = self._slope(pts_t, pts_m)
        # the noise is what is left of the points after a smooth curve (a
        # quadratic: a drift that speeds up is not noise, it is flagged
        # below) is taken out
        x = np.asarray(pts_t) - pts_t[0]
        coef = np.polyfit(x, pts_m, 2 if len(x) >= 5 else 1)
        obs = float(np.ptp(np.asarray(pts_m) - np.polyval(coef, x)))
        mid = len(pts_t) // 2
        slope2 = self._slope(pts_t[mid:], pts_m[mid:])
        dt2 = pts_t[-1] - pts_t[mid]
        noise = min(self._fast_noise,
                    max(self.NOISE_FLOOR * self._fast_noise, 2. * obs))
        rate_fit, rate2 = abs(slope), abs(slope2)
        rising = rate2 > (self.REST_RISE * rate_fit
                          + 2. * max(obs, 500.) / dt2)
        jump = False
        if press_direction in (-1, 1) and trigger_delta:
            for i in range(1, len(pts_t)):
                allow = (self._leg_rate * (pts_t[i] - pts_t[i - 1])
                         + self.LEG_JUMP_SLACK * abs(trigger_delta))
                if (pts_m[i] - pts_m[i - 1]) * press_direction > allow:
                    jump = True
        return {'tare1': int(round(tare1)), 't1': float(t1),
                'tare2': int(round(means[-1])), 't2': pts_t[-1],
                'span': span, 'change': means[-1] - pts_m[0],
                'rate_fit': rate_fit, 'rate2': rate2, 'dt2': dt2,
                'noise_obs': obs, 'noise': noise, 'rising': bool(rising),
                'jump': jump, 'blocks': nb}

    # Called once leg 1's tare is applied (start_legs came first).  tare is
    # the one leg 1 arms on, t its time.
    def note_rest(self, info, tare, t):
        r = dict(info)
        r['blocked'] = bool(r['rising'] or r['jump'])
        r['why'] = ('press-side jump in the rest' if r['jump']
                    else 'drift rising in the rest' if r['rising'] else 'ok')
        self._rest = r
        self._rate_ref = (r['tare1'], r['t1'])
        self._noise_used = r['noise']
        self._fast_prev = (tare, t)
        self._fast_prev_raw = r['rate2']

    # Leg 1 of a descent that rested first: the same plan as the later
    # legs (0.5 x delta absorbed at 2 x the rate), from the rate the rest
    # measured (the fit over the whole window, or the last half's if that
    # is larger) plus the noise over the window it was measured in.
    def first_leg_time(self, trigger_delta=None):
        base = self._leg_time
        plan = {'leg': 1, 'time': base, 'why': 'rest off'}
        self.last_fast_plan = plan
        self._leg_plans.append(plan)
        r = self._rest
        if r is None:
            return base
        if r['blocked']:
            plan['why'] = r['why']
            return base
        delta = self._leg_first_delta or abs(trigger_delta or 0)
        if not delta:
            plan['why'] = 'no trigger delta'
            return base
        noise = r['noise']
        rate = max(r['rate_fit'] + noise / r['span'],
                   r['rate2'] + noise / r['dt2'])
        plan['rate'] = rate
        allowed = self._fast_budget * delta / (self._fast_safety * rate)
        grown = base * self._fast_growth
        use = max(base, min(self._fast_leg_max, allowed, grown))
        plan['why'] = 'rest drift' if use == allowed else 'rest cap'
        use = self._temp_limit(use, plan)
        plan['time'] = use
        return use

    def rest_note(self):
        r = self._rest
        if r is None:
            return "rest off"
        return ("rest %.2f s: tare change %i, drift %.0f counts/s (last half"
                " %.0f counts/s), noise seen %.0f used %.0f, %s"
                % (r['span'], r['change'], r['rate_fit'], r['rate2'],
                   r['noise_obs'], r['noise'], r['why']))

    # The time the leg that is about to run was given (for the next plan)
    def note_leg_time(self, seconds):
        self._leg_last_time = seconds
        self._leg_times.append(seconds)

    # KAN-4682 (bench, I-514 'not consistently'): what became of FAST_ABOVE
    # for this descent, so a stepped home can be matched to its reason
    def note_fast_above(self, note):
        self._fast_above_note = note

    def note_pause(self, seconds):
        self._leg_pauses.append(seconds)

    # One INFO line per descent of more than one leg: what the legs did
    # (bench: the cumulative changes are what the limits judge)
    def log_legs(self, how):
        if self._legs < 2:
            return
        # KAN-4682: the planned leg seconds and the pauses between legs
        # are the bench numbers for the timing before and after
        logging.info("load_cell_probe: Z legs %s: %d legs, tare change"
                     " since leg 1 %i the wrong way (peak %i) / %i toward"
                     " the press (peak %i), last leg %i; planned leg"
                     " seconds [%s], pauses %.2f s in all [%s]; FAST_ABOVE %s;"
                     " leg plans (why each leg got its time, KAN-4682)"
                     " [%s]; %s", how,
                     self._legs, self._leg_total, self._leg_wrong_peak,
                     self._leg_press_total, self._leg_press_peak,
                     self._leg_last_change,
                     ", ".join("%.2f" % (s,) for s in self._leg_times),
                     sum(self._leg_pauses),
                     ", ".join("%.2f" % (s,) for s in self._leg_pauses),
                     self._fast_above_note,
                     ", ".join("%d:%s %.2fs%s" % (
                         p.get('leg', 0), p.get('why', '?'),
                         p.get('time', 0.),
                         (" @%.0f/s" % (p['rate'],)) if 'rate' in p else "")
                         for p in self._leg_plans),
                     self.rest_note())

    def _fire(self, reason):
        self._armed = False
        try:
            homing = self._printer.lookup_object('homing')
            done = homing.abort_active_move(reason)
        except Exception:
            logging.exception("load_cell_probe: could not abort the Z move")
            return
        if done:
            self.last_fired = reason
            logging.warning("load_cell_probe: Z move aborted: %s (tare %i,"
                            " ceiling %i, wrong way %i)", reason,
                            self._tare, self._ceiling, self._wrong)
        else:
            logging.info("load_cell_probe: Z load watch tripped (%s) but no"
                         " move could be aborted", reason)

    def get_status(self):
        # The *_counts are the limits of the descent in progress (0 until
        # one arms); the rest is what is configured, readable at any time,
        # so an install can be checked with no probe having run.
        return {'z_backstop_ceiling_counts': self._ceiling,
                'z_backstop_wrong_way_counts': self._wrong,
                'z_backstop_fired': self.last_fired,
                'z_backstop_armed_rules': self._armed_rules,
                'z_backstop_ceiling_factor': self._ceiling_factor,
                'z_backstop_ceiling_counts_config':
                    self._ceiling_counts_param,
                'z_backstop_wrong_way_factor': self._wrong_way_factor,
                'z_backstop_confirm_time': self._confirm_time,
                'z_backstop_drift_rate_counts': self._drift_rate,
                'z_backstop_leg_time': self._leg_time,
                'z_backstop_leg_rate_counts': self._leg_rate,
                'z_backstop_drift_total_factor': self._total_factor,
                'z_backstop_press_total_factor': self._press_total_factor,
                'z_backstop_fast_leg_max': self._fast_leg_max,
                'z_backstop_fast_drift_factor': self._fast_budget,
                'z_backstop_fast_rest_time': self._fast_rest_time,
                'z_backstop_fast_temp_margin': self._fast_temp_margin,
                'z_backstop_fast_temp_handover_config':
                    self._fast_temp_handover,
                'z_backstop_fast_temp_handover': self.temp_handover(),
                'z_backstop_blank_skipped': self._blank_skipped,
                'z_backstop_blank_resets': self._blank_resets,
                'z_backstop_legs': self._legs,
                'z_backstop_leg_last_change': self._leg_last_change,
                'z_backstop_leg_total_change': self._leg_total,
                'z_backstop_leg_wrong_peak': self._leg_wrong_peak,
                'z_backstop_leg_press_peak': self._leg_press_peak}


# KAN-4364: PrinterHoming.abort_active_move() only ends a move that is
# running.  A probing move in legs pauses (to tare) between them, so for
# the whole sequence this stands in the list of active moves: an abort
# that lands in a pause is kept, and raised before the next leg.
class _LegsAbortLatch:
    def __init__(self):
        self.reason = None

    def involves_z(self):
        return True

    def abort(self, reason):
        if self.reason is None:
            self.reason = reason
        return True


# Execute probing moves using the MCU_trigger_analog
class LoadCellProbingMove:
    def __init__(self, config, load_cell_inst, mcu_trigger_analog, param_helper,
            continuous_tare_filter_helper, config_helper):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        self._mcu_trigger_analog = mcu_trigger_analog
        self._param_helper = param_helper
        self._continuous_tare_filter_helper = continuous_tare_filter_helper
        self._config_helper = config_helper
        self._mcu = mcu_trigger_analog.get_mcu()
        self._z_min_position = probe.lookup_minimum_z(config)
        dispatch = mcu_trigger_analog.get_dispatch()
        probe.LookupZSteppers(config, dispatch.add_stepper)
        # internal state tracking
        self._tare_counts = 0
        # last known zero, used for the raw-range envelope under MCUTARE
        self._approx_tare = None
        # KAN-4770: MCUTARE reuses that zero only while it is this recent
        self._mcutare_max_zero_age = config.getfloat(
            'mcutare_max_zero_age', 5., minval=0.)
        # Z at which the last probe actually triggered, or None when no
        # probe has triggered yet in this session.
        self._last_trigger_z = None
        # Any re-anchoring of the Z frame (G28, SET_KINEMATIC_POSITION)
        # makes the recorded trigger Z meaningless.  homing_move's own
        # set_position after a tap fires this too, before probing_move
        # records the new value, so ordinary taps are unaffected.
        self._printer.register_event_handler("toolhead:set_position",
                                             self._handle_set_position)
        self._safety_min = 0
        self._safety_max = 0
        self._trigger_mode = 'grams'
        # KAN-3370 guard D
        self._z_watch = ZLoadBackstop(config, self._printer, load_cell_inst)

    # Class defaults so a move built without __init__ (tests) is inert
    _z_watch = None
    _approx_tare_t = None
    _mcutare_max_zero_age = 5.
    _tare_time = None
    _trigger_delta = None
    _trigger_press_dir = None
    _collect_start_time = None

    def _start_collector(self):
        toolhead = self._printer.lookup_object('toolhead')
        # homing uses the toolhead last move time which gets special handling
        # to significantly buffer print_time if the move queue has drained
        _est = self._load_cell.sensor.get_mcu().estimated_print_time(
            self._printer.get_reactor().monotonic())
        # The collector only needs the print_time the descent will start at.
        # get_last_move_time() primes BUFFER_TIME_START (0.25 s) out and the
        # descent then inherits that; the drip lead is what the homing move
        # uses itself.  Never earlier than now, so a stale print_time cannot
        # widen the window back into the previous tap.
        _drip = getattr(toolhead, 'get_drip_start_time', None)
        if _drip is not None:
            print_time = max(_drip(), _est)
        else:
            print_time = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        # where the descent that follows starts, in sample time
        self._collect_start_time = print_time
        return collector

    def _log_tare(self, tare_counts):
        helper = self._config_helper
        if helper.get_safety_model() != helper.SAFETY_MODEL_PRELOADED_MAX:
            return
        try:
            direction, headroom, percent = helper.get_tare_headroom(
                tare_counts)
        except Exception as e:
            logging.info("load_cell_probe: tare=%i headroom unknown: %s",
                         tare_counts, e)
            return
        logging.info("load_cell_probe: tare=%i anchor=%i headroom=%i "
                     "(%.1f%%) press_direction=%i", tare_counts,
                     helper.get_reference_max_load_counts(), headroom,
                     percent, int(direction))

    # pauses for the last move to complete and then
    # sets the endstop tare value and range
    def _pause_and_tare(self, gcmd):
        tare, self._tare_time = self._measure_tare(gcmd)
        self._apply_tare(tare, gcmd, self._tare_time)

    # KAN-4364: the measuring half of _pause_and_tare: (tare, the time of
    # its last sample)
    def _measure_tare(self, gcmd):
        collector = self._start_collector()
        num_samples = self._config_helper.get_tare_samples(gcmd)
        # use collect_min collected samples are not wasted
        results = collector.collect_min(num_samples)
        tare_samples = check_sensor_errors(results, self._printer)
        arr = np.array(tare_samples)
        tare_counts = int(round(np.average(arr[:, 2].astype(float))))
        return tare_counts, float(arr[-1, 0])

    # KAN-4682 option B: the nozzle holds still and the cell is read for
    # `seconds` (as many samples as that takes, never fewer than two
    # tares' worth); returns the sample times and counts and the number a
    # tare averages.  The wait is the sample count: no move is queued.
    def _measure_rest(self, gcmd, seconds):
        collector = self._start_collector()
        n = self._config_helper.get_tare_samples(gcmd)
        sps = self._load_cell.sensor.get_samples_per_second()
        want = max(2 * n, int(math.ceil(seconds * sps)))
        results = collector.collect_min(want)
        samples = check_sensor_errors(results, self._printer)
        arr = np.array(samples)
        return arr[:, 0].astype(float), arr[:, 2].astype(float), n

    # ... and the half that stores it and sends it to the MCU
    def _apply_tare(self, tare_counts, gcmd, tare_time=None):
        # KAN-3370: refuse a loaded nozzle before the bad zero is stored
        # anywhere (load cell, approximate tare) or sent to the MCU
        self._log_tare(tare_counts)
        self._config_helper.check_tare_headroom(tare_counts, gcmd)
        # update sos_filter with any gcode parameter changes
        self._continuous_tare_filter_helper.update_from_command(gcmd)
        # update the load cell so it reflects the new tare value
        self._load_cell.tare(tare_counts)
        self._tare_counts = tare_counts
        self._approx_tare = tare_counts
        self._approx_tare_t = tare_time
        # update raw range
        safety_min, safety_max = self._config_helper.get_safety_range(
            tare_counts, gcmd)
        self._mcu_trigger_analog.set_raw_range(safety_min, safety_max)
        self._safety_min = safety_min
        self._safety_max = safety_max
        # choose trigger + scaling model
        trigger_setup = self._config_helper.get_trigger_setup(
            tare_counts, safety_min, safety_max, gcmd)
        # update internal tare value
        sos_filter = self._mcu_trigger_analog.get_sos_filter()
        sos_filter.set_offset_scale(
            int(-tare_counts), trigger_setup['scale'],
            trigger_setup['scale_frac_bits'])
        self._mcu_trigger_analog.set_trigger(
            trigger_setup['trigger_type'], trigger_setup['trigger_value'])
        self._trigger_mode = trigger_setup['trigger_mode']
        self._trigger_delta = trigger_setup.get('trigger_delta_counts')
        self._trigger_press_dir = trigger_setup.get('press_direction')
        logging.debug(
            "load_cell_probe: tare=%i raw_range=[%i,%i] trigger_mode=%s",
            tare_counts, safety_min, safety_max, self._trigger_mode)

    # Arm the trigger without collecting anything: the MCU takes its own
    # zero on the first sample of the descent (sos_filter.c auto_offset).
    # Nothing here waits on print_time, so the move queue does not drain
    # and the descent does not pay BUFFER_TIME_START.
    def _mcu_tare(self, gcmd):
        # The host tare this replaces was also the only pre-descent proof
        # that samples are still arriving.  A descent with a dead sample
        # stream has no trigger and no raw-range guard, so wait for one
        # sample newer than now before arming; the collector raises
        # "LoadCellSampleCollector timed out!" if none comes.
        _now = self._load_cell.sensor.get_mcu().estimated_print_time(
            self._printer.get_reactor().monotonic())
        _fresh = self._load_cell.get_collector()
        _fresh.start_collecting(min_time=_now)
        check_sensor_errors(_fresh.collect_min(1), self._printer)
        tare_counts = self._approx_tare
        # KAN-3370: note_baseline moves _approx_tare without a check, so
        # refuse a loaded nozzle before the load cell takes the zero
        self._config_helper.check_tare_headroom(tare_counts, gcmd)
        self._continuous_tare_filter_helper.update_from_command(gcmd)
        self._load_cell.tare(tare_counts)
        self._tare_counts = tare_counts
        # The raw-range guard is computed from the approximate zero but is
        # checked on the MCU against the RAW sample, before the filter, so
        # it still bounds force no matter what zero the MCU picks.
        self._log_tare(tare_counts)
        safety_min, safety_max = self._config_helper.get_safety_range(
            tare_counts, gcmd)
        self._mcu_trigger_analog.set_raw_range(safety_min, safety_max)
        self._safety_min = safety_min
        self._safety_max = safety_max
        trigger_setup = self._config_helper.get_trigger_setup(
            tare_counts, safety_min, safety_max, gcmd)
        sos_filter = self._mcu_trigger_analog.get_sos_filter()
        # offset=0 is a placeholder the MCU overwrites from its own first
        # sample. auto_offset also makes reset_filter() re-send this every
        # time, which it must: the flag is consumed after one sample.
        sos_filter.set_offset_scale(
            0, trigger_setup['scale'], trigger_setup['scale_frac_bits'],
            auto_offset=True)
        self._mcu_trigger_analog.set_trigger(
            trigger_setup['trigger_type'], trigger_setup['trigger_value'])
        self._trigger_mode = trigger_setup['trigger_mode']
        self._trigger_delta = trigger_setup.get('trigger_delta_counts')
        self._trigger_press_dir = trigger_setup.get('press_direction')

    MAX_BASELINE_DRIFT_COUNTS = 100000

    # KAN-4682: the lowest FAST_ABOVE honoured (mm)
    FAST_ABOVE_MIN = 5.

    # The MCU's auto_offset zero is RELATIVE: it becomes whatever the load
    # is when the descent arms.  Taking it while the nozzle is touching
    # absorbs the contact force into "zero".  Require this much clearance
    # above the last trigger before trusting it; below that, use the stock
    # host tare, whose zero is absolute and therefore safe in contact.
    MIN_TARE_CLEARANCE_MM = 0.2

    # Refresh the approximate zero from the pre-contact part of the tap
    # that just ran. Those samples are already collected and analysed, so
    # this costs nothing, and it tracks drift across a mesh.
    def note_baseline(self, samples):
        head = samples[:64]
        if len(head) < 8:
            return
        counts = sorted(float(s[2]) for s in head)
        candidate = int(round(counts[len(counts) // 2]))
        prev = self._approx_tare
        # KAN-4364: the bound grows with the time since the last accepted
        # baseline (z_move_leg_rate_counts a second), so a hot mesh whose
        # drift once outran it is not stuck on a stale zero for good.
        now = float(head[-1][0])
        bound = self.MAX_BASELINE_DRIFT_COUNTS
        watch = self._z_watch
        if (watch is not None and self._approx_tare_t is not None
                and now > self._approx_tare_t):
            bound = max(bound, watch._leg_rate * (now - self._approx_tare_t))
        # A tap that triggered almost immediately would put contact
        # samples in this window. A zero dragged toward the contact load
        # shrinks |ref_max - tare| and so shrinks the trigger threshold.
        # That fails toward triggering EARLY rather than into the plate,
        # but it is still wrong. Refuse an implausible jump and keep the
        # previous zero: it is still good, because the envelope and the
        # threshold are insensitive to it at this scale. Real thermal
        # drift across a mesh is orders of magnitude under this bound.
        if (prev is not None
                and abs(candidate - prev)
                > bound):
            logging.warning('load_cell_probe: rejected implausible'
                            ' baseline %i (previous %i), keeping previous',
                            candidate, prev)
            return
        self._approx_tare = candidate
        self._approx_tare_t = now

    def _handle_set_position(self):
        self._last_trigger_z = None

    def _nozzle_is_clear(self):
        # True only when the toolhead is known to be clear of the plate.
        # Unknown counts as not clear: the caller then takes the stock host
        # tare, which is slower but valid in contact.
        if self._last_trigger_z is None:
            return False
        try:
            z = self._printer.lookup_object('toolhead').get_position()[2]
        except Exception:
            return False
        if z < self._last_trigger_z + self.MIN_TARE_CLEARANCE_MM:
            logging.info('load_cell_probe: z=%.3f is within %.3f mm of the'
                         ' last trigger at %.3f; using the host tare so the'
                         ' zero is not taken under load',
                         z, self.MIN_TARE_CLEARANCE_MM, self._last_trigger_z)
            return False
        return True

    # KAN-4770: MCUTARE has the host reuse _approx_tare, the zero the last
    # tap left, as the zero the watch (guard D) arms on.  A hot cell drifts
    # far past the ceiling in the time between two meshes, so an old zero
    # must not be reused: the caller then takes a host tare, which also
    # refreshes it.  _approx_tare_t is a sample time (print_time), the same
    # clock as estimated_print_time(monotonic()).  0 turns the check off.
    def _host_zero_is_fresh(self):
        limit = self._mcutare_max_zero_age
        if limit <= 0.:
            return True
        if self._approx_tare_t is None:
            return False
        now = self._load_cell.sensor.get_mcu().estimated_print_time(
            self._printer.get_reactor().monotonic())
        age = now - self._approx_tare_t
        if age > limit:
            logging.info("load_cell_probe: MCUTARE asked for but the host "
                         "zero is %.1f s old (limit %.1f s); taking a host "
                         "tare for this tap", age, limit)
            return False
        return True

    # Probe towards z_min until the trigger_analog on the MCU triggers.
    # KAN-4364: in legs of at most z_move_leg_time each (see ZLoadBackstop):
    # a leg that ends without a trigger stops there (homing.py sets the
    # position it halted at), and the next one tares again at rest and
    # arms on that fixed tare, through check_leg.  Only the last leg may
    # report "No trigger"; anything else (an abort, a sensor error, the
    # KAN-3370 refusal, check_leg) ends the whole descent.  A tap that
    # touches inside its first leg (every tap, confirm and mesh point) is
    # one tare and one arm, as before.
    def probing_move(self, gcmd):
        self._config_helper.validate_probe_setup(gcmd)
        # tare the sensor just before probing
        _mcutare = 0 if gcmd is None else gcmd.get_int('MCUTARE', 0)
        _used_mcutare = False
        if _mcutare and self._approx_tare is not None \
                and self._nozzle_is_clear() and self._host_zero_is_fresh():
            self._mcu_tare(gcmd)
            _used_mcutare = True
            tare_t = None
        else:
            self._tare_time = None
            self._pause_and_tare(gcmd)
            tare_t = self._tare_time
        # get params for the homing move
        toolhead = self._printer.lookup_object('toolhead')
        # SETTLE=<seconds>: queued quiet time before the descent, to bound
        # how much of the travel ring the MCU can bake into its zero.  This
        # is queue time, not a host block: it does not drain the move
        # queue and so does not cost a re-prime.
        _settle = 0. if gcmd is None else gcmd.get_float('SETTLE', 0.)
        if _settle > 0.:
            toolhead.dwell(_settle)
        pos = toolhead.get_position()
        z_min = self._z_min_position
        speed = self._param_helper.get_probe_params(gcmd)['probe_speed']
        phoming = self._printer.lookup_object('homing')
        # [homing] max_search_distance_z bounds the whole descent, not
        # each leg (homing.py applies it per move)
        cap = getattr(phoming, 'max_search_distance_z', 0.) or 0.
        capped = cap > 0. and pos[2] - z_min > cap
        if capped:
            z_min = pos[2] - cap
        watch = self._z_watch
        leg_time = watch.get_leg_time() if watch is not None else 0.
        _mcu = getattr(self, '_mcu', None)
        if _mcu is not None and _mcu.is_fileoutput():
            # batch mode: every move "triggers" at its end (homing.py), so
            # legs would change where a probe lands; one descent there
            leg_time = 0.
        # KAN-4682: FAST_ABOVE=<Z> says the plate is known to be below it
        # (LOAD_CELL_HOME_Z passes it when contact_guard knows the height):
        # the legs that start above it may run longer than z_move_leg_time,
        # as fast_leg_time plans.  No leg crosses it, so the last few mm
        # (and any contact the known height predicts) are on 1 s legs.
        # Not with the MCU zero (no tare time to measure drift from).
        fast_above = None
        if (gcmd is not None and watch is not None and leg_time > 0.
                and watch.fast_enabled()
                and not _used_mcutare and tare_t is not None):
            fast_above = gcmd.get_float('FAST_ABOVE', None)
            if fast_above is not None:
                # never closer to the plate than 5 mm: the legs that find
                # it are the 1 s ones whatever a caller passes
                fast_above = max(self.FAST_ABOVE_MIN, fast_above)
        # KAN-4682 option B: a fast descent first holds still and measures
        # the drift (z_move_fast_rest_time), so leg 1 is planned from it
        # and not a fixed 1 s.  The tare the rest ends on is leg 1's.
        rest = None
        rest_secs = (watch.get_rest_time()
                     if (fast_above is not None and watch is not None)
                     else 0.)
        if rest_secs > 0.:
            tare1, t1 = self._tare_counts, tare_t
            times, counts, n = self._measure_rest(gcmd, rest_secs)
            rest = watch.analyse_rest(t1, tare1, times, counts, n,
                                      self._trigger_delta,
                                      self._trigger_press_dir)
            if rest is not None:
                # a press-side jump in the rest keeps the tare it began
                # on (as check_leg does between legs)
                use = tare1 if rest['jump'] else rest['tare2']
                self._apply_tare(use, gcmd, rest['t2'])
                tare_t = rest['t2']
        if watch is not None:
            watch.start_legs(self._tare_counts, self._trigger_delta, tare_t)
            asked = (gcmd.get_float('FAST_ABOVE', None)
                     if gcmd is not None else None)
            if fast_above is not None:
                note = "%.2f mm" % (fast_above,)
            elif asked is None:
                note = "not asked"
            elif _used_mcutare or tare_t is None:
                note = "ignored (no tare time)"
            else:
                note = "ignored (fast legs off)"
            watch.note_fast_above(note)
            if rest is not None:
                watch.note_rest(rest, self._tare_counts, tare_t)
        latch = None
        note_move = getattr(phoming, 'note_move_active', None)
        if leg_time > 0. and note_move is not None:
            latch = _LegsAbortLatch()
            note_move(latch, True)
        try:
            epos, collector = self._run_legs(
                gcmd, toolhead, phoming, watch, pos, z_min, speed, leg_time,
                capped, cap, tare_t, _used_mcutare, latch, fast_above)
        except Exception:
            if watch is not None:
                watch.log_legs("ended in an error")
            raise
        finally:
            if latch is not None:
                note_move(latch, False)
        if watch is not None:
            watch.log_legs("touched")
        # Where this tap actually stopped.  _nozzle_is_clear() needs it to
        # decide whether the next descent may take a relative MCU zero.
        self._last_trigger_z = epos[2]
        return epos, collector

    # KAN-4364: an abort latched between legs (or after the last one) is
    # raised as the move would have raised it
    def _check_latch(self, latch, collector=None):
        if latch is not None and latch.reason is not None:
            if collector is not None:
                collector.stop_collecting()
            raise self._printer.command_error(
                "Homing aborted: %s" % (latch.reason,))

    def _run_legs(self, gcmd, toolhead, phoming, watch, pos, z_min, speed,
                  leg_time, capped, cap, tare_t, _used_mcutare, latch,
                  fast_above=None):
        cmd_err = self._printer.command_error
        leg = 1
        leg_dur = leg_time
        if (leg_time > 0. and watch is not None and fast_above is not None
                and watch._rest is not None):
            leg_dur = watch.first_leg_time(self._trigger_delta)
        ended = None
        while True:
            self._check_latch(latch)
            if leg > 1:
                # KAN-4364: a fresh tare at rest, checked against the last
                prev = self._tare_counts
                tare, t = self._measure_tare(gcmd)
                dt = (t - tare_t) if tare_t is not None else leg_time
                tare_t = t
                self._apply_tare(watch.check_leg(
                    tare, dt, prev, self._trigger_delta,
                    self._trigger_press_dir), gcmd, t)
                self._check_latch(latch)
                leg_dur = leg_time
                if fast_above is not None and (
                        toolhead.get_position()[2] > fast_above + 1e-6):
                    leg_dur = watch.fast_leg_time(tare, t,
                                                  self._trigger_delta)
            z_now = toolhead.get_position()[2]
            leg_z = z_min
            if leg_time > 0.:
                leg_z = max(z_min, z_now - speed * leg_dur)
            if (fast_above is not None and z_now > fast_above + 1e-6
                    and leg_z < fast_above):
                # a long leg stops where the last few mm begin
                leg_z = max(z_min, fast_above)
            if watch is not None:
                watch.note_leg_time(abs(z_now - leg_z) / speed
                                    if speed > 0. else leg_dur)
            last = leg_z <= z_min
            pos[2] = leg_z
            # start collector after tare samples are consumed
            collector = self._start_collector()
            # KAN-3370 guard D: watch the load for the length of the leg
            if watch is not None:
                watch.arm(self._collect_start_time, self._tare_counts,
                          self._trigger_delta, self._trigger_press_dir,
                          mcutare=(_used_mcutare and leg == 1))
            # do homing move
            if ended is not None and watch is not None:
                # the pause between legs: the move stopped, the tare, the
                # arming (bench: what the leg boundaries cost)
                watch.note_pause(time.monotonic() - ended)
            try:
                epos = phoming.probing_move(self._mcu_trigger_analog, pos,
                                            speed)
            except Exception as e:
                # KAN-463: the collector has no end time and no sample
                # limit, so a failed leg must stop it here or it collects
                # for the life of klippy.
                collector.stop_collecting()
                fired = watch is not None and watch.last_fired is not None
                no_trigger = (isinstance(e, cmd_err)
                              and "No trigger on" in str(e))
                if last and capped and no_trigger and leg > 1:
                    in_homing = getattr(phoming, '_in_homing', None)
                    raise cmd_err(
                        "%s stopped: no contact within %g mm. Check the bed"
                        " is fitted and nothing is wrong with the nozzle"
                        % ("Z homing" if in_homing is not None
                           and in_homing() else "Z probing", cap))
                if last or fired or not no_trigger:
                    raise
                if toolhead.get_position()[2] >= z_now - 1e-6:
                    # no progress: never loop on a leg that did not move
                    raise
                logging.info("load_cell_probe: leg %d reached z=%.3f with"
                             " no contact (tare %i)", leg, leg_z,
                             self._tare_counts)
                leg += 1
                ended = time.monotonic()
                continue
            finally:
                # never left armed, whatever way the move ended
                if watch is not None:
                    watch.disarm()
            break
        # an abort that came after the touch: the result is not trusted
        self._check_latch(latch, collector)
        if leg > 1:
            logging.info("load_cell_probe: contact in leg %d at z=%.3f"
                         " (tare %i)", leg, epos[2], self._tare_counts)
        return epos, collector

    # Wait for the MCU to trigger with no movement
    def probing_test(self, gcmd, timeout):
        self._config_helper.validate_probe_setup(gcmd)
        send_tap_event(self._printer, "load_cell_probe:tap_begin",
                       self._load_cell)
        try:
            self._pause_and_tare(gcmd)
            toolhead = self._printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            self._mcu_trigger_analog.home_start(print_time, 0., 0, 0.)
            return self._mcu_trigger_analog.home_wait(print_time + timeout)
        finally:
            send_tap_event(self._printer, "load_cell_probe:tap_end",
                           self._load_cell)

    def get_last_trigger_time(self):
        return self._mcu_trigger_analog.get_last_trigger_time()

    def get_load_cell(self):
        return self._load_cell

    def get_status(self, eventtime):
        trig_time = self._mcu_trigger_analog.get_last_trigger_time()
        status = {
            'tare_counts': self._tare_counts,
            'raw_safety_min': self._safety_min,
            'raw_safety_max': self._safety_max,
            'trigger_mode': self._trigger_mode,
            'last_trigger_time': trig_time,
        }
        if self._z_watch is not None:
            status.update(self._z_watch.get_status())
        return status


# Perform a single complete tap
class TappingMove:
    def __init__(self, config, load_cell_probing_move, config_helper):
        self._printer = config.get_printer()
        self._load_cell_probing_move = load_cell_probing_move
        self._config_helper = config_helper
        # track results of the last tap
        self._last_result = None
        self._is_last_result_valid = False
        # webhooks support
        self._clients = load_cell.ApiClientHelper(config.get_printer())
        name = config.get_name()
        header = {"header": ["probe_tap_event"]}
        self._clients.add_mux_endpoint("load_cell_probe/dump_taps",
            "load_cell_probe", name, header)

    # perform a probing move and a pullback move
    def run_tap(self, gcmd):
        load_cell_inst = self._load_cell_probing_move.get_load_cell()
        # MUON: the window from the tare to the end of the tap's samples.
        # A sensor that shares its ADC (muon_ads1220) keeps the probe's
        # samples unbroken and the heater quiet for it.
        send_tap_event(self._printer, "load_cell_probe:tap_begin",
                       load_cell_inst)
        try:
            # do the descending move
            epos, collector = self._load_cell_probing_move.probing_move(gcmd)
            try:
                results = self._collect_tap(gcmd, collector)
            except Exception:
                # KAN-463: same leak as a failed descent.  collect_until
                # stops the collector itself when it times out; nothing
                # before it did.
                collector.stop_collecting()
                raise
        finally:
            send_tap_event(self._printer, "load_cell_probe:tap_end",
                           load_cell_inst)
        samples = check_sensor_errors(results, self._printer)
        self._load_cell_probing_move.note_baseline(samples)
        # Analyze the tap data
        ppa = TapAnalysis(samples)
        # broadcast tap event data:
        self._clients.send({'tap': ppa.to_dict()})
        self._is_last_result_valid = True
        self._last_result = epos[2]
        return epos, self._is_last_result_valid

    def _collect_tap(self, gcmd, collector):
        # collect samples from the tap
        toolhead = self._printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        # TAIL=<seconds>: bound the post-trigger collection by the tap
        # itself rather than by a re-primed print_time.  The window can only
        # ever SHORTEN relative to stock (stock's bound is est + 0.25), never
        # extend.  With TAIL the stock get_last_move_time() is not called at
        # all: its side effect is to prime print_time 0.25 s out, which the
        # retract queued next would inherit, holding the nozzle on the plate.
        # Tradeoff: check_sensor_errors then inspects a shorter
        # span, so a sensor error delivered after trig+TAIL is no longer
        # seen.  The descent, trigger, tare and raw-range guard are all
        # untouched -- this runs entirely after the probe has stopped.
        _tail = None if gcmd is None else gcmd.get_float('TAIL', None,
                                                          minval=0.)
        _trig = None
        if _tail is not None:
            _trig = self._load_cell_probing_move.get_last_trigger_time()
        if _tail is not None and _trig:
            _mcu = self._load_cell_probing_move._load_cell.sensor.get_mcu()
            _est = _mcu.estimated_print_time(
                self._printer.get_reactor().monotonic())
            move_end = min(_trig + _tail, _est + 0.25)
        else:
            move_end = toolhead.get_last_move_time()
        return collector.collect_until(move_end)

    def get_status(self, eventtime):
        return {
            'last_z_result': self._last_result,
            'is_last_tap_valid': self._is_last_result_valid
        }


# ProbeSession that implements Tap logic
class TapSession:
    def __init__(self, config, tapping_move,
                 probe_offsets, probe_params_helper):
        self._printer = config.get_printer()
        self._tapping_move = tapping_move
        self._probe_offsets = probe_offsets
        self._probe_params_helper = probe_params_helper
        # Session state
        self._results = []

    def start_probe_session(self, gcmd):
        return self

    def end_probe_session(self):
        self._results = []

    # probe until a single good sample is returned or retries are exhausted
    def run_probe(self, gcmd):
        epos, is_good = self._tapping_move.run_tap(gcmd)
        res = self._probe_offsets.create_probe_result(epos)
        self._results.append(res)

    def pull_probed_results(self):
        res = self._results
        self._results = []
        return res


class LoadCellProbeCommands:
    def __init__(self, config, load_cell_inst, load_cell_probing_move,
                 config_helper):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        self._load_cell_probing_move = load_cell_probing_move
        self._config_helper = config_helper
        self._register_commands()

    def _register_commands(self):
        # Register commands
        gcode = self._printer.lookup_object('gcode')
        gcode.register_command("LOAD_CELL_TEST_TAP",
            self.cmd_LOAD_CELL_TEST_TAP, desc=self.cmd_LOAD_CELL_TEST_TAP_help)
        gcode.register_command("LOAD_CELL_CALIBRATE_MAX",
            self.cmd_LOAD_CELL_CALIBRATE_MAX,
            desc=self.cmd_LOAD_CELL_CALIBRATE_MAX_help)

    cmd_LOAD_CELL_TEST_TAP_help = "Tap the load cell probe to verify operation"

    def cmd_LOAD_CELL_TEST_TAP(self, gcmd):
        taps = gcmd.get_int("TAPS", 3, minval=1, maxval=10)
        timeout = gcmd.get_float("TIMEOUT", 30., minval=1., maxval=120.)
        gcmd.respond_info("Tap the load cell %s times:" % (taps,))
        reactor = self._printer.get_reactor()
        for i in range(0, taps):
            result = self._load_cell_probing_move.probing_test(gcmd, timeout)
            if result == 0.:
                # notify of error, likely due to timeout
                raise gcmd.error("Test timeout out")
            gcmd.respond_info("Tap Detected!")
            # give the user some time for their finger to move away
            reactor.pause(reactor.monotonic() + 0.2)
        gcmd.respond_info("Test complete, %s taps detected" % (taps,))

    cmd_LOAD_CELL_CALIBRATE_MAX_help = (
        "Capture and save the current stable max-load raw ADC value")

    def cmd_LOAD_CELL_CALIBRATE_MAX(self, gcmd):
        default_samples = self._load_cell.get_sensor().get_samples_per_second()
        num_samples = gcmd.get_int("SAMPLES", default_samples, minval=2)
        gcmd.respond_info("Collecting %i samples for max-load calibration..."
                          % (num_samples,))
        max_counts = int(round(self._load_cell.avg_counts(num_samples)))
        self._config_helper.set_reference_max_load_counts(max_counts)
        max_pct = self._load_cell.counts_to_percent(max_counts)
        gcmd.respond_info(
            "reference_max_load_counts: %i (%.2f%%)\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer."
            % (max_counts, max_pct))


class LoadCellPrinterProbe:
    def __init__(self, config):
        cfg_error = config.error
        try:
            global np
            import numpy as np
        except:
            raise cfg_error("[load_cell_probe] requires the NumPy module")
        self._printer = config.get_printer()
        # Sensor types supported by load_cell_probe
        sensors = {}
        sensors.update(hx71x.HX71X_SENSOR_TYPES)
        sensors.update(ads1220.ADS1220_SENSOR_TYPE)
        sensors.update(muon_ads1220.MUON_ADS1220_SENSOR_TYPE)
        sensor_class = config.getchoice('sensor_type', sensors)
        sensor = sensor_class(config)
        self._load_cell = load_cell.LoadCell(config, sensor)
        # Read all user configuration and build modules
        config_helper = LoadCellProbeConfigHelper(config, self._load_cell)
        self._config_helper = config_helper
        self._mcu = self._load_cell.get_sensor().get_mcu()
        self._mcu_trigger_analog = trigger_analog.MCU_trigger_analog(sensor)
        cmd_queue = self._mcu_trigger_analog.get_dispatch().get_command_queue()
        sos_filter = trigger_analog.MCU_SosFilter(self._mcu, cmd_queue, 4)
        self._mcu_trigger_analog.setup_sos_filter(sos_filter)
        continuous_tare_filter_helper = ContinuousTareFilterHelper(
            config, sensor, sos_filter)
        # Probe Interface
        self._param_helper = probe.ProbeParameterHelper(config)
        # KAN-1730: the nozzle is the probe, so a z_offset beyond this is a
        # bad calibration (or Z_OFFSET_APPLY_PROBE run in a loop). Staging
        # one is refused; one already saved is ignored at start (Klipper
        # still starts, homing and calibration work) and printing is blocked
        # until Z is calibrated again (muon_guard reads z_offset_invalid).
        self._cmd_helper = probe.ProbeCommandHelper(
            config, self, z_limit=MAX_Z_OFFSET)
        self._probe_offsets = probe.ProbeOffsetsHelper(config, MAX_Z_OFFSET)
        self._cmd_helper.z_offset_invalid = (
            self._probe_offsets.invalid_z_offset)
        load_cell_probing_move = LoadCellProbingMove(config, self._load_cell,
            self._mcu_trigger_analog, self._param_helper,
            continuous_tare_filter_helper, config_helper)
        self._load_cell_probing_move = load_cell_probing_move
        self._tapping_move = TappingMove(config, load_cell_probing_move,
            config_helper)
        tap_session = TapSession(config, self._tapping_move,
                                 self._probe_offsets, self._param_helper)
        self._probe_session = probe.ProbeSessionHelper(config,
            self._param_helper, tap_session.start_probe_session)
        # printer integration
        LoadCellProbeCommands(config, self._load_cell, load_cell_probing_move,
                              config_helper)
        probe.ProbeVirtualEndstopDeprecation(config)
        self._printer.add_object('probe', self)
        self._printer.register_event_handler("klippy:connect",
                                             self._log_guard_settings)

    def _log_guard_settings(self):
        # One line in klippy.log with what the KAN-3370 guards are set to,
        # for checking an install with no motion (see also get_status).
        try:
            # Not get_status(): at connect the MCU may not answer yet
            st = self._load_cell_probing_move._z_watch.get_status()
            logging.info(
                "load_cell_probe: press_direction %s, min_tare_headroom %g%%"
                ", Z backstop: ceiling %g x delta (fixed %d counts, 0 = off)"
                ", wrong way %g x delta (0 = off), confirm %g s",
                self._get_status_press_direction(),
                self._config_helper.get_min_tare_headroom_percent(),
                st['z_backstop_ceiling_factor'],
                st['z_backstop_ceiling_counts_config'],
                st['z_backstop_wrong_way_factor'],
                st['z_backstop_confirm_time'])
        except Exception:
            logging.exception("load_cell_probe: guard settings unreadable")

    def get_probe_params(self, gcmd=None):
        return self._param_helper.get_probe_params(gcmd)

    def get_offsets(self, gcmd=None):
        return self._probe_offsets.get_offsets(gcmd)

    def start_probe_session(self, gcmd):
        return self._probe_session.start_probe_session(gcmd)

    # For contact_guard, which watches the same load cell between probes
    def get_load_cell(self):
        return self._load_cell

    def get_reference_max_load_counts(self):
        return self._config_helper.get_reference_max_load_counts()

    def get_last_trigger_time(self):
        return self._load_cell_probing_move.get_last_trigger_time()

    # 1. when pressing on the nozzle raises the raw counts above
    # zero_counts, -1. when it lowers them
    def get_press_direction(self, zero_counts):
        helper = self._config_helper
        ref_max = helper.get_reference_max_load_counts()
        if (helper.get_safety_model() == helper.SAFETY_MODEL_PRELOADED_MAX
                and ref_max is not None):
            # The direction the trigger uses: fixed, not inferred from
            # where the zero is.  This is called from contact_guard's
            # sample path, so an unknown direction (anchor 0, no
            # press_direction) must not raise here; probing refuses it.
            try:
                return helper.get_press_direction()
            except self._printer.command_error:
                pass
        return self._load_cell.invert

    def _get_status_press_direction(self):
        helper = self._config_helper
        if (helper.get_safety_model() != helper.SAFETY_MODEL_PRELOADED_MAX
                or helper.get_reference_max_load_counts() is None):
            return None
        try:
            return int(helper.get_press_direction())
        except Exception:
            return None

    def get_status(self, eventtime):
        status = self._cmd_helper.get_status(eventtime)
        status.update(self._load_cell.get_status(eventtime))
        status.update(self._load_cell_probing_move.get_status(eventtime))
        status.update(self._tapping_move.get_status(eventtime))
        status.update({
            # the effective probe z_offset: the stand-in (0.0) when the
            # saved one was out of range (z_offset_invalid is then set)
            'z_offset': self._probe_offsets.z_offset,
            'safety_model': self._config_helper.get_safety_model(),
            'reference_max_load_counts':
                self._config_helper.get_reference_max_load_counts(),
            'press_direction': self._get_status_press_direction(),
            'min_tare_headroom_percent':
                self._config_helper.get_min_tare_headroom_percent(),
        })
        return status


def load_config(config):
    return LoadCellPrinterProbe(config)
