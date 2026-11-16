# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
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
    MIN_CONSECUTIVE = 3

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
        self.last_fired = None

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

    def _on_batch(self, msg):
        # Runs in the sensor's batch timer: it never raises.  False drops
        # the client from the load cell.
        if not self._armed:
            self._registered = False
            return False
        try:
            for sample in msg['data']:
                if sample[0] < self._start_time:
                    continue
                reason = self._judge(sample[2])
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

    def _judge(self, counts):
        dev = counts - self._tare
        if self._ceiling and abs(dev) > self._ceiling:
            self._over_run += 1
        else:
            self._over_run = 0
        if self._wrong and dev * self._direction < -self._wrong:
            self._wrong_run += 1
        else:
            self._wrong_run = 0
        if self._over_run >= self._need:
            return self.REASON_CEILING
        if self._wrong_run >= self._need:
            return self.REASON_WRONG_WAY
        return None

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
        return {'z_backstop_ceiling_counts': self._ceiling,
                'z_backstop_wrong_way_counts': self._wrong,
                'z_backstop_fired': self.last_fired}


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
        collector = self._start_collector()
        num_samples = self._config_helper.get_tare_samples(gcmd)
        # use collect_min collected samples are not wasted
        results = collector.collect_min(num_samples)
        tare_samples = check_sensor_errors(results, self._printer)
        tare_counts = int(round(
            np.average(np.array(tare_samples)[:, 2].astype(float))))
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
                > self.MAX_BASELINE_DRIFT_COUNTS):
            logging.warning('load_cell_probe: rejected implausible'
                            ' baseline %i (previous %i), keeping previous',
                            candidate, prev)
            return
        self._approx_tare = candidate

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

    # Probe towards z_min until the trigger_analog on the MCU triggers
    def probing_move(self, gcmd):
        self._config_helper.validate_probe_setup(gcmd)
        # tare the sensor just before probing
        _mcutare = 0 if gcmd is None else gcmd.get_int('MCUTARE', 0)
        _used_mcutare = False
        if _mcutare and self._approx_tare is not None \
                and self._nozzle_is_clear():
            self._mcu_tare(gcmd)
            _used_mcutare = True
        else:
            self._pause_and_tare(gcmd)
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
        pos[2] = self._z_min_position
        speed = self._param_helper.get_probe_params(gcmd)['probe_speed']
        phoming = self._printer.lookup_object('homing')
        # start collector after tare samples are consumed
        collector = self._start_collector()
        # KAN-3370 guard D: watch the load for the length of the descent
        watch = self._z_watch
        if watch is not None:
            watch.arm(self._collect_start_time, self._tare_counts,
                      self._trigger_delta, self._trigger_press_dir,
                      mcutare=_used_mcutare)
        # do homing move
        try:
            epos = phoming.probing_move(self._mcu_trigger_analog, pos, speed)
        except Exception:
            # KAN-463: the collector has no end time and no sample limit, so
            # a failed descent ("No trigger on probe after full movement")
            # must stop it here or it collects for the life of klippy.
            collector.stop_collecting()
            raise
        finally:
            # never left armed, whatever way the move ended
            if watch is not None:
                watch.disarm()
        # Where this tap actually stopped.  _nozzle_is_clear() needs it to
        # decide whether the next descent may take a relative MCU zero.
        self._last_trigger_z = epos[2]
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
