import logging
import os
import tempfile
import pins
from . import probe, adxl345, homing, tap_onset, tap_trace, tap_confirm
from . import y_tap_log

REG_THRESH_TAP = 0x1D
REG_DUR = 0x21
REG_TAP_AXES = 0x2A
REG_INT_ENABLE = 0x2E
REG_INT_MAP = 0x2F
REG_INT_SOURCE = 0x30

DUR_SCALE = 0.000625  # 0.625 msec / LSB
TAP_SCALE = 0.0625 * adxl345.FREEFALL_ACCEL
INT_SINGLE_TAP = 0x40

TAP_AXIS_BITS = {'x': 0x01, 'y': 0x02, 'z': 0x04}
VALID_TAP_AXES = ''.join(TAP_AXIS_BITS.keys())

ADXL345_REST_TIME = .1
TAP_TEST_SAMPLE_TIME = .000015
TAP_TEST_SAMPLE_COUNT = 4
TAP_TEST_REST_TIME = .001


class _FileOutputRegs:
    # Stand-in for the adxl345 register interface when klippy runs in
    # file-output (regression test) mode and SPI reads return nothing.
    def __init__(self, mcu):
        self.mcu = mcu

    def set_reg(self, reg, val, minclock=0):
        pass

    def read_reg(self, reg, minclock=0, reqclock=0):
        return 0

    def check_connected(self):
        pass


class ADXL345TapEndstop:
    def __init__(self, owner, axis, mcu_endstop, position_endstop=None):
        self._owner = owner
        self._axis = axis
        self._mcu_endstop = mcu_endstop
        self._position_endstop = position_endstop
        self.last_trigger_time = 0.

    def get_axis(self):
        return self._axis

    def get_mcu(self):
        return self._mcu_endstop.get_mcu()

    def add_stepper(self, stepper):
        self._mcu_endstop.add_stepper(stepper)

    def get_steppers(self):
        return self._mcu_endstop.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        print_time = self._owner._adjust_home_start_print_time(
            self, print_time)
        return self._mcu_endstop.home_start(
            print_time, sample_time, sample_count, rest_time,
            triggered=triggered)

    def home_wait(self, home_end_time):
        self.last_trigger_time = self._mcu_endstop.home_wait(home_end_time)
        return self.last_trigger_time

    def query_endstop(self, print_time):
        return self._mcu_endstop.query_endstop(print_time)

    # Interface for probe.HomingViaProbeHelper (used by z wrapper)
    def multi_probe_begin(self):
        if self._axis == 'z':
            self._owner.multi_probe_begin()

    def multi_probe_end(self):
        if self._axis == 'z':
            self._owner.multi_probe_end()

    def probe_prepare(self, hmove):
        if self._axis == 'z':
            self._owner.probe_prepare(hmove)

    def probe_finish(self, hmove):
        if self._axis == 'z':
            self._owner.probe_finish(hmove)

    def get_position_endstop(self):
        if self._position_endstop is None:
            raise self._owner.printer.command_error(
                "adxl345_probe:%s_virtual_endstop does not define"
                " position_endstop" % (self._axis,))
        return self._position_endstop


class ADXL345Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.enable_z_probe = config.getboolean('enable_z_probe', False)
        self.phoming = None

        # Optional activate/deactivate scripts for z probing.
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        int_pin = config.get('int_pin').strip()
        self.inverted = False
        if int_pin.startswith('!'):
            self.inverted = True
            int_pin = int_pin[1:].strip()
        if int_pin not in ('int1', 'int2'):
            raise config.error("int_pin must be either 'int1' or 'int2'")
        self.int_map = 0x40 if int_pin == 'int2' else 0x00

        self.adxl345 = self.printer.lookup_object(config.get('chip', 'adxl345'))
        self._adxl_mcu = self.adxl345.mcu
        if self._adxl_mcu.is_fileoutput():
            self.adxl345 = _FileOutputRegs(self._adxl_mcu)
        self.probe_pin = config.get('probe_pin')
        self.disable_fans = [f.strip() for f in
                             config.get("disable_fans", "").split(",")
                             if f.strip()]
        self._saved_fans = {}

        # Configure per-axis tap profiles.
        default_tap_thresh = config.getfloat('tap_thresh', 5000.,
                                             minval=TAP_SCALE, maxval=100000.)
        default_tap_dur = config.getfloat('tap_dur', 0.01,
                                          above=DUR_SCALE, maxval=0.1)
        default_tap_axes = config.get('tap_axes', 'xyz')

        self.tap_profiles = {
            'x': self._load_tap_profile(
                config, 'x', default_tap_thresh, default_tap_dur, 'x'),
            'y': self._load_tap_profile(
                config, 'y', default_tap_thresh, default_tap_dur, 'y'),
            'z': self._load_tap_profile(
                config, 'z', default_tap_thresh, default_tap_dur,
                default_tap_axes),
        }
        # Delay interrupt arming so startup jerk does not immediately trigger.
        default_homing_arm_delay = config.getfloat(
            'homing_tap_arm_delay', 0.030, minval=0., maxval=1.0)
        self.axis_homing_arm_delay = {
            'x': config.getfloat('x_homing_tap_arm_delay',
                                 default_homing_arm_delay,
                                 minval=0., maxval=1.0),
            'y': config.getfloat('y_homing_tap_arm_delay',
                                 default_homing_arm_delay,
                                 minval=0., maxval=1.0),
            'z': config.getfloat('z_homing_tap_arm_delay',
                                 default_homing_arm_delay,
                                 minval=0., maxval=1.0),
        }

        # Optional multi-tap confirmation for X/Y homing (KAN-197).
        self.tap_confirm = {
            'x': self._load_confirm_profile(config, 'x'),
            'y': self._load_confirm_profile(config, 'y'),
        }

        # KAN-1361: take the recorded tap position from the first
        # accelerometer sample over the tap threshold, not from the time the
        # chip's tap logic raised INT.  Off by default.
        self.tap_trigger_from_samples = config.getboolean(
            'tap_trigger_from_samples', False)
        self.tap_onset_window = config.getfloat(
            'tap_onset_window', 0.06, minval=0.01, maxval=0.5)
        self._onset_client = None
        # KAN-1361 diagnostic: TAP_TRACE ENABLE=1 writes every tap of the
        # traced axes (samples, INT and onset times and positions) to a CSV
        # in the logs dir.  Off at every start; never changes homing.
        # Debug harness only (ADXL_TAP_SIMULATE): {axis: [mm, ...]}
        self._sim_taps = {}
        self._trace = None
        self._trace_axes = ''
        self._trace_pre = 0.06
        self._trace_post = 0.03
        self._trace_home = 0
        self._trace_tap = 0
        self._trace_phase = 'seek'
        self._trace_speed = None
        self._trace_seek_speed = None
        if (self.tap_trigger_from_samples
                and not self._adxl_mcu.is_fileoutput()):
            amap = getattr(self.adxl345, 'axes_map', None)
            if amap is None or [m[0] for m in amap] != [0, 1, 2]:
                raise config.error(
                    "tap_trigger_from_samples needs an identity axes_map"
                    " on the adxl345 chip")

        if self.enable_z_probe:
            self.position_endstop = config.getfloat('z_offset')
        else:
            self.position_endstop = config.getfloat('z_offset', 0.)

        # Klipper rails will ask custom endstops for position_endstop.
        # Provide axis positions from stepper config for X/Y virtual endstops.
        self.axis_position_endstops = {
            'x': self._lookup_axis_position_endstop(config, 'x'),
            'y': self._lookup_axis_position_endstop(config, 'y'),
        }

        # Setup three logical endstops on the same physical INT pin.
        share_type = "adxl345_probe:%s" % (config.get_name(),)
        ppins = self.printer.lookup_object('pins')
        self._mcu_endstops = {}
        for axis in ('x', 'y', 'z'):
            pin_params = ppins.lookup_pin(self.probe_pin, can_invert=True,
                                          can_pullup=True,
                                          share_type=share_type)
            mcu = pin_params['chip']
            self._mcu_endstops[axis] = mcu.setup_pin('endstop', pin_params)

        self._z_wrapper = ADXL345TapEndstop(
            self, 'z', self._mcu_endstops['z'], self.position_endstop)
        self._axis_wrappers = {
            'x': ADXL345TapEndstop(
                self, 'x', self._mcu_endstops['x'],
                self.axis_position_endstops['x']),
            'y': ADXL345TapEndstop(
                self, 'y', self._mcu_endstops['y'],
                self.axis_position_endstops['y']),
        }

        name_parts = config.get_name().split()
        self.base_name = name_parts[0]
        self.name = name_parts[-1]
        self.pin_chip_name = self.base_name
        if len(name_parts) > 1:
            self.pin_chip_name = "%s_%s" % (self.base_name, self.name)
        ppins.register_chip(self.pin_chip_name, self)

        self._tap_mode = None
        self._tap_was_measuring = False
        self.reactor = self.printer.get_reactor()
        self._tap_enable_pending = False
        self._tap_enable_mode = None
        self._tap_enable_minclock = 0
        self._tap_enable_timer = self.reactor.register_timer(
            self._handle_tap_enable_timer, self.reactor.NEVER)
        self._active_axis_homing_wrappers = []
        self._active_axis_homing_arm_delay = 0.
        self._active_axis_homing_arm_print_time = 0.
        self._active_axis_homing_arm_scheduled = False
        self._in_multi_probe = False
        self._adxl_initialized = False

        self._register_mux_commands(config)
        self.printer.register_event_handler('klippy:connect', self.init_adxl)
        if getattr(self._adxl_mcu, 'is_non_critical', False):
            self.printer.register_event_handler(
                self._adxl_mcu.get_non_critical_reconnect_event_name(),
                self._handle_adxl_reconnect)
            self.printer.register_event_handler(
                self._adxl_mcu.get_non_critical_disconnect_event_name(),
                self._handle_adxl_disconnect)
        self.printer.register_event_handler(
            'homing:homing_move_begin', self._handle_homing_move_begin)
        self.printer.register_event_handler(
            'homing:homing_move_end', self._handle_homing_move_end)
        self._y_tap_log = y_tap_log.YTapLog(self)
        self.printer.register_event_handler(
            'homing:homing_move_error', self._handle_y_move_error)
        self.printer.register_event_handler(
            'homing:home_rails_error', self._handle_y_rails_error)
        self.printer.register_event_handler(
            'homing:home_rails_complete', self._handle_y_rails_complete)
        self.printer.register_event_handler(
            'homing:home_rails_end', self._handle_home_rails_end)
        self.printer.register_event_handler(
            'homing:home_rails_begin', self._handle_home_rails_begin)
        self.printer.register_event_handler(
            'klippy:disconnect', self._close_trace)
        self.printer.register_event_handler(
            'gcode:command_error', self._handle_command_error)

        if self.enable_z_probe:
            # Current Klipper API requires explicit parameter + homing helpers.
            self.cmd_helper = probe.ProbeCommandHelper(
                config, self, self._z_wrapper.query_endstop)
            self.probe_offsets = probe.ProbeOffsetsHelper(config)
            self.param_helper = probe.ProbeParameterHelper(config)
            self.homing_helper = probe.HomingViaProbeHelper(
                config, self._z_wrapper, self.probe_offsets, self.param_helper)
            self.probe_session = probe.ProbeSessionHelper(
                config, self.param_helper,
                self.homing_helper.start_probe_session)
            self.printer.add_object('probe', self)
        else:
            self.cmd_helper = None
            self.probe_offsets = None
            self.param_helper = None
            self.homing_helper = None
            self.probe_session = None

    def _register_mux_commands(self, config):
        self.gcode.register_mux_command(
            "SET_ACCEL_PROBE", "CHIP", self.name, self.cmd_SET_ACCEL_PROBE,
            desc=self.cmd_SET_ACCEL_PROBE_help)
        self.gcode.register_mux_command(
            "ADXL_TEST_TAP", "CHIP", self.name, self.cmd_ADXL_TEST_TAP,
            desc=self.cmd_ADXL_TEST_TAP_help)
        if len(config.get_name().split()) == 1:
            self.gcode.register_mux_command(
                "SET_ACCEL_PROBE", "CHIP", None, self.cmd_SET_ACCEL_PROBE,
                desc=self.cmd_SET_ACCEL_PROBE_help)
            self.gcode.register_mux_command(
                "ADXL_TEST_TAP", "CHIP", None, self.cmd_ADXL_TEST_TAP,
                desc=self.cmd_ADXL_TEST_TAP_help)
            self.gcode.register_mux_command(
                "TAP_TRACE", "CHIP", None, self.cmd_TAP_TRACE,
                desc=self.cmd_TAP_TRACE_help)
        self.gcode.register_mux_command(
            "TAP_TRACE", "CHIP", self.name, self.cmd_TAP_TRACE,
            desc=self.cmd_TAP_TRACE_help)
        if self.printer.get_start_args().get('debuginput') is not None:
            # Batch tests only: there is no stop to tap in the harness.
            for key in (self.name, None):
                self.gcode.register_mux_command(
                    "ADXL_TAP_SIMULATE", "CHIP", key,
                    self.cmd_ADXL_TAP_SIMULATE)

    def _parse_tap_axes(self, tap_axes, errfunc, option_name):
        axes = tap_axes.replace(',', '').replace(' ', '').lower()
        if not axes:
            raise errfunc("%s may not be empty" % (option_name,))
        mask = 0
        for axis in axes:
            if axis not in TAP_AXIS_BITS:
                raise errfunc(
                    "%s must contain only [%s], got '%s'"
                    % (option_name, VALID_TAP_AXES, tap_axes))
            mask |= TAP_AXIS_BITS[axis]
        return mask

    def _mask_to_axes(self, mask):
        return ''.join([a for a in 'xyz' if mask & TAP_AXIS_BITS[a]])

    def _load_tap_profile(self, config, axis, default_thresh, default_dur,
                          default_tap_axes):
        tap_thresh = config.getfloat('%s_tap_thresh' % (axis,),
                                     default_thresh,
                                     minval=TAP_SCALE, maxval=100000.)
        tap_dur = config.getfloat('%s_tap_dur' % (axis,), default_dur,
                                  above=DUR_SCALE, maxval=0.1)
        tap_axes = config.get('%s_tap_axes' % (axis,), default_tap_axes)
        tap_axes_mask = self._parse_tap_axes(
            tap_axes, config.error, '%s_tap_axes' % (axis,))
        return {
            'tap_thresh': tap_thresh,
            'tap_dur': tap_dur,
            'tap_axes_mask': tap_axes_mask,
        }

    def _load_confirm_profile(self, config, axis):
        def opt(name, getter, default, **kw):
            base = getter('tap_confirm_%s' % (name,), default, **kw)
            return getter('%s_tap_confirm_%s' % (axis, name), base, **kw)
        profile = {
            'samples': opt('samples', config.getint, 1, minval=1),
            'tolerance': opt('tolerance', config.getfloat, 0.2, above=0.),
            'retries': opt('retries', config.getint, 2, minval=0),
            'retract_dist': opt('retract_dist', config.getfloat, 3.,
                                above=0.),
            'overtravel': opt('overtravel', config.getfloat, None),
            'retract_speed': opt('retract_speed', config.getfloat, None,
                                 above=0.),
            'speed': opt('speed', config.getfloat, None, above=0.),
            'seek_retries': opt('seek_retries', config.getint, 1, minval=0),
            'max_seek_dist': opt('max_seek_dist', config.getfloat, None,
                                 above=0.),
            # KAN-1361: extra retries for a tap that was driven into the
            # stop (tap_confirm.py), and the width of that class.
            'stall_retries': opt('stall_retries', config.getint, 0,
                                 minval=0),
            'stall_dist': opt('stall_dist', config.getfloat,
                              self._default_stall_dist(config, axis),
                              minval=0.),
        }
        overtravel = profile['overtravel']
        if overtravel is not None:
            if overtravel < 0.15:
                raise config.error(
                    "%s_tap_confirm_overtravel must be at least 0.15mm"
                    " to leave room for the homing deceleration distance"
                    % (axis,))
            if overtravel >= profile['retract_dist']:
                raise config.error(
                    "%s_tap_confirm_overtravel must be less than"
                    " tap_confirm_retract_dist" % (axis,))
        return profile

    def _default_stall_dist(self, config, axis):
        # One electrical cycle of the axis' stepper, in mm.
        name = 'stepper_' + axis
        if not config.has_section(name):
            return 0.
        sconfig = config.getsection(name)
        if sconfig.get('gear_ratio', None, note_valid=False) is not None:
            return 0.
        rd = sconfig.getfloat('rotation_distance', None, note_valid=False)
        if not rd:
            return 0.
        return tap_confirm.cycle_dist(rd, sconfig.getint(
            'full_steps_per_rotation', 200, note_valid=False))

    def _lookup_axis_position_endstop(self, config, axis):
        stepper_name = 'stepper_%s' % (axis,)
        if not config.has_section(stepper_name):
            raise config.error(
                "Missing [%s] for adxl345_probe:%s_virtual_endstop"
                % (stepper_name, axis))
        return config.getsection(stepper_name).getfloat('position_endstop')

    def _get_profile(self, axis):
        return self.tap_profiles[axis]

    def _set_profile_regs(self, profile, minclock=0):
        chip = self.adxl345
        chip.set_reg(REG_TAP_AXES, profile['tap_axes_mask'], minclock=minclock)
        chip.set_reg(REG_THRESH_TAP, int(profile['tap_thresh'] / TAP_SCALE),
                     minclock=minclock)
        chip.set_reg(REG_DUR, int(profile['tap_dur'] / DUR_SCALE),
                     minclock=minclock)

    def _merge_profiles(self, wrappers):
        profiles = [self._get_profile(w.get_axis()) for w in wrappers]
        tap_axes_mask = 0
        tap_thresh = TAP_SCALE
        tap_dur = DUR_SCALE
        for p in profiles:
            tap_axes_mask |= p['tap_axes_mask']
            tap_thresh = max(tap_thresh, p['tap_thresh'])
            tap_dur = max(tap_dur, p['tap_dur'])
        return {
            'tap_axes_mask': tap_axes_mask,
            'tap_thresh': tap_thresh,
            'tap_dur': tap_dur,
        }

    def _try_clear_tap(self, minclock=0):
        chip = self.adxl345
        tries = 8
        while tries > 0:
            val = chip.read_reg(REG_INT_SOURCE, minclock=minclock)
            if not (val & INT_SINGLE_TAP):
                return True
            tries -= 1
        return False

    def _reset_axis_homing_state(self):
        self._active_axis_homing_wrappers = []
        self._active_axis_homing_arm_delay = 0.
        self._active_axis_homing_arm_print_time = 0.
        self._active_axis_homing_arm_scheduled = False

    def _eventtime_from_print_time(self, print_time):
        now = self.reactor.monotonic()
        est_print_time = self._adxl_mcu.estimated_print_time(now)
        return now + max(0., print_time - est_print_time)

    def _schedule_tap_enable(self, mode, arm_print_time):
        self._tap_enable_pending = True
        self._tap_enable_mode = mode
        self._tap_enable_minclock = self._adxl_mcu.print_time_to_clock(
            arm_print_time)
        self.reactor.update_timer(
            self._tap_enable_timer,
            self._eventtime_from_print_time(arm_print_time))

    def _adjust_home_start_print_time(self, wrapper, print_time):
        if self._tap_mode != 'axis_homing':
            return print_time
        if wrapper not in self._active_axis_homing_wrappers:
            return print_time
        arm_delay = self._active_axis_homing_arm_delay
        if arm_delay <= 0.:
            return print_time
        if not self._active_axis_homing_arm_scheduled:
            arm_print_time = print_time + arm_delay
            self._active_axis_homing_arm_print_time = arm_print_time
            self._active_axis_homing_arm_scheduled = True
            self._schedule_tap_enable('axis_homing', arm_print_time)
        return self._active_axis_homing_arm_print_time

    def _cancel_tap_enable_timer(self):
        self._tap_enable_minclock = 0
        if not self._tap_enable_pending:
            return
        self._tap_enable_pending = False
        self._tap_enable_mode = None
        self.reactor.update_timer(self._tap_enable_timer, self.reactor.NEVER)

    def _handle_tap_enable_timer(self, eventtime):
        if not self._tap_enable_pending:
            return self.reactor.NEVER
        self._tap_enable_pending = False
        mode = self._tap_enable_mode
        minclock = self._tap_enable_minclock
        self._tap_enable_mode = None
        self._tap_enable_minclock = 0
        if self._tap_mode != mode:
            return self.reactor.NEVER
        chip = self.adxl345
        try:
            # Drop startup motion events that happened during arm delay.
            chip.read_reg(REG_INT_SOURCE, minclock=minclock)
            if not self._try_clear_tap(minclock=minclock):
                raise self.printer.command_error(
                    "ADXL345 tap active before delayed arm")
            chip.set_reg(REG_INT_ENABLE, INT_SINGLE_TAP, minclock=minclock)
        except Exception as e:
            logging.exception("Failed delayed ADXL345 tap arm")
            self.printer.invoke_shutdown(
                "ADXL345 delayed tap arm failed: %s" % (str(e),))
        return self.reactor.NEVER

    def _start_tap(self, profile, mode, arm_delay=0., defer_arm=False):
        if getattr(self, '_stream_stop_error', None):
            raise self.printer.command_error(self._stream_stop_error)
        if self._tap_mode is not None:
            raise self.printer.command_error(
                "ADXL345 tap detection already active (%s)" % (self._tap_mode,))
        chip = self.adxl345
        chip.check_connected()
        toolhead = self.printer.lookup_object('toolhead')
        was_measuring = True
        try:
            toolhead.flush_step_generation()
            toolhead.dwell(ADXL345_REST_TIME)
            print_time = toolhead.get_last_move_time()
            clock = chip.mcu.print_time_to_clock(print_time)
            chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
            chip.read_reg(REG_INT_SOURCE)
            self._set_profile_regs(profile, minclock=clock)
            was_measuring = chip.read_reg(adxl345.REG_POWER_CTL) == 0x08
            if not was_measuring:
                chip.set_reg(adxl345.REG_POWER_CTL, 0x08, minclock=clock)
            if not self._try_clear_tap():
                raise self.printer.command_error(
                    "ADXL345 tap triggered before move, reduce sensitivity.")
            self._cancel_tap_enable_timer()
            if defer_arm:
                pass
            elif arm_delay > 0.:
                self._tap_enable_pending = True
                self._tap_enable_mode = mode
                self.reactor.update_timer(self._tap_enable_timer,
                                          self.reactor.monotonic() + arm_delay)
            else:
                chip.set_reg(REG_INT_ENABLE, INT_SINGLE_TAP, minclock=clock)
        except Exception:
            self._cancel_tap_enable_timer()
            try:
                chip.set_reg(REG_INT_ENABLE, 0x00)
            except Exception:
                logging.exception("Unable to disable ADXL345 interrupt")
            if not was_measuring:
                try:
                    chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
                except Exception:
                    logging.exception("Unable to restore ADXL345 power state")
            raise
        self._tap_was_measuring = was_measuring
        self._tap_mode = mode

    def _stop_tap(self, mode):
        if self._tap_mode is None:
            return
        if mode is not None and self._tap_mode != mode:
            return
        chip = self.adxl345
        toolhead = self.printer.lookup_object('toolhead')
        try:
            self._cancel_tap_enable_timer()
            toolhead.dwell(ADXL345_REST_TIME)
            print_time = toolhead.get_last_move_time()
            clock = chip.mcu.print_time_to_clock(print_time)
            chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
            if not self._tap_was_measuring:
                chip.set_reg(adxl345.REG_POWER_CTL, 0x00, minclock=clock)
            if not self._try_clear_tap():
                raise self.printer.command_error(
                    "ADXL345 tap still active after move, reduce sensitivity.")
        finally:
            self._tap_mode = None
            self._tap_was_measuring = False

    def _set_fan_disabled(self, disable):
        if not self.disable_fans:
            return
        eventtime = self.printer.get_reactor().monotonic()
        if disable:
            for fan_name in self.disable_fans:
                if fan_name in self._saved_fans:
                    continue
                fan_obj = self.printer.lookup_object(fan_name, None)
                if fan_obj is None:
                    logging.warning("Unknown fan '%s' in disable_fans",
                                    fan_name)
                    continue
                if hasattr(fan_obj, 'fan_speed'):
                    self._saved_fans[fan_name] = ('fan_speed',
                                                  fan_obj.fan_speed)
                    fan_obj.fan_speed = 0.
                    if hasattr(fan_obj, 'fan'):
                        fan_obj.fan.set_speed(0.)
                elif hasattr(fan_obj, 'fan'):
                    prev_speed = fan_obj.fan.get_status(eventtime)['speed']
                    self._saved_fans[fan_name] = ('fan', prev_speed)
                    fan_obj.fan.set_speed(0.)
                else:
                    logging.warning("disable_fans object '%s' is unsupported",
                                    fan_name)
        else:
            for fan_name, (stype, sval) in list(self._saved_fans.items()):
                fan_obj = self.printer.lookup_object(fan_name, None)
                if fan_obj is None:
                    continue
                if stype == 'fan_speed' and hasattr(fan_obj, 'fan_speed'):
                    fan_obj.fan_speed = sval
                    if hasattr(fan_obj, 'fan'):
                        fan_obj.fan.set_speed(sval)
                elif stype == 'fan' and hasattr(fan_obj, 'fan'):
                    fan_obj.fan.set_speed(sval)
            self._saved_fans.clear()

    def _lookup_axis_wrapper(self, axis, gcmd=None):
        axis = axis.lower()
        if axis in self._axis_wrappers:
            return self._axis_wrappers[axis]
        if axis == 'z':
            if not self.enable_z_probe:
                if gcmd is not None:
                    raise gcmd.error("AXIS=Z requires enable_z_probe: True")
                raise self.printer.command_error(
                    "AXIS=Z requires enable_z_probe: True")
            return self._z_wrapper
        if gcmd is not None:
            raise gcmd.error("AXIS must be X, Y, or Z")
        raise self.printer.command_error("AXIS must be X, Y, or Z")

    def _wait_for_tap(self, wrapper, timeout):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        wrapper.home_start(print_time, TAP_TEST_SAMPLE_TIME,
                           TAP_TEST_SAMPLE_COUNT, TAP_TEST_REST_TIME,
                           triggered=True)
        trigger_time = wrapper.home_wait(print_time + timeout)
        return trigger_time > 0.

    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop':
            raise pins.error("adxl345_probe virtual pins are only endstops")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("adxl345_probe virtual pins may not invert/pullup")
        pin_name = pin_params['pin']
        if pin_name == 'x_virtual_endstop':
            return self._axis_wrappers['x']
        if pin_name == 'y_virtual_endstop':
            return self._axis_wrappers['y']
        if pin_name == 'z_virtual_endstop':
            if not self.enable_z_probe:
                raise pins.error("enable_z_probe must be true for z_virtual_endstop")
            return self._z_wrapper
        raise pins.error(
            "Unknown adxl345_probe virtual pin '%s' (expected"
            " x_virtual_endstop, y_virtual_endstop, or z_virtual_endstop)"
            % (pin_name,))

    def _handle_homing_move_begin(self, hmove):
        wrappers = [w for w in self._axis_wrappers.values()
                    if w in hmove.get_mcu_endstops()]
        if not wrappers:
            return
        if any(w.get_axis() == 'y' for w in wrappers):
            self._y_tap_log.start(hmove)
        profile = self._merge_profiles(wrappers)
        arm_delay = max([self.axis_homing_arm_delay.get(w.get_axis(), 0.)
                         for w in wrappers])
        self._active_axis_homing_wrappers = wrappers
        self._active_axis_homing_arm_delay = arm_delay
        self._active_axis_homing_arm_print_time = 0.
        self._active_axis_homing_arm_scheduled = False
        self._trace_speed = getattr(hmove, 'muon_speed',
                                    self._trace_seek_speed)
        self._start_onset_client(wrappers, profile)
        self._start_tap(profile, 'axis_homing', defer_arm=arm_delay > 0.)

    def _start_onset_client(self, wrappers, profile):
        # Stream accelerometer samples during the move so the tap onset can
        # be located afterwards.  The MCU stop still comes from the INT pin.
        if getattr(self, '_stream_stop_error', None):
            self._release_onset_client()
        self._onset_client = None
        if len(wrappers) != 1:
            return
        axis = wrappers[0].get_axis()
        if not (self.tap_trigger_from_samples or self._tracing(axis)):
            return
        if axis not in ('x', 'y') or self._mask_to_axes(
                profile['tap_axes_mask']) != axis:
            return
        if self._adxl_mcu.is_fileoutput():
            return
        try:
            self._onset_client = self.adxl345.start_internal_client()
            if self.inverted:
                # adxl345 measurement start rewrites DATA_FORMAT
                self.adxl345.set_reg(adxl345.REG_DATA_FORMAT, 0x2B)
        except Exception:
            logging.exception("adxl345_probe: tap onset sampling unavailable")
            self._release_onset_client()

    def _release_onset_client(self):
        client = self._onset_client
        if client is None:
            return
        client.is_finished = True
        # The bulk reader drops a finished client, and puts the chip in
        # standby, from its batch timer.  Wait for that here so its
        # register I/O cannot interleave with ours ("Unable to obtain
        # 'spi_transfer_response'", 9 Nov) or blind the next tap.
        bulk = getattr(self.adxl345, 'batch_bulk', None)
        if bulk is None:
            self._onset_client = None
            return
        reactor = self.reactor
        deadline = reactor.monotonic() + 1.
        while client.handle_batch in bulk.client_cbs or bulk.is_started:
            now = reactor.monotonic()
            if now >= deadline:
                axes = ''.join(w.get_axis() for w in getattr(
                    self, '_active_axis_homing_wrappers', [])) or 'tap'
                self._stream_stop_error = (
                    "adxl345_probe: %s home aborted: accelerometer stream"
                    " did not stop within 1.0s; a tap armed now could be"
                    " blind and drive into the stop. Re-run G28; if it"
                    " repeats, check other ADXL clients (TAP_TRACE,"
                    " resonance tests)" % (axes,))
                raise self.printer.command_error(self._stream_stop_error)
            reactor.pause(now + 0.01)
        self._onset_client = None
        self._stream_stop_error = None

    def _apply_tap_onset(self, hmove, wrappers):
        # Replace the stepper trigger positions with those at the onset of
        # the tap (tap_trigger_from_samples), and write the tap to the trace
        # when TAP_TRACE is on. Refinement failures keep the hardware
        # trigger positions; stream-stop failures must abort the home.
        client = self._onset_client
        if client is None:
            return
        try:
            if len(wrappers) != 1:
                return
            wrapper = wrappers[0]
            axis = wrapper.get_axis()
            tracing = self._tracing(axis)
            trigger_time = wrapper.last_trigger_time
            names = [n for es, n in hmove.endstops if es is wrapper]
            sps = [sp for sp in hmove.stepper_positions
                   if sp.endstop_name in names]
            if trigger_time <= 0.:
                if tracing:
                    self._trace_write(hmove, sps, axis, None, None, [],
                                      'int')
                return
            wait_to = trigger_time + (self._trace_post if tracing else 0.)
            reactor = self.reactor
            deadline = reactor.monotonic() + 0.6
            while not self._samples_cover(client, wait_to):
                now = reactor.monotonic()
                if now >= deadline:
                    logging.warning("adxl345_probe: %s tap onset: samples"
                                    " not received in time", axis)
                    break
                reactor.pause(now + 0.02)
            samples = [smp for msg in client.msgs for smp in msg['data']]
            thresh = self._get_profile(axis)['tap_thresh']
            onset = None
            if samples and samples[-1][0] >= trigger_time:
                onset = tap_onset.find_tap_onset(
                    samples, axis, thresh, trigger_time,
                    self.tap_onset_window)
            int_steps = dict((sp.stepper_name, sp.trig_pos) for sp in sps)
            applied = 'int'
            if self.tap_trigger_from_samples:
                before = [sp.trig_pos for sp in sps]
                if tap_onset.refine_stepper_positions(sps, onset,
                                                      trigger_time):
                    applied = 'onset'
                    logging.info(
                        "adxl345_probe: %s tap onset %.6f, INT %.6f (%.2fms"
                        " later); trig_pos %s -> %s", axis, onset,
                        trigger_time, (trigger_time - onset) * 1000.,
                        before, [sp.trig_pos for sp in sps])
                else:
                    logging.info("adxl345_probe: %s tap onset not found;"
                                 " keeping the INT trigger time", axis)
            if tracing:
                self._trace_write(hmove, sps, axis, trigger_time, onset,
                                  samples, applied, int_steps)
        except Exception:
            logging.exception("adxl345_probe: tap onset refinement failed")
        finally:
            self._release_onset_client()

    # TAP_TRACE (KAN-1361 diagnostic)
    def _tracing(self, axis):
        return self._trace is not None and axis in self._trace_axes

    def _axis_pos_at(self, axis, print_time):
        # Axis position from the raw MCU step counts at print_time, so taps
        # compare across homing's re-basing of the toolhead position.
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        steps = {}
        pos = {}
        for s in kin.get_steppers():
            n = s.get_name()
            steps[n] = s.get_past_mcu_position(print_time)
            pos[n] = steps[n] * s.get_step_dist()
        return kin.calc_position(pos)['xyz'.index(axis)], steps

    def _axis_pos_from(self, axis, sps, attr):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        by_name = dict((sp.stepper_name, getattr(sp, attr)) for sp in sps)
        if any(v is None for v in by_name.values()):
            return None, None
        pos = {}
        for s in kin.get_steppers():
            n = s.get_name()
            if n in by_name:
                pos[n] = by_name[n] * s.get_step_dist()
            else:
                pos[n] = s.get_mcu_position() * s.get_step_dist()
        return kin.calc_position(pos)['xyz'.index(axis)], by_name

    def _trace_write(self, hmove, sps, axis, int_time, onset, samples,
                     applied, int_steps=None):
        writer = self._trace
        if writer is None:
            return
        try:
            self._trace_tap += 1
            profile = self._get_profile(axis)
            tap = {
                'home': self._trace_home, 'tap': self._trace_tap,
                'phase': self._trace_phase, 'axis': axis,
                'speed': self._trace_speed, 'applied': applied,
                'int_time': int_time, 'onset_time': onset,
                'thresh': profile['tap_thresh'],
                'tap_axes': self._mask_to_axes(profile['tap_axes_mask']),
            }
            tap['start_pos_mm'] = self._axis_pos_from(
                axis, sps, 'start_pos')[0]
            tap['halt_pos_mm'], tap['halt_steps'] = self._axis_pos_from(
                axis, sps, 'halt_pos')
            if int_time is not None:
                tap['int_pos_mm'], all_int = self._axis_pos_at(axis, int_time)
                tap['int_steps'] = int_steps or all_int
            if onset is not None:
                tap['onset_pos_mm'], tap['onset_steps'] = self._axis_pos_at(
                    axis, onset)
            try:
                self._trace_steppers(tap, sps, int_steps)
            except Exception:
                logging.exception("adxl345_probe: TAP_TRACE stepper fields"
                                  " unavailable")
            writer.write_tap(tap, samples)
            logging.info(
                "adxl345_probe: TAP_TRACE %s home %d tap %d (%s): INT %s,"
                " onset %s, %s pos %s -> %s", axis, self._trace_home,
                self._trace_tap, self._trace_phase, int_time, onset,
                applied, tap.get('onset_pos_mm'), tap.get('int_pos_mm'))
        except Exception:
            logging.exception("adxl345_probe: TAP_TRACE write failed")

    def _trace_steppers(self, tap, sps, int_steps):
        # Per stepper: the distance run to the trigger, the commanded
        # follow-through after it, the microstep phase of the step count
        # at the start (after the retract), the trigger and the halt, and
        # the driver's own MSCNT read back at the halt.  MSCNT counts the
        # steps the driver took, not where the rotor is: it shows lost
        # step pulses, never a pole slip.
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        dists = dict((s.get_name(), s.get_step_dist())
                     for s in kin.get_steppers())
        travelled, follow = {}, {}
        phases = {'start': {}, 'int': {}, 'halt': {}}
        mscnt = {}
        for sp in sps:
            name = sp.stepper_name
            trig = (int_steps or {}).get(name, sp.trig_pos)
            sd = dists.get(name, 0.)
            if None in (sp.start_pos, sp.halt_pos, trig):
                continue
            travelled[name] = (trig - sp.start_pos) * sd
            follow[name] = (sp.halt_pos - trig) * sd
            drv = self._lookup_tmc(name)
            if drv is None:
                continue
            offset, nphases = drv.get_phase_offset()
            if offset is None:
                offset = 0
            for what, pos in (('start', sp.start_pos), ('int', trig),
                              ('halt', sp.halt_pos)):
                phases[what][name] = tap_confirm.phase(pos, nphases, offset)
            try:
                reg = drv.fields.lookup_register('mscnt', None)
                if reg is not None:
                    mscnt[name] = drv.fields.get_field(
                        'mscnt', drv.mcu_tmc.get_register(reg))
            except Exception:
                logging.exception("adxl345_probe: MSCNT read failed")
        tap['travelled_mm'] = travelled
        tap['follow_mm'] = follow
        for what in phases:
            tap[what + '_phase'] = phases[what]
        tap['halt_mscnt'] = mscnt

    def _lookup_tmc(self, stepper_name):
        for name, obj in self.printer.lookup_objects():
            if (name.startswith('tmc') and name.endswith(' ' + stepper_name)
                    and hasattr(obj, 'get_phase_offset')):
                return obj
        return None

    def _handle_home_rails_begin(self, homing_state, rails):
        if any(es is self._axis_wrappers['y']
               for rail in rails for es, name in rail.get_endstops()):
            self._y_tap_log.begin()
        if self._trace is None:
            return
        for rail in rails:
            for es, name in rail.get_endstops():
                if es in self._axis_wrappers.values() and self._tracing(
                        es.get_axis()):
                    self._trace_home += 1
                    self._trace_tap = 0
                    self._trace_phase = 'seek'
                    self._trace_seek_speed = rail.get_homing_info().speed
                    return

    def _trace_dir(self):
        log_file = self.printer.get_start_args().get('log_file')
        if log_file:
            return os.path.dirname(os.path.abspath(log_file))
        return tempfile.gettempdir()

    def _close_trace(self):
        writer, self._trace = self._trace, None
        if writer is not None:
            try:
                writer.close()
            except Exception:
                logging.exception("adxl345_probe: TAP_TRACE close failed")
        return writer

    def cmd_ADXL_TAP_SIMULATE(self, gcmd):
        # ADXL_TAP_SIMULATE AXIS=y OFFSETS=0.59,0.8: where the
        # confirm taps of the next home of AXIS trigger, in mm towards the
        # stop from its first tap.  Taps past the list repeat the stop (0).
        axis = gcmd.get('AXIS').strip().lower()
        if axis not in ('x', 'y'):
            raise gcmd.error("ADXL_TAP_SIMULATE AXIS must be x or y")
        try:
            offsets = [float(v) for v in gcmd.get('OFFSETS').split(',')]
        except ValueError:
            raise gcmd.error("ADXL_TAP_SIMULATE OFFSETS must be numbers")
        self._sim_taps[axis] = offsets

    cmd_TAP_TRACE_help = ("KAN-1361 diagnostic: ENABLE=1 records every X/Y"
                          " tap (samples, INT/onset times and positions) to"
                          " a CSV in the logs dir; ENABLE=0 closes it")

    def cmd_TAP_TRACE(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        if enable is None:
            if self._trace is None:
                gcmd.respond_info("TAP_TRACE is off")
            else:
                gcmd.respond_info(
                    "TAP_TRACE is on for %s: %d taps in %s"
                    % (self._trace_axes, self._trace.taps, self._trace.path))
            return
        if not enable:
            writer = self._close_trace()
            if writer is None:
                gcmd.respond_info("TAP_TRACE was already off")
            else:
                gcmd.respond_info("TAP_TRACE off: %d taps in %s"
                                  % (writer.taps, writer.path))
            return
        axes = gcmd.get('AXES', 'y').strip().lower()
        if not axes or any(a not in 'xy' for a in axes):
            raise gcmd.error("TAP_TRACE AXES must be x, y or xy")
        pre = gcmd.get_float('PRE', 0.06, minval=0.005, maxval=0.5)
        post = gcmd.get_float('POST', 0.03, minval=0., maxval=0.2)
        if self._adxl_mcu.is_fileoutput():
            raise gcmd.error("TAP_TRACE needs a live accelerometer")
        amap = getattr(self.adxl345, 'axes_map', None)
        if amap is None or [m[0] for m in amap] != [0, 1, 2]:
            raise gcmd.error("TAP_TRACE needs an identity axes_map on the"
                             " adxl345 chip")
        if self._trace is not None:
            if (axes, pre, post) == (self._trace_axes, self._trace_pre,
                                     self._trace_post):
                gcmd.respond_info("TAP_TRACE already on: %s"
                                  % (self._trace.path,))
                return
            self._close_trace()
        path = tap_trace.trace_path(self._trace_dir())
        try:
            writer = tap_trace.TapTraceWriter(path, pre, post)
        except (IOError, OSError) as e:
            raise gcmd.error("TAP_TRACE cannot write %s: %s" % (path, e))
        self._trace = writer
        self._trace_axes = axes
        self._trace_pre = pre
        self._trace_post = post
        self._trace_home = 0
        self._trace_tap = 0
        self._trace_phase = 'seek'
        gcmd.respond_info(
            "TAP_TRACE on for %s (%.0f ms before INT, %.0f ms after): %s."
            " Streams the accelerometer during traced taps, which sets the"
            " chip to its streaming rate." % (axes, pre * 1000.,
                                              post * 1000., path))

    @staticmethod
    def _samples_cover(client, trigger_time):
        for msg in reversed(client.msgs):
            if msg['data']:
                return msg['data'][-1][0] >= trigger_time
        return False

    def _handle_homing_move_end(self, hmove):
        if not self._active_axis_homing_wrappers:
            return
        hmove_endstops = hmove.get_mcu_endstops()
        if not any(w in hmove_endstops for w in self._active_axis_homing_wrappers):
            return
        wrappers = list(self._active_axis_homing_wrappers)
        hmove.y_tap_raw_steps = {
            stage: {sp.stepper_name: getattr(sp, attr)
                    for sp in hmove.stepper_positions}
            for stage, attr in (('start', 'start_pos'),
                                ('trigger', 'trig_pos'), ('halt', 'halt_pos'))}
        error = getattr(hmove, 'move_error', None)
        try:
            try:
                self._stop_tap('axis_homing')
            finally:
                try:
                    self._apply_tap_onset(hmove, wrappers)
                finally:
                    try:
                        self._release_onset_client()
                    finally:
                        self._reset_axis_homing_state()
        except Exception as e:
            error = e
            raise
        finally:
            if any(w.get_axis() == 'y' for w in wrappers):
                self._y_tap_log.record(hmove, error)

    def _handle_y_move_error(self, hmove, error):
        if not self._y_tap_log.active:
            return ''
        self._y_tap_log.record(hmove, error, halted=False)
        return self._y_tap_log.finish('failed', error)

    def _handle_y_rails_error(self, homing_state, rails, error):
        return self._y_tap_log.finish('failed', error)

    def _handle_y_rails_complete(self, homing_state, rails):
        self._y_tap_log.finish('accepted')

    def _handle_home_rails_end(self, homing_state, rails):
        # Confirm an X/Y tap home with further agreeing taps before the
        # position is trusted (KAN-197).
        matches = []
        for rail in rails:
            for es, name in rail.get_endstops():
                if es in self._axis_wrappers.values():
                    matches.append((rail, es))
        if not matches:
            return
        if len(matches) > 1:
            logging.warning("adxl345_probe: tap confirmation skipped for"
                            " simultaneous multi-axis home")
            return
        rail, wrapper = matches[0]
        axis = wrapper.get_axis()
        cfg = self.tap_confirm[axis]
        if cfg['samples'] <= 1:
            return
        self._confirm_axis_home(homing_state, rail, wrapper, cfg)

    def _confirm_axis_home(self, homing_state, rail, wrapper, cfg):
        toolhead = self.printer.lookup_object('toolhead')
        cmderr = self.printer.command_error
        axis = wrapper.get_axis()
        axis_idx = 'xyz'.index(axis)
        hi = rail.get_homing_info()
        direction = 1. if hi.positive_dir else -1.
        pos_endstop = hi.position_endstop
        speed = cfg['speed'] or hi.speed
        retract_speed = cfg['retract_speed'] or hi.retract_speed
        retract_dist = cfg['retract_dist']
        overtravel = cfg['overtravel']
        if overtravel is None:
            overtravel = retract_dist
        arm_delay = self.axis_homing_arm_delay.get(axis, 0.)
        if retract_dist <= speed * arm_delay:
            raise cmderr(
                "adxl345_probe: %s_tap_confirm_retract_dist (%.2fmm) must"
                " exceed the distance covered during the tap arm delay"
                " (%.1fmm/s * %.3fs = %.2fmm)"
                % (axis, retract_dist, speed, arm_delay, speed * arm_delay))
        range_min, range_max = rail.get_range()
        seek_budget = cfg['max_seek_dist']
        if seek_budget is None:
            seek_budget = range_max - range_min
        endstops = rail.get_endstops()
        steppers = wrapper.get_steppers()
        step_dists = {s.get_name(): s.get_step_dist() for s in steppers}

        # Like HomingMove.check_no_movement(): the debug harness has no
        # physical stop, so trigger positions never agree there.  It
        # compares the taps given to ADXL_TAP_SIMULATE instead.
        is_debug = self.printer.get_start_args().get('debuginput') is not None
        sim = self._sim_taps.pop(axis, None) if is_debug else None
        sim_base = dict(homing_state.trigger_mcu_pos)
        tol = cfg['tolerance']
        stall_dist = cfg['stall_dist']
        budget = tap_confirm.Budget(cfg['retries'], cfg['stall_retries'])

        def run_homing_move(start_dist, move_speed):
            # Pretend to be start_dist away from the endstop so the move
            # has room to reach it, then home towards position_endstop.
            startpos = list(toolhead.get_position())
            startpos[axis_idx] = pos_endstop - direction * start_dist
            toolhead.set_position(startpos)
            homepos = list(startpos)
            homepos[axis_idx] = pos_endstop
            hmove = homing.HomingMove(self.printer, endstops)
            hmove.muon_speed = move_speed
            wrapper.last_trigger_time = 0.
            hmove.homing_move(homepos, move_speed, check_triggered=False)
            if wrapper.last_trigger_time <= 0.:
                return None, start_dist, {
                    'halt': {sp.stepper_name: sp.halt_pos
                             for sp in hmove.stepper_positions}}
            sps = hmove.stepper_positions
            trig = {sp.stepper_name: sp.trig_pos for sp in sps}
            dirs = tap_confirm.tap_dirs(
                {sp.stepper_name: sp.start_pos for sp in sps}, trig)
            travelled = max([abs(sp.trig_pos - sp.start_pos)
                             * step_dists[sp.stepper_name] for sp in sps])
            info = {
                'dirs': dirs, 'travelled': travelled,
                'follow': max([abs(sp.halt_pos - sp.trig_pos)
                               * step_dists[sp.stepper_name] for sp in sps]),
            }
            if sim is not None:
                # Debug harness: the tap lands where the test says, in mm
                # towards the stop from the first tap.
                off = sim.pop(0) if sim else 0.
                trig = {n: sim_base[n] + dirs[n] * int(round(
                    off / step_dists[n])) for n in trig}
            return trig, travelled, info

        def compare(prev, trig, info):
            if is_debug and sim is None:
                return 0., tap_confirm.AGREE
            delta = tap_confirm.signed_delta(prev, trig, info['dirs'],
                                             step_dists)
            kind = tap_confirm.classify(delta, tol, stall_dist)
            names = sorted(step_dists)
            logging.info(
                "adxl345_probe: %s tap compare: prev %s, new %s, delta"
                " %+.3fmm (%s), approach %.1fmm/s, travelled %.3fmm,"
                " follow-through %.3fmm", axis,
                ' '.join(['%s=%d' % (n, prev[n]) for n in names]),
                ' '.join(['%s=%d' % (n, trig[n]) for n in names]),
                delta, kind, speed, info['travelled'], info['follow'])
            return delta, kind

        accepted = [dict(homing_state.trigger_mcu_pos)]
        history = []

        def positions(tap):
            return ' '.join('%s=%d(%.5fmm)' % (n, tap[n],
                           tap[n] * step_dists[n]) for n in sorted(step_dists))

        def summary():
            return '; taps: initial [%s]; %s' % (
                positions(homing_state.trigger_mcu_pos), '; '.join(history))

        seek_left = cfg['seek_retries']
        stalls = 0
        missed = 0
        while len(accepted) < cfg['samples']:
            # Cap follow-through to avoid driving into the stop on a missed tap.
            retractpos = list(toolhead.get_position())
            retractpos[axis_idx] = pos_endstop - direction * retract_dist
            toolhead.move(retractpos, retract_speed)
            self._trace_phase = 'confirm'
            if axis == 'y' and hasattr(self, '_y_tap_log'):
                self._y_tap_log.kind = 'confirm'
            trig, travelled, info = run_homing_move(
                retract_dist + overtravel, speed)
            if trig is None:
                kind = ('missed' if cfg['overtravel'] is not None
                        else 'no_trigger')
                history.append('%s halt [%s]'
                               ' (trigger delta unavailable,'
                               ' approach %.3fmm)' % (
                                   kind,
                                   positions(info['halt']),
                                   retract_dist + overtravel))
                if cfg['overtravel'] is not None:
                    missed += 1
                    if not budget.take('missed'):
                        raise cmderr(
                            "adxl345_probe: %s home aborted: tap not detected"
                            " within %.3fmm of the previous tap %d times;"
                            " either the tap threshold is too high or the"
                            " previous tap was spurious. Seek not resumed,"
                            " to avoid driving into the stop."
                            % (axis, overtravel, missed) + summary())
                    # Undo the approach rebase so missed taps cannot walk
                    # the capped endpoint further into the stop on retry.
                    haltpos = list(toolhead.get_position())
                    haltpos[axis_idx] += direction * overtravel
                    toolhead.set_position(haltpos)
                    continue
                # No tap within reach; retain the bounded seek fallback.
                # Resume the seek from here within the travel budget.
                seek_budget -= travelled
                capped = ""
                if seek_left <= 0 or seek_budget <= 0.:
                    raise cmderr(
                        "adxl345_probe: %s home tap not confirmed; no"
                        " contact within %.1fmm of the reported tap"
                        " and seek retries exhausted%s"
                        % (axis, retract_dist, capped) + summary())
                seek_left -= 1
                seek_dist = seek_budget
                logging.info("adxl345_probe: %s %s,"
                             " resuming seek for up to %.1fmm",
                             axis, ("capped approach had no trigger"
                                    if capped else "tap judged spurious"),
                             seek_dist)
                self._trace_phase = 'resume_seek'
                if axis == 'y' and hasattr(self, '_y_tap_log'):
                    self._y_tap_log.kind = 'resume'
                trig, travelled, info = run_homing_move(seek_dist, hi.speed)
                seek_budget -= travelled
                if trig is None:
                    history.append('resume_seek no_trigger halt [%s]'
                                   ' (trigger delta unavailable)'
                                   % positions(info['halt']))
                    raise cmderr(
                        "adxl345_probe: %s home failed; no tap within"
                        " %.1fmm of resumed seek%s" % (axis, seek_dist, capped)
                        + summary())
                history.append('resume_seek [%s] (new anchor)'
                               % positions(trig))
                accepted = [trig]
                continue
            delta, kind = compare(accepted[0], trig, info)
            history.append('[%s] delta %+.3fmm %s vs anchor [%s]' % (
                positions(trig), delta, kind, positions(accepted[0])))
            if kind == tap_confirm.AGREE:
                accepted.append(trig)
                continue
            # Never accept a pair that differs, however regular the
            # difference: only two taps at the same raw step count prove
            # that both stopped at first contact (KAN-1361).
            if kind == tap_confirm.STALL:
                stalls += 1
            if not budget.take(kind):
                why = ""
                if kind == tap_confirm.STALL:
                    why = ("; %d taps were driven into the stop: the tap"
                           " was not detected at first contact" % (stalls,))
                raise cmderr(
                    "adxl345_probe: %s home taps disagree by %.3fmm"
                    " (tolerance %.3fmm) after %d retries%s"
                    % (axis, abs(delta), tol, budget.used, why) + summary())
            logging.info("adxl345_probe: %s taps disagree by %.3fmm (%s),"
                         " retrying confirmation", axis, abs(delta), kind)
            accepted = [trig]
        homing_state.trigger_mcu_pos.update(accepted[-1])
        logging.info("adxl345_probe: %s home confirmed with %d agreeing"
                     " taps after %d retries (%d driven into the stop)",
                     axis, len(accepted), budget.used, stalls)

    def _handle_command_error(self):
        if self._tap_mode == 'axis_homing':
            try:
                self._stop_tap('axis_homing')
            except Exception:
                logging.exception("ADXL345 axis homing cleanup failed")
            try:
                self._release_onset_client()
            finally:
                self._reset_axis_homing_state()
        elif self._tap_mode == 'tap_test':
            try:
                self._stop_tap('tap_test')
            except Exception:
                logging.exception("ADXL345 tap test cleanup failed")
        elif self._tap_mode == 'probe':
            try:
                self._stop_tap('probe')
            except Exception:
                logging.exception("ADXL345 probe cleanup failed")
            if not self._in_multi_probe:
                self._set_fan_disabled(False)
            try:
                self.deactivate_gcode.run_gcode_from_command()
            except Exception:
                logging.exception("ADXL345 deactivate_gcode failed")

    def init_adxl(self):
        self.phoming = self.printer.lookup_object('homing')
        chip = self.adxl345
        if (getattr(self._adxl_mcu, 'is_non_critical', False)
                and getattr(self._adxl_mcu, 'non_critical_disconnected', False)):
            self._adxl_initialized = False
            logging.info("ADXL345 probe init deferred: MCU '%s' is disconnected",
                         self._adxl_mcu.get_name())
            return
        chip.check_connected()
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0x2B if self.inverted else 0x0B)
        chip.set_reg(REG_INT_MAP, self.int_map)
        chip.set_reg(REG_INT_ENABLE, 0x00)
        self._set_profile_regs(self._get_profile('z'))
        self._try_clear_tap()
        self._adxl_initialized = True

    def _handle_adxl_reconnect(self):
        try:
            self.init_adxl()
        except Exception:
            # Do not fail global reconnect flow on ADXL init errors.
            logging.exception("ADXL345 probe reconnect init failed")

    def _handle_adxl_disconnect(self):
        self._adxl_initialized = False
        self._tap_mode = None
        self._tap_was_measuring = False
        self._reset_axis_homing_state()
        if not self._in_multi_probe:
            self._set_fan_disabled(False)

    # Probe interface (z probing only)
    def _require_z_probe(self):
        if not self.enable_z_probe:
            raise self.printer.command_error(
                "Z probe interface disabled in [adxl345_probe]")

    def multi_probe_begin(self):
        self._require_z_probe()
        self._in_multi_probe = True
        self._set_fan_disabled(True)

    def multi_probe_end(self):
        self._require_z_probe()
        self._set_fan_disabled(False)
        self._in_multi_probe = False

    def probing_move(self, pos, speed):
        self._require_z_probe()
        return self.phoming.probing_move(self._z_wrapper, pos, speed)

    def probe_prepare(self, hmove):
        self._require_z_probe()
        self.activate_gcode.run_gcode_from_command()
        try:
            self._start_tap(self._get_profile('z'), 'probe')
        except Exception:
            try:
                self.deactivate_gcode.run_gcode_from_command()
            except Exception:
                logging.exception("ADXL345 deactivate_gcode failed")
            raise
        if not self._in_multi_probe:
            self._set_fan_disabled(True)

    def probe_finish(self, hmove):
        self._require_z_probe()
        try:
            self._stop_tap('probe')
        finally:
            self.deactivate_gcode.run_gcode_from_command()
            if not self._in_multi_probe:
                self._set_fan_disabled(False)

    def get_position_endstop(self):
        self._require_z_probe()
        return self.position_endstop

    def get_probe_params(self, gcmd=None):
        self._require_z_probe()
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self, gcmd=None):
        self._require_z_probe()
        return self.probe_offsets.get_offsets(gcmd)

    def get_status(self, eventtime):
        if self.cmd_helper is None:
            return {
                'tap_mode': self._tap_mode,
                'profiles': {
                    axis: {
                        'tap_thresh': p['tap_thresh'],
                        'tap_dur': p['tap_dur'],
                        'tap_axes': self._mask_to_axes(p['tap_axes_mask']),
                    }
                    for axis, p in self.tap_profiles.items()
                },
            }
        return self.cmd_helper.get_status(eventtime)

    def start_probe_session(self, gcmd):
        self._require_z_probe()
        return self.probe_session.start_probe_session(gcmd)

    cmd_SET_ACCEL_PROBE_help = (
        "Configure ADXL345 tap profiles for X/Y endstops and optional Z probe. "
        "Params: AXIS=<X|Y|Z|ALL> TAP_THRESH=<mm/s^2> TAP_DUR=<s> "
        "TAP_AXES=<xyz combination> HOMING_TAP_ARM_DELAY=<s>"
    )

    cmd_ADXL_TEST_TAP_help = (
        "Wait for physical ADXL tap interrupts on a selected axis profile. "
        "Params: AXIS=<X|Y|Z> TAPS=<count> TIMEOUT=<s> QUIET_TIME=<s> "
        "[TAP_THRESH=<mm/s^2> TAP_DUR=<s> TAP_AXES=<xyz>]"
    )

    def cmd_SET_ACCEL_PROBE(self, gcmd):
        axis = gcmd.get('AXIS', 'ALL').lower()
        if axis not in ('x', 'y', 'z', 'all'):
            raise gcmd.error("AXIS must be X, Y, Z, or ALL")
        params = gcmd.get_command_parameters()
        has_thresh = 'TAP_THRESH' in params
        has_dur = 'TAP_DUR' in params
        has_axes = 'TAP_AXES' in params
        has_homing_arm_delay = 'HOMING_TAP_ARM_DELAY' in params
        if self._tap_mode is not None and (has_thresh or has_dur or has_axes
                                           or has_homing_arm_delay):
            raise gcmd.error(
                "Can not update tap settings while homing/probing move is active")

        targets = ['x', 'y', 'z'] if axis == 'all' else [axis]
        new_thresh = None
        new_dur = None
        new_axes_mask = None
        if has_thresh:
            new_thresh = gcmd.get_float('TAP_THRESH', minval=TAP_SCALE,
                                        maxval=100000.)
        if has_dur:
            new_dur = gcmd.get_float('TAP_DUR', above=DUR_SCALE, maxval=0.1)
        if has_axes:
            axes = gcmd.get('TAP_AXES')
            new_axes_mask = self._parse_tap_axes(axes, gcmd.error, 'TAP_AXES')
        new_homing_arm_delay = None
        if has_homing_arm_delay:
            new_homing_arm_delay = gcmd.get_float(
                'HOMING_TAP_ARM_DELAY', minval=0., maxval=1.0)

        for target in targets:
            profile = self._get_profile(target)
            if new_thresh is not None:
                profile['tap_thresh'] = new_thresh
            if new_dur is not None:
                profile['tap_dur'] = new_dur
            if new_axes_mask is not None:
                profile['tap_axes_mask'] = new_axes_mask
            if new_homing_arm_delay is not None:
                self.axis_homing_arm_delay[target] = new_homing_arm_delay

        lines = []
        for a in 'xyz':
            p = self._get_profile(a)
            lines.append(
                "ADXL345 profile %s: tap_thresh=%.3f tap_dur=%.6f tap_axes=%s"
                " homing_tap_arm_delay=%.4f"
                % (a.upper(), p['tap_thresh'], p['tap_dur'],
                   self._mask_to_axes(p['tap_axes_mask']),
                   self.axis_homing_arm_delay.get(a, 0.)))
        gcmd.respond_info("\n".join(lines))

    def cmd_ADXL_TEST_TAP(self, gcmd):
        if self._tap_mode is not None:
            raise gcmd.error(
                "ADXL tap detection is busy (%s)" % (self._tap_mode,))
        axis = gcmd.get('AXIS', 'X')
        wrapper = self._lookup_axis_wrapper(axis, gcmd)
        axis = wrapper.get_axis()
        taps = gcmd.get_int('TAPS', 3, minval=1, maxval=50)
        timeout = gcmd.get_float('TIMEOUT', 30., above=0.)
        quiet_time = gcmd.get_float('QUIET_TIME', 0.2, minval=0.)
        params = gcmd.get_command_parameters()

        profile = dict(self._get_profile(axis))
        if 'TAP_THRESH' in params:
            profile['tap_thresh'] = gcmd.get_float(
                'TAP_THRESH', minval=TAP_SCALE, maxval=100000.)
        if 'TAP_DUR' in params:
            profile['tap_dur'] = gcmd.get_float(
                'TAP_DUR', above=DUR_SCALE, maxval=0.1)
        if 'TAP_AXES' in params:
            profile['tap_axes_mask'] = self._parse_tap_axes(
                gcmd.get('TAP_AXES'), gcmd.error, 'TAP_AXES')

        gcmd.respond_info(
            "ADXL_TEST_TAP: axis=%s taps=%d timeout=%.2fs quiet=%.2fs "
            "tap_thresh=%.3f tap_dur=%.6f tap_axes=%s"
            % (axis.upper(), taps, timeout, quiet_time,
               profile['tap_thresh'], profile['tap_dur'],
               self._mask_to_axes(profile['tap_axes_mask'])))

        reactor = self.printer.get_reactor()
        for tap_num in range(taps):
            self._start_tap(profile, 'tap_test')
            try:
                did_trigger = self._wait_for_tap(wrapper, timeout)
            finally:
                self._stop_tap('tap_test')
            if not did_trigger:
                raise gcmd.error(
                    "ADXL_TEST_TAP timed out waiting for tap %d/%d on axis %s"
                    % (tap_num + 1, taps, axis.upper()))
            gcmd.respond_info("ADXL_TEST_TAP: tap %d/%d detected"
                              % (tap_num + 1, taps))
            if quiet_time > 0. and tap_num + 1 < taps:
                reactor.pause(reactor.monotonic() + quiet_time)


def load_config(config):
    return ADXL345Probe(config)


def load_config_prefix(config):
    return ADXL345Probe(config)
