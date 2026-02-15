import logging
import pins
from . import probe, adxl345

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


class ADXL345TapEndstop:
    def __init__(self, owner, axis, mcu_endstop, position_endstop=None):
        self._owner = owner
        self._axis = axis
        self._mcu_endstop = mcu_endstop
        self._position_endstop = position_endstop

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
        return self._mcu_endstop.home_start(
            print_time, sample_time, sample_count, rest_time,
            triggered=triggered)

    def home_wait(self, home_end_time):
        return self._mcu_endstop.home_wait(home_end_time)

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
        self._tap_enable_timer = self.reactor.register_timer(
            self._handle_tap_enable_timer, self.reactor.NEVER)
        self._active_axis_homing_wrappers = []
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

    def _try_clear_tap(self):
        chip = self.adxl345
        tries = 8
        while tries > 0:
            val = chip.read_reg(REG_INT_SOURCE)
            if not (val & INT_SINGLE_TAP):
                return True
            tries -= 1
        return False

    def _cancel_tap_enable_timer(self):
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
        self._tap_enable_mode = None
        if self._tap_mode != mode:
            return self.reactor.NEVER
        chip = self.adxl345
        try:
            # Drop startup motion events that happened during arm delay.
            chip.read_reg(REG_INT_SOURCE)
            if not self._try_clear_tap():
                raise self.printer.command_error(
                    "ADXL345 tap active before delayed arm")
            chip.set_reg(REG_INT_ENABLE, INT_SINGLE_TAP)
        except Exception as e:
            logging.exception("Failed delayed ADXL345 tap arm")
            self.printer.invoke_shutdown(
                "ADXL345 delayed tap arm failed: %s" % (str(e),))
        return self.reactor.NEVER

    def _start_tap(self, profile, mode, arm_delay=0.):
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
            if arm_delay > 0.:
                self._tap_enable_pending = True
                self._tap_enable_mode = mode
                self.reactor.update_timer(
                    self._tap_enable_timer,
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
        profile = self._merge_profiles(wrappers)
        arm_delay = max([self.axis_homing_arm_delay.get(w.get_axis(), 0.)
                         for w in wrappers])
        self._start_tap(profile, 'axis_homing', arm_delay=arm_delay)
        self._active_axis_homing_wrappers = wrappers

    def _handle_homing_move_end(self, hmove):
        if not self._active_axis_homing_wrappers:
            return
        hmove_endstops = hmove.get_mcu_endstops()
        if not any(w in hmove_endstops for w in self._active_axis_homing_wrappers):
            return
        try:
            self._stop_tap('axis_homing')
        finally:
            self._active_axis_homing_wrappers = []

    def _handle_command_error(self):
        if self._tap_mode == 'axis_homing':
            try:
                self._stop_tap('axis_homing')
            except Exception:
                logging.exception("ADXL345 axis homing cleanup failed")
            self._active_axis_homing_wrappers = []
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
        self._active_axis_homing_wrappers = []
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
