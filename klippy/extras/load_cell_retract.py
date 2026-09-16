# Experimental MCU-triggered load-cell retract for one Cartesian Z motor.
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import bisect, logging, math


def build_profile(distance, step_dist, velocity, accel, frequency):
    """Quantized trapezoid; return the exact scheduled clocks for ascent Z."""
    values = (distance, step_dist, velocity, accel, frequency)
    if not all(math.isfinite(v) and v > 0. for v in values):
        raise ValueError("Invalid retract profile parameters")
    steps = int(math.ceil(distance / step_dist))
    if not 1 <= steps <= 4096:
        raise ValueError("MCU retract must contain 1..4096 microsteps")
    distance = steps * step_dist
    peak = min(velocity, math.sqrt(distance * accel))
    ramp_dist = peak * peak / (2. * accel)
    ramp_time = peak / accel
    duration = 2. * ramp_time + (distance - 2. * ramp_dist) / peak
    clocks = []
    for i in range(1, steps + 1):
        x = i * step_dist
        if x < ramp_dist:
            t = math.sqrt(2. * x / accel)
        elif x <= distance - ramp_dist:
            t = ramp_time + (x - ramp_dist) / peak
        else:
            t = duration - math.sqrt(max(0., 2. * (distance - x) / accel))
        clocks.append(int(math.ceil(t * frequency)))
    intervals = [b - a for a, b in zip([0] + clocks, clocks)]
    # Use one (rounded-up) cruise interval. Alternating floor/ceil intervals
    # waste profile RAM; rounding up only delays steps, never speeds them up.
    cruise_interval = int(math.ceil(step_dist / peak * frequency))
    for i in range(steps):
        if i * step_dist >= ramp_dist and (i+1)*step_dist <= distance-ramp_dist:
            intervals[i] = cruise_interval
    clocks, clock = [], 0
    for interval in intervals:
        clock += interval
        clocks.append(clock)
    if clocks[-1] > int(frequency * .5):
        raise ValueError("MCU retract duration exceeds 0.5 seconds")
    if min(intervals) < math.ceil(frequency * .000002):
        raise ValueError("MCU retract step rate exceeds firmware limit")
    segments = []
    i = 0
    while i < steps:
        count, add = 1, 0
        if i + 1 < steps and -32768 <= intervals[i+1] - intervals[i] <= 32767:
            add = intervals[i+1] - intervals[i]
            count = 2
            while (i + count < steps and count < 4096
                   and intervals[i+count] == intervals[i] + count * add):
                count += 1
        segments.append((intervals[i], count, add))
        i += count
    if len(segments) > 512:
        raise ValueError("MCU retract profile exceeds 512 segments")
    return segments, clocks


class AutomaticRetract:
    def __init__(self, config, endstop):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.endstop = endstop
        self.cycle = 0
        self.profile_key = None
        self.result = None
        self.stepper = None
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._identify)

    def _identify(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        # This profile assumes Z is a direct, independent Cartesian coordinate.
        if type(kin).__name__ not in ('CoreXYKinematics', 'CartKinematics'):
            raise self.printer.config_error(
                "MCU retract requires cartesian/corexy kinematics")
        steppers = self.endstop.get_steppers()
        if len(steppers) != 1 or steppers[0].get_name() != 'stepper_z':
            raise self.printer.config_error(
                "MCU retract requires exactly one independent Z stepper")
        self.stepper = steppers[0]
        self.mcu = self.stepper.get_mcu()
        self.oid = self.stepper.get_oid()
        # Cross-MCU trigger propagation uses the C fastreader, as normal homing.
        self.reason = (1 if self.mcu is self.endstop.get_mcu() else 255)
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.mcu.add_config_cmd("config_stepper_retract oid=%d capacity=512"
                                % self.oid)
        self.queue = self.mcu.alloc_command_queue()
        self.segment_cmd = self.mcu.lookup_command(
            "stepper_retract_segment oid=%c index=%hu interval=%u"
            " count=%hu add=%hi", cq=self.queue)
        self.arm_cmd = self.mcu.lookup_command(
            "stepper_retract_arm oid=%c cycle=%u delay=%u dir=%c reason=%c",
            cq=self.queue)
        self.cancel_cmd = self.mcu.lookup_command(
            "stepper_retract_cancel oid=%c", cq=self.queue)
        self.query_cmd = self.mcu.lookup_query_command(
            "stepper_retract_query oid=%c",
            "stepper_retract_state oid=%c cycle=%u state=%c"
            " start=%u end=%u halt=%i pos=%i", oid=self.oid, cq=self.queue)
        dispatch_queue = self.endstop.get_dispatch().get_stepper_command_queue(
            self.stepper)
        self.barrier_cmd = self.mcu.lookup_query_command(
            "stepper_retract_query oid=%c",
            "stepper_retract_state oid=%c cycle=%u state=%c"
            " start=%u end=%u halt=%i pos=%i", oid=self.oid, cq=dispatch_queue)
        self.profile_key = None

    def prepare(self, gcmd, params):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        now = self.reactor.monotonic()
        status = kin.get_status(now)
        if 'z' not in status['homed_axes']:
            raise gcmd.error("Must home Z before MCU retract")
        if self.mcu.is_fileoutput():
            raise gcmd.error("MCU retract requires a connected MCU")
        shaper = self.printer.lookup_object('input_shaper', None)
        if shaper is not None:
            for axis in shaper.get_shapers():
                if axis.get_name() == 'shaper_z' and axis.is_enabled():
                    raise gcmd.error("MCU retract does not support Z shaping")
        if self.printer.lookup_object('dual_carriage', None) is not None:
            raise gcmd.error("MCU retract does not support dual carriage")
        distance = params['load_cell_retract_dist']
        step_dist = self.stepper.get_step_dist()
        velocity = min(params['lift_speed'], kin.max_z_velocity,
                       toolhead.get_max_velocity()[0])
        accel = min(kin.max_z_accel, toolhead.get_max_velocity()[1])
        frequency = self.mcu.seconds_to_clock(1.)
        key = (distance, step_dist, velocity, accel, frequency)
        try:
            segments, clocks = build_profile(*key)
        except ValueError as e:
            raise gcmd.error(str(e))
        lift = len(clocks) * step_dist
        # Even an immediate trigger must leave sufficient headroom to retract.
        if toolhead.get_position()[2] + lift > status['axis_maximum'].z:
            raise gcmd.error("Insufficient Z headroom for MCU retract")
        pulse, unused = self.stepper.get_pulse_duration()
        minimum = (2 * self.mcu.seconds_to_clock(pulse)
                   + self.mcu.seconds_to_clock(.000002))
        if any(min(iv, iv + (n-1)*add) < minimum for iv, n, add in segments):
            raise gcmd.error("Retract exceeds step pulse timing limits")
        self.direction = int(not self.stepper.get_dir_inverted()[0])
        self.result = None
        self.clocks, self.step_dist = clocks, step_dist
        if key != self.profile_key:
            for i, (iv, count, add) in enumerate(segments):
                self.segment_cmd.send([self.oid, i, iv, count, add])
            # Query is an ordering barrier acknowledging the entire profile.
            self.query_cmd.send([self.oid])
            self.profile_key = key

    def get_mcu(self):
        return self.endstop.get_mcu()

    def get_steppers(self):
        return self.endstop.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        try:
            return self.endstop.home_start(
                print_time, sample_time, sample_count, rest_time,
                triggered=triggered, automatic_retract=True,
                arm_callback=self._arm)
        except Exception:
            self._abort()
            raise

    def _arm(self):
        # Register stop handlers before arming. A query on the dispatch queue
        # acknowledges registration on the Z MCU (the queues are independent).
        self.barrier_cmd.send([self.oid])
        self.cycle = (self.cycle % 0xffffffff) + 1
        self.arm_cmd.send([self.oid, self.cycle,
                           self.mcu.seconds_to_clock(.001),
                           self.direction, self.reason])
        state = self.query_cmd.send([self.oid])
        if state['cycle'] != self.cycle or state['state'] != 1:
            raise self.printer.command_error("MCU retract failed to arm")

    def _abort(self):
        try:
            self.cancel_cmd.send([self.oid])
        finally:
            self.printer.invoke_shutdown("Automatic probe retract failed")

    def home_wait(self, home_end_time):
        try:
            trigger_time = self.endstop.home_wait(home_end_time,
                                                  notify_steppers=False)
            deadline = self.reactor.monotonic() + 1.
            while True:
                state = self.query_cmd.send([self.oid])
                if state['cycle'] != self.cycle:
                    raise self.printer.command_error("Stale MCU retract result")
                if state['state'] == 3:
                    break
                if (not trigger_time or state['state'] != 2
                    or self.reactor.monotonic() >= deadline):
                    raise self.printer.command_error("MCU retract incomplete")
                self.reactor.pause(self.reactor.monotonic() + .005)
            if not trigger_time:
                raise self.printer.command_error("Retract without probe trigger")
            sign = 1 if self.direction else -1
            if sign * (state['pos'] - state['halt']) != len(self.clocks):
                raise self.printer.command_error("MCU retract position mismatch")
            elapsed = (state['end'] - state['start']) & 0xffffffff
            pulse, unused = self.stepper.get_pulse_duration()
            tolerance = self.mcu.seconds_to_clock(2*pulse + .000010)
            if not self.clocks[-1] <= elapsed <= self.clocks[-1] + tolerance:
                raise self.printer.command_error("MCU retract timing mismatch")
            # Recover the absolute clocks using the motor MCU's own clock map.
            start = self.mcu.clock32_to_clock64(state['start'])
            self.start_clock = start
            self.ascent_start = self.mcu.clock_to_print_time(start)
            self.ascent_end = self.mcu.clock_to_print_time(
                start + self.clocks[-1])
            self.halt_z = self.stepper.mcu_to_commanded_position(
                sign * state['halt'])
            self.result = state
            # Reset/query only after the firmware has finished the lift.
            self.stepper.note_homing_end()
            logging.info("load_cell_probe: automatic_retract cycle=%d"
                         " trigger_to_ascent=%.6f duration=%.6f steps=%d",
                         self.cycle, self.ascent_start - trigger_time,
                         self.ascent_end - self.ascent_start, len(self.clocks))
            return trigger_time
        except Exception:
            self._abort()
            raise

    def z_at(self, print_time):
        ticks = self.mcu.print_time_to_clock(print_time) - self.start_clock
        count = bisect.bisect_right(self.clocks, ticks)
        return self.halt_z + count * self.step_dist
