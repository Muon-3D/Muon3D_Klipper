# Bench tool: read TMC driver registers in the middle of constant-speed moves
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# For tuning the X/Y drivers and kinematics (scripts/bench/motor_tune_suite.py
# drives it overnight). Not part of a shipped configuration: [motor_tune] is
# added with debug.cfg on a bench machine only.
#
# MOTOR_TUNE_SAMPLE AXIS=X|Y SPEED=<mm/s> ACCEL=<mm/s^2> PASSES=<n>
#                   SAMPLES=<per pass> LABEL=<text> TAG=<json>
#   Moves back and forth along one axis (X 30-170 at Y100, Y 40-150 at X100)
#   at SPEED, and in the constant-speed part of each pass reads, for each X/Y
#   driver: DRV_STATUS, PWM_SCALE, PWM_AUTO, SG4_RESULT and TSTEP. Each
#   reading is one JSON line in log_path, with the speed the driver itself
#   measured (from TSTEP) next to the commanded one. A pure X or Y move runs
#   both CoreXY motors at SPEED.
# MOTOR_TUNE_READ LABEL=<text> TAG=<json>
#   The same registers at standstill (PWM_AUTO after StealthChop tuning).
#
# The registers say:
#   pwm_scale_sum  StealthChop's drive amplitude, 0-255 (0-1023 on some
#                  parts). Near the top it has no voltage left to hold the
#                  current: the speed where that happens bounds StealthChop.
#   pwm_ofs_auto, pwm_grad_auto  StealthChop's learnt tuning (AT#1, AT#2).
#   cs_actual      the current scale the driver is actually using (0-31).
#   sg_result      StallGuard2 load (SpreadCycle, above TCOOLTHRS): lower is
#                  more load, 0 at stall.
#   sg4_result     StallGuard4 load (StealthChop): lower is more load.
#   stealth, s2g*, s2vs*, ot, otpw, ola, olb  mode and fault flags.
import json, logging, math, time

PATHS = {
    'x': ((30., 100.), (170., 100.)),
    'y': ((100., 40.), (100., 150.)),
}
REGISTERS = ('DRV_STATUS', 'PWM_SCALE', 'PWM_AUTO', 'SG4_RESULT', 'TSTEP')
FIELDS = {
    'DRV_STATUS': ('sg_result', 'stealth', 'cs_actual', 'stallguard', 'ot',
                   'otpw', 's2ga', 's2gb', 's2vsa', 's2vsb', 'ola', 'olb',
                   'stst'),
    'PWM_SCALE': ('pwm_scale_sum', 'pwm_scale_auto'),
    'PWM_AUTO': ('pwm_ofs_auto', 'pwm_grad_auto'),
    'SG4_RESULT': ('sg4_result',),
    'TSTEP': ('tstep',),
}
DRIVERS = ('tmc2241', 'tmc2240', 'tmc5160', 'tmc2130', 'tmc2209', 'tmc2208')


class MotorTune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.log_path = config.get('log_path',
                                   '/home/printer_data/logs/motor_tune.jsonl')
        self.stepper_names = config.getlist('steppers',
                                            ('stepper_x', 'stepper_y'))
        self.drivers = {}
        self.steppers = {}
        self.toolhead = self.mcu = None
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.gcode.register_command('MOTOR_TUNE_SAMPLE', self.cmd_SAMPLE,
                                    desc=self.cmd_SAMPLE_help)
        self.gcode.register_command('MOTOR_TUNE_READ', self.cmd_READ,
                                    desc=self.cmd_READ_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.mcu = self.printer.lookup_object('mcu')
        # Steppers are not printer objects: the kinematics hold them.
        for s in self.toolhead.get_kinematics().get_steppers():
            self.steppers[s.get_name()] = s
        for name in self.stepper_names:
            for drv in DRIVERS:
                obj = self.printer.lookup_object('%s %s' % (drv, name), None)
                if obj is not None:
                    self.drivers[name] = obj
                    break

    # Register reads
    def _read_driver(self, name):
        # Never let a read take Klipper down (an exception in a command is
        # a shutdown): log it in the record instead.
        try:
            return self._read_driver_regs(name)
        except self.printer.command_error:
            raise
        except Exception as e:
            logging.exception("motor_tune: reading %s", name)
            return {'error': repr(e)}

    def _read_driver_regs(self, name):
        mcu_tmc = self.drivers[name].mcu_tmc
        fields = mcu_tmc.get_fields()
        out = {}
        for reg in REGISTERS:
            if fields.lookup_register(FIELDS[reg][0], None) is None:
                continue
            try:
                val = mcu_tmc.get_register(reg)
            except self.printer.command_error as e:
                out['error_' + reg] = str(e)
                continue
            for f in FIELDS[reg]:
                if fields.lookup_register(f, None) == reg:
                    out[f] = fields.get_field(f, val)
        tstep = out.get('tstep')
        if tstep and tstep < 0xfffff:
            try:
                out['speed_measured'] = round(
                    self._tstep_speed(name, tstep), 2)
            except Exception:
                logging.exception("motor_tune: speed from TSTEP")
        return out

    def _tstep_speed(self, name, tstep):
        # TSTEP is the time between 1/256 microsteps in TMC clocks
        # (fCLK 12.5 MHz), whatever the configured microstepping.
        # As TMCtstepHelper: step_dist at the configured microstepping,
        # divided down to 1/256.
        stepper = self.steppers[name]
        mcu_tmc = self.drivers[name].mcu_tmc
        step_dist_256 = stepper.get_step_dist() / (
            1 << mcu_tmc.get_fields().get_field('mres'))
        return step_dist_256 * mcu_tmc.get_tmc_frequency() / tstep

    def _log(self, rec):
        rec['ts'] = round(time.time(), 3)
        try:
            with open(self.log_path, 'a') as f:
                f.write(json.dumps(rec, sort_keys=True) + '\n')
        except IOError:
            logging.exception("motor_tune: cannot write %s", self.log_path)

    def _wait_print_time(self, print_time):
        while True:
            now = self.reactor.monotonic()
            est = self.mcu.estimated_print_time(now)
            if est >= print_time:
                return
            # At least 1 ms, so float rounding cannot leave it creeping.
            self.reactor.pause(now + max(0.001, min(0.05, print_time - est)))

    def _parse_common(self, gcmd):
        tag = gcmd.get('TAG', '{}')
        try:
            tag = json.loads(tag)
        except ValueError:
            tag = {'tag': tag}
        return gcmd.get('LABEL', ''), tag

    cmd_READ_help = "Bench: log the X/Y drivers' registers at standstill"
    def cmd_READ(self, gcmd):
        label, tag = self._parse_common(gcmd)
        self.toolhead.wait_moves()
        for name in self.drivers:
            rec = {'kind': 'read', 'label': label, 'stepper': name}
            rec.update(tag)
            rec.update(self._read_driver(name))
            self._log(rec)
            gcmd.respond_info("MOTOR_TUNE_READ %s %s: %s" % (
                label, name, json.dumps(rec, sort_keys=True)))

    cmd_SAMPLE_help = ("Bench: read the X/Y drivers' registers during"
                       " constant-speed moves")
    def cmd_SAMPLE(self, gcmd):
        axis = gcmd.get('AXIS', 'X').lower()
        if axis not in PATHS:
            raise gcmd.error("AXIS must be X or Y")
        speed = gcmd.get_float('SPEED', above=0.)
        accel = gcmd.get_float('ACCEL', 5000., above=0.)
        passes = gcmd.get_int('PASSES', 2, minval=1)
        samples = gcmd.get_int('SAMPLES', 2, minval=1, maxval=10)
        label, tag = self._parse_common(gcmd)
        status = self.toolhead.get_status(self.reactor.monotonic())
        if 'x' not in status['homed_axes'] or 'y' not in status['homed_axes']:
            raise gcmd.error("MOTOR_TUNE_SAMPLE: home X and Y first")
        old_v, old_a = status['max_velocity'], status['max_accel']
        old_mcr = status.get('minimum_cruise_ratio', 0.5)
        a, b = PATHS[axis]
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        ramp = speed * speed / accel        # both ramps together, mm
        cruise = dist - ramp
        run = self.gcode.run_script_from_command
        run("G90\nG1 X%.3f Y%.3f F6000\nM400" % a)
        # MINIMUM_CRUISE_RATIO=0: the whole ramp at ACCEL, so the cruise
        # window computed below is the real one.
        run("SET_VELOCITY_LIMIT VELOCITY=%.3f ACCEL=%.3f"
            " MINIMUM_CRUISE_RATIO=0" % (max(speed, old_v), accel))
        summary = {}
        try:
            for p in range(passes):
                end = b if p % 2 == 0 else a
                if cruise <= 0.:
                    run("G1 X%.3f Y%.3f F%.3f" % (end + (speed * 60.,)))
                    continue
                run("G1 X%.3f Y%.3f F%.3f" % (end + (speed * 60.,)))
                end_time = self.toolhead.get_last_move_time()
                duration = 2. * speed / accel + cruise / speed
                start_time = end_time - duration
                c0 = start_time + speed / accel
                c1 = c0 + cruise / speed
                for i in range(samples):
                    t = c0 + (c1 - c0) * (i + 0.5) / samples
                    self._wait_print_time(t)
                    for name in self.drivers:
                        regs = self._read_driver(name)
                        rec = {'kind': 'sample', 'label': label, 'axis': axis,
                               'speed': speed, 'accel': accel, 'pass': p,
                               'sample': i, 'stepper': name,
                               'cruise_s': round(cruise / speed, 4)}
                        rec.update(tag)
                        rec.update(regs)
                        self._log(rec)
                        s = summary.setdefault(name, [])
                        s.append(regs)
                self.toolhead.wait_moves()
        finally:
            run("SET_VELOCITY_LIMIT VELOCITY=%.3f ACCEL=%.3f"
                " MINIMUM_CRUISE_RATIO=%.3f" % (old_v, old_a, old_mcr))
        if cruise <= 0.:
            gcmd.respond_info("MOTOR_TUNE_SAMPLE %s: no constant-speed part"
                              " at %.0f mm/s, %.0f mm/s^2 over %.0f mm"
                              % (label, speed, accel, dist))
            return
        parts = []
        for name, regs in summary.items():
            def avg(k):
                v = [r[k] for r in regs if k in r]
                return sum(v) / len(v) if v else float('nan')
            parts.append("%s pwm_scale_sum %.0f cs_actual %.1f sg %.0f"
                         " sg4 %.0f measured %.0f mm/s" % (
                             name, avg('pwm_scale_sum'), avg('cs_actual'),
                             avg('sg_result'), avg('sg4_result'),
                             avg('speed_measured')))
        gcmd.respond_info("MOTOR_TUNE_SAMPLE %s %s %.0f mm/s: %s" % (
            label, axis.upper(), speed, "; ".join(parts)))


def load_config(config):
    return MotorTune(config)
