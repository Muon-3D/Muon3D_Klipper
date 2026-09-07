# Blocking smart nozzle wipe command.
#
# This moves the temperature-driven state machine and brush path out of
# recursive Jinja macro expansion.
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class NozzleWipeSmart:
    MODES = ("FULL", "NOHEAT", "FAN_ONLY")

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead = None
        self.heaters = None

        self.x = config.getfloat("x", 100.0)
        self.y = config.getfloat("y", 13.5)
        self.z_top = config.getfloat("z_top", 4.3)
        self.plunge = config.getfloat("plunge", 1.2, minval=0.0)
        self.sx = config.getfloat("sx", 9.0, above=0.0)
        self.sy = config.getfloat("sy", 2.0, above=0.0)
        self.passes = config.getint("passes", 5, minval=1)
        self.f_travel = config.getfloat("f_travel", 7800.0, above=0.0)
        self.f_wipe = config.getfloat("f_wipe", 2000.0, above=0.0)
        self.z_speed = config.getfloat("z_speed", 200.0, above=0.0)
        self.z_hop = config.getfloat("z_hop", 5.0, minval=0.0)

        self.start_temp = config.getfloat("start_temp", 170.0)
        self.wipe_temp = config.getfloat("wipe_temp", 200.0)
        self.wipe_temp_tolerance = config.getfloat(
            "wipe_temp_tolerance", 5.0, minval=0.0)
        self.stop_temp = config.getfloat("stop_temp", 160.0)
        self.fan_off_temp = config.getfloat("fan_off_temp", 160.0)
        self.fan_pct = config.getfloat("fan_pct", 0.20, minval=0.0, maxval=1.0)
        self.poll_ms = config.getint("poll_ms", 1000, minval=1)
        self.max_iter = config.getint("max_iter", 2000, minval=10)

        self.active = False
        self.stop_requested = False
        self.phase = "IDLE"
        self.default_mode = config.get("mode", "FULL").strip().upper()
        if self.default_mode not in self.MODES:
            raise config.error(
                "nozzle_wipe_smart mode must be FULL, NOHEAT, or FAN_ONLY")
        self.mode = self.default_mode
        self.iter = 0
        self.last_error = ""

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.gcode.register_command(
            "NOZZLE_WIPE_SMART", self.cmd_NOZZLE_WIPE_SMART,
            desc=self.cmd_NOZZLE_WIPE_SMART_help)
        self.gcode.register_command(
            "NOZZLE_WIPE_SMART_STOP", self.cmd_NOZZLE_WIPE_SMART_STOP,
            desc=self.cmd_NOZZLE_WIPE_SMART_STOP_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.heaters = self.printer.lookup_object("heaters")

    def get_status(self, eventtime):
        return {
            "active": self.active,
            "phase": self.phase,
            "mode": self.mode,
            "iter": self.iter,
            "stop_requested": self.stop_requested,
            "last_error": self.last_error,
        }

    def _get_params(self, gcmd):
        mode = gcmd.get("MODE", self.default_mode).strip().upper()
        if mode not in self.MODES:
            raise gcmd.error(
                "NOZZLE_WIPE_SMART: MODE must be FULL, NOHEAT, or FAN_ONLY")
        params = {
            "mode": mode,
            "x": gcmd.get_float("X", self.x),
            "y": gcmd.get_float("Y", self.y),
            "z_top": gcmd.get_float("Z_TOP", self.z_top),
            "plunge": gcmd.get_float("PLUNGE", self.plunge, minval=0.0),
            "sx": gcmd.get_float("SX", self.sx, above=0.0),
            "sy": gcmd.get_float("SY", self.sy, above=0.0),
            "passes": gcmd.get_int("PASSES", self.passes, minval=1),
            "f_travel": gcmd.get_float("F_TRAVEL", self.f_travel, above=0.0),
            "f_wipe": gcmd.get_float("F_WIPE", self.f_wipe, above=0.0),
            "z_speed": gcmd.get_float("Z_SPEED", self.z_speed, above=0.0),
            "z_hop": gcmd.get_float("Z_HOP", self.z_hop, minval=0.0),
            "start_temp": gcmd.get_float("START_TEMP", self.start_temp),
            "wipe_temp": gcmd.get_float("WIPE_TEMP", self.wipe_temp),
            "wipe_temp_tolerance": gcmd.get_float(
                "WIPE_TEMP_TOL", self.wipe_temp_tolerance, minval=0.0),
            "stop_temp": gcmd.get_float("STOP_TEMP", self.stop_temp),
            "fan_off_temp": gcmd.get_float("FAN_OFF_TEMP", self.fan_off_temp),
            "fan_pct": gcmd.get_float("FAN_PCT", self.fan_pct,
                                      minval=0.0, maxval=1.0),
            "poll_ms": gcmd.get_int("POLL_MS", self.poll_ms, minval=1),
            "max_iter": gcmd.get_int("MAX_ITER", self.max_iter, minval=10),
        }
        if params["start_temp"] > params["wipe_temp"]:
            raise gcmd.error(
                "NOZZLE_WIPE_SMART: START_TEMP must be <= WIPE_TEMP")
        if params["stop_temp"] >= params["wipe_temp"]:
            raise gcmd.error(
                "NOZZLE_WIPE_SMART: STOP_TEMP should be < WIPE_TEMP")
        return params

    def _active_extruder(self):
        extruder = self.toolhead.get_extruder()
        if not extruder or not extruder.get_name():
            raise self.printer.command_error(
                "NOZZLE_WIPE_SMART: no active extruder")
        return extruder

    def _require_homed_xyz(self):
        eventtime = self.reactor.monotonic()
        homed_axes = self.toolhead.get_status(eventtime)["homed_axes"]
        missing = [axis.upper() for axis in "xyz" if axis not in homed_axes]
        if missing:
            raise self.printer.command_error(
                "NOZZLE_WIPE_SMART: Please home %s first (G28)."
                % ("".join(missing),))

    def _run_script(self, script):
        self.gcode.run_script_from_command(script)

    def _set_fan(self, fan_pct):
        fan_s = int(max(0.0, min(1.0, fan_pct)) * 255.0)
        self._run_script("M106 S%d" % (fan_s,))

    def _fan_off(self):
        self._run_script("M106 S0")

    def _set_heater(self, extruder, temp):
        self.heaters.set_temperature(extruder.get_heater(), temp, wait=False)

    def _get_temp(self, extruder):
        eventtime = self.reactor.monotonic()
        temp, target = extruder.get_heater().get_temp(eventtime)
        return temp, target

    def _pause_poll(self, poll_ms):
        eventtime = self.reactor.monotonic()
        self.toolhead.get_last_move_time()
        self.reactor.pause(eventtime + poll_ms / 1000.0)

    def _wait_moves(self):
        self.toolhead.wait_moves()

    def _park_at_brush(self, params):
        eventtime = self.reactor.monotonic()
        position = self.toolhead.get_status(eventtime)["position"]
        z_safe = params["z_top"] + params["z_hop"]
        lines = [
            "SAVE_GCODE_STATE NAME=nozzle_wipe_smart_init_state",
            "G90",
            "G0 Z%.6f F%.6f" % (z_safe, params["f_travel"]),
        ]
        if position.y < params["y"]:
            lines.append("G0 Y%.6f F%.6f" % (params["y"], params["f_travel"]))
            lines.append("M400")
        lines.extend([
            "G0 X%.6f Y%.6f F%.6f" % (
                params["x"], params["y"], params["f_travel"]),
            "G0 Z%.6f F%.6f" % (params["z_top"], params["z_speed"]),
            "RESTORE_GCODE_STATE NAME=nozzle_wipe_smart_init_state MOVE=0",
        ])
        self._run_script("\n".join(lines))
        self._wait_moves()

    def _lift_from_brush(self, params):
        z_safe = params["z_top"] + params["z_hop"]
        self._run_script("\n".join([
            "SAVE_GCODE_STATE NAME=nozzle_wipe_smart_lift_state",
            "G90",
            "G0 Z%.6f F%.6f" % (z_safe, params["z_speed"]),
            "RESTORE_GCODE_STATE NAME=nozzle_wipe_smart_lift_state MOVE=0",
        ]))
        self._wait_moves()

    def _wipe_cycle(self, params, passes, setup):
        x_min = params["x"] - params["sx"] / 2.0
        x_max = params["x"] + params["sx"] / 2.0
        y_min = params["y"] - params["sy"] / 2.0
        y_max = params["y"] + params["sy"] / 2.0
        z_wipe = params["z_top"] - params["plunge"]
        lines = [
            "SAVE_GCODE_STATE NAME=nozzle_wipe_smart_cycle_state",
            "G90",
        ]
        if setup:
            lines.extend([
                "G0 X%.6f Y%.6f F%.6f" % (
                    params["x"], params["y"], params["f_travel"]),
                "G0 Z%.6f F%.6f" % (params["z_top"], params["z_speed"]),
                "G1 Z%.6f F%.6f" % (z_wipe, params["z_speed"]),
                "G1 X%.6f Y%.6f F%.6f" % (
                    params["x"], y_min, params["f_wipe"]),
            ])
        else:
            lines.append("G1 X%.6f Y%.6f F%.6f" % (
                params["x"], y_min, params["f_wipe"]))
        for i in range(passes):
            lines.extend([
                "G1 X%.6f Y%.6f F%.6f" % (x_max, y_min, params["f_wipe"]),
                "G1 X%.6f Y%.6f F%.6f" % (x_max, y_max, params["f_wipe"]),
                "G1 X%.6f Y%.6f F%.6f" % (x_min, y_max, params["f_wipe"]),
                "G1 X%.6f Y%.6f F%.6f" % (x_min, y_min, params["f_wipe"]),
                "G1 X%.6f Y%.6f F%.6f" % (
                    params["x"], y_min, params["f_wipe"]),
            ])
        lines.append(
            "RESTORE_GCODE_STATE NAME=nozzle_wipe_smart_cycle_state MOVE=0")
        self._run_script("\n".join(lines))
        self._wait_moves()

    def _abort_outputs(self, extruder=None):
        try:
            if extruder is not None:
                self._set_heater(extruder, 0.0)
        except Exception:
            pass
        try:
            self._fan_off()
        except Exception:
            pass

    cmd_NOZZLE_WIPE_SMART_help = "Blocking smart nozzle wipe"
    def cmd_NOZZLE_WIPE_SMART(self, gcmd):
        if self.active:
            raise gcmd.error("NOZZLE_WIPE_SMART: already active")
        params = self._get_params(gcmd)
        self._require_homed_xyz()
        extruder = self._active_extruder()

        self.active = True
        self.stop_requested = False
        self.phase = "INIT"
        self.mode = params["mode"]
        self.iter = 0
        self.last_error = ""
        wipe_threshold = params["wipe_temp"] - params["wipe_temp_tolerance"]

        try:
            self._set_fan(params["fan_pct"])
            if params["mode"] == "FULL":
                self._set_heater(extruder, params["wipe_temp"])

            self._park_at_brush(params)

            if params["mode"] == "FULL":
                self.phase = "HEAT_WAIT"
                gcmd.respond_info(
                    "NOZZLE_WIPE_SMART: MODE=FULL (heat then wipe)")
            elif params["mode"] == "NOHEAT":
                self.phase = "COOL_WIPE"
                self._wipe_cycle(params, 1, 1)
                gcmd.respond_info(
                    "NOZZLE_WIPE_SMART: MODE=NOHEAT "
                    "(wipe immediately until STOP_TEMP)")
            else:
                self.phase = "FAN_COOL"
                gcmd.respond_info(
                    "NOZZLE_WIPE_SMART: MODE=FAN_ONLY "
                    "(fan until FAN_OFF_TEMP)")

            while self.active and not self.printer.is_shutdown():
                if self.stop_requested:
                    self._abort_outputs(extruder)
                    self.active = False
                    self.phase = "IDLE"
                    gcmd.respond_info("NOZZLE_WIPE_SMART: stop requested")
                    return
                if self.iter >= params["max_iter"]:
                    temp, target = self._get_temp(extruder)
                    self._abort_outputs(extruder)
                    self.active = False
                    self.last_error = (
                        "MAX_ITER reached in %s at %.1fC target %.1fC"
                        % (self.phase, temp, target))
                    raise gcmd.error(
                        "NOZZLE_WIPE_SMART: %s" % (self.last_error,))

                self.iter += 1
                temp, target = self._get_temp(extruder)

                if self.phase == "HEAT_WAIT":
                    if temp >= params["start_temp"]:
                        self._wipe_cycle(params, 1, 1)
                        self.phase = "HEAT_WIPE"
                        gcmd.respond_info("NOZZLE_WIPE_SMART: start wiping")
                    else:
                        self._pause_poll(params["poll_ms"])

                elif self.phase == "HEAT_WIPE":
                    if temp >= wipe_threshold:
                        self._set_heater(extruder, 0.0)
                        self.phase = "COOL_WIPE"
                        gcmd.respond_info(
                            "NOZZLE_WIPE_SMART: reached wipe threshold; "
                            "heater off")
                    else:
                        self._wipe_cycle(params, params["passes"], 0)

                elif self.phase == "COOL_WIPE":
                    if temp <= params["stop_temp"]:
                        self._lift_from_brush(params)
                        self.phase = "FAN_COOL"
                        gcmd.respond_info(
                            "NOZZLE_WIPE_SMART: stop wiping; fan cool")
                    else:
                        self._wipe_cycle(params, params["passes"], 0)

                elif self.phase == "FAN_COOL":
                    if temp <= params["fan_off_temp"]:
                        self._fan_off()
                        self.active = False
                        self.phase = "IDLE"
                        gcmd.respond_info("NOZZLE_WIPE_SMART: done; fan off")
                    else:
                        self._pause_poll(params["poll_ms"])

                else:
                    self._abort_outputs(extruder)
                    self.active = False
                    raise gcmd.error(
                        "NOZZLE_WIPE_SMART: unknown phase %s" % (self.phase,))
        except Exception as e:
            self.last_error = str(e)
            self.active = False
            if not self.printer.is_shutdown():
                self._abort_outputs(extruder)
            self.phase = "ERROR"
            raise
        finally:
            if self.phase != "ERROR":
                self.active = False
                self.stop_requested = False
                if self.phase != "IDLE":
                    self.phase = "IDLE"

    cmd_NOZZLE_WIPE_SMART_STOP_help = "Stop smart nozzle wiping"
    def cmd_NOZZLE_WIPE_SMART_STOP(self, gcmd):
        self.stop_requested = True
        try:
            self._abort_outputs(self._active_extruder())
        except Exception:
            self._abort_outputs()
        if not self.active:
            self.phase = "IDLE"
            self.last_error = ""
        gcmd.respond_info("NOZZLE_WIPE_SMART: stop requested")

def load_config(config):
    return NozzleWipeSmart(config)
