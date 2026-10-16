# Print modes that map the slicer's part-fan command onto the real fan
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The M1's part fan is a CPAP blower that is far stronger and louder than
# slicer filament profiles assume. Rather than edit every filament profile
# down to ~30%, the printer keeps a small set of modes (normal, quiet,
# turbo) and maps the slicer's commanded speed through the selected mode's
# curve. A profile asking for 100% gets 30% in normal mode, and the full
# blower only in turbo.
#
# SET_PART_FAN carries the commanded speed; the M1 M106/M107 macros call it.
# M106.1/M107.1 stay the raw physical path for macros tuned to real speeds.
import logging

class FanModes:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.fan_name = config.get('fan', 'fan')
        self.modes = {}
        self.speed_factors = {}
        for mode in config.getlist('modes', ('normal', 'quiet', 'turbo')):
            mode = mode.strip().lower()
            self.modes[mode] = self._parse_curve(
                config, mode, config.get('%s_curve' % (mode,)))
            factor = config.getfloat('%s_speed_factor' % (mode,), None,
                                     above=0.)
            if factor is not None:
                self.speed_factors[mode] = factor
        self.mode = config.get('default_mode', 'normal').lower()
        if self.mode not in self.modes:
            raise config.error("fan_modes: default_mode '%s' is not one of %s"
                               % (self.mode, ", ".join(self.modes)))
        self.commanded = 0.
        self.output = 0.
        self.fan = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_PART_FAN', self.cmd_SET_PART_FAN,
                               desc=self.cmd_SET_PART_FAN_help)
        gcode.register_command('SET_PRINT_MODE', self.cmd_SET_PRINT_MODE,
                               desc=self.cmd_SET_PRINT_MODE_help)
    def _parse_curve(self, config, mode, text):
        # "commanded:output, ..." pairs, both 0..1, commanded increasing.
        points = []
        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                cmd, out = [float(v) for v in item.split(':')]
            except ValueError:
                raise config.error("fan_modes: %s_curve entry '%s' is not"
                                   " commanded:output" % (mode, item))
            if not (0. <= cmd <= 1. and 0. <= out <= 1.):
                raise config.error("fan_modes: %s_curve values must be"
                                   " within 0..1" % (mode,))
            points.append((cmd, out))
        if len(points) < 2 or points[0][0] != 0. or points[-1][0] != 1.:
            raise config.error("fan_modes: %s_curve must start at 0: and end"
                               " at 1:" % (mode,))
        if any(b[0] <= a[0] for a, b in zip(points, points[1:])):
            raise config.error("fan_modes: %s_curve commanded values must"
                               " increase" % (mode,))
        return points
    def _handle_connect(self):
        printer_fan = self.printer.lookup_object(self.fan_name, None)
        if printer_fan is None or not hasattr(printer_fan, 'fan'):
            raise self.printer.config_error(
                "fan_modes: '%s' is not a [fan] section" % (self.fan_name,))
        self.fan = printer_fan.fan
    def map_speed(self, commanded, mode=None):
        points = self.modes[mode or self.mode]
        for (c0, o0), (c1, o1) in zip(points, points[1:]):
            if commanded <= c1:
                return o0 + (o1 - o0) * (commanded - c0) / (c1 - c0)
        return points[-1][1]
    def _apply(self, commanded):
        self.commanded = commanded
        self.output = self.map_speed(commanded)
        # Queued with the moves, as Klipper's own M106 does.
        self.fan.set_speed_from_command(self.output)
    def _fan_speed(self):
        eventtime = self.printer.get_reactor().monotonic()
        return self.fan.get_status(eventtime)['speed']
    cmd_SET_PART_FAN_help = ("Set the part fan to a commanded speed (0..1),"
                             " mapped through the current print mode")
    def cmd_SET_PART_FAN(self, gcmd):
        self._apply(gcmd.get_float('SPEED', minval=0., maxval=1.))
    cmd_SET_PRINT_MODE_help = "Select the print mode (normal, quiet, turbo)"
    def cmd_SET_PRINT_MODE(self, gcmd):
        mode = gcmd.get('MODE', None)
        if mode is None:
            gcmd.respond_info("Print mode: %s (modes: %s)"
                              % (self.mode, ", ".join(self.modes)))
            return
        mode = mode.lower()
        if mode not in self.modes:
            raise gcmd.error("Unknown print mode '%s' (modes: %s)"
                             % (mode, ", ".join(self.modes)))
        # Re-apply the commanded speed under the new curve, unless something
        # drove the fan directly since (M106.1, a toolhead disconnect): then
        # the fan is not showing the commanded speed and must not be revived.
        follow = abs(self._fan_speed() - self.output) < 0.005
        self.mode = mode
        if follow:
            self._apply(self.commanded)
        factor = self.speed_factors.get(mode)
        if factor is not None:
            gcode = self.printer.lookup_object('gcode')
            gcode.run_script_from_command("M220 S%.0f" % (factor * 100.,))
        gcmd.respond_info("Print mode: %s (part fan %.0f%% commanded ->"
                          " %.0f%%)" % (mode, self.commanded * 100.,
                                        self.map_speed(self.commanded) * 100.))
        logging.info("fan_modes: mode %s", mode)
    def get_status(self, eventtime):
        return {'mode': self.mode,
                'modes': list(self.modes),
                'commanded_speed': self.commanded,
                'output_speed': self.output}

def load_config(config):
    return FanModes(config)
