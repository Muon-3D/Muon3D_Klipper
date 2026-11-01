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
# With a speed_table, each curve maps the command to a target blower speed
# in rpm instead of a duty, and the one table converts rpm to duty (ADR 0056).
# The B1 blower does not start below ~6% duty, holds ~3200 rpm to ~8%, and
# gains less speed per step of duty towards the top, so a duty curve gives no
# air for small commands and uneven steps above them. A curve in rpm gives an
# even step of air per step of command, and the table is the only part tied
# to the blower. Open loop: the tachometer is not read.
#
# SET_PART_FAN carries the commanded speed; the M1 M106/M107 macros call it.
# M106.1/M107.1 stay the raw physical path for macros tuned to real speeds.
import logging

class FanModes:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.fan_name = config.get('fan', 'fan')
        self.table = None
        table = config.get('speed_table', None)
        if table is not None:
            self.table = self._parse_table(config, table)
        out_max = self.table[-1][0] if self.table else 1.
        self.modes = {}
        self.speed_factors = {}
        for mode in config.getlist('modes', ('normal', 'quiet', 'turbo')):
            mode = mode.strip().lower()
            self.modes[mode] = self._parse_curve(
                config, mode, config.get('%s_curve' % (mode,)), out_max)
            factor = config.getfloat('%s_speed_factor' % (mode,), None,
                                     above=0.)
            if factor is not None:
                self.speed_factors[mode] = factor
        self.mode = config.get('default_mode', 'normal').lower()
        if self.mode not in self.modes:
            raise config.error("fan_modes: default_mode '%s' is not one of %s"
                               % (self.mode, ", ".join(self.modes)))
        self.commanded = 0.
        # Only with a speed_table; None otherwise.
        self.target_rpm = 0. if self.table else None
        self.output = 0.
        self.fan = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_PART_FAN', self.cmd_SET_PART_FAN,
                               desc=self.cmd_SET_PART_FAN_help)
        gcode.register_command('SET_PRINT_MODE', self.cmd_SET_PRINT_MODE,
                               desc=self.cmd_SET_PRINT_MODE_help)
    def _parse_pairs(self, config, option, text, form):
        points = []
        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                a, b = [float(v) for v in item.split(':')]
            except ValueError:
                raise config.error("fan_modes: %s entry '%s' is not %s"
                                   % (option, item, form))
            points.append((a, b))
        return points
    def _parse_table(self, config, text):
        # "rpm:duty, ..." from a measured sweep, both increasing. The first
        # point is the lowest speed at which the blower follows its duty.
        points = self._parse_pairs(config, 'speed_table', text, 'rpm:duty')
        if len(points) < 2:
            raise config.error("fan_modes: speed_table needs at least two"
                               " rpm:duty points")
        if any(not (rpm > 0. and 0. < duty <= 1.) for rpm, duty in points):
            raise config.error("fan_modes: speed_table rpm must be above 0"
                               " and duty within 0..1")
        if any(b[0] <= a[0] or b[1] <= a[1]
               for a, b in zip(points, points[1:])):
            raise config.error("fan_modes: speed_table rpm and duty must"
                               " both increase")
        return points
    def _parse_curve(self, config, mode, text, out_max):
        # "commanded:output, ..." pairs, commanded 0..1 and increasing.
        # output is a duty 0..1, or rpm up to the table's top with a table.
        points = self._parse_pairs(config, '%s_curve' % (mode,), text,
                                   'commanded:output')
        for cmd, out in points:
            if not (0. <= cmd <= 1. and 0. <= out <= out_max):
                raise config.error("fan_modes: %s_curve commanded must be"
                                   " within 0..1 and output within 0..%g"
                                   % (mode, out_max))
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
    @staticmethod
    def _interp(points, x):
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            if x <= x1:
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        return points[-1][1]
    def duty_for_rpm(self, rpm):
        # A speed above 0 but under the table's first point is raised to that
        # point: the blower has no steady speed below it.
        if rpm <= 0.:
            return 0.
        if rpm <= self.table[0][0]:
            return self.table[0][1]
        return self._interp(self.table, rpm)
    def map_speed(self, commanded, mode=None):
        # The curve's output: a duty, or a target rpm with a speed_table.
        return self._interp(self.modes[mode or self.mode], commanded)
    def map_duty(self, commanded, mode=None):
        out = self.map_speed(commanded, mode)
        if self.table is None:
            return out
        return self.duty_for_rpm(out)
    def _apply(self, commanded):
        self.commanded = commanded
        if self.table is not None:
            self.target_rpm = self.map_speed(commanded)
        self.output = self.map_duty(commanded)
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
        if follow and self.table is not None:
            gcmd.respond_info("Print mode: %s (part fan %.0f%% commanded ->"
                              " %.0f rpm, %.1f%% duty)"
                              % (mode, self.commanded * 100.,
                                 self.target_rpm, self.output * 100.))
        elif follow:
            gcmd.respond_info("Print mode: %s (part fan %.0f%% commanded ->"
                              " %.0f%%)" % (mode, self.commanded * 100.,
                                            self.output * 100.))
        else:
            gcmd.respond_info("Print mode: %s (part fan left as it is: it"
                              " was set directly, not by M106)" % (mode,))
        logging.info("fan_modes: mode %s", mode)
    def get_status(self, eventtime):
        return {'mode': self.mode,
                'modes': list(self.modes),
                'commanded_speed': self.commanded,
                'target_rpm': self.target_rpm,
                'output_speed': self.output}

def load_config(config):
    return FanModes(config)
