# Fail-closed dev-mode check and client command allowlist
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# KAN-1730 (epic KAN-1729, printer security). Outside developer mode a
# client may only START the commands this file's config lists. A client is
# anything that reaches the printer through one of the doors in gcode.py:
# webhooks gcode/script and the restart endpoints (Moonraker, so Control,
# the panel, the slicer and the phone app), the serial pty, and the lines of
# a printed file. Everything the printer runs on its own account -- macro
# bodies, idle_timeout, delayed_gcode, the filament and bed sensors,
# homing_scope cleanup, other extras -- is not checked, and neither is
# what a macro a client started runs inside. The tag follows the top-level
# origin: `G28` from a client passes, and the SET_KINEMATIC_POSITION inside
# the G28 macro runs; the same SET_KINEMATIC_POSITION sent by the client
# directly is refused.
#
# It is an allowlist, not a deny list: a client can reach a refused
# primitive through any macro or extra that forwards to it, so the only safe
# question is "was this command expected from a client". A new macro is a
# reviewed line in [muon_guard] client_allow.
#
# Dev mode. MuonOS puts the printer in developer mode by starting Klipper on
# a writable clone of the config (/home/printer_data/devmode_custom_config)
# instead of the read-only OEM tree. The guard asks Klipper which file it
# was started from. That path is fixed when the process starts, comes from
# an env file only the Aux API writes, and no Moonraker client can change
# it, so it is harder to forge than a flag, a file under a root a client can
# write, or an HTTP answer. It FAILS CLOSED: dev mode is reported only when
# the running config is exactly the dev-mode clone, reached through no
# symlink, and is a readable regular file. Any error, a missing file or any
# other path (the OEM tree, a second model's config, a client-writable
# path) is NOT dev mode.
import fnmatch
import logging
import os
import re
import shlex
import time

from . import move_limits

DEFAULT_DEV_CONFIG = '/home/printer_data/devmode_custom_config/core.cfg'
CACHE_SECONDS = 1.

# The dispatcher resolves these three when the command line carries free
# text after the name ("M117 Printing benchy"), so the first word is the
# command (gcode.py cmd_default).
FREE_TEXT_COMMANDS = ('M117', 'M118', 'M23')

# Commands that start or resume a print; blocked while the saved Z offset
# is out of range (the probe ignores it and reports z_offset_invalid)
PRINT_STARTS = ('PRINT_START', 'SDCARD_PRINT_FILE', 'M24')

# Moves a client may send, held to the Z floor and the notch (KAN-1731)
MOVES = ('G0', 'G1', 'G2', 'G3')

NAME_RE = re.compile(r'^[A-Z0-9_.]+$')
KEY_RE = re.compile(r'^[A-Z0-9_]+$')


def _detect(config_file, dev_config_path):
    if not isinstance(config_file, str) or not config_file:
        return False, "guard on: Klipper reports no config path"
    if not isinstance(dev_config_path, str) or not dev_config_path:
        return False, "guard on: no dev-mode config path is configured"
    if not os.path.isabs(config_file):
        return False, "guard on: the config path is not absolute"
    running = os.path.normpath(config_file)
    if running != os.path.normpath(dev_config_path):
        return False, ("guard on: the running config is not the dev-mode "
                       "clone (%s)" % (running,))
    # MuonOS writes the resolved path, so a link anywhere in the chain
    # means the file is not the one it set up.
    if os.path.realpath(running) != running or os.path.islink(running):
        return False, "guard on: the dev-mode config path goes through a link"
    if not os.path.isfile(running):
        return False, "guard on: the dev-mode config is not a regular file"
    if not os.access(running, os.R_OK):
        return False, "guard on: the dev-mode config is not readable"
    return True, "dev mode: running the dev-mode config clone"


def detect_dev_mode(config_file, dev_config_path):
    """(is_dev_mode, reason). Any error is NOT dev mode."""
    try:
        return _detect(config_file, dev_config_path)
    except Exception as e:
        logging.exception("muon_guard: dev-mode check failed")
        return False, "guard on: dev-mode check failed (%s)" % (
            type(e).__name__,)


# lo..hi, where hi may name a live limit instead of a number: @extruder_max_temp
# or @bed_max_temp, read from the printer config when the command arrives
RANGE_RE = re.compile(r'^(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?|@[a-z_]+)$')
LIMITS = {'@extruder_max_temp': ('extruder', 'max_temp'),
          '@bed_max_temp': ('heater_bed', 'max_temp')}


def _parse_spec(spec, name, error):
    if spec.startswith('~'):
        try:
            return ('regex', re.compile(spec[1:], re.IGNORECASE))
        except re.error:
            raise error("[muon_guard] client_allow: '%s' has a bad pattern "
                        "'%s'" % (name, spec))
    m = RANGE_RE.match(spec)
    if m is None:
        return ('glob', spec.upper())
    lo = float(m.group(1))
    if m.group(2).startswith('@'):
        if m.group(2) not in LIMITS:
            raise error("[muon_guard] client_allow: '%s' names an unknown "
                        "limit '%s'" % (name, m.group(2)))
        return ('range', lo, m.group(2))
    hi = float(m.group(2))
    if lo > hi:
        raise error("[muon_guard] client_allow: '%s' has an empty range "
                    "'%s'" % (name, spec))
    return ('range', lo, hi)


def parse_allowlist(entries, error):
    """entries: 'NAME' or 'NAME TOKEN ...'. Returns {NAME: [rule, ...]},
    where a rule is {'cons': {KEY: spec}, 'open': bool}. A bare NAME allows
    any parameters. With tokens the entry is closed: only the listed KEYs
    may be sent, each held to its spec (a glob such as G28*, a numeric
    range lo..hi, or a whole-value regex written ~pattern), and a key
    that is left out is simply not sent. The
    token '-' allows no parameters at all, and '...' lets other parameters
    through unchecked. A repeated NAME gives alternatives."""
    rules = {}
    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        try:
            parts = shlex.split(entry)
        except ValueError:
            raise error("[muon_guard] client_allow: bad entry '%s'"
                        % (entry,))
        name = parts[0]
        if not NAME_RE.match(name):
            raise error("[muon_guard] client_allow: '%s' is not an upper "
                        "case command name" % (name,))
        rule = {'cons': {}, 'open': len(parts) == 1}
        for token in parts[1:]:
            if token == '-':
                continue
            if token == '...':
                rule['open'] = True
                continue
            key, eq, spec = token.partition('=')
            key = key.strip()
            if not eq or not KEY_RE.match(key) or not spec:
                raise error("[muon_guard] client_allow: '%s' needs "
                            "PARAM=pattern, '-' or '...', not '%s'"
                            % (name, token))
            rule['cons'][key] = _parse_spec(spec, name, error)
        rules.setdefault(name, []).append(rule)
    if not rules:
        raise error("[muon_guard] client_allow is empty: that would refuse "
                    "every client command")
    return rules


def _spec_matches(spec, value, limit=None):
    value = str(value).strip().lstrip('=').strip()
    if spec[0] == 'range':
        try:
            number = float(value)
        except ValueError:
            return False
        hi = spec[2]
        if isinstance(hi, str):
            hi = limit(hi)
        return spec[1] <= number <= hi
    if spec[0] == 'regex':
        return spec[1].fullmatch(value) is not None
    return fnmatch.fnmatchcase(value.upper(), spec[1])


def rule_matches(rule, params, limit=None):
    for key, value in params.items():
        spec = rule['cons'].get(key)
        if spec is None:
            if not rule['open']:
                return False
        elif not _spec_matches(spec, value, limit):
            return False
    return True


class MuonGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.dev_config_path = config.get('dev_config_path',
                                          DEFAULT_DEV_CONFIG)
        # Largest total offset a client may leave in place, from repeated
        # SET_GCODE_OFFSET calls (X, Y, Z in mm), and the largest probe
        # z_offset Z_OFFSET_APPLY_PROBE may leave to be saved; the probe
        # itself refuses a larger saved value at start (load_cell_probe.py).
        xy = config.getfloat('max_gcode_xy_offset', 10., above=0.)
        self.max_offset = (xy, xy, config.getfloat('max_gcode_z_offset', 2.,
                                                   above=0.))
        self.max_probe_z_offset = config.getfloat('max_probe_z_offset', 2.,
                                                  above=0.)
        # KAN-1731: once Z is homed a client move may not go below this
        # plate-side Z (gcode Z without G92 shifts, SET_GCODE_OFFSET kept),
        # nor cross the notch below its traverse Z (bed_mesh's
        # no_go_region_1 / no_go_traverse_z; test/muon_guard checks they
        # agree). Macros -- homing, probing, purge -- are not checked.
        self.z_floor = config.getfloat('client_z_floor', 0.)
        rect = [float(v) for v in config.getlist('client_no_go_region',
                                                 '75, 0, 125, 34')]
        if len(rect) != 4 or rect[0] >= rect[2] or rect[1] >= rect[3]:
            raise config.error("client_no_go_region must be xmin, ymin,"
                               " xmax, ymax")
        self.no_go = [tuple(rect)]
        self.no_go_z = config.getfloat('client_no_go_traverse_z', 6.)
        entries = config.getlist('client_allow', sep='\n')
        self.rules = parse_allowlist(entries, self.printer.config_error)
        self.refused = 0
        self.last_refusal = ""
        self._cache = None
        self.gcode.register_client_gate(self._gate)

    # Dev-mode detector
    def _dev_mode(self):
        now = time.monotonic()
        cache = self._cache
        if cache is not None and now - cache[0] < CACHE_SECONDS:
            return cache[1], cache[2]
        try:
            config_file = self.printer.get_start_args().get('config_file')
            on, why = detect_dev_mode(config_file, self.dev_config_path)
        except Exception as e:
            logging.exception("muon_guard: dev-mode lookup failed")
            on = False
            why = "guard on: dev-mode lookup failed (%s)" % (
                type(e).__name__,)
        self._cache = (now, on, why)
        return on, why

    def is_dev_mode(self):
        return self._dev_mode()[0]

    # Entry gate: called by gcode.py for each command line a client door
    # hands in, before the command runs.
    def _decide(self, origin, gcmd):
        cmd = gcmd.get_command()
        if not cmd:
            return True, ""
        name = cmd
        if ' ' in cmd:
            first = cmd.split()[0]
            if first in FREE_TEXT_COMMANDS:
                name = first
        rules = self.rules.get(name)
        if rules is None:
            # Nothing is registered under this name, so nothing will run:
            # the dispatcher only says "Unknown command". Refusing it would
            # stop a print on a slicer comment-command (M205, M486, ...)
            # that does nothing. A command that exists and is not listed is
            # refused.
            if self.gcode.gcode_handlers.get(name) is None:
                return True, ""
            return False, "'%s' is not on the client allowlist" % (name,)
        if name in MOVES:
            ok, why = self._check_move(name, self._traditional_params(
                name, gcmd))
            if not ok:
                return ok, why
        if any(r['open'] and not r['cons'] for r in rules):
            return self._check_state(name, {})
        if self.gcode.is_traditional_gcode(name):
            # the dispatcher also lists the command's own letter and
            # number (G28 -> {'G': '28'}) among the parameters
            params = dict((k, v) for k, v
                          in gcmd.get_command_parameters().items()
                          if not (k == name[0] and v == name[1:]))
        else:
            gcmd = self.gcode._get_extended_params(gcmd)
            params = gcmd.get_command_parameters()
        if any(rule_matches(r, params, self._limit) for r in rules):
            return self._check_state(name, params)
        return False, ("'%s' is on the client allowlist only with limited "
                       "parameters, and these are not among them" % (name,))

    def _traditional_params(self, name, gcmd):
        return dict((k, v) for k, v in gcmd.get_command_parameters().items()
                    if not (k == name[0] and v == name[1:]))

    # KAN-1731: where the move goes, in toolhead coordinates, the way
    # gcode_move.cmd_G1 / gcode_arcs compute it.
    def _check_move(self, name, params):
        gm = self.printer.lookup_object('gcode_move')
        homed = self.printer.lookup_object('toolhead').get_status(
            0.)['homed_axes']
        start = list(gm.last_position[:3])
        end = list(start)
        for i, axis in enumerate('XYZ'):
            if axis in params:
                v = float(params[axis])
                end[i] = (v + gm.base_position[i] if gm.absolute_coord
                          else start[i] + v)
        if 'z' in homed:
            floor = self.z_floor + gm.homing_position[2]
            if end[2] < floor - move_limits.EPS and end[2] < start[2]:
                return False, ("that move goes to Z %.3f, below the Z floor"
                               " of %.3f" % (end[2] - gm.homing_position[2],
                                             self.z_floor))
        if ('x' in homed and 'y' in homed
                and min(start[2], end[2]) < self.no_go_z
                and (start[0], start[1]) != (end[0], end[1])):
            if name in ('G2', 'G3'):
                pts = move_limits.arc_points(
                    start, end, (float(params.get('I', 0.)),
                                 float(params.get('J', 0.))), name == 'G2')
            else:
                pts = [start[:2], end[:2]]
            rect = move_limits.path_hits_rects(pts, self.no_go)
            if rect is not None:
                return False, ("that move crosses the notch (x %g..%g, y "
                               "%g..%g) below Z %g" % (
                                   rect[0], rect[2], rect[1], rect[3],
                                   self.no_go_z))
        return True, ""

    def _limit(self, name):
        # A live limit from the printer config, e.g. the extruder max_temp.
        # Any failure to read it raises, and the gate refuses.
        section, option = LIMITS[name]
        settings = self.printer.lookup_object('configfile').get_status(
            0.)['settings']
        return float(settings[section][option])

    # Limits on the state a call leaves behind, which a per-call bound on
    # its parameters cannot see: SET_GCODE_OFFSET adds up across calls, and
    # Z_OFFSET_APPLY_PROBE turns that sum into the probe's saved z_offset.
    def _check_state(self, name, params):
        if name == 'SET_GCODE_OFFSET':
            home = self.printer.lookup_object('gcode_move').homing_position
            for i, axis in enumerate('XYZ'):
                if axis in params:
                    total = float(params[axis])
                elif axis + '_ADJUST' in params:
                    total = home[i] + float(params[axis + '_ADJUST'])
                else:
                    continue
                # Past the limit is refused, except a step toward zero: a
                # saved offset PRINT_START applied may already be large, and
                # a babystep back must always work.
                if (not abs(total) <= self.max_offset[i]
                        and not abs(total) < abs(home[i])):
                    return False, ("that would leave the %s offset at "
                                   "%.3f mm, past +-%.1f mm"
                                   % (axis, total, self.max_offset[i]))
        elif name in PRINT_STARTS:
            for pname in ('probe', 'load_cell_probe'):
                obj = self.printer.lookup_object(pname, None)
                if obj is None:
                    continue
                status = obj.get_status(0.)
                if status.get('z_offset_invalid'):
                    from . import probe
                    return False, probe.invalid_z_offset_message(
                        status['z_offset_saved'], self.max_probe_z_offset)
        elif name == 'Z_OFFSET_APPLY_PROBE':
            home = self.printer.lookup_object('gcode_move').homing_position
            probe = (self.printer.lookup_object('probe', None)
                     or self.printer.lookup_object('load_cell_probe', None))
            if probe is None:
                return False, "there is no probe to apply the offset to"
            new = probe.get_offsets()[2] - home[2]
            if not abs(new) <= self.max_probe_z_offset:
                return False, ("that would save a probe z_offset of %.3f "
                               "mm, past +-%.1f mm"
                               % (new, self.max_probe_z_offset))
        return True, ""

    def _gate(self, origin, gcmd):
        try:
            if self._dev_mode()[0]:
                return
            ok, why = self._decide(origin, gcmd)
        except Exception:
            logging.exception("muon_guard: client check failed")
            ok = False
            why = "the guard could not check it"
        if ok:
            return
        self.refused += 1
        self.last_refusal = why
        logging.warning("muon_guard: refused %s command: %s (%s)", origin,
                        gcmd.get_commandline(), why)
        if why.startswith('Saved Z offset'):
            raise gcmd.error(why)
        raise gcmd.error(
            "Refused: %s. Dev mode is off, so a client may only start the "
            "commands on the printer's allowlist. Turn on dev mode at the "
            "printer to run it." % (why,))

    def get_status(self, eventtime):
        on, why = self._dev_mode()
        return {'dev_mode': on, 'reason': why, 'enforcing': not on,
                'refused': self.refused, 'last_refusal': self.last_refusal}


def load_config(config):
    return MuonGuard(config)
