# KAN-1732: scan a print file before it starts. Pure: muon_guard feeds it
# the file and the limits, and refuses the print on what it returns.
#
# Positions are tracked the way gcode_move does (G90/G91, G92 shifts) in
# toolhead coordinates. After a command that may move the head (a macro such
# as PRINT_START, G28) the position is unknown until the file gives an
# absolute coordinate again; nothing is checked against an unknown.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import re

from . import move_limits

ARGS_R = re.compile('([A-Z_]+|[A-Z*])')   # gcode.GCodeDispatch.args_r
SIMPLE_R = re.compile(r'G[0-3](?: +[A-Z][-+0-9.]*)* *$')
EXT_R = re.compile(r'(\S+)=(\S+)')
MAX_VIOLATIONS = 20

Limits = collections.namedtuple(
    'Limits', 'z_floor no_go no_go_z axis_min axis_max max_temp')

MOVES = ('G0', 'G1', 'G2', 'G3')
# temperature commands: parameters holding a target, and the heater
HEATER_CMDS = {'M104': ('extruder', 'SR'), 'M109': ('extruder', 'SR'),
               'M140': ('heater_bed', 'SR'), 'M190': ('heater_bed', 'SR')}
# Commands that do not move the head. Anything else (a macro, G28, PAUSE,
# RESTORE_GCODE_STATE, ...) leaves the position and G90/G91 unknown.
STILL = set("""
G4 G20 G21 G90 G91 G92 M82 M83 M84 M18 M73 M104 M105 M106 M107 M109 M114
M115 M117 M118 M140 M141 M190 M191 M204 M205 M220 M221 M400 M486 M110
SET_VELOCITY_LIMIT SET_PRESSURE_ADVANCE SET_INPUT_SHAPER SET_PRINT_STATS_INFO
EXCLUDE_OBJECT_DEFINE EXCLUDE_OBJECT_START EXCLUDE_OBJECT_END EXCLUDE_OBJECT
SET_FAN_SPEED SET_HEATER_TEMPERATURE TEMPERATURE_WAIT
SET_TEMPERATURE_FAN_TARGET SET_DISPLAY_TEXT RESPOND SAVE_GCODE_STATE
GET_POSITION ECHO STATUS SPEED_SLOW_PRE_PURGE_FOR_TPU
SPEED_NORMAL__POST_PURGE_FOR_TPU SET_PAUSE_AT_LAYER SET_PAUSE_NEXT_LAYER
BED_MESH_PROFILE _SET_MPC_MATERIAL SET_GCODE_OFFSET
""".split())
# ...and of those that do, the ones that home: G92 shifts end there
HOMES = ('G28', 'PRINT_START', 'CHOME')


def parse_line(line):
    """(cmd, params, text) the way gcode._process_commands splits a line;
    cmd is '' for a blank or comment line."""
    text = line.strip()
    cpos = text.find(';')
    if cpos >= 0:
        text = text[:cpos]
    up = text.upper()
    toks = up.split()
    if toks and toks[0] in MOVES and SIMPLE_R.match(up):
        # the slicer's usual "G1 X1.5 Y2 E.1": same result, no regex split
        return toks[0], dict((t[0], t[1:]) for t in toks), text
    parts = ARGS_R.split(up)
    if ''.join(parts[:2]) == 'N':
        cmd = ''.join(parts[3:5]).strip()
    else:
        cmd = ''.join(parts[:3]).strip()
    params = {parts[i]: parts[i + 1].strip()
              for i in range(1, len(parts), 2)}
    return cmd, params, text


class Scanner:
    def __init__(self, limits, allowed=None):
        self.lim = limits
        self.allowed = allowed
        self.absolute = True
        self.pos = [None, None, None]
        self.base = [0., 0., 0.]
        self.violations = []

    def _unknown(self):
        self.pos = [None, None, None]
        self.absolute = None

    def _target(self, params):
        end = list(self.pos)
        for i, axis in enumerate('XYZ'):
            if axis not in params:
                continue
            v = float(params[axis])
            if self.absolute is None:
                end[i] = None
            elif self.absolute:
                end[i] = None if self.base[i] is None else v + self.base[i]
            else:
                end[i] = None if end[i] is None else end[i] + v
        return end

    def _move(self, cmd, params):
        lim = self.lim
        start, end = self.pos, self._target(params)
        self.pos = end
        if end[2] is not None and end[2] < lim.z_floor - move_limits.EPS \
                and (start[2] is None or end[2] < start[2]):
            return "goes to Z %.3f, below the Z floor of %g" % (
                end[2], lim.z_floor)
        if None in start[:2] or None in end[:2]:
            for i in (0, 1):
                v = end[i]
                if v is not None and not (lim.axis_min[i] - move_limits.EPS
                                          <= v <= lim.axis_max[i]
                                          + move_limits.EPS):
                    return "goes to %s %.3f, out of range" % ('XY'[i], v)
            return None
        if cmd in ('G2', 'G3'):
            pts = move_limits.arc_points(
                start, end, (float(params.get('I', 0.)),
                             float(params.get('J', 0.))), cmd == 'G2')
        else:
            pts = [tuple(start[:2]), tuple(end[:2])]
        for x, y in pts:
            if not (lim.axis_min[0] - move_limits.EPS <= x
                    <= lim.axis_max[0] + move_limits.EPS
                    and lim.axis_min[1] - move_limits.EPS <= y
                    <= lim.axis_max[1] + move_limits.EPS):
                return "goes to X %.3f Y %.3f, out of range" % (x, y)
        zs = [z for z in (start[2], end[2]) if z is not None]
        if zs and min(zs) < lim.no_go_z and start[:2] != end[:2]:
            rect = move_limits.path_hits_rects(pts, lim.no_go)
            if rect is not None:
                return ("crosses the notch (x %g..%g, y %g..%g) below Z %g"
                        % (rect[0], rect[2], rect[1], rect[3], lim.no_go_z))
        return None

    def _g92(self, params):
        axes = [a for a in 'XYZ' if a in params]
        if not any(a in params for a in 'XYZE'):
            axes = 'XYZ'
        for i, axis in enumerate('XYZ'):
            if axis in axes:
                v = float(params.get(axis, 0.))
                self.base[i] = (None if self.pos[i] is None
                                else self.pos[i] - v)

    def _temp(self, cmd, params):
        if cmd == 'SET_HEATER_TEMPERATURE':
            heater = params.get('HEATER', '').lower()
            values = [params.get('TARGET')]
        else:
            heater, keys = HEATER_CMDS[cmd]
            values = [params.get(k) for k in keys]
        limit = self.lim.max_temp.get(heater)
        if limit is None:
            return None
        for v in values:
            if v is not None and float(v) > limit:
                return "sets %s to %s, above its max_temp of %g" % (
                    heater, v, limit)
        return None

    def line(self, line):
        cmd, params, text = parse_line(line)
        if not cmd:
            return None
        name = cmd.split()[0] if ' ' in cmd else cmd
        if self.allowed is not None:
            ok, why = self.allowed(cmd, params, line)
            if not ok:
                return why
        try:
            if name in MOVES:
                return self._move(name, params)
            if name == 'G90':
                self.absolute = True
            elif name == 'G91':
                self.absolute = False
            elif name == 'G92':
                self._g92(params)
            elif name in HEATER_CMDS:
                return self._temp(name, params)
            elif name == 'SET_HEATER_TEMPERATURE':
                ext = dict((k.upper(), v) for k, v in EXT_R.findall(text))
                return self._temp(name, ext)
            elif name in HOMES:
                self._unknown()
                self.base = [0., 0., 0.]
            elif name not in STILL:
                self._unknown()
        except ValueError:
            return "cannot be parsed"
        return None

    def scan(self, lines):
        for lineno, line in enumerate(lines, 1):
            why = self.line(line)
            if why:
                self.violations.append((lineno, line.strip(), why))
                if len(self.violations) >= MAX_VIOLATIONS:
                    break
        return self.violations


def scan(lines, limits, allowed=None):
    """[(line number, line, why)] for the first MAX_VIOLATIONS problems."""
    return Scanner(limits, allowed).scan(lines)
