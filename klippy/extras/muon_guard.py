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

DEFAULT_DEV_CONFIG = '/home/printer_data/devmode_custom_config/core.cfg'
CACHE_SECONDS = 1.

# The dispatcher resolves these three when the command line carries free
# text after the name ("M117 Printing benchy"), so the first word is the
# command (gcode.py cmd_default).
FREE_TEXT_COMMANDS = ('M117', 'M118', 'M23')

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


RANGE_RE = re.compile(r'^(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)$')


def _parse_spec(spec, name, error):
    m = RANGE_RE.match(spec)
    if m is None:
        return ('glob', spec.upper())
    lo, hi = float(m.group(1)), float(m.group(2))
    if lo > hi:
        raise error("[muon_guard] client_allow: '%s' has an empty range "
                    "'%s'" % (name, spec))
    return ('range', lo, hi)


def parse_allowlist(entries, error):
    """entries: 'NAME' or 'NAME TOKEN ...'. Returns {NAME: [rule, ...]},
    where a rule is {'cons': {KEY: spec}, 'open': bool}. A bare NAME allows
    any parameters. With tokens the entry is closed: only the listed KEYs
    may be sent, each held to its spec (a glob such as G28*, or a numeric
    range lo..hi), and a key that is left out is simply not sent. The
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


def _spec_matches(spec, value):
    value = str(value).strip().lstrip('=').strip()
    if spec[0] == 'range':
        try:
            number = float(value)
        except ValueError:
            return False
        return spec[1] <= number <= spec[2]
    return fnmatch.fnmatchcase(value.upper(), spec[1])


def rule_matches(rule, params):
    for key, value in params.items():
        spec = rule['cons'].get(key)
        if spec is None:
            if not rule['open']:
                return False
        elif not _spec_matches(spec, value):
            return False
    return True


class MuonGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.dev_config_path = config.get('dev_config_path',
                                          DEFAULT_DEV_CONFIG)
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
        if any(r['open'] and not r['cons'] for r in rules):
            return True, ""
        if self.gcode.is_traditional_gcode(name):
            # the dispatcher also lists the command's own letter and
            # number (G28 -> {'G': '28'}) among the parameters
            params = dict((k, v) for k, v
                          in gcmd.get_command_parameters().items()
                          if not (k == name[0] and v == name[1:]))
        else:
            gcmd = self.gcode._get_extended_params(gcmd)
            params = gcmd.get_command_parameters()
        if any(rule_matches(r, params) for r in rules):
            return True, ""
        return False, ("'%s' is on the client allowlist only with limited "
                       "parameters, and these are not among them" % (name,))

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
