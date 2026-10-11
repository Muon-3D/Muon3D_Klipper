#!/usr/bin/env python3
# Check that the M1's automatic filament load is still armed.
#
# Copyright (C) 2026  Muon 3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Pushing filament into the toolhead starts the load flow on its own: the
# toolhead filament switch's insert_gcode runs INSERT_FILAMENT_SENSOR, which
# runs FILAMENT_INSERT_FLOW whenever the printer is not printing.
#
# KAN-232 commented insert_gcode out on 2026-08-23 because toolhead:gpio3
# floated on one unit. KAN-239 armed it again once the switch was seen
# tracking filament, and the disarm kept coming back: the comment above the
# line told the next reader to comment it out again, and branches cut from a
# disarmed master carried the disarmed line with them. Jack decided on
# 2026-10-10 that the automatic load stays armed. Turning it off is his
# decision, made in a commit that also changes this script, never a side
# effect of other work.
#
# Each way of switching it off that looks harmless in a diff is checked:
#   - the [filament_switch_sensor] section removed
#   - insert_gcode commented out, emptied, or pointed somewhere else
#   - the INSERT_FILAMENT_SENSOR macro removed, or no longer running
#     FILAMENT_INSERT_FLOW
#   - SET_FILAMENT_SENSOR ... ENABLE=0 in any shipped template, which
#     leaves insert_gcode in place but stops the sensor acting on it
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import check_macro_config as cmc  # noqa: E402

SENSOR = 'filament_switch_sensor filament_sensor_toolhead'
INSERT_MACRO = 'INSERT_FILAMENT_SENSOR'
LOAD_FLOW = 'FILAMENT_INSERT_FLOW'
SENSOR_OFF = re.compile(r'\bSET_FILAMENT_SENSOR\b[^\n]*\bENABLE\s*=\s*0\b',
                        re.IGNORECASE)


def _first_word(value):
    words = value.split()
    return words[0].upper() if words else ''


def _calls(value, macro):
    return re.search(r'(?<![\w])%s(?![\w])' % macro, value,
                     re.IGNORECASE) is not None


def check(config_dir):
    """Return a list of failure strings; empty means the load is armed."""
    failures = []
    sensor_seen = False
    insert_values = []
    macro_bodies = []
    for path in cmc.config_files(config_dir):
        with open(path, encoding='utf-8', errors='replace') as handle:
            sections, _indented = cmc.parse(handle.read())
        rel = os.path.relpath(path, config_dir).replace(os.sep, '/')
        for _line, header, options in sections:
            name = ' '.join(header.split())
            for line, key, value_lines in options:
                value = cmc._join_value(value_lines)
                if SENSOR_OFF.search(value):
                    failures.append(
                        "%s:%d: [%s] %s turns a filament sensor off"
                        % (rel, line, header, key))
            if name == SENSOR:
                sensor_seen = True
                insert_values.extend(
                    (rel, line, cmc._join_value(v))
                    for line, key, v in options if key == 'insert_gcode')
            elif name.lower() == 'gcode_macro ' + INSERT_MACRO.lower():
                macro_bodies.extend(
                    (rel, line, cmc._join_value(v))
                    for line, key, v in options if key == 'gcode')

    if not sensor_seen:
        failures.append("[%s] is missing" % SENSOR)
    elif not insert_values:
        failures.append("[%s] has no insert_gcode (commented out or"
                        " deleted)" % SENSOR)
    for rel, line, value in insert_values:
        if _first_word(value) != INSERT_MACRO:
            failures.append("%s:%d: insert_gcode is %r, not %s"
                            % (rel, line, value, INSERT_MACRO))

    if not macro_bodies:
        failures.append("[gcode_macro %s] is missing" % INSERT_MACRO)
    for rel, line, value in macro_bodies:
        if not _calls(value, LOAD_FLOW):
            failures.append("%s:%d: %s no longer runs %s"
                            % (rel, line, INSERT_MACRO, LOAD_FLOW))
    return failures


def main():
    config_dir = os.path.join(ROOT, 'core', 'M1')
    if not os.path.isdir(config_dir):
        sys.stderr.write("missing directory: %s\n" % config_dir)
        return 2
    failures = check(config_dir)
    if failures:
        sys.stderr.write(
            "The M1's automatic filament load has been switched off.\n"
            "Jack decided on 2026-10-10 that it stays armed. Do not disarm\n"
            "it to work around a fault; report the fault to Jack with\n"
            "QUERY_FILAMENT_SENSOR output. See the comment above\n"
            "insert_gcode in core/M1/core.cfg.\n\n")
        for failure in failures:
            sys.stderr.write("  %s\n" % failure)
        return 1
    print("check_m1_armed_hooks: automatic filament load is armed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
