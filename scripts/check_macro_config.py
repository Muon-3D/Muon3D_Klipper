#!/usr/bin/env python3
# Check that the shipped Klipper configuration says what it appears to say.
#
# Copyright (C) 2026  Muon 3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Klipper reads config with RawConfigParser(strict=False, ...) after cutting
# every line at its first '#'. Three things that reads as fine to a human
# are silently something else to that parser:
#
#   1. An indented `[section]` header. configparser's SECTCRE is anchored at
#      column 0, so an indented header is a continuation line of whatever
#      value came before it. The section is never registered; its keys, at
#      column 0 further down, become keys of the still-open previous
#      section. That was KAN-297: `    [gcode_macro TEST_SPEED]` folded the
#      whole macro into WEAR_TEST's gcode and then replaced WEAR_TEST's own
#      description and gcode with the speed test's.
#
#   2. A duplicate key in one section. strict=False keeps the last value and
#      says nothing. That is also how (1) does its damage, so it is checked
#      on its own: a second `gcode:` in a macro is a bug whatever put it
#      there.
#
#   3. A bare undefined name in a template. Klipper renders with jinja2's
#      default Undefined, so `{% if not is_paused %}` where is_paused was
#      never assigned is a constant True, not an error. That was KAN-79, and
#      it is the half of the silent-constant class that
#      check_macro_status_keys.py records as out of its reach: that script
#      resolves `printer.<obj>.<key>` reads, and a bare name has no printer
#      prefix for it to see. KAN-296.
#
# The template check parses each gcode-carrying option with the same
# delimiters Klipper uses -- Environment('{%', '%}', '{', '}') -- and walks
# the AST for every name the template reads. A name that is read but never
# assigned anywhere in the same template, and is not in the context Klipper
# supplies, is a defect. For a gcode_macro that context is its variable_*
# declarations, `printer`, the four action_* functions, `params` and
# `rawparams` (klippy/extras/gcode_macro.py, create_template_context and
# GCodeMacro.cmd). Every other template option gets the same context without
# params, rawparams and variables.
#
# Why not jinja2.meta.find_undeclared_variables: since 2.9 an `{% if %}` body
# is a branch scope, and a name assigned inside one is recorded as resolved
# from the context in case the branch is not taken. Nearly every shipped
# macro sets a name under an `{% if %}` and reads it in the same block, so
# that call reports dozens of correct reads. The defect being caught is a
# name assigned *nowhere* in the template, which is a flow-insensitive
# question, and answering it flow-insensitively has no false positives.
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

try:
    import jinja2
    from jinja2 import nodes
except ImportError:  # pragma: no cover - CI installs klippy-requirements
    jinja2 = None

# Names jinja2 itself defines inside a template body.
JINJA_BUILTINS = frozenset(('loop', 'self', 'varargs', 'kwargs'))

# klippy/extras/gcode_macro.py: PrinterGCodeMacro.create_template_context
TEMPLATE_CONTEXT = frozenset((
    'printer',
    'action_emergency_stop',
    'action_respond_info',
    'action_raise_error',
    'action_call_remote_method',
))
# klippy/extras/gcode_macro.py: GCodeMacro.cmd adds these on top
MACRO_CONTEXT = TEMPLATE_CONTEXT | frozenset(('params', 'rawparams'))

# Anchored at column 0, as configparser's SECTCRE is. Klipper cuts the line
# at '#' before matching, so no comment handling is needed once that is done.
SECTION = re.compile(r"^\[(?P<header>.+)\]\s*$")
# The same shape after leading whitespace: what a section header looks like
# to a human, and what a continuation line looks like to configparser. The
# first word is required to be a plausible module name so that a bracketed
# G-code argument in a macro body does not trip it.
INDENTED_SECTION = re.compile(
    r"^\s+\[(?P<header>[a-z_][a-z0-9_]*(?:\s+[^\]]+)?)\]\s*$")
# configparser's option regex for a non-indented line: key, then ':' or '='.
OPTION = re.compile(r"^(?P<option>[^\s\[][^:=]*?)\s*[:=]\s*(?P<value>.*)$")
MACRO_VARIABLE = re.compile(r"^variable_(?P<name>\w+)$")


def config_files(config_dir):
    for dirpath, _dirnames, filenames in os.walk(config_dir):
        for filename in sorted(filenames):
            if filename.endswith('.cfg'):
                yield os.path.join(dirpath, filename)


def _klipper_lines(text):
    """The file as Klipper's ConfigFileReader hands it to configparser."""
    lines = []
    for line in text.replace('\r\n', '\n').split('\n'):
        pos = line.find('#')
        if pos >= 0:
            line = line[:pos]
        lines.append(line)
    return lines


def parse(text):
    """Sections in file order, each a list of (line, key, value).

    Mirrors what configparser would build from the same text, and records
    what it would silently discard along the way: indented headers and
    repeated keys. `value` is the option's text with its continuation lines
    joined by '\\n', which is what configparser hands to the template loader.
    """
    sections = []       # [(line, header, [(line, key, value_lines)])]
    indented = []       # [(line, header)]
    current = None
    option = None
    for number, line in enumerate(_klipper_lines(text), 1):
        if not line.strip():
            # configparser keeps a blank line inside a value as an empty
            # continuation line; it never ends the option.
            if option is not None:
                option[2].append('')
            continue
        if line[0] in ' \t':
            match = INDENTED_SECTION.match(line)
            if match:
                indented.append((number, match.group('header').strip()))
            if option is not None:
                option[2].append(line.strip())
            continue
        option = None
        match = SECTION.match(line)
        if match:
            current = (number, match.group('header').strip(), [])
            sections.append(current)
            continue
        match = OPTION.match(line)
        if match and current is not None:
            option = (number, match.group('option').strip().lower(),
                      [match.group('value').strip()])
            current[2].append(option)
    return sections, indented


def _join_value(value_lines):
    # configparser strips blank continuation lines from the tail only.
    lines = list(value_lines)
    while lines and not lines[-1]:
        lines.pop()
    return '\n'.join(lines)


def _jinja_env():
    return jinja2.Environment('{%', '%}', '{', '}')


def undeclared_names(env, source, context):
    """Names a template reads that are neither assigned in it nor supplied.

    Raises jinja2.TemplateSyntaxError for a template Klipper could not load
    either; the caller reports that as a failure rather than a crash.
    """
    ast = env.parse(source)
    assigned = set()
    read = set()
    for node in ast.find_all(nodes.Name):
        if node.ctx == 'load':
            read.add(node.name)
        else:
            # 'store' from {% set %} and {% for %} targets, 'param' from
            # {% macro %} arguments.
            assigned.add(node.name)
    for node in ast.find_all(nodes.NSRef):
        # `{% set ns.x = 1 %}` reads the namespace `ns` and must have set it.
        read.add(node.name)
    for node in ast.find_all(nodes.Macro):
        assigned.add(node.name)
    known = assigned | set(context) | set(env.globals) | JINJA_BUILTINS
    return sorted(read - known)


def check(config_dir):
    scanned = list(config_files(config_dir))
    if not scanned:
        return ["found no .cfg files under %s -- a moved or renamed config "
                "tree must fail this check, not pass it" % config_dir]
    if jinja2 is None:
        return ["jinja2 is not importable, so templates cannot be checked. "
                "Run this under the CI virtualenv (klippy-requirements.txt)"]

    env = _jinja_env()
    failures = []
    for path in scanned:
        relative = os.path.relpath(path, ROOT).replace(os.sep, '/')
        with open(path, encoding='utf-8', errors='replace') as handle:
            text = handle.read()
        sections, indented = parse(text)

        for number, header in indented:
            failures.append(
                "%s:%d: [%s] is indented -- configparser reads it as a "
                "continuation of the previous value, so no such section "
                "exists and its keys fall into the section before it"
                % (relative, number, header))

        seen_keys = {}
        for section_line, header, options in sections:
            for number, key, _value in options:
                first = seen_keys.setdefault((header, key), number)
                if first != number:
                    failures.append(
                        "%s:%d: [%s] sets %s again (first at line %d) -- "
                        "strict=False keeps the last one and says nothing"
                        % (relative, number, header, key, first))

        for section_line, header, options in sections:
            module = header.split()[0].lower() if header.split() else ''
            if module == 'gcode_macro':
                context = set(MACRO_CONTEXT)
                for _number, key, _value in options:
                    variable = MACRO_VARIABLE.match(key)
                    if variable:
                        context.add(variable.group('name'))
            else:
                context = set(TEMPLATE_CONTEXT)
            for number, key, value_lines in options:
                if not (key == 'gcode' or key.endswith('_gcode')):
                    continue
                source = _join_value(value_lines)
                try:
                    names = undeclared_names(env, source, context)
                except jinja2.TemplateSyntaxError as e:
                    failures.append(
                        "%s:%d: [%s] %s does not parse as a template: %s"
                        % (relative, number + (e.lineno or 1) - 1,
                           header, key, e.message))
                    continue
                for name in names:
                    failures.append(
                        "%s:%d: [%s] %s reads %r, which nothing assigns or "
                        "supplies -- it renders as Undefined and the "
                        "expression around it becomes a constant"
                        % (relative, _line_of(number, value_lines, name),
                           header, key, name))
    return failures


def _line_of(option_line, value_lines, name):
    """Best-effort line of the first read of `name` inside the option."""
    pattern = re.compile(r"\b%s\b" % re.escape(name))
    for offset, line in enumerate(value_lines):
        if pattern.search(line):
            return option_line + offset
    return option_line


def main():
    config_dir = os.path.join(ROOT, 'core')
    if not os.path.isdir(config_dir):
        sys.stderr.write("missing directory: %s\n" % config_dir)
        return 2

    failures = check(config_dir)
    if failures:
        sys.stderr.write(
            "Config that Klipper reads differently from how it looks.\n"
            "An indented [section] header is a continuation line; a\n"
            "repeated key keeps only its last value; an unassigned name in\n"
            "a template renders as Undefined. None of these is an error to\n"
            "Klipper, which is why each is one here.\n\n")
        for failure in failures:
            sys.stderr.write("  %s\n" % failure)
        sys.stderr.write("\n%d problem%s.\n"
                         % (len(failures), "" if len(failures) == 1 else "s"))
        return 1
    print("check_macro_config: headers, keys and template names are sound")
    return 0


if __name__ == '__main__':
    sys.exit(main())
