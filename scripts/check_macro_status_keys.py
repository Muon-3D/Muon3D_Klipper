#!/usr/bin/env python3
# Check that every printer status key a shipped macro reads is one that
# something actually publishes.
#
# Copyright (C) 2026  Muon 3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Klipper renders macros with jinja2's default Undefined. Reading a *key* that
# no module publishes therefore yields Undefined instead of raising, and
# Undefined is falsy -- so `not printer['mcu toolhead'].disconnected` is a
# constant True and the gate built on it never refuses. That was KAN-287.
#
# Reading a missing *object* already fails loudly, though not where you would
# expect: jinja2 catches the KeyError out of GetStatusWrapper and substitutes
# Undefined, so `printer['mcu tooolhead']` on its own renders as ''. The error
# arrives on the *next* access -- `printer['mcu tooolhead'].anything` raises
# UndefinedError, as does the dotted form. Since a status read is always
# followed by a key, a wrong object name is loud in practice. Only the key
# itself fails silently, which is why this checker keys on names.
#
# The consequence is a deliberate weakening: it asks "does any module publish
# this name", not "does this object publish it". A read of the right key on
# the wrong object (`printer['mcu toolhead'].is_paused`) passes. Mapping a
# config section to the class implementing it is fragile, and the defect
# class being caught is a name that exists nowhere at all.
#
# It does NOT catch a bare undefined name -- `{% if not is_paused %}`, where
# `is_paused` was never assigned, which is what KAN-79 was. That read has no
# `printer` prefix, so nothing here sees it. Covering it needs
# jinja2.meta.find_undeclared_variables over each gcode block, which is a
# separate check.
import ast
import bisect
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Keys published under a computed name, which no static scan can see. Each
# entry names the module that builds it. This is also the escape hatch: a
# macro reading a key from an out-of-tree module belongs here, with its
# owner named, rather than being worked around by deleting the CI step.
DYNAMIC_KEYS = {
    # klippy/kinematics/idex_modes.py: status.update({'carriage_%d' % i: ...})
    'carriage_0': 'klippy/kinematics/idex_modes.py',
    'carriage_1': 'klippy/kinematics/idex_modes.py',
}

# Klipper's environment is Environment('{%', '%}', '{', '}'), so a status read
# can only occur inside `{% ... %}` or `{ ... }`. Scanning those regions
# rather than raw lines keeps prose and message strings out, and picks up
# expressions that wrap across lines.
JINJA_REGION = re.compile(r"\{%.*?%\}|\{[^{}]*\}", re.S)

# printer['obj'].key / printer.obj.key / printer['obj']['key']
# Quote groups are named and back-referenced by name: numbering them is how
# the bracket-key form silently stopped being checked once already.
PRINTER_REF = re.compile(
    r"""printer(?:\s*\[\s*(?P<oq>['"])(?P<obj_q>[^'"]+)(?P=oq)\s*\]"""
    r"""|\.(?P<obj_a>[A-Za-z_]\w*))"""
    r"""\s*(?:\[\s*(?P<kq>['"])(?P<key_q>[^'"]+)(?P=kq)\s*\]"""
    r"""|\.(?P<key_a>[A-Za-z_]\w*))""")

# {% set name = printer['obj'] %} -- an alias, so `name.key` is a status read.
ALIAS_DEF = re.compile(
    r"""set\s+(?P<name>[A-Za-z_]\w*)\s*=\s*printer"""
    # Both whitespace-control forms as well as the plain one. An alias that
    # stops being recognised takes every read through it with it, and the
    # difference is one keystroke that changes nothing about what renders.
    r"""(?:\s*\[\s*(?P<aq>['"])(?P<obj_q>[^'"]+)(?P=aq)\s*\]"""
    r"""|\.(?P<obj_a>[A-Za-z_]\w*))\s*[-+]?%\}""")

# Attributes that belong to the mapping or string itself, not to the status
# report. jinja2 resolves these on the object, so they are not status keys and
# no module publishes them.
#
# `.get` matters most: `printer['mcu toolhead'].get('key', True)` is the
# defensive idiom for exactly the missing-key problem this checker exists to
# find, and rejecting it would fail the build on the safest way to write the
# read.
NOT_STATUS_KEYS = frozenset((
    'get', 'keys', 'values', 'items', 'copy', 'update', 'pop', 'setdefault',
    'lower', 'upper', 'strip', 'split', 'join', 'replace', 'startswith',
    'endswith', 'format', 'count', 'index',
))

SECTION = re.compile(r"^\s*\[(?P<name>[^\]]+)\]")
MACRO_VARIABLE = re.compile(r"^\s*variable_(?P<name>\w+)\s*:")
# A `#` at the start of a line, or after whitespace -- the two forms
# configparser treats as starting a comment.
COMMENT = re.compile(r"(?:^|(?<=\s))[#;]")


def published_status_keys(klippy_dir):
    """Every status key any module under klippy/ publishes."""
    keys = set(DYNAMIC_KEYS)
    for dirpath, _dirnames, filenames in os.walk(klippy_dir):
        for filename in sorted(filenames):
            if not filename.endswith('.py'):
                continue
            path = os.path.join(dirpath, filename)
            with open(path, encoding='utf-8', errors='replace') as handle:
                source = handle.read()
            try:
                tree = ast.parse(source)
            except SyntaxError:
                # klippy still carries python2-only modules; skip them rather
                # than fail. They are not where status is published.
                continue
            for node in ast.walk(tree):
                is_get_status = (isinstance(node, ast.FunctionDef)
                                 and node.name == 'get_status')
                if is_get_status:
                    keys |= _keys_in_get_status(node)
                elif isinstance(node, ast.Assign):
                    keys |= _keys_in_status_assign(node)
    return keys


def _keys_in_get_status(func):
    """Keys built anywhere inside a get_status body."""
    keys = set()
    for node in ast.walk(func):
        if isinstance(node, ast.Dict):
            # return {'a': ..., 'b': ...}, and .update({...})
            keys |= _dict_literal_keys(node)
        elif isinstance(node, ast.Call):
            # dict(a=..., b=...). Restricted to the dict builtin so that a
            # keyword argument to some unrelated call inside get_status cannot
            # enter the published set. No get_status in the tree currently
            # passes a keyword to anything, so this narrows what a future one
            # may do rather than changing what is harvested today.
            if isinstance(node.func, ast.Name) and node.func.id == 'dict':
                for keyword in node.keywords:
                    if keyword.arg:
                        keys.add(keyword.arg)
        elif isinstance(node, ast.Assign):
            # sts['can_extrude'] = ...  (any local name, not just "status")
            for target in node.targets:
                name = _constant_subscript(target)
                if name is not None:
                    keys.add(name)
    return keys


def _keys_in_status_assign(node):
    """Keys written into a status dict from outside get_status.

    Some modules build their status dict elsewhere and have get_status simply
    return it -- manual_probe.py assigns the whole dict in reset_status(), and
    mcu.py fills self._get_status_info[...] from its connect handlers. Both
    forms are invisible to a scan of get_status bodies alone.
    """
    keys = set()
    for target in node.targets:
        name = _constant_subscript(target)
        if name is not None:
            if 'status' in _unparse(target.value).lower():
                keys.add(name)
        elif isinstance(target, (ast.Attribute, ast.Name)):
            if ('status' in _unparse(target).lower()
                    and isinstance(node.value, ast.Dict)):
                keys |= _dict_literal_keys(node.value)
    return keys


def _dict_literal_keys(node):
    return {key.value for key in node.keys
            if isinstance(key, ast.Constant) and isinstance(key.value, str)}


def _constant_subscript(target):
    """The string key of `something['key'] = ...`, else None."""
    if not isinstance(target, ast.Subscript):
        return None
    index = target.slice
    # python 3.8 wraps the subscript in ast.Index
    if index.__class__.__name__ == 'Index':
        index = index.value
    if isinstance(index, ast.Constant) and isinstance(index.value, str):
        return index.value
    return None


def _unparse(node):
    if hasattr(ast, 'unparse'):
        return ast.unparse(node)
    return getattr(node, 'id', '') or getattr(node, 'attr', '')


def _config_files(config_dir):
    for dirpath, _dirnames, filenames in os.walk(config_dir):
        for filename in sorted(filenames):
            if filename.endswith('.cfg'):
                yield os.path.join(dirpath, filename)


def declared_macro_variables(config_dir):
    """variable_<name> declarations, keyed by gcode_macro name."""
    variables = {}
    for path in _config_files(config_dir):
        section = None
        with open(path, encoding='utf-8', errors='replace') as handle:
            for line in handle:
                header = SECTION.match(line)
                if header:
                    section = header.group('name').strip()
                    continue
                if section is None or not section.startswith('gcode_macro '):
                    continue
                variable = MACRO_VARIABLE.match(line)
                if variable:
                    name = section[len('gcode_macro '):].strip()
                    known = variables.setdefault(name, set())
                    known.add(variable.group('name'))
    return variables


def _mask_comments(text):
    """Blank out comments, keeping every character offset intact.

    Klipper parses these files with `inline_comment_prefixes=(';', '#')`, so a
    `#` after whitespace ends the value wherever it appears -- not only at the
    start of a line. Text after one is not code, and flagging a status key
    named in it fails the build over a line the printer never reads.
    """
    out = []
    for line in text.splitlines(keepends=True):
        body = line.rstrip('\r\n')
        ending = line[len(body):]
        comment = COMMENT.search(body)
        if comment:
            out.append(body[:comment.start()]
                       + ' ' * (len(body) - comment.start()) + ending)
        else:
            out.append(line)
    return ''.join(out)


def status_references(config_dir):
    """Every (path, line, object, key, alias) a config reads."""
    references = []
    for path in _config_files(config_dir):
        relative = os.path.relpath(path, ROOT).replace(os.sep, '/')
        with open(path, encoding='utf-8', errors='replace') as handle:
            text = handle.read()
        masked = _mask_comments(text)
        newlines = [i for i, char in enumerate(masked) if char == '\n']
        # Line numbers where a new config section starts; an alias is local to
        # the macro that set it, so crossing one clears them.
        section_lines = [number for number, line
                         in enumerate(masked.splitlines(), 1)
                         if SECTION.match(line)]

        aliases = {}
        section = None
        for region in JINJA_REGION.finditer(masked):
            start = region.start()
            here = bisect.bisect_right(
                section_lines, bisect.bisect_right(newlines, start) + 1)
            if here != section:
                aliases = {}
                section = here
            body = region.group(0)

            def line_of(offset, _start=start):
                # Per match, not per region: a region can span lines, and
                # reporting the opening line sends the reader to the wrong
                # one on exactly the multi-line reads this scanner exists to
                # catch.
                return bisect.bisect_right(newlines, _start + offset) + 1

            for match in ALIAS_DEF.finditer(body):
                aliases[match.group('name')] = (match.group('obj_q')
                                                or match.group('obj_a'))
            for match in PRINTER_REF.finditer(body):
                obj = match.group('obj_q') or match.group('obj_a')
                key = match.group('key_q') or match.group('key_a')
                references.append(
                    (relative, line_of(match.start()), obj, key, None))
            for alias, obj in aliases.items():
                for offset, key in _alias_keys(body, alias):
                    references.append(
                        (relative, line_of(offset), obj, key, alias))
    return references


def _alias_keys(text, alias):
    pattern = re.compile(r"\b%s\s*(?:\[\s*(['\"])([^'\"]+)\1\s*\]"
                         r"|\.([A-Za-z_]\w*))" % re.escape(alias))
    for match in pattern.finditer(text):
        yield match.start(), match.group(2) or match.group(3)


def check(klippy_dir, config_dir):
    published = published_status_keys(klippy_dir)
    if len(published) < 50:
        return ["found only %d published status keys under %s -- the scan is "
                "not looking where it thinks it is" % (len(published),
                                                       klippy_dir)]
    scanned = list(_config_files(config_dir))
    if not scanned:
        return ["found no .cfg files under %s -- a moved or renamed macro "
                "tree must fail this check, not pass it" % config_dir]
    macro_variables = declared_macro_variables(config_dir)

    failures = []
    for relative, number, obj, key, alias in status_references(config_dir):
        if key in NOT_STATUS_KEYS:
            continue
        where = "%s:%d" % (relative, number)
        read = ("%s.%s (alias of printer[%r])" % (alias, key, obj) if alias
                else "printer[%r].%s" % (obj, key))
        if obj.startswith('gcode_macro '):
            name = obj[len('gcode_macro '):].strip()
            if name not in macro_variables:
                # Defined outside this tree; nothing to check it against.
                continue
            if key not in macro_variables[name]:
                failures.append(
                    "%s: %s -- [%s] declares no variable_%s"
                    % (where, read, obj, key))
        elif key not in published:
            failures.append(
                "%s: %s -- no module under klippy/ publishes %r"
                % (where, read, key))
    return failures


def main():
    klippy_dir = os.path.join(ROOT, 'klippy')
    config_dir = os.path.join(ROOT, 'core')
    for path in (klippy_dir, config_dir):
        if not os.path.isdir(path):
            sys.stderr.write("missing directory: %s\n" % path)
            return 2

    failures = check(klippy_dir, config_dir)
    if failures:
        sys.stderr.write(
            "Macro reads a status key nothing publishes.\n"
            "Jinja renders these as Undefined, so the surrounding test\n"
            "becomes a constant and any gate built on it stops refusing.\n"
            "If the key is real and built dynamically, add it to\n"
            "DYNAMIC_KEYS with the module that publishes it.\n\n")
        for failure in failures:
            sys.stderr.write("  %s\n" % failure)
        sys.stderr.write("\n%d bad reference%s.\n"
                         % (len(failures), "" if len(failures) == 1 else "s"))
        return 1
    print("check_macro_status_keys: all macro status reads are published")
    return 0


if __name__ == '__main__':
    sys.exit(main())
