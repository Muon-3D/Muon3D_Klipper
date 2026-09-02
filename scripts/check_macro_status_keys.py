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
# Reading a missing *object* already fails loudly: `printer['mcu tooolhead']`
# raises KeyError out of GetStatusWrapper, and `printer.mcu_toolhead.x` raises
# UndefinedError on the attribute access. Only the key fails silently, which
# is why this checker keys on names.
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
    r"""(?:\s*\[\s*(?P<aq>['"])(?P<obj_q>[^'"]+)(?P=aq)\s*\]"""
    r"""|\.(?P<obj_a>[A-Za-z_]\w*))\s*%\}""")

SECTION = re.compile(r"^\s*\[(?P<name>[^\]]+)\]")
MACRO_VARIABLE = re.compile(r"^\s*variable_(?P<name>\w+)\s*:")
COMMENT = re.compile(r"^\s*#")


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
            # dict(a=..., b=...). Restricted to the dict builtin: harvesting
            # every keyword argument in the body would admit names like
            # `name` or `value` from unrelated calls and mask a real typo.
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
    """Blank out whole-line comments, keeping every byte offset intact."""
    out = []
    for line in text.splitlines(keepends=True):
        if COMMENT.match(line):
            stripped = line.rstrip('\r\n')
            out.append(' ' * len(stripped) + line[len(stripped):])
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
            line = bisect.bisect_right(newlines, region.start()) + 1
            here = bisect.bisect_right(section_lines, line)
            if here != section:
                aliases = {}
                section = here
            body = region.group(0)
            for match in ALIAS_DEF.finditer(body):
                aliases[match.group('name')] = (match.group('obj_q')
                                                or match.group('obj_a'))
            for match in PRINTER_REF.finditer(body):
                obj = match.group('obj_q') or match.group('obj_a')
                key = match.group('key_q') or match.group('key_a')
                references.append((relative, line, obj, key, None))
            for alias, obj in aliases.items():
                for key in _alias_keys(body, alias):
                    references.append((relative, line, obj, key, alias))
    return references


def _alias_keys(text, alias):
    pattern = re.compile(r"\b%s\s*(?:\[\s*(['\"])([^'\"]+)\1\s*\]"
                         r"|\.([A-Za-z_]\w*))" % re.escape(alias))
    for match in pattern.finditer(text):
        yield match.group(2) or match.group(3)


def check(klippy_dir, config_dir):
    published = published_status_keys(klippy_dir)
    if len(published) < 50:
        return ["found only %d published status keys under %s -- the scan is "
                "not looking where it thinks it is" % (len(published),
                                                       klippy_dir)]
    macro_variables = declared_macro_variables(config_dir)

    failures = []
    for relative, number, obj, key, alias in status_references(config_dir):
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
