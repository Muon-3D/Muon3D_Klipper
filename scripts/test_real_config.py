#!/usr/bin/env python3
"""Prove klippy STARTS on each shipped board config.

Every other batch test loads a small test config. None loads the real
core/M1/core.cfg (B1 board) or core/M1-B0/core.cfg (B0 board), so a
misspelt option, a section no module owns, or a pin the firmware lacks
could ship and only show up as a printer that does not start. This runs
klippy's own batch mode (-i/-o/-d, no serial port) on the real files:
config load, the end-of-load option and section check, connect against
the MCU dictionaries built from this tree, "ready", then a harmless gcode
file. It fails on "Option 'x' is not valid in section 'y'", "Section 'z'
is not a valid config section", "Unknown pin" and every other config
error klippy raises.

The shipped files name things outside the repo. They are not edited and
not copied. A generated top-level config includes the real core.cfg and
then overrides only the path valued options (a later section with the
same option wins, see klippy/configfile.py), pointing them at a
temporary directory. The board's own calibration.cfg is passed with -c,
as the printer does, so its conflict check against core runs too. The
driver prints every file it stubbed and every option it redirected.

  python3 scripts/test_real_config.py -d ci_build/dict core/M1 core/M1-B0
  python3 scripts/test_real_config.py -d ci_build/dict --selftest core/M1
  python3 scripts/test_real_config.py --dry-run core/M1   # no klippy run

Variants (see VARIANTS): "shipped" is the config as is, where the
toolhead MCU is non-critical and klippy defers it (a printer started with
the toolhead unplugged); "toolhead-required" also overrides
[mcu toolhead] is_non_critical to False so the toolhead's pins and
modules are built against its dictionary too.
"""
import sys, os, shutil, subprocess, tempfile, optparse

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'klippy'))
import configfile

# Dictionary file (in --dictdir, built by the mcu_compile step from
# test/configs/<name>.config) per MCU section name. rp2040 and
# linuxprocess are the M1's chips; the rp2040 test config leaves every
# WANT_ option at its default, which includes the M1's own set (the
# firmware config in core/M1/firmware/config.rp2040 turns features OFF,
# none ON that the default lacks) plus WANT_MUON_ADS1220.
DICT_FOR = {'mcu': 'rp2040.dict', 'toolhead': 'rp2040.dict',
            'rpi': 'linuxprocess.dict'}

# Options that hold an absolute path outside the repo and what to do about
# each. 'dir' is created, 'empty' is created empty, 'parent' only has its
# directory created (the module treats an absent file as "never written").
STUBS = {
    ('virtual_sdcard', 'path'): 'dir',
    ('save_variables', 'filename'): 'empty',
    ('hotend_fan_latch', 'state_file'): 'parent',
    ('muon_guard', 'dev_config_path'): 'empty',
}
# Absolute paths klippy never opens in batch mode (no serial port). An
# openocd_config under /opt/klipper must still exist in this repo.
UNUSED_IN_BATCH = ('serial',)
REPO_MOUNT = '/opt/klipper/'

# One harmless read-only command, no motion, no heat. Batch mode only
# reads the gcode file once the printer is ready, so its reply in the log
# is proof of "ready". It must be on [muon_guard] client_allow: the file
# is a client door, and M115 or STATUS would be refused (error exit).
GCODE = ['GET_POSITION', '']
READY_MARK = 'gcode homing:'

# Config added by --selftest: (name, config text, text klippy must log).
NEGATIVES = [
    ('unknown option',
     '[load_cell_probe]\nnot_a_real_option: 1\n',
     "Option 'not_a_real_option' is not valid in section "
     "'load_cell_probe'"),
    ('unknown section',
     '[not_a_real_section]\nnot_a_real_option: 1\n',
     "Section 'not_a_real_section' is not a valid config section"),
]

VARIANTS = [
    ('shipped', ''),
    ('toolhead-required', '[mcu toolhead]\nis_non_critical: False\n'),
]

BAD_LOG = ('Config error', 'is not valid in section', 'Unable to parse',
           'Unknown pin', 'Unhandled exception',
           'Transition to shutdown state', 'Internal error',
           'must be specified', 'is not a valid pin name',
           'not a valid config section', 'Unable to load module',
           'Unknown config object')
# Printed, not failed on: logged by code that carries on.
WARN_LOG = ('Traceback', 'Script running error')


class Fail(Exception):
    pass


class Reader(configfile.ConfigFileReader):
    """The real reader, recording every [include] it follows."""
    def __init__(self):
        self.includes = []
    def _resolve_include(self, source_filename, include_spec, fileconfig,
                         visited):
        res = super()._resolve_include(source_filename, include_spec,
                                       fileconfig, visited)
        self.includes.append((source_filename, include_spec.strip(), res))
        return res


def load_board(board):
    core = os.path.join(REPO, board, 'core.cfg')
    rdr = Reader()
    data = rdr.read_config_file(core)
    fc = rdr.build_fileconfig_with_includes(data, core)
    return core, rdr, fc


def check_includes(rdr, notes):
    for src, spec, found in rdr.includes:
        for fname in found:
            if not os.path.abspath(fname).startswith(REPO + os.sep):
                raise Fail("include %r (from %s) resolves outside the repo:"
                           " %s. Includes cannot be overridden by a later"
                           " section; extend scripts/test_real_config.py to"
                           " run on a copy with that line redirected."
                           % (spec, src, fname))
        if not found:
            notes.append("include %r from %s matched no file (a glob, which"
                         " klippy allows)" % (spec, os.path.basename(src)))
    notes.append("%d [include] lines followed, all inside the repo"
                 % (len(rdr.includes),))


def discover(fc, tmp, notes):
    """Return {section: {option: stub path}} for every outside path."""
    overrides = {}
    for section in fc.sections():
        for option in fc.options(section):
            value = fc.get(section, option).strip()
            if not value.startswith('/'):
                continue
            prefix = section.split()[0]
            if prefix == 'mcu':
                if option in UNUSED_IN_BATCH:
                    notes.append("[%s] %s: %s -- not opened in batch mode"
                                 % (section, option, value))
                elif value.startswith(REPO_MOUNT):
                    rel = value[len(REPO_MOUNT):]
                    if not os.path.isfile(os.path.join(REPO, rel)):
                        raise Fail("[%s] %s: %s names a file that is not in"
                                   " this repo (%s)" % (section, option,
                                                        value, rel))
                    notes.append("[%s] %s: %s -- exists in the repo as %s"
                                 % (section, option, value, rel))
                else:
                    raise Fail("unclassified absolute path [%s] %s: %s"
                               % (section, option, value))
                continue
            kind = STUBS.get((section, option))
            if kind is None:
                raise Fail("unclassified absolute path [%s] %s: %s -- add it"
                           " to STUBS in scripts/test_real_config.py"
                           % (section, option, value))
            stub = os.path.join(tmp, 'stub', section.replace(' ', '_'),
                                os.path.basename(value))
            if kind == 'dir':
                os.makedirs(stub)
            else:
                os.makedirs(os.path.dirname(stub), exist_ok=True)
                if kind == 'empty':
                    open(stub, 'w').close()
            overrides.setdefault(section, {})[option] = stub
            notes.append("[%s] %s: %s -> %s (%s)"
                         % (section, option, value, stub, kind))
    return overrides


def mcu_names(fc):
    names = []
    for section in fc.sections():
        parts = section.split()
        if parts[0] == 'mcu' and len(parts) <= 2:
            names.append(parts[1] if len(parts) == 2 else 'mcu')
    return names


def make_calibration(board, tmp, notes):
    src = os.path.join(REPO, board, 'calibration.cfg')
    dst = os.path.join(tmp, 'calibration.cfg')
    shutil.copyfile(src, dst)
    with open(dst) as f:
        if 'reference_max_load_counts' not in f.read():
            raise Fail("%s has no reference_max_load_counts, which the"
                       " preloaded_max load cell model needs" % (src,))
    notes.append("calibration.cfg: copy of %s/calibration.cfg, passed as -c"
                 % (board,))
    return dst


def prepare(board, variant_cfg, extra_cfg, dictdir, tmp):
    notes = []
    core, rdr, fc = load_board(board)
    check_includes(rdr, notes)
    overrides = discover(fc, tmp, notes)
    names = mcu_names(fc)
    dicts = []
    for name in names:
        if name not in DICT_FOR:
            raise Fail("[mcu %s] has no entry in DICT_FOR" % (name,))
        dfile = os.path.join(dictdir, DICT_FOR[name])
        arg = dfile if name == 'mcu' else '%s=%s' % (name, dfile)
        dicts += ['-d', arg]
        notes.append("MCU %s: dictionary %s" % (name, dfile))
    cal = make_calibration(board, tmp, notes)
    top = os.path.join(tmp, 'top.cfg')
    with open(top, 'w') as f:
        f.write('[include %s]\n' % (core,))
        for section in sorted(overrides):
            f.write('[%s]\n' % (section,))
            for option, value in sorted(overrides[section].items()):
                f.write('%s: %s\n' % (option, value))
        if variant_cfg:
            f.write('# variant override (not in the shipped config)\n')
            f.write(variant_cfg)
        if extra_cfg:
            f.write('# selftest addition\n')
            f.write(extra_cfg)
    gcode = os.path.join(tmp, 'input.gcode')
    with open(gcode, 'w') as f:
        f.write('\n'.join(GCODE))
    return top, cal, gcode, dicts, notes


def read_text(path):
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def run_klippy(board, variant, variant_cfg, extra_cfg, dictdir, timeout,
               dry_run):
    tmp = tempfile.mkdtemp(prefix='real_config_')
    keep = False
    try:
        top, cal, gcode, dicts, notes = prepare(
            board, variant_cfg, extra_cfg, dictdir, tmp)
        log = os.path.join(tmp, 'klippy.log')
        args = [sys.executable, os.path.join(REPO, 'klippy', 'klippy.py'),
                top, '-i', gcode, '-o', os.path.join(tmp, 'out'),
                '-l', log, '-v', '-c', cal] + dicts
        sys.stderr.write("---- %s [%s]\n" % (board, variant))
        for n in notes:
            sys.stderr.write("  stub: %s\n" % (n,))
        if dry_run:
            sys.stderr.write("  generated %s:\n%s\n" % (
                top, read_text(top)))
            sys.stderr.write("  would run: %s\n" % (' '.join(args),))
            return '', 0, ''
        try:
            rc = subprocess.run(args, cwd=REPO, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            rc = 'timeout after %ds' % (timeout,)
        text = read_text(log)
        return text, rc, show_failure(top, text)
    finally:
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)


def show_failure(top, text):
    # Called before the temp directory is removed.
    lines = text.splitlines()
    return ("---- generated top-level config (%s):\n%s\n"
            "---- last 60 lines of the klippy log:\n%s\n"
            % (top, read_text(top), '\n'.join(lines[-60:])))


def check_ready(text, rc, board, variant):
    problems = []
    if rc != 0:
        problems.append("klippy exit status %s" % (rc,))
    for bad in BAD_LOG:
        if bad in text:
            problems.append("log contains %r" % (bad,))
    if READY_MARK not in text:
        problems.append("no %r in the log: gcode input is only read once"
                        " the printer is ready, so it never became ready"
                        % (READY_MARK,))
    required = ["Configured MCU 'mcu'", "Configured MCU 'rpi'"]
    if variant == 'toolhead-required':
        required.append("Configured MCU 'toolhead'")
    for req in required:
        if req not in text:
            problems.append("log lacks %r" % (req,))
    return problems


def main():
    opts = optparse.OptionParser("%prog [options] <board dir>...")
    opts.add_option("-d", "--dictdir", dest="dictdir", default="ci_build/dict")
    opts.add_option("--selftest", action="store_true", dest="selftest",
                    help="negative controls: an unknown option and an "
                    "unknown section must fail klippy")
    opts.add_option("--dry-run", action="store_true", dest="dry_run",
                    help="print the stubs and generated config only")
    opts.add_option("--variants", dest="variants", default="all",
                    help="comma list of %s, or all"
                    % (', '.join(v[0] for v in VARIANTS),))
    opts.add_option("--timeout", dest="timeout", type="int", default=300)
    options, boards = opts.parse_args()
    if not boards:
        opts.error("give at least one board dir, e.g. core/M1")
    dictdir = os.path.abspath(options.dictdir)
    if not options.dry_run:
        for dname in sorted(set(DICT_FOR.values())):
            if not os.path.isfile(os.path.join(dictdir, dname)):
                sys.exit("missing dictionary %s (built by mcu_compile)"
                         % (os.path.join(dictdir, dname),))
    variants = VARIANTS
    if options.variants != 'all':
        want = options.variants.split(',')
        variants = [v for v in VARIANTS if v[0] in want]
    failures = []
    for board in boards:
        board = board.rstrip('/')
        try:
            if options.selftest:
                for name, cfg, expect in NEGATIVES:
                    text, rc, shown = run_klippy(
                        board, 'shipped+' + name.replace(' ', '-'), '', cfg,
                        dictdir, options.timeout, options.dry_run)
                    if options.dry_run:
                        continue
                    if rc == 0 or expect not in text:
                        sys.stderr.write(shown)
                        failures.append("%s: %s was NOT caught (exit %s,"
                                        " wanted %r in the log)"
                                        % (board, name, rc, expect))
                    else:
                        sys.stderr.write("  ok: %s %s fails as it should"
                                         " (%r)\n" % (board, name, expect))
                continue
            for variant, vcfg in variants:
                text, rc, shown = run_klippy(
                    board, variant, vcfg, '', dictdir, options.timeout,
                    options.dry_run)
                if options.dry_run:
                    continue
                problems = check_ready(text, rc, board, variant)
                if problems:
                    sys.stderr.write(shown)
                    failures.append("%s [%s]: %s"
                                    % (board, variant, '; '.join(problems)))
                else:
                    sys.stderr.write("  ok: %s [%s] started and reached"
                                     " ready\n" % (board, variant))
                    warns = [ln for ln in text.splitlines()
                             if any(w in ln for w in WARN_LOG)]
                    if warns:
                        sys.stderr.write("  note: %d warning lines, first"
                                         " 10:\n    %s\n" % (
                                             len(warns),
                                             '\n    '.join(warns[:10])))
        except Fail as e:
            failures.append("%s: %s" % (board, e))
    if failures:
        sys.stderr.write("\nFAILED:\n  " + "\n  ".join(failures) + "\n")
        sys.exit(1)
    sys.stderr.write("\nreal config %s: all passed\n"
                     % ('selftest' if options.selftest else 'startup',))


if __name__ == '__main__':
    main()
