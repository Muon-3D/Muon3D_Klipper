"""muon_guard: who may start a command, and when the guard stands down.

Drives the real GCodeDispatch (klippy/gcode.py) with fake command handlers,
the real muon_guard, and the real [muon_guard] section of core/M1/core.cfg.

    PYTHONUTF8=1 python3 test/muon_guard/test_muon_guard.py

What is under test (KAN-1730, epic KAN-1729):
  * the dev-mode detector fails closed;
  * a command that enters at a client door is checked against an allowlist,
    and a macro that client started is not checked again inside;
  * everything is allowed in dev mode.

Expected values are written out here, not imported: the allowlist and the
refusal text are what the tests are about.
"""
import configparser
import fnmatch
import os
import re
import shlex
import stat
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

import gcode as gcode_mod  # noqa: E402
import configfile  # noqa: E402
from extras import muon_guard  # noqa: E402

CORE_CFG = os.path.join(ROOT, 'core', 'M1', 'core.cfg')
MACRO_DIR = os.path.join(ROOT, 'core', 'M1', 'macros')
OEM_CFG = '/opt/klipper/core/M1/core.cfg'
DEV_CFG = '/home/printer_data/devmode_custom_config/core.cfg'

# What a refusal has to say, so the person who hits it knows what to do.
REFUSAL_WORDS = ('dev mode', 'allowlist')

# The commands a client must never be able to start outside dev mode. Each
# one is an escape from the machine's safety envelope, directly or by
# changing the state another macro trusts.
MUST_REFUSE = [
    'SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z',
    'FORCE_MOVE STEPPER=stepper_z DISTANCE=10 VELOCITY=5',
    'SET_CONTACT_GUARD ENABLE=0',
    'SET_GCODE_VARIABLE MACRO=G28 VARIABLE=z_lifted_once VALUE=1',
    'SAVE_CONFIG',
    'SAVE_VARIABLE VARIABLE=resonance_valid VALUE=1',
    '_HOME_Z',
    '_HOME_Z X_OFFSET=40',
    '_HOME_Z_FROM_LAST_PROBE',
    '_HOME_X',
    'AXIS_PROTECTED_MOVE AXIS=Z DIST=10',
    'AXIS_HOME_MOVE AXIS=Z',
    'SET_TMC_CURRENT STEPPER=stepper_z CURRENT=2',
    'SET_STEPPER_ENABLE STEPPER=stepper_z ENABLE=0',
    'STEPPER_BUZZ STEPPER=stepper_z',
    'G28.1 Z',
    'M104.1 S200',
    'SET_HEATER_TEMPERATURE_BASE HEATER=extruder TARGET=200',
    'SET_BED_REMOVAL_DETECTOR ENABLE=0',
    'SET_IDLE_TIMEOUT TIMEOUT=100000',
    'PID_CALIBRATE HEATER=extruder TARGET=200',
    'BED_ATTACHED',
    'TOOLHEAD_CONNECT',
    'BUZZ_EXTRUDER',
    # HOMING_GATE runs {params.CALLBACK} as G-code: it is a way to start
    # anything unless its CALLBACK is held to G28.
    'HOMING_GATE CALLBACK=SET_KINEMATIC_POSITION',
    'HOMING_GATE CALLBACK="G1 X1"',
    'HOMING_GATE CALLBACK=_HOME_Z',
    'HOMING_GATE CALLBACK=G280',
    'HOMING_GATE',
    # not on the list at all
    'NOT_A_COMMAND',
]

# Every command name the fake printer registers, so that "refused" is told
# apart from "unknown command".
EXTRA_REGISTERED = set(c.split()[0] for c in MUST_REFUSE)


CommandError = gcode_mod.CommandError


class ConfigError(Exception):
    pass


class FakeMutex:
    def __enter__(self):
        return self
    def __exit__(self, *a):
        return False
    def test(self):
        return False


class FakeReactor:
    def mutex(self):
        return FakeMutex()
    def monotonic(self):
        return 100.


class FakePrinter:
    config_error = ConfigError
    command_error = CommandError

    def __init__(self, config_file):
        self.start_args = {'config_file': config_file}
        self.objects = {}
        self.shutdowns = []
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return FakeReactor()
    def register_event_handler(self, event, cb):
        pass
    def send_event(self, event, *params):
        pass
    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)
    def request_exit(self, how):
        pass
    def lookup_object(self, name, *default):
        if name in self.objects:
            return self.objects[name]
        if default:
            return default[0]
        raise ConfigError("no object %s" % (name,))
    def add_object(self, name, obj):
        self.objects[name] = obj


class FakeConfig:
    """Same reading rules as klippy's ConfigWrapper.getlist."""
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'muon_guard'
    def get(self, name, default=configfile.sentinel):
        if name in self.values:
            return self.values[name]
        if default is configfile.sentinel:
            raise ConfigError("Option '%s' in section 'muon_guard' must be"
                              " specified" % (name,))
        return default
    def getlist(self, name, default=configfile.sentinel, sep=',',
                count=None):
        value = self.get(name, default)
        if isinstance(value, (list, tuple)):
            return list(value)
        if len(value.strip()) == 0:
            return []
        return [p.strip() for p in value.split(sep)]


def read_core_section(name='muon_guard'):
    """The options of [name] in core.cfg, parsed the way klippy parses."""
    with open(CORE_CFG, encoding='utf-8') as f:
        text = f.read().replace('\r\n', '\n')
    m = re.search(r'^\[%s\]\n(.*?)(?=^\[|\Z)' % re.escape(name), text,
                  re.S | re.M)
    assert m is not None, "core.cfg has no [%s] section" % (name,)
    cp = configparser.RawConfigParser(
        strict=False, inline_comment_prefixes=(';', '#'))
    cp.read_string('[%s]\n%s' % (name, m.group(1)))
    return dict(cp.items(name))


def core_allow():
    return read_core_section()['client_allow']


class Rig:
    """The real dispatch + guard, with fake handlers that record calls."""
    def __init__(self, config_file=OEM_CFG, allow=None, dev=False,
                 values=None, register_gate=True):
        self.tmp = tempfile.mkdtemp(prefix='muon_guard_test_')
        self.dev_cfg = os.path.join(self.tmp, 'devmode', 'core.cfg')
        os.makedirs(os.path.dirname(self.dev_cfg))
        with open(self.dev_cfg, 'w') as f:
            f.write('[printer]\n')
        if dev:
            config_file = self.dev_cfg
        self.printer = FakePrinter(config_file)
        self.gcode = gcode_mod.GCodeDispatch(self.printer)
        self.printer.add_object('gcode', self.gcode)
        self.out = []
        self.gcode.register_output_handler(self.out.append)
        self.calls = []
        opts = {'dev_config_path': self.dev_cfg,
                'client_allow': core_allow() if allow is None else allow}
        opts.update(values or {})
        self.guard = None
        if register_gate:
            self.guard = muon_guard.load_config(FakeConfig(self.printer, opts))
            self.printer.add_object('muon_guard', self.guard)
        self._register_handlers(opts['client_allow'])
        self.gcode.is_printer_ready = True
        self.gcode.gcode_handlers = self.gcode.ready_gcode_handlers

    def _names(self, allow):
        if isinstance(allow, str):
            allow = [p.strip() for p in allow.split('\n')]
        names = set(e.split()[0] for e in allow if e.strip())
        return names | EXTRA_REGISTERED

    def _register_handlers(self, allow):
        # G28 behaves like homing.cfg: it makes the fake homed itself. This
        # runs as a macro would, through run_script_from_command.
        self.gcode.register_command('G28', self._g28)
        # PRINT_START behaves like a start macro that homes and then moves.
        self.gcode.register_command('PRINT_START', self._print_start)
        # the dispatcher's own commands would run for real; record instead
        for name in ('M110', 'M112', 'M115', 'STATUS', 'ECHO', 'HELP',
                     'RESTART', 'FIRMWARE_RESTART'):
            self.gcode.register_command(name, None)
        for name in sorted(self._names(allow)):
            self._register(name)

    def _register(self, name):
        if name in self.gcode.ready_gcode_handlers:
            return
        def handler(gcmd, name=name):
            self.calls.append(gcmd.get_commandline())
        self.gcode.register_command(name, handler)

    def _g28(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        self.gcode.run_script_from_command(
            "SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z\n"
            "FORCE_MOVE STEPPER=stepper_z DISTANCE=10 VELOCITY=5\n"
            "_HOME_Z\nG28.1 Z\nAXIS_PROTECTED_MOVE AXIS=Z DIST=10")

    def _print_start(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        self.gcode.run_script_from_command("G28\n_HOME_Z_FROM_LAST_PROBE")

    # The four doors a client uses.
    def client(self, script, origin='webhooks'):
        del self.calls[:]
        del self.out[:]
        try:
            self.gcode.run_client_script(script, origin)
        except CommandError as e:
            return str(e)
        return None

    def internal(self, script):
        del self.calls[:]
        self.gcode.run_script(script)

    def from_macro(self, script):
        del self.calls[:]
        self.gcode.run_script_from_command(script)


FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: " + msg)


def refused_text_ok(text):
    low = (text or '').lower()
    return all(w in low for w in REFUSAL_WORDS)


######################################################################
# 1. The detector
######################################################################

def test_detector():
    tmp = tempfile.mkdtemp(prefix='muon_guard_det_')
    dev_dir = os.path.join(tmp, 'devmode')
    os.makedirs(dev_dir)
    dev = os.path.join(dev_dir, 'core.cfg')
    with open(dev, 'w') as f:
        f.write('[printer]\n')
    oem_dir = os.path.join(tmp, 'oem')
    os.makedirs(oem_dir)
    oem = os.path.join(oem_dir, 'core.cfg')
    with open(oem, 'w') as f:
        f.write('[printer]\n')
    detect = muon_guard.detect_dev_mode

    on, why = detect(dev, dev)
    check(on is True, "detector: the dev clone as the running config is dev "
          "mode (%s)" % (why,))
    check(isinstance(why, str) and why, "detector: dev mode has a reason")

    on, why = detect(oem, dev)
    check(on is False, "detector: the OEM config is not dev mode")
    check('oem' in why.lower() or 'not' in why.lower(),
          "detector: OEM reason says why (%s)" % (why,))

    # fail-closed cases: every one must be NOT dev mode
    cases = [
        ("config_file is None", None, dev),
        ("config_file is empty", '', dev),
        ("config_file is not a string", 12, dev),
        ("config_file is relative", 'core.cfg', 'core.cfg'),
        ("config path does not exist", os.path.join(dev_dir, 'gone.cfg'),
         os.path.join(dev_dir, 'gone.cfg')),
        ("config path is a directory", dev_dir, dev_dir),
        ("dev path option is empty", dev, ''),
        ("dev path option is None", dev, None),
        ("an unknown path, neither OEM nor dev (a second model)", oem, dev),
        ("a client-writable path, even if it is the configured one's twin",
         os.path.join(tmp, 'core.cfg'), dev),
        ("dev path with a dot-dot detour", dev_dir + '/../oem/core.cfg',
         dev),
    ]
    for label, cfg, devpath in cases:
        on, why = detect(cfg, devpath)
        check(on is False, "detector fails closed: %s" % (label,))
        check(isinstance(why, str) and why,
              "detector: a refusal reason for: %s" % (label,))

    # the same path spelled differently is still the same file
    on, _ = detect(dev_dir + '/./core.cfg', dev)
    check(on is True, "detector: a redundant ./ in the running path is fine")

    # unreadable: the file is there but cannot be read
    if os.name != 'nt' and hasattr(os, 'geteuid') and os.geteuid() != 0:
        os.chmod(dev, 0)
        try:
            on, _ = detect(dev, dev)
            check(on is False, "detector fails closed: unreadable config")
        finally:
            os.chmod(dev, stat.S_IRUSR | stat.S_IWUSR)

    # symlinks: a link anywhere in the chain is not trusted
    link = os.path.join(tmp, 'link.cfg')
    try:
        os.symlink(oem, link)
        have_links = True
    except (OSError, NotImplementedError, AttributeError):
        have_links = False
    if have_links:
        on, _ = detect(link, link)
        check(on is False, "detector fails closed: config path is a symlink")
        dlink = os.path.join(tmp, 'devlink')
        os.symlink(dev_dir, dlink)
        on, _ = detect(os.path.join(dlink, 'core.cfg'),
                       os.path.join(dlink, 'core.cfg'))
        check(on is False, "detector fails closed: a symlinked directory")
        # A dev path that is a link to the OEM file must not read as dev.
        os.remove(link)
        os.symlink(oem, link)
        on, _ = detect(oem, link)
        check(on is False, "detector: a dev path that links to the OEM "
              "config is not dev mode")
    else:
        print("note: symlinks unavailable here, symlink cases skipped")

    # an OS error anywhere is NOT dev mode
    real = os.path.realpath
    def boom(*a, **k):
        raise OSError("disk on fire")
    os.path.realpath = boom
    try:
        on, why = detect(dev, dev)
    finally:
        os.path.realpath = real
    check(on is False and why, "detector fails closed: realpath raises")
    real_isfile = os.path.isfile
    def boom2(*a, **k):
        raise PermissionError("denied")
    os.path.isfile = boom2
    try:
        on, why = detect(dev, dev)
    finally:
        os.path.isfile = real_isfile
    check(on is False and why, "detector fails closed: stat raises")

    # Negative control: the detector really can say yes. Without this the
    # cases above would pass for a detector that always says no.
    on, _ = detect(dev, dev)
    check(on is True, "detector negative control: same path says yes again")


def test_detector_in_the_guard():
    # status carries dev_mode and reason, live
    rig = Rig(config_file=OEM_CFG)
    st = rig.guard.get_status(0.)
    check(st['dev_mode'] is False, "status: OEM config -> dev_mode False")
    check(isinstance(st['reason'], str) and st['reason'],
          "status: OEM config has a reason")
    rig = Rig(dev=True)
    st = rig.guard.get_status(0.)
    check(st['dev_mode'] is True, "status: dev clone -> dev_mode True")
    check(isinstance(st['reason'], str) and st['reason'],
          "status: dev mode has a reason")
    # the file is deleted under a running printer: that is not dev mode
    os.remove(rig.dev_cfg)
    rig.guard._cache = None
    st = rig.guard.get_status(1000.)
    check(st['dev_mode'] is False,
          "status: the dev config vanished -> dev_mode False")
    # a crash inside the detector is not dev mode, and is not a crash
    rig = Rig(dev=True)
    real = muon_guard.detect_dev_mode
    def broken(*a, **k):
        raise RuntimeError("boom")
    muon_guard.detect_dev_mode = broken
    try:
        rig.guard._cache = None
        st = rig.guard.get_status(0.)
        err = rig.client('SET_KINEMATIC_POSITION Z=1')
    finally:
        muon_guard.detect_dev_mode = real
    check(st['dev_mode'] is False,
          "status: detector raised -> dev_mode False")
    check(err is not None,
          "gate: detector raised -> client command still refused")


######################################################################
# 2. Entry tagging and the allowlist
######################################################################

def test_client_refused():
    rig = Rig()
    for cmd in MUST_REFUSE:
        err = rig.client(cmd)
        check(err is not None, "client refused outside dev mode: %s" % cmd)
        check(refused_text_ok(err),
              "refusal for %r names dev mode and the allowlist: %r"
              % (cmd, err))
        check(rig.calls == [],
              "refused command never reaches its handler: %s" % cmd)


def test_other_client_doors():
    # Each door a client uses is tagged. webhooks gcode/script and the
    # restart endpoints, the serial pty, and a printed file's lines.
    for door in ('webhooks', 'pty', 'file'):
        rig = Rig()
        err = rig.client('SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z', door)
        check(err is not None, "door %s: client SET_KINEMATIC refused" % door)
        err = rig.client('G1 X10 F6000', door)
        check(err is None and rig.calls == ['G1 X10 F6000'],
              "door %s: a plain G1 passes" % door)


def test_pty_replies_with_error_text():
    # The pty path is need_ack: a refusal is an error line, not an exception.
    rig = Rig()
    rig.out[:] = []
    rig.gcode._process_commands(['SET_KINEMATIC_POSITION Z=1'],
                                origin='pty')
    check(any(l.startswith('!!') and 'dev mode' in l.lower()
              for l in rig.out),
          "pty: the refusal comes back as an error line: %r" % (rig.out,))
    check(rig.calls == [], "pty: the refused command did not run")


def test_macro_inside_is_allowed():
    rig = Rig()
    err = rig.client('G28')
    check(err is None, "client G28 passes")
    check(rig.calls == ['G28', 'SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z',
                        'FORCE_MOVE STEPPER=stepper_z DISTANCE=10 '
                        'VELOCITY=5', '_HOME_Z', 'G28.1 Z',
                        'AXIS_PROTECTED_MOVE AXIS=Z DIST=10'],
          "the same commands a client cannot send run inside G28's macro: "
          "%r" % (rig.calls,))
    # the SAME command, sent directly, is refused
    err = rig.client('SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z')
    check(err is not None, "the same SET_KINEMATIC_POSITION sent directly "
          "is refused")
    # a nested macro: PRINT_START -> G28 -> internals
    err = rig.client('PRINT_START')
    check(err is None and '_HOME_Z_FROM_LAST_PROBE' in rig.calls,
          "client PRINT_START: its sub-commands, two macros deep, pass")
    # a file line that starts a macro is a client start; its inside is not
    err = rig.client('PRINT_START', 'file')
    check(err is None and 'SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z'
          in rig.calls, "file line PRINT_START: macro internals pass")


def test_script_stops_at_first_refusal():
    rig = Rig()
    err = rig.client('M104 S0\nSET_KINEMATIC_POSITION Z=1\nM140 S0')
    check(err is not None, "a script with one bad line is refused")
    check(rig.calls == ['M104 S0'],
          "lines before the refusal ran, lines after did not: %r"
          % (rig.calls,))


def test_internal_sources_pass():
    # idle_timeout, delayed_gcode, the filament sensors, bed_removal_detector,
    # advanced_homing_movement, pause_resume and the rest call
    # gcode.run_script (or run_script_from_command). They are not client
    # doors, so what they run is not checked.
    rig = Rig()
    for cmd in ('SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z',
                'FORCE_MOVE STEPPER=stepper_z DISTANCE=10 VELOCITY=5',
                '_HOME_Z', 'AXIS_PROTECTED_MOVE AXIS=Z DIST=10',
                'SET_CONTACT_GUARD ENABLE=0', 'M104 S0'):
        rig.internal(cmd)
        check(rig.calls == [cmd], "internal run_script passes: %s" % cmd)
        rig.from_macro(cmd)
        check(rig.calls == [cmd],
              "run_script_from_command passes: %s" % cmd)


def test_tag_is_what_decides():
    # Negative control. Without the tag the same line is not refused, so the
    # refusals above are the tag working and not an accident of the rig.
    rig = Rig()
    rig.internal('SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z')
    check(rig.calls == ['SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z'],
          "untagged run_script of SET_KINEMATIC_POSITION is not refused")
    err = rig.client('SET_KINEMATIC_POSITION Z=100 SET_HOMED=Z')
    check(err is not None, "tagged: the same line is refused")
    # Tagging does not leak: after a client script, an internal one is still
    # internal, and a client script after that is still checked.
    rig.internal('FORCE_MOVE STEPPER=stepper_z DISTANCE=1 VELOCITY=1')
    check(len(rig.calls) == 1, "tag does not leak into the next run_script")
    check(rig.client('FORCE_MOVE STEPPER=stepper_z DISTANCE=1 VELOCITY=1')
          is not None, "a later client script is checked again")


def test_refusal_inside_macro_error_path():
    # A client macro whose inside raises must not leave the tag behind.
    rig = Rig()
    def bad(gcmd):
        raise CommandError("macro failed")
    rig.gcode.register_command('PRINT_END', None)
    rig.gcode.register_command('PRINT_END', bad)
    err = rig.client('PRINT_END')
    check(err == 'macro failed' or 'macro failed' in (err or ''),
          "a failing client macro reports its own error")
    rig.internal('SET_KINEMATIC_POSITION Z=1')
    check(rig.calls == ['SET_KINEMATIC_POSITION Z=1'],
          "after a failed client macro, internals still pass")


def test_forms_of_the_same_command():
    rig = Rig()
    for line in ('set_kinematic_position z=100',
                 'N12 SET_KINEMATIC_POSITION Z=100*55',
                 '  SET_KINEMATIC_POSITION Z=100',
                 'Set_Kinematic_Position Z=1 ; G1 X1',
                 'SET_KINEMATIC_POSITION',
                 'g28.1 z',
                 '_home_z'):
        err = rig.client(line)
        check(err is not None, "refused in any spelling: %r" % (line,))
        check(rig.calls == [], "not run in any spelling: %r" % (line,))
    # comments and blanks are not commands
    for line in ('', '   ', '; SET_KINEMATIC_POSITION Z=1', 'G1 X1 ; hello'):
        err = rig.client(line)
        check(err is None, "comment or blank line passes: %r" % (line,))
    # M117 and M118 messages carry free text; the first word is the command
    err = rig.client('M117 Printing benchy 3 of 4')
    check(err is None, "M117 with free text passes")
    err = rig.client('M118 hello')
    check(err is None, "M118 with free text passes")
    err = rig.client('SET_KINEMATIC_POSITION Z=1 M117')
    check(err is not None, "M117 later in the line does not smuggle it")


def test_homing_gate_callback():
    rig = Rig()
    for line in ('HOMING_GATE CALLBACK=G28',
                 'HOMING_GATE CALLBACK="G28 X Y"',
                 'HOMING_GATE CALLBACK="G28 O"',
                 'homing_gate callback=G28'):
        check(rig.client(line) is None,
              "HOMING_GATE with a G28 callback passes: %s" % line)
    for line in ('HOMING_GATE CALLBACK=G28.1',
                 'HOMING_GATE CALLBACK=SET_KINEMATIC_POSITION',
                 'HOMING_GATE CALLBACK="FORCE_MOVE STEPPER=stepper_z"',
                 'HOMING_GATE CALLBACK=_HOME_Z',
                 'HOMING_GATE CALLBACK='):
        check(rig.client(line) is not None,
              "HOMING_GATE with another callback is refused: %s" % line)


def test_allowlist_commands_pass():
    entries = [e.strip() for e in core_allow().split('\n') if e.strip()]
    check(len(entries) > 40, "core.cfg lists an allowlist (%d entries)"
          % len(entries))
    rig = Rig()
    for entry in entries:
        parts = shlex.split(entry)
        name, cons = parts[0], parts[1:]
        line = name
        for c in cons:
            k, _, v = c.partition('=')
            line += ' %s=%s' % (k, shlex.quote(v.replace('*', 'X')))
        for door in ('webhooks', 'pty', 'file'):
            err = rig.client(line, door)
            check(err is None, "allowlisted passes via %s: %r (%s)"
                  % (door, line, err))
        check(len(rig.calls) >= 1, "allowlisted command ran: %r" % (line,))


def test_dev_mode_allows_all():
    rig = Rig(dev=True)
    st = rig.guard.get_status(0.)
    check(st['dev_mode'] is True, "rig really is in dev mode")
    for cmd in MUST_REFUSE:
        if cmd.split()[0] == 'NOT_A_COMMAND':
            continue
        err = rig.client(cmd)
        check(err is None, "dev mode allows: %s (%s)" % (cmd, err))
        check(len(rig.calls) == 1, "dev mode runs it: %s" % (cmd,))
    # and the same through the other doors
    for door in ('pty', 'file'):
        check(rig.client('SET_KINEMATIC_POSITION Z=1', door) is None,
              "dev mode allows SET_KINEMATIC_POSITION via %s" % door)
    # leaving dev mode (the config is the OEM tree again) refuses again
    rig2 = Rig(config_file=OEM_CFG)
    check(rig2.client('SET_KINEMATIC_POSITION Z=1') is not None,
          "back on the OEM config the same command is refused")


def test_guard_fails_closed():
    rig = Rig()
    real = rig.guard._decide
    def boom(*a, **k):
        raise RuntimeError("bug in the guard")
    rig.guard._decide = boom
    err = rig.client('G1 X10')
    check(err is not None and 'dev mode' in err.lower(),
          "a bug inside the gate refuses, it does not allow: %r" % (err,))
    check(rig.printer.shutdowns == [],
          "a bug inside the gate does not shut the printer down")
    rig.guard._decide = real
    check(rig.client('G1 X10') is None, "and the gate works again after")


def test_virtual_sdcard_lines():
    # Decision: a printed file's lines are CLIENT commands. A file is
    # uploaded by a client, so if its lines were trusted, a client could
    # print SET_KINEMATIC_POSITION. Macros a line starts are still trusted
    # inside (test_macro_inside_is_allowed). The failure handler
    # (on_error_gcode) is the machine's own and stays internal.
    path = os.path.join(ROOT, 'klippy', 'extras', 'virtual_sdcard.py')
    with open(path, encoding='utf-8') as f:
        src = f.read()
    check("self.gcode.run_client_script(line, 'file')" in src,
          "virtual_sdcard sends each file line in as a 'file' client line")
    check("self.gcode.run_script(self.on_error_gcode.render())" in src,
          "virtual_sdcard keeps on_error_gcode internal")
    check("self.gcode.run_script(line)" not in src,
          "virtual_sdcard no longer runs file lines untagged")


def test_door_inventory():
    # The set of places a client command can enter, pinned. A new door is a
    # reviewed change to this list, not a quiet addition.
    expected = {
        os.path.join('klippy', 'webhooks.py'): 3,
        os.path.join('klippy', 'extras', 'virtual_sdcard.py'): 1,
    }
    found = {}
    for base, _, files in os.walk(os.path.join(ROOT, 'klippy')):
        for fn in files:
            if not fn.endswith('.py'):
                continue
            p = os.path.join(base, fn)
            with open(p, encoding='utf-8') as f:
                n = f.read().count('run_client_script(')
            if n:
                found[os.path.relpath(p, ROOT)] = n
    found.pop(os.path.join('klippy', 'gcode.py'), None)   # the definition
    check(found == expected,
          "run_client_script call sites are exactly the known doors: %r"
          % (found,))
    with open(os.path.join(ROOT, 'klippy', 'gcode.py'),
              encoding='utf-8') as f:
        src = f.read()
    check("self.gcode._process_commands(pending_commands, origin='pty')"
          in src or "_process_commands(pending_commands, origin='pty')"
          in src, "the serial pty is tagged as a client door")
    with open(os.path.join(ROOT, 'klippy', 'webhooks.py'),
              encoding='utf-8') as f:
        wsrc = f.read()
    check("run_script(web_request.get_str('script'))" not in wsrc.replace(
        'run_client_script(', 'X'), "webhooks gcode/script is tagged")
    check("run_script('restart')" not in wsrc
          and "run_script('firmware_restart')" not in wsrc,
          "webhooks restart endpoints are tagged")


######################################################################
# 3. Config: the allowlist is data, and it is reviewed
######################################################################

def test_config_errors():
    printer = FakePrinter(OEM_CFG)
    def build(values):
        p = FakePrinter(OEM_CFG)
        p.add_object('gcode', gcode_mod.GCodeDispatch(p))
        return muon_guard.load_config(FakeConfig(p, values))
    for label, values in (
            ("no client_allow", {}),
            ("an empty allowlist", {'client_allow': ''}),
            ("a bad parameter spec", {'client_allow': 'G28\nHOMING_GATE foo'}),
            ("a lowercase name", {'client_allow': 'g28'}),
            ("a name with a space inside quotes",
             {'client_allow': '"G 28"'}),
            ("a glob in the name", {'client_allow': 'G*'})):
        try:
            build(values)
            ok = False
        except ConfigError:
            ok = True
        check(ok, "config error: %s" % (label,))
    # a second gate cannot replace the first
    p = FakePrinter(OEM_CFG)
    g = gcode_mod.GCodeDispatch(p)
    p.add_object('gcode', g)
    muon_guard.load_config(FakeConfig(p, {'client_allow': 'G1'}))
    try:
        muon_guard.load_config(FakeConfig(p, {'client_allow': 'G1'}))
        ok = False
    except ConfigError:
        ok = True
    check(ok, "a second muon_guard cannot replace the registered gate")
    del printer


def macros_in_core():
    names = []
    for fn in sorted(os.listdir(MACRO_DIR)):
        if fn.endswith('.cfg'):
            with open(os.path.join(MACRO_DIR, fn), encoding='utf-8') as f:
                names += re.findall(r'^\[gcode_macro ([^\]\s]+)\]', f.read(),
                                    re.M)
    with open(CORE_CFG, encoding='utf-8') as f:
        names += re.findall(r'^\[gcode_macro ([^\]\s]+)\]', f.read(), re.M)
    return names


# Macros a client may NOT start, each with the reason. A macro that is in
# neither this table nor the core.cfg allowlist fails the test, so adding a
# macro forces someone to decide.
NOT_FOR_CLIENTS = {
    '_CLIENT_VARIABLE': 'variable holder',
    'LED_TIMEOUT_STATE': 'state holder',
    'BUZZ_EXTRUDER': 'debug',
    'WEAR_TEST': 'debug',
    'TEST_SPEED': 'debug',
    'PID_CALIBRATE': 'heats to a target; calibration is a dev tool',
    'BED_REMOVED': 'raised by bed_removal_detector, not by a client',
    'BED_ATTACHED': 'raised by bed_removal_detector, not by a client',
    'TOOLHEAD_CONNECT': 'raised by the toolhead hot-plug path',
    'TOOLHEAD_DISCONNECT': 'raised by the toolhead hot-plug path',
    'TOOLHEAD_UMBILICAL_UNLOCK_STATE': 'internal arm state',
    'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED': 'raised by gcode_button',
    'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_RELEASED': 'raised by gcode_button',
    'HOTEND_FAN_CHECK': 'delayed_gcode / fault path',
    'MD_FAN_CHECK': 'delayed_gcode / fault path',
    'PSU_FAN_CHECK': 'delayed_gcode / fault path',
    'CPU_FAN_CHECK': 'delayed_gcode / fault path',
    'FAN_STOPPAGE_ROUTINE': 'fault path',
    'FILAMENT_INSERT_FLOW': 'raised by the filament sensor insert_gcode',
    'FILAMENT_INSERT_MODE_PROMPT': 'step of the insert flow',
    'FILAMENT_INSERT_SENSORLESS_RESULT': 'step of the insert flow',
    'FILAMENT_INSERT_LOAD_FAILED_PROMPT': 'step of the insert flow',
    'FILAMENT_INSERT_FULL_PURGE': 'step of the insert flow',
    'FILAMENT_INSERT_GRAB_MORE': 'step of the insert flow',
    'IF_BED_TEMP_ANTI_OOZE_NOZZLE_DOCK': 'step of PRINT_START',
    'ANTI_OOZE_NOZZLE_DOCK': 'step of PRINT_START',
    'CANCEL_AND_CLOSE': 'only a prompt button inside HOMING_GATE',
}


def test_every_macro_is_classified():
    allow = [e.strip() for e in core_allow().split('\n') if e.strip()]
    allowed = set(shlex.split(e)[0] for e in allow)
    # underscore macros a client may start: the targets of prompt buttons
    # the printer itself puts on the screen, which Control or the panel send
    # back verbatim. Nothing else with an underscore is on the list.
    prompt_targets = set()
    for fn in sorted(os.listdir(MACRO_DIR)):
        if fn.endswith('.cfg'):
            with open(os.path.join(MACRO_DIR, fn), encoding='utf-8') as f:
                for m in re.finditer(
                        r'^\s*RESPOND TYPE=command MSG="action:prompt_'
                        r'(?:footer_)?button [^|"]*\|(\w+)', f.read(),
                        re.M):
                    prompt_targets.add(m.group(1))
    underscore_allowed = set(n for n in allowed if n.startswith('_'))
    check(underscore_allowed <= prompt_targets,
          "underscore macros on the allowlist are all prompt-button "
          "targets: %r" % (sorted(underscore_allowed - prompt_targets),))
    check(underscore_allowed == set([
        '_BED_RESUME_NOW', '_CANCEL_M600', '_CLOSE_CALIBRATE_PROBE',
        '_DO_CALIBRATE_PROBE', '_DO_UNLOAD_M600', '_HELP_CALIBRATE_PROBE']),
        "the underscore exceptions are exactly the six reviewed ones: %r"
        % (sorted(underscore_allowed),))
    for name in macros_in_core():
        if name.startswith('_') and name not in underscore_allowed:
            continue
        if name in allowed:
            continue
        check(name in NOT_FOR_CLIENTS,
              "macro %s is neither on the allowlist nor in NOT_FOR_CLIENTS"
              % (name,))
    for name in NOT_FOR_CLIENTS:
        check(name not in allowed,
              "%s is listed as not for clients but is on the allowlist"
              % (name,))
    for name in ('SET_KINEMATIC_POSITION', 'FORCE_MOVE', 'SET_CONTACT_GUARD',
                 'SET_GCODE_VARIABLE', 'SAVE_CONFIG', 'SAVE_VARIABLE',
                 'SET_TMC_CURRENT', 'SET_STEPPER_ENABLE', 'STEPPER_BUZZ',
                 'AXIS_PROTECTED_MOVE', 'AXIS_HOME_MOVE', 'G28.1',
                 'SET_BED_REMOVAL_DETECTOR', 'SET_IDLE_TIMEOUT'):
        check(name not in allowed, "%s is not on the allowlist" % (name,))
    for name in ('G0', 'G1', 'G28', 'M104', 'M140', 'M106', 'M109', 'M190',
                 'M84', 'M112', 'PRINT_START', 'PRINT_END', 'PAUSE',
                 'RESUME', 'CANCEL_PRINT', 'SDCARD_PRINT_FILE',
                 'SET_HEATER_TEMPERATURE', 'TURN_OFF_HEATERS'):
        check(name in allowed, "%s is on the allowlist" % (name,))


def test_allowlist_has_a_reason_per_entry():
    # The allowlist is reviewed config, so each entry is commented. A group
    # header comment above a run of entries counts as the reason.
    with open(CORE_CFG, encoding='utf-8') as f:
        text = f.read().replace('\r\n', '\n')
    m = re.search(r'^\[muon_guard\]\n(.*?)(?=^\[|\Z)', text, re.S | re.M)
    body = m.group(1)
    start = body.index('client_allow:')
    lines = body[start:].split('\n')[1:]
    reason = False
    for line in lines:
        s = line.strip()
        if not s:
            continue
        if s.startswith('#'):
            reason = True
            continue
        check(reason or '#' in s, "allowlist entry has a reason: %r" % (s,))


def test_muon_guard_is_loaded_and_not_overridable():
    with open(CORE_CFG, encoding='utf-8') as f:
        text = f.read()
    check(re.search(r'^\[muon_guard\]', text, re.M) is not None,
          "core.cfg loads [muon_guard]")
    opts = read_core_section()
    check(opts.get('dev_config_path') == DEV_CFG,
          "core.cfg names the dev-mode config path MuonOS writes: %r"
          % (opts.get('dev_config_path'),))
    for sec in ('muon_guard', 'contact_guard'):
        check(sec in configfile.SAFETY_SECTIONS,
              "[%s] is a safety section: the calibration overlay cannot "
              "say anything about it" % (sec,))
    # the overlay really is refused
    over = configparser.RawConfigParser()
    over.read_string('[muon_guard]\nclient_allow: SET_KINEMATIC_POSITION\n')
    core = configparser.RawConfigParser()
    core.read_string('[muon_guard]\nclient_allow: G1\n')
    conflicts = configfile.find_calibration_conflicts(over, core)
    check(len(conflicts) == 1 and 'muon_guard' in conflicts[0],
          "the overlay is refused when it mentions [muon_guard]: %r"
          % (conflicts,))


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        try:
            t()
        except Exception as e:
            import traceback
            traceback.print_exc()
            FAILURES.append("%s raised %s: %s" % (t.__name__,
                                                  type(e).__name__, e))
    if FAILURES:
        print("\n%d FAILURE(S)" % (len(FAILURES),))
        for f in FAILURES:
            print(" - " + f)
        return 1
    print("ok: %d tests" % (len(tests),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
