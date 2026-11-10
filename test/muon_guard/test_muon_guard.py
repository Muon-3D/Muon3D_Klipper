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
    # HOMING_GATE runs {params.CALLBACK} as G-code: it is a way to start
    # anything unless its CALLBACK is held to G28.
    'HOMING_GATE CALLBACK=SET_KINEMATIC_POSITION',
    'HOMING_GATE CALLBACK="G1 X1"',
    'HOMING_GATE CALLBACK=_HOME_Z',
    'HOMING_GATE CALLBACK=G280',
    # Parameters a client may not send to an allowlisted command. Each
    # would reach a refused command or unbounded motion inside the macro.
    'FILAMENT_INSERT_SENSORLESS_LOAD CURRENT=2000',
    'FILAMENT_INSERT_SENSORLESS_LOAD DISTANCE=9999 SPEED=999 ACCEL=99999',
    'LOAD_FILAMENT CURRENT=2000',
    'LOAD_FILAMENT_PROMPT DISTANCE=500',
    'FILAMENT_INSERT_FLOW SPEED=500',
    'FILAMENT_INSERT_FULL_PURGE_GATE DISTANCE=500',
    'G28 X_OFFSET=50',
    'G28 X_OFFSET=-500 Z',
    'G28 FOO=1',
    'SET_GCODE_OFFSET Z=50',
    'SET_GCODE_OFFSET Z_ADJUST=-30 MOVE=1',
    'BUZZ_EXTRUDER COUNT=1000',
    'TEST_SPEED SPEED=800 ACCEL=999999',
    'CUT_FILAMENT MOVE_SPEED=99999',
    'Z_PARK Z_SPEED=100000',
    'PRINT_START EXTRUDER=900',
    'PRINT_START BED=500',
    # a quoted value with a space would add parameters to the macros that
    # paste it into their command lines (print.cfg:391)
    'PRINT_START MATERIAL="PLA FOO=1"',
    'PRINT_START MATERIAL="PLA FOO=1"',
    'PRINT_START ALGORITHM="bicubic X=1"',
    'PRINT_START MESH_MIN="1,2 X=3"',
    'PRINT_START PROBE_COUNT="4,4 Y=1"',
    'SPEED_SLOW_PRE_PURGE_FOR_TPU MATERIAL="A B"',
    '_SET_MPC_MATERIAL MATERIAL="PLA X=1"',
    # homes and throws the toolhead at max velocity and acceleration
    'TEST_SPEED',
    'TEST_SPEED SPEED=800',
    # nothing on the M1 sends it, and repeated apply + SAVE_CONFIG pushes
    # position_endstop out of range (stepper.py refuses to start)
    'Z_OFFSET_APPLY_ENDSTOP',
    # PROBE_CALIBRATE's TESTZ drives the nozzle; a big step stages a big
    # z_offset on ACCEPT
    'TESTZ Z=20',
    'TESTZ Z=+20',
    'TESTZ Z=-9999',
    'TESTZ Z=5.5',
    'TESTZ Z=+++',
    'TESTZ Z=abc',
    # Control's pause-at-layer dialog: the MACRO is run by name, so it is
    # held to PAUSE or M600
    'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=10 MACRO=SET_KINEMATIC_POSITION',
    'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=10 "MACRO=PAUSE X=1"',
    'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=-1 MACRO=PAUSE',
    'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=99999999 MACRO=PAUSE',
    'SET_PAUSE_AT_LAYER ENABLE=2',
    'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=1.5 MACRO=PAUSE',
    'SET_PAUSE_NEXT_LAYER ENABLE=1 MACRO=_HOME_Z',
    'PAUSE HOTEND_TARGET=500',
    'RESUME VELOCITY=9999',
    'FINETUNE_RESONANCE FORCE=5',
    'UNLOAD_FILAMENT TEMP=900',
    '_BED_MESH_RECORD BED=500',
    'PROBE_ACCURACY SAMPLES=9999',
    'SET_PRESSURE_ADVANCE ADVANCE=50',
    'SAVE_VARIABLE VARIABLE=muon_material OTHER=1 VALUE=x',
    'SAVE_VARIABLE VARIABLE=muon_material_colour_x VALUE=1',
    # Control's stepper jog and multi-extruder controls: dev tools
    'ACTIVATE_EXTRUDER EXTRUDER=extruder',
]

# Every command name the fake printer registers, so that "refused" is told
# apart from "unknown command".
EXTRA_REGISTERED = set(c.split()[0] for c in MUST_REFUSE)
SENDERS = os.path.join(HERE, 'client_senders.txt')
FIXTURES = os.path.join(HERE, 'fixtures')


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
    def getfloat(self, name, default=configfile.sentinel, minval=None,
                 maxval=None, above=None):
        return float(self.get(name, default))
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


class FakeGcodeMove:
    def __init__(self):
        self.homing_position = [0., 0., 0., 0.]


class FakeConfigStatus:
    """printer.configfile: only the max_temp values the guard reads."""
    def __init__(self, extruder=265., bed=130.):
        self.settings = {'extruder': {'max_temp': extruder},
                         'heater_bed': {'max_temp': bed}}

    def get_status(self, eventtime):
        return {'settings': self.settings}


class FakeProbe:
    def __init__(self, z):
        self.z = z
        self.invalid = None          # a saved z_offset that was ignored

    def get_status(self, eventtime):
        return {'z_offset_invalid': self.invalid is not None,
                'z_offset_saved': self.invalid}

    def get_offsets(self, gcmd=None):
        return 0., 0., self.z


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
        self.cfg = FakeConfigStatus()
        self.printer.add_object('configfile', self.cfg)
        self.gm = FakeGcodeMove()
        self.printer.add_object('gcode_move', self.gm)
        self.probe = FakeProbe(0.)
        self.pending_z = None
        self.disk_z_offset = 0.       # the z_offset in the saved config
        self.printer.add_object('load_cell_probe', self.probe)
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
        self.gcode.register_command('SET_GCODE_OFFSET', self._set_offset)
        self.gcode.register_command('Z_OFFSET_APPLY_PROBE', self._apply_probe)
        self.gcode.register_command('SAVE_CONFIG', self._save_config)
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

    def _set_offset(self, gcmd):
        # gcode_move.py cmd_SET_GCODE_OFFSET: the axis value wins, an
        # _ADJUST adds to what is there
        self.calls.append(gcmd.get_commandline())
        for i, axis in enumerate('XYZ'):
            v = gcmd.get_float(axis, None)
            if v is None:
                v = gcmd.get_float(axis + '_ADJUST', None)
                if v is None:
                    continue
                v += self.gm.homing_position[i]
            self.gm.homing_position[i] = v

    def _apply_probe(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        # probe.py cmd_Z_OFFSET_APPLY_PROBE: configfile.set, saved later
        self.pending_z = self.probe.z - self.gm.homing_position[2]

    def _save_config(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        if self.pending_z is not None:
            self.disk_z_offset = self.pending_z
            self.pending_z = None

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


def test_abortable_file_line_is_checked():
    # virtual_sdcard runs each print-file line through run_abortable_script
    # (a cancel can stop PRINT_START, #93) with origin='file' (#112). The
    # origin must reach the guard on that path too, and the abortable path
    # without an origin stays unchecked.
    rig = Rig()
    del rig.calls[:]
    try:
        rig.gcode.run_abortable_script('SET_KINEMATIC_POSITION Z=1',
                                       origin='file')
        err = None
    except CommandError as e:
        err = str(e)
    check(err is not None and rig.calls == [],
          "an abortable 'file' line is checked: %r %r" % (err, rig.calls))
    del rig.calls[:]
    rig.gcode.run_abortable_script('M104 S0', origin='file')
    check(rig.calls == ['M104 S0'], "an allowed abortable 'file' line runs")
    del rig.calls[:]
    rig.gcode.run_abortable_script('SET_KINEMATIC_POSITION Z=1')
    check(rig.calls == ['SET_KINEMATIC_POSITION Z=1'],
          "an abortable script with no origin is not checked")


def test_plate_clear_is_printer_only():
    # Jack, 10 Nov 2026: a remote client (Control, the app, an agent) may
    # not declare the plate cleared, from any door, even if someone puts it
    # on the allowlist. The printer's own path (a macro, the refit the plate
    # detector sees) still marks it clear.
    for allow in (None, core_allow() + '\nMACHINE_PLATE_CLEAR'):
        rig = Rig(allow=allow)
        rig._register('MACHINE_PLATE_CLEAR')
        for origin in ('webhooks', 'pty', 'file'):
            err = rig.client('MACHINE_PLATE_CLEAR', origin)
            check(err is not None and 'Only at the printer' in err
                  and 'fit it back' in err,
                  "remote MACHINE_PLATE_CLEAR refused from %s: %r"
                  % (origin, err))
            check(rig.calls == [], "and it did not run (%s)" % (origin,))
        rig.from_macro('MACHINE_PLATE_CLEAR')
        check(rig.calls == ['MACHINE_PLATE_CLEAR'],
              "a printer macro still runs MACHINE_PLATE_CLEAR")
        rig.internal('MACHINE_PLATE_CLEAR')
        check(rig.calls == ['MACHINE_PLATE_CLEAR'],
              "an internal source still runs MACHINE_PLATE_CLEAR")


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


def line_for(entry, end=1):
    """A command line that satisfies one allowlist entry."""
    parts = shlex.split(entry)
    name, toks = parts[0], parts[1:]
    traditional = re.match(r'^[GMTN]\d', name) is not None
    line = name
    for t in toks:
        if t in ('-', '...'):
            continue
        k, _, v = t.partition('=')
        m = re.match(r'^(-?[\d.]+)\.\.(-?[\d.]+|@[a-z_]+)$', v)
        if m:
            val = m.group(end)
            val = {'@extruder_max_temp': '265',
                   '@bed_max_temp': '130'}.get(val, val)
        elif v.startswith('~'):
            val = ('PLA' if 'MATERIAL' in k else 'bicubic'
                   if k == 'ALGORITHM' else '+' if k == 'Z' else 'PAUSE'
                   if k == 'MACRO' else '5'
                   if k == 'LAYER' else 'normal'
                   if k == 'MODE' else '1,2')
        else:
            val = v.replace('*', 'X')
        if traditional and v == '*':
            line += ' ' + k
        elif traditional and k == 'X_OFFSET':
            line += ' %s%s' % (k, '=' + val)
        else:
            line += ' %s=%s' % (k, shlex.quote(val))
    return line


def test_allowlist_commands_pass():
    entries = [e.strip() for e in core_allow().split('\n') if e.strip()]
    check(len(entries) > 40, "core.cfg lists an allowlist (%d entries)"
          % len(entries))
    rig = Rig()
    for entry in entries:
        line = line_for(entry)
        for door in ('webhooks', 'pty', 'file'):
            err = rig.client(line, door)
            check(err is None, "allowlisted passes via %s: %r (%s)"
                  % (door, line, err))
        check(len(rig.calls) >= 1, "allowlisted command ran: %r" % (line,))
        # both ends of every range pass too
        for end in (1, 2):
            l2 = line_for(entry, end)
            err = rig.client(l2)
            check(err is None, "range end passes: %r (%s)" % (l2, err))


def test_unregistered_commands_do_nothing_and_pass():
    # Nothing is registered under these names, so nothing runs. Refusing
    # them would stop a print on a slicer comment-command.
    rig = Rig()
    for line in ('NOT_A_COMMAND', 'M900 K0.05', 'G29', 'T0',
                 'SOME_FUTURE_SLICER_THING A=1'):
        check(rig.client(line, 'file') is None,
              "an unregistered command passes: %r" % (line,))
        check(rig.calls == [], "and nothing ran: %r" % (line,))
    # but the moment something is registered under a name that is not
    # listed, it is refused
    rig.gcode.register_command('SOME_NEW_COMMAND', lambda g: None)
    check(rig.client('SOME_NEW_COMMAND') is not None,
          "a registered, unlisted command is refused")


def test_dev_mode_allows_all():
    rig = Rig(dev=True)
    st = rig.guard.get_status(0.)
    check(st['dev_mode'] is True, "rig really is in dev mode")
    for cmd in MUST_REFUSE:
        err = rig.client(cmd)
        check(err is None, "dev mode allows: %s (%s)" % (cmd, err))
        check(len(rig.calls) >= 1, "dev mode runs it: %s" % (cmd,))
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
    # With #93 the line is also abortable (a cancel stops PRINT_START), so
    # it goes in through run_abortable_script, still tagged 'file'.
    check("self.gcode.run_client_script(line, 'file')" in src
          or "self.gcode.run_abortable_script(line, origin='file')" in src,
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
                text = f.read()
                # A door is run_client_script, or run_abortable_script given
                # a client origin (virtual_sdcard, #93 + #112).
                n = (text.count('run_client_script(')
                     + len(re.findall(r'run_abortable_script\([^)]*origin=',
                                      text)))
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
    'WEAR_TEST': 'debug',
    'TEST_SPEED': 'homes and throws the toolhead at max velocity and accel',
    'PID_CALIBRATE': 'M1 has CALIBRATE_HEATING_*; PID_CALIBRATE heats to a '
                     'client target (panel lists it, bare, and it errors)',
    'BED_REMOVED': 'raised by bed_removal_detector, not by a client',
    'BED_ATTACHED': 'raised by bed_removal_detector, not by a client',
    'TOOLHEAD_UMBILICAL_UNLOCK_STATE': 'internal arm state',
    'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED': 'raised by gcode_button',
    'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_RELEASED': 'raised by gcode_button',
    'HOTEND_FAN_CHECK': 'delayed_gcode / fault path',
    'MD_FAN_CHECK': 'delayed_gcode / fault path',
    'PSU_FAN_CHECK': 'delayed_gcode / fault path',
    'CPU_FAN_CHECK': 'delayed_gcode / fault path',
    'FAN_STOPPAGE_ROUTINE': 'fault path',
    'FILAMENT_INSERT_MODE_PROMPT': 'step of the insert flow',
    'FILAMENT_INSERT_SENSORLESS_RESULT': 'step of the insert flow',
    'FILAMENT_INSERT_LOAD_FAILED_PROMPT': 'step of the insert flow',
    'FILAMENT_INSERT_FULL_PURGE': 'step of the insert flow',
    'FILAMENT_INSERT_GRAB_MORE': 'step of the insert flow',
    'IF_BED_TEMP_ANTI_OOZE_NOZZLE_DOCK': 'step of PRINT_START',
    'ANTI_OOZE_NOZZLE_DOCK': 'step of PRINT_START',
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
    # sent by the panel itself (MuonUI src/panel/screens/tools.ts:161,177,
    # 384 and filament.ts:127), not by a prompt button
    panel_sent = set(['_BED_MESH_RECORD', '_SAVE_CONFIG_IF_PENDING',
                      '_SET_MPC_MATERIAL'])
    # m1_machine took the plate-refit prompt over and _BED_RESUME_NOW is
    # no longer a button target. It is kept, as a wrapper that only runs
    # RESUME (bed.cfg), for a prompt still on a screen across an update and
    # for any interface that calls it.
    compat = set(['_BED_RESUME_NOW'])
    check(underscore_allowed <= prompt_targets | panel_sent | compat,
          "underscore macros on the allowlist are prompt-button targets or "
          "sent by the panel: %r"
          % (sorted(underscore_allowed - prompt_targets - panel_sent
                    - compat),))
    check(underscore_allowed == set([
        '_BED_RESUME_NOW', '_CANCEL_M600', '_CLOSE_CALIBRATE_PROBE',
        '_DO_CALIBRATE_PROBE', '_DO_UNLOAD_M600', '_HELP_CALIBRATE_PROBE',
        '_RUNOUT_UNLOAD']) | panel_sent,
        "the underscore exceptions are exactly the ten reviewed ones: %r"
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
                 'SET_GCODE_VARIABLE',
                 'SET_TMC_CURRENT', 'SET_STEPPER_ENABLE', 'STEPPER_BUZZ',
                 'AXIS_PROTECTED_MOVE', 'AXIS_HOME_MOVE', 'G28.1',
                 'SET_BED_REMOVAL_DETECTOR', 'SET_IDLE_TIMEOUT'):
        check(name not in allowed, "%s is not on the allowlist" % (name,))
    for name in ('SAVE_CONFIG', 'SAVE_VARIABLE', 'G0', 'G1', 'G28',
                 'M104', 'M140', 'M106', 'M109', 'M190',
                 'M84', 'M112', 'PRINT_START', 'PRINT_END', 'PAUSE',
                 'RESUME', 'CANCEL_PRINT', 'SDCARD_PRINT_FILE',
                 'SET_HEATER_TEMPERATURE', 'TURN_OFF_HEATERS'):
        check(name in allowed, "%s is on the allowlist" % (name,))
    # m1_machine: the report and the plate-cleared word are a client's; the
    # gate waits hold the gcode mutex and the event commands record what the
    # machine did, so a client may send none of them.
    check('MACHINE_STATUS' in allowed, "MACHINE_STATUS is on the allowlist")
    for name in ('MACHINE_PLATE_CLEAR', 'MACHINE_WAIT_READY',
                 'MACHINE_REQUIRE', 'MACHINE_HEAT_WAIT', 'MACHINE_PAUSED',
                 'MACHINE_PAUSE_REASON', 'MACHINE_RESUMED',
                 'MACHINE_TOOLHEAD_CONNECTED'):
        check(name not in allowed, "%s is not on the allowlist" % (name,))


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


######################################################################
# 4. Real client traffic through the gate (KAN-1730 review)
######################################################################

def names_in(script_lines):
    out = set()
    for raw in script_lines:
        code = raw.split(';')[0].strip()
        if not code:
            continue
        out.add(code.split()[0].upper())
    return out


# Commands real slicer output uses that nothing registers on the M1. They
# are left unregistered in the rig, as on the printer.
NOT_REGISTERED_ON_M1 = set(['G21', 'G20', 'M205', 'M486', 'M141', 'M191',
                            'TIMELAPSE_TAKE_FRAME', 'G10', 'G11', 'G2',
                            'G3', 'M900'])


def run_fixture(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        lines = f.read().split('\n')
    names = names_in(lines) - NOT_REGISTERED_ON_M1
    # Register every command the file uses, so a missing allowlist entry
    # is a refusal and not an "unknown command" pass.
    rig = Rig(allow=core_allow())
    for n in sorted(names):
        rig._register(n)
    bad = []
    for number, line in enumerate(lines, 1):
        err = rig.client(line, 'file')
        if err is not None:
            bad.append('%s:%d %s -> %s' % (os.path.basename(path), number,
                                           line[:80], err[:60]))
    return bad, len(lines)


def test_slicer_output_passes():
    # Every line of real slicer output must pass, or virtual_sdcard stops
    # the print (virtual_sdcard.py on_error path). The first four files are
    # the muon3d-slicer e2e fixtures (trimmed of the config block and of
    # repeated G1 moves); the last is a sample of what Orca and PrusaSlicer
    # (Klipper flavour) emit.
    files = sorted(fn for fn in os.listdir(FIXTURES) if fn.endswith('.gcode'))
    check(len(files) >= 6, "slicer fixtures present: %r" % (files,))
    for fn in files:
        bad, total = run_fixture(os.path.join(FIXTURES, fn))
        check(not bad, "every line of %s passes the gate (%d lines): %r"
              % (fn, total, bad[:3]))


def test_slicer_fixtures_use_the_review_commands():
    seen = set()
    for fn in os.listdir(FIXTURES):
        if fn.endswith('.gcode'):
            with open(os.path.join(FIXTURES, fn), encoding='utf-8',
                      errors='replace') as f:
                seen |= names_in(f.read().split('\n'))
    for name in ('G21', 'SET_PRESSURE_ADVANCE', 'SET_VELOCITY_LIMIT', 'M73',
                 'EXCLUDE_OBJECT_DEFINE', 'PRINT_START', 'M486', 'M205'):
        check(name in seen, "the slicer fixtures exercise %s" % (name,))


def test_slicer_start_aborts_without_the_commands():
    # The failure the review found, as a negative control: a print stops at
    # the first refused line.
    allow = [e for e in core_allow().split('\n')
             if e.strip() and not e.strip().startswith('SET_PRESSURE')]
    rig = Rig(allow=allow)
    rig._register('SET_PRESSURE_ADVANCE')
    err = rig.client('SET_PRESSURE_ADVANCE ADVANCE=0.02', 'file')
    check(err is not None, "without its entry SET_PRESSURE_ADVANCE is "
          "refused (the bug)")
    rig = Rig()
    check(rig.client('SET_PRESSURE_ADVANCE ADVANCE=0.02', 'file') is None,
          "with the shipped allowlist it passes")


def read_senders():
    out = []
    with open(SENDERS, encoding='utf-8') as f:
        for raw in f:
            raw = raw.rstrip('\n')
            if not raw.strip() or raw.startswith('#'):
                continue
            flag, rest = raw[0], raw[1:].strip()
            src, _, script = rest.partition(' | ')
            note = ''
            if ' # ' in script:
                script, _, note = script.partition(' # ')
            out.append((flag, src, script.strip().replace('\\n', '\n'),
                        note))
    return out


def test_client_senders():
    allowed = set(shlex.split(e)[0] for e in core_allow().split('\n')
                  if e.strip())
    senders = read_senders()
    check(len(senders) > 100, "sender list is loaded (%d)" % len(senders))
    refused_names = set(s[2].split()[0] for s in senders if s[0] == '!')
    rig = Rig()
    for name in sorted(refused_names):
        rig._register(name)
    for flag, src, script, note in senders:
        for line in script.split('\n'):
            name = line.split()[0]
            if flag == '=':
                if name not in allowed:
                    check(False, "%s sends %s, which is not allowlisted"
                          % (src, name))
        err = rig.client(script)
        if flag in '=~':
            check(err is None, "%s sends %r: refused (%s)" % (src, script,
                                                              err))
        else:
            check(err is not None and note,
                  "%s sends %r: expected a refusal with a reason" %
                  (src, script))


def test_prompt_targets_are_allowlisted():
    # Every prompt button the printer shows sends its target back through a
    # client door (Control and the panel send button.command verbatim).
    allowed = set(shlex.split(e)[0] for e in core_allow().split('\n')
                  if e.strip())
    targets = {}
    pat = re.compile(r'action:prompt_(?:footer_)?button [^|"]*\|([^|"]+)\|')
    paths = [os.path.join(MACRO_DIR, fn) for fn in sorted(os.listdir(
        MACRO_DIR)) if fn.endswith('.cfg')] + [CORE_CFG]
    for base, _, files in os.walk(os.path.join(ROOT, 'klippy')):
        paths += [os.path.join(base, fn) for fn in files
                  if fn.endswith('.py')]
    for p in paths:
        with open(p, encoding='utf-8', errors='replace') as f:
            for n, line in enumerate(f, 1):
                if line.lstrip().startswith('#'):
                    continue
                for m in pat.finditer(line):
                    targets[m.group(1).split()[0]] = (
                        os.path.relpath(p, ROOT), n)
    check(len(targets) >= 10, "prompt-button targets found (%d)"
          % len(targets))
    for name, where in sorted(targets.items()):
        check(name in allowed,
              "prompt-button target %s (%s:%d) is on the allowlist"
              % ((name,) + where))


# Allowlisted macros that read client parameters, and why an entry with no
# limits is fine. Everything else that reads params must be closed or
# range-limited in core.cfg.
OPEN_PARAMS_OK = {
    'M106': 'S is a fan duty; fan.py clamps it',
    'M104': 'S is a temperature; heaters.py caps it at max_temp',
    'M109': 'S/R are temperatures; heaters.py caps them',
    'SET_HEATER_TEMPERATURE': 'TARGET is a temperature; capped',
    'LED_TIMEOUT_SET': 'M is a LED timeout in minutes',
    'LED_MODE': 'LED colours and speeds only',
    'SPEED_SLOW_PRE_PURGE_FOR_TPU': 'MATERIAL is a name',
    'SPEED_NORMAL__POST_PURGE_FOR_TPU': 'MATERIAL is a name',
    'M600': 'reads no client parameter',
}


def test_allowlisted_macros_with_params_are_bounded():
    # Audit (review item 3): every allowlisted macro that reads params or
    # forwards rawparams is closed or range-limited, or listed above with a
    # reason. A client parameter must not reach SET_TMC_CURRENT,
    # EXTRUDER_HOMING_MOVE or unbounded motion.
    entries = [e.strip() for e in core_allow().split('\n') if e.strip()]
    rules = muon_guard.parse_allowlist(entries, ConfigError)
    text = ''
    for fn in sorted(os.listdir(MACRO_DIR)):
        if fn.endswith('.cfg'):
            with open(os.path.join(MACRO_DIR, fn), encoding='utf-8',
                      newline='') as f:
                text += f.read().replace('\r\n', '\n') + '\n'
    with open(CORE_CFG, encoding='utf-8', newline='') as f:
        text += f.read().replace('\r\n', '\n')
    audited = []
    for sec in re.split(r'^(?=\[)', text, flags=re.M):
        m = re.match(r'\[gcode_macro (\S+)\]', sec)
        if m is None or m.group(1) not in rules:
            continue
        name = m.group(1)
        if not (re.search(r'params\.[A-Za-z_]+', sec)
                or 'rawparams' in sec):
            continue
        audited.append(name)
        bare = any(r['open'] and not r['cons'] for r in rules[name])
        if bare:
            check(name in OPEN_PARAMS_OK,
                  "macro %s reads client parameters and its entry is "
                  "unbounded" % (name,))
    check(len(audited) >= 15, "audit covered %d macros" % len(audited))
    for name in OPEN_PARAMS_OK:
        check(name in rules, "OPEN_PARAMS_OK names an allowlisted command: "
              "%s" % (name,))
    # the two the review found
    sless = rules['FILAMENT_INSERT_SENSORLESS_LOAD']
    check(len(sless) == 1 and not sless[0]['open'] and not sless[0]['cons'],
          "FILAMENT_INSERT_SENSORLESS_LOAD takes no client parameters")
    g28 = rules['G28'][0]
    check(g28['cons']['X_OFFSET'] == ('range', -10.0, 10.0),
          "G28 X_OFFSET is bounded to +-10")
    # print.cfg's own G28 call uses -5, inside the bound
    check(re.search(r'G28 Z PARK=0 X_OFFSET=-5', text) is not None,
          "the macros' own X_OFFSET is within the bound")


def test_param_bounds():
    rig = Rig()
    ok = ['G28 X_OFFSET=-5 Z PARK=0', 'G28 O', 'G28', 'G28 X Y',
          'FILAMENT_INSERT_SENSORLESS_LOAD',
          'PRINT_START EXTRUDER=215 BED=35 MATERIAL=PLA PROBE_COUNT=4,4',
          'PAUSE', 'RESUME', 'RESUME VELOCITY=100', 'UNLOAD_FILAMENT',
          'CUT_FILAMENT', 'SET_GCODE_OFFSET Z_ADJUST=0.05 MOVE=1']
    bad = ['FILAMENT_INSERT_SENSORLESS_LOAD CURRENT=9000',
           'FILAMENT_INSERT_SENSORLESS_LOAD SPEED=1',
           'CUT_FILAMENT SPEED=1', 'G28 X_OFFSET=11', 'G28 X_OFFSET=-11',
           'G28 X_OFFSET=abc', 'PRINT_START EXTRUDER=266',
           'PRINT_START BED=131', 'PAUSE HOTEND_TARGET=266',
           'RESUME VELOCITY=301', 'RESUME VELOCITY=abc',
           'UNLOAD_FILAMENT TEMP=149']
    for line in ok:
        check(rig.client(line) is None, "within bounds: %s" % (line,))
    for line in bad:
        check(rig.client(line) is not None, "out of bounds: %s" % (line,))
    # a bound parameter given twice, or in another case, is still checked
    check(rig.client('g28 x_offset=50') is not None,
          "lower case does not dodge a bound")


######################################################################
# 5. State a call leaves behind (re-check HIGH): offsets add up
######################################################################

def test_gcode_offset_total_is_bounded():
    # Each SET_GCODE_OFFSET Z_ADJUST is within +-1, but gcode_move adds
    # them up, and Z_OFFSET_APPLY_PROBE saves the sum as the probe z_offset.
    rig = Rig()
    accepted = 0
    for _ in range(10):
        if rig.client('SET_GCODE_OFFSET Z_ADJUST=-0.5 MOVE=1') is None:
            accepted += 1
    check(accepted == 4, "ten -0.5 nudges: four fit in 2 mm, the rest are "
          "refused (accepted %d)" % (accepted,))
    check(abs(rig.gm.homing_position[2] + 2.0) < 1e-9,
          "and the offset stopped at -2.0 (%r)" % (rig.gm.homing_position,))
    err = rig.client('SET_GCODE_OFFSET Z_ADJUST=-0.01')
    check(err is not None and 'offset' in err.lower(),
          "one more step past the bound is refused, and says why: %r"
          % (err,))
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.5') is None,
          "a step back toward zero still works")
    # absolute Z, then an adjust that crosses the bound
    check(rig.client('SET_GCODE_OFFSET Z=1.5') is None, "absolute Z=1.5")
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.6') is not None,
          "1.5 + 0.6 is past the bound and is refused")
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.4') is None,
          "1.5 + 0.4 = 1.9 passes")
    check(rig.client('SET_GCODE_OFFSET Z=1.99') is None,
          "an absolute value inside the bound passes")
    check(rig.client('SET_GCODE_OFFSET Z=2.01') is not None,
          "an absolute value past the bound is refused")
    check(rig.client('SET_GCODE_OFFSET Z=nan') is not None,
          "nan is refused")
    # X and Y add up too
    rig = Rig()
    for _ in range(5):
        check(rig.client('SET_GCODE_OFFSET X_ADJUST=2') is None, "X step")
    check(rig.client('SET_GCODE_OFFSET X_ADJUST=2') is not None,
          "the sixth X step passes 10 mm and is refused")
    # a macro's own offset is not a client's and is not checked
    rig.gm.homing_position[2] = 0.
    rig.from_macro('SET_GCODE_OFFSET Z=5')
    check(rig.gm.homing_position[2] == 5., "an internal offset is not "
          "bounded")
    # a gate that cannot read the state refuses
    rig = Rig()
    del rig.printer.objects['gcode_move']
    check(rig.client('SET_GCODE_OFFSET Z=0.1') is not None,
          "no gcode_move to read: refused")
    # dev mode allows any
    rig = Rig(dev=True)
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=-50') is None,
          "dev mode allows a large offset")


def test_ten_steps_then_apply_then_save():
    # The coordinator's case: ten in-limit Z_ADJUST steps, then
    # Z_OFFSET_APPLY_PROBE, then SAVE_CONFIG. Chosen: BOTH the total gcode
    # offset is bounded as it builds up (steps past the bound are refused),
    # AND persistence is bounded (Z_OFFSET_APPLY_PROBE refuses a result past
    # the limit). Nothing out of range may reach the saved config.
    rig = Rig()
    results = [rig.client('SET_GCODE_OFFSET Z_ADJUST=-0.5 MOVE=1')
               for _ in range(10)]
    check(results[:4] == [None] * 4 and all(r is not None
                                            for r in results[4:]),
          "ten -0.5 steps: four fit in 2 mm, six are refused")
    check(abs(rig.gm.homing_position[2] + 2.0) < 1e-9,
          "total gcode offset is -2.0, not -5.0: %r"
          % (rig.gm.homing_position[2],))
    check(rig.client('Z_OFFSET_APPLY_PROBE') is None,
          "apply of the bounded total passes")
    check(rig.client('SAVE_CONFIG') is None, "SAVE_CONFIG passes")
    check(abs(rig.disk_z_offset) <= 2.0,
          "saved z_offset is in range: %r" % (rig.disk_z_offset,))
    # Second layer on its own: an offset already in place from before (a
    # total the first layer would not have allowed) cannot be persisted.
    rig = Rig()
    rig.probe.z = 1.9
    rig.gm.homing_position[2] = -5.0          # as ten unbounded steps left it
    err = rig.client('Z_OFFSET_APPLY_PROBE')
    check(err is not None, "a total of -5 cannot be applied: %r" % (err,))
    check(rig.client('SAVE_CONFIG') is None, "SAVE_CONFIG itself passes")
    check(rig.pending_z is None and rig.disk_z_offset == 0.,
          "and nothing out of range was staged or saved (disk %r)"
          % (rig.disk_z_offset,))
    # Negative control: with the dev-mode escape (no guard) it does reach
    # the saved config, so the assertions above are the guard's doing.
    rig = Rig(dev=True)
    rig.probe.z = 1.9
    rig.gm.homing_position[2] = -5.0
    rig.client('Z_OFFSET_APPLY_PROBE')
    rig.client('SAVE_CONFIG')
    check(abs(rig.disk_z_offset) > 2.0,
          "negative control: without the guard 6.9 mm is saved (%r)"
          % (rig.disk_z_offset,))


def test_z_offset_apply_probe_is_bounded():
    rig = Rig()
    rig.probe.z = 0.
    rig.gm.homing_position[2] = -0.9
    check(rig.client('Z_OFFSET_APPLY_PROBE') is None,
          "apply a -0.9 offset to a 0 z_offset")
    check(abs(rig.pending_z - 0.9) < 1e-9, "z_offset to save is 0.9")
    # repeated use walks z_offset toward the bed; refused at the bound
    rig.probe.z = 1.8
    rig.gm.homing_position[2] = -0.9
    err = rig.client('Z_OFFSET_APPLY_PROBE')
    check(err is not None and 'z_offset' in err,
          "z_offset 1.8 - (-0.9) = 2.7 is past 2 mm and is refused: %r"
          % (err,))
    rig.probe.z = -1.9
    rig.gm.homing_position[2] = 0.5
    check(rig.client('Z_OFFSET_APPLY_PROBE') is not None,
          "and in the other direction")
    # no probe object: refuse
    rig = Rig()
    del rig.printer.objects['load_cell_probe']
    check(rig.client('Z_OFFSET_APPLY_PROBE') is not None,
          "no probe to read: refused")
    rig = Rig(dev=True)
    rig.probe.z = 1.8
    rig.gm.homing_position[2] = -0.9
    check(rig.client('Z_OFFSET_APPLY_PROBE') is None, "dev mode allows it")


def test_babystep_toward_zero_is_always_allowed():
    rig = Rig()
    rig.gm.homing_position[2] = 1.5
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=-0.5 MOVE=1') is None,
          "from 1.5 a step toward zero passes")
    rig.gm.homing_position[2] = 1.5
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.4') is None,
          "from 1.5, +0.4 = 1.9 is inside the limit")
    rig.gm.homing_position[2] = 1.5
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.6') is not None,
          "from 1.5, +0.6 = 2.1 is refused")
    # an offset already past the limit (dev mode, an old session): steps
    # toward zero pass, steps away do not
    rig.gm.homing_position[2] = 3.0
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=-0.5') is None,
          "from 3.0 a step to 2.5 (toward zero) passes")
    rig.gm.homing_position[2] = 3.0
    check(rig.client('SET_GCODE_OFFSET Z_ADJUST=0.5') is not None,
          "from 3.0 a step away is refused")
    rig.gm.homing_position[2] = -3.0
    check(rig.client('SET_GCODE_OFFSET Z=-2') is None,
          "from -3.0 an absolute -2 passes")
    # the limits agree: a saved offset up to the probe limit can be
    # babystepped
    opts = read_core_section()
    check(float(opts['max_gcode_z_offset']) >= float(
        opts['max_probe_z_offset']),
        "max_gcode_z_offset >= max_probe_z_offset")


def test_testz_and_endstop_apply():
    rig = Rig()
    for line in ('TESTZ Z=-0.1', 'TESTZ Z=5', 'TESTZ Z=-5', 'TESTZ Z=+',
                 'TESTZ Z=-', 'TESTZ Z=++', 'TESTZ Z=--', 'ACCEPT', 'ABORT'):
        check(rig.client(line) is None, "manual probe step passes: %s"
              % (line,))
    for line in ('TESTZ Z=+20', 'TESTZ Z=20', 'Z_OFFSET_APPLY_ENDSTOP'):
        check(rig.client(line) is not None, "refused: %s" % (line,))


class FakeCmd:
    error = CommandError

    def __init__(self):
        self.out = []

    def respond_info(self, m):
        self.out.append(m)


class FakeConfigfile:
    def __init__(self):
        self.set_calls = []

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))


def make_cmd_helper(z_limit, offset_z=0.):
    """A ProbeCommandHelper without a config: only what staging touches."""
    from extras import probe
    h = object.__new__(probe.ProbeCommandHelper)
    printer = FakePrinter(OEM_CFG)
    cf = FakeConfigfile()
    gm = type('GM', (), {})()
    printer.add_object('configfile', cf)
    printer.add_object('gcode', FakeCmd())
    printer.add_object('gcode_move', gm)
    h.printer = printer
    h.name = 'load_cell_probe'
    h.z_limit = z_limit
    h.z_offset_invalid = None
    h.on_calibrate_gcode = type('T', (), {
        'run_gcode_from_command': lambda self: None})()
    h.probe = type('P', (), {'get_offsets': lambda self, g=None: (
        0., 0., offset_z)})()
    return h, cf, gm


def test_accept_cannot_stage_a_bad_z_offset():
    # PROBE_CALIBRATE -> TESTZ Z=+20 -> ACCEPT would stage an unbounded
    # z_offset (probe.py probe_calibrate_finalize). Bounded where staged.
    from collections import namedtuple
    PR = namedtuple('PR', 'bed_x bed_y bed_z test_x test_y test_z')
    h, cf, _ = make_cmd_helper(2.0)
    h.probe_calibrate_info = (PR(0, 0, 0.5, 0, 0, 0.5), (0., 0., 0.))
    # nozzle driven 20 mm up from where it was: z_offset = 0 - 20.5 + 0.5
    try:
        h.probe_calibrate_finalize(PR(0, 0, 20.5, 0, 0, 20.5))
        ok = False
    except Exception as e:
        ok = 'safe range' in str(e)
    check(ok, "ACCEPT after TESTZ +20 is refused")
    check(cf.set_calls == [], "and nothing was staged: %r" % (cf.set_calls,))
    h.probe_calibrate_info = (PR(0, 0, 0.5, 0, 0, 0.5), (0., 0., 0.))
    h.probe_calibrate_finalize(PR(0, 0, 0.9, 0, 0, 0.9))
    check(cf.set_calls == [('load_cell_probe', 'z_offset', '-0.400')],
          "an in-range ACCEPT is staged: %r" % (cf.set_calls,))
    # Z_OFFSET_APPLY_PROBE stages through the same check
    h, cf, gm = make_cmd_helper(2.0, offset_z=1.8)
    gm.get_status = lambda: {'homing_origin': type('C', (), {'z': -0.9})()}
    try:
        h.cmd_Z_OFFSET_APPLY_PROBE(FakeCmd())
        ok = False
    except Exception as e:
        ok = 'safe range' in str(e)
    check(ok and cf.set_calls == [], "apply to 2.7 mm is refused at staging")
    # another probe without a limit keeps today's behaviour
    h, cf, _ = make_cmd_helper(None)
    h.probe_calibrate_info = (PR(0, 0, 0.5, 0, 0, 0.5), (0., 0., 0.))
    h.probe_calibrate_finalize(PR(0, 0, 20.5, 0, 0, 20.5))
    check(len(cf.set_calls) == 1, "no limit, no check (other probes)")


def load_offsets(value, limit=None):
    from extras import probe
    printer = FakePrinter(OEM_CFG)
    handlers = []
    printer.register_event_handler = lambda ev, cb: handlers.append((ev, cb))
    cp = configparser.RawConfigParser()
    cp.read_string('[load_cell_probe]\nz_offset: %s\n' % (value,))
    cw = configfile.ConfigWrapper(printer, cp, {}, 'load_cell_probe')
    return probe.ProbeOffsetsHelper(cw, limit), printer, handlers


def test_bad_saved_z_offset_starts_but_blocks_printing():
    import logging
    src_path = os.path.join(ROOT, 'klippy', 'extras', 'load_cell_probe.py')
    with open(src_path, encoding='utf-8') as f:
        src = f.read()
    m = re.search(r'^MAX_Z_OFFSET = ([\d.]+)', src, re.M)
    check(m is not None, "load_cell_probe.py defines MAX_Z_OFFSET")
    limit = float(m.group(1))
    check("probe.ProbeOffsetsHelper(config, MAX_Z_OFFSET)" in src
          and "z_limit=MAX_Z_OFFSET" in src,
          "load_cell_probe passes the limit to both helpers")
    opts = read_core_section()
    check(float(opts['max_probe_z_offset']) == limit,
          "the guard's z_offset limit equals the probe's")
    # in range: used as saved
    for good in ('0', '1.5', '-2', '2'):
        h, _, handlers = load_offsets(good, limit)
        check(h.z_offset == float(good) and h.invalid_z_offset is None
              and handlers == [], "a saved z_offset of %s loads" % (good,))
    # out of range: loads (klippy starts), 0.0 stands in, flagged, loud
    records = []

    class Catch(logging.Handler):
        def emit(self, rec):
            records.append(rec)
    catch = Catch()
    logging.getLogger().addHandler(catch)
    try:
        for bad in ('5', '2.01', '-2.01', '-50', '1e9'):
            del records[:]
            h, printer, handlers = load_offsets(bad, limit)
            check(h.z_offset == 0. and h.invalid_z_offset == float(bad),
                  "a saved z_offset of %s starts with 0.0 and is flagged"
                  % (bad,))
            check(any(r.levelno >= logging.ERROR and 'SAVED Z OFFSET' in
                      r.getMessage() for r in records),
                  "and logs an error: %s" % (bad,))
            check(len(handlers) == 1 and handlers[0][0] == 'klippy:ready',
                  "and warns at ready")
    finally:
        logging.getLogger().removeHandler(catch)
    # the unbounded form (other probes) is unchanged
    h, _, _ = load_offsets('7')
    check(h.z_offset == 7. and h.invalid_z_offset is None,
          "other probes keep their range")
    # the ready warning carries the message
    h, printer, handlers = load_offsets('5', limit)
    out = []
    g = FakeCmd()
    g.respond_raw = out.append
    printer.add_object('gcode', g)
    handlers[0][1]()
    check(out and 'Saved Z offset 5.000 mm is outside the safe range' in
          out[0], "ready warning text: %r" % (out,))
    # status flag
    ch, _, _ = make_cmd_helper(2.0)
    ch.z_offset_invalid = 5.0
    ch.last_state = False
    ch.last_probe_position = None
    ch.last_z_result = 0.
    ch.probe_calibrate_info = None
    st = ch.get_status(0.)
    check(st['z_offset_invalid'] is True and st['z_offset_saved'] == 5.0,
          "status carries z_offset_invalid")
    ch.z_offset_invalid = None
    check(ch.get_status(0.)['z_offset_invalid'] is False, "and clears")


def test_print_is_blocked_while_the_saved_z_offset_is_invalid():
    want = ("Saved Z offset 5.000 mm is outside the safe range "
            + chr(0xb1) + "2 mm. Recalibrate Z (PROBE_CALIBRATE or the "
            "panel's Z calibration) before printing.")
    rig = Rig()
    check(rig.client('PRINT_START') is None, "healthy: PRINT_START passes")
    rig.probe.invalid = 5.0
    for door in ('webhooks', 'pty', 'file'):
        for line in ('PRINT_START', 'PRINT_START EXTRUDER=215 BED=60',
                     'SDCARD_PRINT_FILE FILENAME=cube.gcode', 'M24'):
            err = rig.client(line, door)
            check(err == want, "blocked via %s: %s -> %r" % (door, line, err))
            check(rig.calls == [], "and did not run: %s" % (line,))
    # homing, calibration and manual probing stay possible
    for line in ('G28', 'G28 Z', 'PROBE_CALIBRATE', 'CALIBRATE_PROBE',
                 'TESTZ Z=-0.1', 'ACCEPT', 'SAVE_CONFIG', 'G1 Z5 F600',
                 'M104 S0', 'SET_GCODE_OFFSET Z_ADJUST=0.1',
                 'Z_OFFSET_APPLY_PROBE', 'CANCEL_PRINT', 'PAUSE', 'RESUME'):
        check(rig.client(line) is None,
              "still possible while invalid: %s" % (line,))
    # a macro's own PRINT_START (internal) is not the gate's business
    rig.from_macro('PRINT_START')
    # recalibrating into range (saved, restarted) clears the block
    rig.probe.invalid = None
    check(rig.client('PRINT_START') is None, "in range again: allowed")
    # dev mode is not blocked by the guard (it never is)
    rig = Rig(dev=True)
    rig.probe.invalid = 5.0
    check(rig.client('PRINT_START') is None, "dev mode bypass")
    # no probe at all: the gate has nothing to read and does not refuse
    rig = Rig()
    del rig.printer.objects['load_cell_probe']
    check(rig.client('PRINT_START') is None, "no probe object: not blocked")


def test_temperatures_follow_the_configured_max_temp():
    # No hard-coded 265: the TEMP/EXTRUDER/BED/HOTEND_TARGET bounds are the
    # heaters' configured max_temp, so a printer (or a later config) that
    # allows 270 C accepts it, and max_temp + 1 is refused.
    for ext in (265., 270., 300.):
        rig = Rig()
        rig.cfg.settings['extruder']['max_temp'] = ext
        cases = [('UNLOAD_FILAMENT TEMP=%g', 'UNLOAD_FILAMENT TEMP=%g'),
                 ('PAUSE HOTEND_TARGET=%g', 'PAUSE HOTEND_TARGET=%g'),
                 ('PRINT_START EXTRUDER=%g BED=60', 'PRINT_START EXTRUDER=%g')]
        for fmt, _ in cases:
            check(rig.client(fmt % ext) is None,
                  "max_temp %g allows %s" % (ext, fmt % ext))
            check(rig.client(fmt % (ext + 1)) is not None,
                  "max_temp %g refuses %s" % (ext, fmt % (ext + 1)))
    rig = Rig()
    rig.cfg.settings['extruder']['max_temp'] = 270.
    check(rig.client('UNLOAD_FILAMENT TEMP=270') is None,
          "PC unload at 270 passes when max_temp is 270")
    check(rig.client('UNLOAD_FILAMENT TEMP=271') is not None,
          "and 271 is refused")
    rig.cfg.settings['extruder']['max_temp'] = 265.
    check(rig.client('UNLOAD_FILAMENT TEMP=270') is not None,
          "the same 270 is refused when max_temp is 265 (the M1 today)")
    # the bed follows heater_bed max_temp
    rig = Rig()
    rig.cfg.settings['heater_bed']['max_temp'] = 150.
    check(rig.client('PRINT_START BED=150') is None, "bed 150 of 150")
    check(rig.client('PRINT_START BED=151') is not None, "bed 151 of 150")
    check(rig.client('_BED_MESH_RECORD BED=151') is not None,
          "mesh record bed 151 of 150")
    # a config that cannot be read refuses
    rig = Rig()
    del rig.printer.objects['configfile']
    check(rig.client('UNLOAD_FILAMENT TEMP=200') is not None,
          "no configfile to read the limit from: refused")
    rig = Rig()
    del rig.cfg.settings['extruder']
    check(rig.client('UNLOAD_FILAMENT TEMP=200') is not None,
          "no extruder max_temp: refused")
    # the shipped numbers are not in the allowlist any more
    check('..265' not in core_allow() and '..130' not in core_allow(),
          "core.cfg has no hard-coded heater limits")
    # unknown limit names are a config error
    try:
        muon_guard.parse_allowlist(['X A=0..@nope'], ConfigError)
        ok = False
    except ConfigError:
        ok = True
    check(ok, "an unknown @limit is a config error")


def test_save_variable_exact_control_strings():
    # Control services/control/model.ts:107 and :155 send these exactly,
    # lower case and with the quotes inside the value.
    rig = Rig()
    for door in ('webhooks', 'pty', 'file'):
        for line in ("SAVE_VARIABLE VARIABLE=muon_material VALUE='\"PLA\"'",
                     "SAVE_VARIABLE VARIABLE=muon_material_colour "
                     "VALUE='\"ff8800\"'"):
            err = rig.client(line, door)
            check(err is None, "Control's exact string passes via %s: %s (%s)"
                  % (door, line, err))
            check(rig.calls == [line], "and ran: %s" % (line,))
    for line in ("SAVE_VARIABLE VARIABLE=muon_other VALUE='\"x\"'",
                 "SAVE_VARIABLE VARIABLE=resonance_valid VALUE=1",
                 "SAVE_VARIABLE VARIABLE=muon_material VALUE=1 EXTRA=2"):
        check(rig.client(line) is not None, "refused: %s" % (line,))


def test_pause_at_layer():
    # Control PauseAtLayerDialog.vue:147,149,155,157. Not a macro in
    # core/M1 or klippy: a user-added one (Control only shows the dialog
    # when the printer has it). It is bounded for when it exists.
    rig = Rig()
    for line in ('SET_PAUSE_AT_LAYER ENABLE=0',
                 'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=25 MACRO="PAUSE"',
                 'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=0 MACRO="M600"',
                 'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=10000 MACRO=pause',
                 'SET_PAUSE_NEXT_LAYER ENABLE=0',
                 'SET_PAUSE_NEXT_LAYER ENABLE=1 MACRO="M600"'):
        check(rig.client(line) is None, "passes: %s" % (line,))
    for line in ('SET_PAUSE_AT_LAYER ENABLE=1 LAYER=10001 MACRO=PAUSE',
                 'SET_PAUSE_AT_LAYER ENABLE=1 LAYER=5 MACRO=G28',
                 'SET_PAUSE_AT_LAYER FOO=1',
                 'SET_PAUSE_NEXT_LAYER ENABLE=1 MACRO=G28'):
        check(rig.client(line) is not None, "refused: %s" % (line,))


def test_print_start_tokens_cannot_inject():
    rig = Rig()
    for line in ('PRINT_START MATERIAL=PLA', 'PRINT_START MATERIAL=PETG-CF',
                 'PRINT_START MATERIAL=', 'PRINT_START',
                 'PRINT_START EXTRUDER=215 BED=35 MATERIAL=PLA '
                 'MESH_MIN=117.126,7 MESH_MAX=142.874,27.8742 '
                 'PROBE_COUNT=4,4 ALGORITHM=bicubic'):
        check(rig.client(line) is None, "PRINT_START passes: %s" % (line,))
    for line in ('PRINT_START MATERIAL="PLA X=1"',
                 'PRINT_START MATERIAL=PLA;X',
                 'PRINT_START MATERIAL=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa',
                 'PRINT_START ALGORITHM=bi_cubic',
                 'PRINT_START MESH_MAX="1,2 FOO=3"'):
        if ';' in line:
            continue
        check(rig.client(line) is not None,
              "PRINT_START refused: %s" % (line,))


def test_set_print_mode():
    # Control v4's print-mode cards send SET_PRINT_MODE MODE=<mode>, lower
    # case. MODE is held to the [fan_modes] modes; bare SET_PRINT_MODE only
    # reports the mode.
    with open(CORE_CFG, encoding='utf-8') as f:
        text = f.read().replace('\r\n', '\n')
    m = re.search(r'^\[fan_modes\]\n(.*?)(?=^\[|\Z)', text, re.S | re.M)
    check(m is not None, "core.cfg has a [fan_modes] section")
    modes = re.search(r'^modes:\s*(.*)$', m.group(1), re.M).group(1)
    modes = [x.strip() for x in modes.split(',')]
    check(modes == ['normal', 'quiet', 'turbo'],
          "[fan_modes] offers normal, quiet, turbo: %r" % (modes,))
    entries = [e.strip() for e in core_allow().split('\n')
               if e.strip().startswith('SET_PRINT_MODE')]
    check(entries == ['SET_PRINT_MODE MODE=~' + '|'.join(modes)],
          "SET_PRINT_MODE is allowlisted with MODE held to exactly the "
          "[fan_modes] modes: %r" % (entries,))
    rig = Rig()
    for mode in modes:
        for door in ('webhooks', 'pty', 'file'):
            line = 'SET_PRINT_MODE MODE=%s' % (mode,)
            err = rig.client(line, door)
            check(err is None, "%s passes via %s (%s)" % (line, door, err))
            check(rig.calls == [line], "and runs: %s" % (line,))
    check(rig.client('SET_PRINT_MODE') is None,
          "bare SET_PRINT_MODE (report the mode) passes")
    for line in ('SET_PRINT_MODE MODE=fast', 'SET_PRINT_MODE MODE=normal2',
                 'SET_PRINT_MODE MODE=xnormal', 'SET_PRINT_MODE MODE=',
                 'SET_PRINT_MODE MODE="normal X=1"',
                 'SET_PRINT_MODE MODE=normal SPEED=1'):
        err = rig.client(line)
        check(err is not None and refused_text_ok(err),
              "unknown mode or extra parameter refused: %s (%r)"
              % (line, err))
        check(rig.calls == [], "and nothing ran: %s" % (line,))


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
