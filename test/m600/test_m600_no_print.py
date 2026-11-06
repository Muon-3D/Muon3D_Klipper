"""KAN-1699: M600 with no print only prompts; PAUSE never leaves G91 on.

Control's "Change filament" sends a bare M600.  It used to PAUSE at once,
before the user confirmed anything, even with no print running: Z lift,
fast park, and with a stuck contact_guard latch (KAN-742) a refused park
that left G91 on, one more 2 mm lift per click.

This renders the real M600 / PAUSE / _CANCEL_M600 macros from core/M1 with
Klipper's single-brace Jinja and runs them through a small G-code model:
G90/G91, SAVE/RESTORE_GCODE_STATE, pause_resume PAUSE_BASE, and a
contact_guard that refuses an XY move with no Z rise while its latch holds.
Nothing here moves a motor.

    python3 test/m600/test_m600_no_print.py
"""
import configparser
import os
import sys

import jinja2

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
MACROS = os.path.join(ROOT, 'core', 'M1', 'macros')


class Failure(Exception):
    pass


def load_macros():
    cp = configparser.RawConfigParser(strict=False)
    for name in ('print.cfg', 'filament.cfg'):
        with open(os.path.join(MACROS, name), encoding='utf-8') as f:
            text = f.read()
        # Klipper cuts each line at its first '#' before parsing.
        lines = []
        for ln in text.splitlines():
            if ln.lstrip().startswith('#'):
                ln = ''
            lines.append(ln)
        cp.read_string('\n'.join(lines))
    return cp


MACROS_CP = load_macros()
ENV = jinja2.Environment('{%', '%}', '{', '}')


class Rig:
    def __init__(self, state='standby', paused=False, homed='xyz', hot=False,
                 latch=False, z=20.0):
        self.state = state
        self.paused = paused
        self.homed = homed
        self.hot = hot
        self.latch = latch
        self.x, self.y, self.z = 100.0, 100.0, z
        self.absolute = True
        self.saved = {}
        self.log = []          # every move that reached a motor
        self.cmdlog = []       # every command line run
        self.prompts = []

    def status(self):
        ex = {'can_extrude': 'True' if self.hot else 'False',
              'target': 250.0 if self.hot else 0.0}
        return {
            'pause_resume': {'is_paused': self.paused},
            'print_stats': {'state': self.state},
            'extruder': ex,
            'toolhead': {'homed_axes': self.homed,
                         'axis_maximum': {'x': 250., 'y': 250., 'z': 250.},
                         'position': {'x': self.x, 'y': self.y, 'z': self.z}},
            'configfile': {'settings': {'extruder': {'max_temp': 300}}},
            'gcode_macro PAUSE': self.vars('PAUSE'),
        }

    def vars(self, name):
        v = {}
        for k, val in MACROS_CP['gcode_macro ' + name].items():
            if k.startswith('variable_'):
                v[k[9:]] = float(val)
        return v

    def run_macro(self, name, rawparams=''):
        sec = MACROS_CP['gcode_macro ' + name]
        params = {}
        for tok in rawparams.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                params[k.upper()] = v

        def raise_error(msg):
            raise Failure(msg)
        out = ENV.from_string(sec['gcode']).render(
            printer=self.status(), params=params, rawparams=rawparams,
            action_raise_error=raise_error,
            action_respond_info=lambda m: "")
        for ln in out.splitlines():
            ln = ln.strip()
            if ln and not ln.startswith(';'):
                self.command(ln)

    def command(self, line):
        self.cmdlog.append(line)
        name, _, rest = line.partition(' ')
        name = name.upper()
        rest = rest.strip()
        if name == 'RESUME':
            self.paused = False
        elif name in ('M600', 'PAUSE', '_CANCEL_M600'):
            self.run_macro(name, rest)
        elif name == 'PAUSE_BASE':
            if not self.paused:
                self.paused = True
                self.saved['PAUSE_STATE'] = self.absolute
        elif name == 'G90':
            self.absolute = True
        elif name == 'G91':
            self.absolute = False
        elif name == 'SAVE_GCODE_STATE':
            self.saved[rest.split('=')[1]] = self.absolute
        elif name == 'RESTORE_GCODE_STATE':
            self.absolute = self.saved[rest.split('=')[1]]
        elif name == 'G1':
            self.g1(rest)
        elif name == 'RESPOND':
            if 'MSG="action:prompt_' in rest:
                msg = rest.split('MSG="action:', 1)[1].rstrip('"')
                self.prompts.append(msg)
        # LED_INFO and the rest: no motion, nothing to model.

    def g1(self, rest):
        w = dict((t[0], float(t[1:])) for t in rest.split() if t[0] != 'F')
        nx, ny, nz = self.x, self.y, self.z
        if 'X' in w:
            nx = w['X'] if self.absolute else self.x + w['X']
        if 'Y' in w:
            ny = w['Y'] if self.absolute else self.y + w['Y']
        if 'Z' in w:
            nz = w['Z'] if self.absolute else self.z + w['Z']
        if (nx, ny) != (self.x, self.y) and nz <= self.z and self.latch:
            raise Failure("Nozzle is touching the plate: lift Z before "
                          "moving X or Y")
        self.x, self.y, self.z = nx, ny, nz
        self.log.append('G1 ' + ' '.join(
            '%s%g' % kv for kv in sorted(w.items())))

    def prompt_shown(self):
        return 'prompt_show' in self.prompts


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("ok: " + msg)


def runs(rig, line):
    try:
        rig.command(line)
        return True
    except Failure:
        return False


def test_idle_m600_only_prompts():
    rig = Rig(state='standby', hot=True)
    check(runs(rig, 'M600'), "idle M600 succeeds")
    check(rig.log == [], "idle M600 moves nothing: %r" % rig.log)
    check('PAUSE' not in rig.cmdlog, "idle M600 does not PAUSE")
    check(not rig.paused, "idle M600 leaves the printer unpaused")
    check(rig.prompt_shown(), "idle M600 shows the prompt")
    check(any('Unload|_DO_UNLOAD_M600' in p for p in rig.prompts),
          "the Unload button is offered")


def test_idle_cancel_does_not_resume():
    rig = Rig(state='standby')
    rig.command('M600')
    ok = runs(rig, '_CANCEL_M600')
    check(ok and 'RESUME' not in rig.cmdlog,
          "cancelling an idle prompt does not RESUME")


def test_printing_m600_pauses_and_parks():
    rig = Rig(state='printing', hot=True)
    check(runs(rig, 'M600'), "printing M600 succeeds")
    check(rig.paused, "printing M600 pauses")
    check(rig.log == ['G1 E-1', 'G1 Z2', 'G1 X245 Y245'],
          "printing M600 retracts, lifts and parks: %r" % rig.log)
    check(rig.prompt_shown(), "printing M600 shows the prompt")
    check(rig.absolute, "absolute mode after a printing M600")


def test_m600_twice_lifts_once():
    rig = Rig(state='printing', hot=True)
    runs(rig, 'M600')
    runs(rig, 'M600')
    lifts = [m for m in rig.log if m.startswith('G1 Z')]
    check(len(lifts) == 1, "M600 twice lifts Z once: %r" % rig.log)
    check(rig.prompts.count('prompt_show') == 2, "the prompt is shown again")


def test_idle_m600_twice_does_nothing():
    rig = Rig(state='standby', hot=True)
    runs(rig, 'M600')
    runs(rig, 'M600')
    check(rig.log == [], "idle M600 twice moves nothing")
    check(rig.prompts.count('prompt_show') == 2, "the prompt is shown again")


def test_refused_park_restores_absolute():
    rig = Rig(state='printing', hot=True, latch=True)
    ok = runs(rig, 'M600')
    check(not ok, "the latch refuses the park")
    check(rig.absolute, "absolute mode restored after the refused park")
    z = rig.z
    runs(rig, 'M600')
    check(rig.z == z, "a second click lifts no further (z %g)" % rig.z)


def test_idle_pause_no_motion():
    rig = Rig(state='standby', hot=True)
    check(runs(rig, 'PAUSE'), "idle PAUSE succeeds")
    check(rig.log == [], "idle PAUSE moves nothing: %r" % rig.log)
    check(rig.absolute, "absolute mode after an idle PAUSE")


def test_printing_pause_unhomed_stays_absolute():
    rig = Rig(state='printing', homed='', hot=False)
    runs(rig, 'PAUSE')
    check(rig.log == [], "unhomed PAUSE moves nothing")
    check(rig.absolute, "absolute mode after an unhomed PAUSE")


if __name__ == '__main__':
    bad = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try:
                f()
                print("PASS " + n)
            except AssertionError as e:
                bad += 1
                print("FAIL %s: %s" % (n, e))
    print("%d failed" % bad if bad else "all passed")
    sys.exit(1 if bad else 0)
