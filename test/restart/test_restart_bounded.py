"""KAN-1362: FIRMWARE_RESTART after an emergency stop must come back.

On the bench klippy.log ended at "Attempting MCU 'rpi' config_reset command"
/ "Got EOF when reading from device" and klippy never restarted; only
`systemctl restart klipper` recovered it.  `_restart_via_command` goes
straight from that log line to `SerialReader.disconnect()`, which joined the
serial threads with no timeout, and nothing above it had one either.

Checks, without hardware or an MCU dictionary:

* `SerialReader.disconnect()` returns within a bound when the serialqueue
  or the Python reader thread does not stop, and keeps the stuck queue
  referenced (freeing it under a live thread would be a use-after-free).
* `Printer.run()` runs every klippy:disconnect handler even when a
  klippy:firmware_restart handler raises.
* a restart whose post-run handlers hang is re-exec'd by the restart
  watchdog, with every thread's stack in the log; one that completes is not.
* the audit: every klippy:shutdown / klippy:disconnect /
  klippy:firmware_restart handler and every thread a Muon extra registers is
  one this test knows about and drives within a bound.

    python3 test/restart/test_restart_bounded.py
"""
import ast
import logging
import os
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

import serialhdl  # noqa: E402
import klippy  # noqa: E402

BOUND = 3.


class LogCapture(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.lines = []

    def emit(self, record):
        self.lines.append(record.getMessage())


def capture():
    h = LogCapture()
    logging.getLogger().addHandler(h)
    return h


def bounded(fn, bound=BOUND):
    """Run fn in a daemon thread; True when it returned within bound."""
    t = threading.Thread(target=fn, daemon=True)
    t.start()
    t.join(bound)
    return not t.is_alive()


# SerialReader.disconnect ---------------------------------------------------

class StuckLib:
    """serialqueue_exit that never returns (the C I/O thread is stuck)."""
    def __init__(self):
        self.release = threading.Event()
        self.freed = []

    def serialqueue_exit(self, sq):
        self.release.wait(30.)


def make_reader(lib, thread=None):
    r = object.__new__(serialhdl.SerialReader)
    r.ffi_lib = lib
    r.mcu_name = 'rpi'
    r.warn_prefix = "mcu 'rpi': "
    r.serialqueue = object()
    r.background_thread = thread
    r.serial_dev = None
    r.pending_notifications = {}
    return r


def test_disconnect_returns_when_serialqueue_exit_hangs():
    serialhdl.DISCONNECT_TIMEOUT = 0.3
    lib = StuckLib()
    r = make_reader(lib)
    sq = r.serialqueue
    log = capture()
    try:
        assert bounded(r.disconnect), "disconnect() hung in serialqueue_exit"
        assert r.serialqueue is None and r.background_thread is None
        assert any(e[0] is sq for e in serialhdl._stuck_queues), \
            "a stuck serialqueue must stay referenced, not be freed"
        assert any('rpi' in ln and 'did not stop' in ln
                   for ln in log.lines), log.lines
    finally:
        lib.release.set()
        logging.getLogger().removeHandler(log)


def test_disconnect_returns_when_reader_thread_is_stuck():
    serialhdl.DISCONNECT_TIMEOUT = 0.3
    release = threading.Event()

    class Lib:
        def serialqueue_exit(self, sq):
            pass
    # the Python reader thread stuck in a response handler
    t = threading.Thread(target=lambda: release.wait(30.), daemon=True)
    t.start()
    r = make_reader(Lib(), t)
    try:
        assert bounded(r.disconnect), "disconnect() hung joining the reader"
        assert any(e[1] is t for e in serialhdl._stuck_queues)
    finally:
        release.set()


def test_disconnect_still_clean_when_threads_stop():
    serialhdl.DISCONNECT_TIMEOUT = 0.3
    before = len(serialhdl._stuck_queues)

    class Lib:
        def serialqueue_exit(self, sq):
            pass
    t = threading.Thread(target=lambda: None)
    t.start()
    r = make_reader(Lib(), t)
    assert bounded(r.disconnect)
    assert r.serialqueue is None and len(serialhdl._stuck_queues) == before


def test_real_serialqueue_disconnect_after_peer_eof():
    # The bench sequence: config_reset makes klipper_mcu close its pty,
    # serialqueue logs "Got EOF when reading from device", then disconnect().
    import pty
    import chelper
    ffi_main, ffi_lib = chelper.get_ffi()
    serialhdl.DISCONNECT_TIMEOUT = 5.
    for _ in range(5):
        m, sfd = pty.openpty()
        r = make_reader(ffi_lib)
        r.ffi_main = ffi_main
        r.lock = threading.Lock()
        r.handlers = {}
        r.serialqueue = ffi_main.gc(
            ffi_lib.serialqueue_alloc(sfd, b'u', 0, b'rpi'),
            ffi_lib.serialqueue_free)
        r.background_thread = threading.Thread(target=r._bg_thread,
                                               daemon=True)
        r.background_thread.start()
        time.sleep(0.02)
        os.close(m)
        time.sleep(0.02)
        assert bounded(r.disconnect), "disconnect() hung after peer EOF"
        os.close(sfd)


def test_reexec_from_the_watchdog_thread_replaces_the_process():
    # os.execv from the watchdog thread while the main thread is stuck: the
    # process must come back as a fresh klippy-like process.
    import subprocess
    import tempfile
    script = tempfile.NamedTemporaryFile('w', suffix='.py', delete=False)
    script.write('''import os, sys, threading
sys.path.insert(0, %r)
import klippy
if os.environ.get('KAN1362_REEXECD'):
    print('reexecd', sys.argv[1:])
    sys.exit(0)
os.environ['KAN1362_REEXECD'] = '1'
wd = klippy.RestartWatchdog(0.3, lambda: klippy.reexec_klippy(None))
wd.arm('klippy firmware_restart')
threading.Event().wait(30.)
''' % (os.path.join(ROOT, 'klippy'),))
    script.close()
    try:
        out = subprocess.run([sys.executable, script.name, 'printer.cfg'],
                             capture_output=True, text=True, timeout=20.)
    finally:
        os.unlink(script.name)
    assert out.returncode == 0, out.stderr[-2000:]
    assert "reexecd ['printer.cfg']" in out.stdout, (out.stdout, out.stderr)
    assert 'did not finish within' in out.stderr, out.stderr[-2000:]


# Printer.run post-run and the restart watchdog -----------------------------

class FakeReactor:
    def run(self):
        pass

    def monotonic(self):
        return time.monotonic()


def make_printer(result):
    p = object.__new__(klippy.Printer)
    p.reactor = FakeReactor()
    p.run_result = result
    p.event_handlers = {}
    p.restart_watchdog = None
    return p


def test_disconnect_runs_after_a_raising_firmware_restart_handler():
    p = make_printer('firmware_restart')
    ran = []

    def bad():
        raise RuntimeError("mcu 'rpi' fell over")
    p.register_event_handler('klippy:firmware_restart', bad)
    p.register_event_handler('klippy:firmware_restart',
                             lambda: ran.append('restart 2'))
    p.register_event_handler('klippy:disconnect',
                             lambda: ran.append('disconnect'))
    assert p.run() == 'firmware_restart'
    assert ran == ['restart 2', 'disconnect'], ran


def test_watchdog_reexecs_a_hung_restart():
    fired = threading.Event()
    wd = klippy.RestartWatchdog(0.3, fallback=fired.set)
    p = make_printer('firmware_restart')
    p.restart_watchdog = wd
    release = threading.Event()

    def hung_disconnect_handler():
        release.wait(10.)
    p.register_event_handler('klippy:disconnect', hung_disconnect_handler)
    log = capture()
    try:
        threading.Thread(target=p.run, daemon=True).start()
        assert fired.wait(BOUND), "watchdog did not fire on a hung restart"
        text = '\n'.join(log.lines)
        assert 'hung_disconnect_handler' in text, \
            "the log must show where the restart was stuck"
    finally:
        release.set()
        logging.getLogger().removeHandler(log)


def test_watchdog_quiet_when_restart_completes():
    fired = threading.Event()
    wd = klippy.RestartWatchdog(0.3, fallback=fired.set)
    p = make_printer('firmware_restart')
    p.restart_watchdog = wd
    p.register_event_handler('klippy:disconnect', lambda: None)
    assert p.run() == 'firmware_restart'
    wd.disarm()     # main() does this once the next Printer exists
    assert not fired.wait(0.6), "watchdog fired after a clean restart"


def test_watchdog_not_armed_on_exit():
    fired = threading.Event()
    wd = klippy.RestartWatchdog(0.2, fallback=fired.set)
    p = make_printer('exit')
    p.restart_watchdog = wd
    assert p.run() == 'exit'
    assert not fired.wait(0.5)


def test_reexec_argv_restarts_the_same_klippy():
    argv = klippy.reexec_argv(['/x/klippy/klippy.py', 'printer.cfg', '-l',
                               '/tmp/klippy.log'], '/venv/bin/python')
    assert argv == ['/venv/bin/python', '/x/klippy/klippy.py', 'printer.cfg',
                    '-l', '/tmp/klippy.log'], argv


# The audit -----------------------------------------------------------------

# Muon extras (and Muon-patched upstream ones) that hook the shutdown and
# restart path or start threads.  A new entry must be reviewed: its handler
# must not pause or block (klippy:shutdown runs under assert_no_pause, and a
# blocking klippy:disconnect / klippy:firmware_restart handler is exactly the
# KAN-1362 hang), and a thread must be a daemon or be joined with a timeout.
AUDITED = {
    ('led_effect.py', 'klippy:shutdown', '_handle_shutdown'),
    ('adxl345_probe.py', 'klippy:disconnect', '_close_trace'),
}
MUON_EXTRAS = [
    'adxl345_probe.py', 'bed_removal_detector.py', 'contact_guard.py',
    'homing_scope.py', 'hotend_fan_latch.py', 'load_cell_probe.py',
    'muon_guard.py', 'nozzle_wipe_smart.py', 'power_fan.py', 'led_effect.py',
    'load_cell.py', 'hx71x.py', 'bulk_sensor.py', 'tap_onset.py',
    'tap_trace.py',
]
EVENTS = ('klippy:shutdown', 'klippy:disconnect', 'klippy:firmware_restart')


def audit(name):
    path = os.path.join(ROOT, 'klippy', 'extras', name)
    if not os.path.exists(path):
        return set(), []
    tree = ast.parse(open(path, encoding='utf-8').read(), path)
    hooks, threads = set(), []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        fname = f.attr if isinstance(f, ast.Attribute) else getattr(
            f, 'id', '')
        if (fname == 'register_event_handler' and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value in EVENTS):
            cb = node.args[1]
            cbname = cb.attr if isinstance(cb, ast.Attribute) else getattr(
                cb, 'id', ast.dump(cb))
            hooks.add((name, node.args[0].value, cbname))
        if fname == 'Thread':
            daemon = any(k.arg == 'daemon' for k in node.keywords)
            threads.append((name, node.lineno, daemon))
        if fname in ('run', 'call', 'check_call', 'check_output') and \
                isinstance(f, ast.Attribute) and \
                getattr(f.value, 'id', '') == 'subprocess':
            assert any(k.arg == 'timeout' for k in node.keywords), \
                "%s:%d subprocess without a timeout" % (name, node.lineno)
    return hooks, threads


def test_audit_muon_extras_shutdown_hooks_and_threads():
    hooks, threads = set(), []
    for name in MUON_EXTRAS:
        h, t = audit(name)
        hooks |= h
        threads += t
    unknown = hooks - AUDITED
    assert not unknown, "unaudited shutdown/restart handlers: %r" % (
        sorted(unknown),)
    # bulk_sensor's batch thread is upstream; every other one must be daemon
    bad = [t for t in threads if not t[2]]
    assert not bad, "non-daemon threads in Muon extras: %r" % (bad,)


def test_led_effect_shutdown_does_not_pause():
    # klippy:shutdown runs under reactor.assert_no_pause(); led_effect's
    # handler only flags state and sends the chain, so it must return at once
    # on a chain whose transmit is a no-op.
    from extras import led_effect

    class Chain:
        led_helper = None
    h = object.__new__(led_effect.ledFrameHandler)
    h.effects = []
    h.ledChains = [Chain()]
    h.shutdown = False
    t0 = time.monotonic()
    h._handle_shutdown()
    assert h.shutdown and time.monotonic() - t0 < 0.1


def test_watchdog_does_not_exec_when_disarmed_during_stack_dump():
    fired = threading.Event()
    wd = klippy.RestartWatchdog(0.1, fallback=fired.set)
    dumping = threading.Event()
    resume = threading.Event()
    orig = klippy.thread_stacks

    def slow_stacks():
        dumping.set()
        resume.wait(3.)
        return ''
    klippy.thread_stacks = slow_stacks
    log = capture()
    try:
        wd.arm('klippy firmware_restart')
        assert dumping.wait(BOUND), "watchdog never started its stack dump"
        wd.disarm()         # the restart finished while the dump was running
        resume.set()
        assert not fired.wait(0.5), "watchdog exec'd after disarm()"
    finally:
        resume.set()
        klippy.thread_stacks = orig
        logging.getLogger().removeHandler(log)


class LoopWatchdog:
    def __init__(self, events):
        self.events = events

    def disarm(self):
        self.events.append('disarm')


class LoopReactor:
    def __init__(self, events):
        self.events = events

    def finalize(self):
        self.events.append('finalize')


class LoopPrinter:
    def __init__(self, events, res):
        self.events = events
        self.res = res
        self.restart_watchdog = None

    def run(self):
        self.events.append('run')
        return self.res


def run_loop(results, stuck):
    """Drive klippy.run_restart_loop with fake printers; returns events."""
    events = []
    results = list(results)

    def new_printer():
        events.append('new_printer')
        return LoopReactor(events), LoopPrinter(events, results.pop(0))
    saved = (time.sleep, klippy.reexec_klippy, list(serialhdl._stuck_queues))
    time.sleep = lambda s: None
    klippy.reexec_klippy = lambda bg: events.append('reexec')
    serialhdl._stuck_queues[:] = stuck
    try:
        klippy.run_restart_loop({}, None, 'v', LoopWatchdog(events),
                                new_printer)
    finally:
        time.sleep, klippy.reexec_klippy = saved[0], saved[1]
        serialhdl._stuck_queues[:] = saved[2]
    return events


def test_main_loop_disarms_watchdog_after_in_process_restart():
    ev = run_loop(['firmware_restart', 'exit'], stuck=[])
    # each new Printer is built, then the watchdog is disarmed, then it runs
    assert ev == ['new_printer', 'disarm', 'run', 'finalize',
                  'new_printer', 'disarm', 'run'], ev


def test_main_loop_reexecs_instead_of_restarting_with_stuck_queues():
    ev = run_loop(['firmware_restart', 'exit'], stuck=[object()])
    assert ev == ['new_printer', 'disarm', 'run', 'reexec'], ev
    assert ev.count('new_printer') == 1, "restarted in-process with stuck UARTs"


def test_main_loop_exit_does_not_reexec_even_with_stuck_queues():
    ev = run_loop(['exit'], stuck=[object()])
    assert 'reexec' not in ev, ev


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as e:  # noqa: BLE001
                fails += 1
                print('FAIL', name, repr(e))
    sys.exit(1 if fails else 0)
