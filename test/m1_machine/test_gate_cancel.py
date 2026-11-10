"""m1_machine's start gate, sent by a client rather than a print file.

Review of #107, blocker 1: PRINT_START is on the client allowlist, and a
PRINT_START sent through gcode/script is not a print file, so KAN-382's
abort (virtual_sdcard.request_cancel) has nothing to stop. The gate then
held the gcode mutex for gate_timeout (30 min) with CANCEL_PRINT and every
panel button queued behind it.

These tests drive klippy's real reactor, gcode dispatcher, gcode_macro,
pause_resume, webhooks' gcode/script handler and m1_machine, with the
cancel sent from its own greenlet the way Moonraker's request arrives
(test/print_cancel's harness). They check that both cancel doors stop the
gate within a poll, and that with no cancel the gate gives up after
manual_gate_timeout rather than gate_timeout.

Needs Linux and klippy's requirements, as test/print_cancel does:

    python3 test/m1_machine/test_gate_cancel.py
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'print_cancel'))

import test_cancel_during_print_start as pc  # noqa: E402  (stubs chelper)
from extras import m1_machine  # noqa: E402
import webhooks  # noqa: E402


class Config(pc.Config):
    def has_section(self, name):
        return False


class ColumnOut:
    """z_power with the Z column folded: the gate waits for it."""
    def get_status(self, eventtime):
        return {'z': 'absent', 'state': 'absent', 'reason': ''}


class Plate:
    def plate_present(self):
        return True


class GateHarness(pc.Harness):
    def __init__(self, m1_opts=None):
        pc.Harness.__init__(self, {'PRINT_START': {'gcode': '\n'.join([
            'MACHINE_WAIT_READY ACTION=print',
            'STEP NAME=after_gate'])}})
        p = self.printer
        p.objects['z_power'] = ColumnOut()
        p.objects['bed_removal_detector'] = Plate()
        opts = {'settle_time': 0., 'poll_time': 0.05}
        opts.update(m1_opts or {})
        self.m1 = m1_machine.M1Machine(Config(p, 'm1_machine', opts))
        p.objects['m1_machine'] = self.m1
        # The real gcode/script endpoint (webhooks.GCodeHelper)
        self.helper = webhooks.GCodeHelper(p)
        self.m1._connect()
        self.m1._ready()

    def send_event(self, event, *args):
        return self.printer.send_event(event, *args)

    def client_print_start(self, cancel_at=None, door='cancel',
                           timeout=10.):
        """Send PRINT_START as a client does; optionally cancel it at
        cancel_at seconds through door ('cancel' is Moonraker's
        print/cancel, 'script' a CANCEL_PRINT sent through gcode/script).
        Returns (seconds PRINT_START held the mutex, error text or None)."""
        result = {}

        class Req:
            def __init__(self, script):
                self.script = script

            def get_str(self, name):
                return self.script

        def start(eventtime):
            t0 = self.reactor.monotonic()
            try:
                self.helper._handle_script(Req('PRINT_START'))
                result['error'] = None
            except Exception as e:
                result['error'] = str(e)
            result['held'] = self.reactor.monotonic() - t0

        def cancel(eventtime):
            t0 = self.reactor.monotonic()
            try:
                if door == 'cancel':
                    self.webhooks.endpoints['pause_resume/cancel'](None)
                else:
                    self.helper._handle_script(Req('CANCEL_PRINT'))
            except Exception as e:
                result['cancel_error'] = str(e)
            result['cancel_done'] = self.reactor.monotonic() - t0

        def main(eventtime):
            try:
                now = self.reactor.monotonic()
                self.reactor.register_callback(start, now)
                if cancel_at is not None:
                    self.reactor.register_callback(cancel, now + cancel_at)
                deadline = now + timeout
                while self.reactor.monotonic() < deadline:
                    self.reactor.pause(self.reactor.monotonic() + 0.02)
                    if 'held' in result and (cancel_at is None
                                             or 'cancel_done' in result):
                        break
                else:
                    result['timed_out'] = True
            finally:
                self.reactor.end()

        self.reactor.register_callback(main)
        self.reactor.run()
        self.reactor.finalize()
        return result


class ClientPrintStartGate(unittest.TestCase):
    def test_moonraker_cancel_stops_a_client_gate(self):
        h = GateHarness()
        r = h.client_print_start(cancel_at=0.5, door='cancel')
        self.assertNotIn('timed_out', r)
        self.assertIn('cancelled', r['error'] or '')
        # Stopped at the next poll, not after gate_timeout or the 30 s
        # manual bound
        self.assertLess(r['held'], 1.5)
        self.assertNotIn('step:after_gate', h.log)
        self.assertIsNone(h.m1.gate)
        self.assertIsNone(h.m1.gate_abort)
        # CANCEL_PRINT then ran: with no print it only reports that
        self.assertTrue(any('nothing to clean up' in o for o in h.output),
                        h.output)

    def test_a_cancel_print_script_stops_a_client_gate(self):
        h = GateHarness()
        r = h.client_print_start(cancel_at=0.5, door='script')
        self.assertNotIn('timed_out', r)
        self.assertIn('cancelled', r['error'] or '')
        self.assertLess(r['held'], 1.5)
        self.assertNotIn('step:after_gate', h.log)

    def test_with_no_cancel_it_gives_up_after_the_manual_timeout(self):
        # gate_timeout stays 30 min for print files; a client's PRINT_START
        # gets manual_gate_timeout (here 1 s, 30 s by default).
        h = GateHarness({'manual_gate_timeout': 1.})
        r = h.client_print_start(timeout=10.)
        self.assertNotIn('timed_out', r)
        self.assertIn('Not ready to print', r['error'] or '')
        self.assertGreaterEqual(r['held'], 1.)
        self.assertLess(r['held'], 2.5)
        self.assertNotIn('step:after_gate', h.log)

    def test_a_cancel_with_no_gate_waiting_flags_nothing(self):
        h = GateHarness()
        self.assertFalse(h.m1.request_gate_abort())
        self.assertIsNone(h.m1.gate_abort)

    def test_default_manual_timeout_is_short(self):
        h = GateHarness()
        self.assertEqual(h.m1.manual_gate_timeout, 30.)
        self.assertEqual(h.m1.gate_timeout, 1800.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
