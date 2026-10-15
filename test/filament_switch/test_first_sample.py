"""KAN-558: replay the 15 Oct button/query race through real macros."""
import configparser
import heapq
import pathlib
import sys
import unittest

import jinja2
import test_reconnect_resync as fixtures

buttons = fixtures.buttons
ROOT = pathlib.Path(__file__).resolve().parents[2]


class Reactor(fixtures.FakeReactor):
    def register_callback(self, cb, waketime=0.):
        heapq.heappush(self.callbacks, (max(self.now, waketime),
                                     len(self.callbacks), cb))

    def advance(self, until):
        while self.callbacks and self.callbacks[0][0] <= until:
            self.now, _, cb = heapq.heappop(self.callbacks)
            cb(self.now)
        self.now = until


class ReconnectTest(unittest.TestCase):
    def scenario(self, before, after, raw_time=1.55, settled=3.75):
        printer, gcode, helper = fixtures.make_runout_helper()
        printer.reactor = reactor = Reactor()
        helper.reactor = reactor
        helper._handle_ready()
        helper.filament_present = bool(before)
        mcu = fixtures.FakeMcu()
        mb = buttons.MCU_buttons(printer, mcu)
        debounce = buttons.DebounceButton(
            fixtures.FakeConfig(printer, {'debounce_delay': .2}),
            helper.note_filament_present)
        mb.setup_buttons([{'pin': 'gpio3', 'pullup': 0, 'invert': 0}],
                         debounce.button_handler, resync_on_reconnect=True)
        mb.last_button = before
        # As PrinterButtons does, reset debouncing at each config rebuild.
        reset = getattr(debounce, 'reset', lambda: None)
        reset()
        printer.send_event(fixtures.DISCONNECT)
        mb.build_config()
        reset()
        printer.send_event(fixtures.RECONNECT)
        cfg = configparser.RawConfigParser(strict=False)
        cfg.read(ROOT / 'core/M1/macros/toolhead.cfg')
        env = jinja2.Environment('{%', '%}', '{', '}')
        loads = []
        online = True

        def render(section):
            context = {'mcu toolhead': {
                'non_critical_disconnected': not online},
                'filament_switch_sensor filament_sensor_toolhead':
                    helper.get_status(reactor.now),
                'gcode_macro TOOLHEAD_DISCONNECT': {'last_partfan_s': 0}}
            script = env.from_string(cfg.get(section, 'gcode')).render(
                printer=context)
            for line in script.splitlines():
                line = line.strip()
                if line == 'LOAD_FILAMENT':
                    loads.append(reactor.now)
                if line.startswith('UPDATE_DELAYED_GCODE '):
                    args = dict(p.split('=') for p in line.split()[1:])
                    if args['ID'] != 'TOOLHEAD_CONNECT_LOAD':
                        continue
                    delay = float(args['DURATION'])
                    if delay:
                        reactor.register_callback(
                            lambda et: render(
                                'delayed_gcode TOOLHEAD_CONNECT_LOAD'),
                            reactor.now + delay)

        reactor.advance(.25)
        render('gcode_macro TOOLHEAD_CONNECT')
        # Raw sample at +1.55, debounced value at measured +1.75.
        reactor.advance(raw_time)
        mb.handle_buttons_state({'ack_count': 0, 'state': bytes([after]),
                                 '#receive_time': reactor.now})
        reactor.advance(raw_time + .199)
        self.assertEqual(loads, [], 'loaded before first debounced sample')
        reactor.advance(raw_time + .3)
        self.assertEqual(len(loads), int(bool(after)),
                         'reattach decision used the pre-unplug value')
        self.assertEqual(fixtures.inserts(gcode), [])
        self.assertEqual(helper.settle_until, settled)
        # A transition at +3.0 is still in the sample-based settle window.
        helper.note_filament_present(settled - .75, False)
        helper.note_filament_present(settled - .65, True)
        reactor.advance(settled - .55)
        self.assertEqual(fixtures.inserts(gcode), [])
        reactor.advance(settled + .25)
        helper.note_filament_present(settled + .25, False)
        helper.note_filament_present(settled + .35, True)
        reactor.advance(settled + .45)
        self.assertEqual(len(fixtures.inserts(gcode)), 1)

    def test_step2_removed_while_unplugged(self):
        self.scenario(1, 0)

    def test_step3_inserted_while_unplugged(self):
        self.scenario(0, 1)

    def test_step4_unchanged_filament_still_loads_once(self):
        self.scenario(1, 1)

    def test_sample_delayed_to_five_seconds_still_controls_load(self):
        self.scenario(0, 1, raw_time=4.8, settled=7.)

    def test_late_sample_while_offline_does_not_release_load_gate(self):
        printer, _, helper = fixtures.make_runout_helper()
        printer.send_event(fixtures.DISCONNECT)
        helper.note_filament_present(1., True)
        self.assertFalse(helper.get_status(1.)['sample_ready'])

    def test_queued_old_connection_sample_is_discarded(self):
        printer, _, mb, _, filament = fixtures.make_buttons()
        pending = []
        printer.reactor.register_async_callback = pending.append
        mb.build_config()
        fixtures.McuSide(mb).report(fixtures.raw_state(0, 1))
        printer.send_event(fixtures.DISCONNECT)
        mb.build_config()
        printer.send_event(fixtures.RECONNECT)
        for cb in pending:
            cb(2.)
        self.assertEqual(filament.states, [])

    def test_buttons_query_starts_only_after_configured_reconnect(self):
        printer, mcu, mb, _, _ = fixtures.make_buttons()
        mb.build_config()
        self.assertFalse(any(c.startswith('buttons_query ')
                             for c in mcu.config_cmds))
        self.assertEqual(mcu.query.sent, [])
        printer.send_event(fixtures.RECONNECT)
        self.assertEqual(len(mcu.query.sent), 1)

    def test_runout_also_settles_from_first_debounced_sample(self):
        printer, gcode, helper = fixtures.make_runout_helper()
        printer.objects['idle_timeout'].get_status = (
            lambda et: {'state': 'Printing'})
        helper.runout_gcode = fixtures.FakeTemplate('RUNOUT')
        helper._handle_ready()
        printer.send_event(fixtures.RECONNECT)
        fixtures.edge(printer, helper, 1.75, True)
        fixtures.edge(printer, helper, 3., False)
        self.assertEqual(gcode.scripts, [])
        fixtures.edge(printer, helper, 3.1, True)
        fixtures.edge(printer, helper, 4., False)
        self.assertEqual(gcode.scripts, ['RUNOUT\nM400'])


if __name__ == '__main__':
    unittest.main()
