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
        polls = [0]

        def render(section):
            context = {'mcu toolhead': {
                'non_critical_disconnected': not online},
                'filament_switch_sensor filament_sensor_toolhead':
                    helper.get_status(reactor.now),
                'gcode_macro TOOLHEAD_DISCONNECT': {'last_partfan_s': 0},
                'gcode_macro TOOLHEAD_CONNECT_WAIT': {'polls': polls[0]}}
            script = env.from_string(cfg.get(section, 'gcode')).render(
                printer=context)
            for line in script.splitlines():
                line = line.strip()
                if line == 'LOAD_FILAMENT':
                    loads.append(reactor.now)
                if line.startswith('SET_GCODE_VARIABLE MACRO='
                                   'TOOLHEAD_CONNECT_WAIT'):
                    polls[0] = int(line.split('VALUE=')[1])
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

    def test_registered_debounce_restarts_for_unchanged_sample(self):
        printer, _, helper = fixtures.make_runout_helper()
        mcu = fixtures.FakeMcu()

        class Pins:
            def lookup_pin(self, pin, **kw):
                return {'chip': mcu, 'chip_name': 'toolhead', 'pin': pin,
                        'invert': 0, 'pullup': 0}

        printer.objects.update({'pins': Pins(), 'query_adc': object()})
        config = fixtures.FakeConfig(printer, {'debounce_delay': .2})
        registered = buttons.PrinterButtons(config)
        registered.register_debounce_button(
            'gpio3', helper.note_filament_present, config,
            resync_on_reconnect=True)
        mb = registered.mcu_buttons['toolhead']
        for reconnect, sample in ((0., 1.55), (10., 11.55)):
            printer.reactor.now = reconnect
            printer.send_event(fixtures.DISCONNECT)
            for cb in mcu.config_callbacks:
                cb()
            printer.send_event(fixtures.RECONNECT)
            self.assertFalse(helper.sample_ready)
            printer.reactor.now = sample
            mb.handle_buttons_state({'ack_count': 0, 'state': b'\x01',
                                     '#receive_time': sample})
            printer.reactor.now = sample + .2
            printer.reactor.run_callbacks()
            self.assertTrue(helper.sample_ready,
                            'unchanged pin must cross debounce each time')


class ConnectLoadBoundTest(unittest.TestCase):
    """TOOLHEAD_CONNECT_LOAD gives up after 50 polls (about 5 s)."""
    def setUp(self):
        cfg = configparser.RawConfigParser(strict=False)
        cfg.read(ROOT / 'core/M1/macros/toolhead.cfg')
        self.cfg = cfg
        self.env = jinja2.Environment('{%', '%}', '{', '}')
        self.reactor = Reactor()
        self.polls = 0
        self.loads = []
        self.messages = []
        self.gen = 0
        self.ready = False
        self.detected = True

    def render(self, section):
        sensor = {'sample_ready': self.ready,
                  'filament_detected': self.detected}
        context = {
            'mcu toolhead': {'non_critical_disconnected': False},
            'filament_switch_sensor filament_sensor_toolhead': sensor,
            'gcode_macro TOOLHEAD_DISCONNECT': {'last_partfan_s': 0},
            'gcode_macro TOOLHEAD_CONNECT_WAIT': {'polls': self.polls}}
        script = self.env.from_string(self.cfg.get(section, 'gcode')).render(
            printer=context,
            action_respond_info=lambda m: self.messages.append(m) or '')
        for line in script.splitlines():
            line = line.strip()
            if line == 'LOAD_FILAMENT':
                self.loads.append(self.reactor.now)
            elif line.startswith('SET_GCODE_VARIABLE MACRO='
                                 'TOOLHEAD_CONNECT_WAIT'):
                self.polls = int(line.split('VALUE=')[1])
            elif line.startswith('UPDATE_DELAYED_GCODE ID='
                                 'TOOLHEAD_CONNECT_LOAD'):
                delay = float(line.split('DURATION=')[1])
                # One timer: re-arming replaces the pending run.
                self.gen += 1
                if delay:
                    self.reactor.register_callback(
                        lambda et, g=self.gen: g == self.gen and self.render(
                            'delayed_gcode TOOLHEAD_CONNECT_LOAD'),
                        self.reactor.now + delay)

    def connect(self):
        self.render('gcode_macro TOOLHEAD_CONNECT')

    def test_gives_up_after_the_bound_and_does_not_load(self):
        self.connect()
        self.reactor.advance(30.)
        self.assertEqual(self.loads, [])
        self.assertEqual(len(self.messages), 1)
        self.assertIn('not loading automatically', self.messages[0])
        self.assertEqual(self.polls, 50)
        # Nothing is armed any more: a sample hours later loads nothing.
        self.ready = True
        self.reactor.advance(36000.)
        self.assertEqual(self.loads, [])

    def test_sample_before_the_bound_loads_once(self):
        self.connect()
        self.reactor.advance(3.)
        self.assertEqual(self.loads, [])
        self.ready = True
        self.reactor.advance(30.)
        self.assertEqual(len(self.loads), 1)
        self.assertEqual(self.messages, [])

    def test_new_connect_resets_the_count(self):
        self.connect()
        self.reactor.advance(4.)
        self.assertGreater(self.polls, 30)
        self.connect()
        self.assertEqual(self.polls, 0)
        # Another 4 s is within the bound again: no give-up message yet.
        self.reactor.advance(8.)
        self.assertEqual(self.messages, [])
        self.ready = True
        self.reactor.advance(20.)
        self.assertEqual(len(self.loads), 1)


if __name__ == '__main__':
    unittest.main()
