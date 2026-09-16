"""Host-only regression tests; no MCU or native Klipper build required.

Load the production classes without their hardware driver imports. Tests use
fake clocks and queues to check deadlines, wakeups, and error propagation.
"""
import ast
import pathlib
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_class(path, name, **dependencies):
    source = ast.parse((ROOT / path).read_text(encoding='utf-8'))
    node = next(n for n in source.body
                if isinstance(n, ast.ClassDef) and n.name == name)
    module = ast.Module(body=[node], type_ignores=[])
    exec(compile(module, str(path), 'exec'), dependencies)
    return dependencies[name]


class Reactor:
    def __init__(self):
        self.now = 10.
        self.events = []

    def monotonic(self):
        return self.now

    def completion(self):
        reactor = self

        class Completion:
            done = False

            def complete(self, result):
                self.done = True

            def wait(self, deadline):
                while reactor.events and reactor.events[0][0] <= deadline:
                    reactor.now, callback = reactor.events.pop(0)
                    callback()
                    if self.done:
                        return True
                reactor.now = deadline
        return Completion()


def make_collector():
    reactor = Reactor()
    mcu = types.SimpleNamespace(estimated_print_time=lambda t: t,
                                is_fileoutput=lambda: False)
    sensor = types.SimpleNamespace(get_mcu=lambda: mcu,
        get_status=lambda t: dict(errors=0, overflows=0),
        get_samples_per_second=lambda: 1000.)
    load_cell = types.SimpleNamespace(sensor=sensor, add_client=lambda cb: None)
    printer = types.SimpleNamespace(get_reactor=lambda: reactor,
                                     command_error=RuntimeError)
    cls = load_class('klippy/extras/load_cell.py', 'LoadCellSampleCollector',
                     RETRY_DELAY=.05)
    return cls(printer, load_cell), reactor


class CollectorTests(unittest.TestCase):
    def test_finishing_batch_wakes_before_poll_interval(self):
        collector, reactor = make_collector()
        collector.start_collecting(min_time=10.)
        reactor.events.append((10.012, lambda: collector._on_samples(
            dict(errors=0, overflows=0,
                 data=[(10.005, 1., 1), (10.011, 0., 0)]))))
        samples, errors = collector.collect_until(10.010)
        self.assertEqual(reactor.now, 10.012)
        self.assertEqual(samples, [(10.005, 1., 1)])
        self.assertEqual(errors, 0)

    def test_tare_excludes_moving_samples_and_keeps_errors(self):
        collector, reactor = make_collector()
        collector.start_collecting(min_time=10.010)
        reactor.events.append((10.015, lambda: collector._on_samples(
            dict(errors=1, overflows=2,
                 data=[(10.009, 99., 99), (10.011, 1., 1),
                       (10.012, 2., 2)]))))
        samples, errors = collector.collect_min(2)
        self.assertEqual(len(samples), 2)
        self.assertEqual(errors, (1, 2))
        self.assertEqual(reactor.now, 10.015)

    def test_dead_sensor_times_out_and_stops(self):
        collector, reactor = make_collector()
        collector.start_collecting(min_time=10.)
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            collector.collect_until(10.010)
        self.assertFalse(collector.is_started)
        self.assertIsNone(collector._completion)

    def test_prebuffered_samples_do_not_wait(self):
        collector, reactor = make_collector()
        collector.start_collecting(min_time=10.)
        collector._on_samples(dict(errors=0, overflows=0,
            data=[(10.001, 1., 1), (10.002, 2., 2)]))
        self.assertEqual(len(collector.collect_min(2)[0]), 2)
        self.assertEqual(reactor.now, 10.)


class QueueTests(unittest.TestCase):
    def test_reading_idle_queue_does_not_prime(self):
        cls = load_class('klippy/toolhead.py', 'ToolHead')
        obj = cls.__new__(cls)
        obj.print_time = 1.
        obj._process_lookahead = lambda: None
        obj._calc_print_time = lambda: self.fail('unexpected priming')
        self.assertEqual(obj.get_last_queued_move_time(), 1.)

    def test_pending_moves_are_scheduled_before_read(self):
        cls = load_class('klippy/toolhead.py', 'ToolHead')
        obj = cls.__new__(cls)
        obj.print_time = 1.
        obj._process_lookahead = lambda: setattr(obj, 'print_time', 2.)
        self.assertEqual(obj.get_last_queued_move_time(), 2.)

    def test_move_callback_survives_immediate_flush(self):
        move = types.SimpleNamespace(move_d=1., is_kinematic_move=True,
            end_pos=[0., 0., 1., 0.], timing_callbacks=[],
            accel_t=.01, cruise_t=.03, decel_t=.01)
        cls = load_class('klippy/toolhead.py', 'ToolHead',
                         Move=lambda *args: move)
        obj = cls.__new__(cls)
        obj.kin = types.SimpleNamespace(check_move=lambda m: None)
        obj.extra_axes = []
        obj.commanded_pos = [0.] * 4
        obj.print_time = 0.
        obj.need_check_pause = 100.
        obj.lookahead = types.SimpleNamespace(add_move=lambda m: True)
        obj._process_lookahead = lambda lazy: move.timing_callbacks[0](10.30)
        timings = []
        obj.move(move.end_pos, 10., lambda a, b: timings.extend((a, b)))
        self.assertAlmostEqual(timings[0], 10.25)
        self.assertAlmostEqual(timings[1], 10.30)


class ProbeTests(unittest.TestCase):
    def test_tare_deadline_is_after_travel_and_settling(self):
        for queued, expected in [(9., 10.04), (11., 11.04)]:
            captured = []
            reactor = Reactor()
            toolhead = types.SimpleNamespace(
                get_last_queued_move_time=lambda: queued)
            collector = types.SimpleNamespace(
                start_collecting=lambda **kw: captured.append(kw['min_time']))
            cls = load_class('klippy/extras/load_cell_probe.py',
                             'LoadCellProbingMove')
            obj = cls.__new__(cls)
            obj._printer = types.SimpleNamespace(
                lookup_object=lambda n: toolhead, get_reactor=lambda: reactor)
            obj._mcu = types.SimpleNamespace(estimated_print_time=lambda t: t)
            obj._load_cell = types.SimpleNamespace(get_collector=lambda: collector)
            obj._start_collector(True, .04)
            self.assertAlmostEqual(captured[0], expected)

    def run_tap(self, duration, sensor_error=False, first_home=False):
        reactor = Reactor()
        calls = []
        pos = [0., 0., 5., 0.]
        toolhead = types.SimpleNamespace(get_position=lambda: list(pos))
        toolhead.set_position = lambda p: calls.append(('set_position', p))
        def lift(p, speed, timing_callback):
            calls.append(('lift', p, speed))
            toolhead.pending = lambda: timing_callback(10.25, 10.25 + duration)
        toolhead.manual_move = lift
        toolhead.get_last_queued_move_time = lambda: toolhead.pending()
        def collect(deadline):
            calls.append(('deadline', deadline))
            return [(10.26, 1., 1), (deadline + .01, 9., 9)], sensor_error
        def check(results, printer):
            if results[1]:
                raise RuntimeError('sensor error')
            return results[0]
        homing = types.SimpleNamespace(check_probe_first_home=lambda g: first_home)
        printer = types.SimpleNamespace(get_reactor=lambda: reactor,
            lookup_object=lambda name: dict(toolhead=toolhead, homing=homing)[name],
            command_error=RuntimeError)
        analysis = lambda samples: types.SimpleNamespace(to_dict=lambda: {})
        cls = load_class('klippy/extras/load_cell_probe.py', 'TappingMove',
            ASCENT_DATA_WINDOW_SECONDS=.3, check_sensor_errors=check,
            TapAnalysis=analysis)
        obj = cls.__new__(cls)
        obj._printer = printer
        obj._config_helper = types.SimpleNamespace(get_low_latency=lambda g: True)
        collector = types.SimpleNamespace(collect_until=collect)
        obj._load_cell_probing_move = types.SimpleNamespace(
            probing_move=lambda g: ([0., 0., 2., 0.], collector),
            _param_helper=types.SimpleNamespace(get_probe_params=lambda g:
                dict(load_cell_retract_dist=.4, lift_speed=10.)))
        def fit(g, samples, start, th, raw):
            calls.append(('fit', samples, start, raw))
            return 1.99
        obj._analyze_ascent = fit
        obj._clients = types.SimpleNamespace(send=lambda d: None)
        gcmd = types.SimpleNamespace(get_int=lambda *a, **kw: 0)
        if sensor_error:
            with self.assertRaisesRegex(RuntimeError, 'sensor error'):
                obj.run_tap(gcmd)
            self.assertFalse(obj._is_last_result_valid)
            self.assertFalse(any(c[0] == 'fit' for c in calls))
        else:
            result, valid = obj.run_tap(gcmd)
            self.assertTrue(valid)
            self.assertEqual(result[2], 1.99)
            fit_call = next(c for c in calls if c[0] == 'fit')
            self.assertEqual(len(fit_call[1]), 1)
            self.assertEqual(fit_call[2], 10.25)
        return calls

    def test_short_ascent_collects_through_its_end(self):
        self.assertIn(('deadline', 10.30), self.run_tap(.05))

    def test_long_ascent_only_waits_for_fit_window(self):
        self.assertIn(('deadline', 10.55), self.run_tap(1.))

    def test_sensor_error_prevents_fitting(self):
        self.run_tap(.05, sensor_error=True)

    def test_first_home_normalizes_before_lift(self):
        calls = self.run_tap(.05, first_home=True)
        self.assertEqual(calls[0], ('set_position', [0., 0., 3., 0.]))
        self.assertEqual(calls[1][0], 'lift')


if __name__ == '__main__':
    unittest.main()
