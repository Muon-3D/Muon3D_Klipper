"""KAN-463: a failed tap must not leave its sample collector running.

A tap starts a LoadCellSampleCollector with no end time and no sample limit,
then descends.  If the descent raised ("No trigger on probe after full
movement"), nothing stopped the collector: it stayed subscribed to the load
cell and appended ~1930 samples a second for the life of klippy.  klippy
runs its own gc.collect(2), which has to walk that list, so every 45 s the
reactor stalled a little longer until a stall outlasted the step buffer and
the MCU shut down with "Timer too close" mid-print (boxwood, 2026-10-01).

Drives the real LoadCellSampleCollector through both failure windows, with no
klippy reactor or hardware:

* the descent itself raising (LoadCellProbingMove.probing_move)
* anything raising between the descent and collect_until (TappingMove.run_tap)

    python3 test/load_cell/test_collector_leak.py
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import load_cell, load_cell_probe  # noqa: E402


class ProbeFailed(Exception):
    pass


class FakeMcu:
    def is_fileoutput(self):
        return False


class FakeSensor:
    def get_status(self, eventtime):
        return {"errors": 0, "overflows": 0}
    def get_mcu(self):
        return FakeMcu()


class FakeLoadCell:
    def __init__(self):
        self.sensor = FakeSensor()
        self.clients = []
    def add_client(self, cb):
        self.clients.append(cb)


class FakeToolhead:
    def __init__(self, flush_raises=False):
        self.flush_raises = flush_raises
    def get_position(self):
        return [10., 10., 5., 0.]
    def dwell(self, delay):
        pass
    def flush_step_generation(self):
        if self.flush_raises:
            raise ProbeFailed("flush failed")


class FakeHoming:
    def check_probe_first_home(self, gcmd):
        raise ProbeFailed("ascent homing history failed")
    def probing_move(self, endstop, pos, speed):
        raise ProbeFailed("No trigger on probe after full movement")


class FakeReactor:
    def monotonic(self):
        return 0.


class FakePrinter:
    def __init__(self, toolhead):
        self.objects = {'toolhead': toolhead, 'homing': FakeHoming()}
    def get_reactor(self):
        return FakeReactor()
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class FakeConfigHelper:
    def validate_probe_setup(self, gcmd):
        pass


class FakeParamHelper:
    def get_probe_params(self, gcmd):
        return {'probe_speed': 5.}


def new_collector(printer, cell):
    collector = load_cell.LoadCellSampleCollector(printer, cell)
    collector.start_collecting(min_time=0.)
    return collector


def sample_msg():
    # One bulk message: [time, grams, counts, tare]
    return {'errors': 0, 'overflows': 0, 'data': [[1., 0., 100, 100]]}


def assert_stopped(collector, cell):
    assert cell.clients, "the collector never subscribed"
    assert not collector.is_started, "collector is still started"
    # Klipper drops a client whose callback returns False.  A leaked
    # collector returns True and keeps every sample it is given.
    keep = cell.clients[-1](sample_msg())
    assert keep is False, "collector is still subscribed to the load cell"
    assert collector._samples == [], (
        "collector kept %d sample(s) after the tap failed"
        % len(collector._samples))


def make_probing_move(printer, cell):
    m = object.__new__(load_cell_probe.LoadCellProbingMove)
    m._printer = printer
    m._config_helper = FakeConfigHelper()
    m._param_helper = FakeParamHelper()
    m._approx_tare = None
    m._z_min_position = -2.
    m._mcu_trigger_analog = object()
    m._pause_and_tare = lambda gcmd: None
    m.collector = None
    def start_collector():
        m.collector = new_collector(printer, cell)
        return m.collector
    m._start_collector = start_collector
    return m


def test_descent_raises():
    cell = FakeLoadCell()
    printer = FakePrinter(FakeToolhead())
    m = make_probing_move(printer, cell)
    try:
        m.probing_move(None)
    except ProbeFailed:
        pass
    else:
        raise AssertionError("the descent was expected to raise")
    assert_stopped(m.collector, cell)


class TailCommand:
    def get_float(self, name, default=None, **kwargs):
        assert name == "TAIL"
        return 0.1


def test_raise_between_descent_and_collect():
    cell = FakeLoadCell()
    printer = FakePrinter(FakeToolhead(flush_raises=True))
    collector = new_collector(printer, cell)

    class Descent:
        def probing_move(self, gcmd):
            return [10., 10., 0., 0.], collector

    t = object.__new__(load_cell_probe.TappingMove)
    t._printer = printer
    t._load_cell_probing_move = Descent()
    try:
        t.run_tap(TailCommand())
    except ProbeFailed:
        pass
    else:
        raise AssertionError("run_tap was expected to raise")
    assert_stopped(collector, cell)


def test_ascent_failure_stops_collector():
    cell = FakeLoadCell()
    printer = FakePrinter(FakeToolhead())
    collector = new_collector(printer, cell)

    class Descent:
        def probing_move(self, gcmd):
            return [10., 10., 0., 0.], collector

    t = object.__new__(load_cell_probe.TappingMove)
    t._printer = printer
    t._load_cell_probing_move = Descent()
    try:
        t.run_tap(None)
    except ProbeFailed:
        pass
    else:
        raise AssertionError("ascent history was expected to raise")
    assert_stopped(collector, cell)


def main():
    failed = 0
    for test in (test_descent_raises, test_raise_between_descent_and_collect,
                 test_ascent_failure_stops_collector):
        try:
            test()
            print("ok   %s" % test.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL %s: %s" % (test.__name__, e))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
