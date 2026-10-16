"""KAN-558: steps queued just after a toolhead reconnect must not run backwards.

On boxwood (2026-10-14 13:36 BST) the toolhead MCU reconnected and
TOOLHEAD_CONNECT started LOAD_FILAMENT 0.25 s later. The extruder homing move
was queued before motion_queuing.stats() had run for the reconnected MCU, so
its steppersync still held the placeholder time offset that
setup_mcu_movequeue() sets (0). Steps were generated thousands of seconds
into the MCU's future; the next stats() tick set the real offset, later
steps landed before them, and note_homing_end()'s flush failed with

    stepcompress o=19 i=0 c=65535 a=0: Invalid sequence

which shut Klipper down ("Internal error in stepcompress").

This replays that sequence through the real PrinterMotionQueuing methods and
the real C step generator (c_helper.so via chelper.get_ffi()), with the clock
values from the boxwood log. No reactor, MCU dictionary or hardware.

    python3 test/motion_queuing/test_reconnect_step_time.py
"""
import os
import socket
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

import chelper  # noqa: E402
from extras import motion_queuing  # noqa: E402

MCU_FREQ = 12000000.
# clock_adj from the toolhead clocksync dump at the shutdown: toolhead clock
# 0 corresponds to print_time 15197.540 after the reconnect.
RECONNECT_ADJ = (15197.540, 12000052.750)
# The extruder homing move (EXTRUDER_HOMING_MOVE ... distance=50 speed=15
# accel=50) started at print_time 15199.086 in the log.
MOVE_START = 15199.086
STEP_DIST = 1. / 400.


class McuError(Exception):
    pass


class FakeMcu:
    error = McuError
    def __init__(self, adj):
        self.adj = adj
        self.calibrations = 0
    def seconds_to_clock(self, t):
        return int(t * MCU_FREQ)
    def calibrate_clock(self, print_time, eventtime):
        self.calibrations += 1
        return self.adj
    def estimated_print_time(self, eventtime):
        return eventtime


class FakeReactor:
    def __init__(self):
        self.now = 0.
    def monotonic(self):
        return self.now
    class _NoPause:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
    def assert_no_pause(self):
        return self._NoPause()


def make_motion_queuing(main_mcu):
    # PrinterMotionQueuing.__init__ needs a whole printer; set the fields
    # the methods under test read instead.
    ffi_main, ffi_lib = chelper.get_ffi()
    mq = object.__new__(motion_queuing.PrinterMotionQueuing)
    mq.reactor = FakeReactor()
    mq.trapqs = []
    mq.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
    mq.steppersyncmgr = ffi_main.gc(ffi_lib.steppersyncmgr_alloc(),
                                    ffi_lib.steppersyncmgr_free)
    mq.syncemitters = []
    mq.steppersyncs = []
    mq.movequeue_mcus = []
    mq.steppersyncmgr_gen_steps = ffi_lib.steppersyncmgr_gen_steps
    mq.clear_history_time = 0.
    mq.flush_callbacks = []
    mq.kin_flush_delay = motion_queuing.SDS_CHECK_TIME
    mq.mcu = main_mcu
    mq.can_pause = True
    mq.last_flush_time = mq.last_step_gen_time = 0.
    mq.need_flush_time = mq.need_step_gen_time = 0.
    mq.drip_start_times = []
    return mq


def homing_move(ffi_lib, trapq, start):
    # 50 mm at 15 mm/s, accel 50 mm/s^2: 0.3 s ramps, 3.033 s cruise.
    accel_t = decel_t = 15. / 50.
    cruise_t = (50. - 2. * 0.5 * 50. * accel_t ** 2) / 15.
    ffi_lib.trapq_append(trapq, start, accel_t, cruise_t, decel_t,
                         0., 0., 0., 1., 0., 0., 0., 15., 50.)
    return start + accel_t + cruise_t + decel_t


def drip(mq, start, end):
    # What drip_update_time() does: flush in 50 ms segments.
    flush_time = start
    while flush_time < end:
        flush_time = min(flush_time + motion_queuing.DRIP_SEGMENT_TIME, end)
        mq._advance_flush_time(flush_time - motion_queuing.SDS_CHECK_TIME,
                               flush_time)


def replay_reconnect():
    """Run the boxwood sequence; return (error or None, toolhead mcu)."""
    ffi_main, ffi_lib = chelper.get_ffi()
    main_mcu = FakeMcu((0., MCU_FREQ))
    toolhead = FakeMcu((10.0, MCU_FREQ))   # its first boot, long ago
    mq = make_motion_queuing(main_mcu)
    trapq = mq.allocate_trapq()
    se = mq.allocate_syncemitter(toolhead, "stepper extruder")
    sc = ffi_lib.syncemitter_get_stepcompress(se)
    ffi_lib.stepcompress_fill(sc, 19, int(.000025 * MCU_FREQ), 1, 2)
    sk = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
    ffi_lib.itersolve_set_trapq(sk, trapq, STEP_DIST)
    ffi_lib.syncemitter_set_stepper_kinematics(se, sk)

    a, b = socket.socketpair()
    sq = ffi_lib.serialqueue_alloc(a.fileno(), b'u', 0, b'toolhead test')
    try:
        # Startup: first connect, then a stats() tick before any motion.
        mq.setup_mcu_movequeue(toolhead, sq, 1024)
        assert toolhead.calibrations == 0, \
            "first setup must keep upstream behaviour (no calibration)"
        mq.stats(10.)
        # ... a long session; print_time reaches ~15199.
        mq.last_flush_time = mq.last_step_gen_time = MOVE_START - 1.
        # Umbilical out: stepper._handle_noncritical_disconnect().
        assert ffi_lib.stepcompress_reset(sc, 0) == 0
        # Umbilical back in: the toolhead clock restarted near zero.
        toolhead.adj = RECONNECT_ADJ
        mq.setup_mcu_movequeue(toolhead, sq, 1024)
        # TOOLHEAD_CONNECT -> LOAD_FILAMENT -> EXTRUDER_HOMING_MOVE, queued
        # before the next stats() tick.
        end = homing_move(ffi_lib, trapq, MOVE_START)
        try:
            drip(mq, MOVE_START, MOVE_START + 0.5)
            mq.stats(MOVE_START + 0.5)        # the 1 s stats tick lands
            drip(mq, MOVE_START + 0.5, end)
        except McuError as e:
            return str(e), toolhead
        # note_homing_end(): stepcompress_reset() flushes everything left.
        if ffi_lib.stepcompress_reset(sc, 0):
            return "stepcompress_reset failed", toolhead
        return None, toolhead
    finally:
        ffi_lib.serialqueue_exit(sq)
        ffi_lib.serialqueue_free(sq)
        a.close()
        b.close()


def test_steps_after_reconnect_do_not_run_backwards():
    err, toolhead = replay_reconnect()
    assert err is None, (
        "step generation failed after a toolhead reconnect: %s" % (err,))
    # The reconnect setup itself calibrated, plus the one stats() tick
    # during the move (the stats() at startup is the third).
    assert toolhead.calibrations == 3, toolhead.calibrations


def main():
    tests = [test_steps_after_reconnect_do_not_run_backwards]
    for test in tests:
        test()
        print("ok  %s" % test.__name__)
    return 0


if __name__ == '__main__':
    sys.exit(main())
