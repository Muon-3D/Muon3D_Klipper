# Refuse XY moves while the nozzle is touching the plate
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Watches the load_cell_probe's load cell outside of probing and publishes
# whether the nozzle is pressing on something.  While it is, a G-code move
# that travels in X or Y without also raising Z is refused, so no client
# (MuonUI, the app, a typed G1) can drag the nozzle across the plate.
#
# The guard is a gcode_move transform at the head of the chain, so it sees
# moves in G-code coordinates before skew_correction or bed_mesh alter them.
import logging, collections, sys

REFUSAL = "Nozzle is touching the plate: lift Z before moving X or Y"
# Fail closed: near the plate, or while the last state was contact, an XY
# move with no load cell data covering its start is refused too
NO_DATA = ("contact_guard: no current load cell data, so contact cannot"
           " be ruled out: lift Z before moving X or Y")

# With no load cell samples for this long the guard goes inactive rather
# than act on a stale reading (toolhead MCU disconnected, stream stopped).
STREAM_TIMEOUT = 1.0
# Samples averaged to re-zero the baseline at the start of a homing move,
# and after a Z homing move that lifted (see _lift_zero_sample).
ANCHOR_TIME = 0.020
# After an upward Z homing move, re-zero on the first steady samples seen
# within this much post-move data, or give up.
LIFT_ZERO_WINDOW = 0.5
# A Z homing or probing move re-zeroes on what the cell read before it only
# if Z travelled at least this far down (toward the plate) before the
# trigger.  The M1's cell runs out of range (MCU guard) at ~0.093 mm of
# deflection, so a nozzle already pressing when the move began cannot
# travel this far further down; a probe started on the plate triggers
# almost at once.  Upward travel proves nothing about the start.
HOMING_CLEAR_TRAVEL = 0.1
# Below this a coordinate has not moved (G-code positions are floats).
MOVE_EPSILON = 1e-6
# Sample times are floats; "20 ms" must include the sample at 20 ms.
TIME_EPSILON = 1e-6
# Before deciding a move near the plate, wait for samples this far past the
# end of the queued motion (plus on_time), and give up this long after the
# samples should have arrived.
SYNC_MARGIN = 0.005
# Positive clearance must watch the whole Z travel: a gap between samples
# longer than this (2 kHz is 0.5 ms) restarts its steady window.
CLEAR_MAX_GAP = 0.005
# A reading this close to the cell's end of range is no evidence of
# anything: past ~0.093 mm of deflection the M1's preloaded cell sits flat
# near reference_max_load_counts however hard it is pressed, and the ADC
# clips at its rails.  Fractions of the zero-to-reference_max span and of
# the ADC range.
RANGE_FRACTION = 0.9
RAIL_FRACTION = 0.01
SYNC_TIMEOUT = 0.5
# Commands arriving over the G-code pty are a streamed job; it is still
# running this long after the last batch was seen.
STREAM_JOB_GRACE = 5.0
# Fixed raw zero for CONTACT_GUARD_SIMULATE (batch mode only).
SIM_ZERO_COUNTS = -1000000


class ContactGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Contact is "on" after the load stays above threshold_counts for
        # on_time, and "off" after it stays below half of it for off_time.
        self.threshold = config.getint('threshold_counts', 90000, minval=1)
        self.on_time = config.getfloat('on_time', 0.020, minval=0.)
        self.off_time = config.getfloat('off_time', 0.050, minval=0.)
        # The load cell rings for ~0.4 s after a probe tap.
        self.holdoff = config.getfloat('holdoff_time', 0.5, minval=0.)
        # The baseline follows drift with this time constant, and only
        # while the load is within baseline_band_counts of it.
        self.tau = config.getfloat('baseline_time_constant', 1.0, above=0.)
        self.band = config.getint('baseline_band_counts',
                                  self.threshold // 4, minval=1,
                                  maxval=self.threshold // 2)
        # Relock a steady sub-threshold offset after this long.  A load
        # above the threshold is contact and is never aged out: a stationary
        # toolhead trapq does not mean the nozzle is clear, because
        # FORCE_MOVE runs on its own trapq and the bed can be moved by hand.
        self.relock_time = config.getfloat('relock_time', 0.5, above=0.)
        # Positive clearance.  A load that stays within the band while the
        # executed toolhead Z travels clearance_lift (up only, if latched as
        # contact; either way otherwise; with no position reset in between,
        # and never at the end of the cell's range) is not touching
        # anything stiff: the
        # plate deflects ~0.01 mm per 200000 counts, so a touch changes by
        # millions of counts over 1 mm.  Drift and filament tension do not
        # follow Z.  The steady load becomes the new zero, latched or not.
        self.clearance_lift = config.getfloat('clearance_lift', 1.0,
                                              above=0.)
        # SET_CONTACT_GUARD REZERO=1 is the explicit recovery for an offset
        # that does not follow Z (filament tension after the unlock
        # retract).  It only runs with Z really homed (home_rails, or a
        # homing_override script whose Z probe triggered after clear
        # downward travel) since the last position reset, FORCE_MOVE or
        # Z motor disable, the toolhead at least this high, and a steady
        # load.
        self.rezero_min_z = config.getfloat('rezero_min_z', 10.0)
        # Relocks and drift following together may move the zero in the
        # pressing direction by at most this much in total since the last
        # real zero (Z homing or probing, positive clearance, REZERO);
        # moving back toward unloading gives it back.  Without it, steps or
        # creep each too small to see (jogging Z down onto the plate, a
        # hot end growing into it) add up without limit.  Drift past it
        # latches contact, which a 1 mm Z lift clears.
        self.max_absorbed = config.getint('max_absorbed_counts',
                                          self.threshold // 2, minval=0,
                                          maxval=self.threshold - 1)
        # Moves are checked as they enter the lookahead queue, before the
        # moves ahead of them have run.  Starting below this machine Z (or
        # with Z unhomed) the queued motion could put the nozzle on the
        # plate, so the guard first waits for it to execute and decides on
        # what the load cell reads afterwards.
        self.sync_below_z = config.getfloat('sync_below_z', 3.0)
        self.enabled = True
        # Objects found at connect
        self.probe = self.load_cell = self.print_stats = None
        self.homing_override = self.nozzle_wipe = None
        self.toolhead = self.gcode_io = None
        self.next_transform = None
        self.last_position = [0., 0., 0., 0.]
        # Host state that makes the guard inactive
        self.homing_moves = 0
        self.homing_rails = False
        # Sample processing state
        self.baseline = None
        self.load = 0.
        self.contact = False
        self.data_time = None  # newest sample time seen (print_time)
        self.sample_systime = None  # reactor time the last sample arrived
        self.prev_time = None
        self.in_gap = True
        self.above_since = self.below_since = None
        self.drift_since = None
        self.drift_sum = 0.
        self.drift_count = 0
        self.drift_min = self.drift_max = None
        self.clear_win = None
        # Toolhead position resets (SET_KINEMATIC_POSITION, homing) jump
        # the trapq; samples up to the newest one are no clearance evidence
        self.position_reset_time = float('-inf')
        self.absorbed = 0.  # pressing-direction zero movement since zero
        # The guard's own last error (a refusal, or a REZERO it turned
        # down).  gcode:command_error fires for it once per macro level;
        # none of those say anything about homing.
        self.own_error = None
        self.homing_candidate = None
        self.pending_zero = None  # applied once homing ends without error
        # After a Z homing move that lifted: re-zero on post-move samples
        self.lift_zero = None
        # Time of the first sample after the last gap in the stream
        self.resume_time = None
        # Z homed for real since the last position reset, FORCE_MOVE of a
        # Z stepper, or Z motor disable
        self.z_trusted = False
        # A homing_override probe triggered since the last position reset:
        # the reset that follows it (Z from the probe) keeps the trust
        self.probe_since_reset = False
        self.motion_known = False
        self.motion_base_z = None
        self.motion_moves = []
        self.recent = collections.deque()
        self.stream_systime = None  # last time pty input was seen
        self.sim_time = None
        self.sim_drift = 0.
        self.sim_plate = None
        # Events
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        self.printer.register_event_handler("toolhead:set_position",
                                            self._handle_set_position)
        self.printer.register_event_handler("force_move:manual_move",
                                            self._handle_manual_move)
        self.gcode.register_command('SET_CONTACT_GUARD',
                                    self.cmd_SET_CONTACT_GUARD,
                                    desc=self.cmd_SET_CONTACT_GUARD_help)
        if self.printer.get_start_args().get('debugoutput') is not None:
            # Batch mode has no sensor data; let regression tests feed some
            self.gcode.register_command('CONTACT_GUARD_SIMULATE',
                                        self.cmd_CONTACT_GUARD_SIMULATE)

    def _handle_connect(self):
        self.probe = self.printer.lookup_object('load_cell_probe', None)
        if self.probe is None or not hasattr(self.probe, 'get_load_cell'):
            raise self.printer.config_error(
                "[contact_guard] requires [load_cell_probe]")
        self.load_cell = self.probe.get_load_cell()
        # A host-side subscriber to the stream load_cell already runs
        self.load_cell.add_client(self._handle_batch)
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.homing_override = self.printer.lookup_object('homing_override',
                                                          None)
        # The brush wipe drags the nozzle through the brush under load
        self.nozzle_wipe = self.printer.lookup_object('nozzle_wipe_smart',
                                                      None)
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_io = self.printer.lookup_object('gcode_io', None)
        # A disabled Z motor can be turned by hand: Z is no longer known
        stepper_enable = self.printer.lookup_object('stepper_enable', None)
        if stepper_enable is not None:
            for stepper in self.toolhead.get_kinematics().get_steppers():
                if stepper.is_active_axis('z'):
                    stepper_enable.lookup_enable(
                        stepper.get_name()).register_state_callback(
                            self._handle_z_motor)

    def _handle_ready(self):
        # Registered at ready, after every klippy:connect transform
        # (skew_correction, z_thermal_adjust), so the guard is the head of
        # the chain and judges moves in G-code coordinates.
        gcode_move = self.printer.lookup_object('gcode_move')
        self.next_transform = gcode_move.set_move_transform(self, force=True)
        gcode_move.reset_last_position()

    # Move transform interface
    def get_position(self):
        self.last_position = list(self.next_transform.get_position())
        return list(self.last_position)

    def move(self, newpos, speed):
        self._check_move(newpos)
        self.next_transform.move(newpos, speed)
        self.last_position = list(newpos)

    def _check_move(self, newpos):
        last = self.last_position
        moves_xy = (abs(newpos[0] - last[0]) > MOVE_EPSILON
                    or abs(newpos[1] - last[1]) > MOVE_EPSILON)
        if not moves_xy or newpos[2] > last[2] + MOVE_EPSILON:
            return
        eventtime = self.reactor.monotonic()
        if not self._may_refuse(eventtime):
            return
        possible = self._contact_possible(last, eventtime)
        held = self.contact or self.above_since is not None
        if possible or self._settling() or self.lift_zero is not None:
            # Decide on samples that cover this move's start: after the
            # moves queued ahead of it, past a tap's ring, and with on_time
            # of data since any gap
            if not self._sync():
                logging.info("contact_guard: refused move %s -> %s, no"
                             " samples", last[:3], list(newpos[:3]))
                raise self._own_error(NO_DATA)
            eventtime = self.reactor.monotonic()
        if self._stale(eventtime) and (held or possible):
            # The stream stopped while touching, or near the plate: still
            # touching, as far as anything says
            raise self._own_error(NO_DATA)
        if self.contact and self.is_active(eventtime):
            logging.info("contact_guard: refused move %s -> %s, load %d",
                         last[:3], list(newpos[:3]), self.load)
            raise self._own_error(REFUSAL)

    def _stale(self, eventtime):
        return (self.sample_systime is None
                or eventtime - self.sample_systime > STREAM_TIMEOUT)

    def _settling(self):
        # The detector has had less than on_time of data since a probe's
        # hold-off or a gap: a touch there (PROBE leaves the nozzle on
        # what it hit) is not latched yet
        if self.data_time is None:
            return False
        since = max(self._holdoff_end(), self.resume_time
                    if self.resume_time is not None else float('-inf'))
        return self.data_time < since + self.on_time + SYNC_MARGIN

    def _contact_possible(self, last, eventtime):
        kin_status = self.toolhead.get_kinematics().get_status(eventtime)
        return ('z' not in kin_status['homed_axes']
                or last[2] < self.sync_below_z)

    def _sync(self):
        # Let everything queued ahead of this move execute, then wait for
        # samples long enough past its end for contact to be declared, and
        # past the ring of a tap that just happened.  This stalls only moves
        # that start near the plate.
        self.toolhead.wait_moves()
        target = max(self.toolhead.print_time, self._holdoff_end(),
                     self.resume_time if self.resume_time is not None
                     else float('-inf'))
        target += self.on_time + SYNC_MARGIN
        if self.sim_plate is not None:
            self._sim_plate_until(target)
            return True
        mcu = self.load_cell.sensor.get_mcu()
        if mcu.is_fileoutput():
            # Batch mode: no samples will come and print_time is not clock
            return True
        now = self.reactor.monotonic()
        deadline = (now + SYNC_TIMEOUT
                    + max(0., target - mcu.estimated_print_time(now)))
        # A re-zero pending from a Z lift is also waited for, until the
        # deadline: it may be what releases a false latch
        while (self.data_time is None or self.data_time < target
               or self.lift_zero is not None):
            if now >= deadline:
                break
            now = self.reactor.pause(now + 0.005)
        if self.data_time is None or self.data_time < target:
            logging.info("contact_guard: no samples past %.3f", target)
            return False
        return True

    def _own_error(self, msg):
        self.own_error = self.gcode.error(msg)
        return self.own_error

    # Inactive states
    def _handle_homing_move_begin(self, hmove):
        self.homing_moves += 1
        self.lift_zero = None
        self.own_error = None
        # Remember what the cell read just before the move.  It becomes the
        # zero at the end only if this was a Z move that travelled far
        # enough to prove the nozzle was clear when it began: an X or Y
        # homing move, or a probe started on the plate, proves nothing.
        self.homing_candidate = None
        if self.recent:
            self.homing_candidate = (sum(c for t, c in self.recent)
                                     / len(self.recent))

    def _z_travel(self, hmove):
        # (down, up): how far the Z steppers moved toward and away from the
        # plate before the trigger, in mm, from their own step counts
        batch = self.printer.get_start_args().get('debuginput') is not None
        down = up = 0.
        for sp in getattr(hmove, 'stepper_positions', []):
            if sp.trig_pos is None or not sp.stepper.is_active_axis('z'):
                continue
            if batch:
                # Batch mode simulates no stepper motion (as
                # check_no_movement does): take a Z move as a probe that
                # moved
                return float('inf'), 0.
            mtc = sp.stepper.mcu_to_commanded_position
            moved = mtc(sp.start_pos) - mtc(sp.trig_pos)
            down = max(down, moved)
            up = max(up, -moved)
        return down, up

    def _override_active(self):
        ho = self.homing_override
        return ho is not None and getattr(ho, 'in_script', False)

    def _handle_homing_move_end(self, hmove):
        self.homing_moves = max(0, self.homing_moves - 1)
        candidate, self.homing_candidate = self.homing_candidate, None
        travel, lifted = self._z_travel(hmove)
        if lifted >= self.clearance_lift - MOVE_EPSILON:
            # A real lift (the G28 macro's AXIS_PROTECTED_MOVE Z+10), on the
            # steppers' own counts.  Its samples are not watched, so the
            # reading before it proves nothing; a steady reading after it
            # is not a touch (it would have fallen away) unless the cell is
            # at the end of its range.  This is what lets G28 recover a
            # false latch with Z unhomed, when G1 Z is not allowed.
            self.lift_zero = {'after': self.toolhead.print_time,
                              'start': None, 'samples': []}
            logging.info("contact_guard: Z homing move lifted %.3f mm;"
                         " re-zeroing on the steady load after it", lifted)
            return
        if travel < HOMING_CLEAR_TRAVEL - MOVE_EPSILON:
            return
        if self._override_active():
            # The M1 homes Z this way: a probe from clear travel, then
            # SET_KINEMATIC_POSITION from what it measured.  A failed probe
            # raises, and the command error takes the trust back.
            self.z_trusted = True
            self.probe_since_reset = True
        if candidate is not None:
            # Applied once the command finishes without error
            self.pending_zero = candidate
            logging.info("contact_guard: Z homing move travelled %.3f mm"
                         " down; re-zeroing on what it read before", travel)

    def _apply_pending_zero(self):
        if self.pending_zero is None or self._is_homing():
            return
        self.baseline = self.pending_zero
        self.pending_zero = None
        self._real_zero()

    def _lift_zero_sample(self, t, counts, sign):
        # After an upward Z homing move: the first ANCHOR_TIME of steady,
        # in-range readings becomes the zero
        lz = self.lift_zero
        if lz['start'] is None:
            lz['start'] = t
        if t - lz['start'] > LIFT_ZERO_WINDOW:
            self.lift_zero = None
            logging.info("contact_guard: no steady load after the lift;"
                         " not re-zeroing")
            return False
        if self._out_of_range(counts, sign):
            lz['samples'] = []
            return False
        lz['samples'].append((t, counts))
        values = [c for st, c in lz['samples']]
        if max(values) - min(values) > self.band:
            lz['samples'] = [(t, counts)]
            return False
        if t - lz['samples'][0][0] < ANCHOR_TIME - TIME_EPSILON:
            return False
        self.baseline = float(sum(values)) / len(values)
        self.lift_zero = None
        self._real_zero()
        logging.info("contact_guard: re-zeroed on the load after a Z lift")
        return True

    def _real_zero(self):
        # The baseline was just set from evidence that nothing touched
        self.absorbed = 0.
        self._reset_detector()

    def _handle_home_rails_begin(self, homing_state, rails):
        self.homing_rails = True
        self.own_error = None

    def _handle_home_rails_end(self, homing_state, rails):
        self.homing_rails = False
        if 2 in homing_state.get_axes():
            self.z_trusted = True

    def _trapq_z_before(self, print_time):
        report = self.printer.lookup_object('motion_report', None)
        trapq = None if report is None else report.dtrapqs.get('toolhead')
        if trapq is None:
            return None
        pos, velocity = trapq.get_trapq_position(print_time - TIME_EPSILON)
        return None if pos is None else pos[2]

    def _handle_set_position(self):
        # The trapq gets a zero-length marker at the new position: Z may
        # jump.  That is not motion, so it is no clearance evidence, and Z
        # is no longer known to be where homing put it.
        self.clear_win = None
        self.own_error = None
        if self.toolhead is None:
            return
        print_time = self.toolhead.print_time
        self.position_reset_time = print_time
        if self.homing_rails or self.homing_moves:
            # Inside a homing or probing move: from the steppers' own motion
            return
        old_z = self._trapq_z_before(print_time)
        new_z = self.toolhead.get_position()[2]
        if old_z is not None and abs(new_z - old_z) < MOVE_EPSILON:
            # SET_KINEMATIC_POSITION SET_HOMED=... with Z unchanged
            return
        if self._override_active() and self.probe_since_reset:
            # Z set from the probe that just triggered
            self.probe_since_reset = False
            return
        self.z_trusted = self.probe_since_reset = False

    def _handle_manual_move(self, stepper):
        # FORCE_MOVE / STEPPER_BUZZ moved a stepper behind the toolhead's
        # back; if it drives Z, the toolhead Z no longer says where the
        # nozzle is
        self.own_error = None
        if stepper.is_active_axis('z'):
            self.z_trusted = self.probe_since_reset = False

    def _handle_z_motor(self, print_time, enabled):
        # Off, the axis can be moved by hand; samples either side of an
        # enable change are not comparable for clearance
        self.clear_win = None
        self.position_reset_time = max(self.position_reset_time, print_time)
        if not enabled:
            self.z_trusted = self.probe_since_reset = False

    def _handle_command_error(self):
        # A failed G28 does not send home_rails_end
        self.homing_moves = 0
        self.homing_rails = False
        if (self.own_error is not None
                and sys.exc_info()[1] is self.own_error):
            # The guard's own refusal, at whatever macro level: nothing
            # homed or probed since it was raised (any of that clears the
            # mark), so Z is as trusted as it was
            return
        # A homing or probing move that ended in an error proves nothing
        self.pending_zero = self.lift_zero = None
        self.z_trusted = self.probe_since_reset = False

    def _is_homing(self):
        if self.homing_moves or self.homing_rails:
            return True
        ho = self.homing_override
        return ho is not None and getattr(ho, 'in_script', False)

    def _is_streaming(self, eventtime):
        # A print streamed over the G-code pty (not virtual_sdcard) never
        # sets print_stats.  Its commands run with is_processing_data set.
        # Batch mode's input file also runs through GCodeIO; that is not a
        # job.  Prints sent line by line through the API have no marker.
        io = self.gcode_io
        if io is None or getattr(io, 'is_fileinput', False):
            return False
        if getattr(io, 'is_processing_data', False):
            self.stream_systime = eventtime
        return (self.stream_systime is not None
                and eventtime - self.stream_systime < STREAM_JOB_GRACE)

    def _job_active(self, eventtime):
        if self._is_streaming(eventtime):
            return True
        if self.print_stats is None:
            return False
        state = self.print_stats.get_status(eventtime)['state']
        return state in ('printing', 'paused')

    def _holdoff_end(self):
        # Samples before this are a tap's ring, or older than the trigger
        # and so taken during the probing move itself
        trig = self.probe.get_last_trigger_time()
        if not trig:
            return float('-inf')
        return trig + self.holdoff

    # The states in which the guard never refuses
    def _may_refuse(self, eventtime):
        if not self.enabled or self.probe is None:
            return False
        if self._is_homing() or self._job_active(eventtime):
            return False
        return not getattr(self.nozzle_wipe, 'active', False)

    def is_active(self, eventtime):
        if not self._may_refuse(eventtime):
            return False
        if (self.sample_systime is None
                or eventtime - self.sample_systime > STREAM_TIMEOUT):
            return False
        # Judged on sample time, not the clock: the decision can only be as
        # new as the data it rests on.
        return self.data_time >= self._holdoff_end()

    # Sample processing
    def _handle_batch(self, msg):
        samples = msg.get('data')
        if not samples:
            return True
        self.sample_systime = self.reactor.monotonic()
        anchor_only = False
        if self.homing_moves or self.homing_rails:
            ignore_before = float('inf')
        else:
            ignore_before = self._holdoff_end()
            # Between the moves of a homing_override script (the M1's tare
            # pauses before each probe) keep the readings as the anchor
            # for the next probe's re-zero, but do not judge contact
            anchor_only = self._is_homing()
        self._apply_pending_zero()
        self._update_motion_history(samples[0][0], samples[-1][0])
        sign = self.probe.get_press_direction(
            self.baseline if self.baseline is not None else samples[0][2])
        for sample in samples:
            self._process_sample(sample[0], sample[2], sign, ignore_before,
                                 anchor_only)
        return True

    def _update_motion_history(self, start, end):
        # Executed toolhead Z at each sample time, for positive clearance.
        # Samples and trapq share print_time.  Extract once per batch, not
        # once per 2 kHz sample, and keep only plain tuples.  This is the
        # toolhead trapq only: it says how far the toolhead moved, never
        # that nothing else moved (FORCE_MOVE has its own trapq), so it
        # only ever proves a load does not follow Z.
        self.motion_known = False
        self.motion_base_z = None
        self.motion_moves = []
        if not self.contact and abs(self.load) < self.band:
            return
        report = self.printer.lookup_object('motion_report', None)
        trapq = None if report is None else report.dtrapqs.get('toolhead')
        if trapq is None:
            return
        # trapq_extract_old skips a move starting exactly at the end of the
        # query, so a batch that begins at a reset's zero-length marker
        # would read the position from before it.  Query just past the
        # newest reset instead; samples up to it are never used.
        start = max(start, self.position_reset_time + TIME_EPSILON)
        pos, velocity = trapq.get_trapq_position(start)
        if pos is None:
            return
        moves, cdata = trapq.extract_trapq(start, end)
        self.motion_known = True
        self.motion_base_z = pos[2]
        self.motion_moves = sorted(
            (m.print_time, m.move_t, m.start_v, m.accel, m.start_z, m.z_r)
            for m in moves)

    def _z_at(self, t):
        if not self.motion_known:
            return None
        z = self.motion_base_z
        for print_time, move_t, start_v, accel, start_z, z_r in \
                self.motion_moves:
            if print_time > t:
                break
            move_time = max(0., min(move_t, t - print_time))
            z = start_z + z_r * (start_v + .5 * accel * move_time) * move_time
        return z

    def _reset_detector(self):
        self.contact = False
        self.above_since = self.below_since = None
        self.drift_since = None
        self.drift_sum = 0.
        self.drift_count = 0
        self.drift_min = self.drift_max = None
        self.clear_win = None

    def _absorb(self, new, sign):
        # Drift following and steady relocks.  Toward less pressed is free
        # and gives back what was absorbed; toward more pressed is limited
        # to max_absorbed_counts in total since the last real zero, so the
        # excess reads as load and, past the threshold, as contact.
        step = sign * (new - self.baseline)
        if step > 0.:
            step = min(step, max(0., self.max_absorbed - self.absorbed))
            self.absorbed += step
        else:
            self.absorbed = max(0., self.absorbed + step)
        self.baseline += sign * step

    def _out_of_range(self, counts, sign):
        # Readings the cell cannot vouch for (see RANGE_FRACTION)
        rng = getattr(self.load_cell.sensor, 'get_range', None)
        if rng is not None:
            lo, hi = rng()
            margin = (hi - lo) * RAIL_FRACTION
            if counts <= lo + margin or counts >= hi - margin:
                return True
        get_ref = getattr(self.probe, 'get_reference_max_load_counts', None)
        ref_max = get_ref() if get_ref is not None else None
        if ref_max is None:
            return False
        span = sign * (ref_max - self.baseline)
        return span > 0. and sign * (counts - self.baseline) >= (
            RANGE_FRACTION * span)

    def _new_window(self, z, counts, t=None):
        w = {'zmin': z, 'zmax': z, 'rise': 0., 'min': counts, 'max': counts,
             'sum': 0., 'n': 0, 't': t}
        if t is not None:
            w['sum'], w['n'] = float(counts), 1
        return w

    def _check_clearance(self, t, counts, load, sign):
        # Called for each sample that is contact or outside the band.  True
        # once the load has held steady across clearance_lift of executed Z
        # travel and the baseline was re-zeroed on it.
        z = self._z_at(t) if t > self.position_reset_time else None
        if z is None or self._out_of_range(counts, sign):
            # Without executed Z there is no evidence of clearance, a
            # position reset separates two frames that cannot be compared,
            # and a cell at the end of its range reads flat whatever
            # happens: a crash would look steady
            self.clear_win = None
            return False
        # Two staggered windows: a second one starts once the first has
        # used half the band, and takes over when the first breaks.  Slow
        # drift then never leaves a window younger than half the band's
        # worth of drift, so a lift is not cut short by a restart.
        wins = self.clear_win
        if wins and t - wins[0]['t'] > CLEAR_MAX_GAP:
            # Samples are missing: the Z travel was not watched
            wins = None
        if not wins:
            wins = self.clear_win = [self._new_window(z, counts)]
        for w in wins:
            w['t'] = t
            w['min'] = min(w['min'], counts)
            w['max'] = max(w['max'], counts)
            w['zmin'] = min(w['zmin'], z)
            w['zmax'] = max(w['zmax'], z)
            w['rise'] = max(w['rise'], z - w['zmin'])
            w['sum'] += counts
            w['n'] += 1
        while wins and wins[0]['max'] - wins[0]['min'] > self.band:
            # The load moved: the oldest window is no longer steady
            wins.pop(0)
        if not wins:
            wins = self.clear_win = [self._new_window(z, counts, t)]
        if (len(wins) == 1
                and wins[0]['max'] - wins[0]['min'] > self.band // 2):
            wins.append(self._new_window(z, counts, t))
        win = wins[0]
        if self.contact:
            # Latched: only lifting proves clearance.  Pressing on down
            # with the reading unchanged means the reading stopped
            # measuring, not that nothing is there.
            travel = win['rise']
        else:
            travel = win['zmax'] - win['zmin']
        if travel < self.clearance_lift - MOVE_EPSILON:
            return False
        self.baseline = win['sum'] / win['n']
        self._real_zero()
        logging.info("contact_guard: load %d held steady over %.3f mm of Z;"
                     " re-zeroed", load, travel)
        return True

    def _process_sample(self, t, counts, sign, ignore_before,
                        anchor_only=False):
        if self.data_time is None or t > self.data_time:
            self.data_time = t
        if t < ignore_before:
            # Probing taps and their ring are not contact or drift
            self.in_gap = True
            self.recent.clear()
            return
        if self.in_gap:
            # Continuity is broken; the nozzle may have moved
            self._reset_detector()
            self.recent.clear()
            self.prev_time = None
            self.in_gap = False
            self.resume_time = t
        self.recent.append((t, counts))
        while self.recent and self.recent[0][0] < t - ANCHOR_TIME:
            self.recent.popleft()
        if anchor_only:
            return
        if self.baseline is None:
            self.baseline = float(counts)
        if (self.lift_zero is not None and t > self.lift_zero['after']
                and self._lift_zero_sample(t, counts, sign)):
            self.load = sign * (counts - self.baseline)
            return
        dt = 0. if self.prev_time is None else max(0., t - self.prev_time)
        self.prev_time = t
        load = sign * (counts - self.baseline)
        self.load = load
        # Contact hysteresis
        if load > self.threshold:
            if self.above_since is None:
                self.above_since = t
        else:
            self.above_since = None
        if load < self.threshold * .5:
            if self.below_since is None:
                self.below_since = t
        else:
            self.below_since = None
        if not self.contact:
            if (self.above_since is not None
                    and t - self.above_since >= self.on_time - TIME_EPSILON):
                self.contact = True
                logging.info("contact_guard: contact, load %d", load)
        elif (self.below_since is not None
                and t - self.below_since >= self.off_time - TIME_EPSILON):
            self.contact = False
            logging.info("contact_guard: contact released, load %d", load)
        if ((self.contact or abs(load) >= self.band)
                and self._check_clearance(t, counts, load, sign)):
            self.load = sign * (counts - self.baseline)
            return
        if abs(load) < self.band:
            self.clear_win = None
        if self.contact or self.above_since is not None:
            # The baseline never follows a touch, however long or still it
            # is.  Only release, positive clearance (steady over Z travel),
            # a Z homing or probing move that proves it began clear, or
            # SET_CONTACT_GUARD REZERO=1 ends it.
            self.drift_since = None
            return
        # Baseline: follow small changes slowly
        if abs(load) < self.band:
            self._absorb(self.baseline + (counts - self.baseline)
                         * min(1., dt / self.tau), sign)
            self.drift_since = None
            return
        # Require a steady window, not merely time outside the band. An
        # unlock retract may change filament tension in several steps.
        if (self.drift_since is None
                or counts < self.drift_max - self.band
                or counts > self.drift_min + self.band):
            self.drift_since = t
            self.drift_sum = 0.
            self.drift_count = 0
            self.drift_min = self.drift_max = counts
        self.drift_min = min(self.drift_min, counts)
        self.drift_max = max(self.drift_max, counts)
        self.drift_sum += counts
        self.drift_count += 1
        if t - self.drift_since >= self.relock_time - TIME_EPSILON:
            old, win = self.baseline, self.clear_win
            self._absorb(self.drift_sum / self.drift_count, sign)
            self._reset_detector()
            # A Z move in progress may still prove the offset is not a touch
            self.clear_win = win
            self.load = sign * (counts - self.baseline)
            if self.baseline != old:
                logging.info("contact_guard: baseline relocked, load was %d,"
                             " now %d", load, self.load)

    def get_status(self, eventtime):
        active = self.is_active(eventtime)
        return {
            'contact': active and self.contact,
            'load': int(round(self.load)),
            'active': active,
            'enabled': self.enabled,
        }

    def _rezero(self, force=False):
        # The explicit, bounded recovery: an operator or macro asserts that
        # the steady load the cell reads now is not contact.  Bounded by
        # Z homed and high above the plate, settled samples, and a load
        # that is not changing; otherwise refused, and contact stays.
        if self._is_homing():
            raise self._own_error("contact_guard: cannot re-zero while"
                                   " homing")
        eventtime = self.reactor.monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(eventtime)
        if 'z' not in kin_status['homed_axes'] or not self.z_trusted:
            # SET_KINEMATIC_POSITION, FORCE_MOVE and a Z motor turned off
            # all leave a toolhead Z that says nothing about the nozzle
            raise self._own_error(
                "contact_guard: re-zero needs Z homed by G28 since the last"
                " position reset, FORCE_MOVE or Z motor off")
        z = self.toolhead.get_position()[2]
        if z < self.rezero_min_z:
            raise self._own_error(
                "contact_guard: re-zero needs the toolhead at Z %.1f or"
                " higher (it is at %.2f)" % (self.rezero_min_z, z))
        # Judge the load after everything queued has executed
        self._sync()
        eventtime = self.reactor.monotonic()
        if (not self.recent or self.sample_systime is None
                or eventtime - self.sample_systime > STREAM_TIMEOUT
                or self.data_time < self._holdoff_end()):
            raise self._own_error("contact_guard: re-zero needs current"
                                   " load cell samples")
        counts = [c for t, c in self.recent]
        if max(counts) - min(counts) > self.band:
            raise self._own_error("contact_guard: load is changing, not"
                                   " re-zeroing")
        sign = self.probe.get_press_direction(self.baseline)
        if any(self._out_of_range(c, sign) for c in counts):
            # Flat at the end of the cell's range: no reading at all
            raise self._own_error(
                "contact_guard: the load cell is at the end of its range,"
                " not re-zeroing")
        mean = float(sum(counts)) / len(counts)
        if (self.contact or sign * (mean - self.baseline) >= self.threshold):
            # Toolhead Z says nothing about a part or fixture under the
            # nozzle; a load that does not follow a 1 mm lift does
            if not force:
                raise self._own_error(
                    "contact_guard: the load is at contact level; lift Z"
                    " 1 mm to clear an offset that does not follow Z, or"
                    " REZERO=1 FORCE=1 if nothing is touching")
            logging.warning("contact_guard: REZERO FORCE=1 absorbing a"
                            " contact-level load of %d at Z %.2f",
                            sign * (mean - self.baseline), z)
        old = self.load
        self.baseline = mean
        self._real_zero()
        self.load = 0.
        logging.info("contact_guard: re-zeroed by command at Z %.2f, load"
                     " was %d", z, old)
        return old

    cmd_SET_CONTACT_GUARD_help = (
        "Enable or disable refusing XY moves while the nozzle touches;"
        " REZERO=1 [FORCE=1] re-zeroes a steady offset with the toolhead"
        " high")
    def cmd_SET_CONTACT_GUARD(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        if enable is not None:
            self.enabled = bool(enable)
        if gcmd.get_int('REZERO', 0, minval=0, maxval=1):
            old = self._rezero(
                force=bool(gcmd.get_int('FORCE', 0, minval=0, maxval=1)))
            gcmd.respond_info("contact_guard: re-zeroed, load was %d" % old)
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info("contact_guard: enabled=%d active=%d contact=%d"
                          " load=%d" % (status['enabled'], status['active'],
                                        status['contact'], status['load']))

    def cmd_CONTACT_GUARD_SIMULATE(self, gcmd):
        # Feed synthetic load cell samples through the same client callback
        # the sensor stream uses.  LOAD is counts in the pressing direction.
        # PLATE_Z=<z> STIFFNESS=<counts/mm> instead models a plate at that
        # toolhead Z: samples then follow the executed motion in the trapq,
        # and are generated whenever the guard waits for them.
        plate_z = gcmd.get_float('PLATE_Z', None)
        if plate_z is not None:
            self.sim_plate = (plate_z, gcmd.get_float('STIFFNESS', above=0.))
            return
        if gcmd.get_int('PLATE', 1) == 0:
            self.sim_plate = None
            return
        load = gcmd.get_float('LOAD', 0.)
        duration = gcmd.get_float('DURATION', above=0.)
        rate = gcmd.get_float('RATE', 2000., above=0.)
        drift = gcmd.get_float('DRIFT', 0.)
        after_trigger = gcmd.get_float('AFTER_TRIGGER', None)
        # Samples are never older than the motion already queued, and the
        # toolhead dwells through them below, so the data can never run
        # ahead of the motion it would be measuring.
        move_time = self.toolhead.get_last_move_time()
        if after_trigger is not None:
            start = self.probe.get_last_trigger_time() + after_trigger
        else:
            start = max(self.sim_time or 0., move_time)
        sign = self.probe.get_press_direction(SIM_ZERO_COUNTS)
        count = int(round(duration * rate))
        rows = []
        for i in range(count):
            t = start + i / rate
            # DRIFT moves the raw zero, continuously across commands
            self.sim_drift += drift / rate
            counts = SIM_ZERO_COUNTS + self.sim_drift + sign * load
            rows.append([t, None, int(round(counts)), None])
        self.sim_time = start + count / rate
        self._sim_feed(rows, rate)
        if self.sim_time > move_time:
            self.toolhead.dwell(self.sim_time - move_time)

    def _sim_feed(self, rows, rate):
        batch = int(rate * .020) or 1
        for i in range(0, len(rows), batch):
            self._handle_batch({'data': rows[i:i + batch], 'errors': 0,
                                'overflows': 0})

    def _sim_plate_until(self, target, rate=2000.):
        # The last 0.3 s up to target, from where the trapq says the
        # toolhead was at each sample time
        plate_z, stiffness = self.sim_plate
        motion_report = self.printer.lookup_object('motion_report')
        dtrapq = motion_report.dtrapqs['toolhead']
        start = target - 0.3
        if self.data_time is not None:
            start = max(start, self.data_time + 1. / rate)
        # Earlier LOAD samples may already run past target; positions after
        # the last move are its end position, so continue from there
        end = max(target, start + 0.1)
        sign = self.probe.get_press_direction(SIM_ZERO_COUNTS)
        rows = []
        for i in range(int((end - start) * rate) + 1):
            t = start + i / rate
            pos, velocity = dtrapq.get_trapq_position(t)
            z = pos[2] if pos is not None else self.toolhead.get_position()[2]
            load = max(0., plate_z - z) * stiffness
            counts = SIM_ZERO_COUNTS + self.sim_drift + sign * load
            rows.append([t, None, int(round(counts)), None])
        if rows:
            self.sim_time = rows[-1][0] + 1. / rate
        self._sim_feed(rows, rate)


def load_config(config):
    return ContactGuard(config)
