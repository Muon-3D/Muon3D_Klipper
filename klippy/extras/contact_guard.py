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
import logging, collections

REFUSAL = "Nozzle is touching the plate: lift Z before moving X or Y"

# With no load cell samples for this long the guard goes inactive rather
# than act on a stale reading (toolhead MCU disconnected, stream stopped).
STREAM_TIMEOUT = 1.0
# Samples averaged to re-zero the baseline at the start of a homing move.
ANCHOR_TIME = 0.020
# Below this a coordinate has not moved (G-code positions are floats).
MOVE_EPSILON = 1e-6
# Sample times are floats; "20 ms" must include the sample at 20 ms.
TIME_EPSILON = 1e-6
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
        # A load that sits outside the band but never becomes contact for
        # this long is drift, not a touch, and the baseline jumps to it.
        self.relock_time = config.getfloat('relock_time', 0.5, above=0.)
        self.enabled = True
        # Objects found at connect
        self.probe = self.load_cell = self.print_stats = None
        self.homing_override = self.nozzle_wipe = None
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
        self.recent = collections.deque()
        self.sim_time = None
        self.sim_drift = 0.
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
        if not self.contact:
            return
        eventtime = self.reactor.monotonic()
        if not self.is_active(eventtime):
            return
        last = self.last_position
        moves_xy = (abs(newpos[0] - last[0]) > MOVE_EPSILON
                    or abs(newpos[1] - last[1]) > MOVE_EPSILON)
        raises_z = newpos[2] > last[2] + MOVE_EPSILON
        if moves_xy and not raises_z:
            logging.info("contact_guard: refused move %s -> %s, load %d",
                         last[:3], list(newpos[:3]), self.load)
            raise self.gcode.error(REFUSAL)

    # Inactive states
    def _handle_homing_move_begin(self, hmove):
        self.homing_moves += 1
        # Every homing or probing move starts with the nozzle off the plate
        # (a probe that starts on it fails "triggered prior to movement"),
        # so re-zero on what the cell read just before it.  This is what
        # clears a baseline that a long contact let drift.
        if self.recent:
            self.baseline = (sum(c for t, c in self.recent)
                             / len(self.recent))
            self._reset_detector()

    def _handle_homing_move_end(self, hmove):
        self.homing_moves = max(0, self.homing_moves - 1)

    def _handle_home_rails_begin(self, homing_state, rails):
        self.homing_rails = True

    def _handle_home_rails_end(self, homing_state, rails):
        self.homing_rails = False

    def _handle_command_error(self):
        # A failed G28 does not send home_rails_end
        self.homing_moves = 0
        self.homing_rails = False

    def _is_homing(self):
        if self.homing_moves or self.homing_rails:
            return True
        ho = self.homing_override
        return ho is not None and getattr(ho, 'in_script', False)

    def _is_printing(self, eventtime):
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

    def is_active(self, eventtime):
        if not self.enabled or self.probe is None:
            return False
        if (self.sample_systime is None
                or eventtime - self.sample_systime > STREAM_TIMEOUT):
            return False
        if self._is_homing() or self._is_printing(eventtime):
            return False
        if getattr(self.nozzle_wipe, 'active', False):
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
        if self._is_homing():
            ignore_before = float('inf')
        else:
            ignore_before = self._holdoff_end()
        sign = self.probe.get_press_direction(
            self.baseline if self.baseline is not None else samples[0][2])
        for sample in samples:
            self._process_sample(sample[0], sample[2], sign, ignore_before)
        return True

    def _reset_detector(self):
        self.contact = False
        self.above_since = self.below_since = None
        self.drift_since = None
        self.drift_sum = 0.
        self.drift_count = 0

    def _process_sample(self, t, counts, sign, ignore_before):
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
        self.recent.append((t, counts))
        while self.recent and self.recent[0][0] < t - ANCHOR_TIME:
            self.recent.popleft()
        if self.baseline is None:
            self.baseline = float(counts)
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
                self.drift_since = None
                logging.info("contact_guard: contact, load %d", load)
        elif (self.below_since is not None
                and t - self.below_since >= self.off_time - TIME_EPSILON):
            self.contact = False
            logging.info("contact_guard: contact released, load %d", load)
        if self.contact or self.above_since is not None:
            # The baseline never follows a touch
            self.drift_since = None
            return
        # Baseline: follow small changes slowly
        if abs(load) < self.band:
            self.baseline += (counts - self.baseline) * min(1., dt / self.tau)
            self.drift_since = None
            return
        # Outside the band and not a contact: drift that outran the
        # baseline, or a zero taken under load that is now off.  Contact
        # is a step that crosses the threshold within milliseconds, so a
        # load that lingers here is not one.
        if self.drift_since is None:
            self.drift_since = t
            self.drift_sum = 0.
            self.drift_count = 0
        self.drift_sum += counts
        self.drift_count += 1
        if t - self.drift_since >= self.relock_time - TIME_EPSILON:
            self.baseline = self.drift_sum / self.drift_count
            self.drift_since = None
            logging.info("contact_guard: baseline relocked, load was %d",
                         load)

    def get_status(self, eventtime):
        active = self.is_active(eventtime)
        return {
            'contact': active and self.contact,
            'load': int(round(self.load)),
            'active': active,
            'enabled': self.enabled,
        }

    cmd_SET_CONTACT_GUARD_help = (
        "Enable or disable refusing XY moves while the nozzle touches")
    def cmd_SET_CONTACT_GUARD(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        if enable is not None:
            self.enabled = bool(enable)
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info("contact_guard: enabled=%d active=%d contact=%d"
                          " load=%d" % (status['enabled'], status['active'],
                                        status['contact'], status['load']))

    def cmd_CONTACT_GUARD_SIMULATE(self, gcmd):
        # Feed synthetic load cell samples through the same client callback
        # the sensor stream uses.  LOAD is counts in the pressing direction.
        load = gcmd.get_float('LOAD', 0.)
        duration = gcmd.get_float('DURATION', above=0.)
        rate = gcmd.get_float('RATE', 2000., above=0.)
        drift = gcmd.get_float('DRIFT', 0.)
        after_trigger = gcmd.get_float('AFTER_TRIGGER', None)
        if after_trigger is not None:
            start = self.probe.get_last_trigger_time() + after_trigger
        elif self.sim_time is not None:
            start = self.sim_time
        else:
            start = 0.
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
        batch = int(rate * .020) or 1
        for i in range(0, len(rows), batch):
            self._handle_batch({'data': rows[i:i + batch], 'errors': 0,
                                'overflows': 0})


def load_config(config):
    return ContactGuard(config)
