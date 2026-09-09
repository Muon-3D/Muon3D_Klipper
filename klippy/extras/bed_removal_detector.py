import logging
import collections

class BedRemovalDetector:
    def __init__(self, config):
        self.printer = config.get_printer()
        # thresholds
        self.remove_thr = config.getfloat('remove_threshold', -15.0)
        self.attach_thr = config.getfloat('attach_threshold', -5.0)
        # buffer settings
        buffer_len = config.getint('buffer_length', 5)
        self.window = collections.deque(maxlen=buffer_len)
        # check interval
        self.bed_check_interval = config.getfloat('interval', 0.1)

        # heater and gcode objects
        self.bed = self.printer.lookup_object('heater_bed')
        self.bed_heater = self.bed.heater
        self.gcode = self.printer.lookup_object('gcode')
        # load_object rather than lookup_object, as filament_switch_sensor
        # does for the same object: it returns [pause_resume] whatever order
        # the config sections happen to load in, and a failure here is a
        # config error at startup rather than an exception raised inside a
        # reactor timer, which klippy turns into a shutdown mid-print.
        self.pause_resume = self.printer.load_object(config, 'pause_resume')

        self.bed_removed = False
        # None means no removal is outstanding. Distinct from 0., which means
        # a removal happened while the bed was genuinely off.
        self.restore_target = None
        self.log = logging.getLogger(__name__)

        # register startup handler
        self.printer.register_event_handler('klippy:ready', self.handle_ready)

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.check_bed_temperature, reactor.NOW)
        self.log.info('BedRemovalDetector ready, checking every %.2f s', self.bed_check_interval)

    def check_bed_temperature(self, eventtime):
        raw_temp, _ = self.bed_heater.get_temp(eventtime)
        self.window.append(raw_temp)

        if (self.restore_target is not None and not self.bed_removed
                and not self.pause_resume.is_paused):
            # Plate is back and nothing is waiting to be resumed, so the
            # removal this target belongs to is over. Without this it outlives
            # its own print: cancel instead of resuming, then start a PLA job,
            # pause it with the bed off and pull the plate -- the target from
            # the ABS job before it is still what a resume would reheat to.
            self.restore_target = None

        # only evaluate once we have a full window
        if len(self.window) == self.window.maxlen:
            avg_temp = sum(self.window) / len(self.window)
            if not self.bed_removed and avg_temp < self.remove_thr:
                self.handle_bed_removal(eventtime)
            elif self.bed_removed and avg_temp > self.attach_thr:
                self.handle_bed_reconnection()

        return eventtime + self.bed_check_interval

    def _run_macro(self, eventtime, name):
        """Run a bed macro from its own greenlet.

        Registered as a reactor callback rather than called inline, because
        run_script takes the gcode mutex and can block for as long as whatever
        holds it. Pull the plate during PRINT_START and TEMPERATURE_WAIT is
        holding that mutex on a heater this module has just set to zero, so
        the wait can never be satisfied. Called inline that parks
        check_bed_temperature itself, and with it the sampling window and the
        bedRemoved flag HOMING_GATE reads. Deferring does not rescue the
        macros -- BED_ATTACHED queues behind the same mutex -- it keeps the
        module's own state live, which is what the rest of the config asks
        this module for. Same shape as filament_switch_sensor's runout
        handling, which defers for the same reason.

        The exception guard is that module's too. The heater is already off
        before either macro runs, so a macro that fails costs the operator the
        prompt, not the safety action.
        """
        try:
            self.gcode.run_script(name)
        except Exception:
            self.log.exception('%s failed', name)

    def handle_bed_removal(self, eventtime):
        self.bed_removed = True
        # Remember what the bed was asked for before zeroing it. Nothing else
        # records this, and without it a resume after reattaching continues
        # the print onto a cold plate -- which on ABS or PETG is the warping
        # the operator reattached the bed to avoid.
        _temp, target = self.bed_heater.get_temp(eventtime)
        if self.restore_target is None:
            # First removal of this episode. Pulling the plate, reattaching it
            # and pulling it again without resuming in between would otherwise
            # record the zero the next line set the first time round, and the
            # reheat on resume would silently do nothing.
            self.restore_target = target
        self.bed_heater.set_temp(0.0)
        # Stop feeding the file immediately, the way filament_switch_sensor
        # does on runout: the pause half of pause_resume has to take effect
        # now, not whenever the macro below wins the gcode mutex, because
        # what it is stopping is a toolhead extruding into open air. The
        # PAUSE in BED_REMOVED then completes the pause normally.
        if self._is_printing(eventtime):
            self.pause_resume.send_pause_command()
        reactor = self.printer.get_reactor()
        reactor.register_callback(
            lambda et: self._run_macro(et, 'BED_REMOVED'))
        self.log.info('Bed removal detected, bed heater off '
                      '(was %.1f C)', self.restore_target)

    def _is_printing(self, eventtime):
        idle_timeout = self.printer.lookup_object('idle_timeout')
        return idle_timeout.get_status(eventtime)['state'] == 'Printing'

    def handle_bed_reconnection(self):
        self.bed_removed = False
        reactor = self.printer.get_reactor()
        reactor.register_callback(
            lambda et: self._run_macro(et, 'BED_ATTACHED'))
        self.log.info('Bed attached, monitoring resumed')

    def get_status(self, eventtime):
        # restore_target is read by _BED_RESUME_NOW to put the bed back where
        # it was before resuming. The macro only compares it against zero, so
        # "no removal outstanding" is published as 0. rather than as None.
        return {'bedRemoved': self.bed_removed,
                'restoreTarget': self.restore_target or 0.}


def load_config(config):
    return BedRemovalDetector(config)
