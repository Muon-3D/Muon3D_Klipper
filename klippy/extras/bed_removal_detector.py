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
        # KAN-272: whether the latest removal came while a file was printing,
        # and so paused it. Latched at removal time for BED_REMOVED, which
        # renders later (behind the gcode mutex) and must not re-derive it
        # from a state the pause itself has already changed.
        self.removed_while_printing = False
        # print_stats.print_start_time of the print whose PRINT_END has begun,
        # or None. Keyed to that print, not a bare flag, so it cannot outlive
        # it: a stale mark would silently suppress the KAN-79 pause on the
        # next print. See _in_print_phase.
        self.print_ending_start = None
        self.print_stats = None
        self.virtual_sdcard = None
        # None means no removal is outstanding. Distinct from 0., which means
        # a removal happened while the bed was genuinely off.
        self.restore_target = None
        self.log = logging.getLogger(__name__)

        # register startup handler
        self.printer.register_event_handler('klippy:ready', self.handle_ready)
        self.gcode.register_command(
            'SET_BED_REMOVAL_DETECTOR', self.cmd_SET_BED_REMOVAL_DETECTOR,
            desc=self.cmd_SET_BED_REMOVAL_DETECTOR_help)

    def handle_ready(self):
        # Optional: without [virtual_sdcard] nothing is ever a file print, so
        # a plate pull only turns the heater off and runs BED_REMOVED.
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.virtual_sdcard = self.printer.lookup_object('virtual_sdcard',
                                                         None)
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.check_bed_temperature, reactor.NOW)
        self.log.info('BedRemovalDetector ready, checking every %.2f s',
                      self.bed_check_interval)

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
        self.removed_while_printing = self._in_print_phase(eventtime)
        if self.removed_while_printing:
            self.pause_resume.send_pause_command()
        reactor = self.printer.get_reactor()
        reactor.register_callback(
            lambda et: self._run_macro(et, 'BED_REMOVED'))
        self.log.info('Bed removal detected, bed heater off '
                      '(was %.1f C)', self.restore_target)

    def _in_print_phase(self, eventtime):
        """True while a file is printing and its PRINT_END has not begun.

        KAN-272. This used to read idle_timeout, which says "Printing" after
        any toolhead motion at all, not only while a file prints. On walnut
        on 2026-10-01 startup gcode made it read "Printing" with no file
        loaded, a plate pull paused the idle machine, and the pause carried
        into the next print, where every pause-based fault path then did
        nothing. It also missed a real print sitting in a long heat wait,
        where idle_timeout drops to "Ready".

        A file is printing when print_stats says "printing" and
        virtual_sdcard is reading it. Both, so that neither a stale state
        nor a pause in progress counts. "paused", "complete", "cancelled",
        "error" and "standby" are all outside the print phase: there is
        nothing left to stop.

        The PRINT_END tail is a line of the file, so print_stats still says
        "printing" through it. PRINT_END marks it with
        SET_BED_REMOVAL_DETECTOR PRINT_ENDING=1 after the heaters are off.
        The mark holds the print's start time and only counts while that is
        still the running print's, so the next print is a print phase again
        even if this timer never sampled the gap between them.
        """
        ps = self.print_stats
        vsd = self.virtual_sdcard
        if ps is None or vsd is None:
            return False
        if ps.get_status(eventtime)['state'] != 'printing':
            return False
        if not vsd.is_active():
            return False
        return not self._print_ending()

    def _print_ending(self):
        ps = self.print_stats
        start = self.print_ending_start
        return (ps is not None and start is not None
                and ps.print_start_time == start)

    cmd_SET_BED_REMOVAL_DETECTOR_help = (
        "PRINT_ENDING=1 marks the running print's PRINT_END tail, where a "
        "plate removal no longer pauses the file")

    def cmd_SET_BED_REMOVAL_DETECTOR(self, gcmd):
        ending = gcmd.get_int('PRINT_ENDING', None, minval=0, maxval=1)
        if ending is None:
            return
        ps = self.print_stats
        if not ending:
            self.print_ending_start = None
        elif ps is not None and ps.print_start_time is not None:
            # Outside a print (a console PRINT_END, or CANCEL_PRINT's, which
            # runs after print_stats has stopped) there is nothing to mark,
            # and nothing to pause either.
            self.print_ending_start = ps.print_start_time

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
        #
        # removedWhilePrinting is what BED_REMOVED reads to decide whether to
        # PAUSE and say the print is paused. printEnding is informational.
        return {'bedRemoved': self.bed_removed,
                'restoreTarget': self.restore_target or 0.,
                'removedWhilePrinting': self.removed_while_printing,
                'printEnding': self._print_ending()}


def load_config(config):
    return BedRemovalDetector(config)
