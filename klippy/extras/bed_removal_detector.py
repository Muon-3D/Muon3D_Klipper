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

        self.bed_removed = False
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

        # only evaluate once we have a full window
        if len(self.window) == self.window.maxlen:
            avg_temp = sum(self.window) / len(self.window)
            if not self.bed_removed and avg_temp < self.remove_thr:
                self.handle_bed_removal(eventtime)
            elif self.bed_removed and avg_temp > self.attach_thr:
                self.handle_bed_reconnection()

        return eventtime + self.bed_check_interval

    def _is_printing(self):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is not None:
            return ps.get_status(None).get('state') == 'printing'
        idle = self.printer.lookup_object('idle_timeout', None)
        if idle is not None:
            return idle.get_status(None).get('state') == 'Printing'
        return False

    def _is_paused(self):
        pr = self.printer.lookup_object('pause_resume', None)
        return pr is not None and pr.is_paused

    def handle_bed_removal(self, eventtime):
        self.bed_removed = True
        # KAN-79: pause here rather than from the BED_REMOVED macro. The
        # macro's PAUSE had been commented out while its own prompt told the
        # user the print was paused, so pulling the plate mid-print left the
        # toolhead running the file over an open carriage. A safety action
        # belongs next to the detection, in code, not in a config file that
        # a calibration overlay or a stray edit can silently neuter.
        if self._is_printing() and not self._is_paused():
            try:
                self.gcode.run_script('PAUSE')
            except Exception:
                # Never let a pause failure skip the heater cut below.
                self.log.exception('Bed removed: PAUSE failed')
        self.bed_heater.set_temp(0.0)
        self.gcode.run_script('BED_REMOVED')
        self.log.info('Bed removal detected (printing=%s, paused=%s)',
                      self._is_printing(), self._is_paused())

    def handle_bed_reconnection(self):
        self.bed_removed = False
        #self.gcode.respond_info('Bed Reattached')
        self.gcode.run_script('BED_ATTACHED')
        self.log.info('Bed attached, monitoring resumed')

    def get_status(self, eventtime):
        return {'bedRemoved': self.bed_removed}


def load_config(config):
    return BedRemovalDetector(config)
