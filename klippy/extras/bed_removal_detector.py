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

    def handle_bed_removal(self, eventtime):
        self.bed_removed = True
        self.bed_heater.set_temp(0.0)
        self.gcode.respond_info('Bed Removed')

        idle_timeout = self.printer.lookup_object('idle_timeout')
        is_printing = idle_timeout.get_status(eventtime).get('state') == 'Printing'
        if is_printing:
            self.gcode.run_script('PAUSE')
        self.log.info('Bed removal detected, printer paused')

    def handle_bed_reconnection(self):
        self.bed_removed = False
        self.gcode.respond_info('Bed Reattached')
        self.log.info('Bed reattached, monitoring resumed')

    def get_status(self, eventtime):
        return {'bedRemoved': self.bed_removed}


def load_config(config):
    return BedRemovalDetector(config)
