# Heater/sensor verification code
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

HINT_THERMAL = """
See the 'verify_heater' section in docs/Config_Reference.md
for the parameters that control this check.
"""

class HeaterCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)
        self.heater_name = config.get_name().split()[1]
        self.heater = None
        self.mcu = None
        self.hysteresis = config.getfloat('hysteresis', 5., minval=0.)
        self.max_error = config.getfloat('max_error', 120., minval=0.)
        self.heating_gain = config.getfloat('heating_gain', 2., above=0.)
        default_gain_time = 20.
        if self.heater_name == 'heater_bed':
            default_gain_time = 60.
        self.check_gain_time = config.getfloat(
            'check_gain_time', default_gain_time, minval=1.)
        # KAN-99: how long an outage has to last before the accumulated
        # error is forgotten. Below this the error is carried across the
        # gap, decayed in proportion, so a link that keeps dropping cannot
        # hold the fault off indefinitely. Above it the hot end has had time
        # to cool and the old error no longer describes the hardware.
        self.reconnect_decay_time = config.getfloat(
            'reconnect_decay_time', 120., above=0.)
        self.approaching_target = self.starting_approach = False
        self.last_target = self.goal_temp = self.error = 0.
        self.goal_systime = self.printer.get_reactor().NEVER
        self.suspended_systime = None
        self.check_timer = None
    def handle_connect(self):
        if self.printer.get_start_args().get('debugoutput') is not None:
            # Disable verify_heater if outputting to a debug file
            return
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        try:
            self.mcu = self.heater.mcu_pwm.get_mcu()
        except Exception:
             self.mcu = None
        logging.info("Starting heater checks for %s", self.heater_name)
        reactor = self.printer.get_reactor()
        self.check_timer = reactor.register_timer(self.check_event, reactor.NOW)


        # If the heater lives on a non-critical MCU, subscribe to its events
        if self.mcu is not None and getattr(self.mcu, "is_non_critical", False):
            self.printer.register_event_handler(
                self.mcu.get_non_critical_disconnect_event_name(),
                self._suspend_checks)
            self.printer.register_event_handler(
                self.mcu.get_non_critical_reconnect_event_name(),
                self._resume_checks)

    def handle_shutdown(self):
        if self.check_timer is not None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self.check_timer, reactor.NEVER)
    def check_event(self, eventtime):
        temp, target = self.heater.get_temp(eventtime)
        if temp >= target - self.hysteresis or target <= 0.:
            # Temperature near target - reset checks
            if self.approaching_target and target:
                logging.info("Heater %s within range of %.3f",
                             self.heater_name, target)
            self.approaching_target = self.starting_approach = False
            if temp <= target + self.hysteresis:
                self.error = 0.
            self.last_target = target
            return eventtime + 1.
        self.error += (target - self.hysteresis) - temp
        if not self.approaching_target:
            if target != self.last_target:
                # Target changed - reset checks
                logging.info("Heater %s approaching new target of %.3f",
                             self.heater_name, target)
                self.approaching_target = self.starting_approach = True
                self.goal_temp = temp + self.heating_gain
                self.goal_systime = eventtime + self.check_gain_time
            elif self.error >= self.max_error:
                # Failure due to inability to maintain target temperature
                return self.heater_fault()
        elif temp >= self.goal_temp:
            # Temperature approaching target - reset checks
            self.starting_approach = False
            self.error = 0.
            self.goal_temp = temp + self.heating_gain
            self.goal_systime = eventtime + self.check_gain_time
        elif eventtime >= self.goal_systime:
            # Temperature is no longer approaching target
            self.approaching_target = False
            logging.info("Heater %s no longer approaching target %.3f",
                         self.heater_name, target)
        elif self.starting_approach:
            self.goal_temp = min(self.goal_temp, temp + self.heating_gain)
        self.last_target = target
        return eventtime + 1.
    def heater_fault(self):
        msg = "Heater %s not heating at expected rate" % (self.heater_name,)
        logging.error(msg)
        self.printer.invoke_shutdown(msg + HINT_THERMAL)
        return self.printer.get_reactor().NEVER


    ### Non Crit MCU Buisness
    #
    # The extruder heater sits on the hot-swappable toolhead MCU, so these
    # checks have to stand down while that MCU is away. Both handlers used to
    # zero self.error on the way through (KAN-99). The M1 has a user-operated
    # umbilical release, so a worn connector produces exactly the pattern that
    # defeats: every drop reset the accumulator, and a hot end genuinely
    # failing to heat never reached max_error to trip heater_fault().
    #
    # The error is now carried across the outage and decayed by how long it
    # lasted. A brief flap keeps nearly all of it, so a real fault still
    # trips; an outage past reconnect_decay_time starts clean, so replugging
    # a toolhead after a genuine absence does not fault on stale error.
    #
    # Deliberately NOT added: a shutdown after N reconnects in a window. Once
    # the accumulator survives flapping, a failing heater is caught by the
    # mechanism designed to catch it, and a reconnect counter would only add
    # a new way to halt a healthy print on a connector fault.
    def _suspend_checks(self):
        r = self.printer.get_reactor()
        if self.check_timer is not None:
            r.update_timer(self.check_timer, r.NEVER)
        self.approaching_target = self.starting_approach = False
        self.suspended_systime = r.monotonic()
        self.goal_systime = r.NEVER
        logging.info("verify_heater[%s]: suspended (MCU offline), holding"
                     " error %.1f", self.heater_name, self.error)

    def _resume_checks(self):
        r = self.printer.get_reactor()
        if self.suspended_systime is not None:
            outage = max(0., r.monotonic() - self.suspended_systime)
            self.suspended_systime = None
            if outage >= self.reconnect_decay_time:
                self.error = 0.
            else:
                self.error *= 1. - (outage / self.reconnect_decay_time)
        self.approaching_target = self.starting_approach = False
        self.goal_systime = r.NEVER
        if self.check_timer is not None:
            r.update_timer(self.check_timer, r.NOW)
        logging.info("verify_heater[%s]: resumed (MCU reconnected), error"
                     " %.1f", self.heater_name, self.error)

def load_config_prefix(config):
    return HeaterCheck(config)
