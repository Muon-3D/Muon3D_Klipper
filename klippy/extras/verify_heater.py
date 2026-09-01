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
        self.approaching_target = self.starting_approach = False
        self.last_target = self.goal_temp = self.error = 0.
        self.goal_systime = self.printer.get_reactor().NEVER
        self.check_timer = None
        self.suspend_systime = None
        self.recovery_goal = None
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
        if self.recovery_goal is not None and temp >= self.recovery_goal:
            # The heater has climbed heating_gain since the last credit, so it
            # is working. Clear the error it accrued re-heating what the outage
            # cost it -- that deviation is the outage's doing, not the
            # heater's -- and move the bar up, so the credit has to be earned
            # again for the next stretch of the ramp.
            #
            # This deliberately grants progress, not time. Re-arming the
            # approach state instead would hand out a fresh check_gain_time on
            # every reconnect, which is the deferral this ticket exists to
            # remove; a heater that stops climbing stops earning credit here
            # and its error accumulates to max_error as before.
            self.error = 0.
            self.recovery_goal = temp + self.heating_gain
        if temp >= target - self.hysteresis or target <= 0.:
            # Temperature near target - reset checks
            if self.approaching_target and target:
                logging.info("Heater %s within range of %.3f",
                             self.heater_name, target)
            self.approaching_target = self.starting_approach = False
            self.recovery_goal = None
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
    # These pause the check across a non-critical MCU outage; they do not
    # restart it. The distinction matters on the M1, where the extruder
    # heater is on the hot-swappable toolhead MCU, behind a connector the
    # product invites the user to unplug. Both used to clear self.error and
    # self.goal_systime, so every disconnect/reconnect threw away the
    # evidence gathered so far, and an intermittent umbilical that flapped
    # faster than check_gain_time held the thermal-runaway check permanently
    # disarmed. See KAN-99.
    #
    # What makes carrying the error safe is not the zero-target branch in
    # check_event -- that only clears the error when temp is also within
    # hysteresis of zero, so a 250 C nozzle keeps it across TOOLHEAD_CONNECT's
    # M104 S0. It is that the error is cleared by evidence of the heater
    # working: reaching target, gaining heating_gain during an approach, or
    # recovery_goal below. A heater that shows none of those is exactly the
    # one whose history is worth keeping.
    def _suspend_checks(self):
        r = self.printer.get_reactor()
        if self.check_timer is not None:
            r.update_timer(self.check_timer, r.NEVER)
        # The timer stops here, so self.error cannot grow while the MCU is
        # offline; only observed time is ever charged to a heater. First
        # suspend wins, so a repeated event cannot shorten the outage the
        # resume below measures.
        if self.suspend_systime is None:
            self.suspend_systime = r.monotonic()
        logging.info("verify_heater[%s]: suspended (MCU offline)",
                     self.heater_name)

    def _resume_checks(self):
        r = self.printer.get_reactor()
        if self.printer.is_shutdown():
            # handle_shutdown disarmed the timer deliberately. Re-arming it
            # here would let a carried error fault an already-shut-down
            # printer.
            self.suspend_systime = None
            return
        if self.suspend_systime is not None:
            # Push the gain deadline out by however long we were blind rather
            # than clearing it, so an approach that had 5 s left to show
            # heating_gain still has 5 s left and a reconnect cannot buy a
            # failing heater a fresh window every time the connector twitches.
            # Capped at one check_gain_time: a toolhead left unplugged for an
            # hour must not defer the verdict by an hour.
            outage = max(0., r.monotonic() - self.suspend_systime)
            if self.goal_systime != r.NEVER:
                self.goal_systime += min(outage, self.check_gain_time)
            self.suspend_systime = None
        if self.heater is not None:
            # The heater cooled with its power removed, and re-heating that
            # back is not a fault -- but check_event has no notion of an
            # approach unless the target changed, so it would charge the whole
            # recovery ramp as error and could shut the printer down on
            # healthy hardware. Record what "visibly recovering" looks like
            # from here; check_event clears the error once the heater gets
            # there.
            temp, _target = self.heater.get_temp(r.monotonic())
            self.recovery_goal = temp + self.heating_gain
        if self.check_timer is not None:
            r.update_timer(self.check_timer, r.NOW)
        logging.info("verify_heater[%s]: resumed (MCU reconnected)",
                     self.heater_name)

def load_config_prefix(config):
    return HeaterCheck(config)
