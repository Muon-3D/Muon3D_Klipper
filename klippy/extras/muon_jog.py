# Streaming jog: the toolhead or the plate follows a live drag
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Why a plain G1 stream cannot do this. Klipper plans motion ahead and a
# queued move cannot be taken back (only M112 stops it). A client that
# streams G1 targets gets up to BUFFER_TIME_HIGH (1 s) of motion gathered in
# the lookahead before the first step (each new move pushes the priming
# flush out by PRIMING_CMD_TIME), and every queued millimetre still runs
# before a change of direction can take effect.
#
# What this does instead. MUON_JOG only sets a target. A reactor timer
# feeds the toolhead itself, one short batch at a time, and commits each
# batch at once (toolhead.get_last_move_time), so each batch is planned to
# come to rest at its end and nothing waits in the lookahead. A new batch is
# added only while less than lead_time of motion is queued AND the queued
# path stays within max_ahead. So at any moment the toolhead is never more
# than max_ahead of path (and lead_time + batch_time of time) from where it
# will stop: a STOP, a new target the other way, a lost client (no update
# for keepalive) or any refusal takes effect within that bound. Nothing is
# ever flushed or drained (no flush_step_generation, no M400), so the jog
# never stalls the queue it is feeding.
#
# Safety. Every batch goes through gcode_move.move_with_transform, so the
# contact guard (an XY move while the nozzle presses the plate is refused)
# and the bed mesh apply as for a G1, and the kinematics check the axis
# limits. Before that, targets are clamped to the axis limits and z_min,
# and below the notch height (bed_mesh no_go_traverse_z) an XY jog stops at
# the edge of a bed_mesh no_go_region instead of entering it; a Z jog over
# the notch stops at the notch height. Any error ends the jog (fail closed):
# the queued batches still run (at most max_ahead), nothing more is sent.
import logging, math

EPS = 1e-6
# A batch shorter than this is not worth a trapezoid of its own
MIN_SEGMENT = 0.01


class MuonJog:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.tick_time = config.getfloat('tick_time', 0.025, above=0.,
                                         maxval=0.1)
        # lead_time must be above the toolhead's BUFFER_TIME_START (0.25 s)
        # or every batch starts after a gap
        self.lead_time = config.getfloat('lead_time', 0.3, above=0.,
                                         maxval=1.)
        self.batch_time = config.getfloat('batch_time', 0.1, above=0.,
                                          maxval=0.5)
        self.max_ahead = {
            'XY': config.getfloat('max_ahead_xy', 15., above=0.),
            'Z': config.getfloat('max_ahead_z', 4., above=0.)}
        self.max_speed = {
            'XY': config.getfloat('max_xy_speed', 100., above=0.),
            'Z': config.getfloat('max_z_speed', 15., above=0.)}
        self.keepalive = config.getfloat('keepalive', 0.5, above=0.,
                                         maxval=2.)
        self.z_min = config.getfloat('z_min', 0.)
        self.regions, self.traverse_z = self._read_notch(config)
        self.session = None
        self.reason = ""
        self.sessions = 0
        self.toolhead = self.gcode_move = self.mcu = None
        self.print_stats = self.motion_report = None
        self.timer = self.reactor.register_timer(self._tick)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        for event, why in (
                ('klippy:shutdown', "the printer shut down"),
                ('homing:homing_move_begin', "a homing move started"),
                ('homing:home_rails_begin', "homing started"),
                ('toolhead:set_position', "the position was reset")):
            self.printer.register_event_handler(
                event, (lambda why: lambda *a: self._end(why))(why))
        self.gcode.register_command('MUON_JOG', self.cmd_MUON_JOG,
                                    desc=self.cmd_MUON_JOG_help)

    def _read_notch(self, config):
        # The notch is bed_mesh's no-go regions, read from [bed_mesh] so
        # there is one source. No [bed_mesh] or no regions: no notch.
        if not config.has_section('bed_mesh'):
            return [], None
        mesh = config.getsection('bed_mesh')
        regions = []
        for i in range(1, 100):
            lo = mesh.getfloatlist('no_go_region_%d_min' % (i,), None,
                                   count=2)
            if lo is None:
                break
            hi = mesh.getfloatlist('no_go_region_%d_max' % (i,), count=2)
            regions.append((min(lo[0], hi[0]), min(lo[1], hi[1]),
                            max(lo[0], hi[0]), max(lo[1], hi[1])))
        if not regions:
            return [], None
        return regions, mesh.getfloat('no_go_traverse_z', above=0.)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.mcu = self.printer.lookup_object('mcu')
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.motion_report = self.printer.lookup_object('motion_report',
                                                        None)

    # Gates
    def _printing(self, eventtime):
        if self.print_stats is None:
            return False
        state = self.print_stats.get_status(eventtime).get('state')
        return state in ('printing', 'paused')

    def _kin(self, eventtime):
        return self.toolhead.get_kinematics().get_status(eventtime)

    # Target clamping, in gcode_move.last_position coordinates (G-code
    # plus the G92/offset base), which is what move_with_transform takes
    def _clamp(self, axes, start, target, eventtime):
        kin = self._kin(eventtime)
        lo, hi = list(kin['axis_minimum']), list(kin['axis_maximum'])
        lo[2] = max(lo[2], self.z_min)
        out = list(start)
        why = ""
        for i in ((0, 1) if axes == 'XY' else (2,)):
            v = min(max(target[i], lo[i]), hi[i])
            if abs(v - target[i]) > EPS:
                why = "held at the machine's limits"
            out[i] = v
        if not self.regions:
            return out, why
        if axes == 'XY' and start[2] < self.traverse_z - EPS:
            t = self._notch_entry(start, out)
            if t is not None:
                out[0] = start[0] + t * (out[0] - start[0])
                out[1] = start[1] + t * (out[1] - start[1])
                why = ("stopped at the notch: with the plate below Z%g the "
                       "head may not enter it" % (self.traverse_z,))
        elif axes == 'Z' and out[2] < self.traverse_z - EPS:
            if any(self._inside(start, r) for r in self.regions):
                floor = min(self.traverse_z, start[2])
                if out[2] < floor:
                    out[2] = floor
                    why = ("held at Z%g: the head is over the notch"
                           % (self.traverse_z,))
        return out, why

    def _inside(self, p, r):
        return (r[0] + EPS < p[0] < r[2] - EPS
                and r[1] + EPS < p[1] < r[3] - EPS)

    def _notch_entry(self, a, b):
        # The fraction of a->b that may be travelled before the open
        # interior of any region is entered (None: never entered). Already
        # inside is 0: no XY motion at all. Liang-Barsky clipping.
        best = None
        dx, dy = b[0] - a[0], b[1] - a[1]
        for r in self.regions:
            if self._inside(a, r):
                return 0.
            t0, t1 = 0., 1.
            for p, q in ((-dx, a[0] - r[0]), (dx, r[2] - a[0]),
                         (-dy, a[1] - r[1]), (dy, r[3] - a[1])):
                if abs(p) < EPS:
                    if q <= EPS:
                        t0, t1 = 1., 0.
                    continue
                t = q / p
                if p < 0.:
                    t0 = max(t0, t)
                else:
                    t1 = min(t1, t)
            # Touching an edge or a corner is not entering
            if t1 - t0 > EPS:
                best = t0 if best is None else min(best, t0)
        return best

    # The command
    cmd_MUON_JOG_help = ("Stream a jog: the head (X Y) or the plate (Z) "
                         "heads for the target; STOP=1 stops it")

    def cmd_MUON_JOG(self, gcmd):
        if gcmd.get_int('STOP', 0, minval=0, maxval=1):
            self._end("stopped")
            return
        eventtime = self.reactor.monotonic()
        given = [gcmd.get_float(a, None) for a in 'XYZ']
        if given[2] is not None and (given[0] is not None
                                     or given[1] is not None):
            raise gcmd.error("MUON_JOG moves the head (X Y) or the plate "
                             "(Z), not both at once")
        if all(v is None for v in given):
            raise gcmd.error("MUON_JOG needs a target: X and/or Y, or Z")
        axes = 'Z' if given[2] is not None else 'XY'
        if self._printing(eventtime):
            raise gcmd.error("Not while printing: moving the head would "
                             "ruin the print")
        homed = self._kin(eventtime)['homed_axes']
        need = 'z' if axes == 'Z' else 'xy'
        if not all(a in homed for a in need):
            raise gcmd.error("Home %s first" % (need.upper(),))
        speed = gcmd.get_float('SPEED', self.max_speed[axes], above=0.)
        max_v, _ = self.toolhead.get_max_velocity()
        if axes == 'Z':
            max_v = min(max_v, getattr(self.toolhead.get_kinematics(),
                                       'max_z_velocity', max_v))
        # A speed the bound cannot hold is lowered to one it can
        speed = min(speed, self.max_speed[axes], max_v,
                    self.max_ahead[axes] / (self.lead_time + self.batch_time))
        s = self.session
        if s is not None and s['axes'] != axes:
            self._end("the jog changed axes")
            s = None
        gm = self.gcode_move
        if s is None:
            s = {'axes': axes, 'own': list(gm.last_position), 'batches': []}
            self.sessions += 1
        base = gm.base_position
        target = list(s['own'])
        for i, v in enumerate(given):
            if v is not None:
                target[i] = v + base[i]
        target, why = self._clamp(axes, s['own'], target, eventtime)
        s.update({'target': target, 'speed': speed, 'clamp': why,
                  'final': bool(gcmd.get_int('FINAL', 0, minval=0,
                                             maxval=1)),
                  'updated': eventtime})
        if self.session is None:
            self.session = s
            self.reason = ""
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    # The feeder
    def _end(self, why):
        if self.session is None:
            return
        self.session = None
        self.reason = why
        logging.info("muon_jog: ended: %s", why)
        self.reactor.update_timer(self.timer, self.reactor.NEVER)

    def _tick(self, eventtime):
        s = self.session
        if s is None:
            return self.reactor.NEVER
        if not s['final'] and eventtime - s['updated'] > self.keepalive:
            self._end("no update from the client for %.1f s"
                      % (self.keepalive,))
            return self.reactor.NEVER
        mutex = self.gcode.get_mutex()
        if not mutex.test():
            try:
                with mutex:
                    self._feed(eventtime)
            except Exception as e:
                logging.exception("muon_jog: feed failed")
                self._end("stopped by an error (%s)" % (e,))
        if self.session is None:
            return self.reactor.NEVER
        return eventtime + self.tick_time

    def _ahead(self, s, est):
        s['batches'] = [b for b in s['batches'] if b[0] > est]
        length = sum(b[1] for b in s['batches'])
        t = s['batches'][-1][0] - est if s['batches'] else 0.
        return length, t

    def _feed(self, eventtime):
        s = self.session
        gm = self.gcode_move
        if s is None:
            return
        if any(abs(gm.last_position[i] - s['own'][i]) > EPS
               for i in range(3)):
            self._end("another move took over")
            return
        if self._printing(eventtime):
            self._end("a print started")
            return
        homed = self._kin(eventtime)['homed_axes']
        if not all(a in homed for a in s['axes'].lower()):
            self._end("homing was lost")
            return
        est = self.mcu.estimated_print_time(eventtime)
        ahead, ahead_t = self._ahead(s, est)
        limit = self.max_ahead[s['axes']]
        idx = (0, 1) if s['axes'] == 'XY' else (2,)
        while ahead_t < self.lead_time:
            own = s['own']
            d = [s['target'][i] - own[i] for i in range(3)]
            remaining = math.sqrt(sum(d[i] ** 2 for i in idx))
            if remaining < EPS:
                if s['final']:
                    self._end("arrived")
                return
            seg = min(remaining, s['speed'] * self.batch_time,
                      limit - ahead)
            if seg < remaining and seg < MIN_SEGMENT:
                return
            new = list(own)
            for i in idx:
                new[i] = own[i] + d[i] * seg / remaining
            if (self.regions and s['axes'] == 'XY'
                    and own[2] < self.traverse_z - EPS
                    and self._notch_entry(own, new) is not None):
                self._end("stopped at the notch")
                return
            gm.last_position[:3] = new[:3]
            try:
                gm.move_with_transform(gm.last_position, s['speed'])
            except self.printer.command_error as e:
                gm.reset_last_position()
                self._end("refused: %s" % (e,))
                return
            s['own'] = list(gm.last_position)
            end_t = self.toolhead.get_last_move_time()
            s['batches'].append((end_t, seg))
            ahead += seg
            ahead_t = end_t - est

    def _physical(self, eventtime):
        # Where the toolhead is now as planned (the trapq at the MCU's
        # estimated time), in toolhead coordinates; not toolhead.position,
        # which is where the queue ends
        if self.motion_report is None or self.mcu is None:
            return None
        dtrapq = self.motion_report.dtrapqs.get('toolhead')
        if dtrapq is None:
            return None
        pos, _ = dtrapq.get_trapq_position(
            self.mcu.estimated_print_time(eventtime))
        return None if pos is None else [round(v, 4) for v in pos]

    def get_status(self, eventtime):
        s = self.session
        status = {'active': s is not None, 'reason': self.reason,
                  'sessions': self.sessions,
                  'max_ahead_xy': self.max_ahead['XY'],
                  'max_ahead_z': self.max_ahead['Z'],
                  'position': None, 'stop_position': None, 'axes': '',
                  'target': None, 'ahead': 0., 'clamp': ''}
        if self.toolhead is None:
            return status
        status['position'] = self._physical(eventtime)
        status['stop_position'] = [round(v, 4) for v in
                                   self.toolhead.get_position()[:3]]
        if s is not None:
            base = self.gcode_move.base_position
            status.update({
                'axes': s['axes'], 'clamp': s['clamp'],
                'target': [round(s['target'][i] - base[i], 4)
                           for i in range(3)],
                'ahead': round(sum(b[1] for b in s['batches']), 4)})
        return status


def load_config(config):
    return MuonJog(config)
