# Move geometry for the client move checks (muon_guard, KAN-1731) and the
# print-file preflight (KAN-1732): pure functions, no printer state.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

EPS = 1e-6
ARC_STEP = 1.


def segment_hits_rect(p0, p1, rect):
    """True when the XY segment p0-p1 enters the open interior of rect
    (xmin, ymin, xmax, ymax). Running along an edge does not count."""
    xmin, ymin, xmax, ymax = rect
    (x0, y0), (x1, y1) = p0, p1
    if ((x0 >= xmax and x1 >= xmax) or (x0 <= xmin and x1 <= xmin)
            or (y0 >= ymax and y1 >= ymax) or (y0 <= ymin and y1 <= ymin)):
        return False
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0., 1.
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0),
                 (-dy, y0 - ymin), (dy, ymax - y0)):
        if abs(p) < EPS:
            if q < 0.:
                return False
            continue
        t = q / p
        if p < 0.:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    tm = .5 * (t0 + t1)
    xm, ym = x0 + tm * dx, y0 + tm * dy
    return (xmin + EPS < xm < xmax - EPS) and (ymin + EPS < ym < ymax - EPS)


def arc_points(start, end, offset, clockwise, step=ARC_STEP):
    """XY points along a G2 (clockwise) / G3 arc from start to end around
    start + offset (I, J), at most step mm apart, both ends included."""
    cx, cy = start[0] + offset[0], start[1] + offset[1]
    r = math.hypot(offset[0], offset[1])
    a0 = math.atan2(start[1] - cy, start[0] - cx)
    a1 = math.atan2(end[1] - cy, end[0] - cx)
    sweep = (a0 - a1) if clockwise else (a1 - a0)
    sweep %= 2. * math.pi
    if sweep < EPS:
        sweep = 2. * math.pi
    n = max(1, int(math.ceil(r * sweep / step)))
    sign = -1. if clockwise else 1.
    pts = [(cx + r * math.cos(a0 + sign * sweep * k / n),
            cy + r * math.sin(a0 + sign * sweep * k / n)) for k in range(n)]
    pts.append((end[0], end[1]))
    return pts


def path_hits_rects(points, rects):
    """The first rect any leg of the XY polyline enters, or None."""
    for a, b in zip(points, points[1:]):
        for rect in rects:
            if segment_hits_rect(a, b, rect):
                return rect
    return None
