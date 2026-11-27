#!/usr/bin/env python3
"""
Run robot/Robot.h on the computer, in a simulated room.

    python3 tools/sim.py                          mission 3 beside a wall, for 20 s
    python3 tools/sim.py --scene obstacle         mission 1, then forward for 3 s towards a wall
    python3 tools/sim.py --out sim/bang           name the result
    python3 tools/sim.py --compare sim/bang.csv   draw an earlier run in grey behind
    python3 tools/sim.py -D WALL_GAIN_NUM=3       override a #define without editing
    python3 tools/sim.py --door 2000:2800         a doorway in the wall, x in mm
    python3 tools/sim.py --send 2:'{"stop":true}' a command, 2 s after the first one

robot/Robot.h is compiled as it is, with an ordinary C++ compiler. The
hardware headers (Motors.h, TofSensor.h, Clock.h, ...) are replaced by the
ones in tools/sim/stubs/, which move a model robot among walls instead of
driving pins. Config.h, and Config.local.h if present, are used as they
are: a sensor switched off there reads nothing, as on the robot.

The scenes, and the commands each sends when none is given:

    mur         a straight wall on the left (TP5, TP6)     {"mission":3}
    patrouille  the same wall, a box in the way and a wall
                across the end (TP7)                       {"mission":5}
    obstacle    a wall across the path (TP1)               {"mission":1}, then
                                                           {"left":200,"right":200,"duration":3000}
    salle       a closed room, 3 m by 2.4 m                {"mission":2}

The run writes three files:

    sim/run.csv   one row every 10 ms: position, heading, wheel commands,
                  sensor readings
    sim/run.svg   the path seen from above, then the readings, heading and
                  wheel commands against time; VS Code shows it, and
                  redraws it after each run
    sim/run.log   every line sent to the robot and every line it sent back

The model's numbers (speeds, dead zone, wheel spacing, sensors) are
invented and listed in tools/sim/stubs/SimWorld.h. The robot behaves like
this one, with different amplitudes, periods and distances.
"""

import argparse
import collections
import csv
import html
import json
import math
import re
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / "tools" / "sim"
FLAGS = ["-std=c++17", "-O2", "-Wall"]

SCENES = {
    "mur":        {"wall": True,  "obstacle": False, "sends": ['{"mission":3}']},
    "patrouille": {"wall": True,  "obstacle": True,  "sends": ['{"mission":5}']},
    "obstacle":   {"wall": False, "obstacle": True,
                   "sends": ['{"mission":1}', '{"left":200,"right":200,"duration":3000}']},
    "salle":      {"wall": False, "obstacle": False, "sends": ['{"mission":2}']},
}


def build(robot_dir, defines, workspace):
    """Copy the robot's headers and the stubs together, compile, return the binary."""
    origin = {}                             # copied name -> where it came from
    for header in robot_dir.glob("*.h"):
        shutil.copy(header, workspace)
        origin[header.name] = header
    for stub in (SIM / "stubs").glob("*.h"):
        shutil.copy(stub, workspace)        # replaces the hardware headers
        origin[stub.name] = stub
    shutil.copy(SIM / "main.cpp", workspace)
    origin["main.cpp"] = SIM / "main.cpp"

    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        sys.exit("No C++ compiler found. On Debian or Ubuntu: sudo apt-get install g++")

    binary = workspace / "sim"
    colour = ["-fdiagnostics-color=always"] if sys.stderr.isatty() else []
    command = [compiler, *FLAGS, *colour, *[f"-D{d}" for d in defines],
               "-I", str(workspace), "-o", str(binary), str(workspace / "main.cpp")]
    compiled = subprocess.run(command, capture_output=True, text=True)

    # Point messages at the real files rather than at the temporary copies.
    def where(match):
        source = origin.get(match.group(1))
        if source is None:
            return match.group(0)
        try:
            return str(source.relative_to(Path.cwd()))
        except ValueError:
            return str(source)
    sys.stderr.write(re.sub(re.escape(str(workspace)) + r"/([\w.]+)", where, compiled.stderr))

    if compiled.returncode != 0:
        sys.exit(f"{origin['Robot.h']} did not compile (errors above)")
    return binary


def read_trace(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in row.items()})
            except (TypeError, ValueError):
                break                   # a crashed run's last, unfinished row
    return rows


# ---- Drawing ----------------------------------------------------------------

WIDTH = 920
LEFT, RIGHT = 70, 20
PLOT_W = WIDTH - LEFT - RIGHT
# Each colour can be overridden by a CSS variable of the page the SVG sits in.
# They are set in the SVG's stylesheet, through classes, which every renderer
# reads; var() in an attribute is not universally supported.
COLOURS = {name: f"var(--sim-{name}, {value})" for name, value in {
    "run": "#1f6fd1", "left": "#d1561f", "right": "#1f8f5a", "grey": "#9aa0a6",
    "grid": "#e3e5e8", "axis": "#5f6368", "wall": "#202124", "missing": "#c5221f",
    "text": "#3c4043", "bg": "#ffffff", "body": "#e8f0fb", "beam": "#f3d9cc",
    "obstacle": "#a0662a", "band": "#f1f3f4", "front": "#7b4fc9"}.items()}


def nice_step(span, ticks=6):
    raw = span / ticks
    magnitude = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    for scale in (1, 2, 5, 10):
        if scale * magnitude >= raw:
            return scale * magnitude
    return 10 * magnitude


def ticks(lo, hi, count=6):
    step = nice_step(hi - lo, count)
    first = (lo // step) * step
    values, i = [], 0
    while first + i * step <= hi + 1e-9:
        v = round(first + i * step, 9)
        if v >= lo - 1e-9:
            values.append(v)
        i += 1
    return values


class Panel:
    """A plot area with linear x and y scales; y may point down."""

    def __init__(self, svg, top, height, xr, yr, title, xlabel, ylabel, y_down=False, yticks=6):
        self.svg, self.top, self.height = svg, top, height
        self.x0, self.x1 = xr
        self.y0, self.y1 = yr
        self.y_down = y_down
        svg.append(f'<text x="{LEFT}" y="{top - 12}" class="sim-title">{title}</text>')
        for v in ticks(self.x0, self.x1):
            px = self.px(v)
            svg.append(f'<line x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top + height}" class="sim-grid"/>')
            svg.append(f'<text x="{px:.1f}" y="{top + height + 15}" class="sim-tick" text-anchor="middle">{v:g}</text>')
        for v in ticks(self.y0, self.y1, yticks):
            py = self.py(v)
            svg.append(f'<line x1="{LEFT}" y1="{py:.1f}" x2="{LEFT + PLOT_W}" y2="{py:.1f}" class="sim-grid"/>')
            svg.append(f'<text x="{LEFT - 6}" y="{py + 4:.1f}" class="sim-tick" text-anchor="end">{v:g}</text>')
        svg.append(f'<rect x="{LEFT}" y="{top}" width="{PLOT_W}" height="{height}" class="sim-frame"/>')
        svg.append(f'<text x="{LEFT + PLOT_W}" y="{top + height + 30}" class="sim-label" text-anchor="end">{xlabel}</text>')
        svg.append(f'<text x="{LEFT - 50}" y="{top + height / 2}" class="sim-label" text-anchor="middle" '
                   f'transform="rotate(-90 {LEFT - 50} {top + height / 2})">{ylabel}</text>')

    def px(self, v):
        return LEFT + (v - self.x0) / (self.x1 - self.x0) * PLOT_W

    def py(self, v):
        f = (v - self.y0) / (self.y1 - self.y0)
        return self.top + (f if self.y_down else 1 - f) * self.height

    def line(self, xs, ys, colour, width=1.6, dash=None):
        xs, ys = list(xs), list(ys)
        keep = [i for i in range(len(xs)) if i % 3 == 0 or i == len(xs) - 1]
        xs, ys = [xs[i] for i in keep], [ys[i] for i in keep]
        pts = " ".join(f"{self.px(a):.1f},{self.py(b):.1f}" for a, b in zip(xs, ys))
        extra = f' stroke-dasharray="{dash}"' if dash else ""
        self.svg.append(f'<polyline points="{pts}" fill="none" class="sim-s-{colour}" '
                        f'stroke-width="{width}" stroke-linejoin="round"{extra}/>')

    def dots(self, xs, ys, colour, r=1.6):
        for a, b in zip(xs, ys):
            self.svg.append(f'<circle cx="{self.px(a):.1f}" cy="{self.py(b):.1f}" r="{r}" class="sim-f-{colour}"/>')

    def clip(self, name):
        """A clipPath the size of the plot area, for drawings that may stray outside it."""
        self.svg.append(f'<clipPath id="{name}"><rect x="{LEFT}" y="{self.top}" width="{PLOT_W}" '
                        f'height="{self.height}"/></clipPath>')
        return f'clip-path="url(#{name})"'


def legend(svg, x, y, items):
    for label, colour in items:
        svg.append(f'<rect x="{x}" y="{y - 9}" width="14" height="4" class="sim-f-{colour}"/>')
        svg.append(f'<text x="{x + 20}" y="{y - 4}" class="sim-tick">{label}</text>')
        x += 30 + 7 * len(label)


# The robot's size and sensors, as the run reports them (see main.cpp).
Geometry = collections.namedtuple("Geometry", "half_length half_width side_ahead side_cone front_cone")
DEFAULT_GEOMETRY = Geometry(100, 70, 60, 13.5, 15)


def pieces(walls, obstacle=True):
    """The walls less their gaps, as (ax, ay, bx, by, is_obstacle)."""
    out = []
    for w in walls:
        if w["obstacle"] and not obstacle:
            continue
        length = math.hypot(w["bx"] - w["ax"], w["by"] - w["ay"])
        ux, uy = (w["bx"] - w["ax"]) / length, (w["by"] - w["ay"]) / length
        at = 0
        for a, b in w["gaps"]:
            if a > at:
                out.append((w["ax"] + ux * at, w["ay"] + uy * at, w["ax"] + ux * a, w["ay"] + uy * a, w["obstacle"]))
            at = max(at, b)
        if at < length:
            out.append((w["ax"] + ux * at, w["ay"] + uy * at, w["bx"], w["by"], w["obstacle"]))
    return out


def clip_segment(ax, ay, bx, by, x0, x1, y0, y1):
    """The part of a-b inside the box, or None (Liang-Barsky)."""
    t0, t1 = 0.0, 1.0
    dx, dy = bx - ax, by - ay
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0:
            if q < 0:
                return None
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return None
    return ax + t0 * dx, ay + t0 * dy, ax + t1 * dx, ay + t1 * dy


def flank(r, g):
    """Middle of the left flank, as (distance along the wall, distance to the wall), in mm.
    clearance_mm is measured from the turning point as if the robot were square to
    the wall; at an angle the flank sits further along and nearer the wall."""
    h = math.radians(r["heading_deg"])
    centre = r["clearance_mm"] + g.half_width              # turning point to the wall
    return r["x_mm"] - g.half_width * math.sin(h), centre - g.half_width * math.cos(h)


def corners(r, g):
    """The body's four corners, in the same coordinates as flank()."""
    h = math.radians(r["heading_deg"])
    c, s = math.cos(h), math.sin(h)
    centre = r["clearance_mm"] + g.half_width
    return [(r["x_mm"] + i * g.half_length * c - j * g.half_width * s,
             centre - (i * g.half_length * s + j * g.half_width * c))
            for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def world_corners(r, g):
    """The body's four corners, in the scene's own coordinates."""
    h = math.radians(r["heading_deg"])
    c, s = math.cos(h), math.sin(h)
    return [(r["x_mm"] + i * g.half_length * c - j * g.half_width * s,
             r["y_mm"] + i * g.half_length * s + j * g.half_width * c)
            for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def robot(g):
    """The robot seen from above, in mm, in its own frame: x forward, y towards
    its right side, so that with heading 0 and the wall at the top of a panel
    its left flank faces the wall. Strokes keep their width when scaled."""
    L, W, A = g.half_length, g.half_width, g.side_ahead
    ns = 'vector-effect="non-scaling-stroke"'
    parts = [f'<rect x="{-L}" y="{-W}" width="{2 * L}" height="{2 * W}" rx="12" '
             f'class="sim-f-body sim-s-run" stroke-width="1.5" {ns}/>']
    for wx in (L - 40, -(L - 40)):                     # four wheels, inside the body
        for wy in (-W + 4, W - 26):
            parts.append(f'<rect x="{wx - 30}" y="{wy}" width="60" height="22" rx="5" class="sim-f-wall"/>')
    # The front: a bumper, and an arrow showing which way it drives.
    parts.append(f'<rect x="{L - 10}" y="{-W + 30}" width="10" height="{2 * W - 60}" class="sim-f-run"/>')
    parts.append(f'<path d="M {-25} 0 L {L - 30} 0 M {L - 55} -18 L {L - 30} 0 L {L - 55} 18" '
                 f'fill="none" class="sim-s-run" stroke-width="2" {ns}/>')
    # The side sensor on the left flank.
    parts.append(f'<rect x="{A - 10}" y="{-W - 8}" width="20" height="12" class="sim-f-left"/>')
    return parts


def beam(g):
    """The side sensor's cone, in the robot's frame, as far as the sensor's range."""
    A, W, R = g.side_ahead, g.half_width, 1300
    spread = R * math.tan(math.radians(g.side_cone))
    return (f'<path d="M {A} {-W} L {A - spread} {-W - R} L {A + spread} {-W - R} Z" '
            f'class="sim-f-beam"/>')


def front_axis(g, reach):
    """The front sensor's axis, in the robot's frame, `reach` mm long."""
    L = g.half_length
    return (f'<line x1="{L}" y1="0" x2="{L + reach}" y2="0" class="sim-s-front" stroke-width="1.5" '
            f'stroke-dasharray="6 4" vector-effect="non-scaling-stroke"/>')


def wall(svg, panel, doors, x0, x1, scale=1.0):
    """The wall y = 0 from x0 to x1 (in panel units, mm times scale), with its doorways."""
    segments, at = [], x0 / scale
    for d in sorted(doors):
        segments.append((at, d[0]))
        at = d[1]
    segments.append((at, x1 / scale))
    for a, b in segments:
        if b > a:
            svg.append(f'<line x1="{panel.px(a * scale):.1f}" y1="{panel.py(0):.1f}" x2="{panel.px(b * scale):.1f}" '
                       f'y2="{panel.py(0):.1f}" class="sim-s-wall" stroke-width="5"/>')


def unwrap(degrees):
    """Headings without the jump from +180 to -180, for a robot that turns round."""
    out, offset, last = [], 0.0, None
    for d in degrees:
        if last is not None:
            if d - last > 180:
                offset -= 360
            elif d - last < -180:
                offset += 360
        last = d
        out.append(d + offset)
    return out


def time_marks(svg, panel, events, label):
    """Shaded bands for the time a link or a sensor was down, a line for the moments
    the obstacle went or came back; their names on the first panel only."""
    for a, b, name in events:
        if b is not None:
            x0, x1 = panel.px(min(a, panel.x1)), panel.px(min(b, panel.x1))
            svg.append(f'<rect x="{x0:.1f}" y="{panel.top}" width="{max(x1 - x0, 1):.1f}" '
                       f'height="{panel.height}" class="sim-f-band"/>')
        elif a <= panel.x1:
            px = panel.px(a)
            svg.append(f'<line x1="{px:.1f}" y1="{panel.top}" x2="{px:.1f}" y2="{panel.top + panel.height}" '
                       f'class="sim-s-obstacle" stroke-width="1.2" stroke-dasharray="4 3"/>')
        if label and a <= panel.x1:
            svg.append(f'<text x="{panel.px(a) + 4:.1f}" y="{panel.top + 13}" class="sim-tick">{html.escape(name)}</text>')


def missing_bars(svg, panel, rows, key):
    """One red bar under the panel per stretch without a valid reading."""
    start = None
    for r in rows + [{"t_ms": rows[-1]["t_ms"], key: 1}]:
        ok = r.get(key, 1)
        if not ok and start is None:
            start = r["t_ms"]
        elif ok and start is not None:
            a, b = panel.px(start / 1000), panel.px(r["t_ms"] / 1000)
            svg.append(f'<rect x="{a:.1f}" y="{panel.top + panel.height - 8}" width="{max(b - a, 1):.1f}" '
                       f'height="8" class="sim-f-missing"/>')
            start = None


def broken_line(panel, points, colour, limit):
    """A dashed line through (t, value) points, broken where value is -1 or above limit."""
    stretch = []
    for a, w in points + [(None, -1)]:
        if 0 <= w <= limit:
            stretch.append((a, w))
        elif stretch:
            panel.line([a for a, _ in stretch], [w for _, w in stretch], colour, 1, "3 3")
            stretch = []


def path_panel(svg, top, rows, others, scene, g, hit):
    """Wall scenes: the left flank's path, the vertical axis stretched."""
    doors = [tuple(gap) for w in scene["walls"] if not w["obstacle"] and w["ay"] == 0 == w["by"]
             for gap in ((a + w["ax"], b + w["ax"]) for a, b in w["gaps"])]
    path_pts = [flank(r, g) for r in rows]
    other_pts = [[flank(r, g) for r in o] for _, o in others]
    everything = path_pts + [p for o in other_pts for p in o]
    x_max = max(px for px, _ in everything) / 1000
    x_min = min(0, min(px for px, _ in everything) / 1000)
    # The rest of the scene, if it is near: the box and the far wall.
    extra = [p for p in pieces(scene["walls"]) if not (p[1] == 0 == p[3])]
    near = [p for p in extra if min(p[0], p[2]) / 1000 < x_max + 1]
    if near:
        x_max = max(x_max, max(max(p[0], p[2]) for p in near) / 1000 + 0.2)
    x_max += 0.03 * (x_max - x_min)
    c_max = max(200, max(py for _, py in everything) * 1.1,
                *[-min(p[1], p[3]) * 1.05 for p in near if p[4]])
    c_min = min(0, min(py for _, py in everything))

    p1 = Panel(svg, top, 230, (x_min, max(x_max, 0.5)), (c_min, c_max),
               "Path seen from above (not to scale: the wall is the black line)",
               "distance along the wall (m)", "left flank to wall (mm)", y_down=True)
    if scene["name"] == "patrouille":
        end = max(max(p[0], p[2]) for p in pieces(scene["walls"]) if p[1] == 0 == p[3])
        wall(svg, p1, doors, x_min, min(max(x_max, 0.5), end / 1000), scale=0.001)
    else:
        wall(svg, p1, doors, x_min, max(x_max, 0.5), scale=0.001)
    clip = p1.clip("sim-path")
    gone = not scene["obstacle_at_end"]
    for ax, ay, bx, by, obstacle in near:
        part = clip_segment(ax / 1000, -ay, bx / 1000, -by, p1.x0, p1.x1, min(p1.y0, p1.y1), max(p1.y0, p1.y1))
        if part:
            kind = "obstacle" if obstacle else "wall"
            dash = ' stroke-dasharray="6 4"' if obstacle and gone else ""
            svg.append(f'<line x1="{p1.px(part[0]):.1f}" y1="{p1.py(part[1]):.1f}" x2="{p1.px(part[2]):.1f}" '
                       f'y2="{p1.py(part[3]):.1f}" class="sim-s-{kind}" stroke-width="{3 if obstacle else 5}"'
                       f'{dash} {clip}/>')
    for pts in other_pts:
        p1.line([a / 1000 for a, _ in pts], [b for _, b in pts], "grey", 1.4, "5 3")
    p1.line([a / 1000 for a, _ in path_pts], [b for _, b in path_pts], "run", 2)
    # Start (hollow circle) and end (dot); a red ring where the body touched.
    sx, sy = path_pts[0]
    ex, ey = path_pts[-1]
    svg.append(f'<circle cx="{p1.px(ex / 1000):.1f}" cy="{p1.py(ey):.1f}" r="4" class="sim-f-run"/>')
    svg.append(f'<circle cx="{p1.px(sx / 1000):.1f}" cy="{p1.py(sy):.1f}" '
               f'r="5" stroke-width="2" class="sim-s-run sim-f-bg"/>')
    if hit:
        body = corners(rows[-1], g)
        touch = min(body, key=lambda p: abs(p[1])) if hit == "wall" else max(body, key=lambda p: p[0])
        ty = 0 if hit == "wall" else touch[1]
        svg.append(f'<circle cx="{p1.px(touch[0] / 1000):.1f}" cy="{p1.py(ty):.1f}" r="7" fill="none" '
                   f'class="sim-s-missing" stroke-width="2.5"/>')
    return doors


def impact_panel(svg, top, rows, doors, g):
    """The moment the robot hit the wall y = 0, to scale: equal millimetres both
    ways, around the point that touched. The body has the robot's real shape."""
    last = rows[-1]
    path_pts = [flank(r, g) for r in rows]
    body = corners(last, g)
    touch = min(body, key=lambda p: abs(p[1]))
    height = 200
    near = [d for _, d in body]
    # More room beyond the wall when a doorway is close: the beam goes through it.
    beyond = 200 if any(a - 1000 < touch[0] < b + 1000 for a, b in doors) else 60
    lo, hi = min(0, min(near)) - beyond, max(near) + 120
    scale = height / (hi - lo)                               # px per mm
    width = PLOT_W / scale
    x0 = touch[0] - width / 2
    p0 = Panel(svg, top, height, (x0, x0 + width), (lo, hi),
               "Where it hit the wall, to scale", "distance along the wall (mm)",
               "to wall (mm)", y_down=True)
    clip = p0.clip("sim-zoom")
    # The beam stops at the wall, and goes on through a doorway.
    wall_y = max(top, p0.py(0))
    gaps = "".join(f'<rect x="{p0.px(a):.1f}" y="{top}" width="{p0.px(b) - p0.px(a):.1f}" height="{height}"/>'
                   for a, b in doors)
    svg.append(f'<clipPath id="sim-zoom-near"><rect x="{LEFT}" y="{wall_y:.1f}" width="{PLOT_W}" '
               f'height="{top + height - wall_y:.1f}"/>{gaps}</clipPath>')
    svg.append(f'<g {clip}>')
    wall(svg, p0, doors, x0, x0 + width)
    p0.line([a for a, _ in path_pts], [b for _, b in path_pts], "run", 1.5, "4 3")
    centre = (last["x_mm"], last["clearance_mm"] + g.half_width)
    place = (f'translate({p0.px(centre[0]):.1f} {p0.py(centre[1]):.1f}) '
             f'rotate({-last["heading_deg"]:.1f}) scale({scale:.4f})')
    svg.append(f'<g clip-path="url(#sim-zoom-near)"><g transform="{place}">{beam(g)}</g></g>')
    svg.append(f'<g transform="{place}">' + "".join(robot(g)) + '</g>')
    svg.append(f'<circle cx="{p0.px(touch[0]):.1f}" cy="{p0.py(0):.1f}" r="7" fill="none" '
               f'class="sim-s-missing" stroke-width="2.5"/>')
    svg.append('</g>')
    return height


def top_view(svg, top, rows, others, scene, g, hit):
    """Other scenes: the room seen from above, to scale."""
    xs = [r["x_mm"] for r in rows] + [r["x_mm"] for _, o in others for r in o if "y_mm" in r]
    ys = [r["y_mm"] for r in rows] + [r["y_mm"] for _, o in others for r in o if "y_mm" in r]
    margin = 400
    x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    # The walls near the path come into the picture too.
    walls = pieces(scene["walls"])
    for ax, ay, bx, by, _ in walls:
        part = clip_segment(ax, ay, bx, by, x0 - 300, x1 + 300, y0 - 300, y1 + 300)
        if part:
            x0, x1 = min(x0, part[0] - 100, part[2] - 100), max(x1, part[0] + 100, part[2] + 100)
            y0, y1 = min(y0, part[1] - 100, part[3] - 100), max(y1, part[1] + 100, part[3] + 100)
    # Equal millimetres both ways: widen whichever range is short.
    scale = min(PLOT_W / (x1 - x0), 420 / (y1 - y0))
    height = max(240, (y1 - y0) * scale)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    half_w, half_h = PLOT_W / scale / 2, height / scale / 2
    p = Panel(svg, top, height, (cx - half_w, cx + half_w), (cy - half_h, cy + half_h),
              "Seen from above, to scale", "x (mm)", "y (mm)")
    clip = p.clip("sim-room")
    svg.append(f'<g {clip}>')
    gone = not scene["obstacle_at_end"]
    for ax, ay, bx, by, obstacle in walls:
        kind = "obstacle" if obstacle else "wall"
        dash = ' stroke-dasharray="10 6"' if obstacle and gone else ""
        svg.append(f'<line x1="{p.px(ax):.1f}" y1="{p.py(ay):.1f}" x2="{p.px(bx):.1f}" y2="{p.py(by):.1f}" '
                   f'class="sim-s-{kind}" stroke-width="5"{dash}/>')
    for _, o in others:
        if o and "y_mm" in o[0]:
            p.line([r["x_mm"] for r in o], [r["y_mm"] for r in o], "grey", 1.4, "5 3")
    p.line([r["x_mm"] for r in rows], [r["y_mm"] for r in rows], "run", 1.6)

    def place(r):
        return (f'translate({p.px(r["x_mm"]):.1f} {p.py(r["y_mm"]):.1f}) '
                f'rotate({-r["heading_deg"]:.1f}) scale({scale:.4f})')
    first, last = rows[0], rows[-1]
    svg.append(f'<g transform="{place(first)}" opacity="0.35">' + "".join(robot(g)) + '</g>')
    # Where the front sensor points, as far as the first wall.
    reach = last.get("front_wall_mm", -1)
    if 0 <= reach <= 3500:
        svg.append(f'<g transform="{place(last)}">{front_axis(g, reach)}</g>')
    svg.append(f'<g transform="{place(last)}">' + "".join(robot(g)) + '</g>')
    if hit:
        # The corner nearest to a wall.
        body = world_corners(last, g)
        def gap(c):
            return min(abs((bx - ax) * (ay - c[1]) - (ax - c[0]) * (by - ay)) / math.hypot(bx - ax, by - ay)
                       for ax, ay, bx, by, _ in walls)
        touch = min(body, key=gap)
        svg.append(f'<circle cx="{p.px(touch[0]):.1f}" cy="{p.py(touch[1]):.1f}" r="8" fill="none" '
                   f'class="sim-s-missing" stroke-width="2.5"/>')
    svg.append('</g>')
    return height


def reading_panel(svg, top, rows, others, events, t_max, key, title, truth_label, limit, every, first):
    """Readings of one sensor against time, with the true distance dashed."""
    ok, mm, truth_key = f"{key}_ok", f"{key}_mm", "wall_mm" if key == "side" else "front_wall_mm"
    readings = [(r["t_ms"] / 1000, r[mm]) for r in rows if r.get(ok) and r["t_ms"] % every == 0]
    truth = [(r["t_ms"] / 1000, r.get(truth_key, -1)) for r in rows]
    shown = [w for _, w in truth if 0 <= w <= limit]
    s_max = max([s for _, s in readings] + shown
                + [r[mm] for _, o in others for r in o if r.get(ok)] + [200]) * 1.1
    panel = Panel(svg, top, 180, (0, t_max), (0, s_max), title, "time (s)", mm)
    time_marks(svg, panel, events, first)
    for _, o in others:
        kept = [r for r in o if r.get(ok) and r["t_ms"] % every == 0]
        panel.dots([r["t_ms"] / 1000 for r in kept], [r[mm] for r in kept], "grey", 1.1)
    broken_line(panel, truth, "wall", limit)
    panel.dots([a for a, _ in readings], [b for _, b in readings], "run")
    if shown:
        legend(svg, LEFT + PLOT_W - 330, top - 12, [("reading", "run"), (truth_label, "wall")])
    missing_bars(svg, panel, rows, ok)
    return 180


def draw(rows, others, scene, events, title, ending, geometry, path):
    g = geometry or DEFAULT_GEOMETRY
    t = [r["t_ms"] / 1000 for r in rows]
    hit = None
    if ending and ending.startswith("HIT THE WALL"):
        hit = "wall"
    elif ending and ending.startswith("HIT THE OBSTACLE"):
        hit = "obstacle"
    t_max = max(max(r["t_ms"] for r in rows + [r for _, o in others for r in o]) / 1000, 1)

    svg = []
    top = 90
    if scene["wall_scene"]:
        doors = path_panel(svg, top, rows, others, scene, g, hit)
        top += 230 + 80
        if hit == "wall":
            top += impact_panel(svg, top, rows, doors, g) + 80
    else:
        top += top_view(svg, top, rows, others, scene, g, hit) + 80

    first = True
    if not scene["wall_scene"] or any(r.get("front_ok") for r in rows):
        top += reading_panel(svg, top, rows, others, events, t_max, "front",
                             "Front sensor reading (red bars: no valid reading)",
                             "sensor to wall, straight ahead", 3500, 60, first) + 80
        first = False
    if scene["wall_scene"] or any(r.get("side_ok") for r in rows):
        top += reading_panel(svg, top, rows, others, events, t_max, "side",
                             "Side sensor reading (red bars: no valid reading)",
                             "sensor to wall, square", 1e9, 40, first) + 80
        first = False

    headings = unwrap([r["heading_deg"] for r in rows])
    other_headings = [unwrap([r["heading_deg"] for r in o]) for _, o in others]
    every = headings + [h for o in other_headings for h in o]
    h_lo, h_hi = min(-10, min(every)), max(10, max(every))
    p4 = Panel(svg, top, 140, (0, t_max), (h_lo - 5, h_hi + 5),
               "Heading (positive: turned towards the wall)" if scene["wall_scene"]
               else "Heading (positive: turned left)", "time (s)", "heading (deg)")
    time_marks(svg, p4, events, first)
    for (_, o), h in zip(others, other_headings):
        p4.line([r["t_ms"] / 1000 for r in o], h, "grey", 1.2, "5 3")
    p4.line(t, headings, "run", 1.6)

    top += 140 + 80
    lo = min(-10, min(min(r["left_cmd"], r["right_cmd"]) for r in rows))
    hi = max(10, max(max(r["left_cmd"], r["right_cmd"]) for r in rows))
    p3 = Panel(svg, top, 160, (0, t_max), (lo - 10, hi + 10),
               "Wheel commands", "time (s)", "command (-255 to 255)")
    time_marks(svg, p3, events, False)
    p3.line(t, [r["left_cmd"] for r in rows], "left", 1.3)
    p3.line(t, [r["right_cmd"] for r in rows], "right", 1.3)
    legend(svg, LEFT + 6, top + 175 + 30, [("left", "left"), ("right", "right")])

    height = top + 160 + 50
    run_legend = [(title, "run")] + [(n, "grey") for n, _ in others]
    head = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {height}" '
            f'width="{WIDTH}" height="{height}" font-family="system-ui, sans-serif">',
            f'<style>.sim-title{{font-size:14px;font-weight:600;fill:{COLOURS["text"]}}}'
            f'.sim-tick{{font-size:11px;fill:{COLOURS["axis"]}}}.sim-label{{font-size:12px;fill:{COLOURS["text"]}}}'
            f'.sim-end{{font-size:14px;font-weight:600;fill:{COLOURS["text"]}}}'
            f'.sim-grid{{stroke:{COLOURS["grid"]};stroke-width:1}}'
            f'.sim-frame{{fill:none;stroke:{COLOURS["axis"]};stroke-width:1}}'
            + "".join(f".sim-s-{name}{{stroke:{value}}}.sim-f-{name}{{fill:{value}}}"
                      for name, value in COLOURS.items())
            + '</style>',
            f'<rect width="{WIDTH}" height="{height}" class="sim-f-bg"/>']
    legend(head, LEFT, 22, run_legend)
    head.append(f'<text x="{LEFT + PLOT_W}" y="22" class="sim-tick" text-anchor="end">'
                f'scene {html.escape(scene["name"])}</text>')
    if ending:
        head.append(f'<text x="{LEFT}" y="48" class="sim-end">Ended: {html.escape(ending)}</text>')
    path.write_text("\n".join(head + svg + ["</svg>"]) + "\n")


# ---- Running ----------------------------------------------------------------

def seconds(text, option):
    try:
        value = float(text)
    except ValueError:
        sys.exit(f"{option} {text}: expected a number of seconds")
    if value < 0:
        sys.exit(f"{option} {text}: expected a time from 0 on, counted from the first command")
    return value


def window(text, option, example):
    """FROM:TO in seconds."""
    parts = text.split(":")
    if len(parts) != 2:
        sys.exit(f"{option} {text}: expected FROM:TO in seconds, such as {example}")
    a, b = (seconds(p, option) for p in parts)
    if b <= a:
        sys.exit(f"{option} {text}: the end comes before the start")
    return a, b


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parser.add_argument("--scene", choices=SCENES, default="mur", help="where the robot is (default mur)")
    parser.add_argument("--mission", type=int, help="send {\"mission\": N} at 0 s, instead of the scene's commands")
    parser.add_argument("--send", action="append", default=[], metavar="T:LINE",
                        help="send LINE to the robot T seconds after the start, as typed in the console; "
                             "replaces the scene's commands; repeat for more")
    parser.add_argument("--duration", type=float, default=20, help="seconds (default 20)")
    parser.add_argument("--start", type=float,
                        help="initial gap in mm: left flank to the wall (mur, patrouille; default 260), "
                             "front to the wall ahead (obstacle: 800, salle: 1400)")
    parser.add_argument("--heading", type=float, default=0,
                        help="initial angle, degrees, positive to the left: towards the wall in mur and patrouille")
    parser.add_argument("--door", action="append", default=[], metavar="FROM:TO",
                        help="gap in the wall from x=FROM to x=TO mm (mur, patrouille); repeat for more")
    parser.add_argument("--offline", action="append", default=[], metavar="FROM:TO",
                        help="the link is down from FROM s to TO s; repeat for more")
    parser.add_argument("--remove", type=str, metavar="T", help="take the obstacle away at T s")
    parser.add_argument("--place", type=str, metavar="T", help="put the obstacle back at T s")
    parser.add_argument("--unplug", action="append", default=[], metavar="SENSOR:FROM:TO",
                        help="front or side sensor unplugged from FROM s to TO s, such as front:6:7")
    parser.add_argument("-D", dest="defines", action="append", default=[], metavar="NAME=VALUE",
                        help="#define passed to the compiler; overrides an #ifndef default")
    parser.add_argument("--seed", type=int, default=1, help="sensor noise seed (default 1)")
    parser.add_argument("--compare", action="append", default=[], metavar="CSV",
                        help="earlier run to draw in grey; repeat for more")
    parser.add_argument("--out", default="sim/run", help="output name, without extension (default sim/run)")
    parser.add_argument("--robot", default=str(REPO / "robot"), help=argparse.SUPPRESS)
    # For client/simrobot.js: compile into DIR, print the program's path, stop.
    parser.add_argument("--build", metavar="DIR", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.build:
        binary = build(Path(args.robot), args.defines, Path(args.build))
        print(binary)
        return

    scene = SCENES[args.scene]
    doors = []
    for d in args.door:
        try:
            a, b = (float(v) for v in d.split(":"))
        except ValueError:
            sys.exit(f"--door {d}: expected FROM:TO in mm, such as 2000:2800")
        doors.append((min(a, b), max(a, b)))
    if doors and not scene["wall"]:
        sys.exit(f"--door: the {args.scene} scene has no wall beside the robot (--scene mur or patrouille)")
    # Doors that touch or overlap make one wider opening.
    merged = []
    for a, b in sorted(doors):
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    doors = merged
    if args.duration <= 0:
        sys.exit("--duration: expected a number of seconds above 0")

    # What happens when, in seconds after the start.
    sends = []
    if args.mission is not None:
        sends.append((0.0, json.dumps({"mission": args.mission}, separators=(",", ":"))))
    for s in args.send:
        when, colon, line = s.partition(":")
        if not colon or not line.strip():
            sys.exit(f"--send {s}: expected T:LINE, such as 0:'{{\"mission\":1}}'")
        if "\n" in line:
            sys.exit(f"--send {s}: one line at a time")
        sends.append((seconds(when, "--send"), line.strip()))
    if not sends:
        sends = [(0.0, line) for line in scene["sends"]]
    offline = [window(w, "--offline", "6:8") for w in args.offline]
    for when, line in sends:
        for a, b in offline:
            if a <= when < b:
                sys.exit(f"--send {when:g}:{line}: the link is down then (--offline {a:g}:{b:g})")
    unplugged = []
    for u in args.unplug:
        sensor, colon, rest = u.partition(":")
        if sensor not in ("front", "side") or not colon:
            sys.exit(f"--unplug {u}: expected front:FROM:TO or side:FROM:TO, such as front:6:7")
        unplugged.append((sensor, *window(rest, "--unplug", "front:6:7")))
    moves = []
    for option, value in (("--remove", args.remove), ("--place", args.place)):
        if value is not None:
            if not scene["obstacle"]:
                sys.exit(f"{option}: the {args.scene} scene has no obstacle (--scene obstacle or patrouille)")
            moves.append((option, seconds(value, option)))
    late = [t for t, _ in sends] + [a for a, _ in offline] + [a for _, a, _ in unplugged] + [t for _, t in moves]
    if any(t >= args.duration for t in late):
        sys.exit(f"an event at {max(late):g} s comes after the end of the run: raise --duration")

    # For the picture.
    events = [(a, b, "link down") for a, b in offline]
    events += [(a, b, f"{sensor} sensor unplugged") for sensor, a, b in unplugged]
    events += [(t, None, "obstacle removed" if option == "--remove" else "obstacle back") for option, t in moves]
    present = True
    for option, _ in sorted(moves, key=lambda m: m[1]):
        present = option == "--place"

    missing = [c for c in args.compare if not Path(c).is_file()]
    if missing:
        sys.exit(f"--compare {missing[0]}: no such file. Run that simulation first.")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    trace = out.with_suffix(".csv")
    picture = out.with_suffix(".svg")
    transcript = out.with_suffix(".log")

    def ms(s):
        return str(round(s * 1000))
    command = ["--trace", str(trace), "--transcript", str(transcript),
               "--duration", ms(args.duration), "--scene", args.scene,
               "--heading", str(args.heading), "--seed", str(args.seed)]
    if args.start is not None:
        command += ["--start", str(args.start)]
    for a, b in doors:
        command += ["--door", f"{a}:{b}"]
    for when, line in sends:
        command += ["--send", f"{ms(when)}:{line}"]
    for a, b in offline:
        command += ["--offline", f"{ms(a)}:{ms(b)}"]
    for option, t in moves:
        command += [option, ms(t)]
    for sensor, a, b in unplugged:
        command += ["--unplug", f"{sensor}:{ms(a)}:{ms(b)}"]

    with tempfile.TemporaryDirectory() as workspace:
        binary = build(Path(args.robot), args.defines, Path(workspace))
        # Print the run's output as it comes, and keep the line that says
        # how it ended for the picture.
        ending, geometry = None, None
        world = {"name": args.scene, "walls": [], "wall_scene": scene["wall"], "obstacle_at_end": present}
        with subprocess.Popen([str(binary), *command], stdout=subprocess.PIPE, text=True) as running:
            for line in running.stdout:
                kind, _, rest = line.partition(" ")
                if kind == "robot" and rest.startswith("{"):
                    r = json.loads(rest)
                    geometry = Geometry(r["half_length"], r["half_width"], r["side_ahead"],
                                        r["side_cone"], r["front_cone"])
                    continue
                if kind == "wall" and rest.startswith("{"):
                    world["walls"].append(json.loads(rest))
                    continue
                if kind == "scene" and rest.startswith("{"):
                    continue
                print(line, end="", flush=True)
                if line.startswith("end      "):
                    ending = line[9:].strip()
                elif line.startswith("stuck    "):
                    ending = "stuck: a loop in Robot.h never ends"
        status = running.returncode

    if status == 2:
        sys.exit(1)                     # the reason is printed already
    if status < 0:
        name = signal.Signals(-status).name
        ending = f"crashed ({name})"
        print(f"crashed  {name} in the robot's code", end="")
        if name in ("SIGSEGV", "SIGBUS"):
            print(": it used memory it should not touch, often through an array index out of range."
                  "\n         On the robot this can freeze the program or corrupt other variables.")
        elif name == "SIGFPE":
            print(": an integer division by zero. Check the divisor before dividing.")
        else:
            print()
    elif status not in (0, 3):
        print(f"error    the simulation stopped with status {status}")

    others = [(Path(c).stem, read_trace(c)) for c in args.compare]
    rows = read_trace(trace)
    if not rows:
        sys.exit("the run ended before recording anything")
    draw(rows, others, world, events, out.stem, ending, geometry, picture)
    print(f"wrote    {trace}\n         {picture}\n         {transcript}")
    if status != 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
