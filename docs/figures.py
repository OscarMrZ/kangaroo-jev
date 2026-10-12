"""Figures for REPORT.md from a benchmark results dir. Runs in the container:

    docker compose run --rm kangaroo python3 docs/figures.py results/<run> [course:seed ...]

Writes docs/img/: course and camera photos (offline MuJoCo renders of the benchmark scenes at the nominal start; film
strips of the given episodes re-rendered from their logged base poses, joints at the start posture) and SVG charts over
all seeds.
"""

import csv
import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from kangaroo_systemone.brains import MARGIN  # noqa: E402
from kangaroo_systemone.scenarios import SCENARIOS  # noqa: E402

RUN = Path(sys.argv[1])
OUT = Path(__file__).parent / "img"
BLUE, GREY, INK, QUIET, RULE, RED = "#2f6fdf", "#8a9099", "#1d1f23", "#6b7078", "#e4e6ea", "#d0352b"
ORANGE, GREEN, MAGENTA, PURPLE = "#e07b22", "#2a9d6f", "#c2408f", "#6a4fc4"
# Plotting order. "naive": Jev's eight questions; "native": the CLM-native Choice (kangaroo_systemone/brains.py).
COLORS = {"jev": BLUE, "clm": ORANGE, "clm_native": GREEN, "clm_image_native": MAGENTA, "clm_image_generic": PURPLE,
          "heuristic": GREY}
NAMES = {"jev": "Jev", "clm": "CLM naive", "clm_native": "CLM native", "clm_image_native": "CLM image native",
         "clm_image_generic": "CLM image generic", "heuristic": "rule"}
COLOR_WORDS = {BLUE: "blue", ORANGE: "orange", GREEN: "green", MAGENTA: "magenta", PURPLE: "purple", GREY: "grey"}
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 34)
except OSError:
    FONT = ImageFont.load_default(size=34)
rows = [r for r in csv.DictReader(open(RUN / "episodes.csv")) if r.get("progress")]
courses = [c for c in SCENARIOS if any(r["scenario"] == c for r in rows)]
BRAINS = {b: c for b, c in COLORS.items() if any(r["brain"] == b for r in rows)}  # in plotting order
seeds = sorted({int(r["seed"]) for r in rows})


def episodes(course, brain):
    return [r for r in rows if r["scenario"] == course and r["brain"] == brain]


def steps(course, brain, seed):
    f = RUN / f"{course}_{brain}_s{seed}.jsonl"
    return [json.loads(line) for line in open(f)] if f.exists() else []


def all_steps(course, brain):
    return [s for seed in seeds for s in steps(course, brain, seed)]


# ---- photos ----------------------------------------------------------------------------------------
def scene_dir():
    from ament_index_python.packages import get_package_share_directory as share

    from kangaroo_systemone.sim_setup import VARIANT, setup

    setup(share("kangaroo_mujoco"), share("kangaroo_controller_configuration"))
    return Path(share("kangaroo_mujoco")) / "models" / VARIANT


class Renderer:
    def __init__(self, course):
        self.m = mujoco.MjModel.from_xml_path(str(SCENES / f"scene_jev_{course}.xml"))
        self.d = mujoco.MjData(self.m)
        self.head, self.wide = mujoco.Renderer(self.m, 240, 424), mujoco.Renderer(self.m, 360, 640)

    def place(self, x=0.0, y=0.0, z=None, yaw=0.0):
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        if z is not None:
            self.d.qpos[:3] = x, y, z if z < 0.5 else max(z, 0.86)
        self.d.qpos[3:7] = math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)
        mujoco.mj_forward(self.m, self.d)

    def camera(self):
        self.head.update_scene(self.d, camera="head_camera")
        return self.head.render()

    def overview(self, lookat, distance=8.0, azimuth=20.0, elevation=-38.0):
        cam = mujoco.MjvCamera()
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, distance, azimuth, elevation
        self.wide.update_scene(self.d, camera=cam)
        return self.wide.render()


def yolo_overlay(rgb):
    from kangaroo_systemone.perception import YOLO_CONFIDENCE

    r = YOLO(rgb[..., ::-1], conf=YOLO_CONFIDENCE, retina_masks=True, verbose=False)[0]
    return r.plot()[..., ::-1]


def labelled_grid(images, labels, cols, out, label_h=50):
    w, h = images[0].shape[1], images[0].shape[0]
    rows_n = math.ceil(len(images) / cols)
    sheet = Image.new("RGB", (cols * w + (cols - 1) * 6, rows_n * (h + label_h)), "white")
    draw = ImageDraw.Draw(sheet)
    for i, (im, label) in enumerate(zip(images, labels)):
        x, y = (i % cols) * (w + 6), (i // cols) * (h + label_h)
        sheet.paste(Image.fromarray(im), (x, y))
        draw.text((x + 6, y + h + 6), label, fill=INK, font=FONT)
    sheet.save(out, quality=82)


def photos():
    overviews, cameras = [], []
    for c in courses:
        r = Renderer(c)
        r.place()
        gx, gy = SCENARIOS[c].goal
        overviews.append(r.overview([gx / 2, gy / 2, -0.3], azimuth=-20 if c == "walkway_turn" else 20))
        cameras.append(yolo_overlay(r.camera()))
    names = [c.replace("_", " ") for c in courses]
    labelled_grid(overviews, names, 3, OUT / "courses.jpg")
    labelled_grid(cameras, names, 3, OUT / "camera_start.jpg")


def film_strip(course, seed):
    """What the head camera saw along each brain's path, plus where it ended. Logged poses are in the course frame,
    which is the world frame of the seed-0 scenes."""
    r = Renderer(course)
    strips = []
    for brain in BRAINS:
        log = steps(course, brain, seed)
        outcome = next(e["outcome"] for e in episodes(course, brain) if int(e["seed"]) == seed)
        picks = sorted({0, len(log) // 3, 2 * len(log) // 3, len(log) - 1})
        frames, labels = [], []
        for i in picks:
            x, y, z, yaw = log[i]["pose"]
            r.place(x, y, z, yaw)
            frames.append(np.array(Image.fromarray(yolo_overlay(r.camera())).resize((424, 240))))
            labels.append(f"{NAMES[brain]} · move {i + 1}")
        end = r.overview([x, y, 0.3], distance=5.5, azimuth=math.degrees(yaw) + 200, elevation=-35)
        frames.append(np.array(Image.fromarray(end).resize((424, 240))))
        labels.append(f"{NAMES[brain]} · {outcome}")
        strips += list(zip(frames, labels))
    labelled_grid([f for f, _ in strips], [lab for _, lab in strips], 5, OUT / f"film_{course}.jpg", label_h=46)


# ---- charts (standalone SVG) -----------------------------------------------------------------------
def text(x, y, t, size=12, fill=INK, anchor="start", weight=400):
    return (f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' fill='{fill}' text-anchor='{anchor}' "
            f"font-weight='{weight}'>{t}</text>")


def svg(name, width, height, body):
    (OUT / name).write_text(f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {width} {height:.0f}' width='{width}' "
                            f"font-family='Helvetica, Arial, sans-serif'><rect width='{width}' height='{height:.0f}' "
                            f"fill='white'/>{''.join(body)}</svg>")


def legend():
    return ", ".join(f"{COLOR_WORDS[c]} {NAMES[b]}" for b, c in BRAINS.items())


def num(v):
    return None if v in (None, "", "None") else float(v)


def progress_chart():
    width, left, bar, gap = 760, 150, 15, 16
    height = 74 + len(courses) * (len(BRAINS) * bar + gap)
    X = lambda p: left + p * (width - left - 190)
    prog = {(c, b): [num(r["progress"]) for r in episodes(c, b)] for c in courses for b in BRAINS}
    pooled = {b: [p for c in courses for p in prog[(c, b)]] for b in BRAINS}
    body = [text(0, 18, f"Progress towards the cube ({len(seeds)} seeds per course, 90 moves per episode)", 15, weight=600),
            text(0, 38, "Share of the start-to-cube distance covered at best (100% = reached); bar: mean, dots: seeds.", 12, QUIET)]
    x = left
    for b, color in BRAINS.items():
        body += [f"<rect x='{x}' y='50' width='10' height='10' fill='{color}'/>", text(x + 15, 59, NAMES[b], 11.5)]
        x += 30 + 6.5 * len(NAMES[b])
    for k in (0, 0.5, 1):
        body += [f"<line x1='{X(k)}' x2='{X(k)}' y1='56' y2='{height - 14}' stroke='{RULE}'/>",
                 text(X(k), height - 2, f"{k:.0%}", 10.5, QUIET, "middle")]
    y = 64
    for c in courses:
        body.append(text(left - 10, y + bar + 5, c.replace("_", " "), 12, INK, "end"))
        for b, color in BRAINS.items():
            ps = prog[(c, b)]
            m = np.mean(ps)
            body.append(f"<rect x='{left}' y='{y + 2}' width='{max(X(m) - left, 1):.1f}' height='{bar - 4}' fill='{color}' "
                        f"fill-opacity='0.35' rx='2'/>")
            body += [f"<circle cx='{X(p):.1f}' cy='{y + bar / 2:.1f}' r='3.6' fill='{color}'/>" for p in ps]
            reached = sum(r["outcome"] == "reached" for r in episodes(c, b))
            body.append(text(X(max(ps)) + 9, y + bar - 3, f"{m:.0%} · reached {reached}/{len(ps)}", 11,
                             QUIET if b == "heuristic" else color))
            y += bar
        y += gap
    svg("progress.svg", width, height, body)


def distance_chart():
    cw, ch, cols = 250, 135, 3
    body = [text(0, 16, "Distance to the cube over time", 15, weight=600),
            text(0, 34, f"One line per seed. {legend()}; dashed red: reached (0.9 m). Lines leaving the top "
                        "walk away from it.", 12, QUIET)]
    for i, c in enumerate(courses):
        ox, oy = (i % cols) * (cw + 5), 48 + (i // cols) * (ch + 10)
        gx, gy = SCENARIOS[c].goal
        series = {(b, seed): [(s["t"], math.hypot(gx - s["pose"][0], gy - s["pose"][1])) for s in steps(c, b, seed)]
                  for b in BRAINS for seed in seeds}
        tmax = max([t for s in series.values() for t, _ in s] + [10])
        dmax = math.ceil(max([s[0][1] for s in series.values() if s] + [1]) + 1)
        X = lambda t: ox + 30 + t / tmax * (cw - 40)
        Y = lambda d: oy + ch - 22 - d / dmax * (ch - 44)
        body += [text(ox, oy + 12, c.replace("_", " "), 12, weight=600),
                 f"<line x1='{X(0)}' x2='{X(tmax)}' y1='{Y(0.9):.1f}' y2='{Y(0.9):.1f}' stroke='{RED}' stroke-dasharray='3 3'/>",
                 f"<line x1='{X(0)}' x2='{X(tmax)}' y1='{Y(0)}' y2='{Y(0)}' stroke='{RULE}'/>",
                 text(X(0) - 4, Y(dmax) + 4, f"{dmax:.0f} m", 9.5, QUIET, "end"),
                 text(X(tmax), Y(0) + 14, f"{tmax:.0f} s", 9.5, QUIET, "end"),
                 f"<clipPath id='dclip{i}'><rect x='{X(0) - 2}' y='{Y(dmax)}' width='{cw}' height='{Y(0) - Y(dmax) + 2}'/>"
                 f"</clipPath><g clip-path='url(#dclip{i})'>"]
        for (b, seed), line in sorted(series.items(), key=lambda kv: kv[0][0] != "heuristic"):
            if line:
                pts = " ".join(f"{X(t):.1f},{Y(d):.1f}" for t, d in line)
                body.append(f"<polyline points='{pts}' fill='none' stroke='{BRAINS[b]}' stroke-width='1.4' "
                            f"stroke-opacity='0.7'/>")
        body.append("</g>")
    svg("distance.svg", cols * (cw + 5), 48 + math.ceil(len(courses) / cols) * (ch + 10), body)


def trajectories_chart():
    cw, cols = 250, 3
    cells, heights = [], []
    for c in courses:
        sc = SCENARIOS[c]
        trails = {(b, seed): [s["pose"] for s in steps(c, b, seed)] for b in BRAINS for seed in seeds}
        xs = [0.0, sc.goal[0]] + [v for t in (sc.deck or []) for v in t[:2]] + [o.x + k * o.sx for o in sc.obstacles for k in (-1, 1)]
        ys = [0.0, sc.goal[1]] + [v for t in (sc.deck or []) for v in t[2:]] + [o.y + k * o.sy for o in sc.obstacles for k in (-1, 1)]
        x0, x1, y0, y1 = min(xs) - 1.0, max(xs) + 1.0, min(ys) - 1.0, max(ys) + 1.0  # paths leaving it are clipped
        s = min(cw / (x1 - x0), 160 / (y1 - y0))
        cells.append((c, sc, trails, x0, y1, s))
        heights.append((y1 - y0) * s)
    row_h = [max(heights[i:i + cols]) + 24 for i in range(0, len(courses), cols)]
    body = [text(0, 16, "Paths (top view)", 15, weight=600),
            text(0, 34, "Deck white over a 1.5 m drop, walls grey, people brown, cube red; one line per seed: "
                        f"{legend()}; dots: starts.", 12, QUIET)]
    for i, (c, sc, trails, x0, y1, s) in enumerate(cells):
        ox, oy = (i % cols) * (cw + 8), 48 + sum(row_h[: i // cols])
        X = lambda x: ox + (x - x0) * s
        Y = lambda y: oy + 20 + (y1 - y) * s
        hgt = heights[i]
        body += [text(ox, oy + 13, c.replace("_", " "), 12, weight=600),
                 f"<clipPath id='clip{i}'><rect x='{ox}' y='{oy + 20}' width='{cw}' height='{hgt:.1f}'/></clipPath>",
                 f"<g clip-path='url(#clip{i})'>",
                 f"<rect x='{ox}' y='{oy + 20}' width='{cw}' height='{hgt:.1f}' fill='{'#5b5650' if sc.deck else '#f4f4f2'}'/>"]
        for a, b, c0, d in sc.deck or []:
            body.append(f"<rect x='{X(a):.1f}' y='{Y(d):.1f}' width='{(b - a) * s:.1f}' height='{(d - c0) * s:.1f}' fill='#f2f2ef'/>")
        for o in sc.obstacles:
            fill = {"person": "#a0663a", "wall": "#6f757e"}.get(o.kind, "#9aa0a8")
            body.append(f"<rect x='{X(o.x - o.sx):.1f}' y='{Y(o.y + o.sy):.1f}' width='{max(2 * o.sx * s, 2.5):.1f}' "
                        f"height='{2 * o.sy * s:.1f}' fill='{fill}'/>")
        gx, gy = sc.goal
        body += [f"<circle cx='{X(gx):.1f}' cy='{Y(gy):.1f}' r='{0.9 * s:.1f}' fill='none' stroke='{RED}' stroke-dasharray='3 3'/>",
                 f"<rect x='{X(gx - 0.2):.1f}' y='{Y(gy + 0.2):.1f}' width='{0.4 * s:.1f}' height='{0.4 * s:.1f}' fill='{RED}'/>"]
        for (b, seed), trail in sorted(trails.items(), key=lambda kv: kv[0][0] != "heuristic"):
            if trail:
                pts = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in trail)
                body.append(f"<polyline points='{pts}' fill='none' stroke='{'#7d838c' if b == 'heuristic' else COLORS[b]}' "
                            f"stroke-width='1.4' stroke-opacity='0.75' stroke-linejoin='round'/>")
        body += [f"<circle cx='{X(t[0][0]):.1f}' cy='{Y(t[0][1]):.1f}' r='2.5' fill='#222'/>" for t in trails.values() if t]
        body.append("</g>")
    svg("paths.svg", cols * (cw + 8), 48 + sum(row_h), body)


MODELS = ["jev", "clm"]  # decision makers that answer the safety and progress questions (Jev's eight)


def calibration_chart():
    """p(safe) for walking forward against the measured free run, one panel per model."""
    width, left, panel = 760, 50, 240
    models = [b for b in MODELS if b in BRAINS]
    X = lambda d: left + min(d, 4) / 4.0 * (width - left - 20)
    body = []
    for k, b in enumerate(models):
        top = k * (panel + 20)
        base = top + panel - 40
        Y = lambda p: base - p * (base - top - 60)
        pts = [(s["scene"]["options"]["forward"]["free_run_m"], s["answers"]["safe"]["forward"])
               for c in courses for s in all_steps(c, b) if s.get("answers")]
        tight = [p for d, p in pts if d <= MARGIN["forward"]]
        roomy = [p for d, p in pts if d > MARGIN["forward"]]
        body += [text(0, top + 18, f"{NAMES[b]}'s p(safe) for walking forward: median {np.median(tight):.2f} without "
                                   f"room, {np.median(roomy):.2f} with room", 15, weight=600),
                 text(0, top + 38, f"{sum(p >= 0.5 for p in tight)} of {len(tight)} decisions without room called "
                                   f"safe. One dot per decision.", 12, QUIET),
                 f"<rect x='{X(0)}' y='{top + 60}' width='{X(MARGIN['forward']) - X(0):.1f}' height='{base - top - 60}' "
                 f"fill='{RED}' fill-opacity='0.07'/>",
                 text(X(MARGIN["forward"]) - 4, base - 6, f"forward needs {MARGIN['forward']} m", 10.5, "#b03028", "end"),
                 text(left - 6, top + 54, "p(safe)", 11, QUIET, "end")]
        for v in (0, 0.5, 1):
            body += [f"<line x1='{left}' x2='{width - 20}' y1='{Y(v)}' y2='{Y(v)}' stroke='{RULE}'/>",
                     text(left - 6, Y(v) + 4, f"{v:g}", 10.5, QUIET, "end")]
        for d in range(5):
            body.append(text(X(d), base + 16, f"{d} m", 10.5, QUIET, "middle"))
        body.append(f"<g fill='{COLORS[b]}' fill-opacity='0.3'>" +
                    "".join(f"<circle cx='{X(d):.0f}' cy='{Y(p):.0f}' r='3'/>" for d, p in pts) + "</g>")
    height = len(models) * (panel + 20) + 10
    body.append(text(width - 20, height - 6, "measured free run of walking forward (m)", 11, QUIET, "end"))
    svg("calibration.svg", width, height, body)


def latency_chart():
    """Seconds per decision on a log axis, one histogram per model."""
    width, left, row = 760, 120, 60
    models = [b for b in BRAINS if b != "heuristic"]
    lo, hi = math.log10(0.001), math.log10(10)
    X = lambda v: left + (math.log10(min(max(v, 0.001), 10)) - lo) / (hi - lo) * (width - left - 20)
    edges = [10 ** (lo + i * (hi - lo) / 80) for i in range(81)]
    secs = lambda v: f"{v * 1000:.0f} ms" if v < 0.1 else f"{v:.2f} s"
    body = [text(0, 18, "Decision latency: " + ", ".join(
                f"{NAMES[b]} {secs(np.median([s['latency_s'] for c in courses for s in all_steps(c, b) if s.get('answers')]))}"
                for b in models), 15, weight=600),
            text(0, 38, "Median of moves the model answered. CLM caches embeddings: a situation it has seen before costs "
                        "a few ms.", 12, QUIET)]
    for k, b in enumerate(models):
        lat = [s["latency_s"] for c in courses for s in all_steps(c, b) if s.get("answers")]
        counts = [sum(e0 <= v < e1 for v in lat) for e0, e1 in zip(edges, edges[1:])]
        base = 60 + (k + 1) * row
        body.append(text(left - 8, base - 4, NAMES[b], 12, INK, "end"))
        for (e0, e1), n in zip(zip(edges, edges[1:]), counts):
            h = n / max(counts) * (row - 12)
            body.append(f"<rect x='{X(e0):.1f}' y='{base - h:.1f}' width='{X(e1) - X(e0) - 1:.1f}' height='{h:.1f}' "
                        f"fill='{COLORS[b]}' fill-opacity='0.75'/>")
        body.append(f"<line x1='{left}' x2='{width - 20}' y1='{base}' y2='{base}' stroke='{RULE}'/>")
    base = 60 + len(models) * row
    for v in (0.001, 0.01, 0.1, 1, 10):
        body.append(text(X(v), base + 16, secs(v) if v < 0.1 else f"{v:g} s", 10.5, QUIET, "middle"))
    svg("latency.svg", width, base + 30, body)


if __name__ == "__main__":
    from ultralytics import YOLO as _YOLO

    OUT.mkdir(exist_ok=True)
    SCENES = scene_dir()
    YOLO = _YOLO(os.environ["YOLO_MODEL"])
    photos()
    for spec in sys.argv[2:] or ["pit:3", "wall_gap:0"]:
        course, seed = spec.split(":")
        film_strip(course, int(seed))
    progress_chart(), distance_chart(), trajectories_chart(), calibration_chart(), latency_chart()
    owner = Path(__file__).stat()  # the container runs as root; hand the files to the repo's owner
    for p in [OUT, *OUT.iterdir()]:
        os.chown(p, owner.st_uid, owner.st_gid)
    print("wrote", sorted(p.name for p in OUT.iterdir()))
