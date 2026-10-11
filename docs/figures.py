"""Figures for REPORT.md from a benchmark results dir. Runs in the container:

    docker compose run --rm kangaroo python3 docs/figures.py results/<run>

Writes docs/img/: course and camera photos (offline MuJoCo renders of the benchmark scenes, film strips
re-rendered from the logged base poses with the joints at the start posture) and SVG charts.
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

from kangaroo_jev.brains import MARGIN  # noqa: E402
from kangaroo_jev.scenarios import SCENARIOS  # noqa: E402

RUN = Path(sys.argv[1])
OUT = Path(__file__).parent / "img"
BLUE, GREY, INK, QUIET, RULE, RED = "#2f6fdf", "#8a9099", "#1d1f23", "#6b7078", "#e4e6ea", "#d0352b"
BRAINS = {"jev": BLUE, "heuristic": GREY}
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 34)
except OSError:
    FONT = ImageFont.load_default(size=34)
rows = {(r["scenario"], r["brain"]): r for r in csv.DictReader(open(RUN / "episodes.csv"))}
courses = [c for c in SCENARIOS if (c, "jev") in rows]


def steps(course, brain):
    f = RUN / f"{course}_{brain}.jsonl"
    return [json.loads(line) for line in open(f)] if f.exists() else []


# ---- photos ----------------------------------------------------------------------------------------
def scene_dir():
    from ament_index_python.packages import get_package_share_directory as share

    from kangaroo_jev.sim_setup import VARIANT, setup

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
    from kangaroo_jev.perception import YOLO_CONFIDENCE

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


def film_strip(course):
    """What the head camera saw along each brain's path, plus where it ended."""
    r = Renderer(course)
    strips = []
    for brain in BRAINS:
        log = steps(course, brain)
        picks = sorted({0, len(log) // 3, 2 * len(log) // 3, len(log) - 1})
        frames, labels = [], []
        for i in picks:
            x, y, z, yaw = log[i]["pose"]
            r.place(x, y, z, yaw)
            frames.append(np.array(Image.fromarray(yolo_overlay(r.camera())).resize((424, 240))))
            labels.append(f"{'Jev' if brain == 'jev' else 'rule'} · {log[i]['t']:.0f} s")
        end = r.overview([x, y, 0.3], distance=5.5, azimuth=math.degrees(yaw) + 200, elevation=-35)
        frames.append(np.array(Image.fromarray(end).resize((424, 240))))
        labels.append(f"{'Jev' if brain == 'jev' else 'rule'} · end")
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


def num(v):
    return None if v in (None, "", "None") else float(v)


def progress_chart():
    width, left, bar, gap = 760, 150, 13, 14
    height = 70 + len(courses) * (2 * bar + gap)
    X = lambda p: left + p * (width - left - 170)
    mean = {b: np.mean([num(rows[(c, b)]["progress"]) or 0 for c in courses]) for b in BRAINS}
    body = [text(0, 18, "Progress towards the cube per course (90 s per episode)", 15, weight=600),
            text(0, 38, f"Share of the start-to-cube distance covered at best. Mean: Jev {mean['jev']:.0%}, rule "
                        f"{mean['heuristic']:.0%}.", 12, QUIET),
            f"<rect x='{width - 110}' y='8' width='10' height='10' fill='{BLUE}'/>", text(width - 95, 17, "Jev", 11.5),
            f"<rect x='{width - 60}' y='8' width='10' height='10' fill='{GREY}'/>", text(width - 45, 17, "rule", 11.5)]
    for k in (0, 0.5, 1):
        body += [f"<line x1='{X(k)}' x2='{X(k)}' y1='50' y2='{height - 14}' stroke='{RULE}'/>",
                 text(X(k), height - 2, f"{k:.0%}", 10.5, QUIET, "middle")]
    y = 58
    for c in courses:
        body.append(text(left - 10, y + bar + 4, c.replace("_", " "), 12, INK, "end"))
        for b, color in BRAINS.items():
            r = rows[(c, b)]
            p = num(r["progress"]) or 0
            label = "reached" if r["outcome"] == "reached" else r["outcome"].replace("fell off an edge", "fell off")
            body += [f"<rect x='{left}' y='{y}' width='{max(X(p) - left, 1):.1f}' height='{bar}' fill='{color}' rx='2'/>",
                     text(X(p) + 6, y + bar - 2, f"{p:.0%} · {label}", 11, BLUE if b == "jev" else QUIET)]
            y += bar
        y += gap
    svg("progress.svg", width, height, body)


def distance_chart():
    cw, ch, cols = 250, 135, 3
    body = [text(0, 16, "Distance to the cube over time", 15, weight=600),
            text(0, 34, "Blue Jev, grey rule; dashed red: reached (0.9 m).", 12, QUIET)]
    for i, c in enumerate(courses):
        ox, oy = (i % cols) * (cw + 5), 48 + (i // cols) * (ch + 10)
        gx, gy = SCENARIOS[c].goal
        series = {b: [(s["t"], math.hypot(gx - s["pose"][0], gy - s["pose"][1])) for s in steps(c, b)] for b in BRAINS}
        tmax = max([t for s in series.values() for t, _ in s] + [10])
        dmax = max([d for s in series.values() for _, d in s] + [1])
        X = lambda t: ox + 30 + t / tmax * (cw - 40)
        Y = lambda d: oy + ch - 22 - d / dmax * (ch - 44)
        body += [text(ox, oy + 12, c.replace("_", " "), 12, weight=600),
                 f"<line x1='{X(0)}' x2='{X(tmax)}' y1='{Y(0.9):.1f}' y2='{Y(0.9):.1f}' stroke='{RED}' stroke-dasharray='3 3'/>",
                 f"<line x1='{X(0)}' x2='{X(tmax)}' y1='{Y(0)}' y2='{Y(0)}' stroke='{RULE}'/>",
                 text(X(0) - 4, Y(dmax) + 4, f"{dmax:.0f} m", 9.5, QUIET, "end"),
                 text(X(tmax), Y(0) + 14, f"{tmax:.0f} s", 9.5, QUIET, "end")]
        for b, color in BRAINS.items():
            if series[b]:
                pts = " ".join(f"{X(t):.1f},{Y(d):.1f}" for t, d in series[b])
                body.append(f"<polyline points='{pts}' fill='none' stroke='{color}' stroke-width='2'/>")
    svg("distance.svg", cols * (cw + 5), 48 + math.ceil(len(courses) / cols) * (ch + 10), body)


def trajectories_chart():
    cw, cols = 250, 3
    cells, heights = [], []
    for c in courses:
        sc = SCENARIOS[c]
        trails = {b: [s["pose"] for s in steps(c, b)] for b in BRAINS}
        xs = [0.0, sc.goal[0]] + [v for t in (sc.deck or []) for v in t[:2]] + [o.x + k * o.sx for o in sc.obstacles for k in (-1, 1)]
        ys = [0.0, sc.goal[1]] + [v for t in (sc.deck or []) for v in t[2:]] + [o.y + k * o.sy for o in sc.obstacles for k in (-1, 1)]
        xs += [p[0] for t in trails.values() for p in t]
        ys += [p[1] for t in trails.values() for p in t]
        x0, x1, y0, y1 = min(xs) - 0.4, max(xs) + 0.4, min(ys) - 0.4, max(ys) + 0.4
        s = min(cw / (x1 - x0), 160 / (y1 - y0))
        cells.append((c, sc, trails, x0, y1, s))
        heights.append((y1 - y0) * s)
    row_h = [max(heights[i:i + cols]) + 24 for i in range(0, len(courses), cols)]
    body = [text(0, 16, "Paths (top view)", 15, weight=600),
            text(0, 34, "White deck over a 1.5 m drop (dark), grey walls and boxes, orange people, red cube; "
                        "blue Jev, grey rule, black dot start.", 12, QUIET)]
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
            fill = {"person": "#d9a46a", "wall": "#6f757e"}.get(o.kind, "#9aa0a8")
            body.append(f"<rect x='{X(o.x - o.sx):.1f}' y='{Y(o.y + o.sy):.1f}' width='{max(2 * o.sx * s, 2.5):.1f}' "
                        f"height='{2 * o.sy * s:.1f}' fill='{fill}'/>")
        gx, gy = sc.goal
        body += [f"<circle cx='{X(gx):.1f}' cy='{Y(gy):.1f}' r='{0.9 * s:.1f}' fill='none' stroke='{RED}' stroke-dasharray='3 3'/>",
                 f"<rect x='{X(gx - 0.2):.1f}' y='{Y(gy + 0.2):.1f}' width='{0.4 * s:.1f}' height='{0.4 * s:.1f}' fill='{RED}'/>"]
        for b in ("heuristic", "jev"):
            if trails[b]:
                pts = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in trails[b])
                body.append(f"<polyline points='{pts}' fill='none' stroke='{'#7d838c' if b == 'heuristic' else BLUE}' "
                            f"stroke-width='2' stroke-linejoin='round'/>")
        body.append(f"<circle cx='{X(0):.1f}' cy='{Y(0):.1f}' r='3.5' fill='#222'/></g>")
    svg("paths.svg", cols * (cw + 8), 48 + sum(row_h), body)


def calibration_chart():
    pts = [(s["scene"]["options"][m]["free_run_m"], s["answers"]["safe"][m], m)
           for c in courses for s in steps(c, "jev") if s.get("answers") for m in ("forward", "left", "right")]
    walk = [(d, p) for d, p, m in pts if m == "forward"]
    tight = [p for d, p in walk if d <= MARGIN["forward"]]
    roomy = [p for d, p in walk if d > MARGIN["forward"]]
    width, height, left, base = 760, 290, 50, 240
    X = lambda d: left + min(d, 4) / 4.0 * (width - left - 20)
    Y = lambda p: base - p * (base - 60)
    body = [text(0, 18, f"Jev's p(safe) for walking forward: median {np.median(tight):.2f} without room, "
                        f"{np.median(roomy):.2f} with room", 15, weight=600),
            text(0, 38, f"{sum(p >= 0.5 for p in tight)} of {len(tight)} steps without room still called safe; "
                        f"the geometric veto in code blocks them. {len(pts)} answers in total.", 12, QUIET),
            f"<rect x='{X(0)}' y='60' width='{X(MARGIN['forward']) - X(0):.1f}' height='{base - 60}' fill='{RED}' fill-opacity='0.07'/>",
            text(X(MARGIN["forward"]) + 4, 72, f"forward needs {MARGIN['forward']} m", 10.5, "#b03028"),
            text(width - 20, base + 34, "measured free run of the motion (m)", 11, QUIET, "end"),
            text(left - 6, 54, "p(safe)", 11, QUIET, "end")]
    for k in (0, 0.5, 1):
        body += [f"<line x1='{left}' x2='{width - 20}' y1='{Y(k)}' y2='{Y(k)}' stroke='{RULE}'/>",
                 text(left - 6, Y(k) + 4, f"{k:g}", 10.5, QUIET, "end")]
    for d in range(5):
        body.append(text(X(d), base + 16, f"{d} m", 10.5, QUIET, "middle"))
    for d, p, m in pts:
        body.append(f"<circle cx='{X(d):.1f}' cy='{Y(p):.1f}' r='3' fill='{BLUE}' fill-opacity='{0.55 if m == 'forward' else 0.2}'/>")
    body += [f"<circle cx='{width - 200}' cy='{height - 6}' r='3.5' fill='{BLUE}' fill-opacity='0.55'/>", text(width - 192, height - 2, "forward", 11),
             f"<circle cx='{width - 120}' cy='{height - 6}' r='3.5' fill='{BLUE}' fill-opacity='0.2'/>", text(width - 112, height - 2, "turns", 11)]
    svg("calibration.svg", width, height, body)


def latency_chart():
    lat = sorted(s["latency_s"] for c in courses for s in steps(c, "jev"))
    bins = [0.15 + 0.01 * i for i in range(21)]
    counts = [sum(lo <= v < lo + 0.01 for v in lat) for lo in bins] + [sum(v >= bins[-1] + 0.01 for v in lat)]
    width, height, left, base = 760, 190, 40, 150
    bw = (width - left - 80) / len(counts)
    body = [text(0, 18, f"Decision latency: {np.median(lat):.2f} s median, {np.percentile(lat, 95):.2f} s p95 "
                        f"({len(lat)} requests, 8 questions each)", 15, weight=600),
            text(0, 38, "The robot decides once per second; latency is not the bottleneck.", 12, QUIET),
            text(width - 20, base + 34, "seconds per decision", 11, QUIET, "end")]
    for i, n in enumerate(counts):
        h = n / max(counts) * (base - 55)
        body.append(f"<rect x='{left + i * bw:.1f}' y='{base - h:.1f}' width='{bw - 2:.1f}' height='{h:.1f}' fill='{BLUE}' fill-opacity='0.75'/>")
        if i % 5 == 0 and i < len(bins):
            body.append(text(left + i * bw, base + 16, f"{bins[i]:.2f}", 10.5, QUIET))
    body.append(text(left + len(bins) * bw, base + 16, f"≥{bins[-1] + 0.01:.2f}", 10.5, QUIET))
    svg("latency.svg", width, height, body)


if __name__ == "__main__":
    from ultralytics import YOLO as _YOLO

    OUT.mkdir(exist_ok=True)
    SCENES = scene_dir()
    YOLO = _YOLO(os.environ["YOLO_MODEL"])
    photos()
    for course in sys.argv[2:] or ["pit", "wall_gap"]:
        film_strip(course)
    progress_chart(), distance_chart(), trajectories_chart(), calibration_chart(), latency_chart()
    owner = Path(__file__).stat()  # the container runs as root; hand the files to the repo's owner
    for p in [OUT, *OUT.iterdir()]:
        os.chown(p, owner.st_uid, owner.st_gid)
    print("wrote", sorted(p.name for p in OUT.iterdir()))
