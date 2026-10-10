"""Obstacle courses for the benchmark. The robot starts at the origin facing +x."""

from dataclasses import dataclass

GOAL = (6.0, 0.0)


@dataclass(frozen=True)
class Obstacle:
    kind: str  # "box": plain grey block YOLO cannot name; "person": photo billboard YOLO can detect
    x: float
    y: float
    sx: float = 0.3  # half extents (m)
    sy: float = 0.3
    sz: float = 0.5


def person(x, y):
    return Obstacle("person", x, y, sx=0.03, sy=0.3, sz=0.85)


SCENARIOS = {
    "clear": [],
    "person_ahead": [person(2.5, 0.0)],
    "box_ahead": [Obstacle("box", 2.5, 0.0, sy=0.6)],
    "slalom": [person(2.0, 0.35), Obstacle("box", 3.5, -0.45, sy=0.45), person(5.0, 0.3)],
    "dead_end": [
        Obstacle("box", 1.7, 0.0, sx=0.1, sy=1.0),
        Obstacle("box", 0.8, 1.0, sx=1.0, sy=0.1),
        Obstacle("box", 0.8, -1.0, sx=1.0, sy=0.1),
    ],
}


def clearance(x, y, obstacles):
    """Distance from (x, y) to the nearest obstacle footprint (negative inside)."""
    best = float("inf")
    for o in obstacles:
        dx, dy = abs(x - o.x) - o.sx, abs(y - o.y) - o.sy
        outside = (max(dx, 0.0) ** 2 + max(dy, 0.0) ** 2) ** 0.5
        best = min(best, outside if (dx > 0 or dy > 0) else max(dx, dy))
    return best


def world_xml(obstacles, person_texture):
    geoms = []
    for i, o in enumerate(obstacles):
        material = "person" if o.kind == "person" else "block"
        geoms.append(
            f'<geom name="obstacle_{i}" type="box" pos="{o.x} {o.y} {o.sz}" '
            f'size="{o.sx} {o.sy} {o.sz}" material="{material}"/>'
        )
    gx, gy = GOAL
    return f"""<mujoco model="jev_world">
  <statistic center="0 0 1" extent="1.8" meansize="0.04"/>
  <visual>
    <headlight ambient=".5 .5 .5" diffuse=".7 .7 .7" specular=".1 .1 .1"/>
    <global azimuth="160" elevation="-20"/>
  </visual>
  <asset>
    <texture builtin="gradient" height="512" rgb1="0.45 0.59 0.79" rgb2="0.30 0.41 0.60" type="skybox" width="512"/>
    <texture builtin="checker" height="300" width="300" name="floor" rgb1="0.82 0.82 0.80" rgb2="0.72 0.72 0.70" type="2d"/>
    <material name="floor" texrepeat="8 8" texture="floor" texuniform="true"/>
    <texture name="person" type="cube" file="{person_texture}"/>
    <material name="person" texture="person"/>
    <material name="block" rgba="0.45 0.45 0.48 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" diffuse=".6 .6 .6"/>
    <geom name="floor" type="plane" size="0 0 0.01" material="floor" conaffinity="0" contype="1"/>
    <site name="goal" pos="{gx} {gy} 0.01" size="0.25 0.005" type="cylinder" rgba="0.1 0.8 0.2 0.6"/>
    {chr(10).join("    " + g for g in geoms).strip()}
  </worldbody>
</mujoco>
"""
