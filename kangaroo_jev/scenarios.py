"""Courses for the benchmark: reach the red cube, which is in view from the start. The robot starts at the origin facing +x,
on a surface at z = 0.

Raised courses are built from deck tiles (top at z = 0) over a floor 1.5 m below: walking off a
tile is a fall, which ends the episode.
"""

from dataclasses import dataclass, field

DROP = 1.5  # m from the deck down to the floor below


@dataclass(frozen=True)
class Obstacle:
    kind: str  # "box"/"wall": plain grey, YOLO cannot name them; "person": photo billboard YOLO can detect
    x: float
    y: float
    sx: float = 0.3  # half extents (m)
    sy: float = 0.3
    sz: float = 0.5


@dataclass(frozen=True)
class Scenario:
    goal: tuple  # where the red cube stands; the robot has to find it with its camera
    obstacles: list = field(default_factory=list)
    deck: list | None = None  # walkable tiles (x0, x1, y0, y1); None = an endless floor


CUBE = 0.2  # half size of the red goal cube (m)
PEDESTAL = 0.9  # the cube stands on a grey pedestal this tall, so it stays in view over 1 m walls


def person(x, y):
    return Obstacle("person", x, y, sx=0.03, sy=0.3, sz=0.85)


def wall(x0, x1, y0, y1):
    """A 1 m wall: below the camera, so the cube stays visible over it, but the robot cannot pass."""
    return Obstacle("wall", (x0 + x1) / 2, (y0 + y1) / 2, sx=(x1 - x0) / 2, sy=(y1 - y0) / 2, sz=0.5)


SCENARIOS = {
    # L-shaped walkway, 1.4 m wide: the cube is in view up the left branch, but the straight line to it is a drop.
    "walkway_turn": Scenario(goal=(4.8, 3.0), deck=[(-1.0, 5.5, -0.7, 0.7), (4.1, 5.5, 0.7, 4.0)]),
    # 1 m bridge, offset to the left of the straight line to the cube.
    "narrow_bridge": Scenario(goal=(6.0, 0.0), deck=[(-1.0, 1.5, -1.5, 1.5), (1.5, 4.0, 0.3, 1.3), (4.0, 7.0, -1.5, 1.5)]),
    # 2 x 2 m hole right between the robot and the cube.
    "pit": Scenario(goal=(6.0, 0.0), deck=[(-2.0, 9.0, -4.0, -1.0), (-2.0, 9.0, 1.0, 4.0), (-2.0, 2.0, -1.0, 1.0),
                                           (4.0, 9.0, -1.0, 1.0)]),
    # 2 m walkway with two people and a block on it.
    "crowded_walkway": Scenario(goal=(6.5, 0.0), deck=[(-1.0, 7.2, -1.0, 1.0)],
                                obstacles=[person(2.2, 0.4), Obstacle("box", 3.8, -0.55), person(5.3, 0.45)]),
    # A wall between the robot and the cube; the way through is a 1.2 m gap off to the left.
    "wall_gap": Scenario(goal=(5.0, 0.0), obstacles=[wall(2.4, 2.6, -3.0, 1.2), wall(2.4, 2.6, 2.4, 4.0)]),
    # Two staggered walls: the robot has to weave right, then left.
    "staggered_walls": Scenario(goal=(6.0, 0.0), obstacles=[wall(1.9, 2.1, -0.4, 3.0), wall(3.9, 4.1, -3.0, 0.4)]),
}


def clearance(x, y, obstacles):
    """Distance from (x, y) to the nearest obstacle footprint (negative inside)."""
    best = float("inf")
    for o in obstacles:
        dx, dy = abs(x - o.x) - o.sx, abs(y - o.y) - o.sy
        outside = (max(dx, 0.0) ** 2 + max(dy, 0.0) ** 2) ** 0.5
        best = min(best, outside if (dx > 0 or dy > 0) else max(dx, dy))
    return best


def world_xml(scenario, person_texture):
    geoms = []
    if scenario.deck is None:
        geoms.append('<geom name="floor" type="plane" size="0 0 0.01" material="floor" conaffinity="0" contype="1"/>')
    else:
        geoms.append(f'<geom name="floor" type="plane" pos="0 0 -{DROP}" size="0 0 0.01" material="below"/>')
        for i, (x0, x1, y0, y1) in enumerate(scenario.deck):
            geoms.append(f'<geom name="deck_{i}" type="box" pos="{(x0 + x1) / 2} {(y0 + y1) / 2} -0.15" '
                         f'size="{(x1 - x0) / 2} {(y1 - y0) / 2} 0.15" material="floor"/>')
    for i, o in enumerate(scenario.obstacles):
        material = "person" if o.kind == "person" else "block"
        geoms.append(f'<geom name="obstacle_{i}" type="box" pos="{o.x} {o.y} {o.sz}" '
                     f'size="{o.sx} {o.sy} {o.sz}" material="{material}"/>')
    gx, gy = scenario.goal
    geoms.append(f'<geom name="pedestal" type="box" pos="{gx} {gy} {PEDESTAL / 2}" size="0.15 0.15 {PEDESTAL / 2}" material="block"/>')
    geoms.append(f'<geom name="goal_cube" type="box" pos="{gx} {gy} {PEDESTAL + CUBE}" size="{CUBE} {CUBE} {CUBE}" '
                 f'rgba="0.85 0.08 0.06 1"/>')
    body = "\n    ".join(geoms)
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
    <texture builtin="checker" height="300" width="300" name="below" rgb1="0.36 0.33 0.30" rgb2="0.30 0.28 0.26" type="2d"/>
    <material name="below" texrepeat="4 4" texture="below" texuniform="true"/>
    <texture name="person" type="cube" file="{person_texture}"/>
    <material name="person" texture="person"/>
    <material name="block" rgba="0.45 0.45 0.48 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" diffuse=".6 .6 .6"/>
    {body}
  </worldbody>
</mujoco>
"""
