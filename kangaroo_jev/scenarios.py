"""Courses for the benchmark. The robot starts at the origin facing +x on a surface at z = 0.

Raised courses are built from deck tiles (top at z = 0) over a floor 1.5 m below: walking off a
tile is a fall, which ends the episode.
"""

from dataclasses import dataclass, field

DROP = 1.5  # m from the deck down to the floor below


@dataclass(frozen=True)
class Obstacle:
    kind: str  # "box": plain grey block YOLO cannot name; "person": photo billboard YOLO can detect
    x: float
    y: float
    sx: float = 0.3  # half extents (m)
    sy: float = 0.3
    sz: float = 0.5


@dataclass(frozen=True)
class Scenario:
    goal: tuple
    obstacles: list = field(default_factory=list)
    deck: list | None = None  # walkable tiles (x0, x1, y0, y1); None = an endless floor


def person(x, y):
    return Obstacle("person", x, y, sx=0.03, sy=0.3, sz=0.85)


SCENARIOS = {
    # L-shaped walkway, 1.4 m wide: the goal is up the left branch, straight on is a drop.
    "walkway_turn": Scenario(goal=(2.8, 3.9), deck=[(-1.0, 3.5, -0.7, 0.7), (2.1, 3.5, 0.7, 4.6)]),
    # 1 m bridge, offset to the left of the straight line to the goal.
    "narrow_bridge": Scenario(goal=(6.0, 0.0), deck=[(-1.0, 1.5, -1.5, 1.5), (1.5, 4.0, 0.3, 1.3), (4.0, 7.0, -1.5, 1.5)]),
    # 2 x 2 m hole in the middle of a wide floor, right on the straight line to the goal.
    "pit": Scenario(goal=(6.0, 0.0), deck=[(-2.0, 9.0, -4.0, -1.0), (-2.0, 9.0, 1.0, 4.0), (-2.0, 2.0, -1.0, 1.0),
                                           (4.0, 9.0, -1.0, 1.0)]),
    # 2 m walkway with people and a block on it: pass them without stepping off.
    "crowded_walkway": Scenario(goal=(6.5, 0.0), deck=[(-1.0, 7.2, -1.0, 1.0)],
                                obstacles=[person(2.2, 0.4), Obstacle("box", 3.8, -0.55), person(5.3, 0.45)]),
    # Flat reference: no edges, people and a block.
    "slalom": Scenario(goal=(6.0, 0.0),
                       obstacles=[person(2.0, 0.35), Obstacle("box", 3.5, -0.45, sy=0.45), person(5.0, 0.3)]),
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
    <site name="goal" pos="{gx} {gy} 0.01" size="0.25 0.005" type="cylinder" rgba="0.1 0.8 0.2 0.6"/>
    {body}
  </worldbody>
</mujoco>
"""
