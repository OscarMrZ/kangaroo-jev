# kangaroo-jev

How good is [TypeSafe Jev](https://docs.typesafe.ai) as the decision maker for reactive navigation of
PAL Robotics' [Kangaroo](https://github.com/pal-robotics/kangaroo_robot) biped (4dof-arm configuration)?

```
D435 RGB-D (head) ──► YOLO26n-seg + depth geometry + red cube ──► scene JSON: four options
     (MuJoCo)                + 20 s memory of edges/obstacles          │
                                                                       ▼
            one Jev request: Noul "safe?" + Score "progress" per option (8 questions)
                                                                       │
            code: geometric veto ─► Jev veto (p(safe) < 0.3) ─► highest progress
                                                                       │ fixed velocities
PAL 4dof RL locomotion policy (pal_policy_deployer v357) ◄── /cmd_vel ◄┘
```

Walking is not trained here: it is PAL's published 4dof locomotion policy. This is reactive, semantic
navigation: the red goal cube is in view, there is no map or planner, and once per second the robot picks
one of four motions from a structured scene description. A hand-written rule over the same description runs
as a baseline, and is also evaluated on every Jev step so the logs show where they disagree.

## Run

Everything runs in Docker (ROS 2 Humble, MuJoCo, CPU torch).

```bash
docker compose build
export TYPESAFE_API_KEY=...            # https://console.typesafe.ai

# Full benchmark: every scenario × {jev, heuristic}, each episode in a fresh simulation
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench --brains heuristic --scenarios wall_gap
```

Results land in `results/<timestamp>/`: `summary.md`, `episodes.csv`, and one `.jsonl` per episode with
every scene description, decision, raw Jev answers and latency. `docs/report.html` is the latest report.

To watch the MuJoCo viewer (X11/XWayland), allow local clients and pass your display:

```bash
xhost +local:
VIEWER_DISPLAY=$DISPLAY docker compose run --rm kangaroo bash -c \
  "ros2 launch sim/kangaroo_jev.launch.py scenario:=wall_gap & sleep 25; python3 -m kangaroo_jev.navigator --brain jev --scenario wall_gap"
```

## What Jev gets

`kangaroo_jev/perception.py` builds the state; code does all the geometry:

```json
{
  "goal": {"visible": true, "distance_m": 4.6, "bearing_deg": 33},
  "options": {
    "forward": {"path_clear_m": 4.0, "floor_ends_m": 4.0, "free_run_m": 4.0, "goal_bearing_after_deg": 33},
    "left":    {"path_clear_m": 4.0, "floor_ends_m": 1.0, "free_run_m": 1.0, "goal_bearing_after_deg": 5},
    "right":   {"path_clear_m": 4.0, "floor_ends_m": 1.0, "free_run_m": 1.0, "goal_bearing_after_deg": 61},
    "backward": {"path_clear_m": 4.0, "floor_ends_m": 4.0, "free_run_m": 4.0}
  },
  "objects": [{"label": "person", "confidence": 0.86, "distance_m": 2.1, "bearing_deg": 12, "side": "ahead"}],
  "recent_actions": ["forward", "left"]
}
```

Each option is measured along a robot-wide strip in the direction the robot would face after that motion
(turns are ~28°): distance to the nearest obstacle, to where the floor ends (depth points below the deck,
projected back to deck level), and the cube's bearing afterwards. Points seen in the last 20 s are kept in an
odometry frame, so edges stay known inside the camera's ~0.8 m blind zone. The floor plane is fitted to the
bottom of every frame, because a few degrees of torso pitch otherwise turn far floor into phantom obstacles or
edges. The cube is found by colour and placed with depth; its pedestal is not an obstacle; when it leaves the
view, its last position is carried along with the robot's motion (`goal.remembered`) for 20 s. YOLO detections below 0.5 confidence are dropped. A
`legend` explaining the fields rides in the state, which Jev reads once for all questions.

`kangaroo_jev/brains.py` uses TypeSafe patterns rather than one big question:

- **Speculative fan-out**: one request with a `Noul` "is this motion safe?" and a 4-level `Score` "how much does
  it help reach the cube?" for each of the four motions.
- **Rules in code**: a motion is allowed only with enough measured room (`MARGIN`: 0.8 m forward, 0.5 m for turns;
  backward only to undo a step forward, the camera cannot see behind). Jev can still veto an allowed motion
  (p(safe) < 0.3) for reasons the numbers miss, such as people.
- **Composite decision**: among the allowed motions, the highest progress score wins. Raw answers are logged, so
  thresholds can be changed without re-running inference.

| action | command vx (m/s) | command yaw rate (rad/s) | measured in MuJoCo |
|---|---|---|---|
| forward | 0.5 | 0 | ~0.35 m/s |
| left / right | 0 | ±0.7 | ~28°/s on the spot |
| backward | -0.3 | 0 | ~0.2 m/s |

One omnidirectional policy (v357, trained on vx ∈ [-1, 1], yaw rate up to ±0.7) covers all four.

## Scenarios

`kangaroo_jev/scenarios.py`. Every course has the same goal: reach a red cube on a 0.5 m grey pedestal, in view
from the start. The decision makers get no map and no pose, only the camera-derived scene; the cube's position is
used for scoring only. Raised courses are deck tiles over a floor 1.5 m below, so walking off an edge is a fall that
ends the episode. Walls are 0.8 m tall: the robot cannot pass them, but sees the cube over them. `person` obstacles are photo billboards YOLO
recognises; boxes and walls are plain grey and only depth sees them.

| course | what makes it hard |
|---|---|
| `walkway_turn` | 1.4 m L-shaped walkway; the cube is up the left branch, the straight line to it is a drop |
| `narrow_bridge` | 1 m bridge offset to the left of the straight line to the cube |
| `pit` | 2×2 m hole right between the robot and the cube |
| `crowded_walkway` | 2 m walkway with two people and a block on it |
| `wall_gap` | a wall between the robot and the cube; the way through is a 1.2 m gap off to the left |
| `staggered_walls` | two staggered walls: weave right, then left |

Metrics per episode: success (within 0.9 m of the cube centre, no collision, no fall), progress (share of the
start-to-cube distance covered at best), collision (base footprint within 0.3 m of an obstacle), fall (over, or
off an edge), time, path length, decision latency (mean / p95), Jev confidence, tokens per decision, API
errors, and agreement with the heuristic on the same scenes.

## Upstream setup and the changes it needed

Pinned in `docker/Dockerfile`: `pal-robotics/kangaroo_robot`, `kangaroo_simulation`, and
`pal_policy_deployer` (`eurobin/extras`, which has the 4dof loco baselines), built from source, plus
`mujoco_ros2_control` 0.1.2 (binary). The sim runs the pregenerated
`mjcf_data_floating_4dof_no-end-effector_no-end-effector_fixed` model. `kangaroo_jev/sim_setup.py` edits the
installed copies in place at launch:

- **Camera.** The model has the RealSense frames but no camera, so an idealized D435 is attached to the
  `torso_front_up_rgbd_camera_color_frame` site (30° down): a pinhole RGB-D camera with the D435 depth
  field of view (87°×58°) at 424×240, depth aligned to colour, published by the `mujoco_ros2_control`
  CameraPlugin. A FreeJointStatePublisher plugin publishes the ground-truth base pose.
- **Introspection shim** (`kangaroo_jev/introspection_shim.py`). `kang_mjlab_node` reads robot state from
  ros2_control's pal_statistics introspection topics, which only exist from Jazzy on. The shim rebuilds
  them from `/joint_states` and the torso IMU, at 50 Hz sim time so the policy keeps its trained rate
  when MuJoCo runs slower than real time (~0.8× here).
- **Policy.** PAL's deployer launch with `robot_type:=full_body_4dof` and `model:=v357`, the newest
  4dof baseline. Of the 4dof models (v265, v352–v357, v700) it was the best all-rounder in a
  MuJoCo-only sweep: v265 (the branch default) barely walks forward, v700 does not walk backward.
- `controller_manager` on Humble listens on `~/robot_description`: remapped to `/robot_description`.
- MoveIt is disabled. Without a display, the entrypoint starts Xvfb for the simulator's viewer window.

## Layout

```
docker/            image + entrypoint
sim/               launch file: MuJoCo sim + shim + policy
kangaroo_jev/
  sim_setup.py     D435 camera, worlds, plugins (edits the installed PAL model in place)
  scenarios.py     obstacle courses + clearance metric
  perception.py    RGB-D -> scene description
  brains.py        Jev and heuristic decision makers, fixed velocities
  navigator.py     ROS loop: perceive -> decide -> /cmd_vel
  bench.py         runs episodes, writes results/
```
