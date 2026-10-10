# kangaroo-jev

How good is [TypeSafe Jev](https://docs.typesafe.ai) as the decision maker for reactive navigation of
PAL Robotics' [Kangaroo](https://github.com/pal-robotics/kangaroo_robot) biped (4dof-arm configuration)?

```
D435 RGB-D (head) ──► YOLO26n-seg + depth geometry ──► scene description (JSON) ──► Jev Choice
     (MuJoCo)                                                                     forward / left / right / backward
                                                                                          │ fixed velocities
PAL 4dof RL locomotion policy (pal_policy_deployer v357) ◄──── /cmd_vel ◄───────────────────┘
```

Walking is not trained here: it is PAL's published 4dof locomotion policy. Jev only sees the structured
scene description and picks one of four motions once per second. A hand-written rule over the same
description runs as a baseline, and is also evaluated on every Jev step so the logs show where they disagree.

## Run

Everything runs in Docker (ROS 2 Humble, MuJoCo, CPU torch).

```bash
docker compose build
export TYPESAFE_API_KEY=...            # https://console.typesafe.ai

# Full benchmark: every scenario × {jev, heuristic}, each episode in a fresh simulation
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench --brains heuristic --scenarios slalom
```

Results land in `results/<timestamp>/`: `summary.md`, `episodes.csv`, and one `.jsonl` per episode with
every scene description, decision, probability distribution, confidence and latency.

To watch the MuJoCo viewer (X11/XWayland), allow local clients and pass your display:

```bash
xhost +local:
VIEWER_DISPLAY=$DISPLAY docker compose run --rm kangaroo bash -c \
  "ros2 launch sim/kangaroo_jev.launch.py scenario:=slalom & sleep 25; python3 -m kangaroo_jev.navigator --brain jev --scenario slalom"
```

## What Jev gets

One `Choice` question per step (`kangaroo_jev/brains.py`); the state is built by `kangaroo_jev/perception.py`:

```json
{
  "free_distance_m": {"left": 1.9, "ahead": 1.9, "right": 3.2},
  "objects": [{"label": "person", "confidence": 0.9, "distance_m": 1.9, "bearing_deg": 14, "side": "ahead"}],
  "goal": {"distance_m": 4.1, "bearing_deg": -3},
  "recent_actions": ["forward", "forward"]
}
```

`free_distance_m` comes from the depth image (points above the floor, in the robot's walking strip and
beside it), so it also covers obstacles YOLO cannot name. `objects` are YOLO segmentation detections with
their depth-derived distance and bearing. The goal is given relative to the robot (from the simulator pose).

| action | command vx (m/s) | command yaw rate (rad/s) | measured in MuJoCo |
|---|---|---|---|
| forward | 0.5 | 0 | ~0.35 m/s |
| left / right | 0 | ±0.7 | ~28°/s on the spot |
| backward | -0.3 | 0 | ~0.2 m/s |

One omnidirectional policy (v357, trained on vx ∈ [-1, 1], yaw rate up to ±0.7) covers all four.

## Scenarios

`kangaroo_jev/scenarios.py`: start at the origin facing +x, goal at (6, 0). `person` obstacles are photo
billboards YOLO recognises; `box` obstacles are plain grey blocks only depth can see.
`clear`, `person_ahead`, `box_ahead`, `slalom`, `dead_end` (U-shaped trap facing the robot).

Metrics per episode: success (goal reached with no collision), collision (base footprint within 0.3 m of an
obstacle), fall, time, path length, minimum clearance, decision latency (mean / p95), Jev confidence,
tokens per decision, API errors, and agreement with the heuristic on the same scenes.

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
