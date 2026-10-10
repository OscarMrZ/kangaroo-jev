# kangaroo-jev

How good is [TypeSafe Jev](https://docs.typesafe.ai) as the decision maker for reactive navigation of
PAL Robotics' [Kangaroo](https://github.com/pal-robotics/kangaroo_robot) biped?

```
head RGB-D camera ──► YOLO26n-seg + depth geometry ──► scene description (JSON) ──► Jev Choice
     (MuJoCo)                                                                     forward / left / right / backward
                                                                                          │ fixed velocities
PAL RL locomotion policy (pal_policy_deployer, ONNX) ◄── /cmd_vel + policy switch ◄───────┘
```

Walking is not trained here: it is PAL's published locomotion policies. Jev only sees the structured
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

| action | policy | vx (m/s) | yaw rate (rad/s) |
|---|---|---|---|
| forward | v639 (forward) | 0.3 | 0 |
| left / right | v639 (forward) | 0.2 | ±0.5 |
| backward | v630 (backward) | -0.25 | 0 |

The forward policy was trained only on vx ∈ [0.2, 0.5], so turning happens while walking slowly.

## Scenarios

`kangaroo_jev/scenarios.py`: start at the origin facing +x, goal at (6, 0). `person` obstacles are photo
billboards YOLO recognises; `box` obstacles are plain grey blocks only depth can see.
`clear`, `person_ahead`, `box_ahead`, `slalom`, `dead_end` (U-shaped trap facing the robot).

Metrics per episode: success (goal reached with no collision), collision (base footprint within 0.3 m of an
obstacle), fall, time, path length, minimum clearance, decision latency (mean / p95), Jev confidence,
tokens per decision, API errors, and agreement with the heuristic on the same scenes.

## Upstream setup and the changes it needed

Pinned in `docker/Dockerfile`: `pal-robotics/kangaroo_robot`, `kangaroo_simulation`, and
`pal_policy_deployer` (source), plus `mujoco_ros2_control` 0.1.2 (binary). PAL's files are not edited;
`kangaroo_jev/sim_setup.py` adapts the installed copies at launch:

- **Camera.** The URDF has a torso RGB-D bracket but the MJCF has no camera, so a 320×240, 58° camera is
  mounted on the `torso_front_up_rgbd_camera_color_frame` site (30° down) and published by the
  `mujoco_ros2_control` CameraPlugin. A FreeJointStatePublisher plugin publishes the ground-truth base pose.
- **Introspection shim** (`kangaroo_jev/introspection_shim.py`). `kang_mjlab_node` reads robot state from
  ros2_control's pal_statistics introspection topics, which only exist from Jazzy on. The shim rebuilds
  them from `/joint_states` and the torso IMU, at 50 Hz sim time so the policy keeps its trained rate
  when MuJoCo runs slower than real time (~0.8× here).
- **Policies.** The deployer launch default (`v315`) holds balance but never steps in this simulator
  (also in pure MuJoCo on PAL's training XML). The newer `v639` (forward) / `v630` (backward) walk, so
  `sim/kangaroo_jev.launch.py` loads those two and switches between them.
- `controller_manager` on Humble listens on `~/robot_description`: remapped to `/robot_description`.
- MoveIt is disabled. Without a display, the entrypoint starts Xvfb for the simulator's viewer window.

## Layout

```
docker/            image + entrypoint
sim/               launch file: MuJoCo sim + shim + policies
kangaroo_jev/
  sim_setup.py     camera, worlds, plugins (applied to the installed PAL packages)
  scenarios.py     obstacle courses + clearance metric
  perception.py    RGB-D -> scene description
  brains.py        Jev and heuristic decision makers, fixed velocities
  navigator.py     ROS loop: perceive -> decide -> /cmd_vel
  bench.py         runs episodes, writes results/
```
