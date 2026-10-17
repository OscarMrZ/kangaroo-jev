# kangaroo-systemone

Reactive, camera-driven navigation for PAL Robotics' [Kangaroo](https://github.com/pal-robotics/kangaroo_robot) biped in MuJoCo,
as a benchmark for System One models: [TypeSafe Jev](https://docs.typesafe.ai) (text, or text plus camera frames),
[CLM-8B](https://github.com/Contrastive-LM/CLM), or any model behind a TypeSafe-compatible API makes the decisions.
Walking is PAL's own RL policy; the model picks forward, left, right or backward once per move from what the head camera
sees.

![Courses](docs/img/courses.jpg)

**Results: [REPORT.md](REPORT.md).**

## Installation

Everything runs in Docker (ROS 2 Humble, MuJoCo, CPU only). The optional CLM server needs an NVIDIA GPU and the NVIDIA
Container Toolkit.

```bash
git clone https://github.com/OscarMrZ/kangaroo-systemone.git
cd kangaroo-systemone
docker compose build                  # add --profile clm to also build the CLM server
```

The image pins and builds `pal-robotics/kangaroo_robot`, `kangaroo_simulation` and `pal_policy_deployer`.

## Quick Start

Run one course with the rule-based baseline.

```bash
docker compose run --rm kangaroo bash -c "
  ros2 launch sim/kangaroo_systemone.launch.py scenario:=pit seed:=0 &
  sleep 30 && python3 -m kangaroo_systemone.navigator --brain heuristic --scenario pit --seed 0"
```

Use Jev instead (`--brain jev_multi` adds the camera frame, `jev_multi_10` the last 10).

```bash
export TYPESAFE_API_KEY=...   # https://console.typesafe.ai
docker compose run --rm kangaroo bash -c "
  ros2 launch sim/kangaroo_systemone.launch.py scenario:=pit seed:=0 &
  sleep 30 && python3 -m kangaroo_systemone.navigator --brain jev --scenario pit --seed 0"
```

Use CLM-8B: start its server on the GPU first (Qwen3-8B encoder + CLM head; set `OFFLOAD_GB=1.5` on 16 GB cards), then
run with `--brain clm_native` (questions written for CLM) or `--brain clm` (Jev's questions, for comparison).

```bash
docker compose --profile clm up -d clm
```

Watch the MuJoCo viewer (X11 or XWayland): prefix either command with `xhost +local: && VIEWER_DISPLAY=$DISPLAY`.

Courses: `walkway_turn`, `narrow_bridge`, `pit`, `crowded_walkway`, `wall_gap`, `staggered_walls`. The seed jitters the
start pose (±0.1 m, ±0.15 m, ±6°); seed 0 is the nominal start. An episode ends at the cube, at a fall, or after 90
moves (`--decisions`).

## Decision makers

| Name | What decides | Needs |
| --- | --- | --- |
| `jev` | TypeSafe Jev: a safety `Noul` and a progress `Score` per motion, over perception's scene description | `TYPESAFE_API_KEY` |
| `jev_multi` | the same on Jev 1.14, with the current head-camera frame in the state (experimental image input) | `TYPESAFE_API_KEY` |
| `jev_multi_10` | the same, with the last 10 frames (one per move) and the move made after each | `TYPESAFE_API_KEY` |
| `clm` | CLM-8B on Jev's questions | the `clm` server |
| `clm_native` | CLM-8B, one `Choice` written for a contrastive model | the `clm` server |
| `clm_image_native` | an image CLM on the `clm_native` question plus the frame | [kangaroo-clm](../kangaroo-clm)'s server and `clm` library |
| `heuristic` | hand-written rule | – |

All of them see the same scene, and code applies the same safety margin first; see [REPORT.md](REPORT.md#decision-design).

## Benchmark

Run every course with every decision maker that runs from this repo alone (all but `clm_image_native`) and seeds 0–4,
each episode in a fresh simulation (the CLM server must be up). Narrow it with `--brains`, `--scenarios`, `--seeds`.

```bash
docker compose run --rm kangaroo python3 -m kangaroo_systemone.bench
docker compose run --rm kangaroo python3 -m kangaroo_systemone.bench --brains heuristic --scenarios pit --seeds 0
```

Results go to `results/<run>/`: `summary.md`, `episodes.csv`, and one `.jsonl` per episode with every scene, decision and raw
model answer. Regenerate the report figures from a run (`--out`, `--films course:seed ...`, `--no-photos`).

```bash
docker compose run --rm kangaroo python3 docs/figures.py results/<run>
```

## Try a new System One model

- **A new release of a registered model:** `name@model`, e.g. `--brains jev jev@jev-preview jev_multi@jev-1.14.0`. The
  spec names the episode files and the report rows.
- **A model behind a TypeSafe-compatible API** (`system_one(state, questions, model=...)` returning `answers` with
  `noul`, `score` and `confidence`): subclass `SystemOneBrain` with its client and `request()`, like `ClmBrain`, and add
  it to `BRAINS` in `kangaroo_systemone/brains.py`. It gets Jev's eight questions, the legend and, with `frames`, the
  camera frames (implement `image()`).
- **Anything else:** subclass `Brain` and implement `__call__(scene, frame) -> Decision` (and `reset()` if it keeps
  memory across moves); call `geometrically_safe(scene)` to stay inside the safety margin.

## Layout

| Path | What |
| --- | --- |
| `sim/kangaroo_systemone.launch.py` | MuJoCo sim, introspection shim and PAL walking policy |
| `kangaroo_systemone/sim_setup.py` | D435 camera, course worlds and plugins, applied to the installed PAL packages |
| `kangaroo_systemone/scenarios.py` | courses |
| `kangaroo_systemone/perception.py` | RGB-D frame → scene description |
| `kangaroo_systemone/brains.py` | decision makers and their registry: Jev's questions (text or with frames), CLM-native questions, policy in code, rule-based baseline |
| `docker/clm/` | CLM-8B server: vLLM Qwen3-8B encoder + `clm-serve` |
| `kangaroo_systemone/navigator.py` | perceive → decide → `/cmd_vel` loop, episode metrics |
| `kangaroo_systemone/bench.py` | benchmark runner |
| `kangaroo_systemone/introspection_shim.py` | ros2_control introspection topics for PAL's policy node on Humble |
| `docs/figures.py` | report figures |
