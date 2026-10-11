# kangaroo-jev

Reactive, camera-driven navigation for PAL Robotics' [Kangaroo](https://github.com/pal-robotics/kangaroo_robot) biped in MuJoCo,
with [TypeSafe Jev](https://docs.typesafe.ai) or [CLM-8B](https://github.com/Contrastive-LM/CLM) making the decisions.
Walking is PAL's own RL policy; the model picks forward, left, right or backward once per move from what the head camera
sees.

![Courses](docs/img/courses.jpg)

**Results: [REPORT.md](REPORT.md).**

## Installation

Everything runs in Docker (ROS 2 Humble, MuJoCo, CPU only). The optional CLM server needs an NVIDIA GPU and the NVIDIA
Container Toolkit.

```bash
git clone https://github.com/OscarMrZ/kangaroo-jev.git
cd kangaroo-jev
docker compose build                  # add --profile clm to also build the CLM server
```

The image pins and builds `pal-robotics/kangaroo_robot`, `kangaroo_simulation` and `pal_policy_deployer`.

## Quick Start

Run one course with the rule-based baseline.

```bash
docker compose run --rm kangaroo bash -c "
  ros2 launch sim/kangaroo_jev.launch.py scenario:=pit seed:=0 &
  sleep 30 && python3 -m kangaroo_jev.navigator --brain heuristic --scenario pit --seed 0"
```

Use Jev instead.

```bash
export TYPESAFE_API_KEY=...   # https://console.typesafe.ai
docker compose run --rm kangaroo bash -c "
  ros2 launch sim/kangaroo_jev.launch.py scenario:=pit seed:=0 &
  sleep 30 && python3 -m kangaroo_jev.navigator --brain jev --scenario pit --seed 0"
```

Use CLM-8B: start its server on the GPU first (Qwen3-8B encoder + CLM head; set `OFFLOAD_GB=1.5` on 16 GB cards), then
run with `--brain clm`.

```bash
docker compose --profile clm up -d clm
```

Watch the MuJoCo viewer (X11 or XWayland): prefix either command with `xhost +local: && VIEWER_DISPLAY=$DISPLAY`.

Courses: `walkway_turn`, `narrow_bridge`, `pit`, `crowded_walkway`, `wall_gap`, `staggered_walls`. The seed jitters the
start pose (±0.1 m, ±0.15 m, ±6°); seed 0 is the nominal start. An episode ends at the cube, at a fall, or after 90
moves (`--decisions`).

## Benchmark

Run every course with every decision maker (Jev, CLM, rule) and seeds 0–4, each episode in a fresh simulation (90
episodes, ~4 h; the CLM server must be up).

```bash
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench
docker compose run --rm kangaroo python3 -m kangaroo_jev.bench --brains heuristic --scenarios pit --seeds 0
```

Results go to `results/<run>/`: `summary.md`, `episodes.csv`, and one `.jsonl` per episode with every scene, decision and raw Jev
answer. Regenerate the report figures from a run.

```bash
docker compose run --rm kangaroo python3 docs/figures.py results/<run>
```

## Layout

| Path | What |
| --- | --- |
| `sim/kangaroo_jev.launch.py` | MuJoCo sim, introspection shim and PAL walking policy |
| `kangaroo_jev/sim_setup.py` | D435 camera, course worlds and plugins, applied to the installed PAL packages |
| `kangaroo_jev/scenarios.py` | courses |
| `kangaroo_jev/perception.py` | RGB-D frame → scene description |
| `kangaroo_jev/brains.py` | questions and policy, Jev and CLM clients, rule-based baseline |
| `docker/clm/` | CLM-8B server: vLLM Qwen3-8B encoder + `clm-serve` |
| `kangaroo_jev/navigator.py` | perceive → decide → `/cmd_vel` loop, episode metrics |
| `kangaroo_jev/bench.py` | benchmark runner |
| `kangaroo_jev/introspection_shim.py` | ros2_control introspection topics for PAL's policy node on Humble |
| `docs/figures.py` | report figures |
