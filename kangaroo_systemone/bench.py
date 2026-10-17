"""Closed-loop benchmark. Every (scenario, seed, decision maker) episode runs in a fresh simulation.

python3 -m kangaroo_systemone.bench                                   # every course, DEFAULT_BRAINS, seeds 0-4
python3 -m kangaroo_systemone.bench --brains heuristic --scenarios pit --seeds 0
python3 -m kangaroo_systemone.bench --brains jev jev@jev-preview      # a decision maker with another model
"""

import argparse
import csv
import itertools
import json
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy

from kangaroo_systemone.brains import DEFAULT_BRAINS, brain_spec, make_brain
from kangaroo_systemone.navigator import Navigator, run_episode, spin_in_background
from kangaroo_systemone.scenarios import SCENARIOS

LAUNCH = Path(__file__).resolve().parent.parent / "sim" / "kangaroo_systemone.launch.py"


def start_sim(scenario, seed, log):
    return subprocess.Popen(["ros2", "launch", str(LAUNCH), f"scenario:={scenario}", f"seed:={seed}"],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop_sim(proc):
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(wait)
            return
        except (subprocess.TimeoutExpired, ProcessLookupError):
            continue


def run_in_fresh_sim(scenario, seed, brain, perception, decisions, prefix):
    with open(f"{prefix}.launch.log", "w") as sim_log:
        sim = start_sim(scenario, seed, sim_log)
        node = Navigator(SCENARIOS[scenario], seed)
        executor = spin_in_background(node)
        try:
            # The policy takes over ~8 s after launch; let it settle the robot before driving.
            node.wait_until(lambda: node.pose is not None and node.rgb is not None, 120, "simulation")
            node.wait_until(lambda: node.now() > 15.0, 120, "policy start")
            with open(f"{prefix}.jsonl", "w") as log:
                result = run_episode(node, perception, brain, decisions, log=log)
            steps = [json.loads(line) for line in open(f"{prefix}.jsonl")]
            agree = np.mean([s["action"] == s["heuristic_action"] for s in steps]) if steps else None
            result["agreement_with_heuristic"] = None if agree is None else round(float(agree), 2)
        except TimeoutError as e:
            result = {"outcome": f"error: {e}", "success": False, "collided": False}
        finally:
            executor.shutdown()
            node.destroy_node()
            stop_sim(sim)
            time.sleep(2)
    return result


def summarize(rows, brains):
    """Per course and decision maker over seeds, then every pair of decision makers by (course, seed)."""
    fmt = lambda vs, p=2: f"{np.mean(vs):.{p}f} ± {np.std(vs):.{p}f}" if vs else "-"
    lines = ["| course | brain | episodes | reached | fell | collided | progress | turn share | reversals |",
             "|---|---|---|---|---|---|---|---|---|"]
    for scenario in dict.fromkeys(r["scenario"] for r in rows):
        for b in brains:
            rs = [r for r in rows if r["scenario"] == scenario and r["brain"] == b]
            if rs:
                lines.append(
                    f"| {scenario} | {b} | {len(rs)} | {sum(r['outcome'] == 'reached' for r in rs)} | "
                    f"{sum(r['outcome'].startswith('fell') for r in rs)} | {sum(r['collided'] for r in rs)} | "
                    f"{fmt([r['progress'] for r in rs])} | {fmt([r['turn_share'] for r in rs])} | "
                    f"{fmt([r['reversals'] for r in rs], 1)} |")
    lines += ["", "| brain | episodes | reached | fell | collided | progress | decision latency mean / p95 (s) |",
              "|---|---|---|---|---|---|---|"]
    for b in brains:
        rs = [r for r in rows if r["brain"] == b]
        if rs:
            lat = lambda k: np.mean([r[k] for r in rs if r[k] is not None])
            lines.append(f"| {b} | {len(rs)} | {sum(r['outcome'] == 'reached' for r in rs)} | "
                         f"{sum(r['outcome'].startswith('fell') for r in rs)} | {sum(r['collided'] for r in rs)} | "
                         f"{fmt([r['progress'] for r in rs])} | {lat('decision_latency_mean_s'):.3f} / "
                         f"{lat('decision_latency_p95_s'):.3f} |")
    by = {(r["scenario"], r["seed"], r["brain"]): r["progress"] for r in rows}
    for x, y in itertools.combinations(brains, 2):
        diffs = np.array([by[(c, s, x)] - by[(c, s, y)] for c, s, b in by if b == x and (c, s, y) in by])
        if len(diffs):
            boot = np.random.default_rng(0).choice(diffs, (10000, len(diffs))).mean(axis=1)
            lo, hi = np.percentile(boot, [2.5, 97.5])
            lines += ["", f"{x} - {y} progress, paired by (course, seed): {diffs.mean():+.2f} "
                          f"(95% bootstrap CI {lo:+.2f} to {hi:+.2f}); {x} ahead in {(diffs > 0).sum()}, behind in "
                          f"{(diffs < 0).sum()}, tied in {(diffs == 0).sum()} of {len(diffs)} pairs."]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brains", nargs="+", type=brain_spec, default=DEFAULT_BRAINS, help="decision makers, or name@model")
    ap.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4], help="start pose jitter; 0 = nominal")
    ap.add_argument("--decisions", type=int, default=90, help="move budget per episode (1 s of motion each)")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    from kangaroo_systemone.perception import Perception

    out = Path(args.out) / datetime.now().strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True)
    perception = Perception(os.environ["YOLO_MODEL"])
    brains = {spec: make_brain(spec) for spec in args.brains}
    rclpy.init()
    rows = []
    for seed in args.seeds:
        for scenario in args.scenarios:
            for name, brain in brains.items():
                for attempt in range(2):  # retry once if the robot falls during the policy handover, before driving
                    print(f"[bench] {scenario} / {name} / seed {seed}" +
                          (" (retry: fell before the first decision)" if attempt else ""), flush=True)
                    result = run_in_fresh_sim(scenario, seed, brain, perception, args.decisions,
                                              out / f"{scenario}_{name}_s{seed}")
                    if not (result["outcome"].startswith("fell") and result.get("decisions") == 0):
                        break
                result["startup_retries"] = attempt
                rows.append({"scenario": scenario, "brain": name, "seed": seed, **result})
                print(f"[bench]   {json.dumps(rows[-1])}", flush=True)
                with open(out / "episodes.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, list(dict.fromkeys(k for r in rows for k in r)))
                    w.writeheader()
                    w.writerows(rows)
    (out / "summary.md").write_text(summarize([r for r in rows if "time_s" in r], args.brains))
    print((out / "summary.md").read_text())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
