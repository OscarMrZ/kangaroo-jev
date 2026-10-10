"""Closed-loop benchmark. Every (scenario, brain) episode runs in a fresh simulation.

python3 -m kangaroo_jev.bench                      # all scenarios, jev + heuristic
python3 -m kangaroo_jev.bench --brains heuristic --scenarios clear slalom
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy

from kangaroo_jev.brains import BRAINS
from kangaroo_jev.navigator import Navigator, run_episode, spin_in_background
from kangaroo_jev.scenarios import SCENARIOS

LAUNCH = Path(__file__).resolve().parent.parent / "sim" / "kangaroo_jev.launch.py"


def start_sim(scenario, log):
    return subprocess.Popen(["ros2", "launch", str(LAUNCH), f"scenario:={scenario}"],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop_sim(proc):
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(wait)
            return
        except (subprocess.TimeoutExpired, ProcessLookupError):
            continue


def run_in_fresh_sim(scenario, brain, perception, timeout, prefix):
    with open(f"{prefix}.launch.log", "w") as sim_log:
        sim = start_sim(scenario, sim_log)
        node = Navigator(SCENARIOS[scenario])
        executor = spin_in_background(node)
        try:
            # The policy takes over ~8 s after launch; let it settle the robot before driving.
            node.wait_until(lambda: node.pose is not None and node.rgb is not None, 120, "simulation")
            node.wait_until(lambda: node.now() > 15.0, 120, "policy start")
            with open(f"{prefix}.jsonl", "w") as log:
                result = run_episode(node, perception, brain, timeout, log=log)
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
    lines = ["| brain | success | collided | fell | mean time (s) | decision latency mean / p95 (s) | mean confidence |",
             "|---|---|---|---|---|---|---|"]
    for b in brains:
        rs = [r for r in rows if r["brain"] == b]
        if not rs:
            continue
        mean = lambda k: np.mean([r[k] for r in rs if r[k] is not None]) if any(r[k] is not None for r in rs) else None
        fmt = lambda v, p=2: "-" if v is None else f"{v:.{p}f}"
        lines.append(
            f"| {b} | {sum(r['success'] for r in rs)}/{len(rs)} | {sum(r['collided'] for r in rs)} | "
            f"{sum(r['outcome'].startswith('fell') for r in rs)} | {fmt(mean('time_s'), 1)} | "
            f"{fmt(mean('decision_latency_mean_s'), 3)} / {fmt(mean('decision_latency_p95_s'), 3)} | "
            f"{fmt(mean('confidence_mean'))} |")
    lines += ["", "| scenario | brain | outcome | success | time (s) | min clearance (m) | decisions | agreement w/ heuristic |",
              "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['scenario']} | {r['brain']} | {r['outcome']} | {r['success']} | {r['time_s']} | "
                     f"{r['min_clearance_m']} | {r['decisions']} | {r['agreement_with_heuristic']} |")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brains", nargs="+", choices=BRAINS, default=list(BRAINS))
    ap.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    ap.add_argument("--timeout", type=float, default=90.0, help="sim seconds per episode")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    from kangaroo_jev.perception import Perception

    out = Path(args.out) / datetime.now().strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True)
    perception = Perception(os.environ["YOLO_MODEL"])
    brains = {name: BRAINS[name]() for name in args.brains}
    rclpy.init()
    rows = []
    for scenario in args.scenarios:
        for name, brain in brains.items():
            for attempt in range(2):  # retry once if the robot falls during the policy handover, before driving
                print(f"[bench] {scenario} / {name}" + (" (retry: fell before the first decision)" if attempt else ""),
                      flush=True)
                result = run_in_fresh_sim(scenario, brain, perception, args.timeout, out / f"{scenario}_{name}")
                if not (result["outcome"] == "fell" and result.get("decisions") == 0):
                    break
            result["startup_retries"] = attempt
            rows.append({"scenario": scenario, "brain": name, **result})
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
