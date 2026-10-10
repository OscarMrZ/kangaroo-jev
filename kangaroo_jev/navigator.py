"""Reactive navigation loop: head camera -> perception -> brain -> /cmd_vel.

python3 -m kangaroo_jev.navigator --brain jev --scenario slalom   # with the sim already running
"""

import argparse
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from kangaroo_jev.brains import ACTIONS, BRAINS, HeuristicBrain
from kangaroo_jev.scenarios import SCENARIOS, clearance

GOAL_TOLERANCE = 0.9  # m from the cube centre: standing right in front of it
FALL_HEIGHT = 0.5  # base_link z below this = fallen (below the deck = walked off an edge)
ROBOT_RADIUS = 0.3  # base_link to the outside of the feet, roughly


class Navigator(Node):
    def __init__(self, scenario):
        super().__init__("kangaroo_jev_navigator", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.scenario = scenario
        self.rgb = self.depth = self.pose = None
        self.action = None  # None: stand still
        self.min_clearance = math.inf
        self.path_length = 0.0
        self.create_subscription(Image, "/head_camera/color", self.on_rgb, 1)
        self.create_subscription(Image, "/head_camera/depth", self.on_depth, 1)
        self.create_subscription(FreeJointStateArray, "/base_state", self.on_pose, 1)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_timer(0.05, self.publish_cmd)

    def on_rgb(self, m):
        self.rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)

    def on_depth(self, m):
        self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)

    def on_pose(self, m):
        p = m.free_joints[0].pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None and self.action is not None:
            self.path_length += math.hypot(p.position.x - self.pose[0], p.position.y - self.pose[1])
            self.min_clearance = min(self.min_clearance, clearance(p.position.x, p.position.y, self.scenario.obstacles))
        self.pose = (p.position.x, p.position.y, p.position.z, yaw)

    def publish_cmd(self):
        t = Twist()
        if self.action is not None:
            t.linear.x, t.angular.z = ACTIONS[self.action]["vx"], ACTIONS[self.action]["wz"]
        self.cmd_pub.publish(t)


    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_until(self, condition, timeout, what):
        end = time.time() + timeout
        while not condition():
            if time.time() > end:
                raise TimeoutError(f"timed out waiting for {what}")
            time.sleep(0.1)


def goal_relative(pose, goal):
    x, y, _, yaw = pose
    bearing = math.degrees(math.atan2(goal[1] - y, goal[0] - x) - yaw)
    return {"distance_m": round(math.hypot(goal[0] - x, goal[1] - y), 1), "bearing_deg": round((bearing + 180) % 360 - 180)}


def run_episode(node, perception, brain, timeout=90.0, period=1.0, log=None):
    """Drive until the goal is reached, the robot falls, or `timeout` sim seconds pass."""
    node.wait_until(lambda: node.rgb is not None and node.pose is not None, 60, "camera and pose")
    perception.reset()
    reference, history, latencies, perception_s = HeuristicBrain(), [], [], []
    confidences, tokens, errors = [], 0, 0
    start, outcome = node.now(), "timeout"
    start_distance = closest = goal_relative(node.pose, node.scenario.goal)["distance_m"]
    while node.now() - start < timeout:
        tick = node.now()
        x, y, z, _ = node.pose
        goal = goal_relative(node.pose, node.scenario.goal)  # ground truth, for scoring only
        closest = min(closest, goal["distance_m"])
        if goal["distance_m"] < GOAL_TOLERANCE:
            outcome = "reached"
            break
        if z < FALL_HEIGHT:
            t_fall = node.now()  # let it land before telling a fall over from a fall off the deck
            while node.now() - t_fall < 1.5:
                time.sleep(0.05)
            outcome = "fell off an edge" if node.pose[2] < 0 else "fell"
            break
        t0 = time.perf_counter()
        # The simulator's base pose stands in for leg odometry: perception only uses it to keep recently seen
        # edges and obstacles in place; the decision makers never see it.
        scene = perception.describe(node.rgb, node.depth, (x, y, node.pose[3]), node.now())
        perception_s.append(time.perf_counter() - t0)
        scene["recent_actions"] = history[-8:]
        decision = brain(scene)
        node.action = decision.action
        history.append(decision.action)
        latencies.append(decision.latency_s)
        tokens += decision.input_tokens
        errors += decision.error is not None
        if decision.answers:
            confidences.append(decision.answers["safe"][decision.action])
        if log:
            log.write(json.dumps({
                "t": round(tick - start, 2), "pose": [round(v, 3) for v in node.pose], "scene": scene,
                "action": decision.action, "heuristic_action": reference(scene).action,
                "answers": decision.answers,
                "latency_s": round(decision.latency_s, 3), "error": decision.error,
            }) + "\n")
        while node.now() - tick < period and node.pose[2] >= FALL_HEIGHT:
            time.sleep(0.02)
    node.action = None
    clear = node.min_clearance - ROBOT_RADIUS
    return {
        "outcome": outcome,
        "success": outcome == "reached" and clear > 0,
        "collided": clear <= 0,
        "time_s": round(node.now() - start, 1),
        "path_m": round(node.path_length, 2),
        "min_clearance_m": round(clear, 2) if math.isfinite(clear) else None,
        "start_distance_m": start_distance,
        "closest_distance_m": closest,
        # share of the way from the start to the cube that was covered at best
        "progress": round(max(0.0, min(1.0, (start_distance - closest) / (start_distance - GOAL_TOLERANCE))), 2),
        "decisions": len(history),
        "decision_latency_mean_s": round(float(np.mean(latencies)), 3) if latencies else None,
        "decision_latency_p95_s": round(float(np.percentile(latencies, 95)), 3) if latencies else None,
        "perception_mean_s": round(float(np.mean(perception_s)), 3) if perception_s else None,
        "chosen_p_safe_mean": round(float(np.mean(confidences)), 3) if confidences else None,
        "input_tokens_per_decision": round(tokens / len(history)) if history else None,
        "api_errors": errors,
    }


def spin_in_background(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return executor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brain", choices=BRAINS, default="jev")
    ap.add_argument("--scenario", choices=SCENARIOS, default="walkway_turn", help="the course the sim is running")
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    from kangaroo_jev.perception import Perception

    rclpy.init()
    node = Navigator(SCENARIOS[args.scenario])
    executor = spin_in_background(node)
    with open(f"navigator_{args.brain}.jsonl", "w") as log:
        result = run_episode(node, Perception(os.environ["YOLO_MODEL"]), BRAINS[args.brain](), args.timeout, log=log)
    print(json.dumps(result, indent=2))
    time.sleep(0.2)
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
