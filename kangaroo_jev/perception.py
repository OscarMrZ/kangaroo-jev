"""RGB-D frame -> compact scene description. This dict is everything the decision makers see: no map and
no pose. Code does the geometry (what each of the four motions runs into); the decision makers judge it.
"""

import math

import numpy as np

FOVY_DEG = 58.0  # head_camera, see sim_setup.py
TILT = math.radians(30.0)  # pitch of the torso RGB-D bracket
CAMERA_HEIGHT = 1.35  # m above the floor while walking
OBSTACLE_HEIGHT = 0.2  # points higher than this above the floor are obstacles
DROP_DEPTH = 0.3  # points lower than this below the floor are a drop (an edge or a hole)
CORRIDOR = 0.35  # half-width of the strip the robot sweeps (m)
MAX_RANGE = 4.0  # m; also reported when a strip is clear
TURN = math.radians(28)  # about one second of turning on the spot
MEMORY_S = 20.0  # points stay known this long after leaving the view (the floor within ~0.8 m is never in view)
GRID = 0.1  # m; remembered points are kept on this grid
YOLO_CONFIDENCE = 0.5


def side(bearing_deg):
    return "left" if bearing_deg > 15 else "right" if bearing_deg < -15 else "ahead"


def nearest_in_strip(points, heading):
    """Distance along `heading` (rad, left positive) to the nearest point in the robot-wide strip."""
    if not len(points):
        return MAX_RANGE
    c, s = math.cos(heading), math.sin(heading)
    along = points[:, 0] * c + points[:, 1] * s
    across = -points[:, 0] * s + points[:, 1] * c
    hit = along[(np.abs(across) <= CORRIDOR) & (along > 0)]
    return round(float(min(hit.min(), MAX_RANGE)), 1) if hit.size else MAX_RANGE


class Perception:
    def __init__(self, model_path):
        from ultralytics import YOLO

        self.yolo = YOLO(model_path)
        self.memory = []  # (t, obstacle points, edge points), both in the odometry frame

    def describe(self, rgb, depth, odom, t):
        """rgb: HxWx3 uint8; depth: HxW float32 metres along the optical axis; odom: (x, y, yaw) from
        odometry, used only to keep recently seen points in place as the robot moves; t: seconds."""
        h, w = depth.shape
        f = (h / 2) / math.tan(math.radians(FOVY_DEG) / 2)
        v, u = np.mgrid[0:h, 0:w]
        x_right = (u - w / 2 + 0.5) * depth / f
        y_down = (v - h / 2 + 0.5) * depth / f
        # Camera frame -> level, robot-aligned frame.
        forward = depth * math.cos(TILT) - y_down * math.sin(TILT)
        height = CAMERA_HEIGHT - (depth * math.sin(TILT) + y_down * math.cos(TILT))
        left = -x_right
        valid = np.isfinite(depth) & (depth > 0.05)

        # The red goal cube, by colour; the pedestal under it (same columns, same depth) is part of the goal.
        r, g, b = (rgb[..., i].astype(np.int32) for i in range(3))
        cube = valid & (r > 110) & (r > 2.5 * g) & (r > 2.5 * b)
        goal, goal_part = {"visible": False}, cube
        if cube.sum() > 20:
            distance = float(np.median(forward[cube]))
            bearing = math.degrees(math.atan2(float(np.median(left[cube])), distance))
            goal = {"visible": True, "distance_m": round(distance, 1), "bearing_deg": round(bearing)}
            columns = cube.any(axis=0)
            goal_part = cube | (valid & columns[None, :] & (np.abs(forward - distance) < 0.4))

        obstacle = valid & ~goal_part & (height > OBSTACLE_HEIGHT) & (forward < MAX_RANGE)
        # A ray that lands below the floor crossed the floor's level at h / (h - height) of the way:
        # the walkable surface ends no later than that point.
        to_floor = CAMERA_HEIGHT / (CAMERA_HEIGHT - np.minimum(height, 0.0))
        drop = valid & (height < -DROP_DEPTH) & (forward * to_floor < MAX_RANGE)
        seen_obstacles = np.c_[forward[obstacle], left[obstacle]]
        seen_edges = np.c_[forward[drop] * to_floor[drop], left[drop] * to_floor[drop]]

        # Short-term memory in the odometry frame, so edges and walls stay known once out of view.
        ox, oy, yaw = odom
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        to_odom = lambda p: np.unique(np.round((p @ rot.T + (ox, oy)) / GRID), axis=0).reshape(-1, 2) * GRID
        self.memory = [m for m in self.memory if t - m[0] < MEMORY_S]
        self.memory.append((t, to_odom(seen_obstacles), to_odom(seen_edges)))
        to_robot = lambda p: (p - (ox, oy)) @ rot
        obstacles = to_robot(np.concatenate([m[1] for m in self.memory]))
        edges = to_robot(np.concatenate([m[2] for m in self.memory]))

        bearing = goal.get("bearing_deg")
        options = {}
        for name, heading in (("forward", 0.0), ("left", TURN), ("right", -TURN), ("backward", math.pi)):
            clear, floor = nearest_in_strip(obstacles, heading), nearest_in_strip(edges, heading)
            options[name] = {"path_clear_m": clear, "floor_ends_m": floor, "free_run_m": min(clear, floor)}
            if bearing is not None and name != "backward":
                options[name]["goal_bearing_after_deg"] = round(bearing - math.degrees(heading))

        objects = []
        result = self.yolo(rgb[..., ::-1], conf=YOLO_CONFIDENCE, retina_masks=True, verbose=False)[0]  # YOLO wants BGR
        if result.masks is not None:
            for cls, conf, mask in zip(result.boxes.cls, result.boxes.conf, result.masks.data):
                m = mask.cpu().numpy().astype(bool) & valid
                if not m.any():
                    continue
                distance = float(np.median(forward[m]))
                bearing_o = math.degrees(math.atan2(float(np.median(left[m])), distance))
                objects.append({
                    "label": result.names[int(cls)],
                    "confidence": round(float(conf), 2),
                    "distance_m": round(distance, 1),
                    "bearing_deg": round(bearing_o),
                    "side": side(bearing_o),
                })
        return {"goal": goal, "options": options, "objects": objects}
