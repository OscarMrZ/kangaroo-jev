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


NOMINAL_DOWN = np.array([0.0, math.cos(TILT), math.sin(TILT)])  # floor normal (pointing down) in the camera frame


def floor_plane(points):
    """Least-squares floor plane n.p = h through the given camera-frame points, trimmed once for outliers.
    Returns (unit normal pointing down, camera height); the nominal mount when the fit is implausible."""
    nominal = NOMINAL_DOWN, CAMERA_HEIGHT
    if len(points) < 500:
        return nominal
    pts = points[:: max(1, len(points) // 4000)]
    for _ in range(2):
        abc, *_ = np.linalg.lstsq(pts, np.ones(len(pts)), rcond=None)  # a.x + b.y + c.z = 1
        cam_h = 1.0 / np.linalg.norm(abc)
        normal = abc * cam_h
        resid = np.abs(pts @ normal - cam_h)
        pts = pts[resid < max(0.03, np.percentile(resid, 80))]
    tilt_error = math.degrees(math.acos(min(1.0, float(normal @ NOMINAL_DOWN))))
    return (normal, cam_h) if tilt_error < 15 and 1.1 < cam_h < 1.6 else nominal


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
        self.reset()

    def reset(self):
        """Forget everything remembered; call at the start of every episode."""
        self.memory = []  # (t, obstacle points, edge points), both in the odometry frame
        self.goal_seen = None  # (t, cube position in the odometry frame)

    def describe(self, rgb, depth, odom, t):
        """rgb: HxWx3 uint8; depth: HxW float32 metres along the optical axis; odom: (x, y, yaw) from
        odometry, used only to keep recently seen points in place as the robot moves; t: seconds."""
        h, w = depth.shape
        f = (h / 2) / math.tan(math.radians(FOVY_DEG) / 2)
        v, u = np.mgrid[0:h, 0:w]
        x_right = (u - w / 2 + 0.5) * depth / f
        y_down = (v - h / 2 + 0.5) * depth / f
        valid = np.isfinite(depth) & (depth > 0.05)
        # Camera frame -> level, robot-aligned frame. The torso pitches and rolls a few degrees while walking,
        # which is enough to turn far floor into phantom obstacles or edges, so the floor plane is fitted to
        # the bottom of the image each frame (the floor right in front), falling back to the nominal mount.
        points = np.stack([x_right, y_down, depth], axis=-1)
        down, cam_h = floor_plane(points[int(h * 0.7):][valid[int(h * 0.7):]])
        height = cam_h - points @ down
        fwd_axis = np.array([0.0, -down[2], down[1]]) / math.hypot(down[1], down[2])  # optical axis, levelled
        forward = points @ fwd_axis
        left = points @ np.cross(fwd_axis, down)  # = -x_right for the nominal mount

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
        to_floor = cam_h / (cam_h - np.minimum(height, 0.0))
        drop = valid & (height < -DROP_DEPTH) & (forward * to_floor < MAX_RANGE)
        seen_obstacles = np.c_[forward[obstacle], left[obstacle]]
        seen_edges = np.c_[forward[drop] * to_floor[drop], left[drop] * to_floor[drop]]

        # Short-term memory in the odometry frame, so edges and walls stay known once out of view.
        ox, oy, yaw = odom
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        to_odom = lambda p: np.unique(np.round((p @ rot.T + (ox, oy)) / GRID), axis=0).reshape(-1, 2) * GRID
        self.memory = [m for m in self.memory if 0 <= t - m[0] < MEMORY_S]
        self.memory.append((t, to_odom(seen_obstacles), to_odom(seen_edges)))
        to_robot = lambda p: (p - (ox, oy)) @ rot
        obstacles = to_robot(np.concatenate([m[1] for m in self.memory]))
        edges = to_robot(np.concatenate([m[2] for m in self.memory]))

        # The cube's last known position, so turning away from it (to get round something) does not lose it.
        if goal["visible"]:
            b = math.radians(goal["bearing_deg"])
            self.goal_seen = (t, (ox + goal["distance_m"] * math.cos(yaw + b), oy + goal["distance_m"] * math.sin(yaw + b)))
        elif self.goal_seen and t - self.goal_seen[0] < MEMORY_S:
            dx, dy = self.goal_seen[1][0] - ox, self.goal_seen[1][1] - oy
            b = math.degrees(math.atan2(dy, dx) - yaw)
            goal = {"visible": False, "remembered": True, "seen_s_ago": round(t - self.goal_seen[0]),
                    "distance_m": round(math.hypot(dx, dy), 1), "bearing_deg": round((b + 180) % 360 - 180)}
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
