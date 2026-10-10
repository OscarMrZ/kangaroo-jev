"""RGB-D frame -> compact scene description. This dict is everything the decision makers see:
no map, no pose; the goal is a red cube they have to find with the camera."""

import math

import numpy as np

FOVY_DEG = 58.0  # head_camera, see sim_setup.py
TILT = math.radians(30.0)  # pitch of the torso RGB-D bracket
CAMERA_HEIGHT = 1.35  # m above the floor while walking
OBSTACLE_HEIGHT = 0.2  # points higher than this above the floor are obstacles
DROP_DEPTH = 0.3  # points lower than this below the floor are a drop (an edge or a hole)
CORRIDOR = 0.35  # half-width of the strip the robot sweeps when walking straight (m)
SIDE = 1.2  # left/right regions run from the strip out to this lateral distance (m)
MAX_RANGE = 4.0  # m; also reported when a region is clear


def side(bearing_deg):
    return "left" if bearing_deg > 15 else "right" if bearing_deg < -15 else "ahead"


class Perception:
    def __init__(self, model_path, confidence=0.35):
        from ultralytics import YOLO

        self.yolo = YOLO(model_path)
        self.confidence = confidence

    def describe(self, rgb, depth):
        """rgb: HxWx3 uint8, depth: HxW float32 metres along the optical axis."""
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
        r, g, b = (rgb[..., i].astype(np.int32) for i in range(3))
        cube = valid & (r > 110) & (r > 2.5 * g) & (r > 2.5 * b)  # the red goal cube
        obstacle = valid & ~cube & (height > OBSTACLE_HEIGHT) & (forward < MAX_RANGE)
        # A ray that lands below the floor crossed the floor's level at h / (h - height) of the way:
        # the walkable surface ends no later than that point.
        to_floor = CAMERA_HEIGHT / (CAMERA_HEIGHT - np.minimum(height, 0.0))
        edge, edge_left = forward * to_floor, left * to_floor
        drop = valid & (height < -DROP_DEPTH) & (edge < MAX_RANGE)

        regions = lambda lat: {"left": (lat > CORRIDOR) & (lat <= SIDE), "ahead": np.abs(lat) <= CORRIDOR,
                               "right": (lat < -CORRIDOR) & (lat >= -SIDE)}
        free, drops = {}, {}
        for (name, region), edge_region in zip(regions(left).items(), regions(edge_left).values()):
            d = forward[obstacle & region]
            free[name] = round(float(d.min()), 1) if d.size else MAX_RANGE
            e = edge[drop & edge_region]
            drops[name] = round(float(e.min()), 1) if e.size else MAX_RANGE

        objects = []
        result = self.yolo(rgb[..., ::-1], conf=self.confidence, retina_masks=True, verbose=False)[0]  # YOLO wants BGR
        if result.masks is not None:
            for cls, conf, mask in zip(result.boxes.cls, result.boxes.conf, result.masks.data):
                m = mask.cpu().numpy().astype(bool) & valid
                if not m.any():
                    continue
                distance = float(np.median(forward[m]))
                bearing = math.degrees(math.atan2(float(np.median(left[m])), distance))
                objects.append({
                    "label": result.names[int(cls)],
                    "confidence": round(float(conf), 2),
                    "distance_m": round(distance, 1),
                    "bearing_deg": round(bearing),
                    "side": side(bearing),
                })
        goal = {"visible": False}
        if cube.sum() > 20:
            distance = float(np.median(forward[cube]))
            bearing = math.degrees(math.atan2(float(np.median(left[cube])), distance))
            goal = {"visible": True, "distance_m": round(distance, 1), "bearing_deg": round(bearing)}
        return {"goal": goal, "free_distance_m": free, "drop_distance_m": drops, "objects": objects}
