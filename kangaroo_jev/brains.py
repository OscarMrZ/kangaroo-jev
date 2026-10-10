"""Decision makers: scene description -> one of four discrete motions."""

import time
from dataclasses import dataclass, field

# Fixed velocity commands for the 4dof loco policy (v357, trained on vx in [-1, 1], yaw rate up to 0.7).
# Measured in MuJoCo: ~0.35 m/s forward, ~0.2 m/s backward, ~28 deg/s turning in place.
ACTIONS = {
    "forward": {"vx": 0.5, "wz": 0.0},
    "left": {"vx": 0.0, "wz": 0.7},
    "right": {"vx": 0.0, "wz": -0.7},
    "backward": {"vx": -0.3, "wz": 0.0},
}


@dataclass
class Decision:
    action: str
    latency_s: float = 0.0
    probabilities: dict = field(default_factory=dict)
    confidence: float | None = None
    input_tokens: int = 0
    error: str | None = None


INSTRUCTIONS = {
    "role": "You steer a walking two-legged robot through a room towards a goal.",
    "task": "Choose the robot's next motion. It is executed for about one second, then you choose again.",
    "how_to_read_the_scene": [
        "`free_distance_m.ahead` is the distance to the nearest obstacle in the strip the robot walks through "
        "when going straight; `left` and `right` are the nearest obstacles beside that strip. 4.0 means nothing "
        "within sensing range. These come from a depth camera and include obstacles the camera cannot name.",
        "`objects` are things the camera recognised, with distance and bearing (positive bearing = to the left).",
        "`goal.bearing_deg` is the direction of the goal relative to where the robot faces (positive = left).",
        "`recent_actions` are the previous choices, most recent last.",
    ],
    "priorities": [
        "Never walk into an obstacle or person: keep at least about 0.8 m of clearance.",
        "Otherwise make progress towards the goal.",
        "Avoid oscillating between left and right without making progress.",
    ],
}

CRITERIA = {
    "forward": "Walk straight ahead at about 0.35 m/s. Suitable when the path ahead is free for well over a metre "
               "and the goal is roughly ahead.",
    "left": "Turn left on the spot at about 30 degrees per second. Suitable when the goal is to the left, or when "
            "the way ahead is blocked and the left side is more open.",
    "right": "Turn right on the spot at about 30 degrees per second. Suitable when the goal is to the right, or "
             "when the way ahead is blocked and the right side is more open.",
    "backward": "Step backwards at about 0.2 m/s. Only suitable when something is closer than about 0.6 m ahead "
                "and turning on the spot would still hit it, or the robot is boxed in.",
}


class JevBrain:
    name = "jev"

    def __init__(self, model="jev-latest"):
        from typesafe_sdk import Choice, TypeSafeClient

        self.client = TypeSafeClient()
        self.model = model
        self.question = Choice(instructions=INSTRUCTIONS, criteria=CRITERIA)

    def __call__(self, scene) -> Decision:
        t0 = time.perf_counter()
        try:
            r = self.client.system_one(state=scene, questions={"action": self.question}, model=self.model)
        except Exception as e:  # keep walking with the last action; counted in the benchmark
            return Decision(scene["recent_actions"][-1] if scene["recent_actions"] else "forward",
                            time.perf_counter() - t0, error=repr(e))
        a = r.choices["action"]
        return Decision(a.choice, time.perf_counter() - t0, dict(a.probabilities), a.confidence, r.usage.input_tokens)


class HeuristicBrain:
    """Hand-written reactive rule over the same scene description, as a reference point."""

    name = "heuristic"

    def __call__(self, scene) -> Decision:
        free, bearing = scene["free_distance_m"], scene["goal"]["bearing_deg"]
        more_open = "left" if free["left"] >= free["right"] else "right"
        if free["ahead"] < 0.6 and max(free["left"], free["right"]) < 0.8:
            action = "backward"
        elif free["ahead"] < 1.2:
            action = more_open
        elif abs(bearing) > 20 and free["left" if bearing > 0 else "right"] > 1.2:
            action = "left" if bearing > 0 else "right"
        else:
            action = "forward"
        return Decision(action)


BRAINS = {"jev": JevBrain, "heuristic": HeuristicBrain}
