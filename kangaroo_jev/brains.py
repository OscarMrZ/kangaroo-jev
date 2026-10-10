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

# Policy, kept in code. Known rules stay in code: a motion needs this much measured room (m) to be allowed.
MARGIN = {"forward": 0.8, "left": 0.5, "right": 0.5, "backward": 0.6}
# Jev can still veto an allowed motion for reasons the numbers miss (people, odd objects).
SEMANTIC_VETO = 0.3


def geometrically_safe(scene):
    """Motions with enough measured room. The camera cannot see behind, so stepping back is only allowed
    to undo the previous step forward."""
    ok = [m for m in ACTIONS if scene["options"][m]["free_run_m"] > MARGIN[m]]
    if scene["recent_actions"][-1:] != ["forward"]:
        ok = [m for m in ok if m != "backward"]
    return ok


@dataclass
class Decision:
    action: str
    latency_s: float = 0.0
    answers: dict = field(default_factory=dict)  # raw judgments, for the logs
    input_tokens: int = 0
    error: str | None = None


MOTIONS = {
    "forward": "walk straight ahead for one second, about 0.35 m",
    "left": "turn left on the spot for one second, about 28 degrees",
    "right": "turn right on the spot for one second, about 28 degrees",
    "backward": "step backwards for one second, about 0.2 m",
}

SCENE_GUIDE = [
    "The robot is a walking two-legged robot; its goal is to reach the red cube (`goal`), which stands on a "
    "grey pedestal. Bearings are in degrees relative to where the robot faces, positive = to the left.",
    "`options.<motion>` describes what that motion runs into, measured along a robot-wide strip in the "
    "direction the robot would face afterwards: `path_clear_m` is the distance to the nearest obstacle, "
    "`floor_ends_m` the distance to where the walkable floor ends (a platform edge, a bridge side or a hole; "
    "past it the robot falls 1.5 m); `free_run_m` is the smaller of the two, how far the robot could walk that "
    "way. 4.0 means nothing within 4 m. `goal_bearing_after_deg` is where the cube would be after the motion; "
    "near 0 means facing it.",
    "`options.backward` only includes what was seen in the last few seconds: the camera cannot see behind.",
    "`objects` are things the camera recognised (people need more room than boxes).",
    "`recent_actions` are the previous motions, most recent last.",
]


def safety_question(motion):
    from typesafe_sdk import Noul

    return Noul(
        instructions=f"Is it safe for the robot to {MOTIONS[motion]}, as described by `options.{motion}`? "
                     "`legend` explains the scene fields.",
        criteria={
            "true": "After the motion the robot is still on the floor with room to spare: both `path_clear_m` and "
                    "`floor_ends_m` for this motion are comfortably more than the distance it covers (at least "
                    "about 0.8 m for walking, 0.5 m for turning), and it does not get close to a person.",
            "false": "The motion could take the robot off an edge or into an obstacle or person, or leaves it "
                     "with less than that margin.",
        },
    )


def progress_question(motion):
    from typesafe_sdk import Score

    return Score(
        instructions=f"How much would it help the robot reach the red cube to {MOTIONS[motion]} "
                     f"(`options.{motion}`)? Judge progress only, not safety. `legend` explains the scene fields.",
        criteria=[
            "Hurts: leaves the robot facing a direction where it cannot walk far (free run under about 1 m), or "
            "takes it away from the cube with nothing gained.",
            "Neutral: changes little; for example turning towards the cube when the way towards it is blocked "
            "within about 1 m.",
            "Helps a little: walks a free path that keeps the cube within about 60 degrees, or turns to face a "
            "direction with a long free run (over about 2 m) that leads roughly towards the cube.",
            "Helps a lot: walks the robot closer to a cube that is roughly ahead along a free path, or turns it to "
            "face the cube when the free run that way reaches the cube.",
        ],
    )


class JevBrain:
    """Speculative fan-out: a safety Noul and a progress Score for every motion, in one request.

    Code allows only motions with enough measured room; Jev's safety answer can still veto one, and its
    progress score picks among the rest."""

    name = "jev"

    def __init__(self, model="jev-latest"):
        from typesafe_sdk import TypeSafeClient

        self.client = TypeSafeClient()
        self.model = model
        self.questions = {f"safe_{m}": safety_question(m) for m in ACTIONS}
        self.questions |= {f"progress_{m}": progress_question(m) for m in ACTIONS}

    def __call__(self, scene) -> Decision:
        t0 = time.perf_counter()
        try:
            # The legend rides in the state: Jev reads the state once for all eight questions.
            r = self.client.system_one(state={"legend": SCENE_GUIDE, **scene}, questions=self.questions, model=self.model)
        except Exception as e:  # keep the last motion; counted in the benchmark
            last = scene["recent_actions"][-1] if scene["recent_actions"] else "left"
            return Decision(last, time.perf_counter() - t0, error=repr(e))
        safe = {m: r.nouls[f"safe_{m}"].noul for m in ACTIONS}
        progress = {m: r.scores[f"progress_{m}"].score for m in ACTIONS}
        allowed = [m for m in geometrically_safe(scene) if safe[m] >= SEMANTIC_VETO]
        if allowed:
            action = max(allowed, key=lambda m: (progress[m], safe[m]))
        else:  # nothing passes both checks: turn towards the side with the most measured room
            action = max(("left", "right"), key=lambda m: scene["options"][m]["free_run_m"])
        answers = {"safe": safe, "progress": progress,
                   "progress_confidence": {m: r.scores[f"progress_{m}"].confidence for m in ACTIONS}}
        return Decision(action, time.perf_counter() - t0, answers, r.usage.input_tokens)


class HeuristicBrain:
    """Hand-written reactive rule over the same scene description, as a reference point."""

    name = "heuristic"

    def __call__(self, scene) -> Decision:
        goal, safe = scene["goal"], geometrically_safe(scene)
        if not goal["visible"]:
            preferred = ["left", "forward", "right", "backward"]
        elif abs(goal["bearing_deg"]) <= 15:
            preferred = ["forward", "left" if goal["bearing_deg"] > 0 else "right", "right", "left", "backward"]
        else:
            towards = "left" if goal["bearing_deg"] > 0 else "right"
            away = "right" if towards == "left" else "left"
            # turn towards the cube only if that way is open; otherwise keep walking where it is
            first = [towards] if scene["options"][towards]["free_run_m"] >= 1.5 else []
            preferred = first + ["forward", towards, away, "backward"]
        fallback = max(("left", "right"), key=lambda m: scene["options"][m]["free_run_m"])
        action = next((m for m in preferred if m in safe), fallback)
        return Decision(action)


BRAINS = {"jev": JevBrain, "heuristic": HeuristicBrain}
