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
    "grey pedestal. Bearings are in degrees relative to where the robot faces, positive = to the left. When "
    "the cube is out of view, `goal.remembered` gives where it was last seen, carried along with the robot's "
    "own motion.",
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


def safety_question(motion, sdk):
    return sdk.Noul(
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


def progress_question(motion, sdk):
    return sdk.Score(
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


def questions(sdk):
    """The 8 questions of one request. `sdk` is typesafe_sdk or clm: both have Noul and Score with the same fields."""
    return {f"safe_{m}": safety_question(m, sdk) for m in ACTIONS} | {f"progress_{m}": progress_question(m, sdk)
                                                                       for m in ACTIONS}


class SystemOneBrain:
    """Speculative fan-out: a safety Noul and a progress Score for every motion, in one request.

    Code allows only motions with enough measured room; the model's safety answer can still veto one, and its
    progress score picks among the rest. Subclasses only send the request."""

    def ask(self, state):
        """-> ({motion: p(safe)}, {motion: progress}, {motion: progress confidence}, input tokens)"""
        raise NotImplementedError

    def __call__(self, scene) -> Decision:
        t0 = time.perf_counter()
        try:
            # The legend rides in the state: the model reads the state once for all eight questions.
            safe, progress, confidence, tokens = self.ask({"legend": SCENE_GUIDE, **scene})
        except Exception as e:  # keep the last motion; counted in the benchmark
            last = scene["recent_actions"][-1] if scene["recent_actions"] else "left"
            return Decision(last, time.perf_counter() - t0, error=repr(e))
        allowed = [m for m in geometrically_safe(scene) if safe[m] >= SEMANTIC_VETO]
        if allowed:
            action = max(allowed, key=lambda m: (progress[m], safe[m]))
        else:  # nothing passes both checks: turn towards the side with the most measured room
            action = max(("left", "right"), key=lambda m: scene["options"][m]["free_run_m"])
        answers = {"safe": safe, "progress": progress, "progress_confidence": confidence}
        return Decision(action, time.perf_counter() - t0, answers, tokens or 0)


class JevBrain(SystemOneBrain):
    """TypeSafe Jev over the TypeSafe API."""

    name = "jev"

    def __init__(self, model="jev-latest"):
        import typesafe_sdk

        self.client, self.model, self.questions = typesafe_sdk.TypeSafeClient(), model, questions(typesafe_sdk)

    def ask(self, state):
        r = self.client.system_one(state=state, questions=self.questions, model=self.model)
        return ({m: r.nouls[f"safe_{m}"].noul for m in ACTIONS}, {m: r.scores[f"progress_{m}"].score for m in ACTIONS},
                {m: r.scores[f"progress_{m}"].confidence for m in ACTIONS}, r.usage.input_tokens)


class ClmBrain(SystemOneBrain):
    """CLM-8B (Contrastive-LM) behind its TypeSafe-compatible API, served locally (`docker compose up clm`)."""

    name = "clm"

    def __init__(self, model="clm-latest"):
        import clm

        self.client, self.model, self.questions = clm.CLMClient(), model, questions(clm)
        if not self.client.health():
            raise RuntimeError(f"no CLM server at {self.client.base_url}: start it with `docker compose up -d clm`")

    def ask(self, state):
        r = self.client.system_one(state, self.questions, model=self.model)
        a = r.answers
        # input_tokens: encoder tokens actually computed (cache misses), not the size of the request
        return ({m: a[f"safe_{m}"].noul for m in ACTIONS}, {m: a[f"progress_{m}"].score for m in ACTIONS},
                {m: a[f"progress_{m}"].confidence for m in ACTIONS}, r.usage.input_tokens)


# ---- CLM-native questions -----------------------------------------------------------------------------------
# A contrastive model embeds the state and each answer separately and compares them, so whatever tells the motions
# apart has to be in the answer texts, and numbers have to become words. Safety stays in code: only allowed motions
# are offered.

MOVES = {"forward": "Walk forward", "left": "Turn left", "right": "Turn right", "backward": "Step back"}
NATIVE_QUESTION = "Which move gets the robot closer to the red cube, or helps it find the cube if it is out of sight?"


def where(bearing_deg):
    a, side = abs(bearing_deg), "left" if bearing_deg > 0 else "right"
    if a < 15:
        return "straight ahead"
    if a < 60:
        return f"ahead to the {side}"
    return f"off to the {side}" if a < 120 else "behind the robot"


def room(option):
    what = "the floor ends" if option["floor_ends_m"] <= option["path_clear_m"] else "something is in the way"
    run = option["free_run_m"]
    if run >= 4.0:
        return "open floor as far as it can see"
    if run >= 2.0:
        return f"a long stretch of open floor before {what}"
    return f"a short stretch of open floor before {what}" if run >= 0.8 else f"{what} right in front"


def situation(scene):
    """The scene in two or three plain sentences."""
    g, words = scene["goal"], ["A walking robot is trying to reach a red cube."]
    if g.get("visible"):
        words.append(f"The cube is about {g['distance_m']:.0f} m away, {where(g['bearing_deg'])}.")
    elif g.get("remembered"):
        words.append(f"The cube is out of view; it was last seen about {g['distance_m']:.0f} m away, "
                     f"{where(g['bearing_deg'])}.")
    else:
        words.append("The cube is out of sight and the robot does not know where it is.")
    words += [f"A {o['label']} is about {o['distance_m']:.0f} m away, {where(o['bearing_deg'])}." for o in scene["objects"]]
    if scene["recent_actions"]:
        words.append(f"Its last moves were: {', '.join(scene['recent_actions'][-4:])}.")
    return " ".join(words)


def outcome(motion, scene):
    """What a motion leads to, in words: the answer text CLM compares against the situation."""
    if motion == "backward":
        return "Step back: undo the last step forward."
    o = scene["options"][motion]
    cube = (f", and the cube would then be {where(o['goal_bearing_after_deg'])}" if "goal_bearing_after_deg" in o
            else ", and the cube stays out of sight")
    return f"{MOVES[motion]}: {room(o)}{cube}."


def native_questions(scene, allowed, sdk, mode):
    """'choice': one Choice among the allowed motions. 'noul': a Noul per motion, the outcome in the question."""
    if mode == "choice":
        return {"move": sdk.Choice(instructions=NATIVE_QUESTION, criteria={m: outcome(m, scene) for m in allowed})}
    return {m: sdk.Noul(instructions=f"If the robot does this next: {outcome(m, scene)} Does this move get it closer "
                                     "to the red cube, or help it find the cube?",
                        criteria={"true": "Yes, this move helps the robot reach or find the red cube.",
                                  "false": "No, this move does not help the robot reach the red cube."})
            for m in allowed}


class ClmNativeBrain:
    """CLM-8B with CLM-native questions (see above); the motion with the highest probability among the allowed ones.
    `image`: also send the camera frame (needs an image CLM server and clm.content, e.g. kangaroo-clm's)."""

    name, mode, image = "clm_native", "choice", False

    def __init__(self, model="clm-latest"):
        import clm

        self.sdk, self.client, self.model = clm, clm.CLMClient(), model
        self.frame = None  # set by run_episode before every decision
        if not self.client.health():
            raise RuntimeError(f"no CLM server at {self.client.base_url}: start it with `docker compose up -d clm`")

    def state(self, scene):
        text = situation(scene)
        if not self.image or self.frame is None:
            return text
        import io

        from clm.content import image_part, with_images
        from PIL import Image

        buf = io.BytesIO()
        Image.fromarray(self.frame).save(buf, format="JPEG", quality=92)
        return with_images(text, [image_part(buf.getvalue(), mime="image/jpeg")])

    def __call__(self, scene) -> Decision:
        t0 = time.perf_counter()
        allowed = geometrically_safe(scene)
        if not allowed:  # as for Jev: turn towards the side with the most measured room
            return Decision(max(("left", "right"), key=lambda m: scene["options"][m]["free_run_m"]), 0.0)
        try:
            r = self.client.system_one(self.state(scene), native_questions(scene, allowed, self.sdk, self.mode),
                                       model=self.model)
        except Exception as e:  # keep the last motion; counted in the benchmark
            last = scene["recent_actions"][-1] if scene["recent_actions"] else "left"
            return Decision(last, time.perf_counter() - t0, error=repr(e))
        p = (r.answers["move"].probabilities if self.mode == "choice" else {m: r.answers[m].noul for m in allowed})
        return Decision(max(allowed, key=p.get), time.perf_counter() - t0, {"native": p}, r.usage.input_tokens or 0)


class ClmNativeNoulBrain(ClmNativeBrain):
    name, mode = "clm_native_noul", "noul"


class ClmImageNativeBrain(ClmNativeBrain):
    """kangaroo-clm's image CLM (Qwen3-VL-8B encoder + fine-tuned head) with the CLM-native questions and the frame."""

    name, image = "clm_image_native", True


class HeuristicBrain:
    """Hand-written reactive rule over the same scene description, as a reference point."""

    name = "heuristic"

    def __call__(self, scene) -> Decision:
        goal, safe = scene["goal"], geometrically_safe(scene)
        if "bearing_deg" not in goal:
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


BRAINS = {"jev": JevBrain, "clm": ClmBrain, "clm_native": ClmNativeBrain, "clm_native_noul": ClmNativeNoulBrain,
          "clm_image_native": ClmImageNativeBrain, "heuristic": HeuristicBrain}
