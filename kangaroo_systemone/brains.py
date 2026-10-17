"""Decision makers: scene description (and camera frame) -> one of four discrete motions.

A decision maker is called once per move as `brain(scene, frame)` and reset at the start of every episode. They are
registered in BRAINS; `make_brain("name@model")` builds one with another model, e.g. `jev@jev-preview`.
"""

import argparse
import math
import time
from collections import deque
from dataclasses import dataclass, field
from functools import partial

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


def fallback_turn(scene):
    """Nothing is allowed: turn towards the side with the most measured room."""
    return max(("left", "right"), key=lambda m: scene["options"][m]["free_run_m"])


def failed(scene, t0, e):
    """The model could not be asked: keep the last motion; counted in the benchmark."""
    last = scene["recent_actions"][-1] if scene["recent_actions"] else "left"
    return Decision(last, time.perf_counter() - t0, error=repr(e))


class Brain:
    name = "brain"  # set by make_brain to the spec it was built from

    def reset(self):
        """Forget the episode so far; called at the start of every episode."""

    def __call__(self, scene, frame) -> Decision:
        """scene: perception's description; frame: the head camera's HxWx3 uint8 RGB image it was measured from."""
        raise NotImplementedError


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
# Added to the legend for decision makers that also get camera images.
FRAMES_GUIDE = ("`frames` are images from the robot's head camera, one per move, oldest first: the last one "
                "(`moves_ago` 0) is what the camera sees now, and the numbers above were measured from it. "
                "`action_after` is the motion the robot made right after an earlier frame.")
MIN_SIDE = 280  # px: TypeSafe rejects images with a shorter side; the head camera's frames are 424x240


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


class SystemOneBrain(Brain):
    """Speculative fan-out: a safety Noul and a progress Score for every motion, in one request.

    Code allows only motions with enough measured room; the model's safety answer can still veto one, and its
    progress score picks among the rest. `frames`: also send the last that many camera frames (0: text only).
    Subclasses send the request and turn a frame into an image."""

    def __init__(self, sdk, model, frames=0):
        self.model, self.frames, self.questions = model, frames, questions(sdk)
        self.reset()

    def reset(self):
        self.past = deque(maxlen=max(self.frames - 1, 0))  # (image, motion made after it)

    def request(self, state):
        """-> a response whose `answers` hold the eight questions' answers"""
        raise NotImplementedError

    def image(self, frame):
        raise NotImplementedError(f"{type(self).__name__} does not take images")

    def state(self, scene, image):
        # The legend rides in the state: the model reads the state once for all eight questions.
        if image is None:
            return {"legend": SCENE_GUIDE, **scene}
        n = len(self.past)
        frames = [{"moves_ago": n - i, "image": im, "action_after": a} for i, (im, a) in enumerate(self.past)]
        return {"legend": SCENE_GUIDE + [FRAMES_GUIDE], **scene, "frames": frames + [{"moves_ago": 0, "image": image}]}

    def __call__(self, scene, frame) -> Decision:
        t0 = time.perf_counter()
        image = self.image(frame) if self.frames else None
        try:
            r = self.request(self.state(scene, image))
        except Exception as e:
            decision = failed(scene, t0, e)
        else:
            a = r.answers
            safe = {m: a[f"safe_{m}"].noul for m in ACTIONS}
            progress = {m: a[f"progress_{m}"].score for m in ACTIONS}
            allowed = [m for m in geometrically_safe(scene) if safe[m] >= SEMANTIC_VETO]
            action = max(allowed, key=lambda m: (progress[m], safe[m])) if allowed else fallback_turn(scene)
            answers = {"safe": safe, "progress": progress,
                       "progress_confidence": {m: a[f"progress_{m}"].confidence for m in ACTIONS}}
            # CLM's input_tokens: encoder tokens actually computed (cache misses), not the size of the request
            decision = Decision(action, time.perf_counter() - t0, answers, r.usage.input_tokens or 0)
        if self.frames:
            self.past.append((image, decision.action))
        return decision


class JevBrain(SystemOneBrain):
    """TypeSafe Jev over the TypeSafe API; with `frames`, Jev also sees the camera (experimental image input)."""

    def __init__(self, model="jev-latest", frames=0):
        import typesafe_sdk

        self.client = typesafe_sdk.TypeSafeClient()
        super().__init__(typesafe_sdk, model, frames)

    def image(self, frame):
        from PIL import Image
        from typesafe_sdk import Image as TypeSafeImage

        im = Image.fromarray(frame)
        scale = MIN_SIDE / min(im.size)
        if scale > 1:
            im = im.resize((math.ceil(im.width * scale), math.ceil(im.height * scale)), Image.LANCZOS)
        return TypeSafeImage.from_pil(im, format="jpeg")

    def request(self, state):
        return self.client.system_one(state=state, questions=self.questions, model=self.model,
                                      extra_headers={"TypeSafe-Danger-Zone-Images": "true"} if self.frames else None)


def clm_client():
    import clm

    client = clm.CLMClient()
    if not client.health():
        raise RuntimeError(f"no CLM server at {client.base_url}: start it with `docker compose up -d clm`")
    return client


class ClmBrain(SystemOneBrain):
    """CLM-8B (Contrastive-LM) behind its TypeSafe-compatible API, served locally (`docker compose up clm`)."""

    def __init__(self, model="clm-latest"):
        import clm

        self.client = clm_client()
        super().__init__(clm, model)

    def request(self, state):
        return self.client.system_one(state, self.questions, model=self.model)


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


def native_questions(scene, allowed, sdk):
    """One Choice among the allowed motions, each option the motion's outcome in words."""
    return {"move": sdk.Choice(instructions=NATIVE_QUESTION, criteria={m: outcome(m, scene) for m in allowed})}


class ClmNativeBrain(Brain):
    """CLM-8B with CLM-native questions (see above); the motion with the highest probability among the allowed ones.
    `image`: also send the camera frame (needs an image CLM server and clm.content, e.g. kangaroo-clm's)."""

    def __init__(self, model="clm-latest", image=False):
        import clm

        self.sdk, self.client, self.model, self.image = clm, clm_client(), model, image

    def state(self, scene, frame):
        text = situation(scene)
        if not self.image:
            return text
        import io

        from clm.content import image_part, with_images
        from PIL import Image

        buf = io.BytesIO()
        Image.fromarray(frame).save(buf, format="JPEG", quality=92)
        return with_images(text, [image_part(buf.getvalue(), mime="image/jpeg")])

    def __call__(self, scene, frame) -> Decision:
        t0 = time.perf_counter()
        allowed = geometrically_safe(scene)
        if not allowed:
            return Decision(fallback_turn(scene))
        try:
            r = self.client.system_one(self.state(scene, frame), native_questions(scene, allowed, self.sdk),
                                       model=self.model)
        except Exception as e:
            return failed(scene, t0, e)
        p = r.answers["move"].probabilities
        return Decision(max(allowed, key=p.get), time.perf_counter() - t0, {"native": p}, r.usage.input_tokens or 0)


class HeuristicBrain(Brain):
    """Hand-written reactive rule over the same scene description, as a reference point."""

    def __call__(self, scene, frame=None) -> Decision:
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
        return Decision(next((m for m in preferred if m in safe), fallback_turn(scene)))


# name: (label for reports, factory taking an optional model).
BRAINS = {
    "jev": ("Jev", JevBrain),
    "jev_multi": ("Jev multi", partial(JevBrain, frames=1)),
    "jev_multi_10": ("Jev multi, 10 frames", partial(JevBrain, frames=10)),
    "clm": ("CLM naive", ClmBrain),
    "clm_native": ("CLM native", ClmNativeBrain),
    "clm_image_native": ("CLM image native", partial(ClmNativeBrain, image=True)),
    "heuristic": ("rule", HeuristicBrain),
}
# Everything that runs from this repo alone (clm_image_native needs kangaroo-clm's image server and clm library).
DEFAULT_BRAINS = ["jev", "jev_multi", "jev_multi_10", "clm", "clm_native", "heuristic"]


def brain_spec(spec):
    """argparse type: `name` or `name@model`."""
    if spec.partition("@")[0] not in BRAINS:
        raise argparse.ArgumentTypeError(f"unknown decision maker {spec!r}; choose from {', '.join(BRAINS)}")
    return spec


def make_brain(spec):
    name, _, model = spec.partition("@")
    brain = BRAINS[name][1](**({"model": model} if model else {}))
    brain.name = spec
    return brain


def label(spec):
    """Report label of a spec; runs made elsewhere keep their name."""
    name, _, model = spec.partition("@")
    return (BRAINS[name][0] if name in BRAINS else name) + (f" ({model})" if model else "")
