"""Idempotently adapt the installed PAL packages for this benchmark.

- edits the 4dof MJCF in place: an idealized RealSense D435 (pinhole RGB-D, 87x58 deg, 424x240, depth
  aligned to colour) on the torso camera's colour frame; the model has the camera frames but no camera
- writes one MuJoCo world + scene per benchmark scenario, with the start pose for a seed
- enables the mujoco_ros2_control camera and free-joint-state plugins
- raises the joint-state / IMU broadcaster rates so the 50 Hz policy sees fresh observations
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

from kangaroo_systemone.scenarios import SCENARIOS, start_pose, world_xml

VARIANT = "mjcf_data_floating_4dof_no-end-effector_no-end-effector_fixed"
CAMERA_SITE = "torso_front_up_rgbd_camera_color_frame"
CAMERA = "head_camera"
MARKER = "# --- kangaroo_systemone ---"

PLUGINS_YAML = f"""
{MARKER}
/**:
  ros__parameters:
    mujoco_plugins:
      head_camera_plugin:
        type: mujoco_ros2_control_plugins/CameraPlugin
        camera_publish_rate: 5.0
        {CAMERA}:
          image_topic: /{CAMERA}/color
          depth_topic: /{CAMERA}/depth
          info_topic: /{CAMERA}/camera_info
      base_state_plugin:
        type: mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin
        body_names: [base_link]
        topic: /base_state
        publish_rate: 20.0
"""


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def add_camera(mjcf: Path):
    tree = ET.parse(mjcf)
    if tree.find(f".//camera[@name='{CAMERA}']") is not None:
        return
    parent = tree.find(f".//site[@name='{CAMERA_SITE}']/..")
    site = parent.find(f"site[@name='{CAMERA_SITE}']")
    # ROS link frame (x fwd, z up) -> MuJoCo camera frame (looks along -z, y up).
    site_q = tuple(float(v) for v in site.get("quat").split())
    q = _qmul(site_q, (0.5, 0.5, -0.5, -0.5))
    ET.SubElement(
        parent,
        "camera",
        name=CAMERA,
        pos=site.get("pos"),
        quat=" ".join(f"{v:.6f}" for v in q),
        fovy="58",
        resolution="424 240",
    )
    tree.write(mjcf)


def make_person_texture(path: Path):
    """Crop the most confident person out of Ultralytics' bundled bus.jpg."""
    if path.exists():
        return
    import os

    from PIL import Image
    from ultralytics import ASSETS, YOLO

    image = ASSETS / "bus.jpg"
    boxes = YOLO(os.environ.get("YOLO_MODEL", "yolo26n-seg.pt"))(image, classes=[0], verbose=False)[0].boxes
    x0, y0, x1, y1 = boxes.xyxy[int(boxes.conf.argmax())].tolist()
    # Cube textures must be square; the rotation makes the -x face (towards the robot) upright.
    Image.open(image).crop((x0, y0, x1, y1)).resize((512, 512)).rotate(270).save(path)


def setup(kangaroo_mujoco_share: str, controller_config_share: str, seed: int = 0):
    share = Path(kangaroo_mujoco_share)
    models = share / "models"
    variant = models / VARIANT

    add_camera(variant / "mujoco_description_formatted.xml")

    texture = models / "worlds" / "person.png"
    make_person_texture(texture)
    for name, scenario in SCENARIOS.items():
        (models / "worlds" / f"jev_{name}.xml").write_text(world_xml(scenario, texture, start_pose(seed)))
        (variant / f"scene_jev_{name}.xml").write_text(
            '<mujoco model="scene">\n'
            f'  <include file="../worlds/jev_{name}.xml"/>\n'
            '  <include file="mujoco_description_formatted.xml"/>\n'
            "</mujoco>\n"
        )

    cm = share / "config" / "controller_manager.yaml"
    cm.write_text(cm.read_text().split(MARKER)[0].rstrip() + "\n" + PLUGINS_YAML)

    config = Path(controller_config_share) / "config"
    for name, rate in (("joint_state_broadcaster.yaml", 200), ("sensors_broadcaster.yaml", 200)):
        f = config / name
        f.write_text(re.sub(r"update_rate: \d+", f"update_rate: {rate}", f.read_text()))


if __name__ == "__main__":
    from ament_index_python.packages import get_package_share_directory as share_dir

    setup(share_dir("kangaroo_mujoco"), share_dir("kangaroo_controller_configuration"))
    print("kangaroo_systemone sim setup done")
