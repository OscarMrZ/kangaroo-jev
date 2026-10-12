"""Republish /joint_states + torso IMU as ros2_control introspection data for PAL's policy node.

kang_mjlab_node reads robot state from controller_manager's pal_statistics introspection
topics (ros2_control >= Jazzy), which Humble's controller_manager does not publish. IMU
readings are passed through unchanged: the MJCF IMU site matches the one in pal_mjlab's
training model.

kang_mjlab_node runs inference on every values message it picks up (polling at 50 Hz wall
time), which the shim uses for two things:
- publishing at 50 Hz *sim* time keeps the policy at its trained 50 Hz control rate even when
  MuJoCo runs slower than real time;
- publishing only once rl_impedance_controller is active means the policy's first action is the
  first one applied. Otherwise it runs open-loop while MuJoCo still holds the start pose, and the
  handover can knock the robot over.
"""

import rclpy
from control_msgs.msg import MultiDOFStateStamped
from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

IMU = "torso_imu_sensor"
IMU_FIELDS = [f"orientation.{a}" for a in "wxyz"] + [
    f"{q}.{a}" for q in ("angular_velocity", "linear_acceleration") for a in "xyz"
]
TOPIC = "/controller_manager/introspection_data"
POLICY_PERIOD = 0.02


class IntrospectionShim(Node):
    def __init__(self):
        super().__init__("introspection_shim")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.names_pub = self.create_publisher(StatisticsNames, f"{TOPIC}/names", latched)
        self.values_pub = self.create_publisher(
            StatisticsValues, f"{TOPIC}/values", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(Imu, f"/{IMU}_broadcaster/imu", self.on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.controller_sub = self.create_subscription(
            MultiDOFStateStamped, "/rl_impedance_controller/controller_state", self.on_controller_active, 1
        )
        self.active = False
        self.imu = None
        self.names = None
        self.version = 0
        self.next_t = 0.0

    def on_controller_active(self, _):
        if not self.active:
            self.active = True
            self.destroy_subscription(self.controller_sub)

    def on_imu(self, m: Imu):
        o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        self.imu = [o.w, o.x, o.y, o.z, w.x, w.y, w.z, a.x, a.y, a.z]

    def on_joints(self, js: JointState):
        t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        if not self.active or self.imu is None or len(js.velocity) != len(js.name) or t < self.next_t:
            return
        self.next_t = max(self.next_t + POLICY_PERIOD, t - POLICY_PERIOD)
        names = (
            [f"state_interface.{IMU}/{f}" for f in IMU_FIELDS]
            + [f"state_interface.{n}/position" for n in js.name]
            + [f"state_interface.{n}/velocity" for n in js.name]
        )
        if names != self.names:
            self.names, self.version = names, self.version + 1
            self.names_pub.publish(StatisticsNames(header=js.header, names=names, names_version=self.version))
        values = self.imu + list(js.position) + list(js.velocity)
        self.values_pub.publish(StatisticsValues(header=js.header, values=values, names_version=self.version))


def main():
    rclpy.init()
    rclpy.spin(IntrospectionShim())


if __name__ == "__main__":
    main()
