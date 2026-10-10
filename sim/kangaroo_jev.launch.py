"""Kangaroo in MuJoCo + PAL RL locomotion policies, driven by /cmd_vel.

ros2 launch sim/kangaroo_jev.launch.py scenario:=slalom

Two pal_policy_deployer loco policies are loaded: [0] forward (also turns while walking) and
[1] backward. Switch with the /pal_policy_deployer/switch_policy action.
"""

import os

from ament_index_python.packages import get_package_share_directory as share
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def setup(context):
    from kangaroo_jev.sim_setup import setup

    setup(share("kangaroo_mujoco"), share("kangaroo_controller_configuration"))
    return []


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{share('kangaroo_mujoco')}/launch/kangaroo_mujoco.launch.py"),
        launch_arguments={
            "arm_type": "no-arm",
            "moveit": "False",
            "world_name": ["jev_", LaunchConfiguration("scenario")],
        }.items(),
    )
    policies = Node(
        package="pal_policy_inferer",
        executable="kang_mjlab_node",
        name="pal_policy_deployer",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "policies": ["forward", "backward"],
            "forward.task": "loco",
            "forward.model": LaunchConfiguration("forward_policy"),
            "backward.task": "loco",
            "backward.model": LaunchConfiguration("backward_policy"),
        }],
        remappings=[("/subscriber_controller/desired_state", "/rl_impedance_controller/reference")],
    )
    controller = GroupAction([generate_load_controller_launch_description(
        controller_name="rl_impedance_controller",
        controller_params_file=os.path.join(share("pal_policy_deployer"), "config", "lower_body.yaml"),
    )], forwarding=False)
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="clear"),
        DeclareLaunchArgument("forward_policy", default_value="v639"),
        DeclareLaunchArgument("backward_policy", default_value="v630"),
        OpaqueFunction(function=setup),
        # Humble's controller_manager listens on ~/robot_description; newer distros resolve it globally.
        # Not scoped in a GroupAction: Kangaroo's launch files read their configurations from event handlers.
        SetRemap("/controller_manager/robot_description", "/robot_description"),
        sim,
        ExecuteProcess(cmd=["python3", "-m", "kangaroo_jev.introspection_shim", "--ros-args", "-p", "use_sim_time:=true"],
                       output="screen"),
        # Give controller_manager and the broadcasters time to come up before the policy claims the legs.
        TimerAction(period=8.0, actions=[policies, controller]),
    ])
