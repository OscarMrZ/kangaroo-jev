"""Kangaroo (4dof arms) in MuJoCo + PAL RL locomotion policy, driven by /cmd_vel.

ros2 launch sim/kangaroo_systemone.launch.py scenario:=pit seed:=0
"""

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def include(pkg, launch_file, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{share(pkg)}/launch/{launch_file}"), launch_arguments=args.items()
    )


def setup(context):
    from kangaroo_systemone.sim_setup import setup

    setup(share("kangaroo_mujoco"), share("kangaroo_controller_configuration"),
          int(LaunchConfiguration("seed").perform(context)))
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="pit"),
        DeclareLaunchArgument("seed", default_value="0", description="start pose jitter; 0 = nominal"),
        DeclareLaunchArgument("policy", default_value="v357", description="pal_policy_deployer 4dof loco model"),
        OpaqueFunction(function=setup),
        # Humble's controller_manager listens on ~/robot_description; newer distros resolve it globally.
        # Not scoped in a GroupAction: Kangaroo's launch files read their configurations from event handlers.
        SetRemap("/controller_manager/robot_description", "/robot_description"),
        include("kangaroo_mujoco", "kangaroo_mujoco.launch.py", arm_type="4dof", moveit="False",
                world_name=["jev_", LaunchConfiguration("scenario")]),
        ExecuteProcess(cmd=["python3", "-m", "kangaroo_systemone.introspection_shim", "--ros-args", "-p", "use_sim_time:=true"],
                       output="screen"),
        # Give controller_manager and the broadcasters time to come up before the policy claims the joints.
        TimerAction(period=8.0, actions=[
            include("pal_policy_deployer", "kang_mjlab_policy_deployer.launch.py", use_sim_time="true",
                    model=LaunchConfiguration("policy"), robot_type="full_body_4dof"),
        ]),
    ])
