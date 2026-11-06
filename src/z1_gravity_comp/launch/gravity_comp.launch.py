#!/usr/bin/env python3
"""Launch the gravity compensation node (编号 2) for the Z1 arm.

The Gazebo torque-mode simulation (编号 1) must already be running. This file
builds the same robot_description as robot_state_publisher and hands it to the
node, which computes tau_grav = G(q) with KDL and publishes it to
/z1_torque_controller/torque_commands.

Usage:
  ros2 launch z1_gravity_comp gravity_comp.launch.py
  ros2 launch z1_gravity_comp gravity_comp.launch.py mode:=soft_hold
  ros2 launch z1_gravity_comp gravity_comp.launch.py gravity_factor:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_description = FindPackageShare("z1_description")
    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "z1.urdf.xacro"])

    declared_args = [
        DeclareLaunchArgument(
            "use_gripper", default_value="false", choices=["true", "false"],
            description="Must match the running simulation (torque mode = false)."),
        DeclareLaunchArgument(
            "mode", default_value="zero_g", choices=["zero_g", "soft_hold"],
            description="zero_g: pure floating; soft_hold: weak PD back to the enable pose."),
    ]

    # 注意:数值型参数(rate/gravity_factor)不在 launch 里传——launch 的
    # LaunchConfiguration 是字符串,会被 rclcpp 拒绝赋给 double 参数。
    # 用节点默认值 + ros2 param set /gravity_comp <name> <value> 运行时设置。
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_gripper:=", LaunchConfiguration("use_gripper"),
                " use_gazebo:=false",
            ]
        ),
        value_type=str,
    )

    node = Node(
        package="z1_gravity_comp",
        executable="gravity_comp",
        name="gravity_comp",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "mode": LaunchConfiguration("mode"),
            }
        ],
    )

    return LaunchDescription(declared_args + [node])
