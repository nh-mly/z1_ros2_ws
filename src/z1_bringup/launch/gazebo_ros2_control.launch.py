#!/usr/bin/env python3
"""Z1 in Gazebo Sim with ros2_control (gz_ros2_control).

Gazebo steps the physics, gz_ros2_control runs controller_manager inside the
server and drives the joints through the same controller set as the real arm
(joint_state_broadcaster, joint_trajectory_controller, gripper_controller).
No joint topic bridge is needed - only /clock.

MoveIt can be added on top:
    ros2 launch z1_bringup gazebo_ros2_control.launch.py use_rviz:=false &
    ros2 launch z1_moveit_config move_group.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from z1_bringup.launch_parts import (
    ARM_CONTROLLERS,
    CONTROLLER_MANAGER,
    GRIPPER_CONTROLLER,
    TORQUE_CONTROLLERS,
    controller_spawner,
    common_args,
    gz_sim_includes,
    robot_state_publisher,
    rviz,
    spawn_model,
)

HARDWARE_PLUGIN = "gz_ros2_control/GazeboSimSystem"

# The gz_ros2_control plugin (and with it controller_manager) only exists once
# the model has been spawned, so the spawners are started a bit later and are
# allowed to wait for it.
SPAWNER_DELAY_S = 12.0
SPAWNER_CM_TIMEOUT_S = 60


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare("z1_bringup")
    pkg_description = FindPackageShare("z1_description")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "z1.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_description, "rviz", "z1_display.rviz"])
    clock_bridge_config = PathJoinSubstitution(
        [pkg_bringup, "config", "ros_gz_bridge_clock.yaml"]
    )

    declared_args = common_args() + [
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [pkg_bringup, "config", "z1_controllers_gz_effort.yaml"]
            ),
            description=(
                "controller_manager parameters; the default drives the arm through "
                "the effort interfaces with the official torque law. Pass "
                "config/z1_controllers.yaml for the plain position servo instead."
            ),
        ),
        DeclareLaunchArgument(
            "use_torque_control",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Spawn z1_torque_controller instead of joint_trajectory_controller. "
                "Use together with controllers_file:=config/z1_controllers_gz_torque.yaml "
                "(the torque controller and the trajectory controller claim the same "
                "effort interfaces, so they cannot share one controllers_file)."
            ),
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_gripper:=",
                LaunchConfiguration("use_gripper"),
                " hardware_plugin:=",
                HARDWARE_PLUGIN,
                " use_gazebo:=false",
                " controllers_file:=",
                LaunchConfiguration("controllers_file"),
            ]
        ),
        value_type=str,
    )

    clock_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_bridge"), "/launch/ros_gz_bridge.launch.py"]
        ),
        launch_arguments={
            "bridge_name": "clock_bridge",
            "config_file": clock_bridge_config,
        }.items(),
    )

    spawner_args = [
        "--controller-manager",
        CONTROLLER_MANAGER,
        "--controller-manager-timeout",
        str(SPAWNER_CM_TIMEOUT_S),
    ]

    start_nodes = gz_sim_includes() + [
        clock_bridge,
        robot_state_publisher(robot_description),
        spawn_model(),
    ]

    def spawner_actions(context):
        """力矩模式 spawn 力矩控制器,否则 spawn 默认轨迹控制器。

        launch 里不能用普通 Python 条件判断 LaunchConfiguration 的值,
        所以放在 OpaqueFunction 里按解析后的参数决定 spawn 列表。
        """
        use_torque = (
            context.launch_configurations.get("use_torque_control", "false").lower() == "true"
        )
        controllers = TORQUE_CONTROLLERS if use_torque else ARM_CONTROLLERS
        actions = [controller_spawner(c, spawner_args, use_sim_time=True) for c in controllers]
        if not use_torque:
            actions.append(
                controller_spawner(
                    GRIPPER_CONTROLLER,
                    spawner_args,
                    use_sim_time=True,
                    condition=IfCondition(LaunchConfiguration("use_gripper")),
                )
            )
        return actions

    spawners = TimerAction(
        period=SPAWNER_DELAY_S,
        actions=[OpaqueFunction(function=spawner_actions)],
    )

    return LaunchDescription(
        declared_args + start_nodes + [spawners, rviz(rviz_config)]
    )
