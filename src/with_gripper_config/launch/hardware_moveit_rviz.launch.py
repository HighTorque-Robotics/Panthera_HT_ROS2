import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package paths
    with_gripper_config_path = FindPackageShare('with_gripper_config')

    # Construct relative path to config file
    default_config_file = PathJoinSubstitution([
        with_gripper_config_path,
        'robot_param',
        'Follower_absolute.yaml'
    ])

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to robot configuration YAML file'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position_velocity',
        description='Control mode: position_velocity, pd_control, or full_control'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )

    # ============================================
    # 1. Hardware Launch (robot_state_publisher, controller_manager, controllers)
    # ============================================
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'control_mode': LaunchConfiguration('control_mode'),
        }.items()
    )

    # ============================================
    # 2. MoveIt Config
    # ============================================
    moveit_config = MoveItConfigsBuilder("Panthera-HT_description", package_name="with_gripper_config").to_moveit_configs()

    # For real hardware, use system time, not simulation time
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_robot_description": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,  # CRITICAL: Use system time for real hardware
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # ============================================
    # 3. Move Group Node
    # ============================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ============================================
    # 4. RViz Node - simplified
    # ============================================
    rviz_config_file = PathJoinSubstitution([
        with_gripper_config_path,
        'config',
        'moveit.rviz'
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": False},  # CRITICAL: Use system time for real hardware
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        config_file_arg,
        control_mode_arg,
        rviz_arg,
        hardware_launch,
        move_group_node,
        rviz_node,
    ])
