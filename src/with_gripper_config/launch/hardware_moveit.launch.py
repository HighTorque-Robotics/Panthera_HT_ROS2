import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file = LaunchConfiguration('config_file')
    control_mode = LaunchConfiguration('control_mode')

    # Get package paths
    with_gripper_config_path = FindPackageShare('with_gripper_config')

    # Default robot config file (Leader configuration)
    default_config_file = PathJoinSubstitution([
        FindPackageShare('hightorque_robot'),
        'robot_param',
        'Leader.yaml'
    ])

    # Include hardware launch file
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': config_file,
            'control_mode': control_mode,
        }.items()
    )

    # Include move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'move_group.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to robot configuration YAML file'
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='position_velocity',
            description='Control mode: position_velocity, pd_control, or full_control'
        ),
        hardware_launch,
        move_group_launch,
    ])
