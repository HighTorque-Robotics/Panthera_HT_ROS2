import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    config_file = LaunchConfiguration('config_file')
    control_mode = LaunchConfiguration('control_mode')

    # Get package paths
    with_gripper_config_path = FindPackageShare('with_gripper_config')
    panthera_description_path = FindPackageShare('panthera_ht_description_with_finger')

    # Default robot config file (Leader configuration)
    default_config_file = PathJoinSubstitution([
        FindPackageShare('hightorque_robot'),
        'robot_param',
        'Leader.yaml'
    ])

    # Controller parameters file for real hardware
    controllers_file = PathJoinSubstitution([
        with_gripper_config_path,
        'config',
        'ros2_controllers_hardware.yaml'
    ])

    # URDF file path - need to create a hardware version
    urdf_file = PathJoinSubstitution([
        with_gripper_config_path,
        'config',
        'Panthera-HT_description_hardware.urdf.xacro'
    ])

    # Generate robot_description with hardware interface
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' config_file:=', config_file,
            ' control_mode:=', control_mode
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_file
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Arm Controller Spawner (delayed start after joint_state_broadcaster)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Gripper Controller Spawner (delayed start after arm_controller)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay arm controller start until joint_state_broadcaster is loaded
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Delay gripper controller start until arm_controller is loaded
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
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
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
    ])
