import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get package paths
    gripper_gazebo_path = FindPackageShare('gripper_gazebo')
    panthera_description_path = FindPackageShare('panthera_ht_description_with_finger')

    # Controller parameters file
    controllers_file = PathJoinSubstitution([
        gripper_gazebo_path,
        'config',
        'ros2_controllers.yaml'
    ])

    # URDF file path
    urdf_file = PathJoinSubstitution([
        gripper_gazebo_path,
        'urdf',
        'Panthera-HT_gazebo.urdf.xacro'
    ])

    # Generate robot_description with controller params
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' ros2_control_params:=', controllers_file
        ]),
        value_type=str
    )

    # Set Gazebo resource path for meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            panthera_description_path, ':',
            PathJoinSubstitution([panthera_description_path, '..']), ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # Set Gazebo system plugin path for ros2_control
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            '/opt/ros/jazzy/lib', ':',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # Clock bridge for time synchronization
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'panthera_with_gripper'
        ],
        output='screen'
    )

    # Load controllers - use --strict to ensure controllers are properly loaded
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gz_resource_path,
        gz_plugin_path,
        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
