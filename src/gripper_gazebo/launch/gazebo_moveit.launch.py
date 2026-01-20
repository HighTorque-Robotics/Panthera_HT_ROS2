from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    gripper_gazebo_path = FindPackageShare('gripper_gazebo')
    with_gripper_config_path = FindPackageShare('with_gripper_config')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gripper_gazebo_path,
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Include static virtual joint TFs (publishes world->base_link transform)
    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'static_virtual_joint_tfs.launch.py'
            ])
        ])
    )

    # Include MoveIt move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include MoveIt RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                with_gripper_config_path,
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        static_tfs_launch,
        move_group_launch,
        moveit_rviz_launch,
    ])
