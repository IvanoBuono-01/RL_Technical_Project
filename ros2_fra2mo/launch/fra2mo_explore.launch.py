import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch.actions import GroupAction

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('ros2_fra2mo'), 'rviz_conf', 'explore.rviz')
    
    fra2mo_dir = FindPackageShare('ros2_fra2mo')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_bringup_launch = GroupAction(
    actions=[

        SetRemap(src='/cmd_vel',dst='/fra2mo/cmd_vel'),

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            )
    ]
)




    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            slam_launch,
            nav2_bringup_launch,
            explore_lite_launch,
            rviz_node,
        ]
    )