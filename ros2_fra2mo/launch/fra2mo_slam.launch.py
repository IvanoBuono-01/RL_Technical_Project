import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = os.path.join(get_package_share_directory('ros2_fra2mo'), 'rviz_conf', 'slam_view.rviz')


    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("ros2_fra2mo"), 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
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

    return LaunchDescription([use_sim_time_arg, slam_params_file_arg, slam_node, rviz_node])
