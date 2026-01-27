import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    rviz_config_file = os.path.join(get_package_share_directory('ros2_fra2mo'), 'rviz_conf', 'fra2mo_conf.rviz')
    xacro = os.path.join(get_package_share_directory('ros2_fra2mo'), "urdf", xacro_file_name)
    
    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}
            ]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}
            ]
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

    nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]

    return LaunchDescription(nodes_to_start)
