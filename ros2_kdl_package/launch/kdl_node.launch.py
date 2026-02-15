from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
 
 
def generate_launch_description():

    # 2. Configurazione del nodo rqt_plot
    # Il nodo rqt_plot è contenuto nel pacchetto 'rqt_plot'
    rqt_plot_node_position = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='position_plotter',
        output='screen',
        arguments=[
            '/joint_states/position[0]',
            '/joint_states/position[1]',
            '/joint_states/position[2]',
            '/joint_states/position[3]',
            '/joint_states/position[4]',
            '/joint_states/position[5]',
            '/joint_states/position[6]',
            # Aggiungi qui gli altri giunti/campi che vuoi plottare
        ],
        
    )

    # 2. Configurazione del nodo rqt_plot
    # Il nodo rqt_plot è contenuto nel pacchetto 'rqt_plot'
    rqt_plot_node_velocity = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='velocity_plotter',
        output='screen',
        arguments=[
            '/velocity_controller/commands/data[0]',
            '/velocity_controller/commands/data[1]',
            '/velocity_controller/commands/data[2]',
            '/velocity_controller/commands/data[3]',
            '/velocity_controller/commands/data[4]',
            '/velocity_controller/commands/data[5]',
            '/velocity_controller/commands/data[6]',
        ],
        
    )

    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    kdl_params_file = os.path.join(
        get_package_share_directory('ros2_kdl_package'),  
        'config',
        'kdl_params.yaml'
    )

    # THIS IS FOR THE ACTION CLIENT
    trajectory_action_client = Node(
    package='ros2_kdl_package',
    executable='trajectory_action_client',
    name='trajectory_action_client',
    parameters=[kdl_params_file],
    output='screen'
    )

 
    
    command_interface_arg = DeclareLaunchArgument(
        name='cmd_interface',
        default_value='position',
        description='Choose which interface to use: position, velocity, or effort'
    )
 
    
    cmd_interface = LaunchConfiguration('cmd_interface')   

    ctrl_arg = DeclareLaunchArgument(
        name='ctrl',
        default_value='velocity_ctrl',
        description='Choose which velocity cotrollers use'
    )
 
    
    ctrl = LaunchConfiguration('ctrl')  


 
    ros2_kdl_node = Node(
        package='ros2_kdl_package',             
        executable='ros2_kdl_node',         
        name='ros2_kdl_node',
        output='screen',
        parameters=[{'cmd_interface': cmd_interface},
                    {'ctrl' : ctrl}
                ],
    )

    delay_client = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=ros2_kdl_node,
        on_start=[
            TimerAction(
                period=3.0,
                actions=[trajectory_action_client]
            )
        ]
    )
    )

    # Appena parte rqt_plot_velocity attendi 5 secondi e poi avvia ros2_kdl_node
    delay_ros2_kdl_node= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rqt_plot_node_velocity,
            on_start=[
                TimerAction(
                    period=5.0,  # attesa in secondi prima di lanciare ros2_kdl_node
                    actions=[ros2_kdl_node]
                )
            ]
        )
    ) 
    
    # Bridge per set_pose with delay
    pose_bridge = TimerAction(
    period=5.0,  # delay in secondi
    actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_bridge',
            arguments=[
                '/world/iiwa_personal_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
            ],
            output='screen',
        )
    ]
    )

    return LaunchDescription([
        gui_arg,
        trajectory_action_client,
        command_interface_arg,
        ctrl_arg,
        rqt_plot_node_position,
        rqt_plot_node_velocity,
        delay_ros2_kdl_node,
        pose_bridge,
    ])