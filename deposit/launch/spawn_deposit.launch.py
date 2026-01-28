import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    pkg_deposit = get_package_share_directory('deposit')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_iiwa = get_package_share_directory('iiwa_description')

    gz_model_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',
    value=os.path.join(pkg_deposit, 'models')  # cartella dove ci sono i modelli SDF/DAE/STL
)


    world_file_path = os.path.join(pkg_deposit, 'worlds', 'personal_project_world.sdf')
    xacro_file_fra2mo = os.path.join(pkg_fra2mo, 'urdf', 'fra2mo.urdf.xacro') 
    xacro_file_iiwa = os.path.join(pkg_iiwa, 'config', 'iiwa.config.xacro')
    controllers_file_path = os.path.join(pkg_iiwa, 'config', 'iiwa_controllers.yaml')

    #Descrizione fra2mo
    robot_description_fra2mo = ParameterValue(Command(['xacro ', xacro_file_fra2mo]), value_type=str)

    #Descrizione iiwa
    robot_description_iiwa = ParameterValue(
        Command([
            'xacro ', xacro_file_iiwa,
            ' use_sim:=true',
            ' use_fake_hardware:=false',
            ' namespace:=iiwa'
        ]),
        value_type=str
    )

    #robot state publisher fra2mo
    rsp_fra2mo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_fra2mo',
        namespace='fra2mo',
        parameters=[{
            'robot_description': robot_description_fra2mo,
            'use_sim_time': True
        }]
    )

    #robot state publisher iiwa
    rsp_iiwa = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='iiwa',
        parameters=[{
            'robot_description': robot_description_iiwa,
            'use_sim_time': True
        }]
    )

    #joint state publisher fra2mo
    jsp_fra2mo = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_fra2mo',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file_path, '-r'],
        output='screen'
    )

    position_fra2mo = [-7.9, 3.10, 0.1]

    spawn_fra2mo = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_fra2mo',
                arguments=['-topic', '/fra2mo/robot_description',
                           '-name', 'fra2mo',
                           '-x', str(position_fra2mo[0]),
                           '-y', str(position_fra2mo[1]),
                           '-z', str(position_fra2mo[2])],
                output='screen'
            )
        ]
    )

    position_iiwa = [8.40, -4, 0.1]

    spawn_iiwa = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_iiwa',
                arguments=['-topic', '/iiwa/robot_description',
                           '-name', 'iiwa',
                           '-x', str(position_iiwa[0]),
                           '-y', str(position_iiwa[1]),
                           '-z', str(position_iiwa[2])],
                output='screen'
            )
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}],
    )

    #CONTROLLER SPAWNERS FOR IIWA
    jsb_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/iiwa/controller_manager'],
                output='screen'
            )
        ]
    )

    iiwa_arm_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'iiwa_arm_controller',
                    '--controller-manager', '/iiwa/controller_manager',
                    '--param-file', controllers_file_path
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ign_plugin_path,
        gz_model_path,
        rsp_fra2mo,
        rsp_iiwa,
        jsp_fra2mo,
        gz_sim,
        spawn_fra2mo,
        spawn_iiwa,
        bridge,
        odom_tf,
        jsb_spawner,
        iiwa_arm_controller_spawner
    ])