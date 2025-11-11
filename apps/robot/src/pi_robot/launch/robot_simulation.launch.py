import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pi_robot_pkg = get_package_share_directory('pi_robot')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # --- Gazebo Simulation ---

    # World file
    world = os.path.join(pi_robot_pkg, 'worlds', 'empty.world')

    # Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    # --- Robot Model ---

    # URDF file
    urdf_file_name = 'pi_robot.urdf.xacro'
    urdf_path = os.path.join(pi_robot_pkg, 'urdf', urdf_file_name)

    # Process the URDF file
    robot_description_raw = Command(['xacro ', urdf_path])

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Spawn Robot Entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'pi_robot'],
        output='screen'
    )

    # --- ROS Bridge and Video Server ---

    # ROS Bridge Server
    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_server_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # Web Video Server
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server'
    )

    # Telemetry Node (from the original application)
    telemetry_node = Node(
        package='pi_robot',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        rosbridge_server_launch,
        web_video_server_node,
        telemetry_node,
    ])
