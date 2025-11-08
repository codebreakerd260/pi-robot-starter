import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pi_robot_pkg = get_package_share_directory('pi_robot')
    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')

    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.033',
        description='Radius of the wheels in meters.'
    )
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base', default_value='0.16',
        description='Distance between the two wheels in meters.'
    )
    # ... (add other motor pin args if needed, but defaults are fine for now)

    camera_frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='20.0',
        description='Camera frame rate.'
    )
    camera_width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Camera frame width.'
    )
    camera_height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Camera frame height.'
    )

    # Nodes
    diff_drive_node = Node(
        package='pi_robot',
        executable='diff_drive_node',
        name='diff_drive_node',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            # Using defaults from the node for GPIO pins
        }]
    )

    camera_node = Node(
        package='pi_robot',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'frame_rate': LaunchConfiguration('frame_rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
        }]
    )

    telemetry_node = Node(
        package='pi_robot',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    # ROS Bridge Server
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # Web Video Server
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server'
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_base_arg,
        camera_frame_rate_arg,
        camera_width_arg,
        camera_height_arg,
        diff_drive_node,
        camera_node,
        telemetry_node,
        rosbridge_server,
        web_video_server,
    ])
