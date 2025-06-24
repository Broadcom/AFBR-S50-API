from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_01 = Node(
        package="raw_tof",
        executable="raw_tof"
    )
    control_01 = Node(
        package="pointcloud",
        executable="pointcloud"
    )
    coor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'tof_sensor', 'map']
        )
    launch_description = LaunchDescription(
        [robot_01, control_01, coor])
    return launch_description
