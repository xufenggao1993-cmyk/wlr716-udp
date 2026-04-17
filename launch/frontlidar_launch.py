from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip', default_value='192.168.0.2', description='Lidar IP address')

    lidar_ip = LaunchConfiguration('lidar_ip')

    node = Node(
        package='wlr716_udp',
        executable='wlr716_udp',
        name='frontlidar_ctrl_node',
        output='screen',
        parameters=[{'LidarIPAddress': "192.168.0.2"}]
    )

    return LaunchDescription([
        # lidar_ip_arg,
        node,
    ])
