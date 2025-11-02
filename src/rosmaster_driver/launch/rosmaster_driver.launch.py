from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='rosmaster_driver',
            executable='rosmaster_driver_node',
            name='rosmaster_driver',
            output='screen',
            parameters=[{
                'imu_link': 'imu_link',
                'publish_rate': 10.0,
                'prefix': '',
                'port': '/dev/my_ros_board',
                'debug': False,
            }],
        )
    ])
