from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wheel_imu'),
        'config',
        'wheel_imu_config.yaml',
    )

    return LaunchDescription([
         Node(
            package='wheel_imu',
            executable='wheel_imu_node_publisher',
            name='wheel_imu_pub',
            parameters=[config],
            output='screen',
            ),
        ])
