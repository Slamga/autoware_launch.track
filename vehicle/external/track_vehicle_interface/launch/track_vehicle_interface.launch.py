from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory('vehicle_contrl'),
    #     'config',
    #     'vehicle_contrl.yaml',
    # ),
    return LaunchDescription([
         Node(
            package='track_vehicle_interface',
            executable='track_vehicle_interface_node',
            name='track_vehicle_interface',
            # parameters=[config],
            output='screen',
            ),
        ])
