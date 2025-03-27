from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tb3_autonomy'),
        'config',
        'locations.yaml'
    )

    return LaunchDescription([
            Node(
                package='tb3_autonomy',
                executable='autonomy_node',
                name='autonomy_node',
                parameters=[{'location_file': config}]
            ),
        ])
