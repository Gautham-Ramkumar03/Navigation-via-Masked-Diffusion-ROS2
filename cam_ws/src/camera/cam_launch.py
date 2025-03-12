import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cam_name = "cam0"
    config = os.path.join(
        get_package_share_directory('camera'),
        'camera',
        'config',
        f'{cam_name}.yaml'
    )
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=cam_name,
        output='screen',
        parameters=[config],
        remappings = [
            ('image_raw', f'{cam_name}/image_raw'),
            ('image_raw/compressed', f'{cam_name}/image_compressed'),
            ('image_raw/compressedDepth', f'{cam_name}/compressedDepth'),
            ('image_raw/theora', f'{cam_name}/image_raw/theora'),
            ('camera_info', f'{cam_name}/camera_info')
        ]
    )

    nodes = []
    nodes.append(usb_cam)

    return LaunchDescription(nodes)