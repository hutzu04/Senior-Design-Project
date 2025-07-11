from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('aims_controller'),
        'config',
        'background_cam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='background_cam',  # Node name for the USB camera
            output='screen',
            parameters=[config_file],  # Load parameters from the YAML file
            remappings=[
                ('image_raw', '/aims/background_image_raw')  # Remap the raw image topic
            ]
        )
    ])