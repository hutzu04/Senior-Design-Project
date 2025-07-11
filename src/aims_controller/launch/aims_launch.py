from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start the controller_node normally
        Node(
            package='aims_controller',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        # Start the cv_node directly in the virtual environment
        ExecuteProcess(
            cmd=[
                '/home/aims_user/aims_env/bin/python3',  # Use the Python interpreter from the virtual environment
                '/home/aims_user/aims_ws/src/aims_controller/aims_controller/cv_node.py'
            ],
            output='screen'
        ),
        # Start the joystick_node directly in the virtual environment
        ExecuteProcess(
            cmd=[
                '/home/aims_user/aims_env/bin/python3',  # Use the Python interpreter from the virtual environment
                '/home/aims_user/aims_ws/src/aims_controller/aims_controller/joystick_node.py'
            ],
            output='screen'
        ),
    ])