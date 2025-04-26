from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rm_serial = Node(
        package="rm_serial",
        executable="rm_serial_subpub",
        name="rm_serial",
        output='screen',
    )
    send_cmd = Node(
        package="rm_serial",
        executable="send_cmd_vel",
        name="send_cmd"
    )
    
    return LaunchDescription(
        [
            rm_serial,
            send_cmd
        ]
    )