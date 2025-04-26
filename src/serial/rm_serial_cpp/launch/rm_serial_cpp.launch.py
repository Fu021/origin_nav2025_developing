from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

    start_rm_serial_cpp = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='rm_serial_cpp',
                plugin='rm_serial_cpp::RmSerialCpp',
                name='rm_serial_cpp_node',
            ),
        ]
    )

    return LaunchDescription([start_rm_serial_cpp])
