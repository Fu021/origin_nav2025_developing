from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

    start_lidar_transform = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_transform',
                plugin='lidar_transform::LidarTransform',
                name='lidar_transform_node',
                parameters=[{
                    "lidar_frame": 'livox_frame',
                    "base_link_frame": 'base_link'
                }],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(start_lidar_transform)

    return ld