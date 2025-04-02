from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
def generate_launch_description():

    # start_lidar_transform = LoadComposableNodes(
    #     target_container='container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='lidar_transform',
    #             plugin='lidar_transform::LidarTransform',
    #             name='lidar_transform_node',
    #             parameters=[{
    #                 "lidar_frame": 'livox_frame',
    #                 "base_link_frame": 'base_link'
    #             }],
    #         ),
    #     ]
    # )

    # start_lidar_transform = Node(
    #     package='lidar_transform',
    #     executable='lidar_transform_node',
    #     name='lidar_transform_node',
    #     output='screen',
    #     parameters=[{
    #         "lidar_frame": 'livox_frame',
    #         "base_link_frame": 'base_link'
    #     }],        # prefix='gdb -ex run --args'
    # )

    ladar_container = ComposableNodeContainer(
        name="lidar_transform_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
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
        ],
        output="screen"
    )

    ld = LaunchDescription([ladar_container])

    # ld.add_action(start_lidar_transform)

    return ld