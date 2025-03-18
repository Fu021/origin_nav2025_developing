from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pcd_file = os.path.join(get_package_share_directory('point_lio'),'PCD','601.pcd')

    start_localization = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='small_gicp_localization',
                plugin='small_gicp_localization::SmallGicpLocalization',
                name='small_gicp_localization_node',
                parameters=[{
                    "num_threads": 4,
                    "num_neighbors": 10,
                    "global_leaf_size": 0.25,
                    "registered_leaf_size": 0.25,
                    "max_dist_sq": 1.0,
                    "map_frame": 'map',
                    "odom_frame": 'odom',
                    "base_link_frame": 'base_link',
                    "pcd_file": pcd_file,
                    "cloud_registered_topic": 'cloud_registered',
                }],
            ),
        ]
    )
    # start_localization = Node(
    #     package='small_gicp_localization',
    #     executable='small_gicp_localization_node',
    #     name='small_gicp_localization_node',
    #     output='both',
    #     parameters=[{
    #         'use_sim_time': True,
    #         "num_threads": 4,
    #         "num_neighbors": 10,
    #         "global_leaf_size": 0.25,
    #         "registered_leaf_size": 0.25,
    #         "max_dist_sq": 1.0,
    #         "map_frame": 'map',
    #         "odom_frame": 'odom',
    #         "base_link_frame": 'base_link',
    #         "pcd_file": pcd_file,
    #         "cloud_registered_topic": '/base_link/lidar/pointcloud',
    #     }],
    # )

    ld = LaunchDescription()

    ld.add_action(start_localization)

    return ld