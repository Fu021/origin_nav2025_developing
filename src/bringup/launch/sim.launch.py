from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        parameters=[params_file, {'autostart': True}],
    )

    lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('lidar_transform')+"/launch/lidar_transform.launch.py"
        )
    )

    start_nav2_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace='',
        arguments=["-d", os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    seg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
        )
    )

    load_map_server = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map','simulation','rmuc_2025.yaml')}],
                name='map_server',),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'autostart': True,
                             'node_names': ['map_server']}]),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nav')+"/launch/bringup_launch.py"
        )
    )

    p_to_l = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name='pointcloud_to_laserscan_node',
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.1,
                    'min_height': 0.2,
                    'max_height': 1.0,
                    'angle_min': -3.14159,  # -M_PI/2
                    'angle_max': 3.14159,  # M_PI/2
                    'angle_increment': 0.0043,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.4,
                    'range_max': 15.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            )
        ]
    )

    map_to_odom = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='map_to_odom',
                parameters=[{
                    'translation.x':0.0,
                    'translation.y':0.0,
                    'translation.z':0.0,
                    'rotation.x':0.0,
                    'rotation.y':0.0,
                    'rotation.z':0.0,
                    'rotation.w':1.0,
                    'frame_id':'map',
                    'child_frame_id':'odom'
                }]
            ),
        ]
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('point_lio')+"/launch/mapping_velody16.launch.py"
        )
    )

    return LaunchDescription(
        [
            container,
            lidar_transform,
            start_nav2_rviz,
            seg,
            load_map_server,
            nav2,
            p_to_l,
            map_to_odom,
            pointlio
        ]
    )