from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    rviz_config_file = get_package_share_directory('bringup')+'/rviz/mapping.rviz'
    map_dir = get_package_share_directory('bringup')+'/map/'
    params_file = get_package_share_directory('nav')+'/params/mapper_params_online_async.yaml'

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        #parameters=[params_file, {'autostart': True}],
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('nav')+'/launch/online_async_launch.py'),
        launch_arguments={'map':map_dir+'rmuc'}.items()
    )

    tf_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('bringup')+"/launch/tf.launch.py"
        )
    )

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
        )
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('point_lio_with_grid_map')+"/launch/point_lio.launch.py"
        ),
        launch_arguments={
                        'preprocess.blind':'0.1',
                          'point_filter_num': '1',  
                        #   'space_down_sample': 'True',
                          'filter_size_surf': '0.3',  
                          'filter_size_map': '0.3',
                          'pcd_save.pcd_save_en': 'True', 
                          }.items()
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
    segment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
        )
    )

    lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('lidar_transform')+"/launch/lidar_transform.launch.py"
        )
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([
        container,
        # lidar_transform,
        tf_bringup,
        livox,
        pointlio,
        p_to_l,
        segment,
        # slam_toolbox,
        start_rviz_cmd,
    ])