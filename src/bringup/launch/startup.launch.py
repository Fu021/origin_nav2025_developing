from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction
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
    
    tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('bringup')+"/launch/tf.launch.py"
        )
    )

    livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
        )
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('point_lio_with_grid_map')+"/launch/point_lio.launch.py"
        )
    )

    lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('lidar_transform')+"/launch/lidar_transform.launch.py"
        )
    )

    point_lio_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace='',
        arguments=["-d", os.path.join(bringup_dir, 'rviz', 'loam_livox.rviz')],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
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
    dec = Node(
            package="dec_tree",
            executable="root",
            namespace='',
            output="screen",
        )
    dec_simple = Node(
            package="dec_tree",
            executable="root_simple",
            namespace='',
            output="screen",
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
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'map_newnew.yaml')}],
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', '6floor_mid.yaml')}],
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'blank.yaml')}],
                parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'rmuc_2025_normalized.yaml')}],
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

    fake_baselink = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('fake_baselink')+"/launch/fake_baselink.launch.py"
        )
    )
    p_to_l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        namespace='',
        output="screen",
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

    dp_a = Node(
        package="dip_angle",
        executable="dip_angle",
        namespace='',
        output="screen",
    )

    # p_to_l = LoadComposableNodes(
    #     target_container='container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='pointcloud_to_laserscan',
    #             plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
    #             name='pointcloud_to_laserscan_node',
    #             parameters=[{
    #                 'target_frame': 'base_link',
    #                 'transform_tolerance': 0.1,
    #                 'min_height': 0.2,
    #                 'max_height': 1.0,
    #                 'angle_min': -3.14159,  # -M_PI/2
    #                 'angle_max': 3.14159,  # M_PI/2
    #                 'angle_increment': 0.0043,  # M_PI/360.0
    #                 'scan_time': 0.3333,
    #                 'range_min': 0.4,
    #                 'range_max': 15.0,
    #                 'use_inf': True,
    #                 'inf_epsilon': 1.0
    #             }],
    #             remappings=[('cloud_in',  ['/segmentation/obstacle']),
    #                     ('scan',  ['/scan'])],
    #         )
    #     ]
    # )
    gicp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('small_gicp_localization')+"/launch/small_gicp_localization.launch.py"
        )
    )

    rm_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_serial')+"/launch/rm_serial.launch.py"
        )
    )

    icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            
            get_package_share_directory('icp_registration')+'/launch/icp.launch.py'
        )
    )

    return LaunchDescription(
        [
             rm_serial,
            #  pointlio,
             container,
             tf,
            livox_ros_driver2,
             start_nav2_rviz,
             seg,
             load_map_server,
             p_to_l,
             fake_baselink,
             dp_a,
            # TimerAction(period=4.0, actions=[icp]),
            TimerAction(period=4.0, actions=[nav2]),
            TimerAction(period=8.0, actions=[dec]),
            # TimerAction(period=8.0, actions=[dec_simple]),


            
        ]
    )