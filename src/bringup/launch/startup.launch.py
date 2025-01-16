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

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
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
            get_package_share_directory('point_lio')+"/launch/mapping_mid360.launch.py"
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
                parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'blank.yaml')}],
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

    return LaunchDescription(
        [
            #pointlio,
            container,
            tf,
            lidar_transform,
            livox_ros_driver2,
            #point_lio_rviz,
            start_nav2_rviz,
            seg,
            load_map_server,
            #nav2,
        ]
    )