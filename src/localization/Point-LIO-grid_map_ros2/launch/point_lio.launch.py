import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("point_lio_with_grid_map")

    blind = LaunchConfiguration('preprocess.blind')
    point_filter_num = LaunchConfiguration('point_filter_num')
    space_down_sample = LaunchConfiguration('space_down_sample')
    filter_size_surf = LaunchConfiguration('filter_size_surf')
    filter_size_map = LaunchConfiguration('filter_size_map')
    save_pcd= LaunchConfiguration('pcd_save.pcd_save_en')
    # Declare the argument
    declare_blind = DeclareLaunchArgument(
        'preprocess.blind',
        default_value='0.1',
      )
    declare_point_filter = DeclareLaunchArgument(
        'point_filter_num',
        default_value='1',
        )

    declare_downsample = DeclareLaunchArgument(
        'space_down_sample',
        default_value= 'True'       
        )

    declare_surf = DeclareLaunchArgument(
        'filter_size_surf', 
        default_value='0.3',
        )

    declare_map = DeclareLaunchArgument(
        'filter_size_map', 
        default_value='0.3',
        )
    declare_save_pcd = DeclareLaunchArgument(
        'pcd_save.pcd_save_en', 
        default_value='False',
        )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_node = Node(
        package="point_lio_with_grid_map",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[
            point_lio_cfg_dir,
            {
                'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                # 'prop_at_freq_of_imu': True,
                # 'check_satu': True,
                # 'init_map_size': 10,
                'point_filter_num': point_filter_num,  # Options: 1, 3
                'space_down_sample': space_down_sample,
                'filter_size_surf': filter_size_surf,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                'filter_size_map': filter_size_map,  # Options: 0.5, 0.3, 0.15, 0.1
                # 'cube_side_length': 1000.0,  # Option: 1000
                # 'runtime_pos_log_enable': False,  # Option: True
                'pcd_save.pcd_save_en': save_pcd,
                'preprocess.blind': blind
            }, 
        ],
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution(['/home/sb/origin_nav2025_test/src/bringup', "rviz", "loam_livox"]),
            ".rviz"
        ],
    )

    ld = LaunchDescription(
        [
            declare_namespace,
            declare_blind,
            declare_point_filter,
            declare_downsample,
            declare_surf,
            declare_map,
            declare_save_pcd,
            declare_rviz,
            declare_point_lio_cfg_dir,
            start_point_lio_node,
            # start_rviz_node
        ]
    )

    # ld.add_action()
    # ld.add_action(declare_rviz)
    # ld.add_action(declare_point_lio_cfg_dir)
    # ld.add_action(start_point_lio_node)
    # ld.add_action(start_rviz_node)

    return ld