from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    blind = LaunchConfiguration('preprocess.blind')
    point_filter_num = LaunchConfiguration('point_filter_num')
    space_down_sample = LaunchConfiguration('space_down_sample')
    filter_size_surf = LaunchConfiguration('filter_size_surf')
    filter_size_map = LaunchConfiguration('filter_size_map')
    save_pcd= LaunchConfiguration('pcd_save.pcd_save_en')
    # Declare the argument
    declare_blind = DeclareLaunchArgument(
        'preprocess.blind',
        default_value='0.6',
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
        default_value='0.04',
        )

    declare_map = DeclareLaunchArgument(
        'filter_size_map', 
        default_value='0.04',
        )
    declare_save_pcd = DeclareLaunchArgument(
        'pcd_save.pcd_save_en', 
        default_value='False',
        )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='False',
        description='Flag to launch RViz.')

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        #prefix=['valgrind', ' --leak-check=full', ' --show-leak-kinds=all', ' --track-origins=yes', ' --verbose', ' --log-file=valgrind-pointlio.out'],
        #prefix=['gdb', ' -ex', ' run', ' --args'],
        parameters=[
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 1000,
            'point_filter_num': point_filter_num,  # Options: 1, 3
            'space_down_sample': space_down_sample,
            'filter_size_surf': filter_size_surf,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': filter_size_map,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
            'pcd_save.pcd_save_en': save_pcd,
            'preprocess.blind': blind
        },
    ]
,
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        declare_blind,
        declare_point_filter,
        declare_downsample,
        declare_surf,
        declare_map,
        declare_save_pcd,
        rviz_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
