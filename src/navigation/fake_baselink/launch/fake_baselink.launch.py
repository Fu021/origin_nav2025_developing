from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    fake_vel_transform_node = Node(
        package='fake_baselink',
        executable='fake_baselink_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame_id':'odom',
            'forward_distance':0.2
        }]
    )


    # container = Node(
    #     name='fake_container',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     output='screen'
    # )

    # start_fake_baselink = LoadComposableNodes(
    #     target_container='nav2_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='fake_baselink',
    #             plugin='FakeBaselink',
    #             name='fake_baselink_node',
    #         ),
    #     ]
    # )

    return LaunchDescription([fake_vel_transform_node])
