from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_link_to_livox_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox',
        arguments=[
            '0.0', '0.0', '-0.64',  # translation
            '0.0', '0.0', '0.521', '0.854',  # rotation
            'aft_mapped', 'base_link'
        ]
    )

    aft_mapped_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapped_to_base_link',
        arguments=[
            '0.0', '0.0', '0.0',  # translation
            '0.0', '0.0', '0.0', '1.0',  # rotation
            'aft_mapped', 'livox_frame'
        ]
    )

    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '0.88', '-0.816', '0.0',  # translation
            '0.0', '0.0', '0.0', '1.0',  # rotation
            'map', 'odom'
        ]
    )

    return LaunchDescription([
        base_link_to_livox_node,
        aft_mapped_to_base_link_node,
        # map_to_odom_node,
    ])