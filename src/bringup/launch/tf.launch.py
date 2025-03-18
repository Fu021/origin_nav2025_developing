from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

    load_all_tf = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='base_link_to_livox',
                parameters=[{
                    'translation.x':0.0,
                    'translation.y':-0.08,
                    'translation.z':0.6,
                    'rotation.x':0.707,
                    'rotation.y':-0.707,
                    'rotation.z':0.0,
                    'rotation.w':0.0,
                    'frame_id':'base_link',
                    'child_frame_id':'livox_frame'
                }]
            ),
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='aft_mapped_to_base_link',
                parameters=[{
                    'translation.x':0.0,
                    'translation.y':0.0,
                    'translation.z':0.0,
                    'rotation.x':0.0,
                    'rotation.y':0.0,
                    'rotation.z':0.0,
                    'rotation.w':1.0,
                    'frame_id':'aft_mapped',
                    'child_frame_id':'base_link'
                }]
            ),
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
    
    return LaunchDescription([load_all_tf])