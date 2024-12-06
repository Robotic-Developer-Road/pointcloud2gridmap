import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration,TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value=TextSubstitution(text="map.pcd")
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value=TextSubstitution(text="0.05")
    )
    minz_arg = DeclareLaunchArgument(
        'min_z',
        default_value=TextSubstitution(text="0.1")
    )
    maxz_arg = DeclareLaunchArgument(
        'max_z',
        default_value=TextSubstitution(text="0.15")
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value=TextSubstitution(text="map")
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value=TextSubstitution(text="map")
    )
    
    return LaunchDescription(
        [
            pcd_file_arg,
            resolution_arg,
            minz_arg,
            maxz_arg,
            frame_id_arg,
            map_topic_arg,
            
		    Node(
				package='pcd2grid',
				executable='pcd2grid_node',
				name='pcd2gridmap_node',
				output='log',
				parameters=[{'pcd_file': LaunchConfiguration('pcd_file'),
							 'resolution': LaunchConfiguration('resolution'),
                             'min_z':LaunchConfiguration('min_z'),
                             'max_z': LaunchConfiguration('max_z'),
                             'frame_id': LaunchConfiguration('frame_id'),
                             'map_topic': LaunchConfiguration('map_topic')
				}]
			)
        ]
    )