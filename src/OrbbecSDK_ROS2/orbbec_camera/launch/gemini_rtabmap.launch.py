from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters = {
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': False,
        'use_sim_time': False,
        'queue_size': 30,
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false'
    }

    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    return LaunchDescription([
        # Set database to empty for fresh mapping each time
        SetEnvironmentVariable('RTABMAP_DB_PATH', ''),
        
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch RTAB-Map visualization'
        ),
        
        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['--delete_db_on_start']
        ),
        
        # RTAB-Map visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            condition=LaunchConfiguration('rtabmap_viz')
        ),
    ])
