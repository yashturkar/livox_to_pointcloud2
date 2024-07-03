
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_to_pointcloud2',
            executable='livox_to_pointcloud2_node',
            name='livox_to_pointcloud2_node',
            remappings=[
                ('/livox_pointcloud', '/livox/lidar'),
                ('/converted_pointcloud2', '/livox/lidar/pcd2')
            ],
            output='screen'
        ),
        Node(
            package='livox_to_pointcloud2',
            executable='static_transform_publisher.py',  # This should match the script name
            name='static_transform_publisher',
            output='screen'
        )
    ])