from launch import LaunchDescription
from launch_ros.actions import Node


package_name = 'dummy_drone_chaser'


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package=package_name,
            name='drone',
            executable='dummy_drone_node'
        ),
        Node(
            package=package_name,
            name='drone_chaser',
            executable='chaser_drone_node'
        ),
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])