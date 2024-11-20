from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_truth',
            executable='centering',
            name='centering'
        ),
        Node(
            package='ground_truth',
            executable='check_angle',
            name='check_angle'
        ),
        Node(
            package='ground_truth',
            executable='touch_tree',
            name='touch_tree',
        ),
         Node(
            package='ground_truth',
            executable='touch',
            name='touch',
        ),
        Node(
            package='ground_truth',
            executable='calc_diameter_service',
            name='calc_diameter_service',
        ),
        Node(
            package='ground_truth',
            executable='pixel_dimeter',
            name='pixel_dimeter',
        ),
        Node(
            package='ground_truth',
            executable='record_video_service',
            name='record_video_service',
        ), 
         Node(
            package='ground_truth',
            executable='pub_camera',
            name='pub_camera',
        )
    ])