from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC = 'webcam_topic'


def generate_launch_description():
    talker = Node(
        package='cpp_cv_basics',
        executable='webcam_pub',
        name='webcam_pub',
        parameters=[{
            'topic': TOPIC
        }]
    )

    listener = Node(
        package='cpp_cv_basics',
        executable='webcam_sub',
        name='webcam_sub',
        parameters=[{
            'topic': TOPIC
        }]
    )

    return LaunchDescription([
        talker,
        listener,
    ])
