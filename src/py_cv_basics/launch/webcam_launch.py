from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC = 'webcam_topic'


def generate_launch_description():
    talker = Node(
        package='py_cv_basics',
        executable='img_publisher',
        name='img_publisher',
        parameters=[{
            'topic': TOPIC
        }]
    )

    listener = Node(
        package='py_cv_basics',
        executable='img_subscriber',
        name='img_subscriber',
        parameters=[{
            'topic': TOPIC
        }]
    )

    return LaunchDescription([
        talker,
        listener,
    ])
