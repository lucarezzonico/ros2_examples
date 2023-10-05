from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC = 'chatter_py'


def generate_launch_description():
    talker = Node(
        package='py_pubsub',
        executable='talker',
        name='talker',
        parameters=[{
            'topic': TOPIC
        }]
    )

    listener = Node(
        package='py_pubsub',
        executable='listener',
        name='listener',
        parameters=[{
            'topic': TOPIC
        }]
    )

    return LaunchDescription([
        talker,
        listener,
    ])
