from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    client = Node(
        package='py_srvcli',
        executable='client',
        name='client',
        parameters=[{
            'a_launch': 2,
            'b_launch': 3
        }]
    )

    return LaunchDescription([
        client,
    ])
