from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='lucarezzonico97@gmail.com',
    description=(
        'learning_tf2_py demonstrates how to write a ROS2 Python tf2 broadcaster and '
        'listener with the turtlesim. The turtle_tf2_listener commands turtle2 to '
        'follow turtle1 around as you drive turtle1 using the keyboard.'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',
            'fixed_frame_tf2_broadcaster = learning_tf2_py.fixed_frame_tf2_broadcaster:main',
            'dynamic_frame_tf2_broadcaster = learning_tf2_py.dynamic_frame_tf2_broadcaster:main',
            'turtle_tf2_message_broadcaster = learning_tf2_py.turtle_tf2_message_broadcaster:main',
        ],
    },
)
