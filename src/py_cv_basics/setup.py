from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_cv_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        # (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='lucarezzonico97@gmail.com',
    description='A minimal image publisher and subscriber node that uses OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = py_cv_basics.webcam_pub:main',
            'img_subscriber = py_cv_basics.webcam_sub:main',
        ],
    },
)
