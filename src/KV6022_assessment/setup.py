from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'KV6022_assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.[yaml|txt|xml]*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yaml|pgm|png]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KV6022 Robotics and Automation',
    maintainer_email='hassankivrak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'example_opencv_detector = KV6022_assessment.example_opencv_detector:main',
            'example_waypoint_follower = KV6022_assessment.example_waypoint_follower:main',
            'example_nav_through_poses = KV6022_assessment.example_nav_through_poses:main',
            'example_nav_to_pose = KV6022_assessment.example_nav_to_pose:main',
        ],
    },
)
