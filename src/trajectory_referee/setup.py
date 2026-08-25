from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trajectory_referee'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'routes'), glob(os.path.join('routes', '*.pkl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnwm5',
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
            'trajectory_publisher = trajectory_referee.trajectory_publisher:main',
        ],
    },
)
