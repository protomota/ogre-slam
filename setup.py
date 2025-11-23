from setuptools import setup
import os
from glob import glob

package_name = 'ogre_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=[
        'setuptools',
        'Jetson.GPIO',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Jetson',
    maintainer_email='your-email@example.com',
    description='SLAM mapping and autonomous navigation for Project Ogre',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = ogre_slam.odometry_node:main',
            'dummy_odom_node = ogre_slam.dummy_odom_node:main',
        ],
    },
)
