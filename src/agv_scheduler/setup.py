import glob
import os

from setuptools import setup

package_name = 'agv_scheduler'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob.glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py'),
        ),
        (   os.path.join('share', package_name, 'rviz'),
            glob.glob('rviz/*.rviz')
        ),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'scheduler_node = agv_scheduler.scheduler_node:main',
    ]},
)
