from setuptools import setup
import os
from glob import glob

package_name = 'gbp_planning_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.algorithms',
        f'{package_name}.utils',
        f'{package_name}.nodes',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taehyun Jung',
    maintainer_email='jth8090@khu.ac.kr',
    description='Grid-based planning demos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_grid_planner = gbp_planning_demos.nodes.simple_grid_planner:main',
        ],
    },
)
