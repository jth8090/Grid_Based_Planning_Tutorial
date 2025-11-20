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
        # ament index 등록
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='you@example.com',
    description='Grid-based planning demos (DFS, BFS, Dijkstra, A*) on ROS2 OccupancyGrid.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run gbp_planning_demos simple_grid_planner
            'simple_grid_planner = gbp_planning_demos.nodes.simple_grid_planner:main',
        ],
    },
)
