import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mro_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Byeongjik',
    maintainer_email='jungji0913@khu.ac.kr',
    description='AMR 주행: 셔틀 미션(도킹→베이→Z-05), 작업자 추종·재탐지·Standby, 온보드 안전 감시, 도킹 복귀',
    license='MIT',
    entry_points={
        'console_scripts': [
            'shuttle_mission_node = mro_navigation.nodes.shuttle_mission_node:main',
        'follow_target_node = mro_navigation.nodes.follow_target_node:main',
        'safety_watchdog_node = mro_navigation.nodes.safety_watchdog_node:main',
        'return_dock_node = mro_navigation.nodes.return_dock_node:main',
        ],
    },
)
