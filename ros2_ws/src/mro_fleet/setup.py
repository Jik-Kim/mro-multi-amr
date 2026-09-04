import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mro_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'templates'), glob('templates/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Byeongjik',
    maintainer_email='jungji0913@khu.ac.kr',
    description='관제: 미션 생성·배정, 로봇 상태 집계, 구역 세마포어(교착 방지), 운영자 명령, DB, 관제 UI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fleet_manager_node = mro_fleet.nodes.fleet_manager_node:main',
        'zone_semaphore_node = mro_fleet.nodes.zone_semaphore_node:main',
        'control_ui_node = mro_fleet.nodes.control_ui_node:main',
        ],
    },
)
