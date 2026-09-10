import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mro_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt') + glob('models/*.json')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Byeongjik',
    maintainer_email='jungji0913@khu.ac.kr',
    description='AMR 온보드 인지: OAK-D 작업자 검출·거리 보정·재식별 게이트, ArUco 베이 마커 검출',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amr_detection_node = mro_perception.nodes.amr_detection_node:main',
        'aruco_detector_node = mro_perception.nodes.aruco_detector_node:main',
        'depth_viewer = mro_perception.core.depth_distance:main',
        ],
    },
)
