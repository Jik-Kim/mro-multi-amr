import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mro_webcam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (
            os.path.join('share', package_name, 'config', 'WebCam2'),
            glob('config/WebCam2/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Byeongjik',
    maintainer_email='jungji0913@khu.ac.kr',
    description='고정 웹캠 계층: 웹캠1 작업자 등록(서술자), 웹캠2 전역 위치 힌트, 바닥 호모그래피, KPI 로거. 안전 판단에 관여하지 않음',
    license='MIT',
    entry_points={
        'console_scripts': [
            'webcam_register_node = mro_webcam.nodes.webcam_register_node:main',
            'webcam_hint_node = mro_webcam.nodes.webcam_hint_node:main',
            'webcam2 = mro_webcam.nodes.webcam2:main',
            'webcam2_subscriber = mro_webcam.nodes.webcam2_subscriber:main',
            'kpi_logger_node = mro_webcam.nodes.kpi_logger_node:main',
            'calibrate_homography = mro_webcam.core.homography:main',
            'calibrate_cam2_markers = mro_webcam.nodes.calibrate_cam2_markers:main',
        ],
    },
)
