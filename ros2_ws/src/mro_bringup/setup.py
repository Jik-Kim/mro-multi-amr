import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mro_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Byeongjik',
    maintainer_email='jungji0913@khu.ac.kr',
    description='통합 실행: PC 역할별 launch, 로봇별 파라미터(네임스페이스), 아레나 구역 정의, 지도',
    license='MIT',
    entry_points={
        'console_scripts': [

        ],
    },
)
