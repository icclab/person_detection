from setuptools import find_packages, setup
import os
from glob import glob

import sys

# sys.executable = "/home/user/myenv/bin/python3"

package_name = 'person_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/new/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.cfg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.weights')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.names')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='gkapoordelhi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detect = person_detect.person_detect:main',
            'person_detect_driver = person_detect.person_detect_driver:main',
            'person_detect_driver_orin = person_detect.person_detect_driver_orin:main',
            'person_detect_driver_tinyv4 = person_detect.tinyv4:main',
            'yolo_v8_rap = person_detect.yolo_v8_rap:main',
            'yolo_v8_rap_raw = person_detect.yolo_v8_rap_raw:main',
            'yolo_v4_rap = person_detect.yolov4_rap:main',
            'log_tegrastats = person_detect.log_tegrastats:main',
            'yolo_v4_sub = person_detect.yolov4_sub:main',
            'yolo_v8_sub = person_detect.yolo_v8_sub:main',
            'person_tracker = person_detect.person_tracker:main',
            'person_tracker_rap = person_detect.person_tracker_rap:main',
        ],
    },
)
