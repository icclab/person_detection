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
        (os.path.join('share', package_name, 'launch'), glob('launch/yolov4-tiny.cfg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/yolov4-tiny.weights')),
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
            'person_detect_driver_v8 = person_detect.v8:main',
            'person_detect_driver_tinyv4 = person_detect.tinyv4:main',
        ],
    },
)
