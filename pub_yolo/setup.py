from setuptools import find_packages, setup

package_name = 'pub_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lei',
    maintainer_email='aries94leifu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = pub_yolo.image_pub:main',
            'dynamic_freq = pub_yolo.dynamic_freq:main',
            'dynamic_freq_comp = pub_yolo.dynamic_freq_comp:main',
            'image_pub_comp = pub_yolo.image_pub_comp:main']
    },
)
