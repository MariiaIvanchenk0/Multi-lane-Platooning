import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'omnisim'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Lightweight omni robot simulator for testing ROS 2 multi-agent stacks.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omni_sim_node = omnisim.omni_sim_node:main',
        ],
    },
)
