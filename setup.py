import os
from glob import glob
from setuptools import setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),

        (os.path.join('share', package_name), ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'model.launch*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'longitudinal.launch*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'lateral.launch*.[pxy][yma]*'))),

        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('params', '*.yaml')) + 
         glob(os.path.join('config_sim', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inst',
    maintainer_email='ahmedfahim@uwaterloo.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_simulation_node = simulation.model_simulation:main',
            'longitudinal_controller_node = simulation.longitudinal_controller:main',
            'lateral_controller_node = simulation.lateral_controller:main',
        ],
    },
)
