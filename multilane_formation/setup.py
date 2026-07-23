import os
from glob import glob
from setuptools import setup

package_name = 'multilane_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),

        (os.path.join('share', package_name), ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'platoon.launch*.[pxy][yma]*'))),

        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')) # + 
        #  glob(os.path.join('config_sim', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inst',
    maintainer_email='mariiaivanchenko027@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_simulation_node = multilane_formation.model_simulation:main',
            'controller_node = multilane_formation.controller:main',
            'formation_controller_node = multilane_formation.formation_layer:main',
        ],
    },
)
