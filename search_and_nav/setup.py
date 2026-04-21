from setuptools import find_packages, setup
from glob import glob
import os 

package_name = 'search_and_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='den',
    maintainer_email='haidangle678@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_manager = par_snc.mission_manager:main',
            'exploration_node = par_snc.exploration_node:main',
            'hazard_mapper = par_snc.hazard_mapper:main',
            'path_tracker = par_snc.path_tracker:main',
            'start_detector = par_snc.start_detector:main',
        ],
    },
)
