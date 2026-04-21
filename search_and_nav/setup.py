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
            'mission_manager = search_and_nav.mission_manager:main',
            'exploration_node = search_and_nav.exploration_node:main',
            'hazard_mapper = search_and_nav.hazard_mapper:main',
            'path_tracker = search_and_nav.path_tracker:main',
            'start_detector = search_and_nav.start_detector:main',
            'mock_detection_adapter = search_and_nav.mock_detection_adapter:main',
        ],
    },
)
