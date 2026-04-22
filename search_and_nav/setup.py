from setuptools import find_packages, setup
import os
from glob import glob

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
        
        # Ensure BOTH images and yaml config files are installed
        (os.path.join('share', package_name, 'config'), glob('config/*.png')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='den',
    maintainer_email='haidangle678@gmail.com',
    description='ROS2 Search & Navigation Challenge Package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_node = search_and_nav.mission_node:main',
            'navigation_node = search_and_nav.navigation_node:main',
            'perception_node = search_and_nav.perception_node:main',
        ],
    },
)