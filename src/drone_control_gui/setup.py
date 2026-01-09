from setuptools import setup
import os
from glob import glob

package_name = 'drone_control_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.widgets', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='PyQt5-based GUI for controlling PX4 drones',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control_gui = drone_control_gui.main_window:main',
        ],
    },
)
