from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_serial_display'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wassfila',
    maintainer_email='wassim.filali@gmail.com',
    description='Serial display client for ros_ui_bridge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_display = robot_serial_display.serial_display_node:main',
            'serial_display_test = robot_serial_display.serial_display_test:main',
        ],
    },
)
