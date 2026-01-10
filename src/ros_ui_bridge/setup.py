from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ros_ui_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'proto'), glob('proto/*.proto')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wassfila',
    maintainer_email='wassim.filali@gmail.com',
    description='ROS UI bridge: gRPC status/metrics streaming.',
    license='Apache-2.0',
entry_points={
        'console_scripts': [
            'ui_bridge = ros_ui_bridge.ui_bridge_node:main',
        ],
    },
)
