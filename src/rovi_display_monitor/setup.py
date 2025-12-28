from glob import glob
import os

from setuptools import setup

package_name = 'rovi_display_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wassfila',
    maintainer_email='wassim.filali@gmail.com',
    description='Serial display monitor for Rovi (voltage + CPU over ESP32-S3 USB).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_monitor = rovi_display_monitor.display_monitor_node:main',
        ],
    },
)
