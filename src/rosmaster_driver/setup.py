from setuptools import setup
from glob import glob

package_name = 'rosmaster_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='todo@example.com',
    description='Minimal Rosmaster driver: subscribes to cmd_vel and publishes IMU, mag, voltage, edition, joint_states, and vel_raw.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosmaster_driver_node = rosmaster_driver.driver_node:main',
        ],
    },
)
