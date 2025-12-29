from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'rovi_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wassfila',
    maintainer_email='wassim.filali@gmail.com',
    description='Bringup launch files for Room View Bot teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rovi_session = rovi_bringup.cli.session:main',
            'rovi_bag = rovi_bringup.cli.bag:main',
            'viz_downsample = rovi_bringup.viz_downsample_node:main',
        ],
    },
)
