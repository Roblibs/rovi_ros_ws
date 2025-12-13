from setuptools import setup
import os
from glob import glob

package_name = 'rovi_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'meshes', 'mecanum'), glob('meshes/mecanum/*.STL') + glob('meshes/mecanum/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'sensor'), glob('meshes/sensor/*.STL') + glob('meshes/sensor/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wassfila',
    maintainer_email='wassim.filali@gmail.com',
    description='URDF, meshes, and RViz config for the Rovi robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
