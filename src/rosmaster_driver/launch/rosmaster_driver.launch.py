from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, LogInfo
from launch.substitutions import EnvironmentVariable, TextSubstitution
import os
import sys


def generate_launch_description() -> LaunchDescription:
    actions = []

    # If a virtual environment is active, prepend its site-packages to PYTHONPATH
    venv_root = os.environ.get('VIRTUAL_ENV')
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    venv_site = os.path.join(venv_root, 'lib', py_ver, 'site-packages') if venv_root else ''

    if venv_site and os.path.isdir(venv_site):
        actions.append(
            SetEnvironmentVariable(
                name='PYTHONPATH',
                value=[
                    TextSubstitution(text=venv_site),
                    TextSubstitution(text=':'),
                    EnvironmentVariable('PYTHONPATH', default_value='')
                ]
            )
        )
        # Ensure unbuffered Python stdout/stderr so logs appear immediately
        actions.append(SetEnvironmentVariable(name='PYTHONUNBUFFERED', value='1'))
        actions.append(LogInfo(msg=f"Using venv site-packages: {venv_site}"))
    else:
        actions.append(LogInfo(msg="VIRTUAL_ENV not set or site-packages not found; using existing PYTHONPATH"))

    actions.append(
        Node(
            package='rosmaster_driver',
            executable='rosmaster_driver_node',
            name='rosmaster_driver',
            output='screen',
            parameters=[{
                'imu_link': 'imu_link',
                'publish_rate': 10.0,
                'prefix': '',
                'port': '/dev/my_ros_board',
                'debug': False,
            }],
        )
    )

    return LaunchDescription(actions)
