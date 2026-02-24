from glob import glob
import os
import shutil
import subprocess

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py

package_name = 'ros_ui_bridge'


class build_py(_build_py):
    def run(self) -> None:
        self._generate_proto_stubs()
        super().run()

    def _generate_proto_stubs(self) -> None:
        protoc_bin = shutil.which("protoc")
        if not protoc_bin:  # pragma: no cover
            raise RuntimeError(
                "Missing build dependency for ros_ui_bridge proto generation.\n"
                "Install: sudo apt install -y protobuf-compiler protobuf-compiler-grpc\n"
                "Then rebuild (clean && build)."
            )

        grpc_py_plugin = shutil.which("grpc_python_plugin")
        if not grpc_py_plugin:  # pragma: no cover
            raise RuntimeError(
                "Missing gRPC Python protoc plugin for ros_ui_bridge proto generation.\n"
                "Install: sudo apt install -y protobuf-compiler-grpc\n"
                "Then rebuild (clean && build)."
            )

        here = os.path.abspath(os.path.dirname(__file__))
        proto_dir = os.path.join(here, "proto")
        proto_file = os.path.join(proto_dir, "ui_bridge.proto")

        out_dir = os.path.join(self.build_lib, package_name, "api")
        os.makedirs(out_dir, exist_ok=True)

        cmd = [
            protoc_bin,
            f"-I{proto_dir}",
            f"--python_out={out_dir}",
            f"--plugin=protoc-gen-grpc_python={grpc_py_plugin}",
            f"--grpc_python_out={out_dir}",
            proto_file,
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:  # pragma: no cover
            stderr = (res.stderr or "").strip()
            stdout = (res.stdout or "").strip()
            detail = "\n".join([s for s in (stdout, stderr) if s])
            raise RuntimeError(
                "protoc failed while generating ros_ui_bridge proto stubs.\n"
                f"Command: {' '.join(cmd)}\n"
                f"{detail}"
            )


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
    cmdclass={"build_py": build_py},
)
