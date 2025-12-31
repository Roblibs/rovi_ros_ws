from setuptools import setup
import os
from glob import glob
from pathlib import Path
import sys
import json

package_name = 'rovi_description'

pkg_root = Path(__file__).parent.resolve()
sys.path.insert(0, str(pkg_root))


def _ensure_glb_generated() -> None:
    # Generate the GLB model as a build artifact (do not commit).
    urdf_path = pkg_root / 'urdf' / 'rovi.urdf'
    out_glb = pkg_root / 'models' / 'rovi.glb'
    try:
        from rovi_description.urdf_to_glb import generate_glb_if_needed
    except Exception as exc:
        print(f"[rovi_description] Failed to import URDF->GLB generator: {exc}", file=sys.stderr)
        raise

    try:
        regenerated = generate_glb_if_needed(urdf_path=urdf_path, out_glb_path=out_glb, package_root=pkg_root)
        if regenerated:
            meta_path = Path(f"{out_glb}.meta.json")
            glb_hint = str(out_glb)
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
                glb_rel = meta.get("glb", {}).get("filename")
                if glb_rel is None:
                    glb_rel = meta.get("glb", {}).get("path")
                if isinstance(glb_rel, str) and glb_rel:
                    glb_hint = str((meta_path.parent / glb_rel).resolve())
            except Exception:
                pass
            print(f"[rovi_description] Generated GLB + meta: {glb_hint}", file=sys.stderr)
    except Exception as exc:
        print(f"[rovi_description] Failed to generate GLB from {urdf_path}: {exc}", file=sys.stderr)
        raise


_ensure_glb_generated()

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
        (os.path.join('share', package_name, 'models'), glob('models/*.glb') + glob('models/*.glb.meta.json')),
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
