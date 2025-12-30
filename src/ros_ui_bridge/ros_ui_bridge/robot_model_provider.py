from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class RobotModel:
    glb_bytes: bytes
    sha256: str
    size_bytes: int


def resolve_resource_path(path: str) -> Path:
    value = str(path).strip()
    if not value:
        raise ValueError("Empty path")

    if value.startswith('package://'):
        rest = value[len('package://') :]
        pkg, _, rel = rest.partition('/')
        if not pkg or not rel:
            raise ValueError(f"Invalid package URI (expected package://<pkg>/<path>): {value}")
        share_dir = Path(get_package_share_directory(pkg))
        return (share_dir / rel).resolve()

    out = Path(value)
    if out.is_absolute():
        return out

    # Relative paths are interpreted as relative to ros_ui_bridge's share dir.
    share_dir = Path(get_package_share_directory('ros_ui_bridge'))
    return (share_dir / value).resolve()


class RobotModelProvider:
    def __init__(self, *, glb_path: str | None) -> None:
        self._glb_path_raw = str(glb_path).strip() if glb_path else ""
        self._cached_path: Optional[Path] = None
        self._cached_mtime_ns: Optional[int] = None
        self._cached_model: Optional[RobotModel] = None

    @property
    def glb_path(self) -> str:
        return self._glb_path_raw

    def is_configured(self) -> bool:
        return bool(self._glb_path_raw)

    def load(self) -> RobotModel:
        if not self._glb_path_raw:
            raise FileNotFoundError("robot_model.glb_path is not set")

        path = resolve_resource_path(self._glb_path_raw)
        stat = path.stat()
        mtime_ns = int(stat.st_mtime_ns)

        if (
            self._cached_model is not None
            and self._cached_path == path
            and self._cached_mtime_ns == mtime_ns
        ):
            return self._cached_model

        data = path.read_bytes()
        sha = hashlib.sha256(data).hexdigest()
        model = RobotModel(glb_bytes=data, sha256=sha, size_bytes=len(data))

        self._cached_path = path
        self._cached_mtime_ns = mtime_ns
        self._cached_model = model

        return model

