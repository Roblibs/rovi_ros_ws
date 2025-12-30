from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class RobotModelMeta:
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
        self._cached_meta_path: Optional[Path] = None
        self._cached_meta_mtime_ns: Optional[int] = None
        self._cached_meta: Optional[RobotModelMeta] = None

    @property
    def glb_path(self) -> str:
        return self._glb_path_raw

    @property
    def meta_path(self) -> str:
        if not self._glb_path_raw:
            return ""
        return f"{self._glb_path_raw}.meta.json"

    def is_configured(self) -> bool:
        return bool(self._glb_path_raw)

    def resolve_glb_path(self) -> Path:
        if not self._glb_path_raw:
            raise FileNotFoundError("robot_model.glb_path is not set")

        return resolve_resource_path(self._glb_path_raw)

    def load_meta(self) -> RobotModelMeta:
        if not self._glb_path_raw:
            raise FileNotFoundError("robot_model.glb_path is not set")

        meta_path = resolve_resource_path(self.meta_path)
        stat = meta_path.stat()
        mtime_ns = int(stat.st_mtime_ns)

        if (
            self._cached_meta is not None
            and self._cached_meta_path == meta_path
            and self._cached_meta_mtime_ns == mtime_ns
        ):
            return self._cached_meta

        raw = meta_path.read_text(encoding="utf-8")
        doc = json.loads(raw)
        meta = _parse_robot_model_meta_json(doc)

        self._cached_meta_path = meta_path
        self._cached_meta_mtime_ns = mtime_ns
        self._cached_meta = meta

        return meta


def _parse_robot_model_meta_json(doc: object) -> RobotModelMeta:
    if not isinstance(doc, dict):
        raise ValueError("Robot model meta JSON must be an object")

    glb = doc.get("glb")
    if not isinstance(glb, dict):
        raise ValueError('Robot model meta JSON must contain object key "glb"')

    sha256 = glb.get("sha256")
    if not isinstance(sha256, str) or not sha256:
        raise ValueError('Robot model meta JSON must contain non-empty string "glb.sha256"')

    size_bytes = glb.get("size_bytes")
    if not isinstance(size_bytes, int) or size_bytes < 0:
        raise ValueError('Robot model meta JSON must contain non-negative int "glb.size_bytes"')

    return RobotModelMeta(sha256=sha256, size_bytes=size_bytes)
