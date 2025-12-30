from __future__ import annotations

import hashlib
import json
import math
import struct
from array import array
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class MaterialDef:
    name: str
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class MeshVisual:
    link_name: str
    mesh_path: Path
    mesh_scale: tuple[float, float, float]
    material_name: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]


@dataclass(frozen=True)
class JointDef:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]


def generate_glb_if_needed(*, urdf_path: Path, out_glb_path: Path, package_root: Path) -> bool:
    """Generate a GLB if the URDF or any referenced meshes are newer than the output.

    Returns True if the GLB was regenerated.
    """
    urdf_path = urdf_path.resolve()
    out_glb_path = out_glb_path.resolve()
    package_root = package_root.resolve()

    mesh_paths = _collect_mesh_paths_from_urdf(urdf_path=urdf_path, package_root=package_root)
    # Also include this generator module itself so changes to the exporter regenerate the model.
    input_paths = [urdf_path, *mesh_paths, Path(__file__).resolve()]

    newest_input_mtime = max(p.stat().st_mtime_ns for p in input_paths)
    try:
        out_mtime = out_glb_path.stat().st_mtime_ns
        if out_mtime >= newest_input_mtime:
            return False
    except FileNotFoundError:
        pass

    out_glb_path.parent.mkdir(parents=True, exist_ok=True)
    glb = urdf_to_glb_bytes(urdf_path=urdf_path, package_root=package_root)
    out_glb_path.write_bytes(glb)
    return True


def urdf_to_glb_bytes(*, urdf_path: Path, package_root: Path) -> bytes:
    urdf_path = urdf_path.resolve()
    package_root = package_root.resolve()

    robot_name, materials, joints, visuals = _parse_urdf(urdf_path=urdf_path, package_root=package_root)
    urdf_rel = _try_relpath(urdf_path, package_root=package_root)
    urdf_sha256 = hashlib.sha256(urdf_path.read_bytes()).hexdigest()

    # Build glTF materials.
    gltf_materials: list[dict] = []
    material_index_by_name: dict[str, int] = {}
    for mat in materials.values():
        idx = len(gltf_materials)
        material_index_by_name[mat.name] = idx
        r, g, b, a = mat.rgba
        gltf_materials.append(
            {
                "name": mat.name,
                "doubleSided": True,
                "pbrMetallicRoughness": {
                    "baseColorFactor": [float(r), float(g), float(b), float(a)],
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.9,
                },
            }
        )

    # Always add a default material so visuals with unknown / missing materials have a safe fallback.
    if "default" not in material_index_by_name:
        material_index_by_name["default"] = len(gltf_materials)
        gltf_materials.append(
            {
                "name": "default",
                "doubleSided": True,
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.7, 0.7, 0.7, 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.9,
                },
            }
        )

    # Build glTF meshes from referenced STL files.
    bin_buffer = bytearray()
    buffer_views: list[dict] = []
    accessors: list[dict] = []
    meshes: list[dict] = []

    mesh_cache: dict[tuple[Path, int], int] = {}
    mesh_meta_by_path: dict[Path, dict] = {}

    def add_bytes(data: bytes, *, target: Optional[int]) -> int:
        nonlocal bin_buffer
        offset = len(bin_buffer)
        bin_buffer.extend(data)
        _pad4(bin_buffer)
        view = {"buffer": 0, "byteOffset": offset, "byteLength": len(data)}
        if target is not None:
            view["target"] = int(target)
        buffer_views.append(view)
        return len(buffer_views) - 1

    def add_accessor(*, view_idx: int, count: int, type_name: str, min_vals: Optional[list[float]] = None, max_vals: Optional[list[float]] = None) -> int:
        accessor: dict = {
            "bufferView": view_idx,
            "componentType": 5126,  # FLOAT
            "count": int(count),
            "type": type_name,
        }
        if min_vals is not None:
            accessor["min"] = [float(v) for v in min_vals]
        if max_vals is not None:
            accessor["max"] = [float(v) for v in max_vals]
        accessors.append(accessor)
        return len(accessors) - 1

    def add_stl_mesh(mesh_path: Path, *, material_idx: int) -> int:
        key = (mesh_path, material_idx)
        cached = mesh_cache.get(key)
        if cached is not None:
            return cached

        positions, normals, bounds, stl_sha256, stl_size_bytes = _read_binary_stl(mesh_path)
        mesh_meta_by_path.setdefault(
            mesh_path,
            {
                "path": _try_relpath(mesh_path, package_root=package_root),
                "sha256": stl_sha256,
                "size_bytes": int(stl_size_bytes),
            },
        )
        view_pos = add_bytes(positions.tobytes(), target=34962)  # ARRAY_BUFFER
        view_nrm = add_bytes(normals.tobytes(), target=34962)

        vertex_count = len(positions) // 3
        acc_pos = add_accessor(
            view_idx=view_pos,
            count=vertex_count,
            type_name="VEC3",
            min_vals=[bounds[0], bounds[1], bounds[2]],
            max_vals=[bounds[3], bounds[4], bounds[5]],
        )
        acc_nrm = add_accessor(view_idx=view_nrm, count=vertex_count, type_name="VEC3")

        mesh_idx = len(meshes)
        meshes.append(
            {
                "name": mesh_path.stem,
                "primitives": [
                    {
                        "attributes": {"POSITION": acc_pos, "NORMAL": acc_nrm},
                        "material": int(material_idx),
                    }
                ],
            }
        )
        mesh_cache[key] = mesh_idx
        return mesh_idx

    # Parse link graph and create nodes.
    nodes: list[dict] = []
    link_children_by_parent: dict[str, list[JointDef]] = {}
    all_links: set[str] = set()
    child_links: set[str] = set()
    for joint in joints:
        link_children_by_parent.setdefault(joint.parent, []).append(joint)
        all_links.add(joint.parent)
        all_links.add(joint.child)
        child_links.add(joint.child)

    # Also include links that only have visuals.
    for vis in visuals:
        all_links.add(vis.link_name)

    roots = sorted(all_links - child_links) if all_links else [robot_name]
    root_link = roots[0] if roots else robot_name

    link_node_by_name: dict[str, int] = {}

    def add_node(name: str, *, translation: Optional[tuple[float, float, float]] = None, rotation: Optional[tuple[float, float, float, float]] = None, scale: Optional[tuple[float, float, float]] = None, mesh: Optional[int] = None) -> int:
        node: dict = {"name": name}
        if translation is not None:
            node["translation"] = [float(translation[0]), float(translation[1]), float(translation[2])]
        if rotation is not None:
            node["rotation"] = [float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])]
        if scale is not None:
            node["scale"] = [float(scale[0]), float(scale[1]), float(scale[2])]
        if mesh is not None:
            node["mesh"] = int(mesh)
        nodes.append(node)
        return len(nodes) - 1

    def add_child(parent_idx: int, child_idx: int) -> None:
        node = nodes[parent_idx]
        node.setdefault("children", []).append(int(child_idx))

    # Index visuals by link.
    visuals_by_link: dict[str, list[MeshVisual]] = {}
    for vis in visuals:
        visuals_by_link.setdefault(vis.link_name, []).append(vis)

    def build_link(link_name: str, parent_idx: Optional[int]) -> int:
        link_idx = link_node_by_name.get(link_name)
        if link_idx is None:
            link_idx = add_node(link_name)
            link_node_by_name[link_name] = link_idx

        if parent_idx is not None:
            add_child(parent_idx, link_idx)

        for vis_idx, vis in enumerate(visuals_by_link.get(link_name, [])):
            rgba_name = vis.material_name or "default"
            mat_idx = material_index_by_name.get(rgba_name, material_index_by_name["default"])
            mesh_idx = add_stl_mesh(vis.mesh_path, material_idx=mat_idx)
            q = _quat_from_rpy(*vis.origin_rpy)
            vis_node = add_node(
                f"{link_name}__visual{vis_idx}",
                translation=vis.origin_xyz,
                rotation=q,
                scale=vis.mesh_scale,
                mesh=mesh_idx,
            )
            add_child(link_idx, vis_node)

        for joint in sorted(link_children_by_parent.get(link_name, []), key=lambda j: j.name):
            q = _quat_from_rpy(*joint.origin_rpy)
            origin_node = add_node(
                f"{joint.name}__origin",
                translation=joint.origin_xyz,
                rotation=q,
            )
            add_child(link_idx, origin_node)

            # Rotatable node (this is the one clients should rotate for wheel joints).
            rot_node = add_node(joint.name)
            add_child(origin_node, rot_node)

            build_link(joint.child, rot_node)

        return link_idx

    scene_roots: list[int] = []
    for root in roots:
        scene_roots.append(build_link(root, None))

    extras = {
        "roblibs": {
            "urdf_to_glb": {
                "robot_name": robot_name,
                "coordinate_convention": "ROS (X forward, Y left, Z up)",
                "urdf": {"path": urdf_rel, "sha256": urdf_sha256},
                "meshes": [mesh_meta_by_path[p] for p in sorted(mesh_meta_by_path.keys(), key=lambda x: str(x))],
            }
        }
    }

    gltf: dict = {
        "asset": {"version": "2.0", "generator": "rovi_description.urdf_to_glb", "extras": extras},
        "scene": 0,
        "scenes": [{"name": robot_name, "nodes": scene_roots}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": gltf_materials,
        "buffers": [{"byteLength": len(bin_buffer)}],
        "bufferViews": buffer_views,
        "accessors": accessors,
    }

    json_chunk = json.dumps(gltf, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return _build_glb(json_chunk=json_chunk, bin_chunk=bytes(bin_buffer))


def _collect_mesh_paths_from_urdf(*, urdf_path: Path, package_root: Path) -> list[Path]:
    _robot_name, _materials, _joints, visuals = _parse_urdf(urdf_path=urdf_path, package_root=package_root)
    out: list[Path] = []
    for vis in visuals:
        out.append(vis.mesh_path)
    return sorted(set(out))


def _parse_urdf(*, urdf_path: Path, package_root: Path) -> tuple[str, dict[str, MaterialDef], list[JointDef], list[MeshVisual]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"Expected <robot> root in {urdf_path}")

    robot_name = str(root.get("name") or "robot")

    materials = _parse_materials(root)
    joints = _parse_joints(root)
    visuals = _parse_visuals(root, materials=materials, package_root=package_root, urdf_dir=urdf_path.parent)

    return robot_name, materials, joints, visuals


def _parse_materials(robot: ET.Element) -> dict[str, MaterialDef]:
    out: dict[str, MaterialDef] = {}
    for mat in robot.findall("material"):
        name = str(mat.get("name") or "").strip()
        if not name:
            continue
        color = mat.find("color")
        if color is None:
            continue
        rgba = _parse_rgba(color.get("rgba"))
        out[name] = MaterialDef(name=name, rgba=rgba)
    return out


def _parse_joints(robot: ET.Element) -> list[JointDef]:
    out: list[JointDef] = []
    for joint in robot.findall("joint"):
        name = str(joint.get("name") or "").strip()
        if not name:
            continue
        joint_type = str(joint.get("type") or "fixed").strip()
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        if parent_el is None or child_el is None:
            continue
        parent = str(parent_el.get("link") or "").strip()
        child = str(child_el.get("link") or "").strip()
        if not parent or not child:
            continue
        origin_xyz, origin_rpy = _parse_origin(joint.find("origin"))
        out.append(
            JointDef(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
            )
        )
    return out


def _parse_visuals(
    robot: ET.Element,
    *,
    materials: dict[str, MaterialDef],
    package_root: Path,
    urdf_dir: Path,
) -> list[MeshVisual]:
    out: list[MeshVisual] = []

    for link in robot.findall("link"):
        link_name = str(link.get("name") or "").strip()
        if not link_name:
            continue

        for visual in link.findall("visual"):
            origin_xyz, origin_rpy = _parse_origin(visual.find("origin"))
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            mesh = geometry.find("mesh")
            if mesh is None:
                continue
            filename = str(mesh.get("filename") or "").strip()
            if not filename:
                continue

            mesh_path = _resolve_urdf_path(filename, package_root=package_root, urdf_dir=urdf_dir)
            scale = _parse_xyz(mesh.get("scale")) if mesh.get("scale") else (1.0, 1.0, 1.0)

            material_name = "default"
            mat_el = visual.find("material")
            if mat_el is not None:
                mat_name = str(mat_el.get("name") or "").strip()
                color_el = mat_el.find("color")
                if color_el is not None:
                    rgba = _parse_rgba(color_el.get("rgba"))
                    if mat_name:
                        materials.setdefault(mat_name, MaterialDef(name=mat_name, rgba=rgba))
                    else:
                        mat_name = f"{link_name}_material"
                        materials.setdefault(mat_name, MaterialDef(name=mat_name, rgba=rgba))
                    material_name = mat_name
                elif mat_name and mat_name in materials:
                    material_name = mat_name

            out.append(
                MeshVisual(
                    link_name=link_name,
                    mesh_path=mesh_path,
                    mesh_scale=scale,
                    material_name=material_name,
                    origin_xyz=origin_xyz,
                    origin_rpy=origin_rpy,
                )
            )

    return out


def _resolve_urdf_path(value: str, *, package_root: Path, urdf_dir: Path) -> Path:
    raw = str(value).strip()
    if raw.startswith("package://"):
        rest = raw[len("package://") :]
        pkg, _, rel = rest.partition("/")
        if not pkg or not rel:
            raise ValueError(f"Invalid package URI: {raw}")
        if pkg == "rovi_description":
            return (package_root / rel).resolve()
        raise ValueError(f"Unsupported package URI (only rovi_description supported at build-time): {raw}")

    p = Path(raw)
    if p.is_absolute():
        return p
    return (urdf_dir / p).resolve()


def _parse_origin(origin: Optional[ET.Element]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = _parse_xyz(origin.get("xyz"))
    rpy = _parse_xyz(origin.get("rpy"))
    return xyz, rpy


def _parse_xyz(raw: Optional[str]) -> tuple[float, float, float]:
    if not raw:
        return 0.0, 0.0, 0.0
    parts = str(raw).strip().split()
    if len(parts) != 3:
        return 0.0, 0.0, 0.0
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_rgba(raw: Optional[str]) -> tuple[float, float, float, float]:
    if not raw:
        return 0.7, 0.7, 0.7, 1.0
    parts = str(raw).strip().split()
    if len(parts) != 4:
        return 0.7, 0.7, 0.7, 1.0
    return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return _normalize_quat(qx, qy, qz, qw)


def _normalize_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or math.isinf(norm) or math.isnan(norm):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def _read_binary_stl(path: Path) -> tuple[array, array, tuple[float, float, float, float, float, float]]:
    data = path.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()
    size_bytes = len(data)
    if len(data) < 84:
        raise ValueError(f"Invalid STL (too small): {path}")

    tri_count = struct.unpack_from("<I", data, 80)[0]
    expected_len = 84 + tri_count * 50
    if len(data) < expected_len:
        raise ValueError(f"Invalid STL (unexpected length): {path}")

    positions = array("f")
    normals = array("f")

    min_x = float("inf")
    min_y = float("inf")
    min_z = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")
    max_z = float("-inf")

    view = memoryview(data)[84:expected_len]
    for nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz, _attr in struct.iter_unpack("<12fH", view):
        positions.extend([ax, ay, az, bx, by, bz, cx, cy, cz])
        normals.extend([nx, ny, nz, nx, ny, nz, nx, ny, nz])

        min_x = min(min_x, ax, bx, cx)
        min_y = min(min_y, ay, by, cy)
        min_z = min(min_z, az, bz, cz)
        max_x = max(max_x, ax, bx, cx)
        max_y = max(max_y, ay, by, cy)
        max_z = max(max_z, az, bz, cz)

    if not math.isfinite(min_x):
        min_x = min_y = min_z = 0.0
        max_x = max_y = max_z = 0.0

    return positions, normals, (min_x, min_y, min_z, max_x, max_y, max_z), sha256, size_bytes


def _try_relpath(path: Path, *, package_root: Path) -> str:
    try:
        return str(path.resolve().relative_to(package_root.resolve()))
    except Exception:
        return str(path)


def _pad4(buf: bytearray) -> None:
    pad = (-len(buf)) % 4
    if pad:
        buf.extend(b"\x00" * pad)


def _build_glb(*, json_chunk: bytes, bin_chunk: bytes) -> bytes:
    json_padded = bytearray(json_chunk)
    while len(json_padded) % 4 != 0:
        json_padded.extend(b" ")

    bin_padded = bytearray(bin_chunk)
    while len(bin_padded) % 4 != 0:
        bin_padded.extend(b"\x00")

    header_len = 12
    chunk_header_len = 8
    total_len = header_len + chunk_header_len + len(json_padded) + chunk_header_len + len(bin_padded)

    out = bytearray()
    out.extend(struct.pack("<4sII", b"glTF", 2, total_len))
    out.extend(struct.pack("<I4s", len(json_padded), b"JSON"))
    out.extend(json_padded)
    out.extend(struct.pack("<I4s", len(bin_padded), b"BIN\x00"))
    out.extend(bin_padded)
    return bytes(out)
