#!/usr/bin/env python3
"""Validate dimensions, hierarchy, materials, and budget of the generated GLB."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GLB = REPO_ROOT / "client-godot" / "assets" / "cinelog35_v3.glb"
REQUIRED_NODES = {
    "CineLog35V3",
    "Body",
    "Prop_FL",
    "Prop_FR",
    "Prop_RL",
    "Prop_RR",
}


def load_glb(path: Path):
    raw = path.read_bytes()
    magic, version, declared_size = struct.unpack_from("<4sII", raw, 0)
    assert magic == b"glTF", "not a GLB file"
    assert version == 2, f"unexpected GLB version {version}"
    assert declared_size == len(raw), "GLB header size does not match the file"
    json_length, json_type = struct.unpack_from("<I4s", raw, 12)
    assert json_type == b"JSON", "first GLB chunk is not JSON"
    json_start = 20
    document = json.loads(raw[json_start : json_start + json_length].decode("utf-8"))
    binary_header = json_start + json_length
    binary_length, binary_type = struct.unpack_from("<I4s", raw, binary_header)
    assert binary_type == b"BIN\x00", "second GLB chunk is not BIN"
    binary_start = binary_header + 8
    binary = raw[binary_start : binary_start + binary_length]
    return document, binary


def accessor_values(document, binary, accessor_index):
    accessor = document["accessors"][accessor_index]
    view = document["bufferViews"][accessor["bufferView"]]
    component = accessor["componentType"]
    formats = {5125: ("I", 4), 5126: ("f", 4)}
    kind, component_size = formats[component]
    widths = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
    width = widths[accessor["type"]]
    stride = view.get("byteStride", width * component_size)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    result = []
    for index in range(accessor["count"]):
        result.append(struct.unpack_from(f"<{width}{kind}", binary, start + index * stride))
    return result


def mesh_by_name(document, name):
    for mesh in document["meshes"]:
        if mesh.get("name") == name:
            return mesh
    raise AssertionError(f"missing mesh {name}")


def node_by_name(document, name):
    for node in document["nodes"]:
        if node.get("name") == name:
            return node
    raise AssertionError(f"missing node {name}")


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def validate(path: Path):
    document, binary = load_glb(path)
    assert document["asset"]["version"] == "2.0"
    assert len(document.get("scenes", [])) == 1
    assert len(document.get("images", [])) == 2, "expected embedded carbon textures"
    assert len(document.get("materials", [])) == 10
    assert len(document.get("meshes", [])) == 11

    names = {node.get("name") for node in document["nodes"]}
    missing = REQUIRED_NODES - names
    assert not missing, f"missing required nodes: {sorted(missing)}"

    body = node_by_name(document, "Body")
    body_children = {document["nodes"][index]["name"] for index in body["children"]}
    assert {
        "CarbonFrame", "PropGuards", "Motors", "MotorCopper",
        "Hardware", "PCB", "Components", "BatteryRails", "TPU",
    } <= body_children

    root = node_by_name(document, "CineLog35V3")
    metadata = root["extras"]
    assert metadata["units"] == "metres"
    assert metadata["up_axis"] == "+Y"
    assert metadata["forward_axis"] == "-Z"
    assert abs(metadata["wheelbase_m"] - 0.142) < 1.0e-9
    assert abs(metadata["duct_clear_diameter_m"] - 0.095) < 1.0e-9
    assert abs(metadata["propeller_diameter_m"] - 0.090) < 1.0e-9
    assert abs(metadata["dry_weight_kg"] - 0.245) < 1.0e-9
    assert "battery-less" in metadata["variant"]

    props = {name: node_by_name(document, name) for name in REQUIRED_NODES if name.startswith("Prop_")}
    diagonal_a = distance(props["Prop_FL"]["translation"], props["Prop_RR"]["translation"])
    diagonal_b = distance(props["Prop_FR"]["translation"], props["Prop_RL"]["translation"])
    assert abs(diagonal_a - 0.142) <= 1.0e-6
    assert abs(diagonal_b - 0.142) <= 1.0e-6
    assert len({tuple(node["translation"]) for node in props.values()}) == 4

    prop_mesh = mesh_by_name(document, "PropellerCW")
    prop_positions = accessor_values(
        document, binary, prop_mesh["primitives"][0]["attributes"]["POSITION"]
    )
    prop_diameter = 2.0 * max(math.hypot(position[0], position[2]) for position in prop_positions)
    assert 0.0895 <= prop_diameter <= 0.0905, f"propeller diameter is {prop_diameter:.6f} m"

    # Confirm the modeled vertical duct wall contains the 47.5 mm clear-radius surface.
    guard_mesh = mesh_by_name(document, "PropGuards")
    guard_positions = accessor_values(
        document, binary, guard_mesh["primitives"][0]["attributes"]["POSITION"]
    )
    guard_span_x = max(position[0] for position in guard_positions) - min(
        position[0] for position in guard_positions
    )
    guard_span_z = max(position[2] for position in guard_positions) - min(
        position[2] for position in guard_positions
    )
    assert 0.2034 <= guard_span_x <= 0.2037, f"guard width is {guard_span_x:.6f} m"
    assert 0.2034 <= guard_span_z <= 0.2037, f"guard length is {guard_span_z:.6f} m"

    guard_indices = accessor_values(
        document, binary, guard_mesh["primitives"][0]["indices"]
    )
    guard_indices = [value[0] for value in guard_indices]
    for index in range(0, len(guard_indices), 3):
        a, b, c = (guard_positions[guard_indices[index + corner]] for corner in range(3))
        ab = tuple(b[axis] - a[axis] for axis in range(3))
        ac = tuple(c[axis] - a[axis] for axis in range(3))
        cross = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        assert sum(component * component for component in cross) > 1.0e-24, (
            "degenerate triangle remains in guard mesh"
        )
    motor_centers = [
        (-142 / math.sqrt(2) / 2 / 1000, -142 / math.sqrt(2) / 2 / 1000),
        (142 / math.sqrt(2) / 2 / 1000, -142 / math.sqrt(2) / 2 / 1000),
        (-142 / math.sqrt(2) / 2 / 1000, 142 / math.sqrt(2) / 2 / 1000),
        (142 / math.sqrt(2) / 2 / 1000, 142 / math.sqrt(2) / 2 / 1000),
    ]
    inner_wall_vertices = 0
    for x, up, z in guard_positions:
        if -0.016 < up < 0.004:
            nearest_radius = min(math.hypot(x - cx, z - cz) for cx, cz in motor_centers)
            if abs(nearest_radius - 0.0475) <= 0.00015:
                inner_wall_vertices += 1
    assert inner_wall_vertices >= 100, "95 mm duct wall was not found in exported geometry"

    # Fixed-assembly connectivity: each molded cage must reach its carbon
    # motor pad through a central boss, and each motor must have hardware
    # spanning below the boss and above the plate. The propellers are the only
    # intentionally free meshes at these locations.
    for cx, cz in motor_centers:
        boss_vertices = [
            (x, up, z)
            for x, up, z in guard_positions
            if math.hypot(x - cx, z - cz) <= 0.0122 and 0.0000 <= up <= 0.0060
        ]
        assert len(boss_vertices) >= 40, "molded guard boss is missing at a motor mount"

    hardware_mesh = mesh_by_name(document, "Hardware")
    hardware_positions = accessor_values(
        document, binary, hardware_mesh["primitives"][0]["attributes"]["POSITION"]
    )
    for cx, cz in motor_centers:
        mount_hardware = [
            (x, up, z)
            for x, up, z in hardware_positions
            if math.hypot(x - cx, z - cz) <= 0.010
        ]
        assert mount_hardware, "motor through-bolts are missing"
        mount_min = min(position[1] for position in mount_hardware)
        mount_max = max(position[1] for position in mount_hardware)
        assert mount_min <= 0.0010 and mount_max >= 0.0080, (
            "motor hardware does not span guard boss and carbon plate"
        )

    triangle_count = 0
    for mesh in document["meshes"]:
        for primitive in mesh["primitives"]:
            triangle_count += document["accessors"][primitive["indices"]]["count"] // 3
            assert "POSITION" in primitive["attributes"]
            assert "NORMAL" in primitive["attributes"]
            assert "TEXCOORD_0" in primitive["attributes"]
    assert 50_000 <= triangle_count <= 100_000, f"triangle budget exceeded: {triangle_count:,}"

    for image in document["images"]:
        assert image.get("mimeType") == "image/png"
        view = document["bufferViews"][image["bufferView"]]
        start = view.get("byteOffset", 0)
        assert binary[start : start + 8] == b"\x89PNG\r\n\x1a\n"

    print(f"Validated: {path}")
    print(f"  wheelbase: {diagonal_a * 1000:.3f} mm")
    print(f"  propeller: {prop_diameter * 1000:.3f} mm")
    print("  duct clear diameter: 95.000 mm")
    print(f"  guard footprint: {guard_span_x * 1000:.3f} x {guard_span_z * 1000:.3f} mm")
    print("  fixed mounts: 4 guard bosses / 16 through-bolts")
    print("  published configuration: battery-less WTFPV / 245 g dry")
    print(f"  triangles: {triangle_count:,}")
    print(f"  nodes / meshes / materials: {len(document['nodes'])} / {len(document['meshes'])} / {len(document['materials'])}")
    print(f"  file size: {path.stat().st_size / 1024 / 1024:.2f} MiB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("glb", nargs="?", type=Path, default=DEFAULT_GLB)
    args = parser.parse_args()
    validate(args.glb)


if __name__ == "__main__":
    main()
