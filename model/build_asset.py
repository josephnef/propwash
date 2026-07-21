#!/usr/bin/env python3
"""Build the CineLog35 V3 OpenSCAD source into a self-contained Godot GLB."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import zlib


MODEL_DIR = Path(__file__).resolve().parent
REPO_ROOT = MODEL_DIR.parent
SOURCE = MODEL_DIR / "cinelog35_v3.scad"
DEFAULT_OUTPUT = REPO_ROOT / "client-godot" / "assets" / "cinelog35_v3.glb"
OBJ_DIR = MODEL_DIR / ".build" / "obj"

# The OpenSCAD snapshot this asset is generated with. There is no usable stable
# release: 2021.01 is the newest, and it has neither the manifold backend nor
# OBJ export (src/export_obj.cc does not exist at that tag), both of which
# export_part() below requires. OpenSCAD ships via dated snapshots instead, and
# those ARE pinnable -- files.openscad.org keeps them indefinitely.
#
# CI downloads exactly this build and passes --require-pinned-version. A local
# regen on a nearby snapshot only warns, so day-to-day work is not blocked.
PINNED_OPENSCAD_VERSION = "2026.07.19"

MOTOR_OFFSET_MM = 142.0 / math.sqrt(2.0) / 2.0
PROPELLER_Z_MM = 4.0
CG_Y_M = 0.008

PARTS = [
    ("CarbonFrame", "CarbonFrame", "carbon"),
    ("PropGuards", "PropGuards", "guards"),
    ("Motors", "Motors", "motors"),
    ("Copper", "MotorCopper", "copper"),
    ("Hardware", "Hardware", "hardware"),
    ("PCB", "PCB", "pcb"),
    ("Components", "Components", "components"),
    ("Aluminum", "BatteryRails", "aluminum"),
    ("TPU", "TPU", "tpu"),
    ("PropCW", "PropellerCW", "propeller"),
    ("PropCCW", "PropellerCCW", "propeller"),
]

MATERIALS = {
    "carbon": {
        "name": "CarbonFiber",
        # The weave image is intentionally dark and multiplies this factor.
        # A mid-grey factor yields a physically black, but still readable,
        # carbon laminate in Godot instead of crushing it to zero.
        "base": [0.36, 0.38, 0.40, 1.0],
        "metallic": 0.04,
        "roughness": 0.40,
        "texture": True,
    },
    "guards": {
        "name": "ModifiedPolymer",
        # Satin black polymer still needs enough linear-light value for the
        # molded rail shoulders and webs to read in Godot's neutral lighting.
        "base": [0.014, 0.018, 0.024, 1.0],
        "metallic": 0.0,
        "roughness": 0.46,
    },
    "motors": {
        "name": "MotorAluminum",
        "base": [0.008, 0.011, 0.016, 1.0],
        "metallic": 0.76,
        "roughness": 0.29,
    },
    "hardware": {
        "name": "SteelHardware",
        "base": [0.055, 0.065, 0.078, 1.0],
        "metallic": 0.84,
        "roughness": 0.32,
    },
    "pcb": {
        "name": "PCB",
        "base": [0.006, 0.024, 0.016, 1.0],
        "metallic": 0.12,
        "roughness": 0.42,
    },
    "components": {
        "name": "ElectronicComponents",
        "base": [0.025, 0.032, 0.038, 1.0],
        "metallic": 0.18,
        "roughness": 0.38,
    },
    "copper": {
        "name": "MotorCopper",
        "base": [0.48, 0.145, 0.026, 1.0],
        "metallic": 0.78,
        "roughness": 0.33,
    },
    "tpu": {
        "name": "TPU",
        "base": [0.005, 0.007, 0.009, 1.0],
        "metallic": 0.0,
        "roughness": 0.72,
    },
    "aluminum": {
        "name": "BlackAluminum",
        "base": [0.012, 0.016, 0.022, 1.0],
        "metallic": 0.68,
        "roughness": 0.31,
    },
    "propeller": {
        "name": "SmokedPropeller",
        "base": [0.014, 0.018, 0.025, 1.0],
        "metallic": 0.0,
        "roughness": 0.48,
        "double_sided": True,
    },
}


def find_openscad(explicit: str | None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("OPENSCAD_BIN"):
        candidates.append(Path(os.environ["OPENSCAD_BIN"]).expanduser())
    # Deliberately NOT <repo>/build/openscad: in propwash `build/` is the CMake
    # build directory, and a stray match there would be nonsense.
    candidates.append(Path("/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"))
    on_path = shutil.which("openscad")
    if on_path:
        candidates.append(Path(on_path))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise SystemExit(
        f"OpenSCAD executable not found. Install the pinned snapshot "
        f"({PINNED_OPENSCAD_VERSION}) -- on macOS `brew install --cask "
        f"openscad@snapshot`, on Linux the matching AppImage from "
        f"files.openscad.org/snapshots -- then set OPENSCAD_BIN or pass "
        f"--openscad /path/to/openscad. Note the 2021.01 stable release will "
        f"NOT work: it has no manifold backend and no OBJ export."
    )


def openscad_version(openscad: Path) -> str:
    """Return the reported version, e.g. '2026.06.12'. Empty if unparseable."""
    result = subprocess.run(
        [str(openscad), "--version"], text=True, capture_output=True
    )
    # OpenSCAD prints "OpenSCAD version 2026.06.12" -- and does so on stderr on
    # some builds, stdout on others, so scan both.
    for stream in (result.stdout, result.stderr):
        for token in (stream or "").split():
            if token[:4].isdigit() and token.count(".") >= 2:
                return token
    return ""


def check_pinned_version(openscad: Path, require: bool) -> None:
    found = openscad_version(openscad)
    if found == PINNED_OPENSCAD_VERSION:
        return
    message = (
        f"OpenSCAD is {found or 'an unknown version'}, but this asset is pinned "
        f"to {PINNED_OPENSCAD_VERSION} (see model/OPENSCAD_VERSION)."
    )
    if require:
        raise SystemExit(f"error: {message}")
    print(f"warning: {message} Regenerating anyway; CI pins the exact build.")


def export_part(openscad: Path, part: str, quality: int) -> Path:
    OBJ_DIR.mkdir(parents=True, exist_ok=True)
    output = OBJ_DIR / f"{part}.obj"
    command = [
        str(openscad),
        "--backend=manifold",
        "--export-format=obj",
        "-D",
        f'PART="{part}"',
        "-D",
        f"QUALITY={quality}",
        "-o",
        str(output),
        str(SOURCE),
    ]
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode != 0 or not output.exists():
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise RuntimeError(f"OpenSCAD export failed for {part}")
    return output


def export_all(openscad: Path, quality: int, jobs: int) -> None:
    print(f"Using OpenSCAD: {openscad}")
    print(f"Exporting {len(PARTS)} material groups at quality {quality}...")
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        futures = {
            pool.submit(export_part, openscad, part, quality): part for part, _, _ in PARTS
        }
        for future in concurrent.futures.as_completed(futures):
            part = futures[future]
            path = future.result()
            print(f"  {part:14s} {path.stat().st_size / 1024:8.1f} KiB")


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_mul(a, scalar):
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_normalize(a):
    length = math.sqrt(max(0.0, vec_dot(a, a)))
    if length < 1.0e-12:
        return (0.0, 1.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def scad_to_godot(p, center_on_cg=True):
    up = p[2] * 0.001 - (CG_Y_M if center_on_cg else 0.0)
    return (p[0] * 0.001, up, -p[1] * 0.001)


def parse_obj(path: Path, textured: bool, center_on_cg: bool):
    source_positions = []
    source_triangles = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            _, x, y, z = line.split()[:4]
            source_positions.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            refs = [int(token.split("/")[0]) - 1 for token in line.split()[1:]]
            for i in range(1, len(refs) - 1):
                source_triangles.append((refs[0], refs[i], refs[i + 1]))

    if not source_positions or not source_triangles:
        raise RuntimeError(f"No mesh data found in {path}")

    positions_m = [scad_to_godot(p, center_on_cg=center_on_cg) for p in source_positions]
    # Manifold's OBJ triangulation can retain zero-area faces along coincident
    # CSG seams.  They render as a stray line primitive in Assimp/Godot, so
    # discard them before calculating normals and packing the GLB.
    source_triangles = [
        tri
        for tri in source_triangles
        if vec_dot(
            vec_cross(
                vec_sub(positions_m[tri[1]], positions_m[tri[0]]),
                vec_sub(positions_m[tri[2]], positions_m[tri[0]]),
            ),
            vec_cross(
                vec_sub(positions_m[tri[1]], positions_m[tri[0]]),
                vec_sub(positions_m[tri[2]], positions_m[tri[0]]),
            ),
        ) > 1.0e-24
    ]
    face_normals = []
    incident = [[] for _ in positions_m]
    for face_index, tri in enumerate(source_triangles):
        a, b, c = (positions_m[index] for index in tri)
        normal = vec_normalize(vec_cross(vec_sub(b, a), vec_sub(c, a)))
        face_normals.append(normal)
        for vertex_index in tri:
            incident[vertex_index].append(face_index)

    # Split original vertices across hard edges, averaging faces within 38 degrees.
    corner_cluster = {}
    clusters_by_vertex = []
    crease_cos = math.cos(math.radians(38.0))
    for vertex_index, faces in enumerate(incident):
        clusters = []
        for face_index in faces:
            normal = face_normals[face_index]
            selected = None
            for cluster_index, cluster in enumerate(clusters):
                if vec_dot(normal, vec_normalize(cluster["sum"])) >= crease_cos:
                    selected = cluster_index
                    break
            if selected is None:
                clusters.append({"sum": normal, "faces": [face_index]})
                selected = len(clusters) - 1
            else:
                clusters[selected]["sum"] = vec_add(clusters[selected]["sum"], normal)
                clusters[selected]["faces"].append(face_index)
            corner_cluster[(vertex_index, face_index)] = selected
        clusters_by_vertex.append(clusters)

    positions = []
    normals = []
    texcoords = []
    remap = {}
    for vertex_index, clusters in enumerate(clusters_by_vertex):
        source_mm = source_positions[vertex_index]
        for cluster_index, cluster in enumerate(clusters):
            new_index = len(positions)
            remap[(vertex_index, cluster_index)] = new_index
            positions.append(positions_m[vertex_index])
            normals.append(vec_normalize(cluster["sum"]))
            if textured:
                texcoords.append((source_mm[0] / 8.0, source_mm[1] / 8.0))
            else:
                texcoords.append((0.0, 0.0))

    indices = []
    for face_index, tri in enumerate(source_triangles):
        for vertex_index in tri:
            indices.append(remap[(vertex_index, corner_cluster[(vertex_index, face_index)])])

    return {
        "positions": positions,
        "normals": normals,
        "texcoords": texcoords,
        "indices": indices,
        "source_triangles": source_triangles,
        "source_positions_m": positions_m,
    }


def png_rgba(width: int, height: int, pixels: bytes) -> bytes:
    signature = b"\x89PNG\r\n\x1a\n"

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))

    rows = b"".join(
        b"\x00" + pixels[y * width * 4 : (y + 1) * width * 4] for y in range(height)
    )
    return (
        signature
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows, 9))
        + chunk(b"IEND", b"")
    )


def carbon_textures(size=64):
    color = bytearray()
    normal = bytearray()
    for y in range(size):
        for x in range(size):
            diagonal_a = ((x + y) // 4) % 2
            diagonal_b = ((x - y) // 4) % 2
            weave = diagonal_a ^ diagonal_b
            fine = ((x + y) % 4) == 0
            value = 20 + weave * 3 + (1 if fine else 0)
            color.extend((value, value + 2, value + 4, 255))
            nx = 128 + (3 if diagonal_a else -3)
            ny = 128 + (3 if diagonal_b else -3)
            normal.extend((nx, ny, 254, 255))
    return png_rgba(size, size, bytes(color)), png_rgba(size, size, bytes(normal))


class BinaryBlob:
    def __init__(self):
        self.data = bytearray()

    def add(self, payload: bytes) -> tuple[int, int]:
        while len(self.data) % 4:
            self.data.append(0)
        offset = len(self.data)
        self.data.extend(payload)
        return offset, len(payload)


def pack_floats(values, width):
    flat = [component for value in values for component in value[:width]]
    return struct.pack(f"<{len(flat)}f", *flat)


def min_max(values, width):
    return (
        [min(value[i] for value in values) for i in range(width)],
        [max(value[i] for value in values) for i in range(width)],
    )


def build_glb(meshes, output: Path):
    blob = BinaryBlob()
    document = {
        "asset": {
            "version": "2.0",
            "generator": "OpenSCAD CineLog35 V3 deterministic builder",
            "extras": {
                "source": "source/cinelog35_v3.scad",
                "purpose": "game simulator visual asset; not manufacturing geometry",
            },
        },
        "scene": 0,
        "scenes": [{"name": "CineLog35V3", "nodes": []}],
        "nodes": [],
        "meshes": [],
        "materials": [],
        "accessors": [],
        "bufferViews": [],
        "buffers": [{"byteLength": 0}],
    }

    carbon_color, carbon_normal = carbon_textures()
    document["images"] = []
    for name, payload in (("CarbonWeave", carbon_color), ("CarbonNormal", carbon_normal)):
        offset, length = blob.add(payload)
        view_index = len(document["bufferViews"])
        document["bufferViews"].append({"buffer": 0, "byteOffset": offset, "byteLength": length})
        document["images"].append(
            {"name": name, "bufferView": view_index, "mimeType": "image/png"}
        )
    document["samplers"] = [
        {"magFilter": 9729, "minFilter": 9987, "wrapS": 10497, "wrapT": 10497}
    ]
    document["textures"] = [
        {"name": "CarbonWeave", "sampler": 0, "source": 0},
        {"name": "CarbonNormal", "sampler": 0, "source": 1},
    ]

    material_indices = {}
    for key, config in MATERIALS.items():
        material = {
            "name": config["name"],
            "pbrMetallicRoughness": {
                "baseColorFactor": config["base"],
                "metallicFactor": config["metallic"],
                "roughnessFactor": config["roughness"],
            },
        }
        if config.get("texture"):
            material["pbrMetallicRoughness"]["baseColorTexture"] = {"index": 0}
            material["normalTexture"] = {"index": 1, "scale": 0.08}
        if config.get("alpha"):
            material["alphaMode"] = "BLEND"
        if config.get("double_sided"):
            material["doubleSided"] = True
        material_indices[key] = len(document["materials"])
        document["materials"].append(material)

    mesh_indices = {}
    total_triangles = 0
    for part, mesh_name, material_key in PARTS:
        mesh = meshes[part]
        attributes = {}
        for semantic, values, width in (
            ("POSITION", mesh["positions"], 3),
            ("NORMAL", mesh["normals"], 3),
            ("TEXCOORD_0", mesh["texcoords"], 2),
        ):
            payload = pack_floats(values, width)
            offset, length = blob.add(payload)
            view_index = len(document["bufferViews"])
            document["bufferViews"].append(
                {
                    "buffer": 0,
                    "byteOffset": offset,
                    "byteLength": length,
                    "target": 34962,
                }
            )
            minimum, maximum = min_max(values, width)
            accessor = {
                "bufferView": view_index,
                "componentType": 5126,
                "count": len(values),
                "type": "VEC3" if width == 3 else "VEC2",
            }
            if semantic == "POSITION":
                accessor["min"] = minimum
                accessor["max"] = maximum
            accessor_index = len(document["accessors"])
            document["accessors"].append(accessor)
            attributes[semantic] = accessor_index

        index_payload = struct.pack(f"<{len(mesh['indices'])}I", *mesh["indices"])
        offset, length = blob.add(index_payload)
        view_index = len(document["bufferViews"])
        document["bufferViews"].append(
            {"buffer": 0, "byteOffset": offset, "byteLength": length, "target": 34963}
        )
        accessor_index = len(document["accessors"])
        document["accessors"].append(
            {
                "bufferView": view_index,
                "componentType": 5125,
                "count": len(mesh["indices"]),
                "type": "SCALAR",
                "min": [min(mesh["indices"])],
                "max": [max(mesh["indices"])],
            }
        )
        primitive = {
            "attributes": attributes,
            "indices": accessor_index,
            "material": material_indices[material_key],
            "mode": 4,
        }
        mesh_indices[part] = len(document["meshes"])
        document["meshes"].append({"name": mesh_name, "primitives": [primitive]})
        total_triangles += len(mesh["indices"]) // 3

    def add_node(name, **kwargs):
        index = len(document["nodes"])
        document["nodes"].append({"name": name, **kwargs})
        return index

    body_parts = [
        "CarbonFrame",
        "PropGuards",
        "Motors",
        "Copper",
        "Hardware",
        "PCB",
        "Components",
        "Aluminum",
        "TPU",
    ]
    body_children = [
        add_node(next(name for part, name, _ in PARTS if part == part_name), mesh=mesh_indices[part_name])
        for part_name in body_parts
    ]
    body_node = add_node("Body", children=body_children)

    prop_specs = [
        ("Prop_FL", -MOTOR_OFFSET_MM, MOTOR_OFFSET_MM, "PropCW", 1),
        ("Prop_FR", MOTOR_OFFSET_MM, MOTOR_OFFSET_MM, "PropCCW", -1),
        ("Prop_RL", -MOTOR_OFFSET_MM, -MOTOR_OFFSET_MM, "PropCCW", -1),
        ("Prop_RR", MOTOR_OFFSET_MM, -MOTOR_OFFSET_MM, "PropCW", 1),
    ]
    prop_nodes = []
    for name, x_mm, y_mm, mesh_part, direction in prop_specs:
        translation = scad_to_godot((x_mm, y_mm, PROPELLER_Z_MM), center_on_cg=True)
        prop_nodes.append(
            add_node(
                name,
                mesh=mesh_indices[mesh_part],
                translation=list(translation),
                extras={"spin_direction": direction, "pivot": "motor_shaft"},
            )
        )

    root_node = add_node(
        "CineLog35V3",
        children=[body_node, *prop_nodes],
        extras={
            "units": "metres",
            "up_axis": "+Y",
            "forward_axis": "-Z",
            "wheelbase_m": 0.142,
            "duct_clear_diameter_m": 0.095,
            "propeller_diameter_m": 0.090,
            "variant": "WTFPV non-GPS, published battery-less configuration",
            "dry_weight_kg": 0.245,
            "branding": "unbranded",
            "origin": "estimated dry center of mass",
        },
    )
    document["scenes"][0]["nodes"] = [root_node]

    document["buffers"][0]["byteLength"] = len(blob.data)
    json_bytes = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    json_bytes += b" " * ((4 - len(json_bytes) % 4) % 4)
    binary_bytes = bytes(blob.data)
    binary_bytes += b"\x00" * ((4 - len(binary_bytes) % 4) % 4)
    total_length = 12 + 8 + len(json_bytes) + 8 + len(binary_bytes)
    glb = (
        struct.pack("<4sII", b"glTF", 2, total_length)
        + struct.pack("<I4s", len(json_bytes), b"JSON")
        + json_bytes
        + struct.pack("<I4s", len(binary_bytes), b"BIN\x00")
        + binary_bytes
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(glb)
    return total_triangles, document


def render_preview(meshes, output: Path):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("Pillow is unavailable; skipping preview render", file=sys.stderr)
        return

    panel_width, panel_height = 640, 480
    image = Image.new("RGB", (panel_width * 2, panel_height * 2), (222, 226, 231))
    light = vec_normalize((-0.35, 0.85, 0.45))
    world_triangles = []
    for part, _, material_key in PARTS:
        mesh = meshes[part]
        translations = [(0.0, 0.0, 0.0)]
        if part == "PropCW":
            translations = [
                scad_to_godot((-MOTOR_OFFSET_MM, MOTOR_OFFSET_MM, PROPELLER_Z_MM)),
                scad_to_godot((MOTOR_OFFSET_MM, -MOTOR_OFFSET_MM, PROPELLER_Z_MM)),
            ]
        elif part == "PropCCW":
            translations = [
                scad_to_godot((MOTOR_OFFSET_MM, MOTOR_OFFSET_MM, PROPELLER_Z_MM)),
                scad_to_godot((-MOTOR_OFFSET_MM, -MOTOR_OFFSET_MM, PROPELLER_Z_MM)),
            ]
        base = MATERIALS[material_key]["base"]
        for translation in translations:
            vertices = [vec_add(v, translation) for v in mesh["source_positions_m"]]
            for ia, ib, ic in mesh["source_triangles"]:
                a, b, c = vertices[ia], vertices[ib], vertices[ic]
                normal = vec_normalize(vec_cross(vec_sub(b, a), vec_sub(c, a)))
                center = vec_mul(vec_add(vec_add(a, b), c), 1.0 / 3.0)
                world_triangles.append((a, b, c, normal, base))

    views = [
        ("THREE-QUARTER", (0.275, 0.20, 0.315), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 2100.0),
        ("TOP", (0.001, 0.42, 0.001), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 2100.0),
        ("FRONT", (0.0, 0.075, 0.40), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 2200.0),
        ("RIGHT", (0.40, 0.075, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 2200.0),
    ]
    for view_index, (label, eye, target, up_hint, scale) in enumerate(views):
        panel = Image.new("RGB", (panel_width, panel_height), (222, 226, 231))
        draw = ImageDraw.Draw(panel)
        forward = vec_normalize(vec_sub(target, eye))
        right = vec_normalize(vec_cross(forward, up_hint))
        camera_up = vec_normalize(vec_cross(right, forward))
        triangles = []
        for a, b, c, normal, base in world_triangles:
            center = vec_mul(vec_add(vec_add(a, b), c), 1.0 / 3.0)
            if vec_dot(normal, vec_sub(eye, center)) <= 0.0:
                continue
            projected = []
            depths = []
            for vertex in (a, b, c):
                relative = vec_sub(vertex, target)
                projected.append(
                    (
                        panel_width * 0.5 + vec_dot(relative, right) * scale,
                        panel_height * 0.52 - vec_dot(relative, camera_up) * scale,
                    )
                )
                depths.append(vec_dot(vec_sub(vertex, eye), forward))
            brightness = 0.48 + 0.52 * max(0.0, vec_dot(normal, light))
            # glTF colors are linear; gamma-encode for this small software preview.
            color = tuple(
                max(0, min(255, int(255 * (base[channel] * brightness) ** (1.0 / 2.2))))
                for channel in range(3)
            )
            triangles.append((sum(depths) / 3.0, projected, color))
        for _, polygon, color in sorted(triangles, key=lambda item: item[0], reverse=True):
            draw.polygon(polygon, fill=color)
        draw.rectangle((0, 0, panel_width - 1, panel_height - 1), outline=(180, 185, 192))
        draw.rectangle((14, 14, 124, 38), fill=(35, 40, 47))
        draw.text((22, 21), label, fill=(242, 244, 246))
        image.paste(panel, ((view_index % 2) * panel_width, (view_index // 2) * panel_height))

    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openscad", help="Path to the OpenSCAD executable")
    parser.add_argument("--quality", type=int, default=36, help="OpenSCAD circular resolution")
    parser.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1))
    parser.add_argument("--skip-export", action="store_true", help="Reuse .build/obj files")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--preview",
        type=Path,
        default=None,
        help="Also write a software-rendered preview PNG here (off by default)",
    )
    parser.add_argument(
        "--require-pinned-version",
        action="store_true",
        help=f"Fail unless OpenSCAD is exactly {PINNED_OPENSCAD_VERSION} (CI uses this)",
    )
    args = parser.parse_args()

    if not args.skip_export:
        openscad = find_openscad(args.openscad)
        check_pinned_version(openscad, args.require_pinned_version)
        export_all(openscad, args.quality, args.jobs)
    missing = [part for part, _, _ in PARTS if not (OBJ_DIR / f"{part}.obj").exists()]
    if missing:
        raise SystemExit(f"Missing exported OBJ groups: {', '.join(missing)}")

    meshes = {}
    for part, _, material_key in PARTS:
        meshes[part] = parse_obj(
            OBJ_DIR / f"{part}.obj",
            textured=MATERIALS[material_key].get("texture", False),
            # Propeller meshes are authored around their own shaft pivots and
            # receive node translations below. PropGuards is fixed bodywork;
            # matching by prefix shifted the whole cage 20 mm upward.
            center_on_cg=part not in {"PropCW", "PropCCW"},
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    triangles, _ = build_glb(meshes, args.output)
    print(f"Wrote {args.output} ({args.output.stat().st_size / 1024 / 1024:.2f} MiB)")
    print(f"Triangles: {triangles:,}")
    if args.preview is not None:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        render_preview(meshes, args.preview)
        print(f"Preview: {args.preview}")


if __name__ == "__main__":
    main()
