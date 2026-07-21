#!/usr/bin/env python3
"""Compare a freshly generated GLB against the committed one.

This is the drift gate: it goes red when cinelog35_v3.scad changed but
client-godot/assets/cinelog35_v3.glb was not regenerated. Same contract as
extern/bf_sources.txt ("don't hand-edit it; regenerate") and codec_parity.

Byte equality is checked first and is the normal result -- build_asset.py packs
the GLB deterministically, so the same OpenSCAD on the same machine reproduces
it exactly. It is NOT required, though: the committed file is generated on
arm64 macOS while CI regenerates on x86_64 Linux, and the manifold backend is
not guaranteed to trianglulate bit-identically across architectures. So a byte
mismatch falls back to a structural comparison with tolerances loose enough to
absorb triangulation noise and tight enough that a real geometry edit cannot
slip through.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from validate_glb import load_glb


# Triangulation may differ across architectures; a real .scad edit moves this
# far more than 2%. QUALITY changes move it by tens of percent.
TRIANGLE_TOLERANCE = 0.02
# Bounding boxes are geometry, not tessellation: 0.1 mm is well inside any
# meaningful design change and well outside float noise.
BBOX_TOLERANCE_M = 1.0e-4
# Node translations are computed in exact arithmetic from the .scad constants.
TRANSLATION_TOLERANCE_M = 1.0e-6


def mesh_stats(document):
    """name -> (triangles, bbox_min, bbox_max) for every mesh."""
    stats = {}
    for mesh in document["meshes"]:
        triangles = 0
        low = [float("inf")] * 3
        high = [float("-inf")] * 3
        for primitive in mesh["primitives"]:
            if "indices" in primitive:
                triangles += document["accessors"][primitive["indices"]]["count"] // 3
            else:
                accessor = document["accessors"][primitive["attributes"]["POSITION"]]
                triangles += accessor["count"] // 3
            position = document["accessors"][primitive["attributes"]["POSITION"]]
            for axis in range(3):
                low[axis] = min(low[axis], position["min"][axis])
                high[axis] = max(high[axis], position["max"][axis])
        stats[mesh.get("name")] = (triangles, low, high)
    return stats


def node_translations(document):
    return {
        node.get("name"): tuple(node.get("translation", (0.0, 0.0, 0.0)))
        for node in document["nodes"]
    }


def names(entries):
    return {entry.get("name") for entry in entries}


def compare(fresh: Path, committed: Path) -> list[str]:
    if fresh.read_bytes() == committed.read_bytes():
        print(f"byte-identical: {fresh.name} == {committed.name}")
        return []

    print(
        "note: files are not byte-identical (expected when regenerating on a "
        "different architecture); falling back to structural comparison"
    )
    a, _ = load_glb(fresh)
    b, _ = load_glb(committed)
    problems = []

    for label, key in (("node", "nodes"), ("mesh", "meshes"), ("material", "materials")):
        fresh_names, committed_names = names(a.get(key, [])), names(b.get(key, []))
        if fresh_names != committed_names:
            added = sorted(n for n in fresh_names - committed_names if n)
            removed = sorted(n for n in committed_names - fresh_names if n)
            problems.append(
                f"{label} names differ: added={added or '[]'} removed={removed or '[]'}"
            )

    fresh_extras = a.get("asset", {}).get("extras", {})
    committed_extras = b.get("asset", {}).get("extras", {})
    if fresh_extras != committed_extras:
        problems.append(
            f"asset.extras metadata differs:\n"
            f"  regenerated: {fresh_extras}\n"
            f"  committed:   {committed_extras}"
        )

    fresh_meshes, committed_meshes = mesh_stats(a), mesh_stats(b)
    for name in sorted(set(fresh_meshes) & set(committed_meshes), key=lambda n: n or ""):
        fresh_tris, fresh_low, fresh_high = fresh_meshes[name]
        old_tris, old_low, old_high = committed_meshes[name]
        if old_tris and abs(fresh_tris - old_tris) / old_tris > TRIANGLE_TOLERANCE:
            problems.append(
                f"mesh {name}: {fresh_tris:,} triangles vs committed {old_tris:,} "
                f"(> {TRIANGLE_TOLERANCE:.0%})"
            )
        for axis, label in enumerate("xyz"):
            for fresh_v, old_v, edge in (
                (fresh_low[axis], old_low[axis], "min"),
                (fresh_high[axis], old_high[axis], "max"),
            ):
                if abs(fresh_v - old_v) > BBOX_TOLERANCE_M:
                    problems.append(
                        f"mesh {name}: bbox {edge}.{label} {fresh_v:.6f} vs "
                        f"committed {old_v:.6f} m"
                    )

    fresh_nodes, committed_nodes = node_translations(a), node_translations(b)
    for name in sorted(set(fresh_nodes) & set(committed_nodes), key=lambda n: n or ""):
        for axis, label in enumerate("xyz"):
            fresh_v = fresh_nodes[name][axis]
            old_v = committed_nodes[name][axis]
            if abs(fresh_v - old_v) > TRANSLATION_TOLERANCE_M:
                problems.append(
                    f"node {name}: translation.{label} {fresh_v:.9f} vs "
                    f"committed {old_v:.9f} m"
                )

    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fresh", type=Path, help="the freshly regenerated GLB")
    parser.add_argument("committed", type=Path, help="the GLB tracked in git")
    args = parser.parse_args()

    problems = compare(args.fresh, args.committed)
    if problems:
        print(
            f"\nFAIL: the regenerated model does not match {args.committed}.\n"
            f"If cinelog35_v3.scad was edited on purpose, regenerate and commit:\n"
            f"  python3 model/build_asset.py\n",
            file=sys.stderr,
        )
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    print(f"OK: {args.committed} is up to date with cinelog35_v3.scad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
