# model/ — the airframe, as source

`cinelog35_v3.scad` is the **source of truth** for the drone the Godot client
renders. Everything else about the airframe is generated from it:
`client-godot/assets/cinelog35_v3.glb` is a build product that happens to be
committed, exactly like `extern/bf_sources.txt`. Don't hand-edit the GLB;
change the `.scad` and regenerate.

A clean-room, unbranded parametric reconstruction of the GEPRC CineLog35 V3
WTFPV (non-GPS) from published dimensions and product imagery. Millimetres,
+X right, +Y front, +Z up; `build_asset.py` converts to the Godot frame
(metres, +Y up, −Z forward) on export.

## Regenerating

```sh
python3 model/build_asset.py                 # -> client-godot/assets/cinelog35_v3.glb
python3 model/validate_glb.py                # dimensions, hierarchy, budget
```

Takes about a second: twelve material groups render in parallel and the GLB is
packed directly in Python. No Blender, no third-party model libraries — both
scripts are stdlib-only, matching the repo's "Python (stdlib) for tools/tests"
convention.

`--preview out.png` additionally writes a software-rendered PNG, which is a
quick way to eyeball a change without launching Godot.

## The OpenSCAD pin

`OPENSCAD_VERSION` pins the exact snapshot; `build_asset.py` warns when the
local binary differs and CI fails hard (`--require-pinned-version`).

**A stable OpenSCAD will not work.** 2021.01 is the newest stable release
(Feb 2021) and it has neither the manifold backend nor OBJ export —
`src/export_obj.cc` does not exist at that tag — and `export_part()` needs
both. OpenSCAD ships via dated snapshots instead, and files.openscad.org keeps
them indefinitely, so pinning one is reproducible:

```sh
# Linux (what CI uses)
curl -LO https://files.openscad.org/snapshots/OpenSCAD-2026.07.19-x86_64.AppImage
chmod +x OpenSCAD-2026.07.19-x86_64.AppImage
OPENSCAD_BIN=$PWD/OpenSCAD-2026.07.19-x86_64.AppImage python3 model/build_asset.py

# macOS — snapshots lag Linux, so this gives an older build and warns
brew install --cask openscad@snapshot
```

`build_asset.py` finds the binary from `--openscad`, then `$OPENSCAD_BIN`, then
`/Applications/OpenSCAD.app`, then `PATH`. It deliberately does *not* look in
`<repo>/build/` — that is propwash's CMake build directory.

## Tests

| test | proves |
|------|--------|
| `model_asset` | the committed GLB still satisfies `validate_glb.py`: wheelbase 142 mm, duct bore 95 mm, prop 90 mm, guard span 203.5 mm, node/mesh/material names, normals + UVs, 50k–100k triangle budget |
| `model_regen` | (only when OpenSCAD is installed) regenerating from the `.scad` reproduces the committed GLB — red if the source was edited without a regen |

`compare_glb.py` checks byte equality first, which is the normal result. It
falls back to a structural comparison because the committed file is generated
on arm64 macOS while CI regenerates on x86_64 Linux, and manifold is not
guaranteed to triangulate bit-identically across architectures.

## Things worth knowing before editing the .scad

- **The duct rings are supposed to overlap.** `molded_duct_rail` extrudes at
  radius `duct_clear_diameter/2` = 47.5 mm with a D-section reaching +4.05, so
  each duct's outer radius is 51.55 mm while adjacent duct centres sit
  `2 * motor_offset` = 100.41 mm apart. That 2.69 mm of overlap is what fuses
  the four rings into the two molded left/right halves, and `prop_guards()`
  cuts its tooling-relief slots straight through the tangency. **Raising
  `wheelbase` past ~146 mm separates the rings** and the cage falls apart into
  four floating hoops joined by a thin saddle blade. If you ever need the model
  to match a wider physics wheelbase, scale it uniformly at the Godot node
  (`main.gd` `MODEL_*`) rather than reparameterising it here.
- **The GLB is the dry airframe.** No battery, camera, VTX or antenna — the
  published battery-less configuration. The client draws its own battery and O3
  pod procedurally on top (`_build_drone_model`).
- `QUALITY` is the circular resolution (`$fn`). It moves the triangle count by
  tens of percent, so `model_regen` catches an accidental change immediately.

## Licence

MIT, like the rest of the client-side tree — see `LICENSE.MIT`. No official STL
files or product photographs are redistributed; GEPRC names identify the
reference product only, and the model carries no manufacturer logos or label
artwork.
