# CLAUDE.md

Guidance for Claude Code (claude.ai/code) working in this repository.

## What this is

**propwash** — an FPV drone simulator that compiles **real Betaflight 4.5.2**
into the sim process and drives its scheduler tick-by-tick in deterministic
lockstep. Not networked SITL: the firmware runs *in-process* as a static
library and the physics feeds its virtual gyro/accel, reads its motor outputs,
every tick. Everything downstream (Godot client, gym, tools) speaks a small UDP
protocol.

## Build & test

No external dependencies (kernel `js` API, not SDL2). One pinned submodule.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
ctest --test-dir build            # 31 ctests: 16 always, +13 with a godot binary, +model_regen with an OpenSCAD snapshot, +gym_hover if uv-synced; ~5 min
```

The Betaflight submodule builds as a static lib (`extern/`). Source list is
harvested from an authoritative `make TARGET=SITL` build into
`extern/bf_sources.txt` — don't hand-edit it; regenerate from the object list
if the firmware version changes.

## The tests are the contract (run them after any core change)

| ctest | proves |
|-------|--------|
| `headless_scenario` | BF boots, MSP identity = `BTFL 4.5.2`, arms, angle-hover; prints `STATE_HASH` (printed, not asserted) |
| `contact_drop` | contact solver: 0.5 m drops (level + inverted) settle dead still on physics alone; the inverted one STAYS inverted |
| `crash_scenario` | crash lifecycle: rest never damages; 10 m drop = structural crash; a wrecked quad cannot hover (physics, not script); REPAIR restores hover |
| `collision_e2e` | damage bands over the wire: 1.2 m drop → small damage, 10 m → crash latch, REPAIR clears (reads `prop_damage`/`crash_flags`) |
| `udp_e2e` | Python client drives the wire protocol, hovers |
| `codec_parity` | the two Python wire codecs (GPL `tools/tester/pw_udp.py` vs the gym's MIT `protocol.py`) agree field-for-field — changing one alone goes red |
| `diff_roundtrip` | apply a `diff all` over CLI → save → in-process reboot → settings survive |
| `osd_render` | real `osd.c` renders through the fake displayport → `PW_OSD` |
| `real_tune_hover` | bake the real diff → fresh boot has `p_pitch=53`, `align_board_roll=0` → hovers |
| `godot_client` | the actual GDScript client arms + hovers headless (`--headless`, autotest); must end with zero damage |
| `flythrough` | autonomous angle-mode gate run — gates are SOLID; asserts real clearance + zero damage |
| `quality_tiers` | render tier selection rule as a pure function (cases a single machine can't produce) |
| `fpv_cull` | FPV cull mask + prop→motor index map — invariants a rewrite has already silently dropped once |
| `client_collision` | client detection: hull vs analytic ground + engine-query gate tubes, slop depenetration, Godot→sim manifold conversion |
| `repair_flow` | crash from 10 m, repair in place: damage/latch cleared, quad upright, and no stale-echo pose flicker — the one-deep-pipeline bug that made `T` "not work" |
| `gym_hover` | the Python Gymnasium env spawns a real core, arms + hovers, and passes the env-checker including the step-determinism sub-check (enforced) |
| `sysid_selfcheck` | physics-only motor replay (`PW_MOTOR_IN`) is bit-reproducible across cores; the fitter recovers a hidden physics parameter from a synthetic reference |
| `dshot_device` | virtual DSHOT ESC: boot grace clears, arms + hovers on dshot600 (the default protocol now), SPIN_DIRECTION commands through the stock command queue land in `pw_motor_dir` |
| `turtle_flip` | crashflip end to end: settle inverted, arm with the turtle box, reversed props pivot the quad over its duct edge (emergent contact-solver maneuver), normal re-arm restores directions |
| `rpm_filter` | bidir-DSHOT eRPM: the virtual ESC encodes physics-true rpm into real eRPM period frames; firmware-side telemetry rpm matches physics within quantization and the RPM filter runs during a stable hover |
| `reset_determinism` | one core, the identical input tape twice around `PW_CMD_RESET` → byte-identical streams (fails on any stray rand()/clock in the sim path, or anything leaking through the snapshot reset) |
| `cross_process_determinism` | two fresh cores, identical inputs (one send-jittered) → byte-identical output — the headline claim, gated |
| `model_asset` | the committed airframe GLB still meets its dimensional contract: wheelbase 142 mm, duct bore 95 mm, prop 90 mm, guard span 203.5 mm, node/mesh/material names, normals + UVs, 50k–100k triangles. Pure stdlib Python — no OpenSCAD, no Godot, so it runs on Windows too |
| `model_regen` | regenerating from `model/cinelog35_v3.scad` reproduces the committed GLB — red if the source was edited without a regen (needs an OpenSCAD snapshot; skipped otherwise) |
| `acro_gateline` | the same gate run in REAL acro (self-level off), flown by the demo pilot's cascaded controller — same clearance margins as `flythrough` |
| `demo_hover_quality` | settled hover body-rate RMS is below 0.10 rad/s. Guards a class of regression no position/damage assert can see: a marginally stable cascade holds station while buzzing, which reads on video as the airframe trembling |
| `demo_freestyle` | 22 acro maneuvers through the `park` scene (gates, backflip, loop gate, slalom, roll, 15 m tower dive, orbit, split-S, yaw spin) all complete, no crash latch, near-zero damage |
| `demo_turtle_client` | crash inverted -> turtle -> fly away through the Godot client (the C++ `turtle_flip` covers only the core). Bakes the real tune first — `small_angle = 180` is what lets it arm inverted |
| `shader_compile` | every .gdshader actually parses. Headless never builds the goggle pass, so a broken goggle.gdshader once shipped through the whole suite green while failing on any real renderer |
| `demo_retune` | live retune over the real CLI: pin `roll_srate`, roll, land, disarm, change it over TCP 5761, `save`, reboot, fly the IDENTICAL stick input — peak roll rate must change (683 -> 1031 deg/s). Goes through the Python harness for a private eeprom, since the chapter issues a real `save` |
| `demo_ghost` | determinism through the CLIENT's own state (position integration, contact detection, reply pipeline): record a stick tape, reset, replay it -> EXACTLY 0.0 m separation; then replay with one stick sample 1 us different -> must diverge |

The thirteen client tests (`godot_client`, `flythrough`, `quality_tiers`,
`fpv_cull`, `client_collision`, `repair_flow`, `shader_compile`,
`acro_gateline`, `demo_hover_quality`, `demo_freestyle`, `demo_ghost`,
`demo_turtle_client`, `demo_retune`) only run if a `godot` binary is found (see `find_program(GODOT_BIN ...)` in `CMakeLists.txt`). `model_regen`
only runs if an OpenSCAD **snapshot** is found — the 2021.01 stable release
cannot build the model at all (no manifold backend, no OBJ export).
`gym_hover` only runs if a Python with `numpy`+`gymnasium` is found — prefer
`python/propwash_gym/.venv` (run `uv sync` there first, then re-run
`cmake -B build` so it's detected).
Tests that bind ports must not overlap — leftover `propwash-core` processes
cause spurious failures; kill strays before running
(`pkill -f build/propwash-core`). A physically-connected joystick has RC
priority and will hijack the headless tests — autotest/demo spawn the core
with `--no-js`.

## How the pieces fit

- **`core/sim/`** — the loop. `sim.cpp` accumulates a 20 kHz tick (must exceed
  the 8 kHz gyro rate or the scheduler starves RX/MSP). Each tick:
  `physics.updateGyro` → `BF::update` (advances `micros_passed`, injects sensors,
  runs `BF::scheduler()`) → `physics.updatePhysics`. Physics ported from SimITL.
- **`core/sim/bfbridge.h`** — the C++/Betaflight seam: BF's C headers included
  inside `namespace SimITL::BF` with `extern "C"`, so firmware symbols are
  reachable as `BF::*`.
- **`extern/betaflightext/`** — the override layer. Small on purpose:
  `pw_sitl.c` (modified 4.5.2 SITL target — deterministic time, no UDP threads),
  patched `scheduler.c` and `cli.c`, and a fake max7456 displayport for OSD
  capture. Every modified stock file has a recorded diff in `patches/`.
- **`core/net/server.cpp`** — the UDP protocol server. Boots BF immediately (so
  the Configurator can attach before a client), idle-ticks *only while no client
  is driving* (keeps MSP/CLI alive), streams `PW_STATE_OUT` + `PW_OSD`.
- **`client-godot/`** — Godot 4, pure GDScript. Spawns the core, drives it in
  lockstep, owns collision *detection* (5-sphere hull vs analytic ground +
  engine shape queries for gates/trees → contact manifold on the wire; the
  core resolves contacts as forces — the client never edits velocities),
  converts sim↔Godot handedness (mirror z; quat -x,-y; angular velocity is a
  pseudovector: -x,-y,+z).
- **`client-godot/scripts/world.gd`** — the world: sky, ground, treeline,
  gates, and the scene registry (`PROPWASH_SCENE`). `field` is the original
  layout three ctests assert against; `park` is `field` plus freestyle
  furniture (bando shell with threadable gaps, 14 m scaffold tower, tall loop
  gate, slalom, hand-placed near trees) and is strictly additive.
- **`client-godot/scripts/demo_pilot.gd`** — the demo pilot: a cascaded
  controller that flies in REAL acro (position -> velocity -> accel ->
  thrust-vector attitude -> body-rate PI+FF -> stick), a maneuver library, and
  the chapter state machines (`freestyle`, `turtle`, `ghost`, `retune`,
  `reel`). It deliberately does NOT model Betaflight's rate curve — reading it
  needs MSP, which forfeits determinism — so the inner loop closes on rate
  ERROR and works against any tune.
- **`client-godot/scripts/demo_director.gd`** — camera rig (FPV / chase / LOS
  tripod) with per-maneuver cuts and lower-third captions. The O3 goggle
  treatment and the OSD are FPV-only, via the shader's `feed_mix` uniform.
- **`python/propwash_gym/`** — MIT Gymnasium env (`uv`-managed). Same lockstep
  loop as the Godot client: subprocess-per-env, one PW_STATE_IN→PW_STATE_OUT per
  step, client-side position integration + `ground_manifold`. Carries its own
  MIT copy of the wire codec (`protocol.py`) rather than importing the GPL
  `tools/tester/pw_udp.py`. `reset()` drives arming (gyro-cal warm-up → ARM);
  defaults `gyro_noise=0`. Rollouts are bit-reproducible across `reset()`s
  (the core's snapshot reset makes reset ≡ fresh process), so Gymnasium's
  step-determinism check is enforced — gated core-side by
  `reset_determinism`/`cross_process_determinism`.
- **`model/`** — the airframe as *source*. `cinelog35_v3.scad` is a clean-room
  parametric CineLog35 V3; `build_asset.py` (stdlib only — it hand-packs the
  GLB, no Blender) renders 12 material groups through OpenSCAD and emits
  `client-godot/assets/cinelog35_v3.glb`, a committed build product in the same
  sense as `extern/bf_sources.txt`. The client loads it at runtime through
  `GLTFDocument`, not the Godot importer, so the project still has zero
  imported resources and ctest needs no import pass.
- **`tools/sysid/`** — blackbox replay + system ID (stdlib Python). Reuses
  `tools/tester/pw_udp.py` and adds `PW_INIT` + `PW_MOTOR_IN` codecs (`wire.py`).
  Two replay modes: RC (`PW_STATE_IN`, firmware in the loop) and physics-only
  (`PW_MOTOR_IN`, firmware bypassed — the core sets `motorsState[i].pwm`
  directly and skips `BF::update`). The fitter (`sysid.hooke_jeeves`) injects
  candidate physics via `PW_INIT` (→ `reinitPhysics` on a live core, no
  respawn) and minimises gyro+accel RMSE. Physics-only replay is bit-exact
  across cores and isolates physics-model error from PID error — a bad fit
  points at the physics, not the tune.

## Non-obvious things that will bite you (learned the hard way)

### Timing, lockstep and the scheduler

- **Tick rate must exceed the PID rate.** At exactly 8 kHz, RX/MSP never run
  (scheduler only services them between gyro boundaries) → `rcData` frozen, no
  arming. 20 kHz fixes it.
- **`--server` must boot BF immediately and idle-tick** — otherwise an idle core
  (no client) leaves MSP/CLI/Configurator dead. But the idle tick must NOT fire
  while a client is driving: it advances the same accumulator the client does, so
  any packet later than the 5 ms recv timeout used to inject simulated time
  nobody asked for and made runs unreproducible. That is what `PW_CMD_LOCKSTEP`
  (the default) enforces; `PW_CMD_REALTIME` opts back into free-running.
- **The client is the clock.** It sends a fixed 250 Hz timestep and consumes
  exactly one `PW_STATE_OUT` per frame (a one-deep pipeline). Both matter for
  reproducibility: 1/250 divides exactly into the 50 µs tick quantum, and
  draining a variable number of replies made the client's lag vary run to run.
- **First client contact resets the sim.** Before a client attaches the core is
  idle-ticking, so how much simulated time has already elapsed depends on process
  startup. Without the reset that showed up as a different battery voltage on
  frame 0 of every run.

### Firmware statics and determinism

- **`BF::init()` does not clear the firmware's statics** — scheduler task
  stats, IMU quaternion, PID state, gyro calibration progress, RX latches and
  OSD timers all survive an in-process re-init (142 residue ranges measured).
  That is why `Sim::reset` rewinds the process's writable static sections to
  a snapshot taken right after the first boot (`core/sim/static_snapshot.h`)
  before calling `init()`. Exclusions (dyad's live connection state, the held
  dyad mutex, serial_tcp's port state — live `dyad_Stream` pointers and
  which-ports-listen flags — and the snapshot's own bookkeeping) must be
  registered BEFORE the snapshot is taken; dyad's globals live in one exported
  struct (`pw_dyad_state`, patches/dyad-globals-struct.diff) and serial_tcp
  exports `pw_serial_tcp_*_range` for exactly this.
  Debugging a determinism regression: the failing check prints the first
  divergent frame + field; `PROPWASH_DUMP_STATE` + `state_diff.py` bisect
  any residue to exact symbols.
- **The damage/contact path must not call `randf()`** — one extra draw shifts
  the whole noise stream and every trajectory after it. Wind gusts use
  SimplexNoise (a pure function of sim time + seed) for the same reason.

### Contact and damage

- **Contact response must be forces, not velocity edits.** The virtual
  accelerometer reads `total_force/mass` — a velocity edit is invisible to it,
  so the firmware would never feel touchdown or a crash. That is why the core
  resolves the client's contact manifold inside the tick (`contactForces` in
  `physics.cpp`) and why `PW_STATE_IN` velocities are ignored as of v2.
- **Resting contact needs speculative points + depth continuity.** A quad
  rocking on an asymmetric contact set never settled: separated points
  re-impacted undamped during the 4 ms client-frame gap, and the client's
  rectangle-rule position integration re-anchored depths with an O(a·dt²)
  error every frame. Fix: clients report near-contacts (≤5 mm) at depth 0
  (one-sided dampers), and the core keeps its evolved depth for persisting
  contacts unless the client disagrees by >2 mm.
- **Damage thresholds are approach SPEEDS, never forces** — a parked quad has
  v_n ≈ 0 no matter how hard it presses, so rest can't accumulate damage and
  retuning the contact spring never retunes damage.

### Betaflight build & config quirks

- **DSHOT needed a virtual ESC AND a virtual-time fix.** Stock SITL never
  compiles the dshot stack (`common_pre.h` gates `USE_DSHOT` on `!SITL`); the
  ext `target.h` wrapper re-enables it and `pw_sitl.c` provides the virtual
  ESC (`dshotPwmDevInit`: throttle 48–2047 → the same `pw_motors_pwm` sink,
  SPIN_DIRECTION commands → `pw_motor_dir[]`; hardware `dshot_dpwm.c`
  excluded, ARM-only headers shadowed in `simstubs/`). Separately,
  `motorEnable()` runs during `init()` at virtual `millis()==0` and
  `dshotStreamingCommandsAreEnabled()` reads a zero enable-stamp as "never" —
  so `ARMING_DISABLED_BOOT_GRACE_TIME` would never clear; `BF::update`
  re-stamps it on the first tick. The configured protocol now applies
  as-is (the dump's dshot600 by default); `PROPWASH_FORCE_PWM=1` restores
  the old PWM override as an escape hatch.
- **`cli.c` `pgFind()` null-guard is `#ifdef DEBUG`.** SITL undefs features
  (OSD/VTX) whose settings stay in the value table → `diff`/`dump`/`save`
  segfault. The vendored `cli.c` makes the guard unconditional.
- **Opening the CLI sets `ARMING_DISABLED_CLI`** until a clean `exit`. Load the
  tune in one instance, fly a fresh one (never touches the CLI).
- **Betaflight ignores the `#` CLI escape while the quad is ARMED.** Stock
  upstream behaviour, not a propwash quirk: `fc/tasks.c:147` passes
  `MSP_SKIP_NON_MSP_DATA` when `ARMING_FLAG(ARMED)`, so `mspEvaluateNonMspData`
  never runs and the CLI banner never comes. Raw MSP keeps working the whole
  time — the port answers, the connection is accepted, `#` simply does nothing
  — which makes this maddening to diagnose from outside. Land and disarm first,
  exactly like the real quad. (Cost a long detour: it presents as "the CLI is
  dead while a client drives the sim", because a driven core is usually a
  FLYING one.)
- **`exit` in the CLI REBOOTS the FC and discards unsaved changes.** `cliExit`
  prints "leaving CLI mode, unsaved changes lost" and calls `cliReboot()`, so a
  RAM-only `set` is thrown away on the way out. Use `save` if the change is
  meant to stick — it writes the eeprom and reboots.
- **`ARMING_DISABLED_CLI` survives the in-process reboot.** On real hardware a
  reboot clears RAM; here `BF::init()` does not clear the firmware's statics,
  so a core that has ever opened the CLI can never arm again — which is why
  CLAUDE.md's advice has been "load the tune in one instance, fly a fresh one".
  Within one process the fix is `PW_CMD_RESET`: it rewinds the statics to the
  pre-CLI snapshot, and the following `BF::init()` re-reads the eeprom, so a
  saved retune survives while the arming block clears.
- **After a reset, hold the ARM switch OFF until `arming_disable` reads 0.**
  Raising it during the post-reset gyro calibration means the switch is already
  ON when calibration completes, and Betaflight latches `flip ARM switch OFF
  then ON` and refuses forever.
- **Never block inside the client's `_physics_process`.** The core treats a
  client quiet for longer than `CLIENT_IDLE_MS` as departed and resumes
  idle-ticking (`server.cpp` `clientDriving`), injecting simulated time nobody
  asked for — a blocking CLI call let the quad fly unattended and fall out of
  the air. Client-side network work must be stepped across frames.
- **`accept()` itself fails for aborted backlog connections on Windows**
  (`WSAECONNRESET`; Linux discards them silently). Stock dyad fell through
  that failure and fabricated a CONNECTED stream around `INVALID_SOCKET` —
  the bogus ACCEPT made serial_tcp's newest-client-wins `onAccept` drop the
  *healthy* CLI/MSP connection whenever a retrying client left a corpse in
  the backlog (the old Windows-CI reconnect-after-`save` timeout). The
  vendored dyad.c skips the corpse and keeps draining; the reboot path also
  ends the old client deterministically (`tcpReconfigure`).
- **The core forces `small_angle = 180` at boot** (`bf.cpp`
  `configureDefaultModes`), on the same footing as the aux modes it already
  pins. On hardware small_angle (default 25 deg) is a safety interlock against
  arming while tilted; in a simulator there is nothing to be unsafe about, and
  without it crashflip silently cannot work — the quad lands on its back,
  refuses to arm, and the recovery grinds at full throttle. Forcing it means a
  fresh checkout with NO eeprom behaves like a baked one, instead of requiring
  `tools/bfcli/load_config.sh` before the demo works at all. NB poking
  `imuConfig()->small_angle` alone does nothing: imu.c caches it as a cosine
  (`smallAngleCosZ`) in `imuConfigure()`, so `BF::setSmallAngle` re-runs that
  to recompute the cache in place — which is what removes the old
  save-and-reboot requirement.
- **`align_board_roll = 180`** in the real diff (FC mounted inverted) must be
  zeroed for the sim — the virtual gyro is already airframe-aligned, else it
  flies upside-down. That's what `config/sitl-overrides.txt` is for.
- **Betaflight source uses `#pragma GCC poison sprintf snprintf`** unless
  `SIMULATOR_BUILD` is defined; keep that define on the BF lib.
- **There is no usable *stable* OpenSCAD.** 2021.01 is the newest stable
  release (Feb 2021) and `model/build_asset.py` cannot run on it: it needs
  `--backend=manifold` and `--export-format=obj`, and `src/export_obj.cc` does
  not exist at that tag. OpenSCAD ships via dated snapshots, which pin cleanly
  because files.openscad.org keeps them forever —
  `model/OPENSCAD_VERSION` holds the pinned URL + sha256 that CI downloads.
- **The airframe's duct rings are supposed to overlap, so don't widen the
  wheelbase.** Each duct's outer radius is 51.55 mm while adjacent duct centres
  sit 100.41 mm apart — the 2.69 mm overlap is what fuses the four rings into
  the two molded halves, and `prop_guards()` cuts its tooling-relief slots
  through that tangency. Raising `wheelbase` past ~146 mm separates them and
  the cage becomes four floating hoops. If the physics ever needs a wider quad,
  scale the model uniformly at the Godot node instead — `main.gd` already
  derives that scale and warns when it is not 1.0.
- **The 142 mm wheelbase lives in three places and they must agree**:
  `model/cinelog35_v3.scad` (`wheelbase = 142`), `quadMotorPos` in
  `core/sim/profile_cinelog35.h`, and `MOTOR_OFFSETS` in `main.gd` — plus
  `tools/sysid/profiles.py`. All are ±50.205 mm (142/√2/2). `main.gd` warns at
  load if the model and the profile diverge.
- **`PW_HULL_*` is NOT the wheelbase**, even though `PW_HULL_DUCT_XZ` (0.054)
  sits near it. The five spheres are a tuned approximation of a ducted cage's
  *contact behaviour*: a 30 mm sphere already under-fits a 51.55 mm duct ring,
  so the offset is a contact parameter, not a dimension. Re-deriving it from the
  real airframe (0.0718, which puts the spheres' outer edge on the true 203.5 mm
  guard span) widens the stance enough to break `crash_scenario` and
  `turtle_flip`. Retune the hull as a whole or leave it alone. `codec_parity`
  now compares `HULL` between the two Python codecs so the copies can't drift.
- **Collision never touches the drone mesh.** Contacts come from the analytic
  `HULL_SPHERES` plus engine shape queries, so swapping the visual model is
  physics- and determinism-neutral. Conversely, a prettier model buys you
  nothing in collision fidelity.
- **`CAM_POS` is coupled to the airframe geometry and is easy to knock out of
  frame.** The quad is 22 mm tall seen through a 105 deg lens, so a few mm of
  camera height decides whether the front ducts hold the bottom of the feed or
  the airframe disappears entirely — seating the model on the pad and shrinking
  the wheelbase each did it once. Pushing the camera forward toward the real O3
  nacelle position is worse, not better: past ~-0.03 it clears the front duct
  and the guards leave the frustum sideways. **Layer bits are not visibility** —
  every cull-mask assertion passed with the drone completely off screen, which
  is why `fpv_cull_test` now projects the hull into the FPV camera and requires
  it in the lower band.
- **Nothing headless ever compiles a shader.** `_build_goggle_layer` returns
  early under `--headless`/autotest, so the goggle pass — and therefore
  `goggle.gdshader` — is never built or parsed by the normal test suite. A
  broken shader is silent there and catastrophic on a real renderer: it fails
  the whole shader, the effect vanishes, and Godot emits
  `version_get_shader: Parameter "version" is null` once per frame forever.
  The `shader_compile` ctest parses every shader explicitly for this reason.
  Also note Godot FORBIDS `return` inside a processor function
  (`vertex`/`fragment`/`light`), and processor built-ins like `TEXTURE` are not
  visible inside plain functions — pass the sampler in as a parameter.
- **The drone "trembling" in 3rd person was the PROPELLERS, not the airframe.**
  Measured, after two wrong guesses. `_spin_props` advanced the blade angle
  inside `_physics_process`, i.e. once per physics tick. At 250 Hz physics into
  60 Hz render each frame consumes 4 or 5 steps in a fixed 5:1 ratio (measured
  `4:100 5:20`), so the blades advanced 25% further every sixth frame. On a
  steady hover with constant rpm that measured as **8.8% wobble in rendered
  blade angular rate (4381 +/- 386 deg/s)**, against **0.00 px/frame^2 screen
  jerk for the airframe body, the ducts AND the static world** — the body was
  never trembling at all. Driving the spin on the RENDER frame with the render
  delta takes it to **0.5% (4381 +/- 14)**. Prop angle is a pure visual;
  nothing reads `_prop_angles`, so it belongs on the render clock.
  Consequently the prop nodes are `PHYSICS_INTERPOLATION_MODE_OFF`: they are
  now written every render frame, and interpolating a value that is already
  render-rate reintroduces the wobble.
  WHY IT MISLEADS: spinning blades are the most eye-catching part of the
  airframe, so their stutter reads as "the whole drone is vibrating", and since
  nothing else in the scene is driven by the physics tick, "only the drone"
  looks like evidence about the flight controller. It is not. Two earlier
  hypotheses (a controller limit cycle, then camera interpolation) were both
  wrong; the measurement is what settled it.
- **Never move a physics-interpolated node from `_process`.** Godot renders an
  interpolated node between its last two PHYSICS-frame transforms, so a node
  written every render frame is interpolated between values that are already
  render-rate. The director's chase/LOS cameras and the prop nodes are
  therefore `PHYSICS_INTERPOLATION_MODE_OFF`. The FPV camera is NOT — it is a
  child of the drone, written on the physics frame with it.
- **Aim a follow camera at the DRAWN position, not at `_pos`.** The drone is
  physics-interpolated, so its rendered origin lags the raw physics position by
  a varying fraction of a step — measured 0.8 +/- 1.3 mm, peaking at 4.5 mm
  under acceleration and exactly zero at constant velocity. Use
  `get_global_transform_interpolated()`. Small, but it lands on the drone and
  on nothing else, because no other object has an interpolation gap.
- **`PROPWASH_JITTER_LOG=1` is the tool for all of this.** It reports screen
  jerk (px/frame^2) for the drone body, a point offset on a duct (so rotation
  shows up, which a centre-only probe misses) and a STATIC WORLD point through
  the same camera — plus the blade's rendered angular rate as median +/- MAD.
  Two lessons in the metric itself: a point on a spinning rim is useless as a
  jitter probe because its screen second-difference is dominated by centripetal
  acceleration, and SD alone cannot tell one hitched frame from a systematic
  wobble, which is why MAD is reported alongside it.
- **A trunk is a CYLINDER, not a capsule.** Tree colliders used
  `CapsuleShape3D`, whose bottom hemisphere tapers to a point at ground level —
  so the collider shrank to nothing exactly where a landed or crashed quad
  sits. Measured: the hull reached 5 cm INSIDE the drawn trunk at y = 0.02 m
  while contact at y = 2 m was correct. That is how a quad ends up visually
  wedged into a tree the simulator never reported hitting. `tree_collider`
  gates it by sweeping the hull into a known trunk at seven heights. NB gate
  TUBES legitimately stay capsules — their rounded caps overreach along the
  tube axis only, outside the flyable opening.
- **A demo act that assumes clear space must CHECK for clear space.** The
  turtle act drops the quad on its back and levers it over its own duct edge,
  which is impossible against scenery — and its fly-away target was
  (0, 2.5, -6.0), which is gate 1 with its top bar at 2.05 m. It had been
  flying into that gate on every run. Nothing failed, because the act only
  asserted "did it come upright / did it re-arm", and it did. Damage is no help
  as a signal either: a clean turtle run reads max_dmg 0.46, because the act
  crashes the quad ON PURPOSE. The act now flags any contact whose surface is
  not SURF_GROUND and names it, which found the gate immediately.
- **Godot's headless viewport is 64 px tall**, so anything asserting on screen
  *fractions* is coarse there; assert on projected corner COUNTS instead. And
  because the client runs `physics_interpolation=true`, moving a camera from a
  script and immediately calling `unproject_position` reads the stale
  interpolated transform — the reason `_repair_in_place` calls
  `reset_physics_interpolation()`. Probe camera poses by changing `CAM_POS` and
  rebuilding the scene, not by nudging the node at runtime.

## Diagnostics

- `[pw][rc]` log line (server) prints the 8 RC channels + a decoded arming-block
  reason on every change — the tool for setting up switches / debugging "won't
  arm".
- `[pw][js]` shows joystick detection + calibration load.
- `PROPWASH_CONTACT_LOG=1` (client) prints every new contact event —
  `[pw][contact] t=… n=… surface=… depth=…` — the tool for "did I actually
  touch that gate" and for verifying a course change keeps the demo clear.
- MSP identity / live config: `python3 tools/bfcli/pw_cli.py run "version"`.
- Determinism regression: the failing check prints the first divergent frame +
  field; `PROPWASH_DUMP_STATE=<path>` (core env) + `tools/tester/state_diff.py`
  bisect residual static state to exact symbols.

## Conventions

- C++17 for the core (links Betaflight C directly — Rust would need an FFI wall
  for zero gain). GDScript for the client. Python (stdlib) for tools/tests.
- Match the surrounding style. Keep `protocol/propwash_protocol.h` free of any
  Betaflight include — it is the GPL/MIT boundary.
- A wire-format change touches exactly **five** codecs: `propwash_protocol.h`
  (+ the `static_assert` sizes in `server.cpp`), `server.cpp`,
  `tools/tester/pw_udp.py` (the GPL test codec — all tools/tester + tools/sysid
  import it, don't re-inline struct strings), `client-godot/scripts/protocol.gd`,
  and `python/propwash_gym/src/propwash_gym/protocol.py` (the gym's own MIT copy
  — it can't import the GPL test codec). Bump `PW_VERSION`; mismatches are
  dropped on both sides. The `codec_parity` ctest compares the two Python
  codecs field-for-field, so changing one alone goes red instead of silently
  breaking the gym.
- When you modify a vendored stock Betaflight file, update its recorded diff in
  `extern/betaflightext/patches/`.
- Commit or push only when asked. End commit messages with the Co-Authored-By
  trailer.
