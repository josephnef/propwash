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
ctest --test-dir build            # 9 headless tests; ~2 min
```

The Betaflight submodule builds as a static lib (`extern/`). Source list is
harvested from an authoritative `make TARGET=SITL` build into
`extern/bf_sources.txt` — don't hand-edit it; regenerate from the object list
if the firmware version changes.

## The tests are the contract (run them after any core change)

| ctest | proves |
|-------|--------|
| `headless_scenario` | BF boots, MSP identity = `BTFL 4.5.2`, arms, angle-hover; prints `STATE_HASH` (printed, not asserted) |
| `udp_e2e` | Python client drives the wire protocol, hovers |
| `diff_roundtrip` | apply a `diff all` over CLI → save → in-process reboot → settings survive |
| `osd_render` | real `osd.c` renders through the fake displayport → `PW_OSD` |
| `real_tune_hover` | bake the real diff → fresh boot has `p_pitch=53`, `align_board_roll=0` → hovers |
| `godot_client` | the actual GDScript client arms + hovers headless (`--headless`, autotest) |
| `flythrough` | autonomous angle-mode gate run |
| `quality_tiers` | render tier selection rule as a pure function (cases a single machine can't produce) |
| `fpv_cull` | FPV cull mask + prop→motor index map — invariants a rewrite has already silently dropped once |

`godot_client`/`flythrough` only run if a `godot` binary is found (see
`find_program(GODOT_BIN ...)` in `CMakeLists.txt`). Tests that bind ports must
not overlap — leftover `propwash-core` processes cause spurious failures; kill
strays before running (`pkill -f build/propwash-core`).

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
  lockstep, owns collision, converts sim↔Godot handedness (mirror z; quat -x,-y;
  angular velocity is a pseudovector: -x,-y,+z).

## Non-obvious things that will bite you (learned the hard way)

- **Tick rate must exceed the PID rate.** At exactly 8 kHz, RX/MSP never run
  (scheduler only services them between gyro boundaries) → `rcData` frozen, no
  arming. 20 kHz fixes it.
- **DSHOT blocks arming in SITL.** `ARMING_DISABLED_BOOT_GRACE_TIME` never
  clears because DSHOT streaming commands aren't available on the fake motor
  device. `targetPreInit()` forces `motor_pwm_protocol = PWM` every boot.
- **`cli.c` `pgFind()` null-guard is `#ifdef DEBUG`.** SITL undefs features
  (OSD/VTX) whose settings stay in the value table → `diff`/`dump`/`save`
  segfault. The vendored `cli.c` makes the guard unconditional.
- **Opening the CLI sets `ARMING_DISABLED_CLI`** until a clean `exit`. Load the
  tune in one instance, fly a fresh one (never touches the CLI).
- **`align_board_roll = 180`** in the real diff (FC mounted inverted) must be
  zeroed for the sim — the virtual gyro is already airframe-aligned, else it
  flies upside-down. That's what `config/sitl-overrides.txt` is for.
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
- **A physically-connected joystick has RC priority** and will hijack the
  headless tests — autotest/demo spawn the core with `--no-js`.
- **Betaflight source uses `#pragma GCC poison sprintf snprintf`** unless
  `SIMULATOR_BUILD` is defined; keep that define on the BF lib.

## Diagnostics

- `[pw][rc]` log line (server) prints the 8 RC channels + a decoded arming-block
  reason on every change — the tool for setting up switches / debugging "won't
  arm".
- `[pw][js]` shows joystick detection + calibration load.
- MSP identity / live config: `python3 tools/bfcli/pw_cli.py run "version"`.

## Conventions

- C++17 for the core (links Betaflight C directly — Rust would need an FFI wall
  for zero gain). GDScript for the client. Python (stdlib) for tools/tests.
- Match the surrounding style. Keep `protocol/propwash_protocol.h` free of any
  Betaflight include — it is the GPL/MIT boundary.
- When you modify a vendored stock Betaflight file, update its recorded diff in
  `extern/betaflightext/patches/`.
- Commit or push only when asked. End commit messages with the Co-Authored-By
  trailer.
