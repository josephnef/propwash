# propwash

**An open-source FPV drone simulator that runs *real Betaflight firmware* in the loop.**

Unlike commercial sims (VelociDrone, Liftoff), propwash doesn't approximate the
flight controller — it compiles the pilot's actual Betaflight firmware (pinned
**4.5.2**) into the simulator process and runs the real scheduler, PID loops,
rates, arming logic, failsafe, and OSD against a physics model. Load your literal
`diff all` off the quad and the sim flies with **your** tune.

Built around a GEPRC **CineLog35 V3** (3.5″ ducted cinewhoop, F722), but the
pairing is generic to any serial-ELRS + Betaflight setup.

![disarmed on the pad](docs/demo-disarmed.png)
![hovering with the real tune](docs/demo-hover-real-tune.png)

*FPV view from where the DJI O3 sits — front ducts and props in frame, the real
Betaflight OSD overlaid, live motor RPM in the HUD.*

## Why

Every other sim reimplements or approximates the flight controller. propwash
runs the genuine article, which makes it uniquely suited to three things a
normal sim can't do:

1. **Train on your exact tune** — same PIDs, rates, filters, arming/failsafe
   behaviour as the real quad — including DSHOT600, crashflip (turtle mode)
   and the RPM filter running on bidirectional-DSHOT eRPM that the virtual
   ESC derives from the physics' true motor speeds.
2. **Deterministic lockstep** — identical inputs produce byte-identical
   trajectories, so it can back a reproducible RL gym and replay real blackbox
   logs for sim-vs-real validation.
3. **Configurator-compatible** — the real Betaflight Configurator connects over
   TCP and tunes it live, exactly like a bench quad.
4. **Crashes are real** — contacts are resolved as forces inside the physics
   tick, so the firmware *feels* impacts on its virtual gyro/accel exactly like
   real hardware (its own crash detection works, straight from your dump).
   Gates and trees are solid, impact speed maps to per-motor prop damage, a
   hard crash grounds you because damaged props can't lift the quad — and you
   fix it (`T`) or walk back to the pad (`R`), not respawn through a menu.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ propwash-core   (C++, GPL-3.0, standalone executable)       │
│  ┌───────────────┐   in-process    ┌─────────────────────┐  │
│  │ physics 20kHz │ ── gyro/accel ► │ Betaflight 4.5.2    │  │
│  │ (rigid body,  │ ◄─ motor PWM ── │ (static lib: real   │  │
│  │  motors, bat, │                 │  scheduler + PIDs)  │  │
│  │  contacts,    │                 └──────────┬──────────┘  │
│  │  damage, wind)│                            │ TCP 5761    │
│  └──────┬────────┘                            │             │
│         │  UDP protocol (versioned):          │             │
│         │  pose + contact manifold in,        │             │
└─────────┼  state + damage out ─────────────── ┼────────────┘
          │                                     │
   Godot 4 client / Quest 3 (planned) /   Betaflight Configurator
   Python gym / blackbox replay + sysid        (MSP / CLI)
```

- **In-process, not networked SITL.** The core links Betaflight's C internals
  directly (the [SimITL](https://github.com/AJ92/SimITL) approach) and drives
  its scheduler tick-by-tick, so simulated time only advances with sim steps.
  That determinism is the whole point; stock Betaflight SITL free-runs on wall
  clock and can't give reproducible rollouts.
- **Process boundary = license boundary.** The core is GPL-3.0 (it contains
  Betaflight); every client speaks a documented UDP protocol (`protocol/`, MIT)
  and carries no GPL code.
- **The tick rate (20 kHz) exceeds the gyro/PID rate (8 kHz)** on purpose — the
  scheduler only services non-realtime tasks (RX, MSP) between gyro boundaries.
- **The client senses collisions, the core solves them.** The client owns world
  geometry and position: it tests a shared 5-sphere hull against the world each
  frame and sends the contact manifold (point, normal, depth, surface). The
  core resolves contacts as spring-damper *forces* inside the physics tick —
  never velocity edits, which the virtual accelerometer cannot see — so
  touchdown, ground rest and crashes reach the firmware the way real sensors
  would. Impact speed then drives a deterministic damage model (prop damage,
  prop strikes, structural crashes), and wind is a pure function of sim time +
  seed, so reproducibility survives all of it.

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the details, including the
two lockstep scheduler patches and the OSD/CLI plumbing.

## Layout

| Path | License | What |
|------|---------|------|
| `core/` | GPL-3.0 | propwash-core: sim loop, physics, Betaflight glue, UDP server, joystick |
| `extern/betaflight/` | GPL-3.0 | Betaflight submodule, pinned @ tag `4.5.2` |
| `extern/betaflightext/` | GPL-3.0 | Override layer (SITL-derived target, patched scheduler/cli, fake OSD displayport) + recorded diffs in `patches/` |
| `protocol/` | MIT | Wire protocol — single header, no Betaflight includes (the boundary) |
| `client-godot/` | MIT | Godot 4 frontend: FPV cinewhoop, OSD overlay, joystick/keyboard |
| `python/propwash_gym/` | MIT | Gymnasium env (RL): `uv`-managed, subprocess-per-env, `frame_id` lockstep |
| `tools/tester/` | GPL-3.0 | Headless tests: boot/MSP identity, hover, determinism, OSD, real-tune |
| `tools/sysid/` | GPL-3.0 | Blackbox replay + system ID: fit the physics to a real log (RC & physics-only replay) |
| `tools/bfcli/` | MIT-ish | CLI-over-TCP: apply the pilot's `diff all`, bake eeprom, calibrate joystick |
| `config/` | — | The real `cinelog35v3.diff` + SITL overrides |
| `docs/` | — | Architecture, licensing, screenshots |

## Build

No external dependencies — just a C/C++ toolchain and CMake. (The joystick uses
the Linux kernel `js` API directly; no SDL2.)

```bash
git clone --recursive https://github.com/<you>/propwash
cd propwash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build            # headless: boot, hover, determinism, OSD, real-tune
```

Requires: CMake ≥ 3.20, GCC ≥ 12 or Clang, Linux x86_64. (Builds clean on
GCC 16; the pinned Betaflight needs a couple of `-Wno-error` for newer GCC,
already handled in `extern/CMakeLists.txt`.)

If you cloned without `--recursive`: `git submodule update --init --recursive`.

## Fly it

Install Godot 4.7+ (`pacman -S godot`, `apt install godot`, or grab the
[official binary](https://godotengine.org/download)), then:

```bash
# one-time: bake your tune into an eeprom the client will use
tools/bfcli/load_config.sh                 # writes ./eeprom.bin

PROPWASH_EEPROM=$PWD/eeprom.bin godot --path client-godot
```

The Godot client spawns `build/propwash-core` itself. Controls:

- **RadioMaster Pocket / any EdgeTX handset** in USB Joystick mode — auto-detected
  by name, RC read directly by the core. Calibrate once:
  `./build/propwash-core --js-calibrate`.
- **No radio** — keyboard: arrows = right stick, `W`/`S` throttle, `A`/`D` yaw,
  `E` arm, `Q` angle toggle, `F` turtle switch, `T` repair in place, `R` reset
  to pad.
- **Crashes have consequences.** Gates, trees and the ground are solid; hitting
  them costs momentum and props. The HUD shows per-motor damage, a hard impact
  puts up a `CRASHED` banner, and a wrecked quad genuinely cannot hover (the
  thrust isn't there any more). Disarm and press `T` to repair + set the quad
  upright where it lies, or `R` to reset to the pad. `PROPWASH_STRICT=1`
  disables `T` for deliberate practice — a crash then always ends the flight.
  If your dump sets `crash_recovery = DISARM`, the firmware's own crash
  detection disarms you exactly as on hardware (`CRASH DETECTED` banner; cycle
  the ARM switch to clear). And when you end up upside-down: disarm, flip the
  turtle switch (`F`), arm, and roll — the props reverse over real DSHOT
  spin-direction commands and the quad pivots itself upright over a duct
  edge, exactly the crashflip maneuver from the real quad.
- **Wind** — `PROPWASH_WIND="3,0,0" PROPWASH_GUST=1.5 godot --path client-godot`
  gives a 3 m/s steady wind with 1.5 m/s gusts. Deterministic: the same flags
  reproduce the same run; calm runs are byte-identical to a build without wind.
- **Betaflight Configurator** connects any time to TCP `127.0.0.1:5761` (MSP/CLI)
  and tunes it live.
- The **real Betaflight OSD** is overlaid on the FPV view — crisp, above the
  camera-feed pass, because on a real DJI system the goggles draw it from
  MSP-DisplayPort data rather than it being encoded into the video.
- **DJI O3 feed treatment** — ISP sharpening halos, gentle digital contrast,
  mild corner falloff, a daylight sensor floor, and codec softening driven from
  real angular rate (a whip-pan blows the bitrate budget and recovers). Tuned to
  be subtle: real O3 footage is clean, so if you can point at an individual
  effect it is turned up too far. `PROPWASH_GOGGLE=off` shows the raw render.
- **Second monitor** — if one is attached the sim opens fullscreen on it, leaving
  the primary free for the Configurator and logs. Override with
  `PROPWASH_SCREEN=off` (stay windowed) or `PROPWASH_SCREEN=<index>` (0-based).
- **Lockstep rate** — the client drives the core at a fixed **250 Hz**, one packet
  per step, and consumes exactly one reply per frame. It is deliberately not tied
  to your monitor: the rate is an input to the simulation, so varying it makes
  runs unreproducible. 1/250 also divides exactly into the core's 50 µs tick
  quantum. Display smoothness comes from physics interpolation instead.
- **Quality tiers** — `low`/`medium`/`high`, auto-selected from what the GPU
  actually has to sustain (width × height × refresh), so a 240 Hz panel keeps its
  framerate and a 60 Hz one gets the prettier version. **Every tier renders at
  native resolution**; tiers differ in scene and lighting cost, never in pixels.
  Override with `PROPWASH_QUALITY=<tier>`. If render rate falls far below the
  lockstep rate for a few seconds, the client gives the tick rate back rather
  than let a heavy scene starve the sim. `PROPWASH_SCALE=<0.5-1.0>` exists as an
  explicit opt-in for a GPU that genuinely can't drive the panel — it is never
  applied by default.

### Environment variables

Everything the client understands, in one place — several of these were only
discoverable by reading the source.

| variable | effect |
|---|---|
| `PROPWASH_EEPROM=<path>` | eeprom the client's core boots from (your baked tune) |
| `PROPWASH_CORE=<path>` | propwash-core binary to spawn; defaults to `../build/propwash-core` |
| `PROPWASH_DEMO=acro` | autonomous fly-through of all three gates, prints `[demo] PASS` |
| `PROPWASH_AUTOTEST=1` | headless arm + hover self-test, exits 0/1 (what CI runs) |
| `PROPWASH_SHOTS=<dir>` | save PNG frames at fixed times during a demo run |
| `PROPWASH_QUALITY=low\|medium\|high` | override the auto-selected render tier |
| `PROPWASH_SCREEN=off\|<index>` | stay windowed, or force a monitor |
| `PROPWASH_GOGGLE=off` | disable the O3 feed treatment, show the raw render |
| `PROPWASH_SCALE=<0.5-1.0>` | render 3D below native and upscale; off by default |
| `PROPWASH_JS_MAP="0,1,2,..."` | remap RC channel → joystick axis |
| `PROPWASH_JS_INVERT="2"` | comma list of RC channels to negate |
| `PROPWASH_STRICT=1` | disable `T` repair — a crash always ends the flight |
| `PROPWASH_WIND="x,y,z"` | steady wind in m/s, forwarded to the core |
| `PROPWASH_GUST=<amp>` | gust amplitude in m/s on top of the steady wind |
| `PROPWASH_PORT=<port>` | core UDP port; lets tests and a live session coexist |
| `PROPWASH_NO_JS=1` | spawn the core with `--no-js` (scripted harnesses) |
| `PROPWASH_CONTACT_LOG=1` | print every new contact event (surface, depth) |
| `PROPWASH_FORCE_PWM=1` | (core env) force the old PWM motor protocol instead of the dump's DSHOT |

### Loading the pilot's real tune

`config/cinelog35v3.diff` is a real `diff all` pulled off the FC. Refresh it any
time the quad is on USB, then re-bake:

```bash
# pull from the FC (see the cinelog35-v3 bring-up repo for scripts/bf.py)
python3 scripts/bf.py /dev/ttyACM0 cli "diff all" > config/cinelog35v3.diff
tools/bfcli/load_config.sh
```

`config/sitl-overrides.txt` neutralises settings that describe the *physical*
board and must not apply to a virtual gyro — critically `align_board_roll`
(the real FC is mounted inverted; without this the sim flies upside-down).

### Core standalone

```bash
./build/propwash-core [--server|--realtime|--js-calibrate] [--port 9100] \
                      [--eeprom path] [--js /dev/input/jsN | --no-js] \
                      [--wind x,y,z] [--gust amp]
```

## Reinforcement learning (gym)

[`python/propwash_gym/`](python/propwash_gym/) is a
[Gymnasium](https://gymnasium.farama.org/) environment: each env owns one
`propwash-core` subprocess and drives it in `frame_id` lockstep, so the policy
trains against the pilot's real firmware and the rollouts are reproducible.

```bash
cd python/propwash_gym
uv sync                      # base env; `uv sync --extra rl` adds SB3 + torch
uv run pytest                # env-checker + arm/step/hover against a real core
uv run python examples/ppo_hover.py --timesteps 20000   # PPO smoke run
```

The env task is a hover (arm → hold altitude, level and still). Action is
`[throttle, roll, pitch, yaw]`; observation is
`quat | angvel | linvel | pos_error | motor_rpm`. See its
[README](python/propwash_gym/README.md) for the spaces, reward and the
determinism caveat.

## Matching the real quad (blackbox system ID)

[`tools/sysid/`](tools/sysid/) replays a Betaflight blackbox log through the sim
and fits the physics model to it, so the sim flies like *your* CineLog35 — the
step that certifies it for sim-vs-real work. Two replay modes:

- **RC replay** drives the firmware with the log's stick inputs (firmware +
  physics, end-to-end).
- **Physics-only replay** (`PW_MOTOR_IN`) feeds the log's recorded motor outputs
  straight into the physics, bypassing the firmware — isolating physics-model
  error from PID error, and **bit-reproducible across cores** (the firmware
  residual state that limits closed-loop determinism is out of the loop).

A derivative-free fitter then adjusts `PW_INIT` physics parameters to minimise
the gyro+accel error against the log. `PW_INIT` re-parameterises physics on a
live core, so a fit evaluates hundreds of candidates against one process.

Until the real quad has flown (it's pre-first-hover), the pipeline is validated
on synthetic references: replay is bit-exact, and the fitter recovers a known
parameter it was hidden from. A real log drops in through
`bblog.import_betaflight_csv` with no code changes.

## Status

Working: in-process Betaflight 4.5.2, deterministic lockstep, physics + stable
hover, real collision physics (solid world, contact forces the firmware feels,
per-motor crash damage, repair/reset flow, wind + gusts, ground effect), UDP
protocol, Godot FPV client with the real OSD and a cinewhoop model,
CLI/Configurator data path, real-tune loading, joystick calibration, autonomous
gate fly-through, a **[Gymnasium environment](python/propwash_gym/)** (RL,
`uv`-managed, subprocess-per-env, `frame_id` lockstep), and a
**[blackbox replay + system-ID pipeline](tools/sysid/)** (RC & physics-only
replay, physics-parameter fitting over `PW_INIT`). **16 headless self-tests**
cover boot/MSP identity, hover, contact settling (level *and* inverted), the
crash→repair lifecycle, damage over the wire, OSD render, real-tune hover, the
Godot client's detection, repair flow and gate fly-through (now with clearance
asserts against solid gates), the gym env-checker + hover, and physics-only
replay reproducibility + parameter recovery — all with the real core in the loop.

Planned: Quest 3 / OpenXR build. Physics-only replay is already bit-reproducible
across cores; extending that to the closed loop (the last gap for a fully
deterministic gym across resets) tracks the firmware residual-state work.

## Prior art & credits

- [SimITL](https://github.com/AJ92/SimITL) (GPL-3.0) — in-process BF + the physics model this ports
- [KwadSim / KwadSimServer](https://github.com/timower/KwadSim) (GPL-3.0) — the restartable-server pattern
- [Flightmare](https://github.com/uzh-rpg/flightmare) (MIT) — physics/render decoupling, gym design
- [Betaloop](https://github.com/Aeroloop/betaloop) — MSP virtual-radio RC path
- [Betaflight](https://github.com/betaflight/betaflight) (GPL-3.0) — the firmware that actually flies it

## License

The core and everything that links Betaflight are **GPL-3.0**. `protocol/`,
`client-godot/`, and `python/` are **MIT** — they only speak the socket
protocol. See [`docs/LICENSING.md`](docs/LICENSING.md).
