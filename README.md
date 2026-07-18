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
   behaviour as the real quad.
2. **Deterministic lockstep** — identical inputs produce byte-identical
   trajectories, so it can back a reproducible RL gym and replay real blackbox
   logs for sim-vs-real validation.
3. **Configurator-compatible** — the real Betaflight Configurator connects over
   TCP and tunes it live, exactly like a bench quad.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ propwash-core   (C++, GPL-3.0, standalone executable)       │
│  ┌───────────────┐   in-process    ┌─────────────────────┐  │
│  │ physics 20kHz │ ── gyro/accel ► │ Betaflight 4.5.2    │  │
│  │ (rigid body,  │ ◄─ motor PWM ── │ (static lib: real   │  │
│  │  motors, bat) │                 │  scheduler + PIDs)  │  │
│  └──────┬────────┘                 └──────────┬──────────┘  │
│         │  UDP protocol (versioned)           │ TCP 5761    │
└─────────┼──────────────────────────────────── ┼────────────┘
          │                                     │
   Godot 4 client / Quest 3 (planned) /   Betaflight Configurator
   Python gym (planned) / bbreplay (planned)   (MSP / CLI)
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
| `tools/tester/` | GPL-3.0 | Headless tests: boot/MSP identity, hover, determinism, OSD, real-tune |
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

Install Godot 4.3+ (`pacman -S godot`, `apt install godot`, or grab the
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
  `E` arm, `Q` angle toggle, `R` reset.
- **Betaflight Configurator** connects any time to TCP `127.0.0.1:5761` (MSP/CLI)
  and tunes it live.
- The **real Betaflight OSD** is overlaid on the FPV view.
- **Second monitor** — if one is attached the sim opens fullscreen on it, leaving
  the primary free for the Configurator and logs. Override with
  `PROPWASH_SCREEN=off` (stay windowed) or `PROPWASH_SCREEN=<index>` (0-based).

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
                      [--eeprom path] [--js /dev/input/jsN | --no-js]
```

## Status

Working: in-process Betaflight 4.5.2, deterministic lockstep, physics + stable
hover, UDP protocol, Godot FPV client with the real OSD and a cinewhoop model,
CLI/Configurator data path, real-tune loading, joystick calibration, autonomous
gate fly-through. **7 headless self-tests** cover boot/MSP identity, hover,
determinism, OSD render, real-tune hover, and the Godot client + fly-through.

Planned: Python gym env (RL), blackbox replay (sim-vs-real system ID), Quest 3 /
OpenXR build.

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
