# propwash

An open-source FPV drone simulator that runs **real Betaflight firmware in the loop**.

Unlike commercial sims (VelociDrone, Liftoff), propwash does not approximate the
flight controller — it compiles the pilot's actual Betaflight version (pinned:
**4.5.2**, matching a GEPRC CineLog35 V3) into the simulator process and runs the
real scheduler, PID loops, rates, arming logic, and failsafe against a physics
model. Load your literal `diff all` and the sim flies with *your* tune.

## Architecture

```
┌────────────────────────────────────────────────────────────┐
│ propwash-core  (C++, GPL-3.0, standalone executable)       │
│  ┌──────────────┐   in-process    ┌────────────────────┐   │
│  │ physics 8kHz │ ◄─ motor PWM ── │ Betaflight 4.5.2   │   │
│  │ (rigid body, │ ── gyro/accel ► │ (static lib, real  │   │
│  │  motors, bat)│                 │  scheduler ticks)  │   │
│  └──────┬───────┘                 └─────────┬──────────┘   │
│         │ UDP protocol (versioned)          │ TCP 5761+    │
└─────────┼───────────────────────────────────┼──────────────┘
          │                                   │
   Godot 4 client / Quest 3 /          Betaflight Configurator
   Python gym env / bbreplay                (MSP/CLI)
```

- **Deterministic lockstep**: simulated time advances only with sim ticks —
  identical inputs produce identical trajectories (required for RL and for
  replaying real blackbox logs).
- **Process boundary = license boundary**: the core is GPL-3.0 (contains
  Betaflight); clients speak a documented UDP protocol (`protocol/`, MIT) and
  carry no GPL code.
- **Quest 3 / OpenXR path**: the core stays on a PC; a standalone headset is
  just another UDP client over WiFi.

## Layout

| Path | License | What |
|------|---------|------|
| `core/` | GPL-3.0 | propwash-core executable: sim loop, physics, BF glue, UDP server |
| `extern/betaflight/` | GPL-3.0 | Betaflight submodule pinned @ 4.5.2 |
| `extern/betaflightext/` | GPL-3.0 | Override/shim layer (custom SITL-style target) + recorded patches |
| `tools/tester/` | GPL-3.0 | Headless integration test: boot, MSP identity, arm, hover, determinism |
| `tools/bbreplay/` | GPL-3.0 | Blackbox log replay for sim-vs-real validation (planned) |
| `protocol/` | MIT | Wire protocol: single header, no Betaflight includes |
| `client-godot/` | MIT | Godot 4 frontend (planned) |
| `python/` | MIT | `propwash_gym` gymnasium env (planned) |

## Build & fly

```bash
git clone --recursive <repo>
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build              # headless hover + UDP end-to-end
```

Requires: CMake ≥ 3.20, GCC/Clang with C11 + C++17, Linux (x86_64).

**Fly it** (Godot 4.3+, e.g. `pacman -S godot`):

```bash
godot --path client-godot           # spawns build/propwash-core itself
```

- With a RadioMaster Pocket (EdgeTX USB Joystick mode) plugged in, the core
  reads it directly (auto-detected by name) — ARM ch5, ANGLE ch6, same
  switch plan as the real quad.
- Without a radio: keyboard mode — arrows = right stick, W/S throttle,
  A/D yaw, E arm, Q angle toggle, R reset.
- Betaflight Configurator connects any time to TCP `127.0.0.1:5761`.

Core standalone: `./build/propwash-core [--server|--realtime] [--port 9100]
[--eeprom path] [--js /dev/input/jsN | --no-js]`.

## Why C++ / why in-process

The core links Betaflight's C internals directly (`rxRuntimeState`,
`micros_passed`, `scheduler()`) — the SimITL approach — because it is the only
way to get deterministic time. Stock Betaflight SITL (UDP 9002/9003/9004)
free-runs on wall clock scaled by packet arrival; that is fine for interactive
flying and useless for reproducible RL rollouts.

## Prior art & credits

- [SimITL](https://github.com/AJ92/SimITL) (GPL-3.0) — in-process BF + physics model this project ports
- [KwadSim / KwadSimServer](https://github.com/timower/KwadSim) (GPL-3.0) — the restartable-server pattern
- [Flightmare](https://github.com/uzh-rpg/flightmare) (MIT) — physics/render decoupling, gym design
- [Betaloop](https://github.com/Aeroloop/betaloop) — MSP virtual radio RC path
