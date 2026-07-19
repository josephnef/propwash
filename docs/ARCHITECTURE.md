# Architecture

## One core, many clients

`propwash-core` is a standalone GPL-3.0 executable. Betaflight 4.5.2 is
compiled **in-process as a static library** (`extern/`, the SimITL pattern)
and driven in deterministic lockstep. Clients (Godot frontend, Quest 3,
Python gym, blackbox replay) speak the versioned UDP protocol in
`protocol/propwash_protocol.h` (M2+) — the KwadSimServer pattern, which is
also the GPL license boundary and the Quest 3 network boundary.

## The lockstep loop

Simulated time (`pw_micros_passed`) advances **only** inside
`Sim::update()`, in fixed 50 µs ticks (20 kHz):

```
per tick:  physics.updateGyro(dt)        # gyro/accel + noise -> BF virtual sensors
           BF::update(DELTA, state)      # advance time, run BF::scheduler()
           physics.updatePhysics(dt)     # motors PWM -> forces -> integrate
```

No wall clock reaches the firmware (`extern/betaflightext/.../pw_sitl.c`
routes `micros()/millis()/delay()` to the counter). Identical inputs =>
identical trajectories, verified through the UDP protocol by the
`determinism_udp` test: two fresh cores fed the same input sequence produce
byte-identical output, including when one has sends delayed past the core's
recv timeout. `pw-tester` additionally prints a `STATE_HASH` for the in-process
path.

This holds only in lockstep mode (the default), where simulated time advances
solely on `PW_STATE_IN`. `PW_CMD_REALTIME` lets the core self-tick on the wall
clock and forfeits reproducibility. Note also that traffic on TCP 5761
(Configurator/MSP) is delivered by a wall-clock-driven thread, so a determinism
run must have nothing attached to it.

### Two subtle scheduler facts (hard-won, do not regress)

1. **The tick rate (20 kHz) must exceed the gyro/PID rate (8 kHz).**
   Betaflight's scheduler runs non-realtime tasks (RX, MSP, serial) only on
   calls that land *between* gyro boundaries
   (`schedLoopRemainingCycles > CHECK_GUARD_MARGIN`). Ticking exactly at the
   gyro rate starves RX forever (symptom: `rcData` frozen at 1500,
   `rxIsReceivingSignal()` false). SimITL/KwadSimServer tick at 20 kHz for
   the same reason.
2. **Two lockstep patches to scheduler.c**
   (`extern/betaflightext/patches/scheduler-lockstep.diff`): the busy-wait
   on the cycle counter is removed (frozen time = infinite loop), and the
   gross-overrun recovery no longer overshoots the boundary into the future
   (unreachable when time is frozen inside `scheduler()`).

## Betaflight integration notes

- Override layer is deliberately tiny: `pw_sitl.c` (modified copy of 4.5.2's
  `target/SITL/sitl.c` — recorded diff in `patches/`) + patched
  `scheduler.c`. Everything else is stock 4.5.2, source list harvested from
  an authoritative `make TARGET=SITL` build (`extern/bf_sources.txt`).
- RC injection: `BF::setRcData()` overrides `rxRuntimeState` fn pointers
  (`RX_PROVIDER_UDP` path) — no MSP in the hot path.
- `targetPreInit()` (hook enabled via `TARGET_PREINIT`) forces
  `motor_pwm_protocol = PWM` after any config load: with DSHOT (the real
  quad's setting) `ARMING_DISABLED_BOOT_GRACE_TIME` never clears because
  DSHOT streaming commands never become available on the fake motor device.
- Aux modes ARM=AUX1(ch5) / ANGLE=AUX2(ch6) match the CineLog35 switch plan
  and are applied in RAM (`BF::configureDefaultModes()`).
- Physics (`core/sim/physics.cpp`, ported from SimITL): back-EMF motor
  torque, quadratic prop thrust vs airspeed, per-axis drag, battery
  sag/discharge, SO(3) skew-symmetric integration, propwash LPF, motor
  noise. **`motor_dir` is flipped vs SimITL** to match BF 4.5.2's default
  props-in quad-X yaw mixer — the SimITL signs produce yaw positive
  feedback here (diagonal motors saturate, quad climbs at min throttle).

## Division of labour with clients

The client is the collision **sensor**, the core is the collision **solver**.
The client owns geometry and world position: each `PW_STATE_IN` carries the
authoritative pose plus the frame's contact manifold (up to 6 points: body-
frame contact point, world-frame normal, penetration depth, surface type),
detected against the client's world (analytic ground plane + shape queries
for gates/trees) from the shared 5-sphere hull in `propwash_protocol.h`. The
client depenetrates its position with a 4 mm slop and reports near-contacts
within 5 mm at depth 0 (speculative, so the core can damp an approach before
impact).

The core owns dynamics — as of protocol v2 the velocity fields in
`PW_STATE_IN` are ignored. Contacts are resolved as penalty spring-damper
forces inside the 20 kHz tick's force/moment accumulator, which means the
firmware's virtual accelerometer feels ground support, touchdown and crashes
exactly as real hardware would (a velocity edit never reaches the accel; a
force does). Penetration depths evolve per sub-tick between client frames.

Consequences are core-owned and deterministic: contact approach speed maps
to per-motor prop damage (frame impacts, prop strikes inside a spinning
disc, structural crashes above ~7 m/s surface-scaled), reported back in
`PW_STATE_OUT.prop_damage`/`crash_flags`. A wrecked quad cannot hover
because damaged props cannot lift it — not because a script says so.
`PW_CMD_REPAIR` fits new props; `PW_CMD_RESET` is the full respawn. The
damage path draws no randomness and reads no clock, so reproducibility is
untouched. Wind (`--wind x,y,z`, `--gust amp`) is likewise a pure function
of sim time + seed via simplex noise: calm runs are bit-identical to
pre-wind builds, windy runs reproduce given the same flags.

## Loading the pilot's tune (M3)

The sim runs the pilot's **actual** Betaflight config, not an approximation.

- **CLI/Configurator path**: `tools/bfcli/pw_cli.py` speaks the Betaflight CLI
  over TCP 5761 (the same data path the Configurator GUI uses). `apply` a
  `diff all`, `save` (persists to eeprom, reboots in-process), and the tune
  survives — verified by the `diff_roundtrip` test. The real Betaflight
  Configurator can also connect live to 5761 to tweak PIDs/rates.
- **Config files**: `config/cinelog35v3.diff` is the **real** `diff all`
  pulled off the GEPRC_F722_AIO ("Cinelog35 V3", profiles "85%"/"100%", real
  PIDs p_pitch=53 etc.). `config/sitl-overrides.txt` neutralises settings that
  describe the physical board — critically `align_board_roll = 180` (the FC is
  mounted inverted; the sim's virtual gyro is already airframe-aligned, so
  leaving it would fly the quad upside-down). Motor protocol is forced to PWM
  in `targetPreInit`; DSHOT/RPM settings don't exist in the SITL value table
  and are rejected harmlessly.
- **`ARMING_DISABLED_CLI`**: opening the CLI sets this flag; it clears only on
  a clean `exit` (pw_cli sends it on close). So the load-then-fly flow bakes
  the eeprom in one instance and flies a *fresh* one that never touches the
  CLI — which is also how `load_config.sh` + normal launches work.
- The server boots the firmware immediately (5761 up before any client), so
  the Configurator / pw_cli can attach right away.
- **One-shot bake**: `tools/bfcli/load_config.sh` writes the tune into an
  eeprom once; every later launch flies it.
- **cli.c fix**: stock Betaflight guards `pgFind()==NULL` in `dumpPgValue`
  behind `#ifdef DEBUG`. SITL undefs whole features (OSD/VTX/...) while their
  settings stay in the value table, so `diff`/`dump`/`save` dereferenced NULL
  and segfaulted. Vendored cli.c makes the guard unconditional
  (`patches/cli-pgfind-null-guard.diff`).

## OSD (M3)

The real Betaflight OSD renders. `target.h` (a `#include_next` wrapper over
stock SITL) re-enables `USE_OSD`+`USE_MAX7456`; the AUTO device selection in
`fc/init.c` then binds a **fake max7456 displayport** (`io/displayport_fake.c`,
ported from SimITL) that captures the 16x30 character grid into `osdScreen[]`
instead of driving SPI. `bf.cpp` copies that grid into the sim state each
tick; the server streams it as `PW_OSD` (~15 Hz); the Godot client renders it
as a centered monospace overlay, FPV-goggle style. The `osd_render` test
confirms `osd.c`/`osd_warnings.c` produce real content (e.g. the `ARMSWITCH`
warning) end-to-end.

## Verification layers

1. `tools/tester/msp_check.py` — MSP over TCP 5761: firmware identifies as
   BTFL 4.5.2 (proof the real firmware is live).
2. `pw-tester` (ctest) — gyro calibration clears arming flags, arms via
   ch5, angle-mode hover 15 s within ±1 cm / tilt < 2°, deterministic state
   hash. 22 sim-seconds run in ~0.5 s wall (≈40× realtime, single thread).
3. (M5) blackbox replay against real CineLog35 logs.
