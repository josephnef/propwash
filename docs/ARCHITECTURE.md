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
identical trajectories: `pw-tester` asserts equal state hashes across runs.

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

The client owns terrain/collision: each `PW_STATE_IN` carries the
authoritative pose/velocities (normally echoing the previous `PW_STATE_OUT`,
corrected on contact), plus `contact` so the physics damps toward the
client's resolution. The core owns everything between RC input and rigid
body state.

## Verification layers

1. `tools/tester/msp_check.py` — MSP over TCP 5761: firmware identifies as
   BTFL 4.5.2 (proof the real firmware is live).
2. `pw-tester` (ctest) — gyro calibration clears arming flags, arms via
   ch5, angle-mode hover 15 s within ±1 cm / tilt < 2°, deterministic state
   hash. 22 sim-seconds run in ~0.5 s wall (≈40× realtime, single thread).
3. (M5) blackbox replay against real CineLog35 logs.
