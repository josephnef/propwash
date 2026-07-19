# tools/sysid — blackbox replay & system identification

Replay a Betaflight blackbox log through the sim and fit the physics model to
it, so the simulator flies like the *real* CineLog35 — the certification step
that makes propwash trustworthy for the teleoperator data flywheel.

Stdlib-only Python 3 (no numpy). Everything drives `propwash-core` over the UDP
protocol; parameters are injected with `PW_INIT`, which re-parameterises the
physics on a live core, so a fit evaluates hundreds of candidates against one
process.

## Two replay modes

| mode | packet | what runs | isolates |
|------|--------|-----------|----------|
| **RC** (`--mode rc`) | `PW_STATE_IN` | firmware **+** physics, as flown | end-to-end behaviour |
| **motor** (`--mode motor`) | `PW_MOTOR_IN` | **physics only** — firmware bypassed | the physics model, free of PID error |

Physics-only replay feeds the log's recorded ESC commands straight into the
motor/prop/airframe model. That's the clean signal for a physics fit: the
resulting gyro/accel trajectory is a pure function of the physics parameters,
with the firmware's control loop out of the picture.

## Pipeline

```bash
# 1. record a reference (until the real quad has flown, the sim stands in)
python3 tools/sysid/record.py --mode motor --out ref.csv

# 2. replay it and score the sim-vs-reference error
python3 tools/sysid/replay.py ref.csv --mode motor

# 3. fit physics parameters to the reference
python3 tools/sysid/sysid.py ref.csv --fit quad_mass,prop_thrust_factor.2
```

Fittable parameters (`profiles.FITTABLE`): `quad_mass`,
`prop_thrust_factor.2` (static max thrust/prop), `frame_drag_constant`,
`prop_torque_factor`, `quad_inv_inertia.0/.1`. The fitter is a derivative-free
pattern search (`sysid.hooke_jeeves`) minimising gyro+accel RMSE — the two
signals a real blackbox always carries.

> **Identifiability.** A pure vertical climb constrains only the thrust/mass
> *ratio* — fit one with the other held, or use a maneuver that also excites
> rotation (`maneuver.roll_doublet`) to separate inertia/torque. `sysid.py`
> fits whatever subset you name; whether that subset is identifiable is a
> property of the maneuver, not the tool.

## Using a real Betaflight log

Once the CineLog35 flies and logs to its blackbox, decode it and import:

```bash
blackbox_decode LOG00001.BFL          # -> LOG00001.01.csv
python3 -c "import bblog; log = bblog.import_betaflight_csv('LOG00001.01.csv'); \
            bblog.write_log('real.csv', log)"
python3 tools/sysid/sysid.py real.csv --fit quad_mass,prop_thrust_factor.2
```

`import_betaflight_csv` maps `gyroADC`/`accSmooth`/`motor`/`rcCommand` onto the
native schema (best-effort — check its docstring for the unit/range
assumptions, which vary by firmware and by the flags passed to
`blackbox_decode`).

## Self-check (`sysid_selfcheck` ctest)

With no real log yet, `sysid_check.py` certifies the *pipeline* on synthetic
references:

1. **Reproducibility** — the same motor sequence replayed through two
   independent cores gives an identical gyro/accel trajectory (observed RMSE
   `0.00e+00`). Physics-only replay is bit-stable: the residual-firmware-state
   non-determinism that makes the gym's step-determinism an xfail does not apply
   when the firmware is bypassed.
2. **Recovery** — from a reference generated with the true profile, the fitter
   starts at a wrong guess and recovers the true `prop_thrust_factor.2` (2.800 →
   2.801, RMSE 0.39 → 0.0015).

## Files

| file | role |
|------|------|
| `profiles.py` | baseline CineLog35 `PwInit` vector + fittable params/bounds |
| `wire.py` | `PW_INIT` / `PW_MOTOR_IN` codec (on top of `tools/tester/pw_udp.py`) |
| `runner.py` | `Session`: one core, `run_motor` / `run_rc` → native logs |
| `bblog.py` | log schema, CSV I/O, `blackbox_decode` importer, RMSE metrics |
| `maneuver.py` | excitation sequences (collective climb, roll doublet, RC climb) |
| `record.py` / `replay.py` / `sysid.py` | the CLI pipeline |
| `sysid_check.py` | the `sysid_selfcheck` ctest |
