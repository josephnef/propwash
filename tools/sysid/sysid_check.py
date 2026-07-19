#!/usr/bin/env python3
"""Self-validation for the blackbox-replay + system-ID pipeline, wired into
ctest as `sysid_selfcheck`. With no real flight log yet, it proves the pipeline
is correct on synthetic references the sim generates:

  1. physics-only replay is reproducible — the same motor sequence, replayed
     through two independent cores, yields the same gyro/accel trajectory.
     (Firmware is bypassed in PW_MOTOR_IN, so the residual-firmware-state
     non-determinism that makes the gym's step-determinism xfail does not
     apply here — physics-only replay IS bit-stable.)

  2. system ID recovers a known parameter — take a reference generated with the
     true profile, start the fitter from a wrong guess, and confirm it recovers
     the true value and slashes the sim-vs-reference error.

A real CineLog35 blackbox drops into exactly this path (bblog.import_betaflight_csv
-> replay/sysid) once the quad has flown.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import bblog  # noqa: E402
import maneuver  # noqa: E402
import profiles  # noqa: E402
import sysid  # noqa: E402
from runner import Session  # noqa: E402

CORE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), "..", "..", "build", "propwash-core")
DT = 1.0 / 500.0
FIELDS = bblog.GYRO + bblog.ACCEL


def main():
    motors = maneuver.collective_climb(DT, seconds=1.4)

    # 1. reproducibility of physics-only replay across two independent cores
    a = Session(CORE)
    b = Session(CORE)
    try:
        a.set_profile(profiles.CINELOG35)
        b.set_profile(profiles.CINELOG35)
        ref = a.run_motor(motors, DT)
        rep = b.run_motor(motors, DT)
    finally:
        a.close()
        b.close()
    determinism = bblog.rmse(ref, rep, FIELDS)
    print(f"1. physics-only replay reproducibility: RMSE {determinism:.2e} "
          f"(gyro+accel, two cores)")
    print(f"   reference: {len(ref)} frames, final alt {ref[-1]['py']:.2f} m, "
          f"peak |accel| {max(abs(f['ay']) for f in ref):.1f} m/s^2")
    if determinism > 1e-3:
        print("FAIL: physics-only replay is not reproducible across cores")
        return 1

    # 2. recover a known parameter (static max thrust) from a wrong start
    key = "prop_thrust_factor.2"
    truth = profiles.get_param(profiles.CINELOG35, key)
    lo, hi = profiles.FITTABLE[key]
    guess = 0.5 * (lo + hi)               # deliberately wrong start (mid-bounds)

    s = Session(CORE)
    try:
        # error at the wrong guess, for a before/after comparison
        obj = sysid.make_objective(s, ref, [key], DT)
        err0 = obj([guess])
        fitted, err1, n = sysid.fit(CORE, ref, [key], x0=[guess],
                                    iters=80, session=s)
    finally:
        s.close()

    got = profiles.get_param(fitted, key)
    print(f"2. system ID of {key}: truth {truth:.3f}, "
          f"guess {guess:.3f} -> fitted {got:.3f} ({n} evals)")
    print(f"   RMSE {err0:.4f} (guess) -> {err1:.4f} (fitted)")

    ok_recovered = abs(got - truth) < 0.20
    ok_improved = err1 < 0.5 * err0
    if not ok_recovered:
        print(f"FAIL: did not recover the parameter (|{got:.3f}-{truth:.3f}| too large)")
        return 1
    if not ok_improved:
        print("FAIL: fit did not materially reduce the error")
        return 1

    print("SYSID SELFCHECK PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
