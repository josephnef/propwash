#!/usr/bin/env python3
"""Replay a reference log through the sim and score the divergence.

  replay.py ref.csv --mode motor      # physics-only: feed the log's motors
  replay.py ref.csv --mode rc         # closed loop: feed the log's RC

Motor mode isolates the physics model (no firmware); RC mode exercises the
firmware + physics as flown. Prints per-axis gyro/accel RMSE — the sim-vs-log
error a system-ID fit minimises.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import bblog  # noqa: E402
import profiles  # noqa: E402
from runner import Session  # noqa: E402

DEFAULT_CORE = os.path.join(os.path.dirname(__file__), "..", "..",
                            "build", "propwash-core")


def replay_motor(session, ref, dt):
    motors = [(f["mot0"], f["mot1"], f["mot2"], f["mot3"]) for f in ref]
    return session.run_motor(motors, dt)


def replay_rc(session, ref, dt):
    rc_by_i = [[f[f"rc{k}"] for k in range(8)] for f in ref]

    def rc_fn(t):
        i = min(len(rc_by_i) - 1, int(round(t / dt)))
        return rc_by_i[i]
    return session.run_rc(rc_fn, len(ref) * dt, dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("--core", default=DEFAULT_CORE)
    ap.add_argument("--mode", choices=["motor", "rc"], default="motor")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    ref = bblog.read_log(args.ref)
    if len(ref) < 2:
        print("FAIL: reference log too short")
        return 1
    dt = ref[1]["t"] - ref[0]["t"]

    s = Session(args.core)
    try:
        s.set_profile(profiles.CINELOG35)
        sim = replay_motor(s, ref, dt) if args.mode == "motor" \
            else replay_rc(s, ref, dt)
    finally:
        s.close()

    if args.out:
        bblog.write_log(args.out, sim)

    fields = bblog.GYRO + bblog.ACCEL
    total = bblog.rmse(ref, sim, fields)
    pf = bblog.per_field_rmse(ref, sim, fields)
    print(f"replay ({args.mode}): {len(sim)} frames vs {len(ref)} reference")
    print(f"  gyro RMSE  gx={pf['gx']:.4f} gy={pf['gy']:.4f} gz={pf['gz']:.4f} rad/s")
    print(f"  accel RMSE ax={pf['ax']:.3f} ay={pf['ay']:.3f} az={pf['az']:.3f} m/s^2")
    print(f"  combined RMSE {total:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
