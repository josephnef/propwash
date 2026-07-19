#!/usr/bin/env python3
"""Record a reference flight log from propwash-core.

  record.py --mode motor --out ref.csv        # open-loop motor maneuver (sysid)
  record.py --mode rc    --out flight.csv      # firmware flies an RC maneuver

The motor-mode log carries the exact commanded ESC values, which is what a real
Betaflight blackbox stores and what physics-only replay/sysid consume. Until the
real quad has flown, this stands in for a genuine log; a real one drops into the
same pipeline via bblog.import_betaflight_csv.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import bblog  # noqa: E402
import maneuver  # noqa: E402
import profiles  # noqa: E402
from runner import Session  # noqa: E402

DEFAULT_CORE = os.path.join(os.path.dirname(__file__), "..", "..",
                            "build", "propwash-core")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--core", default=DEFAULT_CORE)
    ap.add_argument("--mode", choices=["motor", "rc"], default="motor")
    ap.add_argument("--out", default="ref.csv")
    ap.add_argument("--dt", type=float, default=1.0 / 500.0)
    ap.add_argument("--seconds", type=float, default=1.6)
    args = ap.parse_args()

    profile = profiles.CINELOG35
    s = Session(args.core)
    try:
        s.set_profile(profile)
        if args.mode == "motor":
            motors = maneuver.collective_climb(args.dt, seconds=args.seconds)
            frames = s.run_motor(motors, args.dt)
        else:
            frames = s.run_rc(maneuver.rc_hover_climb(), args.seconds + 5.4,
                              args.dt)
    finally:
        s.close()

    bblog.write_log(args.out, frames)
    print(f"recorded {len(frames)} frames ({args.mode}) -> {args.out}")
    print(f"  final alt {frames[-1]['py']:.2f} m, "
          f"peak |gyro| {max(abs(f['gx']) for f in frames):.2f} rad/s")


if __name__ == "__main__":
    main()
