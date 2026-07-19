#!/usr/bin/env python3
"""Standalone gym smoke check, wired into ctest as `gym_hover`.

Runs without installing the package (adds ``src`` to sys.path) so ctest can
drive it with any interpreter that has numpy + gymnasium. Spawns a real
propwash-core, runs the Gymnasium env-checker, then flies a short hover and
asserts the quad armed and left the pad.

    python3 gym_smoke.py [path/to/propwash-core]

Exit 0 = the env obeys the API contract and flies.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import propwash_gym  # noqa: E402,F401  (registers env ids)
from propwash_gym.env import PropwashEnv  # noqa: E402


def main():
    if len(sys.argv) > 1:
        os.environ["PROPWASH_CORE"] = sys.argv[1]

    from gymnasium.utils.env_checker import check_env
    env = PropwashEnv(episode_seconds=2.0, arm_seconds=5.4)
    try:
        # Includes Gymnasium >=1.0's step-determinism assertion: two resets
        # plus identical actions must yield identical observations. Holds
        # since the core's snapshot/restore reset (reset ≡ fresh process,
        # gated by reset_determinism / cross_process_determinism); enforced
        # here so a regression fails the gym smoke too.
        check_env(env, skip_render_check=True)
        print("1. gymnasium env-checker: PASS (incl. step determinism)")
    finally:
        env.close()

    env = PropwashEnv(episode_seconds=6.0, arm_seconds=5.4, target_altitude=2.0)
    try:
        obs, info = env.reset(seed=0)
        assert env.observation_space.contains(obs), "reset obs out of space"
        if not info["armed"]:
            print(f"FAIL: did not arm (flags=0x{info['arming_disable_flags']:x})")
            return 1
        print(f"2. reset armed the quad (obs dim {obs.shape[0]})")

        peak, ret = 0.0, 0.0
        for _ in range(env.max_steps):
            obs, r, term, trunc, info = env.step(
                np.array([0.15, 0, 0, 0], np.float32))
            ret += r
            peak = max(peak, info["altitude"])
            if term or trunc:
                break
        print(f"3. hover: peak alt {peak:.2f} m, return {ret:.1f}")
        ok = peak > 0.3
        print("GYM SMOKE PASS" if ok else "GYM SMOKE FAIL (never left the pad)")
        return 0 if ok else 1
    finally:
        env.close()


if __name__ == "__main__":
    sys.exit(main())
