"""Smoke tests for PropwashEnv. These spawn a real propwash-core, so they need
the C++ build present (build/propwash-core or $PROPWASH_CORE); they skip
cleanly if it is missing rather than failing a Python-only checkout.
"""
import numpy as np
import pytest

import propwash_gym  # noqa: F401  (registers env ids)
from propwash_gym.env import PropwashEnv, _find_core


def _have_core():
    try:
        _find_core(None)
        return True
    except FileNotFoundError:
        return False


core = pytest.mark.skipif(not _have_core(), reason="propwash-core not built")


@core
def test_gymnasium_env_checker():
    """The env obeys the Gymnasium API contract — INCLUDING the >=1.0
    step-determinism assertion (two resets + identical actions => identical
    observations). That sub-check was a known xfail until the core gained its
    snapshot/restore reset (reset ≡ fresh process, gated core-side by the
    reset_determinism / cross_process_determinism ctests); it is enforced
    here so a regression fails the gym too."""
    from gymnasium.utils.env_checker import check_env
    env = PropwashEnv(episode_seconds=2.0, arm_seconds=5.4)
    try:
        # skip_render_check: this env has no render mode
        check_env(env, skip_render_check=True)
    finally:
        env.close()


@core
def test_reset_arms_and_step_shapes():
    env = PropwashEnv(episode_seconds=2.0, arm_seconds=5.4)
    try:
        obs, info = env.reset(seed=0)
        assert obs.shape == env.observation_space.shape
        assert env.observation_space.contains(obs)
        assert info["armed"], f"did not arm: flags=0x{info['arming_disable_flags']:x}"

        total = 0.0
        for _ in range(50):
            obs, r, term, trunc, info = env.step(env.action_space.sample())
            assert env.observation_space.contains(obs)
            total += r
            if term or trunc:
                break
        assert np.isfinite(total)
    finally:
        env.close()


@core
def test_hover_action_climbs():
    """A steady near-hover throttle should get the quad off the pad — proves
    the firmware is actually flying, not just echoing."""
    env = PropwashEnv(episode_seconds=6.0, arm_seconds=5.4, target_altitude=2.0)
    try:
        env.reset(seed=1)
        peak = 0.0
        for _ in range(env.max_steps):
            # gentle collective, sticks centred; ANGLE mode holds it level
            _, _, term, trunc, info = env.step(np.array([0.15, 0, 0, 0], np.float32))
            peak = max(peak, info["altitude"])
            if term or trunc:
                break
        assert peak > 0.3, f"quad never left the pad (peak alt {peak:.2f} m)"
    finally:
        env.close()
