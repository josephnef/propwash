"""propwash_gym — a Gymnasium environment backed by the propwash simulator,
which runs the pilot's real Betaflight firmware in the loop.

    import gymnasium as gym
    import propwash_gym          # registers the env ids

    env = gym.make("propwash/Hover-v0")

See :class:`propwash_gym.env.PropwashEnv` for the observation/action spaces and
the constructor knobs (core path, eeprom, control rate, target altitude, ...).
"""
from gymnasium.envs.registration import register

from .env import PropwashEnv

__all__ = ["PropwashEnv", "register_envs"]


def register_envs():
    """Register propwash env ids. Called on import, and also exposed as the
    ``gymnasium.envs`` entry point so ``gym.make`` works without an explicit
    ``import propwash_gym``."""
    register(
        id="propwash/Hover-v0",
        entry_point="propwash_gym.env:PropwashEnv",
        max_episode_steps=None,   # the env truncates itself (episode_seconds)
    )


register_envs()
