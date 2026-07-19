"""Minimal PPO hover smoke run — proves the env trains, not that it's tuned.

    uv sync --extra rl
    uv run python examples/ppo_hover.py --timesteps 20000

Needs the RL extra (stable-baselines3 + torch) and a built propwash-core.
Each parallel env spawns its own core subprocess, so keep n_envs modest.
"""
import argparse

import gymnasium as gym
import propwash_gym  # noqa: F401  (registers env ids)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=20_000)
    ap.add_argument("--n-envs", type=int, default=2)
    ap.add_argument("--save", default="ppo_hover.zip")
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env

    # subprocess-per-env: each PropwashEnv already owns a core process, so a
    # DummyVecEnv (in-process) is fine — the cores are the parallelism.
    venv = make_vec_env(
        "propwash/Hover-v0", n_envs=args.n_envs,
        env_kwargs=dict(episode_seconds=12.0))

    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=256,
                gamma=0.99, verbose=1)
    model.learn(total_timesteps=args.timesteps)
    model.save(args.save)
    print(f"saved {args.save}")

    # quick eval
    eval_env = gym.make("propwash/Hover-v0", episode_seconds=12.0)
    obs, _ = eval_env.reset()
    ret, done = 0.0, False
    while not done:
        act, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, info = eval_env.step(act)
        ret += r
        done = term or trunc
    print(f"eval return {ret:.1f}, final alt {info['altitude']:.2f} m")
    eval_env.close()
    venv.close()


if __name__ == "__main__":
    main()
