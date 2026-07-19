# propwash-gym

A [Gymnasium](https://gymnasium.farama.org/) environment for **propwash** — the
FPV simulator that runs the pilot's *real Betaflight firmware* in the loop. The
policy trains against the exact PIDs, rates, filters and failsafe on the
physical CineLog35, over the same deterministic lockstep protocol the Godot
client uses. MIT-licensed; it speaks only the UDP wire protocol and links no
GPL code.

## Install (uv)

```bash
cd python/propwash_gym
uv sync                 # base: gymnasium + numpy
uv sync --extra rl      # + stable-baselines3 + torch (for examples/)
```

The env drives a `propwash-core` subprocess, so build the C++ core first:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
```

`PropwashEnv` finds the core via, in order: the `core_path=` arg,
`$PROPWASH_CORE`, `propwash-core` on `PATH`, then `build/propwash-core` in the
repo.

## Use

```python
import gymnasium as gym
import propwash_gym            # registers the env ids

env = gym.make("propwash/Hover-v0")
obs, info = env.reset(seed=0)  # arms the quad on the pad (gyro cal + ARM)
for _ in range(1000):
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        obs, info = env.reset()
env.close()
```

- **Action** `Box(4,)` in `-1..1`: `[throttle, roll, pitch, yaw]` (throttle -1
  idle → +1 full). Arming and the ANGLE-mode switch are driven by the env.
- **Observation** `Box(17,)`: `quat(4) | angular_velocity(3) | linear_velocity(3)
  | position_error(3) | motor_rpm(4)`.
- **Reward**: hover shaping — alive bonus, penalties for altitude error,
  horizontal drift, speed, spin and stick effort, an uprightness bonus, and a
  −10 terminal on crash / flip / failsafe-disarm / out-of-bounds.

Each env owns one core process on its own UDP port, so vectorised training just
spawns N of them. Every `step()` is one PW_STATE_IN → PW_STATE_OUT round trip in
`frame_id` lockstep: simulated time only advances when the trainer steps, so
rollouts are reproducible and never race.

### Train (smoke)

```bash
uv run python examples/ppo_hover.py --timesteps 20000 --n-envs 2
```

This is a *smoke run* (proves the loop trains), not a tuned agent.

## Test

```bash
uv run pytest            # env-checker + arm/step/hover; skips if core not built
```

## Notes

- **ANGLE vs ACRO**: default is ANGLE (`angle_mode=True`) so the firmware
  self-levels and the RL problem is throttle/position. Pass `angle_mode=False`
  for raw ACRO stabilisation.
- **Determinism**: the env defaults to `gyro_noise=0` (deterministic lockstep —
  simulated time only advances on a step). Note the sim is **not yet
  bit-reproducible across `reset()`s**: residual firmware state that
  `BF::init()` doesn't clear varies between sessions (the repo's
  `determinism_check` is red for the same reason — that's the M5 system-ID
  work). Consequently Gymnasium's `check_env` step-determinism sub-check is a
  known xfail; the rest of the API contract passes. Set `gyro_noise>0` to train
  a policy robust to sensor noise.
