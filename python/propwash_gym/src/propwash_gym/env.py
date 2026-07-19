"""PropwashEnv — a Gymnasium environment that flies real Betaflight.

Each env owns one ``propwash-core`` subprocess (subprocess-per-env, so vectorised
training just spawns N cores on N ports) and drives it in **frame_id lockstep**:
every :meth:`step` sends exactly one PW_STATE_IN and blocks on the matching
PW_STATE_OUT. Because the core only advances simulated time on a PW_STATE_IN,
the environment is deterministic and never races the trainer.

The task is a hover: arm, take off from the pad, and hold a target altitude
while staying level and still. The agent commands the four flight axes; arming
and the flight-mode switch are driven by the env (ANGLE mode by default, so the
firmware self-levels — the RL problem is then throttle/position, not raw
stabilisation, which is the useful starting point for the data-flywheel work).

The core runs the pilot's real firmware, so the PIDs, rates, filters and
failsafe the policy trains against are exactly the ones on the physical quad.
"""
from __future__ import annotations

import os
import shutil
import socket
import subprocess
import tempfile
import time

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from . import protocol as pw


def _find_core(explicit: str | None) -> str:
    """Locate build/propwash-core: explicit arg, $PROPWASH_CORE, PATH, then
    the in-tree build dir relative to this file."""
    for cand in (explicit, os.environ.get("PROPWASH_CORE")):
        if cand and os.path.exists(cand):
            return cand
    on_path = shutil.which("propwash-core")
    if on_path:
        return on_path
    here = os.path.dirname(__file__)
    guess = os.path.abspath(
        os.path.join(here, "..", "..", "..", "..", "build", "propwash-core"))
    if os.path.exists(guess):
        return guess
    raise FileNotFoundError(
        "propwash-core not found — pass core_path=, set $PROPWASH_CORE, or "
        "build it (cmake --build build).")


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class PropwashEnv(gym.Env):
    """Hover task on top of propwash-core.

    Action (Box, 4, -1..1): ``[throttle, roll, pitch, yaw]``. Throttle -1 is
    idle, +1 full; roll/pitch/yaw are the usual stick deflections.

    Observation (Box, 17): orientation quaternion (4, w-x-y-z), angular
    velocity (3, rad/s), linear velocity (3, m/s), position error to the target
    (3, m), and normalised per-motor RPM (4), i.e.
    ``quat | angvel | linvel | pos_err | rpm4``.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        core_path: str | None = None,
        eeprom_path: str | None = None,
        control_hz: float = 100.0,
        target_altitude: float = 2.0,
        episode_seconds: float = 15.0,
        arm_seconds: float = 5.4,
        angle_mode: bool = True,
        gyro_noise: float = 0.0,
        port: int | None = None,
        boot_settle: float = 2.0,
        render_mode=None,
    ):
        super().__init__()
        self.core_path = _find_core(core_path)
        self.eeprom_path = eeprom_path      # None → core uses a fresh default
        self.dt = 1.0 / control_hz
        self.target_alt = float(target_altitude)
        self.max_steps = int(episode_seconds * control_hz)
        self.arm_steps = int(arm_seconds * control_hz)
        self.angle_mode = angle_mode
        # 0 → deterministic lockstep (the project's headline property; needed
        # for reproducible rollouts and the Gymnasium step-determinism check).
        # Raise it to train a policy robust to sensor noise.
        self.gyro_noise = float(gyro_noise)
        self.port = port
        self.boot_settle = boot_settle
        self.render_mode = render_mode

        # obs: quat(4) + angvel(3) + linvel(3) + pos_err(3) + rpm(4) = 17
        high = np.array(
            [1] * 4 + [50] * 3 + [30] * 3 + [50] * 3 + [1] * 4,
            dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        self._proc: subprocess.Popen | None = None
        self._sock: socket.socket | None = None
        self._addr = None
        self._owns_eeprom = False
        self._frame = 0
        self._steps = 0
        # client-integrated world pose (see the reference clients: the core is
        # velocity-authoritative, the client integrates position from linvel)
        self._pos = [0.0, 0.0, 0.0]
        self._rot = (1.0, 0.0, 0.0, 0.0)
        self._max_rpm = 40000.0  # for RPM normalisation only

    # -- process lifecycle --------------------------------------------------
    def _spawn(self):
        if self._proc is not None:
            return
        port = self.port or _free_udp_port()
        self.port = port
        eeprom = self.eeprom_path
        if eeprom is None:
            eeprom = tempfile.mktemp(suffix=".bin")
            self._owns_eeprom = True
        cmd = [self.core_path, "--server", "--no-js",
               "--eeprom", eeprom, "--port", str(port)]
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(2.0)
        self._addr = ("127.0.0.1", port)
        self._eeprom = eeprom
        time.sleep(self.boot_settle)

    def _send_command(self, command: int):
        self._sock.sendto(pw.pack_command(command), self._addr)

    def _tick(self, rc) -> pw.StateOut:
        """One lockstep advance: send this frame's pose+RC, get the core's
        truth back, integrate world position client-side."""
        contacts = pw.ground_manifold(self._pos, self._rot)  # depenetrates pos
        pkt = pw.pack_state_in(self._frame, self.dt, rc, self._pos, self._rot,
                               [0, 0, 0], [0, 0, 0], contacts=contacts,
                               gyro_noise=self.gyro_noise)
        self._sock.sendto(pkt, self._addr)
        while True:
            data, _ = self._sock.recvfrom(2048)
            magic, _v, typ, _len = pw.HDR.unpack(data[:8])
            if magic == pw.MAGIC and typ == pw.PW_STATE_OUT:
                break  # skip interleaved PW_OSD
        out = pw.unpack_state_out(data[8:8 + pw.SOUT.size])
        self._rot = out.quat
        self._pos = [self._pos[i] + out.linear_velocity[i] * self.dt
                     for i in range(3)]
        self._frame += 1
        return out

    def _rc(self, action, arm: bool) -> list:
        """Map a policy action to the 8 RC channels (AETR + aux)."""
        thr, roll, pitch, yaw = (float(np.clip(a, -1, 1)) for a in action)
        return [roll, pitch, thr, yaw,
                1.0 if arm else -1.0,          # ch5: ARM
                1.0 if self.angle_mode else -1.0,  # ch6: ANGLE
                -1.0, -1.0]

    # -- Gym API ------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._spawn()
        # start from a known state: reset firmware + physics on the pad
        self._pos = [0.0, 0.0, 0.0]
        self._rot = (1.0, 0.0, 0.0, 0.0)
        self._frame = 0
        self._steps = 0
        self._send_command(pw.PW_CMD_RESET)
        time.sleep(0.05)

        # warm-up: sit idle with the arm switch off while the firmware finishes
        # gyro calibration (arming-disable flags clear), then flip ARM on and
        # tick until the firmware reports ARMED.
        idle = [0, 0, -1.0, 0, -1.0,
                1.0 if self.angle_mode else -1.0, -1.0, -1.0]
        for _ in range(self.arm_steps):
            self._tick(idle)
        out = None
        for _ in range(int(1.0 / self.dt)):  # up to 1 s to latch ARMED
            out = self._tick(self._rc([-1, 0, 0, 0], arm=True))
            if out.armed:
                break

        obs = self._obs(out)
        info = {"armed": bool(out.armed),
                "arming_disable_flags": out.arming_disable_flags}
        return obs, info

    def step(self, action):
        out = self._tick(self._rc(action, arm=True))
        self._steps += 1

        obs = self._obs(out)
        reward, terminated = self._reward(out, action)
        truncated = self._steps >= self.max_steps
        info = {
            "armed": bool(out.armed),
            "altitude": self._pos[1],
            "crashed": out.crashed,
            "vbat": out.vbat,
        }
        return obs, reward, terminated, truncated, info

    def _obs(self, out: pw.StateOut) -> np.ndarray:
        pe = [self._pos[0], self._pos[1] - self.target_alt, self._pos[2]]
        rpm = [min(1.0, r / self._max_rpm) for r in out.motor_rpm]
        v = list(out.quat) + list(out.angular_velocity) \
            + list(out.linear_velocity) + pe + rpm
        return np.asarray(v, dtype=np.float32)

    def _reward(self, out: pw.StateOut, action):
        # upright-ness: world-up projected onto body-up (1 = level, -1 = inverted)
        w, x, y, z = out.quat
        up = 1 - 2 * (x * x + z * z)
        alt_err = abs(self._pos[1] - self.target_alt)
        horiz = (self._pos[0] ** 2 + self._pos[2] ** 2) ** 0.5
        speed = sum(v * v for v in out.linear_velocity) ** 0.5
        spin = sum(v * v for v in out.angular_velocity) ** 0.5
        effort = float(np.mean(np.square(action[1:])))  # penalise stick, not thr

        reward = (
            1.0                       # alive bonus
            - 1.0 * alt_err
            - 0.3 * horiz
            - 0.1 * speed
            - 0.02 * spin
            - 0.05 * effort
            + 0.2 * up
        )

        terminated = False
        if out.crashed or up < 0.0:          # crashed or tipped past 90 deg
            reward -= 10.0
            terminated = True
        elif not out.armed:                   # firmware disarmed us (failsafe)
            reward -= 10.0
            terminated = True
        elif horiz > 8.0 or self._pos[1] > 12.0:  # flew out of bounds
            reward -= 10.0
            terminated = True
        return float(reward), terminated

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        if self._owns_eeprom and getattr(self, "_eeprom", None):
            try:
                os.remove(self._eeprom)
            except OSError:
                pass

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
