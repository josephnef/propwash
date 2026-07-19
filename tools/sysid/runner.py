"""Drive propwash-core to produce flight logs — the primitive record, replay and
sysid all share.

A Session owns one core subprocess. Because PW_INIT re-parameterises the physics
on a live core (Server::applyInit -> reinitPhysics), the system-ID fitter can
evaluate hundreds of candidate parameter sets against ONE core instead of
respawning per evaluation.

Two drive modes:
  run_motor : PW_MOTOR_IN — firmware bypassed, physics driven by given motors
              (physics-only replay; the clean signal for a physics fit).
  run_rc    : PW_STATE_IN — the firmware flies the RC, as the Godot client does.
"""
import os
import socket
import sys

sys.path.insert(0, os.path.dirname(__file__))
import wire  # noqa: E402
import bblog  # noqa: E402


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _decode(out):
    o = wire.SOUT.unpack(out)
    return {
        "quat": o[2:6],
        "angvel": o[6:9],
        "linvel": o[9:12],
        "accel": o[15:18],
        "rpm": o[18:22],
        "armed": o[26],
        "dis": o[27],
        "vbat": o[30],
    }


class Session:
    def __init__(self, core, port=None, settle=2.0, eeprom=None):
        self.port = port or free_port()
        self.proc, self.eeprom = wire.spawn(core, self.port, eeprom=eeprom,
                                             settle=settle)
        self.sock = wire.open_socket(2.0)
        self.addr = ("127.0.0.1", self.port)

    def set_profile(self, profile):
        wire.send_init(self.sock, self.addr, profile)

    def reset(self):
        wire.send_command(self.sock, self.addr, wire.PW_CMD_RESET)

    def run_motor(self, motors, dt):
        """Replay a per-frame motor sequence (each a 4-tuple 0..1) through the
        physics alone. Returns a native log."""
        self.reset()
        pos = [0.0, 0.0, 0.0]
        rot = (1.0, 0.0, 0.0, 0.0)
        frames = []
        for i, mt in enumerate(motors):
            contacts = wire.ground_manifold(pos, rot)  # depenetrates pos
            pkt = wire.pack_motor_in(i, dt, mt, pos, rot, contacts)
            d = _decode(wire.step(self.sock, self.addr, pkt))
            rot = d["quat"]
            lv = d["linvel"]
            pos = [pos[k] + lv[k] * dt for k in range(3)]
            frames.append(_frame(i * dt, [0]*8, mt, d, pos))
        return frames

    def run_rc(self, rc_fn, seconds, dt):
        """Fly an RC maneuver through the firmware. rc_fn(t) -> 8 channels.
        Motors are approximated from reported RPM (the wire exposes RPM, not the
        ESC command); a REAL blackbox carries true ESC motors — see bblog."""
        self.reset()
        pos = [0.0, 0.0, 0.0]
        rot = (1.0, 0.0, 0.0, 0.0)
        frames = []
        n = int(seconds / dt)
        maxrpm = 33000.0
        for i in range(n):
            t = i * dt
            rc = rc_fn(t)
            contacts = wire.ground_manifold(pos, rot)
            pkt = wire.pack_state_in(i, dt, rc, pos, rot, [0]*3, [0]*3,
                                     contacts=contacts)
            d = _decode(wire.step(self.sock, self.addr, pkt))
            rot = d["quat"]
            lv = d["linvel"]
            pos = [pos[k] + lv[k] * dt for k in range(3)]
            mot = [min(1.0, r / maxrpm) for r in d["rpm"]]
            frames.append(_frame(t, rc, mot, d, pos))
        return frames

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        wire.stop(self.proc, self.eeprom)


def _frame(t, rc, mot, d, pos):
    fr = {"t": t, "vbat": d["vbat"]}
    for k in range(8):
        fr[f"rc{k}"] = rc[k]
    for k in range(4):
        fr[f"mot{k}"] = mot[k]
    fr["gx"], fr["gy"], fr["gz"] = d["angvel"]
    fr["ax"], fr["ay"], fr["az"] = d["accel"]
    fr["qw"], fr["qx"], fr["qy"], fr["qz"] = d["quat"]
    fr["vx"], fr["vy"], fr["vz"] = d["linvel"]
    fr["px"], fr["py"], fr["pz"] = pos
    # fill any remaining schema fields with 0 for a complete row
    for k in bblog.FIELDS:
        fr.setdefault(k, 0.0)
    return fr
