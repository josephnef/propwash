"""Wire helpers for system identification — Pw_INIT (physics params) and
PW_MOTOR_IN (physics-only motor replay), on top of the shared codec in
tools/tester/pw_udp.py so the protocol has one definition.

Everything else (header, PW_STATE_IN/OUT, ground_manifold, spawn/stop, step)
is reused from pw_udp verbatim.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tester"))
import pw_udp  # noqa: E402  (shared MAGIC/VERSION/HDR/SOUT/step/ground_manifold)

from pw_udp import (  # noqa: E402,F401  re-export the common surface
    MAGIC, VERSION, HDR, SOUT, PW_STATE_OUT,
    spawn, stop, open_socket, step, ground_manifold, send_command,
    pack_state_in, PW_CMD_RESET, MAX_CONTACTS,
)

PW_INIT = 1
PW_MOTOR_IN = 8

# PwInit — exactly the C struct order (pragma pack(1)); see propwash_protocol.h.
PINIT = struct.Struct("<12fB4f3f3fff3f12fB3f4fI")
assert PINIT.size == 190, PINIT.size

# PwMotorIn: frame u32, dt f32, motor 4f, pos 3f, quat wxyz 4f, contact_count u8,
# then 6x contact (point 3f, normal 3f, depth f, surface u8).
MOTOR_IN = struct.Struct("<If4f3f4fB" + "3f3ffB" * MAX_CONTACTS)
assert MOTOR_IN.size == 227, MOTOR_IN.size


def pack_init(p):
    """Encode a PW_INIT from a profile dict (see profiles.CINELOG35)."""
    vals = []
    vals += p["motor_kv"]
    vals += p["motor_R"]
    vals += p["motor_I0"]
    vals.append(int(p["prop_blade_count"]))
    vals += [p["prop_max_rpm"], p["prop_a_factor"],
             p["prop_torque_factor"], p["prop_inertia"]]
    vals += p["prop_thrust_factor"]
    vals += p["frame_drag_area"]
    vals.append(p["frame_drag_constant"])
    vals.append(p["quad_mass"])
    vals += p["quad_inv_inertia"]
    for mp in p["quad_motor_pos"]:
        vals += mp
    vals.append(int(p["bat_cell_count"]))
    vals += [p["bat_capacity_mah"], p["bat_capacity_charged_mah"],
             p["max_voltage_sag"]]
    vals += [p["min_propwash_speed"], p["max_propwash_speed"],
             p["propwash_angle_of_attack"], p["propwash_factor"]]
    vals.append(int(p["seed"]))
    payload = PINIT.pack(*vals)
    return HDR.pack(MAGIC, VERSION, PW_INIT, len(payload)) + payload


def pack_motor_in(frame, dt, motor, pos, rot, contacts=None):
    """Encode a PW_MOTOR_IN. motor is 4 ESC commands 0..1; pos/rot as in
    pack_state_in; contacts up to 6 (point, normal, depth, surface)."""
    cts = list(contacts or [])[:MAX_CONTACTS]
    flat = []
    for point, normal, depth, surface in cts:
        flat += [*point, *normal, depth, surface]
    for _ in range(MAX_CONTACTS - len(cts)):
        flat += [0.0] * 7 + [0]
    payload = MOTOR_IN.pack(frame, dt, *motor, *pos, *rot, len(cts), *flat)
    return HDR.pack(MAGIC, VERSION, PW_MOTOR_IN, len(payload)) + payload


def send_init(sock, addr, profile):
    """Send PW_INIT and drain the PW_INIT_ACK reply."""
    sock.sendto(pack_init(profile), addr)
    try:
        while True:
            sock.recvfrom(2048)  # ack (+ maybe an OSD) — we just clear the queue
            break
    except OSError:
        pass
