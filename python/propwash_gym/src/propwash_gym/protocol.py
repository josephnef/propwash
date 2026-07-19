"""propwash wire protocol — a standalone, MIT-clean codec for the gym.

This mirrors ``protocol/propwash_protocol.h`` (protocol v2) and is deliberately
independent of ``tools/tester/pw_udp.py`` (which lives in the GPL test tree):
the gym is MIT and speaks only the documented socket protocol, so it carries
its own copy of the wire format.

Only the packets the gym needs are implemented: PW_STATE_IN / PW_STATE_OUT and
PW_COMMAND. Struct layouts are asserted against the sizes baked into the C
header so a protocol bump fails loudly here instead of corrupting silently.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = 0x48535750  # "PWSH" little-endian
VERSION = 2

# packet types (PwPacketType)
PW_STATE_IN, PW_STATE_OUT, PW_OSD, PW_RC_OVERRIDE, PW_COMMAND = 3, 4, 5, 6, 7
# commands (PwCommandType)
PW_CMD_RESET, PW_CMD_PAUSE, PW_CMD_RESUME, PW_CMD_REALTIME, PW_CMD_LOCKSTEP, \
    PW_CMD_REPAIR = 1, 2, 3, 4, 5, 6
# surface materials (PwSurfaceType)
PW_SURF_GROUND, PW_SURF_GATE, PW_SURF_TREE, PW_SURF_OBJECT = 0, 1, 2, 3

MAX_CONTACTS = 6
RC_CHANNELS = 8
MOTOR_COUNT = 4

HDR = struct.Struct("<IBBH")
CMD = struct.Struct("<I")

# PwStateIn: frame u32, dt f32, rc 8f, pos 3f, quat wxyz 4f, angvel 3f,
# linvel 3f, gyro_noise f, prop_damage 4f, ground_effect 4f, vbat f,
# contact u8, contact_count u8, then 6x contact (point 3f, normal 3f,
# depth f, surface u8).
SIN = struct.Struct("<If8f3f4f3f3ff4f4ffBB" + "3f3ffB" * MAX_CONTACTS)
# PwStateOut: frame u32, sim_us u64, quat 4f, angvel 3f, linvel 3f, pos 3f,
# accel 3f, motor_rpm 4f, motor_status 4B, armed B, arming_disable u32,
# flight_mode u32, beeper B, vbat f, amperage f, prop_damage 4f, crash_flags B.
SOUT = struct.Struct("<IQ4f3f3f3f3f4f4BBIIBff4fB")

assert SIN.size == 308, SIN.size
assert SOUT.size == 131, SOUT.size

# --- collision hull (mirrors PW_HULL_* in the C header) -------------------
# belly sphere + 4 duct spheres; every sender derives ground contacts from the
# same five spheres so the core sees one consistent hull.
HULL = [((0.0, 0.030, 0.0), 0.045)] + [
    ((sx * 0.054, 0.010, sz * 0.054), 0.030)
    for sx in (1, -1) for sz in (1, -1)
]
REST_H = 0.020          # body origin height resting on flat ground
CONTACT_SLOP = 0.004    # depenetration residual keeps the core spring-loaded
CONTACT_MARGIN = 0.005  # speculative band: near-contacts reported at depth 0


@dataclass
class StateOut:
    """Decoded PW_STATE_OUT — the core's per-tick truth."""
    frame_id: int
    sim_time_us: int
    quat: tuple            # orientation, (w, x, y, z)
    angular_velocity: tuple  # rad/s
    linear_velocity: tuple   # m/s
    position: tuple          # world m, core-integrated
    acceleration: tuple      # m/s^2
    motor_rpm: tuple
    motor_status: tuple
    armed: int
    arming_disable_flags: int
    flight_mode_flags: int
    beeper: int
    vbat: float
    amperage: float
    prop_damage: tuple
    crash_flags: int

    @property
    def crashed(self) -> bool:
        """bit0 = sim structural-crash latch (cleared only by REPAIR/RESET)."""
        return bool(self.crash_flags & 0x1)


def pack_command(command: int) -> bytes:
    return HDR.pack(MAGIC, VERSION, PW_COMMAND, CMD.size) + CMD.pack(command)


def pack_state_in(frame, dt, rc, pos, rot, angvel, linvel,
                  gyro_noise=0.0002, vbat=16.8, contacts=None,
                  ground_effect=None):
    """Encode one PW_STATE_IN. ``contacts`` is up to 6 tuples
    (point_body(3), normal_world(3), depth, surface)."""
    ge = ground_effect if ground_effect is not None else [0.0] * MOTOR_COUNT
    cts = list(contacts or [])[:MAX_CONTACTS]
    flat = []
    for point, normal, depth, surface in cts:
        flat += [*point, *normal, depth, surface]
    for _ in range(MAX_CONTACTS - len(cts)):
        flat += [0.0] * 7 + [0]
    payload = SIN.pack(frame, dt, *rc, *pos, *rot, *angvel, *linvel,
                       gyro_noise, *([0.0] * MOTOR_COUNT), *ge, vbat,
                       1 if cts else 0, len(cts), *flat)
    return HDR.pack(MAGIC, VERSION, PW_STATE_IN, len(payload)) + payload


def unpack_state_out(payload: bytes) -> StateOut:
    o = SOUT.unpack(payload)
    return StateOut(
        frame_id=o[0], sim_time_us=o[1],
        quat=o[2:6], angular_velocity=o[6:9], linear_velocity=o[9:12],
        position=o[12:15], acceleration=o[15:18],
        motor_rpm=o[18:22], motor_status=o[22:26],
        armed=o[26], arming_disable_flags=o[27], flight_mode_flags=o[28],
        beeper=o[29], vbat=o[30], amperage=o[31],
        prop_damage=o[32:36], crash_flags=o[36],
    )


def _quat_mat(w, x, y, z):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def ground_manifold(pos, rot):
    """Hull-vs-ground (plane y=0) contacts, the way the reference clients build
    them: depenetrate ``pos[1]`` IN PLACE (4 mm slop) and return up to 6
    contacts for :func:`pack_state_in`, including speculative depth-0 contacts
    within 5 mm of the surface. ``rot`` is the sim-frame quaternion (w,x,y,z).
    """
    m = _quat_mat(*rot)
    hits = []
    max_depth = 0.0
    for center, r in HULL:
        cw_y = pos[1] + sum(m[1][j] * center[j] for j in range(3))
        depth = r - cw_y
        if depth > -CONTACT_MARGIN:
            low = tuple(center[j] + m[1][j] * (-r) for j in range(3))
            hits.append((low, depth))
            max_depth = max(max_depth, depth)
    dy = max(0.0, max_depth - CONTACT_SLOP)
    pos[1] += dy
    return [(low, (0.0, 1.0, 0.0), max(depth - dy, 0.0), PW_SURF_GROUND)
            for low, depth in hits[:MAX_CONTACTS]]
