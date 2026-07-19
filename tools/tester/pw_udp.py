"""Shared UDP client plumbing for the propwash protocol tests.

The header/payload structs were copy-pasted identically into three test scripts
before this existed; they live here now so the wire format has one definition.

Note the OSD skip in step(): the core interleaves PW_OSD packets on the same
socket, so a naive recv() will hand you an OSD frame where you expected state.
"""
import os
import socket
import struct
import subprocess
import tempfile
import time

MAGIC = 0x48535750
VERSION = 2
PW_STATE_IN, PW_STATE_OUT, PW_OSD = 3, 4, 5
PW_COMMAND = 7
PW_CMD_RESET, PW_CMD_REALTIME, PW_CMD_LOCKSTEP, PW_CMD_REPAIR = 1, 4, 5, 6
CMD = struct.Struct("<I")

MAX_CONTACTS = 6

HDR = struct.Struct("<IBBH")
# frame u32, dt f32, rc 8f, pos 3f, quat wxyz 4f, angvel 3f, linvel 3f,
# gyro noise f, prop_damage 4f, ground_effect 4f, vbat f, contact u8,
# contact_count u8, then 6x contact (point_body 3f, normal_world 3f,
# depth f, surface u8)
SIN = struct.Struct("<If8f3f4f3f3ff4f4ffBB" + "3f3ffB" * MAX_CONTACTS)
# ... beeper u8, vbat f, amperage f, prop_damage 4f, crash_flags u8
SOUT = struct.Struct("<IQ4f3f3f3f3f4f4BBIIBff4fB")
assert SIN.size == 308, SIN.size    # PwStateIn, protocol v2
assert SOUT.size == 131, SOUT.size  # PwStateOut, protocol v2

# Field names for SOUT.unpack()'s flat tuple, used by the determinism
# tooling to say WHICH field diverged instead of just "hashes differ".
SOUT_FIELDS = (
    ["frame_id", "sim_time_us"]
    + ["orientation." + c for c in "wxyz"]
    + ["angular_velocity." + c for c in "xyz"]
    + ["linear_velocity." + c for c in "xyz"]
    + ["position." + c for c in "xyz"]
    + ["acceleration." + c for c in "xyz"]
    + ["motor_rpm[%d]" % i for i in range(4)]
    + ["motor_status[%d]" % i for i in range(4)]
    + ["armed", "arming_disable_flags", "flight_mode_flags", "beeper",
       "vbat", "amperage"]
    + ["prop_damage[%d]" % i for i in range(4)]
    + ["crash_flags"]
)
assert len(SOUT_FIELDS) == len(SOUT.unpack(b"\0" * SOUT.size))


def report_first_divergence(frames_a, frames_b, label_a="A", label_b="B"):
    """Print where two PW_STATE_OUT streams first differ, field by field.
    Returns the first divergent frame index, or -1 if byte-identical.

    Frame 0 divergence means init residue; a late divergence points at
    filter/noise/accumulator state."""
    n = min(len(frames_a), len(frames_b))
    for f in range(n):
        if frames_a[f] == frames_b[f]:
            continue
        a = SOUT.unpack(frames_a[f])
        b = SOUT.unpack(frames_b[f])
        diffs = [i for i in range(len(a)) if a[i] != b[i]]
        print("first divergence at frame %d (sim_t=%.4f s), %d field(s):"
              % (f, a[1] / 1e6, len(diffs)))
        for i in diffs:
            print("  %-24s %s=%r  %s=%r"
                  % (SOUT_FIELDS[i], label_a, a[i], label_b, b[i]))
        return f
    if len(frames_a) != len(frames_b):
        print("streams identical for %d frames but lengths differ (%d vs %d)"
              % (n, len(frames_a), len(frames_b)))
        return n
    return -1


def spawn(core, port, eeprom=None, extra=None, settle=2.0, env=None):
    """Start a propwash-core. Always --no-js: a connected handset has RC
    priority and would silently override the packets we send.

    `env` adds/overrides environment variables (e.g. PROPWASH_DUMP_STATE);
    `settle` is the boot wait — also the lever that decides how many idle
    ticks run before first contact."""
    if eeprom is None:
        eeprom = tempfile.mktemp(suffix=".bin")
    cmd = [core, "--server", "--no-js", "--eeprom", eeprom, "--port", str(port)]
    if extra:
        cmd += extra
    full_env = None
    if env:
        full_env = dict(os.environ)
        full_env.update(env)
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, env=full_env)
    time.sleep(settle)
    return proc, eeprom


def stop(proc, eeprom=None):
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    if eeprom and os.path.exists(eeprom):
        try:
            os.remove(eeprom)
        except OSError:
            pass


def open_socket(timeout=2.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    return sock


def pack_state_in(frame, dt, rc, pos, rot, angvel, linvel,
                  gyro_noise=0.0002, vbat=16.8, contact=0,
                  ground_effect=None, contacts=None):
    """contacts: up to 6 tuples (point_body(3), normal_world(3), depth,
    surface) in the sim frame; the core resolves them as contact forces."""
    ge = ground_effect if ground_effect is not None else [0.0] * 4
    cts = list(contacts or [])[:MAX_CONTACTS]
    flat = []
    for point, normal, depth, surface in cts:
        flat += [*point, *normal, depth, surface]
    for _ in range(MAX_CONTACTS - len(cts)):
        flat += [0.0] * 7 + [0]
    payload = SIN.pack(frame, dt, *rc, *pos, *rot, *angvel, *linvel,
                       gyro_noise, *([0.0] * 4), *ge, vbat, contact,
                       len(cts), *flat)
    return HDR.pack(MAGIC, VERSION, PW_STATE_IN, len(payload)) + payload


# Collision hull, mirroring PW_HULL_* in protocol/propwash_protocol.h:
# belly sphere + 4 duct spheres. Every sender derives its ground manifold
# from these so the core sees one consistent hull.
HULL = [((0.0, 0.030, 0.0), 0.045)] + [
    ((sx * 0.054, 0.010, sz * 0.054), 0.030)
    for sx in (1, -1) for sz in (1, -1)]
CONTACT_SLOP = 0.004    # depenetration residual keeps the core spring loaded
CONTACT_MARGIN = 0.005  # speculative band: near-contacts reported at depth 0
SURF_GROUND = 0


def _quat_mat(w, x, y, z):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def ground_manifold(pos, rot):
    """Hull-vs-ground (plane y=0) contacts the way the real clients build
    them: depenetrates pos[1] IN PLACE (4 mm slop) and returns up to 6
    contacts (point_body, normal_world, depth, surface) for pack_state_in,
    including speculative depth-0 contacts within 5 mm of the surface.
    rot is the sim-frame quaternion (w, x, y, z)."""
    m = _quat_mat(*rot)
    hits = []
    max_depth = 0.0
    for center, r in HULL:
        cw_y = pos[1] + sum(m[1][j] * center[j] for j in range(3))
        depth = r - cw_y
        if depth > -CONTACT_MARGIN:
            # lowest point of the sphere in the body frame: center + R^T*(0,-r,0)
            low = tuple(center[j] + m[1][j] * (-r) for j in range(3))
            hits.append((low, depth))
            max_depth = max(max_depth, depth)
    dy = max(0.0, max_depth - CONTACT_SLOP)
    pos[1] += dy
    return [(low, (0.0, 1.0, 0.0), max(depth - dy, 0.0), SURF_GROUND)
            for low, depth in hits[:6]]


def send_command(sock, addr, command):
    """Send a PW_COMMAND. Used to RESET before a measured run so the session
    starts from a known state: the core idle-ticks while no client is attached
    (that is what keeps the Configurator alive), so how much simulated time has
    already elapsed at first contact depends on process startup timing."""
    payload = CMD.pack(command)
    sock.sendto(HDR.pack(MAGIC, VERSION, PW_COMMAND, len(payload)) + payload, addr)


def step(sock, addr, pkt):
    """Send one PW_STATE_IN and return the raw PW_STATE_OUT payload bytes.

    Returns raw bytes rather than an unpacked tuple so callers can hash the
    exact wire content without float formatting getting in the way.
    """
    sock.sendto(pkt, addr)
    while True:
        data, _ = sock.recvfrom(2048)
        magic, _ver, typ, _plen = HDR.unpack(data[:8])
        if magic == MAGIC and typ == PW_STATE_OUT:
            return data[8:8 + SOUT.size]
