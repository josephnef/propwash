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
VERSION = 1
PW_STATE_IN, PW_STATE_OUT, PW_OSD = 3, 4, 5
PW_COMMAND = 7
PW_CMD_RESET, PW_CMD_REALTIME, PW_CMD_LOCKSTEP = 1, 4, 5
CMD = struct.Struct("<I")

HDR = struct.Struct("<IBBH")
# frame u32, dt f32, rc 8f, pos 3f, quat wxyz 4f, angvel 3f, linvel 3f,
# gyro noise f, 4f, 4f, vbat f, contact u8
SIN = struct.Struct("<If8f3f4f3f3ff4f4ffB")
SOUT = struct.Struct("<IQ4f3f3f3f3f4f4BBIIBff")


def spawn(core, port, eeprom=None, extra=None, settle=2.0):
    """Start a propwash-core. Always --no-js: a connected handset has RC
    priority and would silently override the packets we send."""
    if eeprom is None:
        eeprom = tempfile.mktemp(suffix=".bin")
    cmd = [core, "--server", "--no-js", "--eeprom", eeprom, "--port", str(port)]
    if extra:
        cmd += extra
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
                  gyro_noise=0.0002, vbat=16.8, contact=0):
    payload = SIN.pack(frame, dt, *rc, *pos, *rot, *angvel, *linvel,
                       gyro_noise, *([0.0] * 4), *([0.0] * 4), vbat, contact)
    return HDR.pack(MAGIC, VERSION, PW_STATE_IN, len(payload)) + payload


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
