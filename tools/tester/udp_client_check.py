#!/usr/bin/env python3
"""End-to-end M2 check: drives propwash-core over the UDP protocol exactly
like the Godot client does (lockstep PW_STATE_IN/PW_STATE_OUT, client-side
pose integration and ground plane), arms and hovers.

Start the core first:  ./build/propwash-core --no-js --eeprom /tmp/e2e.bin
Then:                  python3 tools/tester/udp_client_check.py

Exit 0 = armed + hovered + latency sane.
"""
import math
import socket
import struct
import sys
import time

MAGIC = 0x48535750
VERSION = 1
PW_STATE_IN, PW_STATE_OUT = 3, 4

HDR = struct.Struct("<IBBH")
# frame u32, dt f32, rc 8f, pos 3f, quat wxyz 4f, angvel 3f, linvel 3f,
# noise f32, damage 4f, ground 4f, vbat f32, contact u8
SIN = struct.Struct("<If8f3f4f3f3ff4f4ffB")
SOUT = struct.Struct("<IQ4f3f3f3f3f4f4BBIIBff")


def quat_to_up(w, x, y, z):
    # body-up (0,1,0) rotated by quat -> world; return world y component
    return 1 - 2 * (x * x + z * z)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    addr = ("127.0.0.1", 9100)

    dt = 1.0 / 100.0
    pos = [0.0, 0.0, 0.0]
    rot = (1.0, 0.0, 0.0, 0.0)  # w,x,y,z
    angvel = [0.0] * 3
    linvel = [0.0] * 3
    throttle = -1.0
    armed_seen = False
    min_up = 1.0
    alts = []
    latencies = []

    T_ARM, T_END, TARGET = 5.2, 20.0, 2.0
    frames = int(T_END / dt)

    for f in range(frames):
        t = f * dt
        arm = 1.0 if t >= T_ARM else -1.0

        rc = [0.0, 0.0, throttle, 0.0, arm, 1.0, -1.0, -1.0]
        contact = 1 if pos[1] <= 0.001 else 0

        payload = SIN.pack(f, dt, *rc, *pos, *rot, *angvel, *linvel,
                           0.0002, *([0.0] * 4), *([0.0] * 4), 16.8, contact)
        pkt = HDR.pack(MAGIC, VERSION, PW_STATE_IN, len(payload)) + payload

        t0 = time.perf_counter()
        sock.sendto(pkt, addr)
        # the server also emits PW_OSD packets; skip anything but STATE_OUT
        while True:
            data, _ = sock.recvfrom(2048)
            magic, ver, typ, plen = HDR.unpack(data[:8])
            if magic == MAGIC and typ == PW_STATE_OUT:
                break
        latencies.append(time.perf_counter() - t0)
        o = SOUT.unpack(data[8:8 + SOUT.size])
        (frame_id, sim_us,
         qw, qx, qy, qz,
         avx, avy, avz, lvx, lvy, lvz,
         px, py, pz, ax, ay, az,
         r0, r1, r2, r3, s0, s1, s2, s3,
         armed, dis, mode, beep, vbat, amps) = o

        rot = (qw, qx, qy, qz)
        angvel = [avx, avy, avz]
        linvel = [lvx, lvy, lvz]
        pos = [pos[i] + linvel[i] * dt for i in range(3)]
        if pos[1] <= 0.0:
            pos[1] = 0.0
            if linvel[1] < 0.0:
                linvel[1] = 0.0
            angvel = [0.0] * 3

        if armed:
            armed_seen = True
            u = -0.3 + 0.5 * (TARGET - pos[1]) - 0.4 * linvel[1]
            u = max(-1.0, min(0.6, u))
            step = 2.0 * dt
            throttle = max(throttle - step, min(throttle + step, u))
        else:
            throttle = -1.0

        if t >= T_ARM + 8.0:
            alts.append(pos[1])
            min_up = min(min_up, quat_to_up(qw, qx, qy, qz))

        if f % 200 == 0:
            print(f"t={t:5.1f} alt={pos[1]:5.2f} thr={throttle:5.2f} "
                  f"armed={armed} dis=0x{dis:x} vbat={vbat:4.1f}")

    lat_ms = sorted(latencies)
    p50 = lat_ms[len(lat_ms) // 2] * 1000
    p99 = lat_ms[int(len(lat_ms) * 0.99)] * 1000
    tilt_deg = math.degrees(math.acos(max(-1, min(1, min_up))))
    print(f"\nlatency p50={p50:.2f} ms p99={p99:.2f} ms (send->reply, incl. sim step)")
    print(f"hover window: alt [{min(alts):.2f}, {max(alts):.2f}] target {TARGET}, max tilt {tilt_deg:.2f} deg")

    ok = (armed_seen
          and abs(sum(alts) / len(alts) - TARGET) < 0.5
          and max(alts) - min(alts) < 1.0
          and tilt_deg < 5.0
          and p99 < 50.0)
    print("E2E CHECK PASS" if ok else "E2E CHECK FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
