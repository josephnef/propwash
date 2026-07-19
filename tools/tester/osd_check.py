#!/usr/bin/env python3
"""M3 OSD check: drive propwash-core over the protocol and confirm the real
Betaflight OSD renders — i.e. osd.c/osd_warnings.c write through the fake
max7456 displayport into osdScreen[] and reach the client as PW_OSD packets.

Spawns the core itself. Exit 0 = a non-empty OSD grid was received.
"""
import socket
import sys

import pw_udp


def main():
    core = sys.argv[1] if len(sys.argv) > 1 else "build/propwash-core"
    port = 9112

    proc, eeprom = pw_udp.spawn(core, port)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", port)
        grid = None

        # constant resting pose + ground manifold (the position is a fixed
        # input tape here; the manifold is what keeps the core resting on its
        # contact springs instead of free-falling under a pinned pose)
        pos = [0.0, 0.0, 0.0]
        contacts = pw_udp.ground_manifold(pos, (1.0, 0.0, 0.0, 0.0))

        best = 0
        for f in range(500):
            arm = 1.0 if f > 60 else -1.0
            rc = [0, 0, -1, 0, arm, 1.0, -1, -1]
            pkt = pw_udp.pack_state_in(f, 0.02, rc, pos, (1, 0, 0, 0),
                                       (0, 0, 0), (0, 0, 0), contact=1,
                                       contacts=contacts)
            s.sendto(pkt, addr)
            s.settimeout(0.05)
            try:
                while True:
                    d, _ = s.recvfrom(2048)
                    _, _, typ, _ = pw_udp.HDR.unpack(d[:8])
                    if typ == pw_udp.PW_OSD:
                        g = d[8:8 + 480]
                        # keep the grid with the most content: the OSD refreshes
                        # asynchronously, so any single packet may be mid-clear
                        n = sum(1 for c in g if 32 < c < 127)
                        if n >= best:
                            best = n
                            grid = g
            except socket.timeout:
                pass

        if grid is None:
            print("FAIL: no PW_OSD packet received")
            return 1

        printable = best
        print("=== OSD 16x30 (from real Betaflight OSD) ===")
        for y in range(16):
            row = grid[y * 30:(y + 1) * 30]
            print("|" + "".join(chr(c) if 32 <= c < 127 else " " for c in row) + "|")
        print(f"printable glyphs: {printable}")

        ok = printable >= 5
        print("OSD CHECK PASS" if ok else "OSD CHECK FAIL")
        return 0 if ok else 1
    finally:
        pw_udp.stop(proc, eeprom)


if __name__ == "__main__":
    sys.exit(main())
