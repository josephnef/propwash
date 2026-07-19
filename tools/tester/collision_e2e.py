#!/usr/bin/env python3
"""Contact/damage end-to-end over the wire protocol: a client-side pose tape
drops the quad onto its own ground manifold and reads the consequences back
from PW_STATE_OUT.

  1.2 m drop  -> frame damage in (0, 0.5], no crash latch
  REPAIR      -> damage cleared
  10 m drop   -> structural crash: all motors >= 0.85, crash_flags bit0
  REPAIR      -> cleared again

Deliberately NOT asserted here: cross-process hash equality. Two fresh cores
still differ through firmware state that BF::init() does not fully clear (a
pre-existing, documented issue — see the determinism_check.py note in
CMakeLists.txt); gating this test on it would only make it flaky. Within one
core the tape is deterministic.

Spawns the core itself. Exit 0 = all phases held.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pw_udp  # noqa: E402

DT = 1.0 / 250.0


def main():
    core = sys.argv[1] if len(sys.argv) > 1 else "build/propwash-core"
    port = 9138
    proc, eeprom = pw_udp.spawn(core, port)
    sock = pw_udp.open_socket()
    addr = ("127.0.0.1", port)

    pos = [0.0, 0.0, 0.0]
    rot = (1.0, 0.0, 0.0, 0.0)
    av = [0.0] * 3
    lv = [0.0] * 3
    rc = [0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0]  # disarmed throughout

    # (time, action) tape: teleports are legal — the client owns position
    T_DROP_A, T_CHECK_A = 2.0, 4.5
    T_REPAIR_A, T_CHECK_RA = 5.0, 5.5
    T_DROP_B, T_CHECK_B = 6.0, 9.5
    T_REPAIR_B, T_END = 10.0, 10.5

    fails = 0

    def check(cond, msg):
        nonlocal fails
        if not cond:
            print("FAIL:", msg)
            fails += 1

    try:
        f = 0
        t = 0.0
        while t < T_END:
            t = f * DT
            if f == int(T_DROP_A / DT):
                pos[1] = 1.2   # ~4.8 m/s impact: damage, not destruction
            if f == int(T_DROP_B / DT):
                pos[1] = 10.0  # ~12 m/s impact: structural crash
            if f in (int(T_REPAIR_A / DT), int(T_REPAIR_B / DT)):
                pw_udp.send_command(sock, addr, pw_udp.PW_CMD_REPAIR)

            contacts = pw_udp.ground_manifold(pos, rot)
            pkt = pw_udp.pack_state_in(f, DT, rc, pos, rot, av, lv,
                                       contact=1 if contacts else 0,
                                       contacts=contacts)
            o = pw_udp.SOUT.unpack(pw_udp.step(sock, addr, pkt))
            rot = tuple(o[2:6])
            av = list(o[6:9])
            lv = list(o[9:12])
            dmg = o[-5:-1]
            flags = o[-1]
            for i in range(3):
                pos[i] += lv[i] * DT

            if f == int(1.5 / DT):
                check(max(dmg) == 0.0 and flags == 0,
                      f"rest damaged: dmg={dmg} flags={flags}")
            if f == int(T_CHECK_A / DT):
                print(f"after 1.2 m drop: dmg={['%.3f' % d for d in dmg]} flags={flags}")
                check(0.0 < max(dmg) <= 0.5,
                      f"1.2 m drop damage {max(dmg):.3f} outside (0, 0.5]")
                check(flags == 0, f"1.2 m drop latched CRASHED (flags={flags})")
            if f == int(T_CHECK_RA / DT):
                check(max(dmg) == 0.0 and flags == 0,
                      f"repair A did not clear: dmg={dmg} flags={flags}")
            if f == int(T_CHECK_B / DT):
                print(f"after 10 m drop:  dmg={['%.3f' % d for d in dmg]} flags={flags}")
                check(min(dmg) >= 0.85,
                      f"10 m drop min damage {min(dmg):.3f} < 0.85")
                check(flags & 1, "10 m drop did not latch CRASHED")
                check(all(s & 2 for s in o[22:26]),
                      f"MotorDamaged not set on all motors: {o[22:26]}")
            f += 1

        # final repair must leave a clean quad
        contacts = pw_udp.ground_manifold(pos, rot)
        pkt = pw_udp.pack_state_in(f, DT, rc, pos, rot, av, lv,
                                   contact=1, contacts=contacts)
        o = pw_udp.SOUT.unpack(pw_udp.step(sock, addr, pkt))
        check(max(o[-5:-1]) == 0.0 and o[-1] == 0,
              f"repair B did not clear: dmg={o[-5:-1]} flags={o[-1]}")

        print("COLLISION E2E PASS" if fails == 0 else f"COLLISION E2E FAIL ({fails})")
        return 0 if fails == 0 else 1
    finally:
        sock.close()
        pw_udp.stop(proc, eeprom)


if __name__ == "__main__":
    sys.exit(main())
