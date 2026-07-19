#!/usr/bin/env python3
"""Same-process reset determinism: ONE core, the identical input tape played
twice around a PW_CMD_RESET, must produce byte-identical output streams.

This is the narrower, gating tier of the determinism claim. Cross-process
determinism (determinism_check.py) is still broken by a boot-window residue
that survives BF::init(); this test dodges that coin — both tapes run in the
same process, so whatever the boot decided, it decided once — and instead
pins two things at ctest speed:

  1. the whole sim path is a pure function of its inputs (physics, contact
     solver, damage, noise streams — a stray rand()/clock call fails here);
  2. sim.reset() actually resets: if the second tape diverges, state
     accumulated DURING the first tape leaked through the reset.

The tape includes ground contacts and a gate side-scrape, so the contact
solver's friction/torque path and the damage model are covered.

Exit 0 = byte-identical.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pw_udp  # noqa: E402
import determinism_check as dc  # noqa: E402

PORT = 9139


def main():
    if len(sys.argv) < 2:
        print("usage: reset_determinism.py <propwash-core>")
        return 1
    core = sys.argv[1]

    proc, eeprom = pw_udp.spawn(core, PORT)
    sock = pw_udp.open_socket()
    addr = ("127.0.0.1", PORT)
    try:
        h1, f1 = dc.play_tape(sock, addr)
        pw_udp.send_command(sock, addr, pw_udp.PW_CMD_RESET)
        h2, f2 = dc.play_tape(sock, addr)
    finally:
        sock.close()
        pw_udp.stop(proc, eeprom)

    print(f"tape 1: {h1}")
    print(f"tape 2: {h2}")
    if h1 == h2:
        print("reset determinism: PASS (reset restores the exact state)")
        return 0
    pw_udp.report_first_divergence(f1, f2, "tape1", "tape2")
    print("reset determinism: FAIL — state accumulated during tape 1 leaked")
    print("  through sim.reset(). The divergent field above says where.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
