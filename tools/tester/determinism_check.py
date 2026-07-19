#!/usr/bin/env python3
"""Client-driven determinism: identical input sequences must produce identical
output, byte for byte.

This is the claim the whole project rests on — docs/ARCHITECTURE.md says
"Identical inputs => identical trajectories" — and until now nothing tested it
through the UDP protocol. The in-process pw-tester only printed a hash that
nothing compared.

It was also not true. Three identical runs of the Godot demo used to end 20 cm
apart, because the core injected a 5 ms idle tick on every socket recv timeout,
including any packet the client was late sending. That is what this guards.

Two FRESH processes rather than one process with a RESET between: this way the
test does not depend on how complete PW_CMD_RESET happens to be.

Nothing may be attached to TCP 5761 while this runs — the dyad thread reads
gettimeofday(), so attached MSP/CLI traffic makes byte arrival wall-clock
dependent and would legitimately perturb the firmware.
"""
import hashlib
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pw_udp  # noqa: E402

DT = 1.0 / 250.0        # divides exactly into the core's 50 us tick quantum
FRAMES = 1500           # 6 s of sim; long enough to arm and fly
PORT_A, PORT_B = 9131, 9132


def run(core, port, jitter):
    """Fly a fixed input sequence and hash every output frame.

    `jitter` deliberately delays some sends past the core's 5 ms recv timeout.
    That used to make the core inject idle ticks and diverge; with lockstep it
    must make no difference at all, which is precisely what this asserts.
    """
    proc, eeprom = pw_udp.spawn(core, port)
    sock = pw_udp.open_socket()
    addr = ("127.0.0.1", port)
    digest = hashlib.sha256()
    try:
        # No explicit RESET needed: the core discards its pre-client idle time
        # on first contact, so a session always starts from the same physics
        # state regardless of when the client happened to attach.
        pos = [0.0, 0.12, 0.0]
        for f in range(FRAMES):
            t = f * DT
            # deterministic, scripted stick input — no wall clock anywhere
            throttle = -1.0 if t < 1.0 else (-0.15 if t < 2.0 else -0.2)
            arm = 1.0 if t >= 1.0 else -1.0
            rc = [0.0, 0.0, throttle, 0.0, arm, 1.0, -1.0, -1.0]
            pkt = pw_udp.pack_state_in(
                f, DT, rc, pos, [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                contact=1 if pos[1] <= 0.13 else 0)
            if jitter and f % 97 == 0:
                # longer than the core's 5 ms recv timeout, on purpose
                time.sleep(0.012)
            out = pw_udp.step(sock, addr, pkt)
            digest.update(out)
    finally:
        sock.close()
        pw_udp.stop(proc, eeprom)
    return digest.hexdigest()


def main():
    if len(sys.argv) < 2:
        print("usage: determinism_check.py <propwash-core>")
        return 1
    core = sys.argv[1]

    print(f"run A: {FRAMES} frames at dt={DT:.6f}, steady send rate")
    a = run(core, PORT_A, jitter=False)
    print(f"  sha256 {a}")

    print(f"run B: same inputs, but some sends delayed past the 5 ms timeout")
    b = run(core, PORT_B, jitter=True)
    print(f"  sha256 {b}")

    if a == b:
        print("determinism: PASS (identical inputs -> identical outputs)")
        return 0
    print("determinism: FAIL — same inputs produced different trajectories.")
    print("  The usual cause is simulated time advancing from something other")
    print("  than PW_STATE_IN: check the idle tick in Server::run and that")
    print("  lockstep mode is active.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
