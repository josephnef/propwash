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


def play_tape(sock, addr, jitter=False, frames=FRAMES):
    """Send the fixed input tape and collect every output frame.

    The pose is a fixed TAPE (never integrated from replies): a resting
    ground manifold every frame, plus a scripted gate side-scrape over
    frames 900-950 so the run also covers the contact solver's
    friction/torque path and the damage model. No wall clock anywhere.

    Returns (sha256 hexdigest, list of raw PW_STATE_OUT payloads) — the raw
    frames are what report_first_divergence() localizes a mismatch with.
    Also used by reset_determinism.py, which replays this exact tape twice
    in one process around a PW_CMD_RESET.
    """
    pos = [0.0, 0.0, 0.0]
    rest = pw_udp.ground_manifold(pos, (1.0, 0.0, 0.0, 0.0))
    scrape = ((0.054, 0.02, 0.054), (-1.0, 0.0, 0.0), 0.008, 1)
    digest = hashlib.sha256()
    raw = []
    for f in range(frames):
        t = f * DT
        # deterministic, scripted stick input
        throttle = -1.0 if t < 1.0 else (-0.15 if t < 2.0 else -0.2)
        arm = 1.0 if t >= 1.0 else -1.0
        rc = [0.0, 0.0, throttle, 0.0, arm, 1.0, -1.0, -1.0]
        contacts = rest + ([scrape] if 900 <= f < 950 else [])
        pkt = pw_udp.pack_state_in(
            f, DT, rc, pos, [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
            contact=1, contacts=contacts)
        if jitter and f % 97 == 0:
            # longer than the core's 5 ms recv timeout, on purpose
            time.sleep(0.012)
        out = pw_udp.step(sock, addr, pkt)
        digest.update(out)
        raw.append(out)
    return digest.hexdigest(), raw


def run(core, port, jitter):
    """One fresh core, one tape. Returns (hexdigest, raw frames).

    No explicit RESET needed: the core discards its pre-client idle time on
    first contact, so a session always starts from the same physics state
    regardless of when the client happened to attach.
    """
    proc, eeprom = pw_udp.spawn(core, port)
    sock = pw_udp.open_socket()
    addr = ("127.0.0.1", port)
    try:
        return play_tape(sock, addr, jitter=jitter)
    finally:
        sock.close()
        pw_udp.stop(proc, eeprom)


def run_histogram(core, n):
    """N fresh cores, identical tapes, no jitter: the class histogram of the
    boot-window coin. A deterministic sim prints one class with count N."""
    classes = {}
    for i in range(n):
        h, _ = run(core, PORT_A + i, jitter=False)
        classes.setdefault(h, []).append(i)
        print(f"  run {i}: {h[:16]}…")
    print(f"\n{len(classes)} distinct trajectory class(es) over {n} runs:")
    for h, runs in sorted(classes.items(), key=lambda kv: -len(kv[1])):
        print(f"  {h[:16]}…  x{len(runs)}  (runs {runs})")
    return 0 if len(classes) == 1 else 1


def main():
    if len(sys.argv) < 2:
        print("usage: determinism_check.py <propwash-core> [--runs N]")
        return 1
    core = sys.argv[1]

    if "--runs" in sys.argv:
        n = int(sys.argv[sys.argv.index("--runs") + 1])
        return run_histogram(core, n)

    print(f"run A: {FRAMES} frames at dt={DT:.6f}, steady send rate")
    a, fa = run(core, PORT_A, jitter=False)
    print(f"  sha256 {a}")

    print(f"run B: same inputs, but some sends delayed past the 5 ms timeout")
    b, fb = run(core, PORT_B, jitter=True)
    print(f"  sha256 {b}")

    if a == b:
        print("determinism: PASS (identical inputs -> identical outputs)")
        return 0
    print("determinism: FAIL — same inputs produced different trajectories.")
    pw_udp.report_first_divergence(fa, fb, "A", "B")
    print("  Frame-0 divergence = init residue surviving the first-contact")
    print("  reset; a late divergence points at filter/noise state. If time")
    print("  itself differs, check the idle tick in Server::run / lockstep.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
