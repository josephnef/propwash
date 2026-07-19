#!/usr/bin/env python3
"""Bisect the reset residue to exact symbols by diffing firmware memory.

Spawns cores with PROPWASH_DUMP_STATE so each writes its writable static
sections to a file right after the first-contact reset, plays the standard
determinism tape to classify the run by its output hash, and keeps going
until it has at least two dumps of each trajectory class. Then, byte by
byte:

    stable WITHIN each class  AND  differing ACROSS classes  ->  residue
    unstable within a class                                   ->  noise
                                             (ASLR pointers self-filter)

Residue offsets are resolved to symbols via `nm -n` — the dump records
UNSLID section addresses, so the mapping ignores ASLR.

Class forcing: half the runs use a short boot settle (calibration still
running at first contact), half the normal one (calibration finished) —
the two sides of the boot-window coin.

usage: state_diff.py <propwash-core> [max_runs]
"""
import os
import subprocess
import sys
import tempfile
from bisect import bisect_right

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pw_udp  # noqa: E402
import determinism_check as dc  # noqa: E402

BASE_PORT = 9160
SETTLES = [2.0, 0.25]  # alternate: calibration done vs still running


def parse_dump(path):
    """-> list of (segname, sectname, unslid_addr, bytes)"""
    sections = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                break
            parts = line.decode("ascii", "replace").split()
            assert parts[0] == "SECT", parts
            seg, sect, addr, size = parts[1], parts[2], int(parts[3], 16), int(parts[4])
            sections.append((seg, sect, addr, f.read(size)))
    return sections


def one_run(core, port, settle):
    dump = tempfile.mktemp(suffix=".pwstate")
    proc, eeprom = pw_udp.spawn(core, port, settle=settle,
                                env={"PROPWASH_DUMP_STATE": dump})
    sock = pw_udp.open_socket()
    addr = ("127.0.0.1", port)
    try:
        digest, _ = dc.play_tape(sock, addr)
    finally:
        sock.close()
        pw_udp.stop(proc, eeprom)
    sections = parse_dump(dump)
    os.remove(dump)
    return digest, sections


def load_symbols(core):
    """nm -n over __DATA-ish symbols -> sorted [(addr, name)]"""
    out = subprocess.run(["nm", "-n", core], capture_output=True, text=True).stdout
    syms = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1].lower() in ("d", "b", "s", "c"):
            syms.append((int(parts[0], 16), parts[2]))
    return syms


def symbolize(syms, addr):
    addrs = [a for a, _ in syms]
    i = bisect_right(addrs, addr) - 1
    if i < 0:
        return "?"
    base, name = syms[i]
    off = addr - base
    return name if off == 0 else "%s+0x%x" % (name, off)


def main():
    core = sys.argv[1] if len(sys.argv) > 1 else "build/propwash-core"
    max_runs = int(sys.argv[2]) if len(sys.argv) > 2 else 12

    classes = {}  # digest -> [sections, ...]
    for i in range(max_runs):
        settle = SETTLES[i % len(SETTLES)]
        digest, sections = one_run(core, BASE_PORT + i, settle)
        classes.setdefault(digest, []).append(sections)
        print("run %2d (settle %.2fs): class %s  (%d so far)"
              % (i, settle, digest[:12], len(classes[digest])))
        counts = sorted(len(v) for v in classes.values())
        if len(classes) >= 2 and counts[-2] >= 2:
            break

    ranked = sorted(classes.items(), key=lambda kv: -len(kv[1]))
    if len(ranked) < 2:
        print("only one class in %d runs — vary SETTLES further" % max_runs)
        return 1
    (dig_a, dumps_a), (dig_b, dumps_b) = ranked[0], ranked[1]
    print("\nclass A %s… x%d   class B %s… x%d"
          % (dig_a[:12], len(dumps_a), dig_b[:12], len(dumps_b)))

    syms = load_symbols(core)
    findings = []
    for si in range(len(dumps_a[0])):
        seg, sect, base, _ = dumps_a[0][si]
        blobs_a = [d[si][3] for d in dumps_a]
        blobs_b = [d[si][3] for d in dumps_b]
        size = min(min(map(len, blobs_a)), min(map(len, blobs_b)))
        run_start = -1
        for off in range(size + 1):
            residue = False
            if off < size:
                a0 = blobs_a[0][off]
                b0 = blobs_b[0][off]
                stable_a = all(b[off] == a0 for b in blobs_a)
                stable_b = all(b[off] == b0 for b in blobs_b)
                residue = stable_a and stable_b and a0 != b0
            if residue and run_start < 0:
                run_start = off
            elif not residue and run_start >= 0:
                findings.append((seg, sect, base + run_start, off - run_start,
                                 bytes(blobs_a[0][run_start:off]),
                                 bytes(blobs_b[0][run_start:off])))
                run_start = -1

    print("\n%d residue range(s) — stable within class, differing across:\n"
          % len(findings))
    for seg, sect, addr, size, va, vb in findings:
        print("  %-28s %s,%s +%d bytes" % (symbolize(syms, addr), seg.strip("_"),
                                           sect, size))
        print("      A=%s  B=%s" % (va[:16].hex(), vb[:16].hex()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
