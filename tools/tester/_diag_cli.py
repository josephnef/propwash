#!/usr/bin/env python3
"""TEMPORARY diagnostic: why is the TCP CLI/MSP dead on the macOS CI runner?

Spawns propwash-core in --realtime with stdout/stderr captured, connects to
the CLI (TCP 5761), probes `get p_pitch` with retries, then dumps the core's
stdout (boot log) and stderr (serial_tcp trail: bind port / New connection /
[NEW] / [CLS]). Run standalone: _diag_cli.py <core>
"""
import os
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bfcli"))
from pw_cli import Cli  # noqa: E402


def main():
    core = sys.argv[1]
    eeprom = tempfile.mktemp(suffix=".bin")
    out = tempfile.mktemp(suffix=".out")
    err = tempfile.mktemp(suffix=".err")
    fo, fe = open(out, "wb"), open(err, "wb")
    proc = subprocess.Popen([core, "--realtime", "--no-js", "--eeprom", eeprom],
                            stdout=fo, stderr=fe)
    def probe(tag):
        try:
            raw = socket.create_connection(("127.0.0.1", 5761), timeout=2.0)
            raw.settimeout(1.5)
            raw.sendall(b"#\r\n")
            time.sleep(0.3)
            try:
                b = raw.recv(4096)
            except socket.timeout:
                b = b"(timeout)"
            raw.sendall(b"get p_pitch\r\n")
            time.sleep(0.3)
            try:
                g = raw.recv(4096)
            except socket.timeout:
                g = b"(timeout)"
            raw.close()
            print(f"[diag] {tag}: enter={b[:40]!r}  get={g[:80]!r}")
        except OSError as e:
            print(f"[diag] {tag}: connect failed: {e}")

    try:
        time.sleep(2.0)
        print(f"[diag] core alive={proc.poll() is None}")
        probe("fresh")

        # reproduce diff_roundtrip EXACTLY via pw_cli.Cli: set + save (drain
        # 1.5s, connection open through the in-process reboot), clean close,
        # sleep, reconnect, get. This is where the real tests break.
        cli = Cli()
        cli.enter()
        cli.cmd("set p_pitch = 99", settle=0.1)
        cli.cmd("save", settle=1.5)
        cli.close()
        print("[diag] applied set + save via Cli (like diff_roundtrip)")
        time.sleep(1.5)
        cli = Cli()
        cli.enter()
        r = cli.cmd("get p_pitch", settle=0.4)
        cli.close()
        print(f"[diag] post-save get p_pitch via Cli: {r.strip()[:80]!r}")
        probe("post-save raw")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        fo.close(); fe.close()
        for label, path in (("STDOUT", out), ("STDERR", err)):
            with open(path, "r", errors="replace") as f:
                print(f"\n[diag] core {label}:\n{f.read()}")
        for p in (eeprom, out, err):
            if os.path.exists(p):
                os.remove(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
