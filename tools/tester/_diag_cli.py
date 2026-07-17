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
    try:
        time.sleep(2.0)
        print(f"[diag] core alive={proc.poll() is None}")
        # can we even open a raw TCP socket to 5761?
        for attempt in range(8):
            try:
                raw = socket.create_connection(("127.0.0.1", 5761), timeout=2.0)
                print(f"[diag] attempt {attempt}: raw TCP connect OK")
                raw.settimeout(1.0)
                raw.sendall(b"#\r\n")
                try:
                    data = raw.recv(4096)
                    print(f"[diag]   after '#': {len(data)}B {data[:80]!r}")
                except socket.timeout:
                    print("[diag]   after '#': (timeout, no bytes)")
                raw.sendall(b"get p_pitch\r\n")
                try:
                    data = raw.recv(4096)
                    print(f"[diag]   get p_pitch: {data[:120]!r}")
                except socket.timeout:
                    print("[diag]   get p_pitch: (timeout, no bytes)")
                raw.close()
            except OSError as e:
                print(f"[diag] attempt {attempt}: connect failed: {e}")
            time.sleep(1.0)
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
