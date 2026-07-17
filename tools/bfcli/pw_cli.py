#!/usr/bin/env python3
"""Betaflight CLI over propwash-core's TCP UART1 (port 5761).

The same text CLI the Betaflight Configurator drives — used here to load the
pilot's real `diff all` into the in-process firmware and read it back, with
no GUI and no hardware.

Usage:
  pw_cli.py apply  <diff-file> [--host H] [--port P] [--save]
  pw_cli.py dump   [--host H] [--port P]          # prints `diff all`
  pw_cli.py run    "<cli command>" [...]

`save` reboots the FC; propwash-core re-runs init() in-process and the
config persists in eeprom.bin, so a fresh connection reads the new config.
"""
import argparse
import socket
import sys
import time


class Cli:
    def __init__(self, host="127.0.0.1", port=5761, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        # short recv timeout so _drain polls at the settle granularity rather
        # than blocking for the whole connection timeout after the last byte
        self.sock.settimeout(0.05)

    def _drain(self, settle=0.25):
        out = b""
        end = time.time() + settle
        while time.time() < end:
            try:
                d = self.sock.recv(4096)
                if not d:
                    break
                out += d
                end = time.time() + settle  # keep reading while data flows
            except socket.timeout:
                continue  # no data this poll; keep waiting until `settle`
        return out.decode(errors="replace")

    def enter(self):
        # a bare '#' on the MSP serial drops Betaflight into the CLI
        self.sock.sendall(b"#\r\n")
        return self._drain()

    def cmd(self, line, settle=0.25):
        self.sock.sendall(line.encode() + b"\r\n")
        return self._drain(settle)

    def close(self, leave=True):
        # leave CLI mode so the firmware clears ARMING_DISABLED_CLI; without
        # this the FC stays "in CLI" after the socket closes and won't arm
        if leave:
            try:
                self.sock.sendall(b"exit\r\n")
                self._drain(0.2)
            except OSError:
                pass  # already rebooted (e.g. after `save`)
        self.sock.close()


def apply_diff(args):
    cli = Cli(args.host, args.port)
    banner = cli.enter()
    if "CLI" not in banner and "#" not in banner:
        print("warning: no CLI banner seen", file=sys.stderr)

    applied, rejected = 0, []
    with open(args.file) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            resp = cli.cmd(line, settle=0.1)
            applied += 1
            low = resp.lower()
            if "unknown" in low or "invalid" in low or "error" in low:
                rejected.append((line, resp.strip()))

    if args.save:
        print("saving (FC reboots)...")
        cli.cmd("save", settle=1.0)

    cli.close()
    print(f"applied {applied} command(s); {len(rejected)} rejected")
    for line, resp in rejected:
        print(f"  REJECTED: {line}\n            -> {resp[:120]}")
    return 1 if rejected else 0


def dump(args):
    cli = Cli(args.host, args.port)
    cli.enter()
    out = cli.cmd("diff all", settle=1.0)
    cli.close()
    sys.stdout.write(out)
    return 0


def run(args):
    cli = Cli(args.host, args.port)
    cli.enter()
    for c in args.cmd:
        sys.stdout.write(cli.cmd(c, settle=0.4))
    cli.close()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5761)
    sub = ap.add_subparsers(dest="mode", required=True)

    a = sub.add_parser("apply")
    a.add_argument("file")
    a.add_argument("--save", action="store_true")
    a.set_defaults(func=apply_diff)

    d = sub.add_parser("dump")
    d.set_defaults(func=dump)

    r = sub.add_parser("run")
    r.add_argument("cmd", nargs="+")
    r.set_defaults(func=run)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
