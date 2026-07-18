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
        # Return as soon as the CLI prompt ("# ") comes back — the command has
        # been processed — which throttles the caller so a burst of commands
        # can't overrun the FC's small serial RX ring. That ring overwrites the
        # OLDEST unread bytes, so without this flow control a rapid `diff` apply
        # silently loses its early lines on a slower host (this is exactly why
        # aux/p_pitch didn't persist on the Windows CI runner while the later
        # rateprofile lines did). Fall back to the `settle` idle window when
        # output arrives without a trailing prompt, and cap the total wait so a
        # `save` that reboots (and never prompts) can't hang.
        out = b""
        # The prompt is the primary signal. The idle fallback must be generous:
        # on a slow host (Windows CI) the response trickles in chunks, and a
        # short idle would fire mid-trickle — returning before the prompt and
        # defeating the flow control, which is exactly what dropped the early
        # diff lines. 2 s of true silence means the command produced no prompt
        # (e.g. `save`, which reboots without one).
        idle = max(settle, 2.0)
        hard_end = time.time() + max(settle, 12.0)
        idle_end = time.time() + idle
        while time.time() < hard_end:
            try:
                d = self.sock.recv(4096)
                if not d:
                    break  # connection closed (e.g. after `save` reboots)
                out += d
                if out.endswith(b"# "):
                    break  # prompt is back -> command finished
                idle_end = time.time() + idle  # reset idle window on any data
            except socket.timeout:
                if time.time() >= idle_end:
                    break  # genuinely idle without a prompt
                continue
        return out.decode(errors="replace")

    def enter(self, timeout=8.0):
        # A bare '#' on the MSP serial drops Betaflight into the CLI. After a
        # `save` the FC reboots (re-runs init + gyro calibration) and doesn't
        # service the serial port for a moment; on a slow host (e.g. CI) the
        # banner can lag well past a single fixed drain, so poll for the "# "
        # prompt. Send '#' ONCE and only re-poke after a long gap: on a slow
        # host (Windows CI) an every-0.5 s re-poke queues several extra '# '
        # prompts, and each later cmd() then reads a STALE prompt and returns
        # early — defeating the flow control and dropping commands. After the
        # banner, flush any trailing bytes so the next command starts aligned.
        out = b""
        end = time.time() + timeout
        try:
            self.sock.sendall(b"#\r\n")
        except OSError:
            return ""
        last_poke = time.time()
        while time.time() < end:
            try:
                d = self.sock.recv(4096)
                if not d:
                    break
                out += d
                if b"# " in out:            # CLI prompt reached — ready
                    break
            except socket.timeout:
                if time.time() - last_poke > 3.0:  # rare: first '#' was dropped
                    try:
                        self.sock.sendall(b"#\r\n")
                    except OSError:
                        break
                    last_poke = time.time()
                continue
        self._flush()
        return out.decode(errors="replace")

    def _flush(self):
        # Read and discard anything still pending (duplicate prompts, late
        # banner bytes) until the link goes quiet, so the next command's
        # response can't be confused with leftovers.
        end = time.time() + 0.6
        while time.time() < end:
            try:
                if not self.sock.recv(4096):
                    break
                end = time.time() + 0.3
            except socket.timeout:
                break

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


def diff_commands(path):
    """Applicable CLI lines from a diff file: skip comments, and skip
    `batch start/end` + the trailing `save`. Batch mode makes `save` refuse
    once any line errors (a real FC diff contains settings that don't exist
    in SITL — dshot_bidir, gps_*, board_name, ... — which would otherwise
    block the save). `defaults nosave` is kept so we start from a clean base.
    """
    out = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            low = line.lower()
            if low.startswith("batch ") or low == "save":
                continue
            out.append(line)
    return out


def apply_diff(args):
    cli = Cli(args.host, args.port)
    banner = cli.enter()
    if "CLI" not in banner and "#" not in banner:
        print("warning: no CLI banner seen", file=sys.stderr)

    applied, rejected = 0, []
    for line in diff_commands(args.file):
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


def bake(args):
    """Apply a diff then overrides in ONE CLI session and save once. Order
    matters: overrides (e.g. align_board_roll = 0) must land after the diff
    (align_board_roll = 180) so they win. Rejections of SITL-absent settings
    are warnings, not failures.
    """
    cli = Cli(args.host, args.port)
    cli.enter()
    applied, rejected = 0, 0
    files = [args.diff] + (list(args.overrides) if args.overrides else [])
    for path in files:
        for line in diff_commands(path):
            resp = cli.cmd(line, settle=0.05).lower()
            applied += 1
            if "unknown" in resp or "invalid" in resp or "error" in resp:
                rejected += 1
    cli.cmd("save", settle=1.5)
    cli.close()  # sends exit; save already rebooted so it's a no-op
    print(f"baked: {applied} applied, {rejected} rejected (SITL-absent), saved")
    return 0


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

    bk = sub.add_parser("bake")
    bk.add_argument("diff")
    bk.add_argument("overrides", nargs="*")
    bk.set_defaults(func=bake)

    d = sub.add_parser("dump")
    d.set_defaults(func=dump)

    r = sub.add_parser("run")
    r.add_argument("cmd", nargs="+")
    r.set_defaults(func=run)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
