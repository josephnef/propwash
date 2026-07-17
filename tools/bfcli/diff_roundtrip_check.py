#!/usr/bin/env python3
"""M3 check: load the CineLog35 diff into the in-process firmware over the
CLI (TCP 5761 = the Configurator data path), save, reboot, and verify the
settings survive a fresh connection — i.e. the pilot's real tune is what the
sim flies.

Spawns propwash-core itself (like run_e2e.sh), so it needs no external setup.

Usage: diff_roundtrip_check.py <propwash-core> <diff-file> [<overrides-file>]
Exit 0 = all probed settings read back as written.
"""
import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(__file__))
from pw_cli import Cli  # noqa: E402


def load_diff_lines(path):
    lines = []
    with open(path) as f:
        for raw in f:
            s = raw.strip()
            if s and not s.startswith("#"):
                lines.append(s)
    return lines


def main():
    if len(sys.argv) < 3:
        print("usage: diff_roundtrip_check.py <core> <diff> [overrides]", file=sys.stderr)
        return 2
    core, diff = sys.argv[1], sys.argv[2]
    overrides = sys.argv[3] if len(sys.argv) > 3 else None

    eeprom = tempfile.mktemp(suffix=".bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    proc = subprocess.Popen([core, "--realtime", "--no-js", "--eeprom", eeprom],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(2.0)

        # ---- apply diff + overrides over the CLI, then save (reboots)
        cli = Cli()
        cli.enter()
        applied = 0
        for line in load_diff_lines(diff) + (load_diff_lines(overrides) if overrides else []):
            cli.cmd(line, settle=0.05)
            applied += 1
        cli.cmd("save", settle=1.5)
        cli.close()
        print(f"applied {applied} CLI command(s), saved")

        # firmware rebooted in-process; give it a moment
        time.sleep(1.5)

        # ---- fresh connection: read back a representative set of settings
        cli = Cli()
        cli.enter()

        # probe: (cli 'get' name, expected substring)
        probes = [
            ("failsafe_procedure", "DROP"),
            ("small_angle", "180"),
            ("yaw_deadband", None),   # just must exist
        ]
        results = {}
        for name, _ in probes:
            resp = cli.cmd(f"get {name}", settle=0.3)
            results[name] = resp

        rates_type_get = cli.cmd("get rates_type", settle=0.3)
        diff_out = cli.cmd("diff all", settle=1.5)
        cli.close()

        ok = True
        for name, expect in probes:
            r = results[name]
            got = "= " in r or ":" in r
            if not got:
                print(f"FAIL: `get {name}` returned nothing:\n{r}")
                ok = False
            elif expect and expect not in r:
                print(f"FAIL: {name} expected '{expect}', got: {r.strip()[:80]}")
                ok = False
            else:
                # extract the value line for display
                val = next((ln for ln in r.splitlines() if name in ln), r.strip()[:60])
                print(f"ok  : {val.strip()}")

        # the aux switch plan (ARM ch5 / ANGLE ch6) must survive the reboot
        for needle in ("aux 0 0 0", "aux 1 1 1"):
            if needle in diff_out:
                print(f"ok  : switch plan present ({needle}...)")
            else:
                print(f"FAIL: expected '{needle}...' in diff all")
                ok = False

        # a NON-default value from the diff (not something configureDefaultModes
        # sets, not a firmware default) must survive the reboot -> proves diff
        # *content* round-tripped through the eeprom, not just the RAM aux.
        rt_line = next((ln for ln in rates_type_get.splitlines()
                        if "rates_type" in ln), rates_type_get.strip())
        print(f"note: rates_type after reboot -> {rt_line.strip()[:80]}")
        if "ACTUAL" in rates_type_get:
            print("ok  : diff content persisted (rates_type = ACTUAL)")
        else:
            print("FAIL: rates_type did not persist as ACTUAL")
            ok = False

        print("DIFF ROUNDTRIP PASS" if ok else "DIFF ROUNDTRIP FAIL")
        return 0 if ok else 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        if os.path.exists(eeprom):
            os.remove(eeprom)


if __name__ == "__main__":
    sys.exit(main())
