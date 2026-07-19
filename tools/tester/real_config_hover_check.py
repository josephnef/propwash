#!/usr/bin/env python3
"""The M3 payoff check: bake the pilot's REAL diff into an eeprom, prove a
fresh instance boots it, then fly a hover with the pilot's actual PIDs/rates.

Three short-lived instances (each uses a proven pattern, no CLI-vs-arm clash):
  1. loader   (--realtime): apply diff + overrides over the CLI, save.
  2. readback (--realtime): fresh boot, CLI `get p_pitch` == 53 (real PID).
  3. fly      (--server):   fresh boot, NO CLI, drive an angle-mode hover.

Exit 0 = the real tune persists and hovers.
"""
import math
import os
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bfcli"))
from pw_cli import Cli, diff_commands  # noqa: E402

import pw_udp  # noqa: E402


def spawn(core, mode, eeprom, port=None):
    args = [core, mode, "--no-js", "--eeprom", eeprom]
    if port:
        args += ["--port", str(port)]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    root = os.path.join(os.path.dirname(__file__), "..", "..")
    core = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "build/propwash-core")
    diff = os.path.join(root, "config/cinelog35v3.diff")
    overrides = os.path.join(root, "config/sitl-overrides.txt")
    port = 9113
    eeprom = tempfile.mktemp(suffix=".bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    try:
        # ---- 1. bake the real tune into the eeprom
        loader = spawn(core, "--realtime", eeprom)
        time.sleep(2.0)
        cli = Cli()
        cli.enter()
        n = 0
        for line in diff_commands(diff) + diff_commands(overrides):
            cli.cmd(line, settle=0.03)
            n += 1
        cli.cmd("save", settle=1.5)
        cli.close()
        stop(loader)
        time.sleep(0.5)
        print(f"1. baked real tune: {n} CLI lines saved to eeprom")

        # ---- 2. fresh instance proves the eeprom holds the real tune
        rb = spawn(core, "--realtime", eeprom)
        time.sleep(2.0)
        cli = Cli()
        cli.enter()
        pid = cli.cmd("get p_pitch", settle=0.4).replace("\r", "")
        align = cli.cmd("get align_board_roll", settle=0.4).replace("\r", "")
        cli.close()
        stop(rb)
        time.sleep(0.5)
        pline = next((l for l in pid.split("\n") if "p_pitch =" in l), "")
        aline = next((l for l in align.split("\n") if "align_board_roll =" in l), "")
        print(f"2. fresh boot from eeprom: {pline.strip()} | {aline.strip()}")
        if "53" not in pline:
            print("FAIL: eeprom did not persist the real tune (p_pitch != 53)")
            return 1
        if "= 0" not in aline:
            print("FAIL: align_board_roll not neutralised (would fly inverted)")
            return 1

        # ---- 3. fly a hover with the pilot's PIDs (no CLI on this instance)
        fly = spawn(core, "--server", eeprom, port)
        time.sleep(2.0)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        addr = ("127.0.0.1", port)
        dt = 1.0 / 250.0
        pos = [0.0, 0.0, 0.0]
        rot = (1.0, 0.0, 0.0, 0.0)
        angvel = [0.0] * 3
        linvel = [0.0] * 3
        throttle = -1.0
        armed_seen = False
        alts, tilts = [], []
        T_ARM, T_END, TARGET = 5.2, 20.0, 2.0

        for f in range(int(T_END / dt)):
            t = f * dt
            arm = 1.0 if t >= T_ARM else -1.0
            rc = [0, 0, throttle, 0, arm, 1.0, -1, -1]
            # hull-vs-ground manifold; depenetrates pos in place
            contacts = pw_udp.ground_manifold(pos, rot)
            pkt = pw_udp.pack_state_in(f, dt, rc, pos, rot, angvel, linvel,
                                       contact=1 if contacts else 0,
                                       contacts=contacts)
            out = pw_udp.step(s, addr, pkt)
            o = pw_udp.SOUT.unpack(out)
            (_, _, qw, qx, qy, qz, avx, avy, avz, lvx, lvy, lvz,
             px, py, pz, ax, ay, az, r0, r1, r2, r3, s0, s1, s2, s3,
             armed, dis, mode, beep, vbat, amps,
             d0, d1, d2, d3, cflags) = o
            rot = (qw, qx, qy, qz)
            angvel = [avx, avy, avz]
            linvel = [lvx, lvy, lvz]
            pos = [pos[i] + linvel[i] * dt for i in range(3)]
            if armed:
                armed_seen = True
                u = -0.3 + 0.5 * (TARGET - pos[1]) - 0.4 * linvel[1]
                u = max(-1.0, min(0.6, u))
                throttle = max(throttle - 2.0 * dt, min(throttle + 2.0 * dt, u))
            else:
                throttle = -1.0
            if t >= T_ARM + 8.0:
                alts.append(pos[1])
                tilts.append(1 - 2 * (qx * qx + qz * qz))
            if f % 500 == 0:
                print(f"   t={t:5.1f} alt={pos[1]:5.2f} thr={throttle:5.2f} armed={armed} dis=0x{dis:x} vbat={vbat:.1f}")
        stop(fly)

        lo, hi = min(alts), max(alts)
        tilt_deg = math.degrees(math.acos(max(-1, min(1, min(tilts)))))
        print(f"3. hover: alt [{lo:.2f}, {hi:.2f}] target {TARGET}, max tilt {tilt_deg:.2f} deg")
        ok = (armed_seen and abs(sum(alts) / len(alts) - TARGET) < 0.6
              and hi - lo < 1.2 and tilt_deg < 8.0)
        print("REAL-TUNE HOVER PASS" if ok else "REAL-TUNE HOVER FAIL")
        return 0 if ok else 1
    finally:
        if os.path.exists(eeprom):
            os.remove(eeprom)


if __name__ == "__main__":
    sys.exit(main())
