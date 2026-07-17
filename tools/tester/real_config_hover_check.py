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
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bfcli"))
from pw_cli import Cli, diff_commands  # noqa: E402

MAGIC = 0x48535750
HDR = struct.Struct("<IBBH")
SIN = struct.Struct("<If8f3f4f3f3ff4f4ffB")
SOUT = struct.Struct("<IQ4f3f3f3f3f4f4BBIIBff")
PW_STATE_IN, PW_STATE_OUT = 3, 4


def spawn(core, mode, eeprom, port=None, errpath=None):
    args = [core, mode, "--no-js", "--eeprom", eeprom]
    if port:
        args += ["--port", str(port)]
    err = open(errpath, "wb") if errpath else subprocess.DEVNULL
    out = open(errpath + ".out", "wb") if errpath else subprocess.DEVNULL
    return subprocess.Popen(args, stdout=out, stderr=err)


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
        loerr = tempfile.mktemp(suffix=".loerr")
        loader = spawn(core, "--realtime", eeprom, errpath=loerr)
        time.sleep(2.0)
        cli = Cli()
        lbanner = cli.enter()
        lprobe = cli.cmd("get p_pitch", settle=0.4)   # does the LOADER CLI work?
        print(f"[diag] loader enter={len(lbanner)}B  get p_pitch={lprobe.strip()[:50]!r}")
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
        rberr = tempfile.mktemp(suffix=".rberr")
        rb = spawn(core, "--realtime", eeprom, errpath=rberr)
        time.sleep(2.0)
        cli = Cli()
        banner = cli.enter()
        pid = cli.cmd("get p_pitch", settle=0.4).replace("\r", "")
        align = cli.cmd("get align_board_roll", settle=0.4).replace("\r", "")
        cli.close()
        pline = next((l for l in pid.split("\n") if "p_pitch =" in l), "")
        aline = next((l for l in align.split("\n") if "align_board_roll =" in l), "")
        print(f"2. fresh boot from eeprom: {pline.strip()} | {aline.strip()}")
        if "53" not in pline:
            print("FAIL: eeprom did not persist the real tune (p_pitch != 53)")
            # -------- DIAGNOSTIC (temporary) --------
            print(f"[diag] rb alive={rb.poll() is None}  eeprom={eeprom} "
                  f"size={os.path.getsize(eeprom) if os.path.exists(eeprom) else 'MISSING'}")
            print(f"[diag] enter banner ({len(banner)}B): {banner!r}")
            print(f"[diag] raw p_pitch ({len(pid)}B): {pid!r}")
            print(f"[diag] raw align  ({len(align)}B): {align!r}")
            # retry probe: is the CLI merely slow to come up, or permanently dead?
            for attempt in range(6):
                time.sleep(1.5)
                try:
                    c2 = Cli()
                    b2 = c2.enter()
                    r2 = c2.cmd("get p_pitch", settle=0.6)
                    c2.close()
                    print(f"[diag] retry {attempt}: banner={len(b2)}B resp={r2.strip()[:60]!r}")
                    if "p_pitch" in r2:
                        break
                except OSError as e:
                    print(f"[diag] retry {attempt}: connect failed: {e}")
            for suffix, label in ((".out", "stdout"), ("", "stderr")):
                p = rberr + suffix
                if os.path.exists(p):
                    with open(p, "r", errors="replace") as f:
                        print(f"[diag] readback core {label}:\n{f.read()[-2000:]}")
            stop(rb)
            time.sleep(0.5)
            return 1
        stop(rb)
        time.sleep(0.5)
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
            contact = 1 if pos[1] <= 0.001 else 0
            payload = SIN.pack(f, dt, *rc, *pos, *rot, *angvel, *linvel,
                               0.0002, *([0] * 4), *([0] * 4), 16.8, contact)
            s.sendto(HDR.pack(MAGIC, 1, PW_STATE_IN, len(payload)) + payload, addr)
            while True:
                d, _ = s.recvfrom(2048)
                if HDR.unpack(d[:8])[2] == PW_STATE_OUT:
                    break
            o = SOUT.unpack(d[8:8 + SOUT.size])
            (_, _, qw, qx, qy, qz, avx, avy, avz, lvx, lvy, lvz,
             px, py, pz, ax, ay, az, r0, r1, r2, r3, s0, s1, s2, s3,
             armed, dis, mode, beep, vbat, amps) = o
            rot = (qw, qx, qy, qz)
            angvel = [avx, avy, avz]
            linvel = [lvx, lvy, lvz]
            pos = [pos[i] + linvel[i] * dt for i in range(3)]
            if pos[1] <= 0.0:
                pos[1] = 0.0
                if linvel[1] < 0.0:
                    linvel[1] = 0.0
                angvel = [0.0] * 3
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
