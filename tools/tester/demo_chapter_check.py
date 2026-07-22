#!/usr/bin/env python3
"""Run a Godot demo chapter that needs the pilot's REAL tune baked in.

Most demo chapters boot from a default eeprom and need no help. The turtle
chapter does: arming inverted requires `small_angle = 180`, and Betaflight
refuses to arm past 25 degrees of tilt otherwise. That setting lives in the
pilot's own dump (config/cinelog35v3.diff), so the chapter has to fly the real
tune rather than defaults.

This bakes the diff into a temporary eeprom over the CLI — the same thing
real_config_hover_check.py does, and the same thing a pilot does with
tools/bfcli/load_config.sh — then runs the client against it.

Deliberately NOT a committed eeprom.bin. The eeprom is a build product of the
text dump, exactly like client-godot/assets/cinelog35_v3.glb is of the .scad
and extern/bf_sources.txt is of the SITL object list; `*eeprom.bin` is
gitignored for that reason. Baking here keeps the reviewable text as the single
source of truth, so refreshing the tune off the real quad is a diff, not a
binary blob.

Usage:
  demo_chapter_check.py <propwash-core> <godot> <chapter> [port] [scene]
"""
import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bfcli"))
from pw_cli import Cli, diff_commands  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def bake_real_tune(core, eeprom):
    """Apply the pilot's diff + the SITL overrides and save to `eeprom`."""
    diff = os.path.join(ROOT, "config", "cinelog35v3.diff")
    overrides = os.path.join(ROOT, "config", "sitl-overrides.txt")
    loader = subprocess.Popen(
        [core, "--realtime", "--no-js", "--eeprom", eeprom],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        cli = Cli()
        cli.enter()
        n = 0
        for line in diff_commands(diff) + diff_commands(overrides):
            cli.cmd(line, settle=0.03)
            n += 1
        cli.cmd("save", settle=1.5)
        cli.close()
    finally:
        stop(loader)
    time.sleep(0.5)
    return n


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    core, godot, chapter = sys.argv[1], sys.argv[2], sys.argv[3]
    port = sys.argv[4] if len(sys.argv) > 4 else "9153"
    scene = sys.argv[5] if len(sys.argv) > 5 else ""

    eeprom = tempfile.mktemp(suffix=".bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)
    try:
        n = bake_real_tune(core, eeprom)
        print(f"baked the real tune: {n} CLI lines -> {eeprom}")

        env = dict(os.environ)
        env.update({
            "PROPWASH_DEMO": chapter,
            "PROPWASH_EEPROM": eeprom,
            "PROPWASH_CORE": core,
            "PROPWASH_PORT": port,
            # a connected handset outranks scripted RC and would fly the test
            "PROPWASH_NO_JS": "1",
        })
        if scene:
            env["PROPWASH_SCENE"] = scene
        rc = subprocess.call(
            [godot, "--headless", "--path", os.path.join(ROOT, "client-godot")],
            env=env)
        print(f"chapter '{chapter}' exited {rc}")
        return rc
    finally:
        if os.path.exists(eeprom):
            os.remove(eeprom)


if __name__ == "__main__":
    sys.exit(main())
