#!/usr/bin/env python3
"""Pin the two independent Python wire codecs against drift (issue #22).

tools/tester/pw_udp.py (GPL test tree) and
python/propwash_gym/src/propwash_gym/protocol.py (MIT packaged module) each carry
their own copy of the v2 wire format — by necessity: the gym is an installable
package and can't import from tools/, and tools/ shouldn't depend on an installed
gym. This check imports BOTH and compares them field-for-field, so a protocol
change that touches only one turns into a red `codec_parity` test instead of a
silent gym breakage.

Pure stdlib, no core spawn. The gym codec is loaded straight from its file so we
don't drag in propwash_gym/__init__ (which imports gymnasium/numpy).
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

sys.path.insert(0, HERE)
import pw_udp  # noqa: E402


def _load_gym_codec():
    path = os.path.join(ROOT, "python", "propwash_gym", "src",
                        "propwash_gym", "protocol.py")
    spec = importlib.util.spec_from_file_location("gym_protocol", path)
    mod = importlib.util.module_from_spec(spec)
    # register before exec so the @dataclass in protocol.py can resolve its own
    # __module__ (dataclasses looks the module up in sys.modules)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    gp = _load_gym_codec()

    # (label, pw_udp value, gym value) — every pair must be identical
    checks = [
        ("MAGIC", pw_udp.MAGIC, gp.MAGIC),
        ("VERSION", pw_udp.VERSION, gp.VERSION),
        ("MAX_CONTACTS", pw_udp.MAX_CONTACTS, gp.MAX_CONTACTS),
        ("HDR.format", pw_udp.HDR.format, gp.HDR.format),
        ("HDR.size", pw_udp.HDR.size, gp.HDR.size),
        ("CMD.format", pw_udp.CMD.format, gp.CMD.format),
        ("SIN.format", pw_udp.SIN.format, gp.SIN.format),
        ("SIN.size", pw_udp.SIN.size, gp.SIN.size),
        ("SOUT.format", pw_udp.SOUT.format, gp.SOUT.format),
        ("SOUT.size", pw_udp.SOUT.size, gp.SOUT.size),
        # packet types
        ("PW_STATE_IN", pw_udp.PW_STATE_IN, gp.PW_STATE_IN),
        ("PW_STATE_OUT", pw_udp.PW_STATE_OUT, gp.PW_STATE_OUT),
        ("PW_OSD", pw_udp.PW_OSD, gp.PW_OSD),
        ("PW_COMMAND", pw_udp.PW_COMMAND, gp.PW_COMMAND),
        # commands
        ("PW_CMD_RESET", pw_udp.PW_CMD_RESET, gp.PW_CMD_RESET),
        ("PW_CMD_REALTIME", pw_udp.PW_CMD_REALTIME, gp.PW_CMD_REALTIME),
        ("PW_CMD_LOCKSTEP", pw_udp.PW_CMD_LOCKSTEP, gp.PW_CMD_LOCKSTEP),
        ("PW_CMD_REPAIR", pw_udp.PW_CMD_REPAIR, gp.PW_CMD_REPAIR),
        # surface enum
        ("SURF_GROUND", pw_udp.SURF_GROUND, gp.PW_SURF_GROUND),
    ]

    bad = [(name, a, b) for name, a, b in checks if a != b]
    for name, a, b in checks:
        flag = "  " if a == b else "!!"
        print(f"{flag} {name:16s} pw_udp={a!r:35s} gym={b!r}")

    if bad:
        print(f"\nCODEC PARITY FAIL — {len(bad)} field(s) drifted:")
        for name, a, b in bad:
            print(f"  {name}: pw_udp={a!r} != gym={b!r}")
        return 1
    print("\nCODEC PARITY PASS — both Python codecs agree on the v2 wire format")
    return 0


if __name__ == "__main__":
    sys.exit(main())
