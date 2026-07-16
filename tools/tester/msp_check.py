#!/usr/bin/env python3
"""MSP identity check against a running propwash-core (TCP 5761 = UART1).

Verifies the in-process firmware really is Betaflight 4.5.2: queries
MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION and MSP_STATUS.

Usage: msp_check.py [host [port]]   (defaults 127.0.0.1 5761)
Exit 0 = firmware identified as BTFL 4.5.x and MSP_STATUS answered.
"""
import socket
import struct
import sys

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_STATUS = 101


def msp_request(sock, cmd):
    payload = b""
    frame = b"$M<" + bytes([len(payload), cmd]) + payload
    crc = 0
    for b in frame[3:]:
        crc ^= b
    sock.sendall(frame + bytes([crc]))

    # read response: $M> len cmd data crc
    def readn(n):
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("closed")
            buf += chunk
        return buf

    hdr = readn(3)
    if hdr != b"$M>":
        raise ValueError(f"bad header {hdr!r}")
    length, rcmd = readn(2)
    data = readn(length)
    readn(1)  # crc
    if rcmd != cmd:
        raise ValueError(f"cmd mismatch {rcmd} != {cmd}")
    return data


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5761

    sock = socket.create_connection((host, port), timeout=5)
    sock.settimeout(5)

    api = msp_request(sock, MSP_API_VERSION)
    variant = msp_request(sock, MSP_FC_VARIANT).decode()
    ver = msp_request(sock, MSP_FC_VERSION)
    status = msp_request(sock, MSP_STATUS)

    fc_version = f"{ver[0]}.{ver[1]}.{ver[2]}"
    cycle_time, i2c_err, sensors, modes, profile = struct.unpack("<HHHIB", status[:11])

    print(f"MSP API      : {api[1]}.{api[2]}")
    print(f"FC variant   : {variant}")
    print(f"FC version   : {fc_version}")
    print(f"cycle time   : {cycle_time} us")
    print(f"sensor mask  : 0x{sensors:04x}")

    ok = variant == "BTFL" and fc_version.startswith("4.5")
    print("MSP CHECK PASS" if ok else "MSP CHECK FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
