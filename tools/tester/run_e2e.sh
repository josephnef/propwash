#!/usr/bin/env bash
# Launches propwash-core in server mode and drives it with the Python UDP
# client (the same loop the Godot client runs). Used by ctest.
set -u
CORE="${1:?usage: run_e2e.sh <path-to-propwash-core>}"
DIR="$(cd "$(dirname "$0")" && pwd)"
EEPROM="$(mktemp /tmp/pw-e2e-XXXX.bin)"
rm -f "$EEPROM"

"$CORE" --no-js --eeprom "$EEPROM" &
CPID=$!
trap 'kill $CPID 2>/dev/null; rm -f "$EEPROM"' EXIT
sleep 2

python3 "$DIR/udp_client_check.py"
