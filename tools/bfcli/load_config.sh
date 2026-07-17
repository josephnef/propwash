#!/usr/bin/env bash
# Bake the CineLog35 tune into a propwash eeprom, once. After this, every
# launch of propwash-core with the same --eeprom flies the pilot's config
# (exactly like a real FC: the eeprom persists on disk).
#
#   tools/bfcli/load_config.sh [eeprom-path]
#
# Default eeprom: ./eeprom.bin (what propwash-core uses by default).
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CORE="${PROPWASH_CORE:-$ROOT/build/propwash-core}"
EEPROM="${1:-$ROOT/eeprom.bin}"
DIFF="$ROOT/config/cinelog35v3.diff"
OVERRIDES="$ROOT/config/sitl-overrides.txt"

[ -x "$CORE" ] || { echo "core not built: $CORE" >&2; exit 1; }

echo "loading $DIFF (+ overrides) into $EEPROM ..."
"$CORE" --realtime --no-js --eeprom "$EEPROM" >/dev/null 2>&1 &
CPID=$!
trap 'kill $CPID 2>/dev/null' EXIT
sleep 2

python3 "$HERE/pw_cli.py" apply "$DIFF" --save
python3 "$HERE/pw_cli.py" apply "$OVERRIDES" --save

echo "done. run:  $CORE --eeprom $EEPROM"
