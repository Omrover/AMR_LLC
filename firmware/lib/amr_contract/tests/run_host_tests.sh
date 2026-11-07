#!/usr/bin/env bash
# Pruebas en host de amr_contract: C (gcc) + validación cruzada con el decodificador Python.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
LIB="$(dirname "$HERE")"
FIXTURES="$LIB/../../../protocol/fixtures/commands.txt"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

${CC:-gcc} -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
  -fsanitize=address,undefined -g -I"$LIB" "$LIB/amr_contract.c" "$HERE/host_test.c" -lm -o "$OUT/host_test"
"$OUT/host_test" "$FIXTURES" "$OUT/telemetry.txt"
python3 "$HERE/check_telemetry.py" "$OUT/telemetry.txt"
