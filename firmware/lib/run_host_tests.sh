#!/usr/bin/env bash
# Corre las pruebas en host (gcc + ASan/UBSan) de todas las librerías portables del firmware
# y del núcleo de la app tlaloc_llc.
#   bash AMR_LLC/firmware/lib/run_host_tests.sh
# Otro compilador / sin sanitizers (p. ej. zig cc en Windows):
#   CC="zig cc" SANITIZE="" bash AMR_LLC/firmware/lib/run_host_tests.sh
set -euo pipefail
LIB="$(cd "$(dirname "$0")" && pwd)"
APP="$LIB/../tlaloc_llc"
ODOM="${AMR_ODOM_DIR:-$LIB/../../../AMR_localization/amr_odom}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
CFLAGS=(-std=c99 -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES -Wall -Wextra -Werror -Wshadow ${SANITIZE--fsanitize=address,undefined} -g)
fail=0

run() {  # nombre, fuentes...
  local name="$1"; shift
  echo "== $name"
  if ${CC:-gcc} "${CFLAGS[@]}" -I"$LIB/roboclaw" -I"$LIB/amr_drive" -I"$LIB/bno08x" -I"$LIB/gim6010" -I"$ODOM" \
      -I"$LIB/amr_contract" -I"$APP/src" "$@" -lm -o "$OUT/$name" && "$OUT/$name"; then :; else fail=1; fi
}

bash "$LIB/amr_contract/tests/run_host_tests.sh" || fail=1
bash "$ODOM/tests/run_host_tests.sh" || fail=1
run roboclaw "$LIB/roboclaw/roboclaw.c" "$LIB/roboclaw/roboclaw_sim.c" "$LIB/roboclaw/tests/host_test.c"
run amr_drive "$LIB/amr_drive/amr_drive.c" "$LIB/roboclaw/roboclaw.c" "$LIB/roboclaw/roboclaw_sim.c" \
    "$ODOM/omni.c" "$LIB/amr_drive/tests/host_test.c"
run gim6010 "$LIB/gim6010/gim6010.c" "$LIB/gim6010/gim6010_sim.c" "$LIB/gim6010/gim_arm.c" \
    "$LIB/gim6010/tests/host_test.c"
run bno08x "$LIB/bno08x/bno08x.c" "$LIB/bno08x/bno08x_sim.c" "$LIB/bno08x/tests/host_test.c"
# Núcleo del firmware de Tlaloc en lazo cerrado con RoboClaw y BNO080 simuladas.
run tlaloc_llc "$APP/src/llc.c" "$APP/src/gripper.c" "$LIB/amr_contract/amr_contract.c" \
    "$LIB/gim6010/gim6010.c" "$LIB/gim6010/gim6010_sim.c" "$LIB/gim6010/gim_arm.c" \
    "$LIB/amr_drive/amr_drive.c" "$LIB/roboclaw/roboclaw.c" "$LIB/roboclaw/roboclaw_sim.c" \
    "$LIB/bno08x/bno08x.c" "$LIB/bno08x/bno08x_sim.c" "$ODOM/omni.c" "$ODOM/ekf.c" \
    "$APP/tests/host_test.c"

if [ "$fail" -eq 0 ]; then echo "TODAS LAS PRUEBAS DE FIRMWARE: PASSED"; else echo "HAY FALLAS"; exit 1; fi
