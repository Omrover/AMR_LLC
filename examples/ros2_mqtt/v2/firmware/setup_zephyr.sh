#!/usr/bin/env bash
# One-time NUC setup for the class 10 firmware (no sudo, ~4.5 GB, ~10 min).
# Installs Zephyr v4.5.0 with only the NXP HAL + the Zephyr SDK ARM toolchain.
#
#   ./setup_zephyr.sh                 # installs into ~/zephyr-class10
#   ./setup_zephyr.sh /opt/zc10       # or any directory you own
# Afterwards, in every firmware terminal:
#   source ~/zephyr-class10/env.sh
#
# Host requirements (Ubuntu 24.04): git curl xz-utils cmake (>= 3.20) python3-venv
set -euo pipefail

DIR=$(realpath -m "${1:-$HOME/zephyr-class10}")
ZEPHYR_VERSION=v4.5.0
SDK_VERSION=1.0.1
SDK_URL=https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}
SDK_FILE=zephyr-sdk-${SDK_VERSION}_linux-x86_64_minimal.tar.xz

for tool in git curl xz cmake python3; do
  command -v "$tool" >/dev/null || { echo "Missing host tool: $tool" >&2; exit 1; }
done
mkdir -p "$DIR"
cd "$DIR"

echo "== Python venv with west"
[ -d .venv ] || python3 -m venv .venv
# shellcheck disable=SC1091
. .venv/bin/activate
pip install -q --upgrade pip west ninja

echo "== Zephyr ${ZEPHYR_VERSION} (NXP HAL + CMSIS only)"
[ -d ws/.west ] || west init -m https://github.com/zephyrproject-rtos/zephyr \
  --mr "$ZEPHYR_VERSION" ws
cd ws
west config manifest.project-filter -- '-.*,+cmsis_6,+cmsis,+hal_nxp'
west update --narrow -o=--depth=1
pip install -q -r zephyr/scripts/requirements-base.txt
cd "$DIR"

echo "== Zephyr SDK ${SDK_VERSION}, arm-zephyr-eabi toolchain"
if [ ! -d "zephyr-sdk-${SDK_VERSION}/gnu/arm-zephyr-eabi" ]; then
  curl -fsSL "$SDK_URL/sha256.sum" -o sdk.sha256
  curl -fSL "$SDK_URL/$SDK_FILE" -o "$SDK_FILE"
  grep " ${SDK_FILE}\$" sdk.sha256 | sha256sum -c -
  tar xf "$SDK_FILE"
  rm -f "$SDK_FILE" sdk.sha256
  (cd "zephyr-sdk-${SDK_VERSION}" && ./setup.sh -h -t arm-zephyr-eabi)
fi

cat > env.sh <<EOF
# source this file before using west for the class 10 firmware
. "$DIR/.venv/bin/activate"
export ZEPHYR_BASE="$DIR/ws/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$DIR/zephyr-sdk-${SDK_VERSION}"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
EOF
echo "Done. Run: source $DIR/env.sh"
