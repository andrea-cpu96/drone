#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${PROJECT_ROOT}/flight_controller"
KERNEL_PATH="${APP_DIR}/build/zephyr/zephyr.elf"
QEMU_BIN="${QEMU_BIN:-${HOME}/zephyr-sdk-1.0.1/hosttools/sysroots/x86_64-pokysdk-linux/usr/bin/qemu-system-arm}"

if [[ ! -f "${KERNEL_PATH}" ]]; then
  echo "Kernel image not found at ${KERNEL_PATH}"
  echo "Run ./scripts/build_qemu.sh first."
  exit 1
fi

if [[ ! -x "${QEMU_BIN}" ]]; then
  echo "QEMU binary not found or not executable: ${QEMU_BIN}"
  exit 1
fi

echo "Launching QEMU simulation..."
exec "${QEMU_BIN}" -cpu cortex-m3 -machine lm3s6965evb -nographic -serial tcp:127.0.0.1:1234 -kernel "${KERNEL_PATH}"
