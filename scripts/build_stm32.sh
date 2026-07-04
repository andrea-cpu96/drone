#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${PROJECT_ROOT}/virtual_pid/virtual_pid"

if [[ ! -f "${HOME}/zephyrproject/.venv/bin/activate" ]]; then
  echo "Missing virtual environment at ${HOME}/zephyrproject/.venv/bin/activate"
  exit 1
fi

if [[ ! -f "${HOME}/zephyrproject/zephyr/zephyr-env.sh" ]]; then
  echo "Missing Zephyr environment script at ${HOME}/zephyrproject/zephyr/zephyr-env.sh"
  exit 1
fi

source "${HOME}/zephyrproject/.venv/bin/activate"
source "${HOME}/zephyrproject/zephyr/zephyr-env.sh"

cd "${APP_DIR}"
echo "Building firmware for STM32 target: stm32f411e_disco"
rm -rf build
west build -b stm32f411e_disco
