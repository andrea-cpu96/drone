#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_DIR="${PROJECT_ROOT}/flight_controller"

cd "${PYTHON_DIR}"
echo "Starting drone server simulator..."

if command -v python3 >/dev/null 2>&1; then
  exec python3 serial_monitor.py
elif command -v python >/dev/null 2>&1; then
  exec python serial_monitor.py
else
  echo "Python was not found in PATH."
  exit 1
fi
