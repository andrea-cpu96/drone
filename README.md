Virtual PID & Drone Simulation (Zephyr + QEMU + Python)
 
This project implements a real‑time Software‑in‑the‑Loop (SITL) co‑simulation between:

a firmware running on Zephyr RTOS inside QEMU, and

a Python simulation acting as the physical plant (drone dynamics)

The two components communicate through a virtual serial link over TCP.

🚀 Quick Start Guide

To run the simulation, you need two WSL terminals.

1) Environment Setup (in both terminals)

Before running anything, activate the Python virtual environment and load Zephyr’s environment:

# 1. Activate Zephyr's Python virtual environment
source ~/zephyrproject/.venv/bin/activate

# 2. Initialize the Zephyr build environment
source ~/zephyrproject/zephyr/zephyr-env.sh

2) Terminal 1 — Run the Simulation Server (Python)

The Python script acts as a TCP server listening on port 1234.Start it before launching QEMU.

cd ~/embedded_projects/virtual_pid
python serial_monitor.py

The terminal will wait for QEMU to connect.

3) Terminal 2 — Compile and Run the Firmware (Zephyr)

A. Clean Build

Run this from the project root (where CMakeLists.txt is located):

rm -rf build/ && west build -b qemu_cortex_m3

B. Launch QEMU (TCP Client Mode)

To avoid West’s flag parsing issues on WSL, run QEMU manually:

/home/andrea/zephyr-sdk-1.0.1/hosttools/sysroots/x86_64-pokysdk-linux/usr/bin/qemu-system-arm \
  -cpu cortex-m3 \
  -machine lm3s6965evb \
  -nographic \
  -serial tcp:127.0.0.1:1234 \
  -kernel build/zephyr/zephyr.elf

🔍 Expected Output

When QEMU starts, it connects immediately to the Python server.

Terminal 1 (Python)

[+] QEMU connected from: ('127.0.0.1', 33826)
python reply: *** Booting Zephyr OS build v4.4.0 ***
python reply: Hello World! qemu_cortex_m3/ti_lm3s6965

Terminal 2 (QEMU)

You will see Zephyr boot logs and your application output.

🛑 How to Stop the Simulation

Stop Python server: press CTRL + C in Terminal 1

Exit QEMU (nographic mode): press CTRL + A then X in Terminal 2