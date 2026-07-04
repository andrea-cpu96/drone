# Virtual PID & Drone Simulation (Zephyr + QEMU + Python)
 
This project implements a real‑time Software‑in‑the‑Loop (SITL) co‑simulation between:

a firmware running on Zephyr RTOS inside QEMU, and

a Python simulation acting as the physical plant (drone dynamics)

The two components communicate through a virtual serial link over TCP.

🚀 Quick Start Guide

Automation helpers are available in the scripts folder. From the project root, you can use:

- `./scripts/build_stm32.sh` to build the firmware for the STM32 target
- `./scripts/build_qemu.sh` to build the firmware for the QEMU target
- `./scripts/start_drone_server_simulator.sh` to start the Python serial monitor server
- `./scripts/run_qemu_simulation.sh` to launch QEMU with the built firmware

To run the simulation, you need two shells.

1) Environment Setup (in both shells)

Before running anything, activate the Python virtual environment and load Zephyr’s environment:

# 1. Activate Zephyr's Python virtual environment
`source ~/zephyrproject/.venv/bin/activate`

# 2. Initialize the Zephyr build environment
`source ~/zephyrproject/zephyr/zephyr-env.sh`

2) Shell 1 — Run the Simulation Server (Python)

The Python script acts as a TCP server listening on port 1234. Start it before launching QEMU.

```bash
cd ~/embedded_projects/drone
./scripts/start_drone_server_simulator.sh
```

The shell will wait for QEMU to connect.

3) Shell 2 — Compile and Run the Firmware (Zephyr)

A. Build for QEMU

```bash
cd ~/embedded_projects/drone
./scripts/build_qemu.sh
```

B. Launch QEMU (TCP Client Mode)

```bash
cd ~/embedded_projects/drone
./scripts/run_qemu_simulation.sh
```

C. Build for STM32 hardware

```bash
cd ~/embedded_projects/drone
./scripts/build_stm32.sh
```

🔍 Expected Output

When QEMU starts, it connects immediately to the Python server.

Shell 1 (Python)

[+] QEMU connected from: ('127.0.0.1', 33826)  
python reply: *** Booting Zephyr OS build v4.4.0 ***  
python reply: Hello World! qemu_cortex_m3/ti_lm3s6965

Shell 2 (QEMU)

You will see Zephyr boot logs and your application output.