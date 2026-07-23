# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_controller)

target_sources(app PRIVATE src/main.c
                           src/log.c
                           src/pid/pid.c
                           src/esc/esc.c
                           src/sensors/sensors.c
                           src/comm/uart.c)

# Real-hardware comm/sensor drivers. Their file-scope DEVICE_DT_GET(DT_ALIAS(...))
# only resolves on the real board overlay, and their call sites in sensors.c are
# all guarded out in simulation mode, so exclude them from the simulation build.
target_sources_ifndef(CONFIG_SIMULATION_MODE app PRIVATE
                      src/sensors/barometer/ms5611.c
                      src/sensors/tof/vl53l1x.c
                      src/sensors/imu/bmi088.c
                      src/comm/i2c.c
                      src/comm/spi.c)

target_include_directories(app PRIVATE src
                                       src/pid
                                       src/esc
                                       src/sensors
                                       src/sensors/barometer
                                       src/sensors/tof
                                       src/sensors/imu
                                       src/comm)
