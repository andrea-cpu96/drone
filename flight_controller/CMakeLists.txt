# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_controller)

target_sources(app PRIVATE src/main.c
                           src/pid/pid.c
                           src/esc/esc.c
                           src/sensors/sensors.c
                           # src/sensors/barometer/ms5611.c
                           src/comm/i2c.c
                           src/comm/uart.c)

target_include_directories(app PRIVATE src
                                       src/pid
                                       src/esc
                                       src/sensors
                                       src/sensors/barometer
                                       src/comm)
