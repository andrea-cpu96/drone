# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_controller)

target_sources(app PRIVATE src/main.c
                           src/pid/pid.c
                           src/esc/esc.c)

target_include_directories(app PRIVATE src
                                       src/pid
                                       src/esc)