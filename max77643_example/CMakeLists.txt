# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world_1)

target_sources(app PRIVATE 
    src/max77643.c
    src/main.c
)




target_include_directories(app PRIVATE
    src
    ${ZEPHYR_BASE}/include
    ${ZEPHYR_BASE}/drivers
)

zephyr_library_include_directories(${ZEPHYR_BASE}/include)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
