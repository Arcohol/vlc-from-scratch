cmake_minimum_required(VERSION 3.20.0)

set(BOARD nrf5340dk/nrf5340/cpuapp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oh_my_dmd)

target_sources(app PRIVATE
    src/main.c
    src/dmd_control.c
    src/vlc_tx.c
)

target_include_directories(app PRIVATE src)
