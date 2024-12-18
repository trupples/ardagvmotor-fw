cmake_minimum_required(VERSION 3.20.0)

# Bring in OOT board definition
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

target_sources(app PRIVATE src/main.c src/tmc9660.c objdict/CO_OD.c)
zephyr_include_directories(objdict)
