cmake_minimum_required(VERSION 3.20.0)

# Bring in OOT board definition
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

zephyr_include_directories(src)

#target_sources(app PRIVATE src/main.c src/tmc9660.c objdict/OD.c)
#zephyr_include_directories(objdict)

target_sources(app PRIVATE src/i2cmain.c)
