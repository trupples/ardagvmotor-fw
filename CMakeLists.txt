cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
zephyr_library_import(maxhal /home/ioan/hal_adi)
project(motor_control)

target_sources(app PRIVATE src/main.c src/tmc9660.c src/nesimtit.c)

