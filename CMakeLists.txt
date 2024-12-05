cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

add_compile_definitions(ESTI_NESIMTIT) # Bodged SPI and CAN :)

target_sources(app PRIVATE src/main.c src/tmc9660.c src/nesimtit.c)
