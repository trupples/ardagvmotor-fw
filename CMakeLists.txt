cmake_minimum_required(VERSION 3.20.0)

add_compile_definitions(ESTI_NESIMTIT) # Bodged SPI and CAN :)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

target_sources(app PRIVATE src/main.c src/tmc9660.c src/nesimtit.c)
