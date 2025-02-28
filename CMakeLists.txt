cmake_minimum_required(VERSION 3.20.0)

# Get git revision or tag
execute_process(
	COMMAND git describe --tags --long --always --dirty
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_DESCRIBE
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_compile_definitions(GIT_REVISION_STR="${GIT_DESCRIBE}")


# Bring in OOT board definition
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_control)

zephyr_include_directories(src)

target_sources(app PRIVATE src/main.c src/tmc9660.c)

