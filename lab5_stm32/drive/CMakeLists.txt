cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab5_drivetrain)

set(NODE_ID 3)
include(${CMAKE_CURRENT_SOURCE_DIR}/../shared/shared.cmake)

target_sources(app PRIVATE
	src/node_drive.c
	src/motor.c
	src/pid.c
)
