cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab3_drivetrain)

target_sources(app PRIVATE
	src/blink.c
	src/canbus.c
	src/dispatch.c
	src/tp.c
	src/zone.c
	src/tx_task.c
	src/pid.c
	src/motor.c
	src/main.c
	src/relay.c
)

target_include_directories(app PRIVATE include ../../common)

