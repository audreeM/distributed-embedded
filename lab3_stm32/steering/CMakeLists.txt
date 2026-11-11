cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab3_steering)

target_sources(app PRIVATE
	src/blink.c
	src/canbus.c
	src/dispatch.c
	src/main.c
	src/relay.c
	src/steer.c
	src/tp.c
	src/tx_tasks.c
	src/zone.c
)

target_include_directories(app PRIVATE include ../../common)
