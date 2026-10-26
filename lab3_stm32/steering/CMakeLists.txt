cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab3_steering)

# N2, the steering zone. Task table (Lab 3 Part 2) -> file:
#   CAN_RX_A/B  canbus.c (driver callback) + dispatch.c (dedup, routing)
#   RELAY       relay.c
#   STEER       steer.c   (servo mapping from lab2_stm32/src/servo.c)
#   BLINK_F     blink.c
#   HB_TX, FORCE_TX  tx_tasks.c
#   HB_MON, TEST_BTN, error state  zone.c
#   watchdog    main.c
target_sources(app PRIVATE
	src/main.c
	src/tp.c
	src/canbus.c
	src/dispatch.c
	src/relay.c
	src/steer.c
	src/blink.c
	src/tx_tasks.c
	src/zone.c
)

target_include_directories(app PRIVATE include ../../common)
