cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(distributed-embedded)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
    src/main.c      # 3.1 control loop + serial test commands
    src/encoder.c   # 3.1 encoders
    src/motor.c     # 3.1 H-bridge forward drive
    # src/servo.c   # TODO 3.3
    # src/blinker.c # TODO 3.4
    # src/current.c # TODO 3.5
    # src/link.c    # TODO Part 2
)
