# Lab 3 cockpit zone (N1) for the Raspberry Pi. Build on the Pi: make
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
CFLAGS  += -D_GNU_SOURCE -I../../common -I.
LDLIBS  += -lm

SRCS := main.c canbus.c gpio_tp.c wheel.c health.c zone.c blinker.c

cockpit: $(SRCS) *.h ../../common/can_catalog.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

clean:
	rm -f cockpit

.PHONY: clean
