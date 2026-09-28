# Cockpit zone for the Raspberry Pi (Linux). Build on the Pi: make
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
CFLAGS  += -D_GNU_SOURCE -I../common -Isrc
LDLIBS  += -lm

SRCS := src/main.c src/serial.c src/gpio_tp.c

cockpit: $(SRCS) src/*.h ../common/link_proto.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

test: ../common/test_link_proto.c ../common/link_proto.h
	$(CC) $(CFLAGS) -o test_link_proto ../common/test_link_proto.c && ./test_link_proto

clean:
	rm -f cockpit test_link_proto

.PHONY: test clean
