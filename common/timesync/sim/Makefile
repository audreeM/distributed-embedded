# Laptop test of the time sync math. Build: make   Run: ./ts_sim [-v]
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra
CFLAGS  += -I.. -I../../raft

ts_sim: ts_sim.c ../timesync.c ../timesync.h ../../raft/raft_msg.h
	$(CC) $(CFLAGS) -o $@ ts_sim.c ../timesync.c -lm

clean:
	rm -rf ts_sim *.dSYM

.PHONY: clean
