# Laptop simulation of the Raft core. Build: make   Run: ./raft_sim [scenario] [-v]
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra
CFLAGS  += -I.. -I../..

SRCS := raft_sim.c ../raft.c ../raft_sm.c
HDRS := ../raft.h ../raft_msg.h ../raft_sm.h ../raft_config.h ../../can_catalog.h

raft_sim: $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

# Same sources with sanitizers, for catching memory bugs before the STM32 does.
asan: $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -fsanitize=address,undefined -o raft_sim_asan $(SRCS)

clean:
	rm -rf raft_sim raft_sim_asan *.dSYM

.PHONY: clean asan
