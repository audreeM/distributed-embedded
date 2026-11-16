# Lab 5 cockpit zone (N1) for the Raspberry Pi. Build on the Pi: make
#   make RAFT_LEADER=0xFF RAFT_SEED=2      LEADER=random with seed 2 (checkoff 8)
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
COMMON  := ../../common
CFLAGS  += -D_GNU_SOURCE -I$(COMMON) -I$(COMMON)/raft -I$(COMMON)/timesync -I.
LDLIBS  += -lm -lpthread

ifdef RAFT_LEADER
CFLAGS  += -DRAFT_DEFAULT_LEADER=$(RAFT_LEADER)
endif
ifdef RAFT_SEED
CFLAGS  += -DRAFT_SEED=$(RAFT_SEED)
endif
ifdef RAFT_LOG_SIZE
CFLAGS  += -DRAFT_LOG_SIZE=$(RAFT_LOG_SIZE)
endif

SRCS := main.c canbus.c gpio_tp.c wheel.c health.c zone.c blinker.c \
        gtime.c edges.c raftnode.c \
        $(COMMON)/raft/raft.c $(COMMON)/raft/raft_sm.c $(COMMON)/raft/raft_client.c \
        $(COMMON)/timesync/timesync.c

cockpit: $(SRCS) *.h $(COMMON)/*.h $(COMMON)/raft/*.h $(COMMON)/timesync/*.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

clean:
	rm -f cockpit

.PHONY: clean
