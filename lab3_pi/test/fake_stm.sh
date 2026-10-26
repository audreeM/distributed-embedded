#!/bin/bash
# Fake the STM32 heartbeats so the cockpit can be tested without the boards.
#
#   ./fake_stm.sh                        both zones on both buses (normal)
#   ./fake_stm.sh "101" "can0 can1"      steering only -> drivetrain fails
#   ./fake_stm.sh "101 102" "can0"       bus A only    -> bus B degraded
#
# Ctrl-C to stop (-> both zones fail, then the cockpit is isolated).
IDS=${1:-"101 102"}
BUSES=${2:-"can0 can1"}

s=0
while true; do
	d=0$(printf %X $s)01000000000000
	for b in $BUSES; do
		for id in $IDS; do
			cansend "$b" "$id#$d"
		done
	done
	s=$(((s + 1) % 16))
	sleep 0.02
done
