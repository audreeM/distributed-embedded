#!/bin/bash
# Pretend to be N3 (drivetrain) until the real N3 firmware is ready: sends
# HB_N3 (0x102) every 20 ms, state NORMAL, both buses healthy.
#
#   ./sim_n3.sh       on both buses
#   ./sim_n3.sh a     on bus A only (N3 "lost" bus B)
#   ./sim_n3.sh b     on bus B only (N3 "lost" bus A)
#   Ctrl-C            N3 "fails": N2 and the cockpit go to ERROR after ~66 ms
#
# The payload never changes (seq stays 0), so receivers treat the repeats as
# duplicates. That's fine: liveness is recorded before duplicates are dropped.
# Payload: seq 0 | state 1 (NORMAL) | health 0x03 (A and B ok) | 5 zero bytes.
HB=0001030000000000

case "${1:-ab}" in
	a)  BUSES="can0" ;;
	b)  BUSES="can1" ;;
	*)  BUSES="can0 can1" ;;
esac

PIDS=""
for IF in $BUSES; do
	# -i: keep going if this bus stops ACKing (jumper pulled)
	cangen "$IF" -I 102 -L 8 -D "$HB" -g 20 -i &
	PIDS="$PIDS $!"
done
trap 'kill $PIDS 2>/dev/null' EXIT
echo "simulated N3 heartbeat on: $BUSES (Ctrl-C to stop)"
wait
