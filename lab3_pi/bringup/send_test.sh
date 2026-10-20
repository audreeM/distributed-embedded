#!/bin/bash
# Pi -> STM direction: send ID 0x7F1 every 100 ms on both buses (data byte 0 counts up).
# The STM bring-up firmware prints these as "RX A id 0x7f1" / "RX B id 0x7f1". Ctrl-C to stop.
cangen can0 -I 7F1 -L 3 -D i -g 100 &
A=$!
cangen can1 -I 7F1 -L 3 -D i -g 100 &
B=$!
trap 'kill $A $B 2>/dev/null' EXIT
echo "sending 0x7F1 on can0 and can1 every 100 ms, Ctrl-C to stop"
wait
