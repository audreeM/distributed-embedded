#!/bin/bash
# Bring can0 (bus A) and can1 (bus B) up at 500 kbit/s. Run after every reboot.
# restart-ms 100: automatic bus-off recovery (R4.8).
#
# Linux names the two MCP2515s in probe order, so can0 is not always the CE0
# module. This script renames them so can0 = SPI0 CE0 = bus A and
# can1 = SPI0 CE1 = bus B, every time.

echo "--- driver probe (want 'successfully initialized' for spi0.0 and spi0.1)"
dmesg | grep -i mcp251x

spidev_of() {
	basename "$(readlink -f /sys/class/net/$1/device)"
}

if [ "$(spidev_of can0)" = "spi0.1" ] && [ "$(spidev_of can1)" = "spi0.0" ]; then
	echo "--- can0/can1 came up swapped, renaming so can0 = CE0 (bus A)"
	sudo ip link set can0 down
	sudo ip link set can1 down
	sudo ip link set can0 name cantmp
	sudo ip link set can1 name can0
	sudo ip link set cantmp name can1
fi

for IF in can0 can1; do
	if ! ip link show "$IF" > /dev/null 2>&1; then
		echo "$IF does not exist - check dmesg above, SPI/INT wiring, config.txt"
		continue
	fi
	sudo ip link set "$IF" down
	# Short TX queue: while a jumper is pulled, frames pile up here and are
	# flushed late when it goes back in. 4 keeps that within what the
	# receivers' duplicate filter rejects (up to 7 old frames, can_catalog.h).
	sudo ip link set "$IF" txqueuelen 4
	sudo ip link set "$IF" up type can bitrate 500000 restart-ms 100
	echo "--- $IF ($(spidev_of $IF))"
	ip -details link show "$IF" | grep -E "state|bitrate|clock"
done
