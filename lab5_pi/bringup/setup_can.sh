#!/bin/bash
# One-time Pi setup for Lab 3: enable SPI0 and the two MCP2515 overlays, install can-utils.
#   CAN A module: SPI0 CE0 (pin 24), INT GPIO25 (pin 22)  -> can0
#   CAN B module: SPI0 CE1 (pin 26), INT GPIO24 (pin 18)  -> can1
# Usage: sudo ./setup_can.sh [oscillator_hz]   (default 16000000 - read the module's crystal)
set -e

OSC=${1:-16000000}
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt   # older Raspberry Pi OS

if [ "$(id -u)" -ne 0 ]; then
	echo "run with sudo"; exit 1
fi

apt-get install -y can-utils

if grep -q "^dtoverlay=mcp2515-can0" "$CONFIG"; then
	echo "$CONFIG already has the mcp2515 overlays:"
	grep -n "spi=on\|mcp2515" "$CONFIG"
	echo "edit the oscillator= values by hand if they are wrong"
else
	cp "$CONFIG" "$CONFIG.bak-lab3"
	# [all] so the lines are not swallowed by an earlier [pi4]/[cm4] conditional section
	cat >> "$CONFIG" <<CFG

[all]
# Lab 3 dual CAN (XIAO MCP2515 modules on SPI0)
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=$OSC,interrupt=25
dtoverlay=mcp2515-can1,oscillator=$OSC,interrupt=24
CFG
	echo "added overlays to $CONFIG (backup at $CONFIG.bak-lab3), oscillator=$OSC"
fi

echo "now: sudo reboot, then ./can_up.sh"
