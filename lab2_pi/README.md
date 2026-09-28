# Cockpit zone (Raspberry Pi)

A plain Linux program, not Zephyr. It **replaces** `proxy_receiver`: only one process can bind UDP :8000, so stop `proxy_receiver`/`wheel_monitor` before you run it. Protocol details: [../LINK_PROTOCOL.md](../LINK_PROTOCOL.md).

## One-time Pi setup

1. `sudo raspi-config` → Interface Options → Serial Port → login shell over serial: **No**, serial hardware: **Yes**.
2. Add these lines to `/boot/firmware/config.txt`:
   ```
   enable_uart=1
   dtoverlay=disable-bt
   ```
   `disable-bt` moves the full PL011 UART onto GPIO14/15, so the baud rate no longer changes with the core clock.
3. `sudo systemctl disable hciuart`, then `sudo usermod -aG dialout,gpio $USER`, then reboot.
4. Check that `ls -l /dev/serial0` points at `ttyAMA0`.

## Build and run

Copy the whole `distributed-embedded` folder to the Pi; `lab2_pi` needs `../common`. Then:
```
cd distributed-embedded/lab2_pi
make test        # parser unit tests
make
./cockpit        # or: sudo ./cockpit -r   (SCHED_FIFO, for timing captures)
```
Start the Mac proxy as in Appendix A. `--interval-ms 20` is recommended: a quick double press of the self-test button can fall between 50 ms samples.

Fill in `src/config.h` first: the `BTN_IDX_*` values from Part 1, and the axis calibration if yours differs from the Mac defaults.

Output, 5 lines/s:
```
UDP   1234 (lost 0) | CMD   1300 thr= 42 brk=  0 str= -17 btn=L-- | STM NORMAL f=0x00 ack=  3 crcErr=0 rngErr=0 I=0/0/0 mA hb=19.9/20.0/20.1 ms (n=49)
```
`hb` is the min/avg/max STATUS-frame interval over the last second, which is the 20 ms ± 10 % heartbeat check. The `I=` currents come from the CURRENTS frame sent right after each STATUS.

Keys (type, then press Enter): `b` sends out-of-range throttle for 1 s (checkoff step 8), `c` sends one corrupt-CRC frame (should be dropped, crcErr goes up), `q` quits.

## Pins

| Pi header | Use |
|---|---|
| 8 (GPIO14 TXD) | → STM32 PB7 |
| 10 (GPIO15 RXD) | ← STM32 PB6 |
| 6 (GND) | STM32 GND |
| 16 (GPIO23) | UDP_RX test point |
| 18 (GPIO24) | CMD_TX test point |
