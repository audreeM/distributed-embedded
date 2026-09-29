#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "serial.h"

int serial_open(const char *dev)
{
	int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		perror(dev);
		return -1;
	}

	struct termios tio;

	if (tcgetattr(fd, &tio) < 0) {
		perror("tcgetattr");
		close(fd);
		return -1;
	}
	/* Raw mode: no line buffering, no echo, and no translating bytes like
	 * 0x0D or 0x11 -- our frames are binary, so every byte must pass through
	 * untouched.
	 */
	cfmakeraw(&tio);
	cfsetispeed(&tio, B115200);
	cfsetospeed(&tio, B115200);
	/* 8N1, no modem control lines, no hardware flow control (only TX, RX
	 * and GND are wired).
	 */
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	/* read() returns right away with whatever bytes are available. */
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
		perror("tcsetattr");
		close(fd);
		return -1;
	}
	tcflush(fd, TCIOFLUSH); /* throw away bytes left over from before we started */
	return fd;
}
