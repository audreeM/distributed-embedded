/*
 * Direct register access through /dev/gpiomem: a toggle is one store, so the
 * test point edge lands within a microsecond of the event it marks. (libgpiod
 * would work too, but its API differs between Pi OS releases.)
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include "gpio_tp.h"

/* Word (32-bit) offsets into the BCM2711 GPIO register block:
 *   GPFSELn - 3 bits per pin select its function (001 = output)
 *   GPSET0  - writing 1 to bit N drives pin N high (other pins untouched)
 *   GPCLR0  - writing 1 to bit N drives pin N low
 */
#define GPFSEL0 0
#define GPSET0  7
#define GPCLR0  10

static volatile uint32_t *gpio;
static uint32_t level; /* last value we drove, one bit per pin */

int tp_init(const int *pins, int npins)
{
	/* /dev/gpiomem exposes only the GPIO registers, so it works without
	 * root for users in the "gpio" group (unlike /dev/mem).
	 */
	int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

	if (fd < 0) {
		perror("open /dev/gpiomem (test points disabled)");
		return -1;
	}
	void *map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

	close(fd);
	if (map == MAP_FAILED) {
		perror("mmap /dev/gpiomem (test points disabled)");
		return -1;
	}
	gpio = map;

	for (int i = 0; i < npins; i++) {
		/* 10 pins per GPFSEL register, 3 bits each. */
		int reg = GPFSEL0 + pins[i] / 10;
		int shift = (pins[i] % 10) * 3;

		gpio[reg] = (gpio[reg] & ~(7u << shift)) | (1u << shift); /* output */
		gpio[GPCLR0] = 1u << pins[i];
	}
	level = 0;
	return 0;
}

void tp_toggle(int pin)
{
	if (!gpio) {
		return;
	}
	/* We remember the level ourselves instead of reading it back, which
	 * saves a register read on the timing path.
	 */
	level ^= 1u << pin;
	gpio[(level & (1u << pin)) ? GPSET0 : GPCLR0] = 1u << pin;
}

void tp_release(void)
{
	if (gpio) {
		gpio[GPCLR0] = level;
		level = 0;
	}
}
