/*
 * Lab 3 SPI wiring probe for the two XIAO MCP2515 modules on the handout pins.
 *
 * Bit-bangs the bus as plain GPIO (no SPI peripheral, no CAN driver) and, for
 * each module, reports:
 *   - whether its INT pin is driven (module powered, INT wire connected)
 *   - whether anything drives D11/D12 while no module is selected
 *   - which of D11/D12 the module's SO (data out) drives once CS goes low
 *   - CANSTAT/CANCTRL after a RESET command (0x80 / 0x87 on a healthy MCP2515)
 *
 * The SO line is identified with pull-up/pull-down reads before anything is
 * driven, so the STM never drives a line the module is also driving.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

struct pin {
	const struct device *port;
	gpio_pin_t num;
	const char *name;
};

#define PIN(node, n, label) { .port = DEVICE_DT_GET(DT_NODELABEL(node)), .num = n, .name = label }

struct module {
	const char *name;
	struct pin cs;
	struct pin irq;
};

static const struct pin sck = PIN(gpioa, 5, "D13/PA5");
static const struct pin d11 = PIN(gpioa, 7, "D11/PA7");
static const struct pin d12 = PIN(gpioa, 6, "D12/PA6");

static const struct module modules[] = {
	{ "CAN A", PIN(gpiob, 6, "D10/PB6"), PIN(gpioa, 10, "D2/PA10") },
	{ "CAN B", PIN(gpioc, 7, "D9/PC7"), PIN(gpiob, 5, "D4/PB5") },
};

#define MCP_RESET   0xC0
#define MCP_READ    0x03
#define MCP_CANSTAT 0x0E

enum level { FLOATING = -1, LOW = 0, HIGH = 1 };

static void drive(const struct pin *p, int level)
{
	gpio_pin_configure(p->port, p->num, level ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW);
}

static void release(const struct pin *p)
{
	gpio_pin_configure(p->port, p->num, GPIO_INPUT);
}

static void set(const struct pin *p, int level)
{
	gpio_pin_set_raw(p->port, p->num, level);
}

/* Read with the internal pull-up, then pull-down: same answer both times = something drives it. */
static enum level sense(const struct pin *p)
{
	int up;
	int down;

	gpio_pin_configure(p->port, p->num, GPIO_INPUT | GPIO_PULL_UP);
	k_busy_wait(100);
	up = gpio_pin_get_raw(p->port, p->num);
	gpio_pin_configure(p->port, p->num, GPIO_INPUT | GPIO_PULL_DOWN);
	k_busy_wait(100);
	down = gpio_pin_get_raw(p->port, p->num);
	release(p);

	return (up == down) ? (enum level)up : FLOATING;
}

static const char *level_str(enum level l)
{
	switch (l) {
	case HIGH: return "driven HIGH";
	case LOW:  return "driven LOW";
	default:   return "floating";
	}
}

/* SPI mode 0, MSB first, ~50 kHz. MCP2515 samples SI on SCK rising, shifts SO on falling. */
static uint8_t xfer(const struct pin *mosi, const struct pin *miso, uint8_t out)
{
	uint8_t in = 0;

	for (int bit = 7; bit >= 0; bit--) {
		set(mosi, (out >> bit) & 1);
		k_busy_wait(10);
		set(&sck, 1);
		k_busy_wait(5);
		in = (uint8_t)((in << 1) | (gpio_pin_get_raw(miso->port, miso->num) & 1));
		k_busy_wait(5);
		set(&sck, 0);
	}
	return in;
}

static void read_after_reset(const struct module *m, const struct pin *mosi,
			     const struct pin *miso, gpio_flags_t pull, uint8_t *canstat,
			     uint8_t *canctrl)
{
	drive(mosi, 0);
	/* with a pull-up a line nobody drives reads 0xFF, with a pull-down 0x00 */
	gpio_pin_configure(miso->port, miso->num, GPIO_INPUT | pull);

	set(&m->cs, 0);
	xfer(mosi, miso, MCP_RESET);
	set(&m->cs, 1);
	k_msleep(5);

	set(&m->cs, 0);
	xfer(mosi, miso, MCP_READ);
	xfer(mosi, miso, MCP_CANSTAT);
	*canstat = xfer(mosi, miso, 0x00);
	*canctrl = xfer(mosi, miso, 0x00); /* READ auto-increments to CANCTRL (0x0F) */
	set(&m->cs, 1);

	release(mosi);
	release(miso);
}

static void probe(const struct module *m)
{
	enum level irq, idle11, idle12, sel11, sel12;
	const struct pin *miso = NULL;
	const struct pin *mosi = NULL;
	uint8_t canstat;
	uint8_t canctrl;

	printk("\n== %s  (CS %s, INT %s)\n", m->name, m->cs.name, m->irq.name);

	irq = sense(&m->irq);
	printk("  INT pin            : %-12s %s\n", level_str(irq),
	       irq == HIGH ? "ok - module powered, INT wire connected"
			   : "PROBLEM - expected driven HIGH: check module 3V3/GND and INT (D6) wire");

	idle11 = sense(&d11);
	idle12 = sense(&d12);
	printk("  no module selected : D11 %-12s D12 %-12s %s\n", level_str(idle11),
	       level_str(idle12),
	       (idle11 == FLOATING && idle12 == FLOATING)
		       ? "ok"
		       : "PROBLEM - something drives a data line while every CS is high "
			 "(a module whose CS wire is loose thinks it is selected)");

	set(&m->cs, 0);
	k_busy_wait(50);
	sel11 = sense(&d11);
	sel12 = sense(&d12);
	set(&m->cs, 1);
	printk("  this module CS low : D11 %-12s D12 %-12s (its SO should now be driven)\n",
	       level_str(sel11), level_str(sel12));

	if (sel12 != FLOATING && sel11 == FLOATING) {
		miso = &d12;
		mosi = &d11;
	} else if (sel11 != FLOATING && sel12 == FLOATING) {
		miso = &d11;
		mosi = &d12;
	}

	if (sel11 != FLOATING && sel12 != FLOATING) {
		printk("  -> PROBLEM: both D11 and D12 are driven. They are shorted together or "
		       "tied to another output\n");
		return;
	}

	if (miso == NULL) {
		/*
		 * SO may stay high-impedance until the chip actually shifts data out.
		 * Run RESET + READ in both orientations, reading the return line once
		 * with a pull-up and once with a pull-down: identical bytes = driven.
		 */
		static const struct {
			const struct pin *mosi;
			const struct pin *miso;
		} tries[] = { { &d11, &d12 }, { &d12, &d11 } };

		printk("  SO not driven at CS low alone; trying a real RESET + READ both ways\n");
		ARRAY_FOR_EACH(tries, i) {
			uint8_t st_up, ct_up, st_dn, ct_dn;

			read_after_reset(m, tries[i].mosi, tries[i].miso, GPIO_PULL_UP, &st_up, &ct_up);
			read_after_reset(m, tries[i].mosi, tries[i].miso, GPIO_PULL_DOWN, &st_dn,
					 &ct_dn);
			printk("    SI=%s SO=%s: pull-up %02x %02x, pull-down %02x %02x -> %s\n",
			       tries[i].mosi->name, tries[i].miso->name, st_up, ct_up, st_dn, ct_dn,
			       (st_up == 0x80 && ct_up == 0x87 && st_dn == 0x80 && ct_dn == 0x87)
				       ? "MCP2515 OK in this orientation"
				       : (st_up == st_dn && ct_up == ct_dn) ? "driven, but wrong values"
									    : "not driven (floating)");
		}
		printk("  -> If both are 'not driven': the module never answers. Check its CS (D7), "
		       "SCK (D8) and that SO really goes to D11/D12.\n");
		return;
	}

	read_after_reset(m, mosi, miso, GPIO_PULL_UP, &canstat, &canctrl);
	printk("  SO lands on %s, SI on %s. After RESET: CANSTAT %02x CANCTRL %02x "
	       "(healthy: 80 87)\n", miso->name, mosi->name, canstat, canctrl);

	if (canstat == 0x80 && canctrl == 0x87) {
		printk("  -> MCP2515 OK. %s\n",
		       miso == &d12 ? "Wiring is correct for the SPI peripheral."
				    : "Data lines are SWAPPED: move the module SO wire to D12 and "
				      "SI to D11.");
	} else if (canstat == 0xFF && canctrl == 0xFF) {
		printk("  -> PROBLEM: SO went quiet during the read. Check SCK (Nucleo D13 -> module D8).\n");
	} else {
		printk("  -> PROBLEM: module answers but with wrong values. Check SCK (D13 -> D8) "
		       "and the SI wire (%s -> module).\n", mosi->name);
	}
}

int main(void)
{
	printk("\nLab 3 SPI wiring probe (bit-banged, no SPI peripheral)\n");

	drive(&sck, 0);
	ARRAY_FOR_EACH_PTR(modules, m) {
		drive(&m->cs, 1);
	}
	release(&d11);
	release(&d12);

	while (1) {
		printk("\n================================================================\n");
		ARRAY_FOR_EACH_PTR(modules, m) {
			probe(m);
		}
		k_msleep(3000);
	}

	return 0;
}
