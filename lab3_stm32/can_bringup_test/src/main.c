/*
 * Lab 3 Part 1 checkpoint: send one frame every 100 ms on both CAN modules,
 * print every frame received on either one, and print each controller's state
 * once a second. This is only a wiring / bit-timing check, not the Lab 3 app.
 *
 * Frame sent: ID 0x7F0 + node, data = { counter, 0x0A or 0x0B (which module), node }
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define NODE_ID        CONFIG_BRINGUP_NODE_ID
#define TX_ID          (0x7F0 + NODE_ID)
#define TX_PERIOD_MS   100
#define STATUS_MS      1000

struct bus {
	const char *name;
	uint8_t tag;              /* data[1] of our frame, so candump shows which module sent it */
	const struct device *dev;
	struct gpio_dt_spec int_pin;  /* MCP2515 INT, read back only to show whether it is asserted */
	struct spi_dt_spec spi;       /* same SPI device the driver uses, for the register dump */
	struct k_msgq *rxq;
	bool up;
	atomic_t tx_ok;           /* frame made it onto the wire and was ACKed */
	atomic_t tx_err;          /* driver reported a TX error */
	uint32_t tx_full;         /* can_send() found all 3 MCP2515 TX buffers busy */
	uint32_t tx_fail;         /* can_send() refused for another reason (e.g. bus-off) */
	uint32_t rx;
};

CAN_MSGQ_DEFINE(rxq_a, 16);
CAN_MSGQ_DEFINE(rxq_b, 16);

static struct bus buses[] = {
	{ .name = "A", .tag = 0x0A, .dev = DEVICE_DT_GET(DT_NODELABEL(can_a)),
	  .int_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(can_a), int_gpios),
	  .spi = SPI_DT_SPEC_GET(DT_NODELABEL(can_a), SPI_WORD_SET(8)), .rxq = &rxq_a },
	{ .name = "B", .tag = 0x0B, .dev = DEVICE_DT_GET(DT_NODELABEL(can_b)),
	  .int_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(can_b), int_gpios),
	  .spi = SPI_DT_SPEC_GET(DT_NODELABEL(can_b), SPI_WORD_SET(8)), .rxq = &rxq_b },
};

/*
 * Bring-up diagnostic only: read MCP2515 registers behind the driver's back.
 * Safe here because the driver only touches SPI from its INT thread, and INT
 * is what we are debugging. Do not copy this into the real app.
 */
static uint8_t mcp_read(const struct spi_dt_spec *spi, uint8_t addr)
{
	uint8_t tx[3] = { 0x03 /* READ */, addr, 0 };
	uint8_t rx[3] = { 0 };
	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rx_buf, .count = 1 };

	if (spi_transceive_dt(spi, &txs, &rxs) != 0) {
		return 0xEE;
	}
	return rx[2];
}

static void mcp_dump(struct bus *b)
{
	static const char *const modes[] = { "normal", "sleep", "loopback", "listen-only",
					     "config", "?", "?", "?" };
	uint8_t canstat = mcp_read(&b->spi, 0x0E);
	uint8_t txb[3] = { mcp_read(&b->spi, 0x30), mcp_read(&b->spi, 0x40),
			   mcp_read(&b->spi, 0x50) };

	/* TXBnCTRL bits: 3 TXREQ (waiting to send), 4 TXERR, 5 MLOA (lost arbitration), 6 ABTF */
	printk("  MCP %s: mode %s | TXB0 %02x TXB1 %02x TXB2 %02x | CANINTE %02x CANINTF %02x "
	       "EFLG %02x\n", b->name, modes[canstat >> 5], txb[0], txb[1], txb[2],
	       mcp_read(&b->spi, 0x2B), mcp_read(&b->spi, 0x2C), mcp_read(&b->spi, 0x2D));
}

static const char *state_str(enum can_state s)
{
	switch (s) {
	case CAN_STATE_ERROR_ACTIVE:  return "ERROR-ACTIVE";
	case CAN_STATE_ERROR_WARNING: return "ERROR-WARNING";
	case CAN_STATE_ERROR_PASSIVE: return "ERROR-PASSIVE";
	case CAN_STATE_BUS_OFF:       return "BUS-OFF";
	case CAN_STATE_STOPPED:       return "STOPPED";
	default:                      return "?";
	}
}

/* Called from the driver (interrupt context) when a queued frame finishes. */
static void tx_done(const struct device *dev, int error, void *user_data)
{
	struct bus *b = user_data;

	ARG_UNUSED(dev);
	atomic_inc(error == 0 ? &b->tx_ok : &b->tx_err);
}

static void bus_init(struct bus *b)
{
	/* mask 0 = accept every standard (11-bit) ID */
	const struct can_filter all_std = { .id = 0, .mask = 0, .flags = 0 };
	struct can_timing t;
	uint32_t core_clock;
	int err;

	if (!device_is_ready(b->dev)) {
		printk("CAN %s: NOT READY - MCP2515 did not answer over SPI "
		       "(check SCK/MOSI/MISO/CS, 3V3, GND)\n", b->name);
		return;
	}

	if (can_get_core_clock(b->dev, &core_clock) == 0 &&
	    can_calc_timing(b->dev, &t, 500000, 875) >= 0) {
		printk("CAN %s: core clock %u Hz, prescaler %u, prop %u, phase1 %u, "
		       "phase2 %u, sjw %u (sample point 87.5%%)\n", b->name, core_clock,
		       t.prescaler, t.prop_seg, t.phase_seg1, t.phase_seg2, t.sjw);
	}

	err = can_add_rx_filter_msgq(b->dev, b->rxq, &all_std);
	if (err < 0) {
		printk("CAN %s: add filter failed (%d)\n", b->name, err);
		return;
	}

	err = can_start(b->dev);
	if (err != 0) {
		printk("CAN %s: start failed (%d)\n", b->name, err);
		return;
	}

	b->up = true;
	printk("CAN %s: started\n", b->name);
}

static void send_on(struct bus *b, struct can_frame *frame)
{
	int err;

	frame->data[1] = b->tag;
	err = can_send(b->dev, frame, K_NO_WAIT, tx_done, b);
	if (err == -EAGAIN) {
		b->tx_full++;
	} else if (err != 0) {
		b->tx_fail++;
	}
}

static void drain_rx(struct bus *b)
{
	struct can_frame f;

	while (k_msgq_get(b->rxq, &f, K_NO_WAIT) == 0) {
		b->rx++;
		printk("RX %s  id 0x%03x  [%u]", b->name, f.id, f.dlc);
		for (int i = 0; i < can_dlc_to_bytes(f.dlc); i++) {
			printk(" %02x", f.data[i]);
		}
		printk("\n");
	}
}

static void print_status(struct bus *b)
{
	enum can_state state;
	struct can_bus_err_cnt cnt;

	if (!b->up) {
		printk("STATUS %s: not running\n", b->name);
		return;
	}
	if (can_get_state(b->dev, &state, &cnt) != 0) {
		printk("STATUS %s: can_get_state failed\n", b->name);
		return;
	}
	/* INT is active low: 1 here = the MCP2515 is holding INT asserted */
	printk("STATUS %s: %-13s tec %3u rec %3u | tx ok %lu err %lu full %u fail %u | rx %u | INT %s\n",
	       b->name, state_str(state), cnt.tx_err_cnt, cnt.rx_err_cnt,
	       (unsigned long)atomic_get(&b->tx_ok), (unsigned long)atomic_get(&b->tx_err),
	       b->tx_full, b->tx_fail, b->rx,
	       gpio_pin_get_dt(&b->int_pin) ? "ASSERTED(low)" : "idle(high)");
}

int main(void)
{
	struct can_frame tx = { .id = TX_ID, .dlc = 3 };
	uint8_t counter = 0;
	int64_t next_tx = k_uptime_get();
	int64_t next_status = next_tx + STATUS_MS;

	printk("\nLab 3 CAN bring-up: node N%d, ID 0x%03x every %d ms on A and B\n",
	       NODE_ID, TX_ID, TX_PERIOD_MS);

	ARRAY_FOR_EACH_PTR(buses, b) {
		bus_init(b);
	}

	while (1) {
		tx.data[0] = counter++;
		tx.data[2] = NODE_ID;
		ARRAY_FOR_EACH_PTR(buses, b) {
			if (b->up) {
				send_on(b, &tx);
			}
		}

		ARRAY_FOR_EACH_PTR(buses, b) {
			drain_rx(b);
		}

		if (k_uptime_get() >= next_status) {
			ARRAY_FOR_EACH_PTR(buses, b) {
				print_status(b);
				mcp_dump(b);
			}
			next_status += STATUS_MS;
		}

		next_tx += TX_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next_tx));
	}

	return 0;
}
