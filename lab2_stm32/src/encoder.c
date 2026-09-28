/*
 * Encoders: TIM3/TIM4 count edges in hardware (encoder mode).
 * We read the 16-bit counter and keep a 32-bit signed total.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "encoder.h"
#include "config.h"

/* With counts-per-revolution = 1 the counter wraps every 65535 counts. */
#define COUNTER_PERIOD 65535

static const struct device *const qdec[2] = {
	DEVICE_DT_GET(DT_ALIAS(qdec_left)),
	DEVICE_DT_GET(DT_ALIAS(qdec_right)),
};
static const bool invert[2] = { ENC_L_INVERT, ENC_R_INVERT };

static int32_t last_raw[2];
static int32_t total[2];

static int32_t read_raw(int side)
{
	struct sensor_value v;

	sensor_sample_fetch_chan(qdec[side], SENSOR_CHAN_ENCODER_COUNT);
	sensor_channel_get(qdec[side], SENSOR_CHAN_ENCODER_COUNT, &v);
	return v.val1;
}

int encoder_init(void)
{
	for (int s = 0; s < 2; s++) {
		if (!device_is_ready(qdec[s])) {
			return -ENODEV;
		}
		last_raw[s] = read_raw(s);
	}
	return 0;
}

int32_t encoder_read(int side)
{
	int32_t raw = read_raw(side);
	int32_t delta = raw - last_raw[side];

	/* Counter wrapped around: take the short way. */
	if (delta > COUNTER_PERIOD / 2) {
		delta -= COUNTER_PERIOD;
	} else if (delta < -COUNTER_PERIOD / 2) {
		delta += COUNTER_PERIOD;
	}
	last_raw[side] = raw;

	total[side] += invert[side] ? -delta : delta;
	return total[side];
}
