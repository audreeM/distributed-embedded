#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

static const struct adc_dt_spec adc_ch =
	ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Hardware constants */
#define R_TOP_OHM      10000
#define R_BOT_OHM      20000
#define SENS_UV_PER_A  185000      /* ACS712-05B: 185 mV/A */
#define N_SAMPLES      64
#define FULL_SCALE_MA  5000

static int read_avg_mv(int32_t *mv_out)
{
	int16_t raw;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int32_t sum = 0;

	adc_sequence_init_dt(&adc_ch, &seq);

	for (int i = 0; i < N_SAMPLES; i++) {
		int err = adc_read_dt(&adc_ch, &seq);
		if (err) {
			return err;
		}
		int32_t mv = raw;
		err = adc_raw_to_millivolts_dt(&adc_ch, &mv);
		if (err) {
			return err;
		}
		sum += mv;
	}
	*mv_out = sum / N_SAMPLES;
	return 0;
}

/* pin delta (mV) -> sensor delta (mV) -> mA */
static int32_t delta_mv_to_ma(int32_t delta_mv)
{
	int64_t sensor_uv = (int64_t)delta_mv * 1000 *
			    (R_TOP_OHM + R_BOT_OHM) / R_BOT_OHM;
	return (int32_t)(sensor_uv * 1000 / SENS_UV_PER_A);
}

int main(void)
{
	int32_t zero_mv, mv;

	if (!adc_is_ready_dt(&adc_ch) || adc_channel_setup_dt(&adc_ch)) {
		printk("ADC setup failed\n");
		return 0;
	}

	/* Zero-current calibration: keep the load disconnected at boot */
	if (read_avg_mv(&zero_mv)) {
		printk("ADC read failed\n");
		return 0;
	}
	printk("Zero offset at pin: %d mV (expect ~1667)\n", zero_mv);

	while (1) {
		if (read_avg_mv(&mv) == 0) {
			int32_t ma = delta_mv_to_ma(mv - zero_mv);

			printk("pin=%d mV  current=%d mA%s\n", mv, ma,
			       (ma > FULL_SCALE_MA || ma < -FULL_SCALE_MA)
				       ? "  [OUT OF RATED RANGE]" : "");
		}
		k_msleep(500);
	}
	return 0;
}