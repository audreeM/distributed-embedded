#include <stdlib.h>
#include <zephyr/drivers/adc.h>
#include "config.h"
#include "link.h"

#define SENSE_PERIOD_MS 10
#define N_SAMPLES       8          /* small: 3 channels x N must finish well inside 10 ms */
#define R_TOP_OHM       10000
#define R_BOT_OHM       20000
#define SENS_UV_PER_A   185000     /* ACS712-05B */

/* order matches io-channels in the overlay: PC0, PA4, PC1 */
static const struct adc_dt_spec ch[3] = {
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),  /* motor L */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),  /* motor R */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),  /* servo   */
};
static int32_t zero_mv[3];

static int read_avg_mv(const struct adc_dt_spec *c, int32_t *out)
{
	int16_t raw;
	struct adc_sequence seq = { .buffer = &raw, .buffer_size = sizeof(raw) };
	int32_t sum = 0;

	adc_sequence_init_dt(c, &seq);
	for (int i = 0; i < N_SAMPLES; i++) {
		if (adc_read_dt(c, &seq)) {
			return -EIO;
		}
		int32_t mv = raw;
		adc_raw_to_millivolts_dt(c, &mv);
		sum += mv;
	}
	*out = sum / N_SAMPLES;
	return 0;
}

static uint16_t to_ma(int32_t delta_mv)
{
	int64_t uv = (int64_t)delta_mv * 1000 * (R_TOP_OHM + R_BOT_OHM) / R_BOT_OHM;
	int32_t ma = abs((int32_t)(uv * 1000 / SENS_UV_PER_A));  /* frame field is unsigned */
	return ma > UINT16_MAX ? UINT16_MAX : (uint16_t)ma;
}

K_TIMER_DEFINE(sense_timer, NULL, NULL);

static void sense_thread(void *p1, void *p2, void *p3)
{
	for (int i = 0; i < 3; i++) {
		if (!adc_is_ready_dt(&ch[i]) || adc_channel_setup_dt(&ch[i]) ||
		    read_avg_mv(&ch[i], &zero_mv[i])) {   /* boot calibration: motors off */
			printk("current sensor %d init failed\n", i);
			return;
		}
	}

	k_timer_start(&sense_timer, K_MSEC(SENSE_PERIOD_MS), K_MSEC(SENSE_PERIOD_MS));
	for (;;) {
		k_timer_status_sync(&sense_timer);
		int32_t mv[3];
		uint16_t ma[3] = {0};

		for (int i = 0; i < 3; i++) {
			if (read_avg_mv(&ch[i], &mv[i]) == 0) {
				ma[i] = to_ma(mv[i] - zero_mv[i]);
			}
		}
		link_set_currents(ma[0], ma[1], ma[2]);   /* next status_tick sends these */
	}
}

K_THREAD_DEFINE(sense_tid, STACK_SZ, sense_thread, NULL, NULL, NULL, PRIO_SENSE, 0, SYS_FOREVER_MS);
