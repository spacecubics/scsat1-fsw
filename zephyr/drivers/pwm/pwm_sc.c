/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_pwm

#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_sc, CONFIG_PWM_LOG_LEVEL);

/* Registers */
#define SC_PWM_ISR_OFFSET       (0x0000) /* PWM Interrupt Status Register */
#define SC_PWM_IER_OFFSET       (0x0004) /* PWM Interrupt Enable Register */
#define SC_PWM_OUTENR_OFFSET    (0x0010) /* PWM Output Enable Register */
#define SC_PWM_OPCYCCR_OFFSET   (0x0014) /* PWM Output Pulse Cycle Control Register */
#define SC_PWM_OPSYUPR_OFFSET   (0x0018) /* PWM Output Pulse Sync Update Register */
#define SC_PWM_OPEGCR01_OFFSET  (0x0030) /* PWM Output Pulse Edge Control Register1 */
#define SC_PWM_OAONUMR01_OFFSET (0x0070) /* PWM Output Auto Off Num Register1 */
#define SC_PWM_OAOTIMR01_OFFSET (0x00B0) /* PWM Output Auto Off Time Register1 */
#define SC_PWM_INENR_OFFSET     (0x0100) /* PWM Input Enable Register */
#define SC_PWM_IMPSCR_OFFSET    (0x0104) /* PWM Input Measurement Prescale Register */
#define SC_PWM_IMEGSELR_OFFSET  (0x0108) /* PWM Input Measurement Edge Select Register */
#define SC_PWM_IMUPMDR_OFFSET   (0x010C) /* PWM Input Measurement Update Mode Register */
#define SC_PWM_IMVR01_OFFSET    (0x0110) /* PWM Input Measurement Value Register1 */
#define SC_PWM_VER_OFFSET       (0xF000) /* AHB PWM Controller IP Version Register */

/* PWM Output Pulse Cycle Control Register */
#define SC_PWM_OPPSC(x) ((x) << 16)
#define SC_PWM_OPPER(x) ((x))

/* PWM Output Pulse Edge Control Register */
#define SC_PWM_OPPOSEG(x) ((x) << 16)
#define SC_PWM_OPNEGEG(x) ((x))

/* AHB PWM Controller IP Version Register */
#define SC_PWM_VER_MAJOR(x) (((x)&0xff000000) >> 24)
#define SC_PWM_VER_MINOR(x) (((x)&0x00ff0000) >> 16)
#define SC_PWM_VER_PATCH(x) (((x)&0x0000ffff) >> 0)

/* Clock cycle period based on a 48MHz system clock. */
#define CLK_CYCLE_TIME_NS PWM_KHZ(48000)

struct sc_pwm_cfg {
	mm_reg_t base;
	uint32_t max_out_channel;
};

static inline void sc_pwm_enable(const struct sc_pwm_cfg *cfg, uint32_t channel)
{
	sys_set_bits(cfg->base + SC_PWM_OUTENR_OFFSET, BIT(channel-1));
}

static inline void sc_pwm_disable(const struct sc_pwm_cfg *cfg, uint32_t channel)
{
	sys_clear_bits(cfg->base + SC_PWM_OUTENR_OFFSET, BIT(channel-1));
}

static inline void sc_pwm_set_always_hi(const struct sc_pwm_cfg *cfg, uint32_t channel)
{
	sys_write32(0, cfg->base + SC_PWM_OPEGCR01_OFFSET + ((channel-1) * 4));
}

/*
 * The implementation of this API is experimental and has only been
 * tested with a 10kHz PWM. Additionally, the current PWM IP core has
 * a common setting for the Output Pulse Cycle Control Register across
 * all channels. Therefore, further consideration is needed if we want
 * to achieve different frequencies across multiple channels.
 */
static void sc_pwm_set_pulse_edge(const struct sc_pwm_cfg *cfg, uint32_t channel,
			uint32_t period_cycles, uint32_t pulse_cycles)
{
	mm_reg_t addr = cfg->base + SC_PWM_OPEGCR01_OFFSET + ((channel-1) * 4);
	uint32_t period_count = 100;
	uint32_t prescale_cout;
	uint8_t pos_start_count = 0;
	uint8_t neg_start_count;
	uint32_t prescale_time;
	uint32_t period_time;

	LOG_DBG("channel=%d, pulse_cycles=%d[ns], period_cycles=%d[ns], duty_cycle=%d[%%]",
		channel, pulse_cycles, period_cycles, (pulse_cycles * 100U / period_cycles));

	/* Set Output Pulse Cycle Control Register */
	period_count = 100; /* Now Fixed */
	prescale_cout = period_cycles / CLK_CYCLE_TIME_NS / period_count;
	LOG_DBG("Output Period count: %d", prescale_cout);
	LOG_DBG("Output Prescale cout: %d", prescale_cout);

	sys_write32(SC_PWM_OPPSC(prescale_cout - 1) | SC_PWM_OPPER(period_count - 1),
				cfg->base + SC_PWM_OPCYCCR_OFFSET);

	/* PWM Output Pulse Edge Control Register */
	prescale_time = CLK_CYCLE_TIME_NS * (prescale_cout);
	period_time = prescale_time * (period_count);
	LOG_DBG("Output prescale time: %d [ns]", prescale_time);
	LOG_DBG("Output period time: %d [ns]", period_time);

	neg_start_count = pulse_cycles / (prescale_cout * CLK_CYCLE_TIME_NS);
	LOG_DBG("Output Pos time: %d[ns]", prescale_time * neg_start_count);

	sys_write32(SC_PWM_OPPOSEG(pos_start_count) | SC_PWM_OPNEGEG(neg_start_count), addr);
}

static int sc_pwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct sc_pwm_cfg *cfg = dev->config;
	int ret = 0;

	if (channel == 0 || channel > cfg->max_out_channel) {
		LOG_ERR("Invalid chanell number: %d (Range: 1 - %d)",
				channel, cfg->max_out_channel);
		ret = -ENOTSUP;
		goto end;
	}

	if (flags || PWM_POLARITY_NORMAL != 0) {
		LOG_ERR("SC OBC supports only the active-high pulse polarity");
		ret = -ENOTSUP;
		goto end;
	}

	/*
	 * Zephyr API does not have a disable function; instead,
	 * user should set it to pulse at 0.
	 * Zephyr document said:
	 *  "Passing 0 as pulse will cause the pin to be driven to a
	 *   constant inactive level."
	 */
	if (pulse_cycles == 0) {
		sc_pwm_disable(cfg, channel);
		goto end;
	}

	if (period_cycles == pulse_cycles) {
		sc_pwm_set_always_hi(cfg, channel);
	} else {
		sc_pwm_set_pulse_edge(cfg, channel, period_cycles, pulse_cycles);
	}

	sc_pwm_enable(cfg, channel);

end:
	return ret;
}

static int sc_pwm_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	/* Not implemented yet */
	return -ENOTSUP;;
}

static int sc_pwm_init(const struct device *dev)
{
	const struct sc_pwm_cfg *cfg = dev->config;
	uint32_t v;

	/* Dump Version information */
	v = sys_read32(cfg->base + SC_PWM_VER_OFFSET);
	LOG_DBG("Space Cubics PWM controller v%d.%d.%d initialized",
			SC_PWM_VER_MAJOR(v), SC_PWM_VER_MINOR(v), SC_PWM_VER_PATCH(v));

	return 0;
}

static const struct pwm_driver_api sc_pwm_driver_api = {
	.set_cycles = sc_pwm_set_cycles,
	.get_cycles_per_sec = sc_pwm_get_cycles_per_sec,
};

#define PWM_SC_INIT(n)								\
	static const struct sc_pwm_cfg sc_pwm_cfg_##n = {			\
		.base = DT_INST_REG_ADDR(n),					\
		.max_out_channel = DT_INST_PROP(n, max_out_channel)		\
	};									\
	DEVICE_DT_INST_DEFINE(n, sc_pwm_init, NULL, NULL, &sc_pwm_cfg_##n,	\
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,		\
			      &sc_pwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SC_INIT)
