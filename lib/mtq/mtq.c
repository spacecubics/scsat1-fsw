/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pwm.h>
#include "mtq.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mtq, CONFIG_SC_LIB_MTQ_LOG_LEVEL);

struct mtq_pwm_config {
	uint8_t channel;
	uint32_t period_cycles;
	uint32_t pulse_cycles;
};

struct mtq_config {
	struct mtq_pwm_config in1;
	struct mtq_pwm_config in2;
};

static const struct device *get_mtq_device(enum mtq_axes axes)
{
	const struct device *pwm0 = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	const struct device *pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));

	switch (axes) {
	case MTQ_AXES_MAIN_X:
	case MTQ_AXES_MAIN_Y:
	case MTQ_AXES_MAIN_Z:
		if (device_is_ready(pwm0)) {
			return pwm0;
		}
		break;
	case MTQ_AXES_BKUP_X:
	case MTQ_AXES_BKUP_Y:
	case MTQ_AXES_BKUP_Z:
		if (device_is_ready(pwm1)) {
			return pwm1;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int get_mtq_channel(enum mtq_axes axes, struct mtq_config *cfg)
{
	int ret = 0;

	switch (axes) {
	case MTQ_AXES_MAIN_X:
	case MTQ_AXES_BKUP_X:
		cfg->in1.channel = 1;
		cfg->in2.channel = 2;
		break;
	case MTQ_AXES_MAIN_Y:
	case MTQ_AXES_BKUP_Y:
		cfg->in1.channel = 3;
		cfg->in2.channel = 4;
		break;
	case MTQ_AXES_MAIN_Z:
	case MTQ_AXES_BKUP_Z:
		cfg->in1.channel = 5;
		cfg->in2.channel = 6;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int get_mtq_pwm_cfg(enum mtq_polarity pol, float duty, struct mtq_config *cfg)
{
	int ret = 0;

	/* Now fixed at 20KHZ */
	cfg->in1.period_cycles = PWM_KHZ(20);
	cfg->in2.period_cycles = PWM_KHZ(20);

	switch (pol) {
	case MTQ_POL_PLUS:
		/* IN1: Hi, IN2: Low */
		cfg->in1.pulse_cycles = cfg->in1.period_cycles * duty;
		cfg->in2.pulse_cycles = 0;
		break;
	case MTQ_POL_MINUS:
		/* IN1: Low, IN2: Hi */
		cfg->in1.pulse_cycles = 0;
		cfg->in2.pulse_cycles = cfg->in2.period_cycles * duty;
		break;
	case MTQ_POL_NON:
		/* IN1: Hi, IN2: Hi */
		cfg->in1.pulse_cycles = cfg->in1.period_cycles;
		cfg->in2.pulse_cycles = cfg->in2.period_cycles;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int mtq_start(enum mtq_axes axes, enum mtq_polarity pol, float duty)
{
	int ret;
	const struct device *pwm = get_mtq_device(axes);
	struct mtq_config cfg;
	pwm_flags_t flags = PWM_POLARITY_NORMAL;

	if (duty <= 0.0f || duty > 1.0f) {
		LOG_ERR("Invalid argument. duty: %.1f", (double)duty);
		ret = -EINVAL;
		goto end;
	}

	ret = get_mtq_channel(axes, &cfg);
	if (ret < 0) {
		LOG_ERR("Invalid argument. axes: %d", axes);
		goto end;
	}

	ret = get_mtq_pwm_cfg(pol, duty, &cfg);
	if (ret < 0) {
		LOG_ERR("Invalid argument. axes: %d", axes);
		goto end;
	}

	LOG_INF("CH: %d Period: %d Pulse: %d", cfg.in1.channel, cfg.in1.period_cycles,
		cfg.in1.pulse_cycles);
	ret = pwm_set_cycles(pwm, cfg.in1.channel, cfg.in1.period_cycles, cfg.in1.pulse_cycles,
			     flags);
	if (ret < 0) {
		LOG_ERR("Faile to start the PWM %s IN1. (%d)", mtq_axes_name[axes], ret);
	} else {
		LOG_INF("Start PWM %s IN1 to %s", mtq_axes_name[axes], mtq_pol_name[pol]);
	}

	LOG_INF("CH: %d Period: %d Pulse: %d", cfg.in2.channel, cfg.in2.period_cycles,
		cfg.in2.pulse_cycles);
	ret = pwm_set_cycles(pwm, cfg.in2.channel, cfg.in2.period_cycles, cfg.in2.pulse_cycles,
			     flags);
	if (ret < 0) {
		LOG_ERR("Faile to start the PWM %s IN2. (%d)", mtq_axes_name[axes], ret);
	} else {
		LOG_INF("Start PWM %s IN2 to %s", mtq_axes_name[axes], mtq_pol_name[pol]);
	}
end:
	return ret;
}

int mtq_stop(enum mtq_axes axes)
{
	int ret;
	const struct device *pwm = get_mtq_device(axes);
	struct mtq_config cfg;
	pwm_flags_t flags = PWM_POLARITY_NORMAL;

	ret = get_mtq_channel(axes, &cfg);
	if (ret < 0) {
		LOG_ERR("Invalid argument. axes: %d", axes);
		goto end;
	}

	ret = pwm_set_cycles(pwm, cfg.in1.channel, 1, 0, flags);
	if (ret < 0) {
		LOG_ERR("Faile to stop the PWM %s IN1. (%d)", mtq_axes_name[axes], ret);
	} else {
		LOG_INF("Stop PWM %s IN1.", mtq_axes_name[axes]);
	}

	ret = pwm_set_cycles(pwm, cfg.in2.channel, 1, 0, flags);
	if (ret < 0) {
		LOG_ERR("Faile to stop the PWM %s IN2. (%d)", mtq_axes_name[axes], ret);
	} else {
		LOG_INF("Stop PWM %s IN2.", mtq_axes_name[axes]);
	}
end:
	return ret;
}
