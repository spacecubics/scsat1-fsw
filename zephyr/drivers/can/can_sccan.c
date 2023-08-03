/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_can

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sc_can, CONFIG_CAN_LOG_LEVEL);

/* Registers */
#define SCCAN_ENR_OFFSET    (0x0000) /* CAN Enable Register */
#define SCCAN_TQPR_OFFSET   (0x0008) /* CAN Time Quantum Prescaler Register */
#define SCCAN_BTSR_OFFSET   (0x000C) /* CAN Bit Timing Setting Register */
#define SCCAN_ECNTR_OFFSET  (0x0010) /* CAN Error Count Register */
#define SCCAN_STSR_OFFSET   (0x0018) /* CAN Status Register */
#define SCCAN_FIFOSR_OFFSET (0x001C) /* CAN FIFO Status Register */
#define SCCAN_ISR_OFFSET    (0x0020) /* CAN Interrupt Status Register */
#define SCCAN_IER_OFFSET    (0x0024) /* CAN Interrupt Enable Register */
#define SCCAN_TMR1_OFFSET   (0x0030) /* CAN TX Message Register1 */
#define SCCAN_TMR2_OFFSET   (0x0034) /* CAN TX Message Register2 */
#define SCCAN_TMR3_OFFSET   (0x0038) /* CAN TX Message Register3 */
#define SCCAN_TMR4_OFFSET   (0x003C) /* CAN TX Message Register4 */
#define SCCAN_THPMR1_OFFSET (0x0040) /* CAN TX High Priority Message Register1 */
#define SCCAN_THPMR2_OFFSET (0x0044) /* CAN TX High Priority Message Register2 */
#define SCCAN_THPMR3_OFFSET (0x0048) /* CAN TX High Priority Message Register3 */
#define SCCAN_THPMR4_OFFSET (0x004C) /* CAN TX High Priority Message Register4 */
#define SCCAN_RMR1_OFFSET   (0x0050) /* CAN RX Message Register1 */
#define SCCAN_RMR2_OFFSET   (0x0054) /* CAN RX Message Register2 */
#define SCCAN_RMR3_OFFSET   (0x0058) /* CAN RX Message Register3 */
#define SCCAN_RMR4_OFFSET   (0x005C) /* CAN RX Message Register4 */
#define SCCAN_AFER_OFFSET   (0x0060) /* CAN Acceptance Filter Enable Register */
#define SCCAN_AFIMR1_OFFSET (0x0070) /* CAN Acceptance Filter ID Mask Register1 */
#define SCCAN_AFIVR1_OFFSET (0x0074) /* CAN Acceptance Filter ID Value Register1 */
#define SCCAN_AFIMR2_OFFSET (0x0090) /* CAN Acceptance Filter ID Mask Register2 */
#define SCCAN_AFIVR2_OFFSET (0x0094) /* CAN Acceptance Filter ID Value Register2 */
#define SCCAN_AFIMR3_OFFSET (0x00B0) /* CAN Acceptance Filter ID Mask Register3 */
#define SCCAN_AFIVR3_OFFSET (0x00B4) /* CAN Acceptance Filter ID Value Register3 */
#define SCCAN_AFIMR4_OFFSET (0x00D0) /* CAN Acceptance Filter ID Mask Register4 */
#define SCCAN_AFIVR4_OFFSET (0x00D4) /* CAN Acceptance Filter ID Value Register4 */
#define SCCAN_FIFORR_OFFSET (0x00F0) /* CAN FIFO and Buffer Reset Register */
#define SCCAN_STMCR_OFFSET  (0x0100) /* CAN Self Test Mode Control Register */
#define SCCAN_PSLMCR_OFFSET (0x0200) /* CAN PHY Sleep Mode Control Register */
#define SCCAN_VER_OFFSET    (0xF000) /* CAN Controller IP Version Register */

/* CAN Enable Register */
#define SCCAN_CANEN BIT(0)

/* CAN Bit Timing Setting Register */
#define SCCAN_BTSR_SJW(x) ((x) << 7)
#define SCCAN_BTSR_TS2(x) ((x) << 4)
#define SCCAN_BTSR_TS1(x) ((x))

/* CAN Status Register */
#define SCCAN_RXFFL              BIT(7)
#define SCCAN_TXFFL              BIT(6)
#define SCCAN_TXHBFL             BIT(5)
#define SCCAN_TXFNEP             BIT(4)
#define SCCAN_ESTS(x)            (((x)&GENMASK(3, 2)) >> 2)
#define SCCAN_EWRN               BIT(1)
#define SCCAN_BBUSY              BIT(0)
#define SCCAN_ESTS_CAN_DISABLE   (0U)
#define SCCAN_ESTS_ERROR_ACTIVE  (1U)
#define SCCAN_ESTS_ERROR_PASSIVE (2U)
#define SCCAN_ESTS_BUS_OFF       (3U)

/* CAN Interrupt Enable Register */
#define SCCAN_BUSOFFENB  BIT(13)
#define SCCAN_ACKERENB   BIT(12)
#define SCCAN_BITERENB   BIT(11)
#define SCCAN_STFERENB   BIT(10)
#define SCCAN_FMERENB    BIT(9)
#define SCCAN_CRCERENB   BIT(8)
#define SCCAN_RXFOVFENB  BIT(7)
#define SCCAN_RXFUDFENB  BIT(6)
#define SCCAN_RXFVALENB  BIT(5)
#define SCCAN_RCVDNENB   BIT(4)
#define SCCAN_TXFOVFENB  BIT(3)
#define SCCAN_TXHBOVFENB BIT(2)
#define SCCAN_ARBLSTENB  BIT(1)
#define SCCAN_TRNSDNENB  BIT(0)
#define SCCAN_IER_ALL                                                                              \
	(SCCAN_BUSOFFENB | SCCAN_ACKERENB | SCCAN_BITERENB | SCCAN_STFERENB | SCCAN_FMERENB |      \
	 SCCAN_CRCERENB | SCCAN_RXFOVFENB | SCCAN_RXFUDFENB | SCCAN_RXFVALENB | SCCAN_RCVDNENB |   \
	 SCCAN_TXFOVFENB | SCCAN_TXHBOVFENB | SCCAN_ARBLSTENB | SCCAN_TRNSDNENB)

/* CAN Controller IP Version Register */
#define SCCAN_VER_MAJOR(x) (((x)&0xff000000) >> 24)
#define SCCAN_VER_MINOR(x) (((x)&0x00ff0000) >> 16)
#define SCCAN_VER_PATCH(x) (((x)&0x0000ffff) >> 0)

/* CAN FIFO and Buffer Reset Register */
#define SCCAN_FIFORR_TXHPBRST  BIT(17)
#define SCCAN_FIFORR_TXFIFORST BIT(16)
#define SCCAN_FIFORR_RXFIFORST BIT(0)
#define SCCAN_FIFO_CLEAR_ALL                                                                       \
	(SCCAN_FIFORR_TXHPBRST | SCCAN_FIFORR_TXFIFORST | SCCAN_FIFORR_RXFIFORST)

/* Timeout configuration for enable/disable CAN */
#define SCCAN_ENABLE_RETRIES     (10U)
#define SCCAN_ENABLE_DELAY_USEC  K_USEC(10)
#define SCCAN_DISABLE_RETRIES    (10U)
#define SCCAN_DISABLE_DELAY_MSEC K_MSEC(10)

/* Timeout parameter for mutex_lock */
#define SCCAN_MUTEX_LOCK_TIMEOUT K_MSEC(100)

typedef void (*irq_init_func_t)(const struct device *dev);

struct sc_can_cfg {
	uint32_t reg_addr;
	irq_init_func_t irq_init;
	uint32_t clock_frequency;
	uint32_t bus_speed;
	uint8_t sjw;
	uint16_t sample_point;
	uint32_t max_bitrate;
};

struct sc_can_data {
	/*
	 * These mutex protects:
	 *  - Enabling/Disabling operations for CAN at the same time
	 *      (Including operations that require CAN to stop)
	 *  - Set CAN Timing operations
	 *      (need to set the multiple register)
	 */
	struct k_mutex enadis_mutex;
	struct k_mutex timing_mutex;
};

static inline uint32_t sc_can_get_status_reg(const struct sc_can_cfg *config)
{
	return sys_read32(config->reg_addr + SCCAN_STSR_OFFSET);
}

static bool sc_can_is_enabled(const struct sc_can_cfg *config)
{
	uint32_t status_reg;

	status_reg = sc_can_get_status_reg(config);

	/*
	 * CAN IP core prepares the error status as below.
	 *   - 0b00: CAN_EN Disable
	 *   - 0b01: Error Active
	 *   - 0b10: Error Passive
	 *   - 0b11: Bus OFF
	 * When CAN IP core starts CAN communication, it transitions to
	 * one of the statuses other than 0b00 (CAN_EN Disable).
	 */
	if (SCCAN_ESTS(status_reg) == SCCAN_ESTS_CAN_DISABLE) {
		return false;
	}

	return true;
}

static bool sc_can_is_disabled(const struct sc_can_cfg *config)
{
	return !sc_can_is_enabled(config);
}

static int sc_can_enable(const struct sc_can_cfg *config)
{
	int ret = -EIO;
	int retries = SCCAN_ENABLE_RETRIES;

	/*
	 * When set the CAN_EN register to `Enable`, it becomes an ERROR_ACTIVE
	 * state after detecting consecutive 11-bit recessive on the CAN bus.
	 */
	while (retries-- > 0) {
		sys_set_bits(config->reg_addr + SCCAN_ENR_OFFSET, SCCAN_CANEN);
		if (sc_can_is_enabled(config)) {
			ret = 0;
			break;
		} else {
			k_sleep(SCCAN_ENABLE_DELAY_USEC);
		}
	}

	if (ret != 0) {
		LOG_ERR("Timeout trying to enable CAN");
	}

	return ret;
}

static int sc_can_disable(const struct sc_can_cfg *config)
{
	int ret = -EBUSY;
	int retries = SCCAN_DISABLE_RETRIES;

	/*
	 * If CAN Controller is sending or receiving the frame,
	 * writing of CAN DISABLE register might not be reflected,
	 * so retry a certain number of times until confirm disabled CAN.
	 */
	while (retries-- > 0) {
		sys_clear_bits(config->reg_addr + SCCAN_ENR_OFFSET, SCCAN_CANEN);
		if (sc_can_is_disabled(config)) {
			ret = 0;
			break;
		} else {
			k_sleep(SCCAN_DISABLE_DELAY_MSEC);
		}
	}

	if (ret != 0) {
		LOG_ERR("Timeout trying to disable CAN");
	}

	return ret;
}

static int sc_can_get_state(const struct device *dev, enum can_state *state,
			    struct can_bus_err_cnt *err_cnt)
{
	return 0;
}

static void sc_can_isr(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	uint32_t isr;

	isr = sys_read32(config->reg_addr + SCCAN_ISR_OFFSET);
	LOG_DBG("IRQ Status 0x%08x", isr);

	sys_write32(isr, config->reg_addr + SCCAN_ISR_OFFSET);
}

static inline void sc_can_clear_all_fifo(const struct sc_can_cfg *config)
{
	sys_set_bits(config->reg_addr + SCCAN_FIFORR_OFFSET, SCCAN_FIFO_CLEAR_ALL);
}

static inline void sc_can_set_time_quantum_prescaler(const struct sc_can_cfg *config,
						     const struct can_timing *timing)
{
	sys_write32(timing->prescaler - 1, config->reg_addr + SCCAN_TQPR_OFFSET);
}

static void sc_can_set_bit_timing(const struct sc_can_cfg *config, const struct can_timing *timing)
{
	uint32_t timing_reg = 0;

	timing_reg |= SCCAN_BTSR_TS1(timing->prop_seg + timing->phase_seg1 - 1);
	timing_reg |= SCCAN_BTSR_TS2(timing->phase_seg2 - 1);
	timing_reg |= SCCAN_BTSR_SJW(timing->sjw - 1);
	sys_write32(timing_reg, config->reg_addr + SCCAN_BTSR_OFFSET);
}

static void sc_can_enable_irq(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;

	sys_set_bits(config->reg_addr + SCCAN_IER_OFFSET, SCCAN_IER_ALL);
	config->irq_init(dev);
}

static int sc_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	return 0;
}

static int sc_can_start(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;

	if (sc_can_is_enabled(config)) {
		ret = -EALREADY;
		goto nolock;
	}

	ret = k_mutex_lock(&data->enadis_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	ret = sc_can_enable(config);

	k_mutex_unlock(&data->enadis_mutex);

nolock:
	return ret;
}

static int sc_can_stop(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;

	if (!sc_can_is_enabled(config)) {
		ret = -EALREADY;
		goto nolock;
	}

	ret = k_mutex_lock(&data->enadis_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	ret = sc_can_disable(config);
	if (ret == 0) {
		sc_can_clear_all_fifo(config);
	}

	k_mutex_unlock(&data->enadis_mutex);

nolock:
	return ret;
}

static int sc_can_set_mode(const struct device *dev, can_mode_t mode)
{
	return 0;
}

static int sc_can_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret = 0;

	if (sc_can_is_enabled(config)) {
		LOG_ERR("Failed to set timing because enabled CAN. Must be disabled to set "
			"timing.");
		ret = -EBUSY;
		goto nolock;
	}

	LOG_DBG("Presc: %d, TS1: %d, TS2: %d, SJW: %d", timing->prescaler, timing->phase_seg1,
		timing->phase_seg2, timing->sjw);

	ret = k_mutex_lock(&data->timing_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	sc_can_set_time_quantum_prescaler(config, timing);

	sc_can_set_bit_timing(config, timing);

	k_mutex_unlock(&data->timing_mutex);

nolock:
	return ret;
}

static void sc_can_set_state_change_callback(const struct device *dev,
					     can_state_change_callback_t cb, void *user_data)
{
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int sc_can_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct sc_can_cfg *config = dev->config;

	ARG_UNUSED(timeout);

	if (!sc_can_is_enabled(config)) {
		return -ENETDOWN;
	}

	return -ENOTSUP;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

static int sc_can_send(const struct device *dev, const struct can_frame *frame, k_timeout_t timeout,
		       can_tx_callback_t callback, void *user_data)
{
	return 0;
}

static int sc_can_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *cb_arg,
				const struct can_filter *filter)
{
	return 0;
}

static void sc_can_remove_rx_filter(const struct device *dev, int filter_id)
{
}

static int sc_can_init(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct can_timing timing;
	int32_t ret;
	uint32_t v;

	/* Set timing according to dts default setting */
	timing.sjw = config->sjw;
	ret = can_calc_timing(dev, &timing, config->bus_speed, config->sample_point);
	if (ret == -EINVAL) {
		LOG_ERR("Can't find timing for given param");
		return -EIO;
	}
	sc_can_set_timing(dev, &timing);

	sc_can_enable_irq(dev);

	/* Dump Version information */
	v = sys_read32(config->reg_addr + SCCAN_VER_OFFSET);
	LOG_DBG("Space Cubics CAN controller v%d.%d.%d initialized", SCCAN_VER_MAJOR(v),
		SCCAN_VER_MINOR(v), SCCAN_VER_PATCH(v));

	return 0;
}

static int sc_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct sc_can_cfg *config = dev->config;

	*rate = config->clock_frequency;

	return 0;
}

static int sc_can_get_max_filters(const struct device *dev, bool ide)
{
	return 0;
}

static int sc_can_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct sc_can_cfg *config = dev->config;

	*max_bitrate = config->max_bitrate;

	return 0;
}

static const struct can_driver_api sc_can_driver_api = {
	.get_capabilities = sc_can_get_capabilities,
	.start = sc_can_start,
	.stop = sc_can_stop,
	.set_mode = sc_can_set_mode,
	.set_timing = sc_can_set_timing,
	.send = sc_can_send,
	.add_rx_filter = sc_can_add_rx_filter,
	.remove_rx_filter = sc_can_remove_rx_filter,
	.get_state = sc_can_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = sc_can_recover,
#endif
	.set_state_change_callback = sc_can_set_state_change_callback,
	.get_core_clock = sc_can_get_core_clock,
	.get_max_filters = sc_can_get_max_filters,
	.get_max_bitrate = sc_can_get_max_bitrate,
	/*
	 * ISO 11898-1:
	 *   Nominal length of time segments (in absence of synchronization)
	 *   - Sync_Seg shall be one time quantum long.
	 *   - The information processing time shall be less than or equal to two time quanta long.
	 *   - Prop_Seg shall be programmable to be 1, 2, 3, . . ., 8 or more time quanta long.
	 *     It shall be programmed to compensate the delay times of the actual network,
	 *     rounded up to the nearest integer time quantum.
	 *   - Phase_Seg1 shall be programmable to be 1, 2, 3, . . ., 8 or more time quanta long.
	 *   - Phase_Seg2 shall be programmed to the maximum of Phase_Seg1 and the information processing
	 *     time.
	 *   - SJW shall be programmable between 1 and the minimum of Phase_Seg1 and 4.
	 *
	 *   In a CAN implementation, Prop_Seg and Phase_Seg1 do not need to be programmable separately;
	 *   it is sufficient to program the sum of Prop_Seg and Phase_Seg1.
	 *   The total number of time quanta in a bit time shall be programmable at least from 8 to 25.
	 * In this driver:
	 *  - SJW
	 *    set to 1 - 4.
	 *  - prop_seg
	 *    set to 1 - 8.
	 *  - phase_seg1
	 *    set to 1 - 8.
	 *  - phase_seg2
	 *    set to 1 - 8.
	 *  - prescaler
	 *    SC CAN IP core have 16 bit register value for set the prescaler. so set to 0x1000.
	 */
	.timing_min = { .sjw = 0x1,
			.prop_seg = 0x01,
			.phase_seg1 = 0x01,
			.phase_seg2 = 0x01,
			.prescaler = 0x01 },
	.timing_max = { .sjw = 0x4,
			.prop_seg = 0x08,
			.phase_seg1 = 0x08,
			.phase_seg2 = 0x08,
			.prescaler = 0x1000 }
};

#define SCCAN_INIT(n)                                                                              \
	static void sc_can_##n##_irq_init(const struct device *dev);                               \
	static const struct sc_can_cfg sc_can_cfg_##n = {                                          \
		.reg_addr = DT_INST_REG_ADDR(n),                                                   \
		.irq_init = sc_can_##n##_irq_init,                                                 \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
		.bus_speed = DT_INST_PROP(n, bus_speed),                                           \
		.sjw = DT_INST_PROP(n, sjw),                                                       \
		.max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(n, 1000000),                    \
	};                                                                                         \
	static struct sc_can_data sc_can_data_##n = {                                              \
		.enadis_mutex = Z_MUTEX_INITIALIZER(sc_can_data_##n.enadis_mutex),                 \
		.timing_mutex = Z_MUTEX_INITIALIZER(sc_can_data_##n.timing_mutex),                 \
	};                                                                                         \
	CAN_DEVICE_DT_INST_DEFINE(n, sc_can_init, NULL, &sc_can_data_##n, &sc_can_cfg_##n,         \
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, &sc_can_driver_api);      \
	static void sc_can_##n##_irq_init(const struct device *dev)                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, sc_can_isr, DEVICE_DT_INST_GET(n), 0);             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(SCCAN_INIT)
