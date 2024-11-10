/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_can

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

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

/* CAN Error Count Register */
#define SCCAN_RXECNT(x) (((x) & GENMASK(15, 8)) >> 8)
#define SCCAN_TXECNT(x) (((x) & GENMASK(7, 0)))

/* CAN Bit Timing Setting Register */
#define SCCAN_BTSR_SJW(x) ((x) << 7)
#define SCCAN_BTSR_TS2(x) ((x) << 4)
#define SCCAN_BTSR_TS1(x) ((x))

/* CAN Status Register */
#define SCCAN_RXFFL              BIT(7)
#define SCCAN_TXFFL              BIT(6)
#define SCCAN_TXHBFL             BIT(5)
#define SCCAN_TXFNEP             BIT(4)
#define SCCAN_ESTS(x)            (((x) & GENMASK(3, 2)) >> 2)
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
#define SCCAN_VER_MAJOR(x) (((x) & 0xff000000) >> 24)
#define SCCAN_VER_MINOR(x) (((x) & 0x00ff0000) >> 16)
#define SCCAN_VER_PATCH(x) (((x) & 0x0000ffff) >> 0)

/* CAN Interrupt Status Register */
#define SCCAN_BUSOFF  BIT(13)
#define SCCAN_ACKER   BIT(12)
#define SCCAN_BITER   BIT(11)
#define SCCAN_STFER   BIT(10)
#define SCCAN_FMER    BIT(9)
#define SCCAN_CRCER   BIT(8)
#define SCCAN_RXFOVF  BIT(7)
#define SCCAN_RXFUDF  BIT(6)
#define SCCAN_RXFVAL  BIT(5)
#define SCCAN_RCVDN   BIT(4)
#define SCCAN_TXFOVF  BIT(3)
#define SCCAN_TXHBOVF BIT(2)
#define SCCAN_ARBLST  BIT(1)
#define SCCAN_TRNSDN  BIT(0)

/* CAN TX Message Register1 */
#define SCCAN_TXID1(x)    (x << 21)
#define SCCAN_TXSRTR(x)   (x << 20)
#define SCCAN_TXIDE(x)    (x << 19)
#define SCCAN_TXID1(x)    (x << 21)
#define SCCAN_TXID_EX1(x) ((x & GENMASK(28, 18)) << 3)
#define SCCAN_TXID_EX2(x) ((x & GENMASK(17, 0)) << 1)
#define SCCAN_TXERTR(x)   (x)

/* CAN RX Message Register1 */
#define SCCAN_RXID1_STD(x)        ((x & GENMASK(31, 21)) >> 21)
#define SCCAN_RXID1_EXT(x)        ((x & GENMASK(31, 21)) >> 3)
#define SCCAN_RX_STD_REMOTE_FRAME BIT(20)
#define SCCAN_RX_EXT_FRAME        BIT(19)
#define SCCAN_RXID2_EXT(x)        ((x & GENMASK(18, 1)) >> 1)
#define SCCAN_RX_EXT_REMOTE_FRAME BIT(0)

/* CAN RX Message Register2: */
#define SCCAN_DLC(x) (x & GENMASK(3, 0))

/* CAN Acceptance Filter ID Mask Register */
#define SCCAN_AFID1(x)    (x << 21)
#define SCCAN_AFSRTR(x)   (x << 20)
#define SCCAN_AFIDE(x)    (x << 19)
#define SCCAN_AFID1(x)    (x << 21)
#define SCCAN_AFID_EX1(x) ((x & GENMASK(28, 18)) << 3)
#define SCCAN_AFID_EX2(x) ((x & GENMASK(17, 0)) << 1)
#define SCCAN_AFERTR(x)   (x)

/* CAN FIFO and Buffer Reset Register */
#define SCCAN_FIFORR_TXHPBRST  BIT(17)
#define SCCAN_FIFORR_TXFIFORST BIT(16)
#define SCCAN_FIFORR_RXFIFORST BIT(0)
#define SCCAN_FIFO_CLEAR_ALL                                                                       \
	(SCCAN_FIFORR_TXHPBRST | SCCAN_FIFORR_TXFIFORST | SCCAN_FIFORR_RXFIFORST)

/* CAN Self Test Mode Control Register */
#define SCCAN_STM_DISABLE (0U)
#define SCCAN_STM_ENABLE  (1U)

/* Timeout configuration for enable/disable CAN */
#define SCCAN_ENABLE_RETRIES     (10U)
#define SCCAN_ENABLE_DELAY_USEC  K_USEC(10)
#define SCCAN_DISABLE_RETRIES    (10U)
#define SCCAN_DISABLE_DELAY_MSEC K_MSEC(10)

/* Timeout parameter for mutex_lock */
#define SCCAN_MUTEX_LOCK_TIMEOUT K_MSEC(100)

typedef void (*irq_init_func_t)(const struct device *dev);

struct sc_can_cfg {
	const struct can_driver_config common;
	uint32_t reg_addr;
	irq_init_func_t irq_init;
	uint32_t clock_frequency;
	uint8_t tx_fifo_depth;
	uint8_t max_filter;
};

struct sc_can_tx_cb_data {
	can_tx_callback_t tx_cb;
	void *tx_cb_arg;
};

struct sc_can_rx_filters {
	struct can_filter rx_filter;
	can_rx_callback_t rx_cb;
	void *rx_cb_arg;
};

struct sc_can_data {
	struct can_driver_data common;
	/*
	 * These mutex protects:
	 *  - Enabling/Disabling operations for CAN at the same time
	 *      (Including operations that require CAN to stop)
	 *  - Set CAN Timing operations
	 *      (need to set the multiple register)
	 *  - TX operations
	 *      (need to set the multiple register)
	 *  - RX filter list
	 *      (need to protect of multi-threaded list operations)
	 */
	struct k_mutex enadis_mutex;
	struct k_mutex timing_mutex;
	struct k_mutex tx_mutex;
	struct k_mutex rx_mutex;
	struct sc_can_tx_cb_data *tx_cb_data_list;
	uint8_t tx_head;
	uint8_t tx_tail;
	struct sc_can_rx_filters *rx_filters;
	enum can_state state;
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

static inline bool sc_can_filter_is_used(const struct sc_can_cfg *config, int filter_id)
{
	return sys_test_bit(config->reg_addr + SCCAN_AFER_OFFSET, filter_id) != 0;
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

static uint32_t sc_can_get_idr(uint32_t id, bool extended, bool rtr)
{
	uint32_t idr;

	if (extended) {
		idr = (SCCAN_TXID_EX1(id) | SCCAN_TXSRTR(1) | SCCAN_TXIDE(extended) |
		       SCCAN_TXID_EX2(id) | SCCAN_TXERTR(rtr));
	} else {
		idr = (SCCAN_TXID1(id) | SCCAN_TXSRTR(rtr) | SCCAN_TXIDE(extended));
	}

	return idr;
}

static void sc_can_set_idr(const struct sc_can_cfg *config, const struct can_frame *frame)
{
	uint32_t idr;

	idr = sc_can_get_idr(frame->id, (frame->flags & CAN_FRAME_IDE),
			     (frame->flags & CAN_FRAME_RTR));
	sys_write32(idr, config->reg_addr + SCCAN_TMR1_OFFSET);
}

static void sc_can_get_idr_and_dlc(const struct sc_can_cfg *config, struct can_frame *frame)
{
	uint32_t idr;

	idr = sys_read32(config->reg_addr + SCCAN_RMR1_OFFSET);

	if (idr & SCCAN_RX_EXT_FRAME) {
		frame->id = (SCCAN_RXID1_EXT(idr) | SCCAN_RXID2_EXT(idr));
		frame->flags |= CAN_FRAME_IDE;
		if (idr & SCCAN_RX_EXT_REMOTE_FRAME) {
			frame->flags |= CAN_FRAME_RTR;
		}
	} else {
		frame->id = SCCAN_RXID1_STD(idr);
		if (idr & SCCAN_RX_STD_REMOTE_FRAME) {
			frame->flags |= CAN_FRAME_RTR;
		}
	}

	frame->dlc = SCCAN_DLC(sys_read32(config->reg_addr + SCCAN_RMR2_OFFSET));
}

static inline void sc_can_set_dlc(const struct sc_can_cfg *config, uint8_t dlc)
{
	sys_write32(dlc, config->reg_addr + SCCAN_TMR2_OFFSET);
}

static void sc_can_set_data_frame(const struct sc_can_cfg *config, const struct can_frame *frame)
{
	sys_write32(sys_be32_to_cpu(frame->data_32[0]), config->reg_addr + SCCAN_TMR3_OFFSET);
	sys_write32(sys_be32_to_cpu(frame->data_32[1]), config->reg_addr + SCCAN_TMR4_OFFSET);
}

static void sc_can_get_data_frame(const struct sc_can_cfg *config, struct can_frame *frame)
{
	uint32_t data0_reg;
	uint32_t data1_reg;

	/* RMR3/RMR4 must be read to clear the FIFO regardless of the dlc */

	data0_reg = sys_read32(config->reg_addr + SCCAN_RMR3_OFFSET);
	frame->data_32[0] = sys_cpu_to_be32(data0_reg);

	data1_reg = sys_read32(config->reg_addr + SCCAN_RMR4_OFFSET);
	frame->data_32[1] = sys_cpu_to_be32(data1_reg);

	LOG_DBG("Receiving %d bytes. Id: 0x%x, ID type: %s %s", frame->dlc, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");
}

static void sc_can_tx_done(const struct device *dev, int status)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	can_tx_callback_t callback;

	callback = data->tx_cb_data_list[data->tx_tail].tx_cb;
	if (callback == NULL) {
		LOG_ERR("TX callback is not registerd.");
		return;
	}

	callback(dev, status, data->tx_cb_data_list[data->tx_tail].tx_cb_arg);
	data->tx_cb_data_list[data->tx_tail].tx_cb = NULL;

	data->tx_tail++;
	if (data->tx_tail >= config->tx_fifo_depth) {
		data->tx_tail = 0;
	}

	return;
}

static inline void sc_can_enable_acceptance_filter(const struct sc_can_cfg *config, int filter_id)
{
	sys_set_bits(config->reg_addr + SCCAN_AFER_OFFSET, BIT(filter_id));
}

static void sc_can_set_acceptance_filter(const struct sc_can_cfg *config, int filter_id,
					 const struct can_filter *filter)
{
	uint32_t mask_reg = 0;
	uint32_t value_reg = 0;
	bool extended = filter->flags & CAN_FILTER_IDE;
	bool rtr = IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

	if (filter->flags & CAN_FILTER_IDE) {
		mask_reg = (SCCAN_AFID_EX1(filter->mask) | SCCAN_AFSRTR(1) | SCCAN_AFIDE(extended) |
			    SCCAN_AFID_EX2(filter->mask) | SCCAN_AFERTR(rtr));
		value_reg = (SCCAN_AFID_EX1(filter->id) | SCCAN_AFSRTR(1) | SCCAN_AFIDE(extended) |
			     SCCAN_AFID_EX2(filter->id) | SCCAN_AFERTR(rtr));
	} else {
		mask_reg = (SCCAN_TXID1(filter->mask) | SCCAN_AFSRTR(rtr) | SCCAN_AFIDE(extended));
		value_reg = (SCCAN_TXID1(filter->id) | SCCAN_AFSRTR(rtr) | SCCAN_AFIDE(extended));
	}

	sys_write32(mask_reg, config->reg_addr + SCCAN_AFIMR1_OFFSET + (filter_id * 0x20));
	sys_write32(value_reg, config->reg_addr + SCCAN_AFIVR1_OFFSET + (filter_id * 0x20));

	return;
}

/*
 * SC CAN controller has four "Acceptance filter" feature, so this driver use
 * it as the "RX filter".
 * And, due to the specifications of the SC CAN controller, Acceptance Filter
 * can only be set when disable CAN.
 * However, Zephyr specification don't defines the timing for adding/removing
 * RX filter. In fact, CAN sample application (samples/drivers/can/counter) is
 * called can_add_rx_filter_msgq() after can_start().
 * Therefore, in this driver, temporarily disabled CAN and set
 * "Acceptance filer" to SC CAN controller.
 */
static int sc_can_add_acceptance_filter(const struct device *dev, int filter_id,
					can_rx_callback_t cb, void *cb_arg,
					const struct can_filter *filter)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;
	bool need_enable = false;

	ret = k_mutex_lock(&data->enadis_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	if (sc_can_is_enabled(config)) {
		/* The controller must be disabled when updating acceptance filter */
		ret = sc_can_disable(config);
		if (ret != 0) {
			goto unlock;
		}
		need_enable = true;
	}

	sc_can_set_acceptance_filter(config, filter_id, filter);
	sc_can_enable_acceptance_filter(config, filter_id);

	data->rx_filters[filter_id].rx_filter = *filter;
	data->rx_filters[filter_id].rx_cb = cb;
	data->rx_filters[filter_id].rx_cb_arg = cb_arg;

	if (need_enable) {
		ret = sc_can_enable(config);
	}

unlock:
	k_mutex_unlock(&data->enadis_mutex);

nolock:
	return ret;
}

static int sc_can_remove_acceptance_filter(const struct device *dev, int filter_id)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;
	bool need_enable = false;

	ret = k_mutex_lock(&data->enadis_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	if (sc_can_is_enabled(config)) {
		/* The controller must be disabled when updating acceptance filter */
		ret = sc_can_disable(config);
		if (ret != 0) {
			goto unlock;
		}
		need_enable = true;
	}

	sys_clear_bits(config->reg_addr + SCCAN_AFER_OFFSET, BIT(filter_id));
	data->rx_filters[filter_id].rx_cb = NULL;

	if (need_enable) {
		ret = sc_can_enable(config);
	}

unlock:
	k_mutex_unlock(&data->enadis_mutex);

nolock:
	return ret;
}

static void sc_can_rx_cb(const struct device *dev, struct can_frame *frame)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	uint8_t filter_id;

	/* In this driver, Call all RX callback functions that match the registered filter. */
	for (filter_id = 0; filter_id < config->max_filter; filter_id++) {
		if (data->rx_filters[filter_id].rx_cb == NULL) {
			continue;
		}

		if (can_frame_matches_filter(frame, &data->rx_filters[filter_id].rx_filter)) {
			data->rx_filters[filter_id].rx_cb(dev, frame,
							  data->rx_filters[filter_id].rx_cb_arg);
			LOG_DBG("Filter matched. ID: %d", filter_id);
		}
	}

	return;
}

static void sc_can_rx_isr(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct can_frame frame = {0};

	sc_can_get_idr_and_dlc(config, &frame);

	sc_can_get_data_frame(config, &frame);

	sc_can_rx_cb(dev, &frame);

	return;
}

static void sc_can_get_error_count(const struct sc_can_cfg *config, struct can_bus_err_cnt *err_cnt)
{
	uint32_t errcnt_reg;

	errcnt_reg = sys_read32(config->reg_addr + SCCAN_ECNTR_OFFSET);
	err_cnt->tx_err_cnt = SCCAN_TXECNT(errcnt_reg);
	err_cnt->rx_err_cnt = SCCAN_RXECNT(errcnt_reg);
}

static void _sc_can_get_state(const struct device *dev, enum can_state *state,
			      struct can_bus_err_cnt *err_cnt)
{
	const struct sc_can_cfg *config = dev->config;
	uint32_t status_reg;

	status_reg = sc_can_get_status_reg(config);
	switch (SCCAN_ESTS(status_reg)) {
	case SCCAN_ESTS_CAN_DISABLE:
		*state = CAN_STATE_STOPPED;
		break;
	case SCCAN_ESTS_ERROR_ACTIVE:
		if (status_reg & SCCAN_EWRN) {
			*state = CAN_STATE_ERROR_WARNING;
		} else {
			*state = CAN_STATE_ERROR_ACTIVE;
		}
		break;
	case SCCAN_ESTS_ERROR_PASSIVE:
		*state = CAN_STATE_ERROR_PASSIVE;
		break;
	case SCCAN_ESTS_BUS_OFF:
	default:
		*state = CAN_STATE_BUS_OFF;
		break;
	}

	sc_can_get_error_count(config, err_cnt);
}

static int sc_can_get_state(const struct device *dev, enum can_state *state,
			    struct can_bus_err_cnt *err_cnt)
{
	_sc_can_get_state(dev, state, err_cnt);
	return 0;
}

static void sc_can_state_change(const struct device *dev)
{
	struct sc_can_data *data = dev->data;
	const can_state_change_callback_t cb = data->common.state_change_cb;
	void *user_data = data->common.state_change_cb_user_data;
	struct can_bus_err_cnt err_cnt;
	enum can_state new_state;

	_sc_can_get_state(dev, &new_state, &err_cnt);
	if (data->state == new_state) {
		return;
	}

	data->state = new_state;
	LOG_DBG("Can state change new: %u, old: %u", new_state, data->state);

	if (cb == NULL) {
		return;
	}

	cb(dev, new_state, err_cnt, user_data);
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

static bool sc_can_is_tx_fifo_full(const struct sc_can_cfg *config)
{
	uint32_t status_reg;

	status_reg = sys_read32(config->reg_addr + SCCAN_FIFOSR_OFFSET);
	if (status_reg & SCCAN_TXFFL) {
		return true;
	} else {
		return false;
	}
}

static void sc_can_notify_disable_to_tx_cb(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	can_tx_callback_t callback;

	for (int i = 0; i < config->tx_fifo_depth; i++) {
		callback = data->tx_cb_data_list[i].tx_cb;
		if (callback != NULL) {
			callback(dev, -ENETDOWN, data->tx_cb_data_list[i].tx_cb_arg);
			callback = NULL;
		}
	}
}

static int sc_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	if (cap == NULL) {
		return -EINVAL;
	}

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK;

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
	if (ret == 0) {
		/* Notify state change to callback, if enabled */
		sc_can_state_change(dev);
	}

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

		sc_can_notify_disable_to_tx_cb(dev);

		/* Notify state change to state change callback */
		sc_can_state_change(dev);
	}

	k_mutex_unlock(&data->enadis_mutex);

nolock:
	return ret;
}

static void sc_can_isr(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	uint32_t isr;

	isr = sys_read32(config->reg_addr + SCCAN_ISR_OFFSET);
	LOG_DBG("IRQ Status 0x%08x", isr);

	sys_write32(isr, config->reg_addr + SCCAN_ISR_OFFSET);

	if (isr & SCCAN_BUSOFF) {
		sc_can_state_change(dev);
	}
	if (isr & SCCAN_ACKER) {
		/* TODO:
		 * Current SC CAN Controller notify the interrupt every times while
		 * ACK error detected, so this driver received the many interrupt.
		 * However we plan to improve the Interrupt Status Register.
		 * Now, when an ACK error occurs, stop the CAN control.
		 */
		LOG_ERR("ACK error has occurred. Since there is no one on the CAN bus, will stop "
			"the CAN.");
		CAN_STATS_ACK_ERROR_INC(dev);
		sc_can_stop(dev);
	}
	if (isr & SCCAN_BITER) {
		/* SC CAN does not distinguish between BIT0 and 1 errors,
		 * so it counts on BIT0. */
		CAN_STATS_BIT0_ERROR_INC(dev);
		sc_can_tx_done(dev, SCCAN_BITER);
	}
	if (isr & SCCAN_STFER) {
	}
	if (isr & SCCAN_FMER) {
	}
	if (isr & SCCAN_CRCER) {
	}
	if (isr & SCCAN_RXFOVF) {
	}
	if (isr & SCCAN_RXFUDF) {
	}
	if (isr & SCCAN_RXFVAL) {
		sc_can_rx_isr(dev);
	}
	if (isr & SCCAN_RCVDN) {
	}
	if (isr & SCCAN_TXFOVF) {
		sc_can_tx_done(dev, SCCAN_TXFOVF);
	}
	if (isr & SCCAN_TXHBOVF) {
		/* TX High Priority Buffer is not used yet */
	}
	if (isr & SCCAN_ARBLST) {
		sc_can_tx_done(dev, SCCAN_ARBLST);
	}
	if (isr & SCCAN_TRNSDN) {
		sc_can_tx_done(dev, 0);
	}
}

static int sc_can_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct sc_can_cfg *config = dev->config;
	uint32_t mode_reg = 0;

	if ((mode & ~CAN_MODE_LOOPBACK) != 0) {
		LOG_ERR("Unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if (sc_can_is_enabled(config)) {
		return -EBUSY;
	}

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/*
		 * Change to Loop Back mode.
		 * In FPGA IP core specification, it is called
		 * "Self Test mode (STM)".
		 */
		mode_reg = SCCAN_STM_ENABLE;
	} else {
		/* Change to Normal Mode */
		mode_reg = SCCAN_STM_DISABLE;
	}

	sys_write32(mode_reg, config->reg_addr + SCCAN_STMCR_OFFSET);

	LOG_DBG("Set mode:%d", mode);

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
	struct sc_can_data *data = dev->data;

	data->common.state_change_cb = cb;
	data->common.state_change_cb_user_data = user_data;
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
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
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret = 0;

	__ASSERT_NO_MSG(callback != NULL);

	LOG_DBG("Sending %d bytes on %s. Id: 0x%x, ID type: %s %s", frame->dlc, dev->name,
		frame->id, (frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		ret = -EINVAL;
		goto nolock;
	}

	if ((frame->flags & ~(CAN_FRAME_IDE)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		ret = -ENOTSUP;
		goto nolock;
	}

	if (sc_can_is_disabled(config)) {
		ret = -ENETDOWN;
		goto nolock;
	}

	if (sc_can_is_tx_fifo_full(config)) {
		ret = -EAGAIN;
		goto nolock;
	}

	ret = k_mutex_lock(&data->tx_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	/* Save call back function */
	/* TODO:
	 * In the current FPA CAN IP core, CAN Packets stored in the TX Buffer
	 * transmit to the CAN Bus in a FIFO (First In First Out).
	 * Therefore, there is no issue with managing the list of TX callbacks
	 * using an array.
	 * However, due to future modifications in the FPGA CAN IP core, the
	 * transmission order of CAN Packets stored in the TX buffer will be
	 * rearranged based on the CAN ID (priority).
	 * So, we will need to change the management of TX callbacks from an
	 * array to something like an slist.
	 */
	data->tx_cb_data_list[data->tx_head].tx_cb = callback;
	data->tx_cb_data_list[data->tx_head].tx_cb_arg = user_data;
	data->tx_head++;
	if (data->tx_head == config->tx_fifo_depth) {
		data->tx_head = 0;
	}

	sc_can_set_idr(config, frame);

	sc_can_set_dlc(config, frame->dlc);

	sc_can_set_data_frame(config, frame);

	k_mutex_unlock(&data->tx_mutex);

nolock:
	return ret;
}

static int sc_can_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *cb_arg,
				const struct can_filter *filter)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;
	unsigned int filter_id;

	LOG_DBG("Setting filter ID: 0x%x, mask: 0x%x", filter->id, filter->mask);

	ret = k_mutex_lock(&data->rx_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto nolock;
	}

	for (filter_id = 0; filter_id < config->max_filter; filter_id++) {
		if (!sc_can_filter_is_used(config, filter_id)) {
			ret = sc_can_add_acceptance_filter(dev, filter_id, cb, cb_arg, filter);
			if (ret == 0) {
				ret = filter_id;
			}
			break;
		}
	}

	k_mutex_unlock(&data->rx_mutex);

	if (filter_id == config->max_filter) {
		ret = -ENOSPC;
		LOG_ERR("No free filter left");
	} else {
		LOG_DBG("Filter added. ID: %d", filter_id);
	}

nolock:
	return ret;
}

static void sc_can_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct sc_can_cfg *config = dev->config;
	struct sc_can_data *data = dev->data;
	int ret;

	if (filter_id >= config->max_filter) {
		LOG_ERR("Filter ID of %d exceeds maximum (%d)", filter_id, config->max_filter);
		goto nolock;
	}

	ret = k_mutex_lock(&data->rx_mutex, SCCAN_MUTEX_LOCK_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("Failed to remove RX filter because getting mutex lock is timed out.");
		goto nolock;
	}

	ret = sc_can_remove_acceptance_filter(dev, filter_id);
	if (ret < 0) {
		LOG_ERR("Failed to remove RX filter on CAN IP.");
		goto unlock;
	}

	LOG_DBG("Filter removed. ID: %d", filter_id);

unlock:
	k_mutex_unlock(&data->rx_mutex);

nolock:
	return;
}

static int sc_can_init(const struct device *dev)
{
	const struct sc_can_cfg *config = dev->config;
	struct can_timing timing;
	int32_t ret;
	uint32_t v;

	/* Set timing according to dts default setting */
	ret = can_calc_timing(dev, &timing, config->common.bitrate, config->common.sample_point);
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

/*
 * This API specification is here:
 *   Get the maximum standard (11-bit) CAN ID filters if false, or extended (29-bit)
 *   CAN ID filters if true
 * Acceptance filter of SC CAN Controller supports both standard and extended, so
 * always returns the same value.
 */
static int sc_can_get_max_filters(const struct device *dev, bool ide)
{
	const struct sc_can_cfg *config = dev->config;

	ARG_UNUSED(ide);

	return config->max_filter;
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
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = sc_can_recover,
#endif
	.set_state_change_callback = sc_can_set_state_change_callback,
	.get_core_clock = sc_can_get_core_clock,
	.get_max_filters = sc_can_get_max_filters,
	/*
	 * ISO 11898-1:
	 *   Nominal length of time segments (in absence of synchronization)
	 *   - Sync_Seg shall be one time quantum long.
	 *   - The information processing time shall be less than or equal to two time quanta long.
	 *   - Prop_Seg shall be programmable to be 1, 2, 3, . . ., 8 or more time quanta long.
	 *     It shall be programmed to compensate the delay times of the actual network,
	 *     rounded up to the nearest integer time quantum.
	 *   - Phase_Seg1 shall be programmable to be 1, 2, 3, . . ., 8 or more time quanta long.
	 *   - Phase_Seg2 shall be programmed to the maximum of Phase_Seg1 and the information
	 * processing time.
	 *   - SJW shall be programmable between 1 and the minimum of Phase_Seg1 and 4.
	 *
	 *   In a CAN implementation, Prop_Seg and Phase_Seg1 do not need to be programmable
	 * separately; it is sufficient to program the sum of Prop_Seg and Phase_Seg1. The total
	 * number of time quanta in a bit time shall be programmable at least from 8 to 25. In this
	 * driver:
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
	.timing_min = {.sjw = 0x1,
		       .prop_seg = 0x01,
		       .phase_seg1 = 0x01,
		       .phase_seg2 = 0x01,
		       .prescaler = 0x01},
	.timing_max = {.sjw = 0x4,
		       .prop_seg = 0x08,
		       .phase_seg1 = 0x08,
		       .phase_seg2 = 0x08,
		       .prescaler = 0x1000}};

#define SCCAN_INIT(n)                                                                              \
	static void sc_can_##n##_irq_init(const struct device *dev);                               \
	static struct sc_can_tx_cb_data tx_cb_data_list_##n[DT_INST_PROP(n, tx_fifo_depth)];       \
	static struct sc_can_rx_filters rx_filters_##n[DT_INST_PROP(n, max_filter)];               \
	static const struct sc_can_cfg sc_can_cfg_##n = {                                          \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(n, 0, 1000000),                            \
		.reg_addr = DT_INST_REG_ADDR(n),                                                   \
		.irq_init = sc_can_##n##_irq_init,                                                 \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
		.tx_fifo_depth = DT_INST_PROP(n, tx_fifo_depth),                                   \
		.max_filter = DT_INST_PROP(n, max_filter),                                         \
	};                                                                                         \
	static struct sc_can_data sc_can_data_##n = {                                              \
		.enadis_mutex = Z_MUTEX_INITIALIZER(sc_can_data_##n.enadis_mutex),                 \
		.timing_mutex = Z_MUTEX_INITIALIZER(sc_can_data_##n.timing_mutex),                 \
		.tx_cb_data_list = tx_cb_data_list_##n,                                            \
		.tx_head = 0,                                                                      \
		.tx_tail = 0,                                                                      \
		.rx_filters = rx_filters_##n,                                                      \
		.state = CAN_STATE_STOPPED,                                                        \
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
