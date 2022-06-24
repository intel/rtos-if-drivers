/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_ia_can

#include <logging/log.h>
LOG_MODULE_REGISTER(intel_can, LOG_LEVEL_DBG);

#include <drivers/clock_control.h>
#include <sys/util.h>
#include <string.h>
#include <kernel.h>
#include <errno.h>
#include <stdbool.h>
#include "can_intel.h"
#include "intel/hal_can.h"

/* Convenient macro to get the controller instance. */
#define CAN_GET_INSTANCE(dev) \
	(((const struct can_intel_config_t *)dev->config)->instance)


static struct can_bittiming_t can_baudrate_20kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_20_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_20_KBPS,
};

static struct can_bittiming_t can_baudrate_50kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_50_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_50_KBPS,
};

static struct can_bittiming_t can_baudrate_100kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_100_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_100_KBPS,
};

static struct can_bittiming_t can_baudrate_125kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_125_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_125_KBPS,
};

static struct can_bittiming_t can_baudrate_250kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_250_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_250_KBPS,
};

static struct can_bittiming_t can_baudrate_500kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_500_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_500_KBPS,
};

static struct can_bittiming_t can_baudrate_800kbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_800_KBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_800_KBPS,
};

static struct can_bittiming_t can_baudrate_1mbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_1_MBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_1_MBPS,
};

static struct can_bittiming_t can_baudrate_2mbps = {
	.sjw = CAN_BAUD_RATE_SJW,
	.phase_seg1 = CAN_BAUD_RATE_PHASE_SEG1_2_MBPS,
	.phase_seg2 = CAN_BAUD_RATE_PHASE_SEG2_2_MBPS,
};

extern void intel_can_isr(IN intel_instance_t *instance);
static void can_intel_isr(void *arg);
static int can_intel_init(const struct device *dev);
const struct device *can_dev_list[CAN_MAX_INSTANCE];
static struct k_sem g_sem;
static int can_intel_set_mode(const struct device *dev, enum can_mode mode);
static int can_intel_set_timing(const struct device *dev,
				const struct can_timing *timing,
				const struct can_timing *timing_data);
static int can_intel_get_core_clock(const struct device *dev, uint32_t *rate);

static const struct can_driver_api can_intel_api_funcs = {
	.set_mode = can_intel_set_mode,
	.set_timing = can_intel_set_timing,
	.send = can_intel_send,
	.attach_isr = can_intel_attach_isr,
	.detach = can_intel_detach,
	#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_intel_recover,
	#endif
	#ifdef CONFIG_CAN_IOCTL
	.ioctl = can_intel_ioctl,
	#endif
	.get_state = can_intel_get_state,
	.register_state_change_isr = can_intel_register_state_change_isr,
	.get_core_clock = can_intel_get_core_clock,
	.timing_min = { MIN_TIMING_SJW, MIN_TIMING_PROP_SEG,
			MIN_TIMING_PHASE_SEG1, MIN_TIMING_PHASE_SEG2,
			MIN_TIMING_PRESCALAR },
	.timing_max = { MAX_TIMING_SJW, MAX_TIMING_PROP_SEG,
			MAX_TIMING_PHASE_SEG1, MAX_TIMING_PHASE_SEG2,
			MAX_TIMING_PRESCALAR },
#ifdef CONFIG_CAN_FD_MODE
	.timing_min_data = { MIN_TIMING_DATA_SJW, MIN_TIMING_DATA_PROP_SEG,
			     MIN_TIMING_DATA_PHASE_SEG1,
			     MIN_TIMING_DATA_PHASE_SEG2,
			     MIN_TIMING_DATA_PRESCALAR },
	.timing_max_data = { MAX_TIMING_DATA_SJW, MAX_TIMING_DATA_PROP_SEG,
			     MAX_TIMING_DATA_PHASE_SEG1,
			     MAX_TIMING_DATA_PHASE_SEG2,
			     MAX_TIMING_DATA_PRESCALAR }
#endif

};


static int can_intel_set_mode(const struct device *dev, enum can_mode mode)
{
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	int ret = -EIO;
	enum can_op_mode intel_can_mode = CAN_MODE_NONE;

	switch (mode) {
	case CAN_NORMAL_MODE:
		intel_can_mode = CAN_MODE_NORMAL;
#ifdef CONFIG_CAN_FD_MODE
		intel_can_mode = CAN_MODE_FDBS;
#endif
		break;
	case CAN_SILENT_MODE:
		intel_can_mode = CAN_MODE_MONITOR;
		break;
	case CAN_LOOPBACK_MODE:
		intel_can_mode = CAN_MODE_LOOPBACK_EXTERNAL;
		break;
	case CAN_SILENT_LOOPBACK_MODE:
		intel_can_mode = CAN_MODE_LOOPBACK_INTERNAL;
		break;
#ifdef CONFIG_CAN_FD_MODE
	case CAN_FD_MODE:
		intel_can_mode = CAN_MODE_FD;
		break;
	case CAN_FD_BPRS_MODE:
		intel_can_mode = CAN_MODE_FDBS;
		break;
#endif
	default:
		goto err;
	}
	ret = intel_can_set_mode(instance, intel_can_mode);
	if (ret < 0) {
		ret = -EIO;
		goto err;
	}

err:
	return ret;
}


static int can_intel_set_timing(const struct device *dev,
				const struct can_timing *timing,
				const struct can_timing *timing_data)
{
	struct  can_bittiming_t arbit_timing = { 0 };
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	int ret = 0;

	if (timing) {
		arbit_timing.sjw = timing->sjw - 1;
		arbit_timing.phase_seg1 = timing->phase_seg1 +
					  timing->prop_seg - 1;
		arbit_timing.phase_seg2 = timing->phase_seg2 - 1;
		arbit_timing.brp = timing->prescaler - 1;
		intel_can_set_bitrate(instance, &arbit_timing);
	} else {
		ret = -EINVAL;
	}

#ifdef CONFIG_CAN_FD_MODE
	struct  can_fast_bittiming_t data_timing = { 0 };

	if (timing_data) {
		data_timing.fast_sjw = timing_data->sjw - 1;
		data_timing.fast_phase_seg1 = timing_data->phase_seg1 +
					      timing_data->prop_seg - 1;
		data_timing.fast_phase_seg2 = timing_data->phase_seg2 - 1;
		data_timing.fast_brp = timing_data->prescaler - 1;
		intel_can_set_fast_bitrate(instance, &data_timing);
	} else {
		ret = -EINVAL;
	}
#endif
	return ret;
}

static int can_intel_get_core_clock(const struct device *dev, uint32_t *rate)
{
	if (!rate) {
		return -EIO;
	}
	*rate = (uint32_t)200000000;
	return 0;
}

#ifdef CONFIG_CAN_DISABLE_AUTO_RETRANSMIT
#define DISABLE_AUTO_RETRANS (1)
#else
#define DISABLE_AUTO_RETRANS (0)
#endif

/*  CAN IRQ handler declaration.  */
#define  CAN_IRQ_HANDLER_DECL(n) \
	static void can_intel_config_irq_##n(const struct device *dev)

#define SET_IRQ_FLAG(n) (((DT_INST_IRQ_HAS_CELL(n, sense)) == 0) ? \
			 (DT_INST_IRQ(n, sense)) : 0)

#define CAN_IRQ_HANDLER_DEFINE(n)								  \
	static void can_intel_config_irq_##n(const struct device *dev)				  \
	{											  \
		ARG_UNUSED(dev);								  \
		if (DT_INST_ON_BUS(n, pcie)) {							  \
			if (DT_INST_IRQN(n) == PCIE_IRQ_DETECT) {				  \
				BUILD_ASSERT(IS_ENABLED(CONFIG_DYNAMIC_INTERRUPTS),		  \
					     "pse PCI auto-IRQ needs CONFIG_DYNAMIC_INTERRUPTS"); \
				unsigned int irq;						  \
				irq = pcie_alloc_irq(DT_INST_REG_ADDR(n));			  \
				if (irq == PCIE_CONF_INTR_IRQ_NONE) {				  \
					return;							  \
				}								  \
				irq_connect_dynamic(irq,					  \
						    DT_INST_IRQ(n, priority),			  \
						    (void (*)(const void *))can_intel_isr,	  \
						    DEVICE_DT_INST_GET(n),			  \
						    SET_IRQ_FLAG(n));				  \
				pcie_irq_enable(DT_INST_REG_ADDR(n), irq);			  \
			}									  \
		}										  \
	}

/* Setting configuration and init function. */
#define CAN_INTEL_DEVICE_INIT(n)						  \
	static struct can_params_t can_intel_params_##n = {			  \
		.bit_timing.phase_seg1 = DT_INST_PROP(n, phase_seg1),		  \
		.bit_timing.phase_seg2 = DT_INST_PROP(n, phase_seg2),		  \
		.bit_timing.brp = CONFIG_CAN_BRP,				  \
		.bit_timing.sjw = DT_INST_PROP(n, sjw),				  \
		.fast_bit_timing.fast_phase_seg1 = DT_INST_PROP(n,		  \
								phase_seg1_data), \
		.fast_bit_timing.fast_phase_seg2 = DT_INST_PROP(n,		  \
								phase_seg2_data), \
		.fast_bit_timing.fast_brp = CONFIG_CAN_FAST_BRP,		  \
		.fast_bit_timing.fast_sjw = DT_INST_PROP(n, sjw_data),		  \
		.std_filts_cnt = CONFIG_CAN##n##_STD_FILTER_COUNT,		  \
		.ext_filts_cnt = CONFIG_CAN##n##_EXT_FILTER_COUNT,		  \
		.rx_fifo0_cnt = CONFIG_CAN##n##_RX_FIFO0_COUNT,			  \
		.rx_fifo1_cnt = CONFIG_CAN##n##_RX_FIFO1_COUNT,			  \
		.rx_buf_cnt = CONFIG_CAN##n##_RX_BUF_COUNT,			  \
		.tx_evt_fifo_cnt = CONFIG_CAN##n##_TX_EVENT_FIFO_COUNT,		  \
		.tx_buf_cnt = CONFIG_CAN##n##_TX_BUF_COUNT,			  \
		.tx_fifo_cnt = CONFIG_CAN##n##_TX_FIFO_COUNT,			  \
		.rx_fifo0_word_size = CONFIG_CAN##n##_RX_FIFO0_SIZE,		  \
		.rx_fifo1_word_size = CONFIG_CAN##n##_RX_FIFO1_SIZE,		  \
		.rx_buf_word_size = CONFIG_CAN##n##_RX_BUF_SIZE,		  \
		.tx_buf_word_size = CONFIG_CAN##n##_TX_BUF_SIZE,		  \
		.rx_fifo0_wm = CONFIG_CAN##n##_RX_FIFO0_WM,			  \
		.rx_fifo1_wm = CONFIG_CAN##n##_RX_FIFO1_WM,			  \
		.tx_evt_fifo_wm = CONFIG_CAN##n##_TX_EVT_FIFO_WM,		  \
		.gfc_reject = CONFIG_CAN##n##_GFC_REJECT,			  \
		.gfc_remote_reject = CONFIG_CAN##n##_GFC_REMOTE_REJECT,		  \
		.time_counter = CONFIG_CAN##n##_TIME_COUNT,			  \
		.rx_fifo_mode = CAN_FIFO_OVERWRITE,				  \
		.disable_auto_retransmit = DISABLE_AUTO_RETRANS			  \
	};									  \
	CAN_IRQ_HANDLER_DECL(n);						  \
	static struct can_intel_config_t config_info_##n = {			  \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				  \
		.instance = INTEL_CAN_INSTANCE(n),				  \
		.params = &can_intel_params_##n,				  \
		.config_irq = &can_intel_config_irq_##n,			  \
		.pcie = true,							  \
		.pcie_id = DT_INST_REG_SIZE(n),					  \
	};									  \
	static struct can_intel_data_t can_intel_drv_data_##n = {		  \
		.id = CAN_##n,							  \
	};									  \
	DEVICE_DT_INST_DEFINE(n,						  \
			      &can_intel_init,					  \
			      NULL,						  \
			      &can_intel_drv_data_##n,				  \
			      &config_info_##n, POST_KERNEL,			  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		  \
			      &can_intel_api_funcs);				  \
	int CAN##n##_MAX_FILTER = CONFIG_CAN##n##_STD_FILTER_COUNT		  \
				  + CONFIG_CAN##n##_EXT_FILTER_COUNT;		  \
	static struct filter_data_t						  \
		can##n##_filter_list[CONFIG_CAN##n##_STD_FILTER_COUNT		  \
				     + CONFIG_CAN##n##_EXT_FILTER_COUNT];	  \
	static struct can_mailbox mb##n;					  \
	CAN_IRQ_HANDLER_DEFINE(n)

DT_INST_FOREACH_STATUS_OKAY(CAN_INTEL_DEVICE_INIT)

int32_t register_parity_callback(const struct device *dev,
				 can_parity_callback_t cb)
{
	int32_t ret = INTEL_DRIVER_OK;
	struct can_intel_data_t *data = DEV_DATA(dev);

	__ASSERT(cb != NULL, "callback is null");
	data->parity_cb = cb;
	return ret;
}


static void handle_rx_event(enum can_id id, uint32_t val, void *data)
{
	uint32_t state;
	struct zcan_frame msg;
	struct can_frame_t *frame = (struct can_frame_t *)data;
	uint32_t filter_index = frame->info.filter_index;

	if (val == CAN_STD_MSG_RECEIVE) {
		msg.id_type = CAN_STANDARD_IDENTIFIER;
		msg.id = frame->info.id;
	} else {
		msg.id_type = CAN_EXTENDED_IDENTIFIER;
		msg.id = frame->info.id;
		filter_index = (id == CAN_0) ?
			       (CONFIG_CAN0_STD_FILTER_COUNT + filter_index)
			       : (CONFIG_CAN1_STD_FILTER_COUNT + filter_index);
	}
	msg.can_id = id;
	msg.rtr = ((frame->info.flags & CAN_BUFFER_RTR_FLAG) >> 2);
	if (id == CAN_0) {
		if (msg.rtr != can0_filter_list[filter_index].rtr) {
			return;
		}
	} else {
		if (msg.rtr != can1_filter_list[filter_index].rtr) {
			return;
		}
	}
	msg.dlc = frame->info.length;
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	msg.timestamp = frame->info.timestamp;
#endif
	uint8_t *src = (uint8_t *)&(frame->data);
	uint8_t *dst = (uint8_t *)&msg.data_32;

	if (!msg.rtr) {
		for (int i = 0; i < frame->info.length; i++) {
			*dst++ = *src++;
		}
	}
	if (id == CAN_0) {
		state = irq_lock();
		if (can0_filter_list[filter_index].cb) {
			can0_filter_list[filter_index].cb(&msg,
					can0_filter_list[filter_index].cb_arg);
		}

		irq_unlock(state);
	} else {
		state = irq_lock();
		if (can1_filter_list[filter_index].cb) {
			can1_filter_list[filter_index].cb(&msg,
					can1_filter_list[filter_index].cb_arg);
		}

		irq_unlock(state);
	}

}
static void handle_tx_event(IN intel_instance_t *inst, enum can_id id, uint32_t val, void *data)
{
	int32_t err = CAN_TX_OK;
	struct can_mailbox *mail_box = NULL;
	struct can_intel_data_t *drv_data = DEV_DATA(can_dev_list[id]);

	struct can_bus_err_cnt err_cnt = { 0, 0 };
	struct can_err_cnt intel_err_cnt = { 0, 0 };
	enum can_state can_state;

	if (id == CAN_0) {
		mail_box = &mb0;
	} else {
		mail_box = &mb1;
	}

	if (val == CAN_TX_OCCURRED) {
		k_sem_take(&(mail_box->mailbox_sem), K_NO_WAIT); /* TODO */
		if ((mail_box != NULL) && (mail_box->tx_callback != NULL)) {
			mail_box->tx_callback(CAN_TX_OK, mail_box->cb_arg);
		} else {
			drv_data->tx_err = CAN_TX_OK;
			k_sem_give(&drv_data->sync_tx_sem);
		}
		k_sem_give(&(mail_box->mailbox_sem));
	} else {
		switch (val) {
		case CAN_ERR_ACCESS_TO_RESERVED:
		case CAN_ERR_PROT_DATA:
		case CAN_ERR_LOG_OVERFLOW:
		case CAN_ERR_MSG_RAM_FAILURE:
		case CAN_ERR_ACK:
			err = CAN_TX_ERR;
			can_state = CAN_ERROR_ACTIVE;
			break;
		case CAN_ERR_PASSIVE:
			err = CAN_TX_ERR_PASSIVE;
			can_state = CAN_ERROR_PASSIVE;
			break;
		case CAN_ERR_WARNING:
			err = CAN_TX_ERR_WARNING;
			can_state = CAN_ERROR_ACTIVE;
			break;
		case CAN_ERR_PROT_ARBITRATION:
			err = CAN_TX_ARB_LOST;
			can_state = CAN_ERROR_ACTIVE;
			break;
		case CAN_ERR_BUS_OFF:
			err = CAN_TX_BUS_OFF;
			can_state = CAN_BUS_OFF;
			break;
		default:
			err = CAN_TX_UNKNOWN;
			can_state = CAN_BUS_UNKNOWN;
			break;
		}

		if (drv_data->state_cb) {
			intel_can_get_state_err_cnt(inst, &intel_err_cnt);
			err_cnt.tx_err_cnt = intel_err_cnt.tx_err_cnt;
			err_cnt.rx_err_cnt = intel_err_cnt.rx_err_cnt;
			drv_data->state_cb(can_state, err_cnt);
		}
		k_sem_take(&(mail_box->mailbox_sem), K_NO_WAIT);
		if ((mail_box != NULL) && mail_box->tx_callback) {
			mail_box->tx_callback(err, mail_box->cb_arg);
		} else {
			drv_data->tx_err = err;
			k_sem_give(&drv_data->sync_tx_sem);
		}
		k_sem_give(&(mail_box->mailbox_sem));
	}
}

static void handle_parity_event(enum can_id id, uint32_t val, void *data)
{
	uint32_t state;

	state = irq_lock();
	if ((id == CAN_0) && can_intel_drv_data_0.parity_cb) {
		can_intel_drv_data_0.parity_cb(val);
	} else if ((id == CAN_1) && can_intel_drv_data_1.parity_cb) {
		can_intel_drv_data_1.parity_cb(val);
	}
	irq_unlock(state);
}

static void intel_can_callback(IN intel_instance_t *inst, enum can_id id,
		uint32_t val, void *data)
{
	if (((val == CAN_STD_MSG_RECEIVE) || (val == CAN_EXT_MSG_RECEIVE))
	    && (data != NULL)) {
		handle_rx_event(id, val, data);
	} else if ((val == CAN_PARITY_ERR_OCCURRED_UPPER) ||
		   (val == CAN_PARITY_ERR_OCCURRED_LOWER)) {
		handle_parity_event(id, val, data);
	} else {
		handle_tx_event(inst, id, val, data);
	}
}

static void intel_can_fill_baudrate_data(void)
{
	uint32_t hbw_clock = 200000000;

	can_baudrate_20kbps.brp = CAN_BAUD_RATE_PHASE_BRP_20_KBPS(hbw_clock);
	can_baudrate_50kbps.brp = CAN_BAUD_RATE_PHASE_BRP_50_KBPS(hbw_clock);
	can_baudrate_100kbps.brp = CAN_BAUD_RATE_PHASE_BRP_100_KBPS(hbw_clock);
	can_baudrate_125kbps.brp = CAN_BAUD_RATE_PHASE_BRP_125_KBPS(hbw_clock);
	can_baudrate_250kbps.brp = CAN_BAUD_RATE_PHASE_BRP_250_KBPS(hbw_clock);
	can_baudrate_500kbps.brp = CAN_BAUD_RATE_PHASE_BRP_500_KBPS(hbw_clock);
	can_baudrate_800kbps.brp = CAN_BAUD_RATE_PHASE_BRP_800_KBPS(hbw_clock);
	can_baudrate_1mbps.brp = CAN_BAUD_RATE_PHASE_BRP_1_MBPS(hbw_clock);
	can_baudrate_2mbps.brp = CAN_BAUD_RATE_PHASE_BRP_2_MBPS(hbw_clock);
}

static int can_intel_init(const struct device *dev)
{
	struct can_intel_config_t *cfg = DEV_CFG(dev);
	struct can_intel_data_t *drv_data = DEV_DATA(dev);
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	int ret = 0, i = 0;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (cfg->pcie) {
		struct pcie_mbar mbar;

		drv_data->pcie_bdf = pcie_bdf_lookup(cfg->pcie_id);
		if (!pcie_probe(drv_data->pcie_bdf, cfg->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(drv_data->pcie_bdf, 0, &mbar);
		pcie_set_cmd(drv_data->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
		pcie_set_cmd(drv_data->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);
		intel_set_base_addr(instance, DEVICE_MMIO_GET(dev));
		intel_set_base_addr(instance, DEVICE_MMIO_GET(dev));
		intel_set_phy_addr(instance, mbar.phys_addr);
	} else
#endif
	{
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}
	intel_can_fill_baudrate_data();
	k_sem_init(&drv_data->tx_sem, 1, 1);
	k_sem_init(&drv_data->set_filter_sem, 1, 1);
	k_sem_init(&drv_data->sync_tx_sem, 0, 1);
	k_sem_init(&g_sem, 1, 1);
	k_sem_take(&g_sem, K_NO_WAIT);
	can_dev_list[drv_data->id] = dev;
	if (drv_data->id == CAN_0) {
		k_sem_init(&(mb0.mailbox_sem), 1, 1);
		for (i = 0; i < CAN0_MAX_FILTER; i++) {
			memset(&can0_filter_list[i], 0,
			       sizeof(struct filter_data_t));
		}
	} else {
		k_sem_init(&(mb1.mailbox_sem), 1, 1);
		for (i = 0; i < CAN1_MAX_FILTER; i++) {
			memset(&can1_filter_list[i], 0,
			       sizeof(struct filter_data_t));
		}
	}
	k_sem_give(&g_sem);
	ret = intel_can_init(instance, drv_data->id, (can_callback_t)&intel_can_callback,
			     cfg->params);
	if (ret) {
		goto err;
	}
	cfg->config_irq(dev);

	/* Set to normal operation mode */
	ret = intel_can_set_mode(instance, CAN_MODE_NORMAL);
err:
	return ret;
}

int can_intel_send(const struct device *dev, const struct zcan_frame *msg,
		   k_timeout_t timeout, can_tx_callback_t callback,
		   void *callback_arg)
{
	struct can_intel_data_t *drv_data = DEV_DATA(dev);
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	struct k_sem *tx_sem = &(drv_data->tx_sem);
	struct k_sem *sync_tx_sem = &(drv_data->sync_tx_sem);
	struct can_frame_t frame;
	uint8_t *src = NULL;
	int ret = CAN_TX_OK, i = 0;

	__ASSERT((msg != 0), "Msg is NULL");
	__ASSERT((msg->dlc != 0), "Dlc is zero");
	__ASSERT((msg->data != NULL), "Dataptr is null");

	if (intel_can_get_status(instance) == CAN_STATE_BUS_OFF) {
		return CAN_TX_BUS_OFF;
	}

	k_sem_take(tx_sem, K_FOREVER);
	if (drv_data->id == CAN_0) {
		mb0.tx_callback = callback;
		mb0.cb_arg = callback_arg;
	}
	if (drv_data->id == CAN_1) {
		mb1.tx_callback = callback;
		mb1.cb_arg = callback_arg;
	}
	if (msg->id_type == CAN_STANDARD_IDENTIFIER) {
		frame.info.id = msg->id;
	} else if (msg->id_type == CAN_EXTENDED_IDENTIFIER) {
		frame.info.id = msg->id;
	}
	frame.info.buff_num = CAN_INVALID_BUFNUM;
	frame.info.flags = (msg->rtr ? CAN_BUFFER_RTR_FLAG : 0);

	frame.info.isfifo = 1;
	frame.info.length = msg->dlc;
	frame.info.mm = 0;
	src = (uint8_t *)&(msg->data_32);
	for (i = 0; i < frame.info.length; i++) {
		frame.data[i] = *src++;
	}

	if (!callback) {
		k_sem_reset(sync_tx_sem);
	}


	ret = intel_can_send_message(instance, msg->id_type,
				     (void *)&frame);
	if (ret < 0) {
		ret = CAN_TX_ERR;
		goto err;
	}

	if (!callback) {
		k_sem_take(sync_tx_sem, K_FOREVER);
		ret = drv_data->tx_err;

	}
err:
	k_sem_give(tx_sem);
	if (ret != CAN_TX_OK) {
		if ((drv_data->id == CAN_0) && (mb0.tx_callback != NULL)) {
			mb0.tx_callback = NULL;
		} else if ((drv_data->id == CAN_1)
			   && (mb1.tx_callback != NULL)) {
			mb1.tx_callback = NULL;
		}
	}
	return ret;
}

static int can_intel_get_free_filter_index(enum can_id id,
					   struct filter_data_t *arr,
					   enum can_msg_id msg_id)
{
	int i = 0, filter_nr = CAN_NO_FREE_FILTER;
	int max_filters = (id == CAN_0) ? CAN0_MAX_FILTER : CAN1_MAX_FILTER;

	if (msg_id ==  CAN_STD_ID) {
		i = 0;
	} else {
		i = (id == CAN_0) ? CONFIG_CAN0_STD_FILTER_COUNT
		    : CONFIG_CAN1_STD_FILTER_COUNT;
	}
	if (!arr) {
		goto err;
	}
	while (i < max_filters) {
		if (!(arr[i].configured)) {
			break;
		}
		i++;
	}
	if (i != max_filters) {
		arr[i].configured = 1;
		filter_nr = i;
	} else {
		filter_nr = CAN_NO_FREE_FILTER;
		goto err;
	}

	if (msg_id == CAN_EXT_ID) {
		filter_nr = (id == CAN_0) ?
			    (filter_nr - CONFIG_CAN0_STD_FILTER_COUNT)
			    : (filter_nr - CONFIG_CAN1_STD_FILTER_COUNT);
	}
err:
	return filter_nr;
}


static int can_intel_check_avilable_filter(const struct device *dev,
					   enum can_msg_id msg_id)
{
	int filter_nr = CAN_NO_FREE_FILTER;
	struct can_intel_data_t *drv_data = DEV_DATA(dev);

	k_sem_take(&g_sem, K_FOREVER);
	if (drv_data->id == CAN_0) {
		filter_nr = can_intel_get_free_filter_index(drv_data->id,
				(struct filter_data_t *)&can0_filter_list,
				msg_id);
	} else {
		filter_nr = can_intel_get_free_filter_index(drv_data->id,
				(struct filter_data_t *)&can1_filter_list,
				msg_id);
	}

	k_sem_give(&g_sem);
	return filter_nr;
}

int can_intel_attach_isr(const struct device *dev, can_rx_callback_t isr,
			 void *callback_arg, const struct zcan_filter *filter)
{
	struct can_intel_data_t *drv_data = DEV_DATA(dev);
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	struct can_filter_t filt;
	int filter_nr = CAN_NO_FREE_FILTER, num, filter_array_index;
	enum can_msg_id msg_id;

	k_sem_take(&(drv_data->set_filter_sem), K_FOREVER);
	msg_id = filter->id_type;
	num = can_intel_check_avilable_filter(dev, msg_id);
	if (num ==  CAN_NO_FREE_FILTER) {
		goto err;
	}

	filt.id1 = filter->id;
	filt.id2 = filter->id_mask;
	if (filter->id_type == CAN_STANDARD_IDENTIFIER) {
		filter_array_index = num;
	} else {
		filter_array_index = (drv_data->id == CAN_0) ?
				     (num + CONFIG_CAN0_STD_FILTER_COUNT)
				     : (num + CONFIG_CAN1_STD_FILTER_COUNT);
	}
	if ((drv_data->id == CAN_0) &&
	    (filter_array_index >= CAN0_MAX_FILTER)) {
		filter_nr = CAN_NO_FREE_FILTER;
		goto err;
	}
	if ((drv_data->id == CAN_1) &&
	    (filter_array_index >= CAN1_MAX_FILTER)) {
		filter_nr = CAN_NO_FREE_FILTER;
		goto err;
	}

	filt.buf_num = CAN_INVALID_BUFNUM;
	filt.fifo_id = CAN_FIFO_0;
	filt.filter_type = CAN_FILTER_CLASSIC;
	filt.op = CAN_FILTER_OP_FIFO0;
	filt.filter_num = num;
	filter_nr = num;
	k_sem_take(&g_sem, K_FOREVER);
	if (intel_can_config_filter(instance, &filt, msg_id) < 0) {
		if (drv_data->id == CAN_0) {
			can0_filter_list[filter_array_index].configured = 0;
		} else if (drv_data->id == CAN_1) {
			can1_filter_list[filter_array_index].configured = 0;
		}
		filter_nr = CAN_NO_FREE_FILTER;
		k_sem_give(&g_sem);
		goto err;
	}
	drv_data->attached_filter_count++;
	if (drv_data->id == CAN_0) {
		can0_filter_list[filter_array_index].cb = isr;
		can0_filter_list[filter_array_index].cb_arg =
			callback_arg;
		can0_filter_list[filter_array_index].filter_id_type =
			((msg_id ==  CAN_STD_ID) ?
			 CAN_STANDARD_IDENTIFIER :
			 CAN_EXTENDED_IDENTIFIER);
		can0_filter_list[filter_array_index].id1 = filt.id1;
		can0_filter_list[filter_array_index].id2 = filt.id2;
		can0_filter_list[filter_array_index].rtr = filter->rtr;
	} else {
		can1_filter_list[filter_array_index].cb = isr;
		can1_filter_list[filter_array_index].cb_arg =
			callback_arg;
		can1_filter_list[filter_array_index].filter_id_type =
			((msg_id ==  CAN_STD_ID) ?
			 CAN_STANDARD_IDENTIFIER :
			 CAN_EXTENDED_IDENTIFIER);
		can1_filter_list[filter_array_index].id1 = filt.id1;
		can1_filter_list[filter_array_index].id2 = filt.id2;
		can1_filter_list[filter_array_index].rtr = filter->rtr;
	}
	k_sem_give(&g_sem);
err:
	k_sem_give(&(drv_data->set_filter_sem));
	return filter_nr;
}

void can_intel_detach(const struct device *dev, int filter_nr)
{

	struct can_intel_data_t *drv_data = DEV_DATA(dev);
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	struct can_filter_t filter;


	k_sem_take(&(drv_data->set_filter_sem), K_FOREVER);
	filter.op = CAN_FILTER_OP_DISABLE;
	filter.buf_num = CAN_INVALID_BUFNUM;
	filter.filter_num = filter_nr;
	k_sem_take(&g_sem, K_FOREVER);
	if (drv_data->id == CAN_0) {
		can0_filter_list[filter_nr].configured = 0;
		can0_filter_list[filter_nr].cb = NULL;
		intel_can_config_filter(instance, &filter,
				can0_filter_list[filter_nr].filter_id_type);
	} else {
		can1_filter_list[filter_nr].configured = 0;
		can1_filter_list[filter_nr].cb = NULL;
		intel_can_config_filter(instance, &filter,
				can1_filter_list[filter_nr].filter_id_type);
	}

	drv_data->attached_filter_count--;
	if (drv_data->attached_filter_count <= 0) {
		drv_data->attached_filter_count = 0;
	}
	k_sem_give(&g_sem);
	k_sem_give(&(drv_data->set_filter_sem));
}

int can_intel_recover(const struct device *dev, k_timeout_t timeout)
{
	int32_t ret, ticks = timeout.ticks;
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);

	if (ticks == 0) {
		ret = intel_can_recover(instance);
		if (ret == INTEL_DRIVER_OK) {
			return ret;
		}
	} else {
		while (ticks != 0) {
			ret = intel_can_recover(instance);
			if (ret == INTEL_DRIVER_OK) {
				return ret;
			}
			k_sleep(K_TICKS(1));
			ticks--;
		}
	}
	return CAN_TIMEOUT;
}

enum can_state can_intel_get_state(const struct device *dev,
				   struct can_bus_err_cnt *err_cnt)
{
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	enum can_states state = intel_can_get_state_err_cnt(instance,
			(struct can_err_cnt *)err_cnt);

	if (state == CAN_STATE_ERR_ACTIVE) {
		return CAN_ERROR_ACTIVE;
	} else if (state == CAN_STATE_ERR_PASSIVE) {
		return CAN_ERROR_PASSIVE;
	} else if (state == CAN_STATE_BUS_OFF) {
		return CAN_BUS_OFF;
	} else {
		return CAN_BUS_UNKNOWN;
	}
}

void can_intel_register_state_change_isr(const struct device *dev,
					 can_state_change_isr_t isr)
{
	struct can_intel_data_t *can_dev = DEV_DATA(dev);

	can_dev->state_cb = isr;
}

int can_intel_ioctl(const struct device *dev, uint32_t can_cmd, void *data)
{
	int ret = 0;
#if defined(CONFIG_CAN_FD_MODE)
	uint32_t fd_val;
#endif
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);
	struct can_parity_err_t *err = NULL;

	switch (can_cmd) {
	case CAN_IOCTL_PARITY_INJECTION_ENABLE:
		err = (struct can_parity_err_t *)data;
		ret = intel_can_inject_parity_err(instance,
						  err, 1);
		break;
	case CAN_IOCTL_PARITY_INJECTION_DISABLE:
		err = (struct can_parity_err_t *)data;
		ret = intel_can_inject_parity_err(instance,
						  err, 0);
		break;
	case CAN_IOCTL_POWER_OFF:
		intel_can_power_control(instance, 0);
		break;
	case CAN_IOCTL_POWER_ON:
		intel_can_power_control(instance, 1);
		break;
#if defined(CONFIG_CAN_FD_MODE)
	case CAN_IOCTL_FD_MODE:
		fd_val =  *((uint32_t *)(data));
		if ((!!fd_val) == true) {
			ret = intel_can_set_mode(instance, CAN_MODE_FD);
		} else {
			ret = intel_can_set_mode(instance, CAN_MODE_NORMAL);
		}
		break;
#endif
	default:
		ret = -1;
	}
	return ret;
}

static int can_intel_match_filter(struct zcan_frame *msg,
				  struct filter_data_t *arr)
{
	int i = 0;
	int res = 0;
	uint32_t mask1 = 0, mask2 = 0, max_filter = 0;

	if ((!arr) || (!msg)) {
		goto err;
	}
	max_filter = (msg->can_id == CAN_0) ? CAN0_MAX_FILTER : CAN1_MAX_FILTER;
	for (i = 0; i < max_filter; i++) {
		mask1 = arr[i].id1
			| arr[i].id2;
		if (msg->id_type == CAN_STANDARD_IDENTIFIER) {
			mask2 = msg->id | arr[i].id2;
			if (mask1 == mask2) {
				res = 1;
				break;
			}
		} else {
			mask2 = msg->id | arr[i].id2;
			if (mask1 == mask2) {
				res = 1;
				break;
			}
		}
	}
err:
	return res;
}
int can_intel_check_filter_matched(struct zcan_frame *msg)
{
	int res = 0;

	if (!msg) {
		goto exit;
	}
	if (msg->can_id == CAN_0) {
		res = can_intel_match_filter(msg,
				(struct filter_data_t *) &can0_filter_list);

	} else if (msg->can_id == CAN_1) {
		res = can_intel_match_filter(msg,
				(struct filter_data_t *) &can1_filter_list);
	}
exit:
	return res;
}

static void can_intel_isr(void *arg)
{
	const struct device *dev = arg;
	intel_instance_t *instance = CAN_GET_INSTANCE(dev);

	intel_can_isr(instance);
}


#ifdef CONFIG_NET_SOCKETS_CAN
#include "../drivers/can/socket_can_generic.h"

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
#define SOCKET_CAN_DEV_NAME_0 "SOCKET_CAN_0"

CAN_DEFINE_MSGQ(socket_can_msgq_0, 5);
K_THREAD_STACK_DEFINE(rx_thread_stack_0, RX_THREAD_STACK_SIZE);

static int socket_can_init_0(const struct device *dev)
{
	const struct device *can_dev = DEVICE_DT_INST_GET(0);

	if (!can_dev) {
		LOG_ERR("Can dev0 is null\n");
		return -1;
	}
	struct socket_can_context *socket_context = dev->data;

	LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)",
		dev, dev->name, can_dev, can_dev->name);

	socket_context->can_dev = can_dev;
	socket_context->msgq = &socket_can_msgq_0;
	socket_context->tx_cb = tx_irq_callback_0;

	socket_context->rx_tid =
		k_thread_create(&socket_context->rx_thread_data,
				rx_thread_stack_0,
				K_THREAD_STACK_SIZEOF(rx_thread_stack_0),
				rx_thread, socket_context, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

NET_DEVICE_INIT(can_0, SOCKET_CAN_DEV_NAME_0, socket_can_init_0,
		device_pm_control_nop, &socket_can_context_0, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		&socket_can_api,
		CANBUS_RAW_L2, NET_L2_GET_CTX_TYPE(CANBUS_RAW_L2), CAN_MTU);
#endif

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)
#define SOCKET_CAN_DEV_NAME_1 "SOCKET_CAN_1"

CAN_DEFINE_MSGQ(socket_can_msgq_1, 5);
K_THREAD_STACK_DEFINE(rx_thread_stack_1, RX_THREAD_STACK_SIZE);
static struct socket_can_context socket_can_context_1;

static int socket_can_init_1(const struct device *dev)
{
	const struct device *can_dev = DEVICE_DT_INST_GET(1);

	if (!can_dev) {
		LOG_ERR("Can dev1 is null\n");
	}
	struct socket_can_context *socket_context = dev->data;

	LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)",
		dev, dev->name, can_dev, can_dev->name);

	socket_context->can_dev = can_dev;
	socket_context->msgq = &socket_can_msgq_1;

	socket_context->tx_cb = tx_irq_callback_1;
	socket_context->rx_tid =
		k_thread_create(&socket_context->rx_thread_data,
				rx_thread_stack_1,
				K_THREAD_STACK_SIZEOF(rx_thread_stack_1),
				rx_thread, socket_context, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

NET_DEVICE_INIT(can_1, SOCKET_CAN_DEV_NAME_1, socket_can_init_1,
		device_pm_control_nop, &socket_can_context_1, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		&socket_can_api,
		CANBUS_RAW_L2, NET_L2_GET_CTX_TYPE(CANBUS_RAW_L2), CAN_MTU);

#endif
#endif  /* CONFIG_NET_SOCKETS_CAN */
