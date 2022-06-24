/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _CAN_INTEL_H_
#define _CAN_INTEL_H_

#include <drivers/can.h>
#include "intel/hal_can.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif

#define CAN_INVALID_BUFNUM 255

#define CAN_BAUD_RATE_SJW 4
#define OSC_DIV_FACTOR (1000)

#define CAN_BAUD_RATE_PHASE_SEG1_20_KBPS 22
#define CAN_BAUD_RATE_PHASE_SEG2_20_KBPS 15
#define CAN_QUANTA_20_KBPS                              (0.00125)
#define CAN_BAUD_RATE_PHASE_BRP_20_KBPS(x) ((uint32_t) \
					    (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_20_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_50_KBPS 22
#define CAN_BAUD_RATE_PHASE_SEG2_50_KBPS 15
#define CAN_QUANTA_50_KBPS                              (0.0005)
#define CAN_BAUD_RATE_PHASE_BRP_50_KBPS(x) ((uint32_t) \
					    (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_50_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_100_KBPS 22
#define CAN_BAUD_RATE_PHASE_SEG2_100_KBPS 15
#define CAN_QUANTA_100_KBPS                             (0.00025)
#define CAN_BAUD_RATE_PHASE_BRP_100_KBPS(x) ((uint32_t)	\
					     (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_100_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_125_KBPS 7
#define CAN_BAUD_RATE_PHASE_SEG2_125_KBPS 6
#define CAN_QUANTA_125_KBPS                             (0.0005)
#define CAN_BAUD_RATE_PHASE_BRP_125_KBPS(x) ((uint32_t)	\
					     (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_125_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_250_KBPS 7
#define CAN_BAUD_RATE_PHASE_SEG2_250_KBPS 6
#define CAN_QUANTA_250_KBPS                             (0.00025)
#define CAN_BAUD_RATE_PHASE_BRP_250_KBPS(x) ((uint32_t)	\
					     (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_250_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_500_KBPS 10
#define CAN_BAUD_RATE_PHASE_SEG2_500_KBPS 7
#define CAN_QUANTA_500_KBPS                             (0.0001)
#define CAN_BAUD_RATE_PHASE_BRP_500_KBPS(x) ((uint32_t)	\
					     (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_500_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_800_KBPS 1
#define CAN_BAUD_RATE_PHASE_SEG2_800_KBPS 1
#define CAN_QUANTA_800_KBPS                             (0.00025)
#define CAN_BAUD_RATE_PHASE_BRP_800_KBPS(x) ((uint32_t)	\
					     (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_800_KBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_1_MBPS 4
#define CAN_BAUD_RATE_PHASE_SEG2_1_MBPS 3
#define CAN_QUANTA_1_MBPS                               (0.0001)
#define CAN_BAUD_RATE_PHASE_BRP_1_MBPS(x) ((uint32_t) \
					   (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_1_MBPS)) - 1)

#define CAN_BAUD_RATE_PHASE_SEG1_2_MBPS 1
#define CAN_BAUD_RATE_PHASE_SEG2_2_MBPS 1
#define CAN_QUANTA_2_MBPS                               (0.0001)
#define CAN_BAUD_RATE_PHASE_BRP_2_MBPS(x) ((uint32_t) \
					   (((x) / OSC_DIV_FACTOR) * (CAN_QUANTA_2_MBPS)) - 1)

#define MIN_TIMING_SJW 1
#define MIN_TIMING_PROP_SEG 1
#define MIN_TIMING_PHASE_SEG1 1
#define MIN_TIMING_PHASE_SEG2 1
#define MIN_TIMING_PRESCALAR 1

#define MAX_TIMING_SJW 128
#define MAX_TIMING_PROP_SEG 1
#define MAX_TIMING_PHASE_SEG1 256
#define MAX_TIMING_PHASE_SEG2 128
#define MAX_TIMING_PRESCALAR 512

#define MIN_TIMING_DATA_SJW 1
#define MIN_TIMING_DATA_PROP_SEG 1
#define MIN_TIMING_DATA_PHASE_SEG1 1
#define MIN_TIMING_DATA_PHASE_SEG2 1
#define MIN_TIMING_DATA_PRESCALAR 1

#define MAX_TIMING_DATA_SJW 16
#define MAX_TIMING_DATA_PROP_SEG 1
#define MAX_TIMING_DATA_PHASE_SEG1 32
#define MAX_TIMING_DATA_PHASE_SEG2 16
#define MAX_TIMING_DATA_PRESCALAR 32

struct can_mailbox {
	can_tx_callback_t tx_callback;
	void *cb_arg;
	struct k_sem mailbox_sem;
	uint32_t error_flags;
};

typedef void (*can_parity_callback_t)(uint32_t evt);

struct can_intel_data_t {
	DEVICE_MMIO_RAM;
	enum can_id id;
	int32_t tx_err;
	struct k_sem tx_sem;
	struct k_sem sync_tx_sem;
	struct k_sem set_filter_sem;
	can_parity_callback_t parity_cb;
	can_state_change_isr_t state_cb;
	int32_t attached_filter_count;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	pcie_bdf_t pcie_bdf;
#endif
};

struct can_intel_config_t {
	DEVICE_MMIO_ROM;
	intel_instance_t *instance;
	struct can_params_t *params;
	void (*config_irq)(const struct device *dev);
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

enum can_rx_notify_t {
	RX_NOTIFY_CB,
	RX_NOTIFY_MQ
};


struct filter_data_t {
	enum can_ide filter_id_type;
	bool configured;
	uint32_t id1;
	uint32_t id2;
	can_rx_callback_t cb;
	void *cb_arg;
	enum can_rx_notify_t rx_notify;
	uint8_t rtr;
};

enum can_baud_rate_t {
	CAN_BITRATE_20_KBPS     = 20000U,
	CAN_BITRATE_50_KBPS     = 50000U,
	CAN_BITRATE_100_KBPS    = 100000U,
	CAN_BITRATE_125_KBPS    = 125000U,
	CAN_BITRATE_250_KBPS    = 250000U,
	CAN_BITRATE_500_KBPS    = 500000U,
	CAN_BITRATE_800_KBPS    = 800000U,
	CAN_BITRATE_1_MBPS      = 1000000U,
	CAN_BITRATE_2_MBPS      = 2000000U,
};


int can_intel_check_filter_matched(struct zcan_frame *msg);

#define DEV_DATA(dev) ((struct can_intel_data_t *const)(dev)->data)
#define DEV_CFG(dev) \
	((struct can_intel_config_t *)(dev)->config)

int can_intel_ioctl(const struct device *dev, uint32_t can_cmd, void *data);
int can_intel_configure(const struct device *dev, enum can_mode conf_mode,
			uint32_t bitrate);
int can_intel_send(const struct device *dev, const struct zcan_frame *msg,
		   k_timeout_t timeout, can_tx_callback_t callback,
		   void *callback_arg);
int can_intel_attach_isr(const struct device *dev, can_rx_callback_t isr,
			 void *callback_arg, const struct zcan_filter *filter);
void can_intel_detach(const struct device *dev, int filter_nr);
int can_intel_recover(const struct device *dev, k_timeout_t timeout);
enum can_state can_intel_get_state(const struct device *dev,
				   struct can_bus_err_cnt *err_cnt);
void can_intel_register_state_change_isr(const struct device *dev,
					 can_state_change_isr_t isr);
int32_t register_parity_callback(const struct device *dev,
				 can_parity_callback_t cb);
#endif /*_CAN_INTEL_H_*/

