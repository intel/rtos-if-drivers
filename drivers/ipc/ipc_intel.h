/*
 * Copyright (c) 2020-2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __DRIVERS_IPC_INTEL__H
#define __DRIVERS_IPC_INTEL__H

#ifdef __cplusplus
extern "C" {
#endif
#include <intel/hal_ipc.h>
#include "sys/atomic.h"

#define IPC_BUSY_BIT 31
#define IPC_WRITE_TINEOUT_1S 1000
#define IPC_WRITE_TINEOUT_5S 5000
#define IPC_WRITE_IN_PROC_BIT 0
#define IPC_WRITE_BUSY_BIT 1
#define IPC_READ_BUSY_BIT 2
#define IPC_PEER_READY_BIT 3

#define VALID_DRBL(drbl) (drbl & BIT(IPC_BUSY_BIT))

typedef void (*ipc_config_func_t)(void);

struct ipc_intel_config {
	intel_instance_t *inst;
	uint32_t id;
	void (*irq_config)(const struct device *dev);
	int32_t default_timeout;
	bool need_sync;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

struct ipc_intel_data {
	ipc_new_msg_f rx_msg_notify_cb;
	struct k_sem device_write_msg_sem;
	struct k_mutex device_write_lock;
	atomic_t status;
	uint32_t power_status;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	pcie_bdf_t pcie_bdf;
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_IPC_INTEL__H */
