/*
 * Copyright (c) 2020-2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT intel_ipc
#include <kernel.h>
#include <init.h>
#include <errno.h>
#include <arch/cpu.h>
#include <zephyr/types.h>
#include <soc.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <device.h>
#include <sys/sys_io.h>
#include <intel/hal_ipc.h>
#include <drivers/ipc.h>
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif
#include "ipc_intel.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(ipc_intel, CONFIG_IPC_LOG_LEVEL);
#ifdef CONFIG_SHARED_IRQ
#include <shared_irq.h>
#endif

void ipc_intel_isr(IN void *arg)
{
	const struct device *dev = arg;
	const struct ipc_intel_config *info = dev->config;

	intel_ipc_isr(info->inst);
}

K_SEM_DEFINE(csr_sem, 0, 1);

static void ipc_intel_event_dispose(IN intel_instance_t *inst,
				    IN uint32_t event, INOUT void *params)
{
	__ASSERT((params != NULL), "bad params\n");
	const struct device *dev = (const struct device *)params;
	struct ipc_intel_data *ipc = dev->data;

	LOG_DBG("%s %p %p", __func__, ipc->rx_msg_notify_cb, params);
	switch (event) {
	case INTEL_IPC_EVENT_MSG_IN:
		LOG_DBG("INTEL_IPC_EVENT_MSG_IN");
		if (ipc->rx_msg_notify_cb != NULL) {
			ipc->rx_msg_notify_cb(dev, NULL);
		} else {
			LOG_WRN("no handler for ipc new msg");
		}
		break;
	case INTEL_IPC_EVENT_MSG_PEER_ACKED:
		LOG_DBG("INTEL_IPC_EVENT_MSG_PEER_ACKED");
		if (atomic_test_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT)) {
			LOG_DBG("INTEL_IPC_EVENT_MSG_PEER_ACKED");
			k_sem_give(&ipc->device_write_msg_sem);
		}
		break;
	default:
		return;
	}
}

static int ipc_intel_init(const struct device *dev)
{
	__ASSERT((dev != NULL), "bad params\n");
	/* allocate reso.urce and context*/
	const struct ipc_intel_config *info = dev->config;
	struct ipc_intel_data *ipc = dev->data;

	k_sem_init(&ipc->device_write_msg_sem, 0, 1);
	k_mutex_init(&ipc->device_write_lock);
	ipc->status = 0;

#if defined(CONFIG_PM_DEVICE)
	ipc->power_status = PM_DEVICE_STATE_ACTIVE;
#endif

	LOG_DBG("data %p conf: %p ipc_ctx %p CB:%p", ipc, info, info->inst,
		&ipc->rx_msg_notify_cb);
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (info->pcie) {
		struct pcie_mbar mbar;

		ipc->rx_msg_notify_cb = NULL;
		ipc->pcie_bdf = pcie_bdf_lookup(info->pcie_id);
		if (!pcie_probe(ipc->pcie_bdf, info->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(ipc->pcie_bdf, 0, &mbar);
		pcie_set_cmd(ipc->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);

		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr, mbar.size,
			   K_MEM_CACHE_NONE);
		pcie_set_cmd(ipc->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);

		intel_set_base_addr(info->inst, DEVICE_MMIO_GET(dev));

		LOG_DBG("base reg add phy:%lu  vrtlAdd:%ld dev:%p\n",
			mbar.phys_addr, DEVICE_MMIO_GET(dev), (void *)dev);
		LOG_DBG("conf: %p ipc_ctx: %p CB:%p base_add:%ld\n", ipc,
			info->inst, ipc->rx_msg_notify_cb,
			info->inst->base_addr);

		ipc->rx_msg_notify_cb =
			NULL; /* TODO - fixme why data getting updated ?? */

		LOG_DBG("conf: %p ipc_ctx: %p CB:%p base_add:%ld\n", ipc,
			info->inst, ipc->rx_msg_notify_cb,
			info->inst->base_addr);
	} else {
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}
#else
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
#endif

	intel_ipc_init(info->inst, ipc_intel_event_dispose, (void *)dev);
	if (!info->need_sync) {
		atomic_set_bit(&ipc->status, IPC_PEER_READY_BIT);
	}
	info->irq_config(dev);
	LOG_DBG("IPC driver initialized on device: %p", dev);
	LOG_DBG("data: %p conf: %p ipc_ctx: %p CB:%p", ipc, info, info->inst,
		ipc->rx_msg_notify_cb);

	intel_ipc_set_host_ready(info->inst, true);
	return 0;
}

static int ipc_intel_write_msg(const struct device *dev, uint32_t drbl,
			       uint8_t *msg, uint32_t msg_size,
			       uint32_t *ack_drbl, uint8_t *ack_msg,
			       uint32_t ack_msg_size)
{
	__ASSERT((dev != NULL), "bad params\n");
	const struct ipc_intel_config *info = dev->config;
	struct ipc_intel_data *ipc = dev->data;
	int ret = 0;

	LOG_DBG("write_msg: %p, drbl=%08x", dev, drbl);

	/* check params, check status */

	if ((msg_size > IPC_DATA_LEN_MAX) ||
	    ((msg_size > 0) && (msg == NULL)) ||
	    (ack_msg_size > IPC_DATA_LEN_MAX) ||
	    ((ack_msg_size > 0) && (ack_msg == NULL)) ||
	    ((drbl & BIT(IPC_BUSY_BIT)) == 0)) {
		LOG_ERR("bad params when sending ipc msg on device: %p", dev);
		return -EINVAL;
	}

	k_mutex_lock(&ipc->device_write_lock, K_FOREVER);
	if (!atomic_test_bit(&ipc->status, IPC_PEER_READY_BIT)) {
		LOG_WRN("peer is not ready");
		goto write_err;
	}
	if (info->need_sync) {
		ret = k_sem_take(&csr_sem, K_MSEC(info->default_timeout));
		if (ret) {
			LOG_WRN("IPC wait csr timeout on device: %p", dev);
			goto write_err;
		}
	}
	/* write data regs */
	if (msg_size > 0) {
		ret = intel_ipc_write_msg(info->inst, msg, msg_size);
		if (ret != INTEL_DRIVER_OK) {
			LOG_ERR("IPC write data fail on device: %p", dev);
			if (info->need_sync) {
			}
			goto write_err;
		}
	}

	atomic_set_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);

	/* write drbl regs to interrupt peer*/
	ret = intel_ipc_write_dbl(info->inst, drbl);

	if (info->need_sync) {
	}

	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("IPC write doorbell fail on device: %p", dev);
		atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
		goto write_err;
	}

	/* wait for busy-bit-consumed interrupt */
	ret = k_sem_take(&ipc->device_write_msg_sem,
			 K_MSEC(info->default_timeout));
	if (ret) {
		LOG_WRN("IPC write timeout on device: %p", dev);
		atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
		intel_ipc_write_dbl(info->inst, 0);
		goto write_err;
	}
	if (ack_msg_size > 0) {
		ret = intel_ipc_read_ack_msg(info->inst, ack_msg, ack_msg_size);
		if (ret) {
			LOG_ERR("IPC read ack failed on device: %p", dev);
			atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
			goto write_err;
		}
	}
	if (ack_drbl) {
		ret = intel_ipc_read_ack_drbl(info->inst, ack_drbl);
		if (ret) {
			LOG_ERR("IPC read ack failed on device: %p", dev);
			atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
			goto write_err;
		}
	}

	atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);

	k_mutex_unlock(&ipc->device_write_lock);

	return 0;

write_err:
	k_mutex_unlock(&ipc->device_write_lock);
	return ret;
}

static int ipc_intel_set_rx_notify(const struct device *dev, ipc_new_msg_f cb)
{
	__ASSERT((dev != NULL), "bad params\n");

	const struct ipc_intel_config *info = dev->config;
	struct ipc_intel_data *ipc = dev->data;

	LOG_DBG("%s %p", __func__, cb);

	if (cb == NULL) {
		LOG_ERR("bad params when add ipc callback on device: %p", dev);
		return -EINVAL;
	}

	if (ipc->rx_msg_notify_cb == NULL) {
		ipc->rx_msg_notify_cb = cb;
		return 0;
	}

	LOG_ERR("ipc rx callback already exists: data:%p conf:%p rx_msg_notify_cb:%p",
		ipc, info, ipc->rx_msg_notify_cb);
	return -EADDRINUSE;
}

static int ipc_intel_read_drbl(const struct device *dev, uint32_t *drbl)
{
	__ASSERT((dev != NULL), "bad params\n");
	if (drbl == NULL) {
		LOG_ERR("bad params when read drbl on device: %p", dev);
		return -EINVAL;
	}
	int ret;
	const struct ipc_intel_config *info = dev->config;

	ret = intel_ipc_read_dbl(info->inst, drbl);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("IPC read doorbell fail on device: %p", dev);
		return -EIO;
	}
	return 0;
}

static int ipc_intel_read_msg(const struct device *dev, uint32_t *drbl,
			      uint8_t *msg, uint32_t msg_size)
{
	__ASSERT((dev != NULL), "bad params\n");
	int ret = 0;
	const struct ipc_intel_config *info = dev->config;

	if (drbl) {
		ret = intel_ipc_read_dbl(info->inst, drbl);
		if (ret != INTEL_DRIVER_OK) {
			LOG_ERR("IPC read drbl fail on device: %p", dev);
			return -EIO;
		}
	}
	if (msg_size == 0) {
		return 0;
	}
	if (msg == NULL) {
		LOG_ERR("bad params when read data on device: %p", dev);
		return -EINVAL;
	}
	ret = intel_ipc_read_msg(info->inst, msg, msg_size);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("IPC read data failed on device: %p", dev);
		return -EIO;
	}
	return 0;
}

static int ipc_intel_send_ack(const struct device *dev, uint32_t ack_drbl,
			      uint8_t *ack_msg, uint32_t ack_msg_size)
{
	int ret = 0;

	__ASSERT((dev != NULL), "bad params\n");
	if ((ack_drbl & BIT(IPC_BUSY_BIT)) ||
	    (ack_msg_size > IPC_DATA_LEN_MAX) ||
	    ((ack_msg_size > 0) && (ack_msg == NULL))) {
		LOG_ERR("bad params when sending ack on device: %p", dev);
		return -EIO;
	}

	const struct ipc_intel_config *info = dev->config;

	if (ack_msg != NULL) {
		ret = intel_ipc_send_ack_msg(info->inst, ack_msg, ack_msg_size);
		if (ret != INTEL_DRIVER_OK) {
			LOG_ERR("IPC send ack msg fail on device: %p", dev);
			goto ack_end;
		}
	}

	ret = intel_ipc_send_ack_drbl(info->inst, ack_drbl);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("IPC send ack drl fail on device: %p", dev);
	}

ack_end:

	return ret;
}

struct ipc_driver_api intel_ipc_api = { .ipc_set_rx_notify =
						ipc_intel_set_rx_notify,
					.ipc_read_drbl = ipc_intel_read_drbl,
					.ipc_read_msg = ipc_intel_read_msg,
					.ipc_send_ack = ipc_intel_send_ack,
					.ipc_write_msg = ipc_intel_write_msg };

/* not PCI(e) */
#define IPC_INTEL_IRQ_INIT_PCIE0(n)				       \
	static void ipc_##n##_irq_init(const struct device *port)      \
	{							       \
		ARG_UNUSED(port);				       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    ipc_intel_isr, DEVICE_DT_INST_GET(n), 0);  \
		irq_enable(DT_INST_IRQN(n));			       \
	}

#define IPC_INTEL_IRQ_FLAGS(n) \
	(((DT_INST_IRQ_HAS_CELL(n, sense)) == 0) ? (DT_INST_IRQ(n, sense)) : 0)
/* PCI(e) with auto IRQ detection */
#define IPC_INTEL_IRQ_INIT_PCIE1(n)					     \
	static void ipc_##n##_irq_init(const struct device *port)	     \
	{								     \
		ARG_UNUSED(port);					     \
		BUILD_ASSERT(DT_INST_IRQN(n) == PCIE_IRQ_DETECT,	     \
			     "Only runtime IRQ configuration is supported"); \
		BUILD_ASSERT(IS_ENABLED(CONFIG_DYNAMIC_INTERRUPTS),	     \
			     "DW I2C PCI needs CONFIG_DYNAMIC_INTERRUPTS");  \
		unsigned int irq = pcie_alloc_irq(DT_INST_REG_ADDR(n));	     \
									     \
		if (irq == PCIE_CONF_INTR_IRQ_NONE) {			     \
			LOG_ERR("Error connect intr for BDF:%x!\n",	     \
				DT_INST_REG_ADDR(n));			     \
			return;						     \
		}							     \
		LOG_DBG("Connect:%x %p %x\n", DT_INST_IRQ(n, priority),	     \
			DEVICE_DT_INST_GET(n), IPC_INTEL_IRQ_FLAGS(n));	     \
		irq_connect_dynamic(irq, DT_INST_IRQ(n, priority),	     \
				    ipc_intel_isr, port,		     \
				    IPC_INTEL_IRQ_FLAGS(n));		     \
		pcie_irq_enable(DT_INST_REG_ADDR(n), irq);		     \
	}

#define IPC_INTEL_IRQ_INIT(n) \
	_CONCAT(IPC_INTEL_IRQ_INIT_PCIE, DT_INST_ON_BUS(n, pcie)) (n)

#define INIT_PCIE0(n)
#define INIT_PCIE1(n) .pcie = true, .pcie_id = DT_INST_REG_SIZE(n),
#define INIT_PCIE(n) _CONCAT(INIT_PCIE, DT_INST_ON_BUS(n, pcie)) (n)

#define INTEL_IPC_DEFINE(n)						 \
	IPC_INTEL_IRQ_INIT(n)						 \
	static struct ipc_intel_data ipc_data_##n;			 \
									 \
	static struct ipc_intel_config ipc_config_##n = {		 \
		.inst = INTEL_IPC_INSTANCE(n),				 \
		.id = 0,						 \
		.irq_config = ipc_##n##_irq_init,			 \
		.default_timeout = DT_INST_PROP(n, timeout_ms),		 \
		INIT_PCIE(n) };						 \
	DEVICE_DT_INST_DEFINE(						 \
		n, ipc_intel_init, NULL, &ipc_data_##n, &ipc_config_##n, \
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &intel_ipc_api);

DT_INST_FOREACH_STATUS_OKAY(INTEL_IPC_DEFINE)
