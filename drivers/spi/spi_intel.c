/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_intel);

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <kernel.h>
#include <drivers/spi.h>
#include <intel/hal_spi.h>
#include <intel/hal_dma.h>
#include "intel_spi_xfer.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif

#define SPI_NOT_USE_DMA (-1)

#define SPI_INST_GET(dev) (((const struct spi_intel_config *)dev->config)->inst)

struct spi_intel_config {
	intel_instance_t *inst;
	uint32_t id;
	void (*irq_config)(const struct device *dev);
	uint32_t clock_freq;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

struct spi_intel_data {
	pcie_bdf_t pcie_bdf;
	struct intel_spi_xfer xfer;
	uint32_t id;
	bool tx_data_updated;
	bool rx_data_updated;
	uint32_t tx_dummy_len;
	uint32_t rx_dummy_len;
	bool is_locked;
#ifdef CONFIG_SPI_INTEL_USE_DMA
	int tx_dma_instance;
	int rx_dma_instance;
	int tx_channel;
	int rx_channel;
#endif
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	uint32_t device_power_state;
#endif
};

static int spi_intel_configure(const struct device *dev,
			       const struct spi_config *config)
{
	struct spi_intel_data *data = dev->data;
	const struct spi_intel_config *info = dev->config;
	uint32_t word_size, cpol, cpha, loopback;

	LOG_DBG("%s", __func__);

	word_size = SPI_WORD_SIZE_GET(config->operation);
	intel_spi_control(info->inst, INTEL_SPI_IOCTL_DATA_WIDTH, word_size);

	/* CPOL and CPHA */
	cpol = SPI_MODE_GET(config->operation) & SPI_MODE_CPOL;
	cpha = SPI_MODE_GET(config->operation) & SPI_MODE_CPHA;

	if ((cpol == 0) && (cpha == 0)) {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_CPOL0_CPHA0, 0);
	} else if ((cpol == 0) && (cpha == 1U)) {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_CPOL0_CPHA1, 0);
	} else if ((cpol == 1) && (cpha == 0U)) {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_CPOL1_CPHA0, 0);
	} else {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_CPOL1_CPHA1, 0);
	}

	/* MSB and LSB */
	if (config->operation & SPI_TRANSFER_LSB) {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_LSB, 0);
	}

	/* Set loopack */
	loopback = SPI_MODE_GET(config->operation) & SPI_MODE_LOOP;
	intel_spi_control(info->inst, INTEL_SPI_IOCTL_LOOPBACK, loopback);

	/* Set baudrate */
	intel_spi_control(info->inst, INTEL_SPI_IOCTL_SPEED_SET,
			  config->frequency);

	intel_spi_control(info->inst, INTEL_SPI_IOCTL_CS_HW, config->slave);

	/* Set LOCK_ON */
	if ((config->operation) & SPI_LOCK_ON) {
		data->is_locked = true;
	}

	data->xfer.config = config;
	spi_xfer_cs_configure(&data->xfer);

	return 0;
}

static int transceive(const struct device *dev, const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs, bool asynchronous,
		      struct k_poll_signal *signal)
{
	const struct spi_intel_config *info = dev->config;
	struct spi_intel_data *spi = dev->data;
	struct intel_spi_xfer *xfer = &spi->xfer;
	int ret;
	uint32_t transfer_bytes = 0;
	uint8_t *data_out = NULL, *data_in = NULL;
	uint32_t i, dummy_len = 0;
	const struct spi_buf *buf;
	bool is_multibufs = false;

	LOG_DBG("%s", __func__);

	spi_xfer_lock(&spi->xfer, asynchronous, signal);

	/* If device locked, will return error, need to release first */
	if ((spi->is_locked) && (!spi_xfer_configured(&spi->xfer, config))) {
		ret = -EIO;
		goto out;
	}

	/* If it is multi-bufsets, if yes, even users set DMA transfer enable,
	 * use interrupt instead. Reason is DW SPI have no feature to configure
	 * cs pin manually, DMA operation cannot support multi buffer set.
	 */
	if ((tx_bufs && tx_bufs->count > 1) || (rx_bufs && rx_bufs->count > 1)) {
		is_multibufs = true;
	}

	if ((tx_bufs) && (rx_bufs)) {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_BUFFER_SETS, 1);
	} else {
		intel_spi_control(info->inst, INTEL_SPI_IOCTL_BUFFER_SETS, 0);
	}


	/* If need to configure, re-configure */
	if (config != NULL) {
		spi_intel_configure(dev, config);
	}

	spi->tx_data_updated = false;
	spi->rx_data_updated = false;
	/* Set buffers info */
	spi_xfer_buffers_setup(&spi->xfer, tx_bufs, rx_bufs, 1);

	if (xfer->tx_count > xfer->rx_count) {
		spi->tx_dummy_len = 0;
		for (i = xfer->rx_count; i < xfer->tx_count; i++) {
			buf = xfer->current_tx + i;
			dummy_len += buf ? buf->len : 0;
		}
		spi->rx_dummy_len = dummy_len;
	} else if (xfer->tx_count < xfer->rx_count) {
		spi->rx_dummy_len = 0;
		for (i = xfer->tx_count; i < xfer->rx_count; i++) {
			buf = xfer->current_rx + i;
			dummy_len += buf ? buf->len : 0;
		}
		spi->tx_dummy_len = dummy_len;
	} else {
		spi->tx_dummy_len = 0;
		spi->rx_dummy_len = 0;
	}

	if ((xfer->tx_len == 0) && (xfer->rx_len == 0)) {
		spi_xfer_cs_control(&spi->xfer, false);
		spi_xfer_complete(&spi->xfer, 0);
		return 0;
	}

	if (xfer->tx_len == 0) {
		/* rx only, nothing to tx */
		data_out = NULL;
		data_in = (uint8_t *)xfer->rx_buf;
		transfer_bytes = xfer->rx_len;
		spi->tx_dummy_len -= transfer_bytes;
	} else if (xfer->rx_len == 0) {
		/* tx only, nothing to rx */
		data_out = (uint8_t *)xfer->tx_buf;
		data_in = NULL;
		transfer_bytes = xfer->tx_len;
		spi->rx_dummy_len -= transfer_bytes;
	} else if (xfer->tx_len == xfer->rx_len) {
		/* rx and tx are the same length */
		data_out = (uint8_t *)xfer->tx_buf;
		data_in = (uint8_t *)xfer->rx_buf;
		transfer_bytes = xfer->tx_len;
	} else if (xfer->tx_len > xfer->rx_len) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		data_out = (uint8_t *)xfer->tx_buf;
		data_in = xfer->rx_buf;
		transfer_bytes = xfer->rx_len;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		data_out = (uint8_t *)xfer->tx_buf;
		data_in = xfer->rx_buf;
		transfer_bytes = xfer->tx_len;
	}

	spi_xfer_cs_control(&spi->xfer, true);

#ifdef CONFIG_SPI_INTEL_USE_DMA
	if ((spi->tx_dma_instance >= 0) && (spi->tx_channel >= 0) &&
	    (spi->rx_dma_instance >= 0) && (spi->rx_channel >= 0) &&
	    (is_multibufs == false)) {
		intel_instance_t *tx, *rx;

		tx = intel_get_dma_instance(spi->tx_dma_instance);
		rx = intel_get_dma_instance(spi->rx_dma_instance);

		ret = intel_spi_dma_transfer(info->inst,
					     tx, spi->tx_channel, data_out,
					     rx, spi->rx_channel, data_in,
					     transfer_bytes);
	} else
#endif
	{
#ifdef CONFIG_SPI_INTEL_POLLED_MODE
		ret = intel_spi_transfer(info->inst, data_out, data_in,
					 transfer_bytes, true);
#else
		ret = intel_spi_transfer(info->inst, data_out, data_in,
					 transfer_bytes, false);
#endif
	}

	if (ret != INTEL_DRIVER_OK) {
		goto out;
	}

	ret = spi_xfer_wait_for_completion(&spi->xfer);
out:
	spi_xfer_release(&spi->xfer, ret);

	return ret;
}

static int spi_intel_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_intel_transceive_async(const struct device *dev,
				      const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      struct k_poll_signal *async)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_intel_release(const struct device *dev,
			     const struct spi_config *config)
{
	struct spi_intel_data *spi = dev->data;

	if (!spi_xfer_configured(&spi->xfer, config)) {
		return -EINVAL;
	}

	/* Lock status to false */
	spi->is_locked = false;
	spi_xfer_unlock_unconditionally(&spi->xfer);

	return 0;
}

static void spi_intel_isr(const void *arg)
{
	const struct device *dev = arg;
	intel_instance_t *inst = SPI_INST_GET(dev);

	intel_spi_isr(inst);
}

void spi_intel_callback(uint32_t event, void *param)
{
	struct device *dev = param;
	struct spi_intel_data *spi = dev->data;
	const struct spi_intel_config *info = dev->config;
	struct intel_spi_xfer *xfer = &spi->xfer;
	int error;

	/* Update the semaphore of spi context */
	if (event == INTEL_SPI_EVENT_DATA_LOST) {
		error = -EIO;
	} else {
		error = 0;
	}

	if ((event == INTEL_SPI_EVENT_COMPLETE) ||
	    (event == INTEL_SPI_EVENT_DATA_LOST)) {
		spi_xfer_cs_control(&spi->xfer, false);
		spi_xfer_complete(&spi->xfer, error);
	} else if (event == INTEL_SPI_EVENT_TX_FINISHED) {
		spi_xfer_update_tx(xfer, 1, xfer->tx_len);
		if (xfer->tx_len != 0) {
			intel_spi_update_tx_buf(info->inst, xfer->tx_buf,
						xfer->tx_len);
			if ((xfer->rx_len == 0) &&
			    (spi->rx_data_updated == false)) {
				/* Update rx length if always no rx */
				intel_spi_update_rx_buf(info->inst, NULL,
							spi->rx_dummy_len);
				spi->rx_data_updated = true;
			}
		} else if (spi->tx_data_updated == false) {
			intel_spi_update_tx_buf(info->inst, NULL,
						spi->tx_dummy_len);
			spi->tx_data_updated = true;
		}
	} else if (event == INTEL_SPI_EVENT_RX_FINISHED) {
		spi_xfer_update_rx(xfer, 1, xfer->rx_len);
		if (xfer->rx_len != 0) {
			intel_spi_update_rx_buf(info->inst, xfer->rx_buf,
						xfer->rx_len);
		}
	}
}

static const struct spi_driver_api intel_spi_api = {
	.transceive = spi_intel_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_intel_transceive_async,
#endif  /* CONFIG_SPI_ASYNC */
	.release = spi_intel_release,
};

int spi_intel_init(const struct device *dev)
{
	const struct spi_intel_config *info = dev->config;
	struct spi_intel_data *spi = dev->data;
	int ret;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (info->pcie) {
		struct pcie_mbar mbar;

		spi->pcie_bdf = pcie_bdf_lookup(info->pcie_id);
		if (!pcie_probe(spi->pcie_bdf, info->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(spi->pcie_bdf, 0, &mbar);
		pcie_set_cmd(spi->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
		pcie_set_cmd(spi->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);
		intel_set_base_addr(info->inst, DEVICE_MMIO_GET(dev));
		intel_set_phy_addr(info->inst, mbar.phys_addr);
	} else {
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}
#else
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
#endif

	ret = intel_spi_init(info->inst, spi_intel_callback, (void *)dev);
	if (ret != INTEL_DRIVER_OK) {
		return -ENODEV;
	}

	/* Init and connect IRQ */
	info->irq_config(dev);

	spi_xfer_unlock_unconditionally(&spi->xfer);

	/* Lock state to false */
	spi->is_locked = false;

	return 0;
}

/* not PCI(e) */
#define SPI_INTEL_IRQ_INIT_PCIE0(n)				       \
	static void spi_##n##_irq_init(const struct device *port)      \
	{							       \
		ARG_UNUSED(port);				       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    spi_intel_isr, DEVICE_DT_INST_GET(n), 0);  \
		irq_enable(DT_INST_IRQN(n));			       \
	}

#define SPI_INTEL_IRQ_FLAGS(n) (((DT_INST_IRQ_HAS_CELL(n, sense)) == 0) ? \
				(DT_INST_IRQ(n, sense)) : 0)

/* PCI(e) with auto IRQ detection */
#define SPI_INTEL_IRQ_INIT_PCIE1(n)					     \
	static void spi_##n##_irq_init(const struct device *port)	     \
	{								     \
		ARG_UNUSED(port);					     \
		BUILD_ASSERT(DT_INST_IRQN(n) == PCIE_IRQ_DETECT,	     \
			     "Only runtime IRQ configuration is supported"); \
		BUILD_ASSERT(IS_ENABLED(CONFIG_DYNAMIC_INTERRUPTS),	     \
			     "DW I2C PCI needs CONFIG_DYNAMIC_INTERRUPTS");  \
		unsigned int irq = pcie_alloc_irq(DT_INST_REG_ADDR(n));	     \
		if (irq == PCIE_CONF_INTR_IRQ_NONE) {			     \
			return;						     \
		}							     \
		irq_connect_dynamic(irq, DT_INST_IRQ(n, priority),	     \
				    spi_intel_isr, DEVICE_DT_INST_GET(n),    \
				    SPI_INTEL_IRQ_FLAGS(n));		     \
		pcie_irq_enable(DT_INST_REG_ADDR(n), irq);		     \
	}

#define SPI_INTEL_IRQ_INIT(n) \
	_CONCAT(SPI_INTEL_IRQ_INIT_PCIE, DT_INST_ON_BUS(n, pcie))(n)

#define INIT_PCIE0(n)
#define INIT_PCIE1(n) \
	.pcie = true, \
	.pcie_id = DT_INST_REG_SIZE(n),
#define INIT_PCIE(n) _CONCAT(INIT_PCIE, DT_INST_ON_BUS(n, pcie))(n)

#ifdef CONFIG_SPI_INTEL_USE_DMA
#define INIT_DMA(n)					  \
	.tx_dma_instance = 1,				  \
	.rx_dma_instance = 2,				  \
	.tx_channel = DT_INST_PROP(n, peripheral_id) * 2, \
	.rx_channel = (DT_INST_PROP(n, peripheral_id) * 2) + 1,
#else
#define INIT_DMA(n)
#endif

#define INTEL_SPI_DEFINE(n)					     \
	SPI_INTEL_IRQ_INIT(n)					     \
	static struct spi_intel_data spi_##n##_data = {		     \
		SPI_XFER_INIT_LOCK(spi_##n##_data, xfer),	     \
		SPI_XFER_INIT_SYNC(spi_##n##_data, xfer),	     \
		.id = DT_INST_PROP(n, peripheral_id),		     \
		INIT_DMA(n)					     \
	};							     \
	static const struct spi_intel_config spi_##n##_config = {    \
		.inst = INTEL_SPI_INSTANCE(n),			     \
		.id = DT_INST_PROP(n, peripheral_id),		     \
		.irq_config = spi_##n##_irq_init,		     \
		.clock_freq = DT_INST_PROP(n, clock_frequency),	     \
		INIT_PCIE(n)					     \
	};							     \
	DEVICE_DT_INST_DEFINE(n, spi_intel_init, NULL,		     \
			      &spi_##n##_data, &spi_##n##_config,    \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
			      &intel_spi_api);

DT_INST_FOREACH_STATUS_OKAY(INTEL_SPI_DEFINE)
