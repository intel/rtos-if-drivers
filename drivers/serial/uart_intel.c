/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_ia_uart

#include <errno.h>
#include <drivers/uart.h>
#include <init.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <zephyr/types.h>
#include <soc.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#include <intel/hal_uart.h>
#include <intel/hal_dma.h>
LOG_MODULE_REGISTER(uart, LOG_LEVEL_ERR);

#define LINE_CONTROL INTEL_UART_LC_8N1

static void uart_intel_isr(void *arg);
static void uart_intel_cb(const struct device *port);

/* Ctrlr 1 used for tx operation. */
#define UART_DMA_CONTROLLER_TX (1)
/* Ctrlr 2 used for rx operation.  */
#define UART_DMA_CONTROLLER_RX (2)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif
/* Convenient macro to get the controller instance. */
#define UART_GET_INSTANCE(dev) \
	(((const struct uart_intel_config_info *)dev->config)->instance)

#define GET_MUTEX(dev)				  \
	(((const struct uart_intel_config_info *) \
	  dev->config)->mutex)

/* Convenient macro to get tx semamphore */
#define GET_TX_SEM(dev)				  \
	(((const struct uart_intel_config_info *) \
	  dev->config)->tx_sem)

/* Convenient macro to get rx sempahore */
#define GET_RX_SEM(dev)				  \
	(((const struct uart_intel_config_info *) \
	  dev->config)->rx_sem)

struct uart_intel_config_info {

	DEVICE_MMIO_ROM;

	/* Specifies the uart instance for configuration. */
	intel_instance_t *instance;

	/* Specifies the baudrate for the uart instance. */
	uint32_t baud_rate;

	/* Specifies the clock frequency for the uart instance. */
	uint32_t clk_freq;

	/* Specifies the line control settings */
	intel_uart_lc_t line_ctrl;

	/* Enable / disable hardware flow control for UART. */
	bool hw_fc;
	intel_uart_dma_xfer_t *dma_tx_xfer;
	intel_uart_dma_xfer_t *dma_rx_xfer;
	intel_uart_transfer_t *xfer_tx;
	intel_uart_transfer_t *xfer_rx;
	intel_uart_unsol_rx_t *unsol_rx;

	uint32_t dma_rx_chnl;
	uint32_t dma_tx_chnl;
	struct k_mutex *mutex;
	struct k_sem *tx_sem;
	struct k_sem *rx_sem;

	/* UART irq configuration function when supporting interrupt
	 * mode.
	 */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t uart_irq_config_func;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_bdf_t pcie_bdf;
	pcie_id_t pcie_id;
#endif
};

struct uart_intel_drv_data {

	DEVICE_MMIO_RAM;
	pcie_bdf_t pcie_bdf;
	struct uart_config uart_config;
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
	struct uart_event evt;
	uart_callback_t async_cb;
	void *async_user_data;
	uart_unsol_rx_cb_t unsol_rx_usr_cb;
	void *unsol_rx_usr_cb_param;
};

static int get_xfer_error(int bsp_err)
{
	int err;

	switch (bsp_err) {
	case INTEL_DRIVER_OK:
		err = 0;
		break;
	case INTEL_USART_ERROR_CANCELED:
		err = -ECANCELED;
		break;
	case INTEL_DRIVER_ERROR:
		err = -EIO;
		break;
	case INTEL_DRIVER_ERROR_PARAMETER:
		err = -EINVAL;
		break;
	case INTEL_DRIVER_ERROR_UNSUPPORTED:
		err = -ENOTSUP;
		break;
	default:
		err = -EFAULT;
	}
	return err;
}

static uint32_t uart_intel_decode_line_err(uint32_t bsp_line_err)
{
	uint32_t zephyr_line_err = 0;

	if (bsp_line_err  &  INTEL_UART_RX_OE) {
		zephyr_line_err = UART_ERROR_OVERRUN;
	}

	if (bsp_line_err & INTEL_UART_RX_PE) {
		zephyr_line_err = UART_ERROR_PARITY;
	}

	if (bsp_line_err  & INTEL_UART_RX_FE) {
		zephyr_line_err = UART_ERROR_FRAMING;
	}

	if (bsp_line_err & INTEL_UART_RX_BI) {
		zephyr_line_err = UART_BREAK;
	}

	return zephyr_line_err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_intel_fifo_fill(const struct device *dev, const uint8_t *tx_data,
				int size)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_fifo_fill(instance, tx_data, size);
}

static int uart_intel_fifo_read(const struct device *dev, uint8_t *rx_data,
				const int size)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_fifo_read(instance, rx_data, size);
}

static void uart_intel_irq_tx_enable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_tx_enable(instance);
}

static void uart_intel_irq_tx_disable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_tx_disable(instance);
}

static int uart_intel_irq_tx_ready(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_irq_tx_ready(instance);
}

static int uart_intel_irq_tx_complete(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_is_tx_complete(instance);
}

static void uart_intel_irq_rx_enable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_rx_enable(instance);
}

static void uart_intel_irq_rx_disable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_rx_disable(instance);
}

static int uart_intel_irq_rx_ready(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_is_irq_rx_ready(instance);
}

static void uart_intel_irq_err_enable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_err_enable(instance);
}

static void uart_intel_irq_err_disable(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_irq_err_disable(instance);
}

static int uart_intel_irq_is_pending(const struct device *dev)
{

	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	return intel_uart_is_irq_pending(instance);
}

static int uart_intel_irq_update(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_update_irq_cache(instance);
	return 1;
}

static void uart_intel_irq_callback_set(const struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *user_data)
{
	struct uart_intel_drv_data *drv_data = dev->data;

	drv_data->user_cb = cb;
	drv_data->user_data = user_data;

}

static void unsol_rx_cb(void *data, int bsp_err, uint32_t line_status, int len)
{
	const struct device *dev = data;
	struct uart_intel_drv_data *drv_data = dev->data;
	void *usr_param;
	uint32_t line_err;
	int err;

	err = get_xfer_error(bsp_err);
	usr_param = drv_data->unsol_rx_usr_cb_param;
	line_err = uart_intel_decode_line_err(line_status);
	drv_data->unsol_rx_usr_cb(dev, usr_param, err, line_err, len);

}

static int uart_intel_enable_unsol_receive(const struct device *dev,
					   uint8_t *buff, int32_t size,
					   uart_unsol_rx_cb_t cb, void *param)
{

	__ASSERT(buff != NULL, "");
	__ASSERT(cb != NULL, "");
	__ASSERT(size != 0, "");

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}

	const struct uart_intel_config_info *config = dev->config;
	struct uart_intel_drv_data *drv_data = dev->data;

	drv_data->unsol_rx_usr_cb = cb;
	drv_data->unsol_rx_usr_cb_param = param;
	intel_instance_t *inst = UART_GET_INSTANCE(dev);

	config->unsol_rx->buffer = buff;
	config->unsol_rx->size = size;
	config->unsol_rx->cb_data = (void *)dev;
	config->unsol_rx->unsol_rx_callback = unsol_rx_cb;
	if (intel_uart_enable_unsol_rx(inst, config->unsol_rx)) {
		k_sem_give(GET_RX_SEM(dev));
		LOG_ERR("Failed to enable unsol RX");
		return -EIO;
	}
	return 0;
}

static int  uart_intel_get_unsol_data(const struct device *dev, uint8_t *buff,
				      int32_t len)
{

	__ASSERT(buff != NULL, "");
	int ret;
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	ret = intel_uart_get_unsol_data(instance, buff, len);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Failed to get unsol data. ret:%d", ret);
		return -EIO;
	}
	return 0;
}



static int uart_intel_disable_unsol_receive(const struct device *dev)
{

	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (intel_uart_disable_unsol_rx(instance) != INTEL_DRIVER_OK) {
		LOG_ERR("Failed to disable unsol RX");
		return -EIO;
	}
	k_sem_give(GET_RX_SEM(dev));
	return 0;
}

static int uart_intel_get_unsol_data_len(const struct device *dev, int *p_len)
{

	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (intel_uart_get_unsol_data_len(instance, p_len) != INTEL_DRIVER_OK) {
		LOG_ERR("Failed to get unsol data length");
		return -EIO;
	}
	return 0;
}

#endif

static void uart_intel_isr(void *arg)
{
	const struct device *dev = arg;
	struct uart_intel_drv_data *drv_data = dev->data;

	if (drv_data->user_cb) {
		drv_data->user_cb(dev, drv_data->user_data);
	} else {
		uart_intel_cb(dev);
	}
}

static void uart_intel_cb(const struct device *port)
{
	intel_instance_t *instance = UART_GET_INSTANCE(port);

	intel_uart_isr_handler(instance);
}

#ifdef CONFIG_UART_LINE_CTRL
static int uart_intel_line_ctrl_set(const struct device *dev, uint32_t ctrl, uint32_t val)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	intel_uart_config_t cfg = { 0 };
	uint32_t mask;
	int ret;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		intel_uart_get_config(instance, &cfg);
		cfg.baud_rate = val;
		ret = intel_uart_set_config(instance, &cfg);
		break;

	case UART_LINE_CTRL_RTS:
		if (val) {
			ret = intel_uart_assert_rts(instance);
		} else {
			ret = intel_uart_de_assert_rts(instance);
		}
		break;
#ifdef CONFIG_UART_RS_485
	case UART_LINE_CTRL_RS_485:
		if (val) {
			ret = intel_uart_rs485_enable(instance);
		} else {
			ret = intel_uart_rs485_disable(instance);
		}
		break;
#endif

	case UART_LINE_CTRL_BREAK_CONDN:
		if (val) {
			ret = intel_uart_set_break_con(instance);
		} else {
			ret = intel_uart_clr_break_con(instance);
		}
		break;

	case UART_LINE_CTRL_LOOPBACK:
		if (val) {
			ret = intel_uart_set_loopback_mode(instance);
		} else {
			ret = intel_uart_clr_loopback_mode(instance);
		}
		break;

	case UART_LINE_CTRL_AFCE:
		if (val) {
			ret = intel_uart_auto_fc_enable(instance);
		} else {
			ret = intel_uart_auto_fc_disable(instance);
		}
		break;

	case UART_LINE_CTRL_LINE_STATUS_REPORT_MASK:
		mask = 0;
		if (val  & UART_ERROR_OVERRUN) {
			mask |= INTEL_UART_RX_OE;
		}

		if (val & UART_ERROR_PARITY) {
			mask |= INTEL_UART_RX_PE;
		}

		if (val & UART_ERROR_FRAMING) {
			mask |= INTEL_UART_RX_FE;
		}

		if (val & UART_BREAK) {
			mask |= INTEL_UART_RX_BI;
		}
		ret = intel_set_ln_status_report_mask(instance, mask);
		break;

	default:
		ret = -ENODEV;
	}
	k_mutex_unlock(GET_MUTEX(dev));
	ret = get_xfer_error(ret);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Set line control failed. ret:%d", ret);
	}
	return ret;
}

static int uart_intel_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	intel_uart_config_t cfg = { 0 };
	uint32_t mask;
	int ret;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		ret = intel_uart_get_config(instance, &cfg);
		*val = cfg.baud_rate;
		break;

	case UART_LINE_CTRL_RTS:
		ret = intel_uart_read_rts(instance, (uint32_t *)val);
		break;

	case UART_LINE_CTRL_LOOPBACK:
		ret = intel_uart_get_loopback_mode(instance, (uint32_t *)val);
		break;

	case UART_LINE_CTRL_AFCE:
		ret = intel_uart_get_config(instance, &cfg);
		*val = cfg.hw_fc;
		break;

	case UART_LINE_CTRL_LINE_STATUS_REPORT_MASK:
		mask = 0;
		*val = 0;
		ret = intel_get_ln_status_report_mask(instance,
						      (uint32_t *)&mask);
		*val |= ((mask & INTEL_UART_RX_OE) ? UART_ERROR_OVERRUN : 0);
		*val |= ((mask & INTEL_UART_RX_PE) ? UART_ERROR_PARITY : 0);
		*val |= ((mask & INTEL_UART_RX_FE) ? UART_ERROR_FRAMING : 0);
		*val |= ((mask & INTEL_UART_RX_BI) ? UART_BREAK : 0);
		break;

	case UART_LINE_CTRL_CTS:
		ret = intel_uart_read_cts(instance, (uint32_t *)val);
		break;

	default:
		ret = -ENODEV;
	}

	k_mutex_unlock(GET_MUTEX(dev));
	ret = get_xfer_error(ret);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Get line control failed. ret:%d", ret);
	}
	return ret;
}

#endif /* CONFIG_UART_LINE_CTRL */

#ifdef CONFIG_UART_ASYNC_API
static void uart_irq_xfer_tx_common_cb(void *data, int err, uint32_t status,
				       uint32_t len)
{
	const struct device *dev = data;
	struct uart_intel_drv_data *drv_data = dev->data;

	if (drv_data->async_cb) {
		drv_data->async_user_data = (void *)(uintptr_t)uart_intel_decode_line_err(status);
		drv_data->async_cb(dev, &drv_data->evt, drv_data->async_user_data);
	}
	k_sem_give(GET_TX_SEM(dev));
}

static void uart_irq_xfer_rx_common_cb(void *data, int err, uint32_t status,
				       uint32_t len)
{
	const struct device *dev = data;
	struct uart_intel_drv_data *drv_data = dev->data;
	uint32_t status_err;

	if (drv_data->async_cb) {
		status_err = uart_intel_decode_line_err(status);
		drv_data->async_user_data = (void *)(uintptr_t)uart_intel_decode_line_err(status);
		drv_data->async_cb(dev, &drv_data->evt, drv_data->async_user_data);
	}
	k_sem_give(GET_RX_SEM(dev));

}

static int uart_intel_async_callback_set(const struct device *dev,
					 uart_callback_t cb,
					 void *user_data)
{
	struct uart_intel_drv_data *drv_data = dev->data;

	drv_data->async_cb = cb;
	drv_data->async_user_data = user_data;

	return 0;
}

static int uart_intel_write_buffer_async(const struct device *dev,
					 const uint8_t *tx_buf, size_t tx_buf_size, int32_t timeout)
{
	int ret;

	__ASSERT(tx_buf != NULL, "");
	__ASSERT(tx_buf_size != 0, "");


	if (k_sem_take(GET_TX_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}

	struct uart_intel_drv_data *drv_data = dev->data;
	const struct uart_intel_config_info *config = dev->config;

	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (drv_data->async_cb) {
		drv_data->evt.type = UART_TX_DONE;
	}

#if CONFIG_UART_INTEL_USE_DMA
	const intel_instance_t *dma =
		intel_get_dma_instance(UART_DMA_CONTROLLER_TX);
	config->dma_tx_xfer->dma_dev = dma;
	config->dma_tx_xfer->channel = config->dma_tx_chnl;
	config->dma_tx_xfer->data = (uint8_t *)tx_buf;
	config->dma_tx_xfer->len = tx_buf_size;
	config->dma_tx_xfer->callback = uart_irq_xfer_tx_common_cb;
	config->dma_tx_xfer->cb_param = (void *)dev;
	ret = intel_uart_dma_write_async(instance, config->dma_tx_xfer);
#else
	config->xfer_tx->data = (uint8_t *)tx_buf;
	config->xfer_tx->data_len = tx_buf_size;
	config->xfer_tx->callback = uart_irq_xfer_tx_common_cb;
	config->xfer_tx->callback_data = (void *)dev;
	ret = intel_uart_write_async(instance, config->xfer_tx);
#endif

	if (ret != INTEL_DRIVER_OK) {
		ret = get_xfer_error(ret);
		k_sem_give(GET_TX_SEM(dev));
		return ret;
	}

	return 0;
}

static int uart_intel_write_abort_async(const struct device *dev)
{
	int ret;
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	struct uart_intel_drv_data *drv_data = dev->data;

	if (drv_data->async_cb) {
		drv_data->evt.type = UART_TX_ABORTED;
	}

#if CONFIG_UART_INTEL_USE_DMA
	ret = intel_uart_dma_write_terminate(instance);
#else
	ret = intel_uart_async_write_terminate(instance);
#endif
	if (ret != INTEL_DRIVER_OK) {
		ret = get_xfer_error(ret);
		return ret;
	}

	return ret;

}
static int uart_intel_read_buffer_async(const struct device *dev,
					uint8_t *rx_buf, size_t rx_buf_size, int32_t timeout)
{
	int ret;

	__ASSERT(rx_buf != NULL, "");
	__ASSERT(rx_buf_size != 0, "");

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}

	const struct uart_intel_config_info *config = dev->config;
	struct uart_intel_drv_data *drv_data = dev->data;

	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (drv_data->async_cb) {
		drv_data->evt.type = UART_RX_RDY;
	}

#if CONFIG_UART_INTEL_USE_DMA
	const intel_instance_t *dma =
		intel_get_dma_instance(UART_DMA_CONTROLLER_RX);
	config->dma_rx_xfer->dma_dev = dma;
	config->dma_rx_xfer->channel = config->dma_rx_chnl;
	config->dma_rx_xfer->data = rx_buf;
	config->dma_rx_xfer->len = rx_buf_size;
	config->dma_rx_xfer->callback = uart_irq_xfer_rx_common_cb;
	config->dma_rx_xfer->cb_param = (void *)dev;
	ret = intel_uart_dma_read_async(instance, config->dma_rx_xfer);
#else

	config->xfer_rx->data = rx_buf;
	config->xfer_rx->data_len = rx_buf_size;
	config->xfer_rx->callback = uart_irq_xfer_rx_common_cb;
	config->xfer_rx->callback_data = (void *)dev;
	ret = intel_uart_read_async(instance, config->xfer_rx);
#endif
	if (ret != INTEL_DRIVER_OK) {
		k_sem_give(GET_RX_SEM(dev));
		ret = get_xfer_error(ret);
		LOG_ERR("Read async failed. ret:%d", ret);
		return ret;
	}

	return 0;
}
static int uart_intel_read_disable_async(const struct device *dev)
{
	int ret;
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	struct uart_intel_drv_data *drv_data = dev->data;

	if (drv_data->async_cb) {
		drv_data->evt.type = UART_RX_DISABLED;
	}

#if CONFIG_UART_INTEL_USE_DMA
	ret = intel_uart_dma_read_terminate(instance);
#else
	ret = intel_uart_async_read_terminate(instance);
#endif
	if (ret != INTEL_DRIVER_OK) {
		ret = get_xfer_error(ret);
		return ret;
	}

	return ret;
}

static int uart_intel_read_buf_rsp(const struct device *dev, uint8_t *buf,
				   size_t len)
{
	return -ENOTSUP;
}

#endif /* CONFIG_UART_ASYNC_API */
#ifdef CONFIG_UART_DRV_CMD
static int uart_intel_drv_cmd(const struct device *dev, uint32_t cmd, uint32_t p)
{

#ifdef CONFIG_UART_RS_485
	int ret;
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (cmd == UART_DRIVER_CMD_SET_RX_ONLY_MODE) {
		ret = intel_uart_set_rx_only_mode(instance, p);
		goto ret_status;
	} else if (cmd == UART_DRIVER_CMD_SET_TX_ONLY_MODE) {
		ret = intel_uart_set_tx_only_mode(instance, p);
		goto ret_status;
	} else {
		LOG_ERR("Invalid transfer mode\n");
		return -EINVAL;
	}

ret_status:
	ret = get_xfer_error(ret);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Failed to set RX only mode. ret:%d", ret);
	}
	return ret;
#endif
	LOG_ERR("Invalid UART device");
	return -ENODEV;
}
#endif /* CONFIG_UART_DRV_CMD */

#ifdef CONFIG_UART_RS_485
static int uart_intel_rs_485_config_set(const struct device *dev,
					struct uart_rs_485_config *config)
{
	int ret;
	intel_uart_rs485_config_t bsp_rs_485_config = { 0 };
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	bsp_rs_485_config.de_assertion_time = config->de_assertion_time_ns;
	bsp_rs_485_config.de_deassertion_time = config->de_deassertion_time_ns;
	if (config->transfer_mode == UART_RS485_XFER_MODE_FULL_DUPLEX) {
		bsp_rs_485_config.transfer_mode =
			INTEL_UART_RS485_XFER_MODE_FULL_DUPLEX;
	} else {
		bsp_rs_485_config.transfer_mode =
			INTEL_UART_RS485_XFER_MODE_HALF_DUPLEX;
		bsp_rs_485_config.de_re_tat = config->de_re_tat_ns;
		bsp_rs_485_config.re_de_tat = config->re_de_tat_ns;
	}

	if (config->de_polarity == UART_RS485_POL_ACTIVE_LOW) {
		bsp_rs_485_config.de_polarity = INTEL_UART_RS485_POL_ACTIVE_LOW;
	} else {
		bsp_rs_485_config.de_polarity =
			INTEL_UART_RS485_POL_ACTIVE_HIGH;
	}

	if (config->re_polarity == UART_RS485_POL_ACTIVE_LOW) {
		bsp_rs_485_config.re_polarity =
			INTEL_UART_RS485_POL_ACTIVE_LOW;
	} else {
		bsp_rs_485_config.re_polarity =
			INTEL_UART_RS485_POL_ACTIVE_HIGH;
	}
	bsp_rs_485_config.de_en = true;
	bsp_rs_485_config.re_en = true;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	ret = intel_uart_rs485_set_config(instance, &bsp_rs_485_config);
	k_mutex_unlock(GET_MUTEX(dev));

	ret = get_xfer_error(ret);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Device configuration error, ret:%d", ret);
	}
	return ret;
}
#endif


static int uart_intel_poll_in(const struct device *dev, unsigned char *data)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	uint32_t status;
	int ret = 0;

	intel_uart_get_status(instance, (uint32_t *) &status);

	/* In order to check if there is any data to read from UART
	 * controller we should check if the INTEL_UART_RX_BUSY bit from
	 * 'status' is not set. This bit is set only if there is any
	 * pending character to read.
	 */
	if (!(status & INTEL_UART_RX_BUSY)) {
		ret = -1;
	} else {
		if (intel_uart_read(instance, data, (uint32_t *)&status)) {
			ret = -1;
		}
	}
	return ret;
}

static void uart_intel_poll_out(const struct device *dev,
				unsigned char data)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	intel_uart_write(instance, data);
}

static int uart_intel_err_check(const struct device *dev)
{
	intel_instance_t *instance = UART_GET_INSTANCE(dev);
	uint32_t status;
	int ret_status = 0;

	intel_uart_get_status(instance, (uint32_t *const)&status);
	if (status &  INTEL_UART_RX_OE) {
		ret_status = UART_ERROR_OVERRUN;
	}

	if (status & INTEL_UART_RX_PE) {
		ret_status = UART_ERROR_PARITY;
	}

	if (status & INTEL_UART_RX_FE) {
		ret_status = UART_ERROR_FRAMING;
	}

	if (status & INTEL_UART_RX_BI) {
		ret_status = UART_BREAK;
	}

	return ret_status;
}

static int uart_intel_read_buffer_polled(const struct device *dev, uint8_t *buff,
					 int len,  uint32_t *line_status)
{
	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");
	__ASSERT(line_status != NULL, "");

	uint32_t comp_len, bsp_line_err;
	int ret;
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}

#ifdef CONFIG_UART_INTEL_USE_DMA
	const struct uart_intel_config_info *config = dev->config;
	const intel_instance_t *dma =
		intel_get_dma_instance(UART_DMA_CONTROLLER_RX);

	ret = intel_uart_dma_read_polled(instance, dma,
					 config->dma_rx_chnl, buff, len, &bsp_line_err);
	comp_len = len;
#else
	ret = intel_uart_read_buffer(instance, buff, (uint32_t)len,
				     (uint32_t *)&comp_len, (uint32_t *)&bsp_line_err);
#endif

	if (ret != INTEL_DRIVER_OK) {
		ret = get_xfer_error(ret);
		LOG_ERR("Failed to read buffer, ret:%d", ret);
	} else {
		ret = comp_len;
	}

	*line_status = uart_intel_decode_line_err(bsp_line_err);
	k_sem_give(GET_RX_SEM(dev));
	return ret;
}

static int uart_intel_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	return -ENOTSUP;
}

static int uart_intel_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	return -ENOTSUP;
}

static int uart_pse_init(const struct device *dev)
{
	const struct uart_intel_config_info *config = dev->config;
	intel_uart_config_t cfg = { 0 };
	intel_instance_t *instance = UART_GET_INSTANCE(dev);

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	struct uart_intel_drv_data *drv_data = dev->data;

	if (config->pcie) {
		struct pcie_mbar mbar;

		drv_data->pcie_bdf = pcie_bdf_lookup(config->pcie_id);
		if (!pcie_probe(drv_data->pcie_bdf, config->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(drv_data->pcie_bdf, 0, &mbar);
		pcie_set_cmd(drv_data->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
		pcie_set_cmd(drv_data->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);
		intel_set_base_addr(instance, DEVICE_MMIO_GET(dev));
		intel_set_phy_addr(instance, mbar.phys_addr);
	} else
#endif
	{
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}

	cfg.line_control = INTEL_UART_LC_8N1;
	cfg.baud_rate = config->baud_rate;
	cfg.hw_fc = config->hw_fc;
	cfg.clk_speed_hz = config->clk_freq;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->uart_irq_config_func(dev);
#endif
	intel_uart_set_config(instance, &cfg);
	return 0;
}

static const struct uart_driver_api api = {
	.poll_in = uart_intel_poll_in,
	.poll_out = uart_intel_poll_out,
	.err_check = uart_intel_err_check,
	.read_buffer_polled = uart_intel_read_buffer_polled,
	.configure = uart_intel_configure,
	.config_get = uart_intel_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_intel_fifo_fill,
	.fifo_read = uart_intel_fifo_read,
	.irq_tx_enable = uart_intel_irq_tx_enable,
	.irq_tx_disable = uart_intel_irq_tx_disable,
	.irq_tx_ready = uart_intel_irq_tx_ready,
	.irq_tx_complete = uart_intel_irq_tx_complete,
	.irq_rx_enable = uart_intel_irq_rx_enable,
	.irq_rx_disable = uart_intel_irq_rx_disable,
	.irq_rx_ready = uart_intel_irq_rx_ready,
	.irq_err_enable = uart_intel_irq_err_enable,
	.irq_err_disable = uart_intel_irq_err_disable,
	.irq_is_pending = uart_intel_irq_is_pending,
	.irq_update = uart_intel_irq_update,
	.irq_callback_set = uart_intel_irq_callback_set,
	.enable_unsol_receive = uart_intel_enable_unsol_receive,
	.disable_unsol_receive = uart_intel_disable_unsol_receive,
	.get_unsol_data = uart_intel_get_unsol_data,
	.get_unsol_data_len = uart_intel_get_unsol_data_len,
#endif
#ifdef CONFIG_UART_LINE_CTRL
	.line_ctrl_set = uart_intel_line_ctrl_set,
	.line_ctrl_get = uart_intel_line_ctrl_get,
#endif  /* CONFIG_UART_LINE_CTRL */
#if CONFIG_UART_ASYNC_API
	.callback_set = uart_intel_async_callback_set,
	.tx = uart_intel_write_buffer_async,
	.tx_abort = uart_intel_write_abort_async,
	.rx_enable = uart_intel_read_buffer_async,
	.rx_disable = uart_intel_read_disable_async,
	.rx_buf_rsp = uart_intel_read_buf_rsp,
#endif
#ifdef CONFIG_UART_DRV_CMD
	.drv_cmd = uart_intel_drv_cmd,
#endif  /* CONFIG_UART_DRV_CMD */
#ifdef CONFIG_UART_RS_485
	.rs_485_config_set = uart_intel_rs_485_config_set
#endif
};

/* Helper macro to set flow control. */
#define UART_CONFIG_FLOW_CTRL_SET(n) \
	.hw_fc = DT_INST_PROP(n, hw_flow_control)

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/*  UART IRQ handler declaration.  */
#define  UART_IRQ_HANDLER_DECL(n) \
	static void irq_config_uart_##n(const struct device *dev)

/* Setting configuration function. */
#define UART_CONFIG_IRQ_HANDLER_SET(n) \
	.uart_irq_config_func = irq_config_uart_##n

#define SET_IRQ_FLAG(n) (((DT_INST_IRQ_HAS_CELL(n, sense)) == 0) ? \
			 (DT_INST_IRQ(n, sense)) : 0)

#define UART_IRQ_HANDLER_DEFINE(n)								  \
	static void irq_config_uart_##n(const struct device *dev)				  \
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
						    (void (*)(const void *))uart_intel_isr,	  \
						    DEVICE_DT_INST_GET(n),			  \
						    SET_IRQ_FLAG(n));				  \
				pcie_irq_enable(DT_INST_REG_ADDR(n), irq);			  \
			}									  \
		}										  \
	}

#else /*CONFIG_UART_INTERRUPT_DRIVEN */
#define UART_IRQ_HANDLER_DECL(n)
#define UART_CONFIG_IRQ_HANDLER_SET(n) (0)

#define UART_IRQ_HANDLER_DEFINE(n)
#endif  /* !CONFIG_UART_INTERRUPT_DRIVEN */

/* Device init macro for UART instance. As multiple uart instances follow a
 * similar definition of data structures differing only in the instance
 * number. This macro makes adding instances simpler.
 */

#define UART_INTEL_DEV_CFG(n)							\
	UART_IRQ_HANDLER_DECL(n);						\
	static K_MUTEX_DEFINE(uart_##n##_mutex);				\
	static K_SEM_DEFINE(uart_##n##_tx_sem, 1, 1);				\
	static K_SEM_DEFINE(uart_##n##_rx_sem, 1, 1);				\
	static intel_uart_dma_xfer_t dma_tx_xfer_##n;				\
	static intel_uart_dma_xfer_t dma_rx_xfer_##n;				\
	static intel_uart_transfer_t xfer_tx_##n;				\
	static intel_uart_transfer_t xfer_rx_##n;				\
	static intel_uart_unsol_rx_t unsol_rx_##n;				\
	static const struct uart_intel_config_info config_info_##n  = {		\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				\
		.instance = INTEL_UART_INSTANCE(n),				\
		.baud_rate = DT_INST_PROP(n, current_speed),			\
		.clk_freq = DT_INST_PROP(n, clock_frequency),			\
		UART_CONFIG_FLOW_CTRL_SET(n),					\
		.dma_tx_xfer = &dma_tx_xfer_##n,				\
		.dma_rx_xfer = &dma_rx_xfer_##n,				\
		.xfer_tx = &xfer_tx_##n,					\
		.xfer_rx = &xfer_rx_##n,					\
		.unsol_rx = &unsol_rx_##n,					\
		.dma_rx_chnl = DT_INST_PROP(n, dma_rx_chnl_id),			\
		.dma_tx_chnl = DT_INST_PROP(n, dma_tx_chnl_id),			\
		.mutex = &uart_##n##_mutex,					\
		.tx_sem = &uart_##n##_tx_sem,					\
		.rx_sem = &uart_##n##_rx_sem,					\
		.line_ctrl = LINE_CONTROL,					\
		.pcie = true,							\
		.pcie_id = DT_INST_REG_SIZE(n),					\
		UART_CONFIG_IRQ_HANDLER_SET(n)					\
	};									\
										\
	static struct uart_intel_drv_data drv_data_##n;				\
	DEVICE_DT_INST_DEFINE(n, &uart_pse_init, NULL,				\
			      &drv_data_##n, &config_info_##n,			\
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &api);						\
	UART_IRQ_HANDLER_DEFINE(n)						\


DT_INST_FOREACH_STATUS_OKAY(UART_INTEL_DEV_CFG)
