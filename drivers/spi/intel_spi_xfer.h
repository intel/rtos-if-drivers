/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private API for SPI drivers
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_XFER_H_
#define ZEPHYR_DRIVERS_SPI_SPI_XFER_H_

#include <drivers/gpio.h>
#include <drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

enum spi_xfer_runtime_op_mode {
	SPI_XFER_RUNTIME_OP_MODE_MASTER = BIT(0),
	SPI_XFER_RUNTIME_OP_MODE_SLAVE  = BIT(1),
};

struct intel_spi_xfer {
	const struct spi_config *config;

	struct k_sem lock;
	struct k_sem sync;
	int sync_status;

#ifdef CONFIG_SPI_ASYNC
	struct k_poll_signal *signal;
	bool asynchronous;
#endif /* CONFIG_SPI_ASYNC */
	const struct spi_buf *current_tx;
	size_t tx_count;
	const struct spi_buf *current_rx;
	size_t rx_count;

	const uint8_t *tx_buf;
	size_t tx_len;
	uint8_t *rx_buf;
	size_t rx_len;

#ifdef CONFIG_SPI_SLAVE
	int recv_frames;
#endif /* CONFIG_SPI_SLAVE */
};

#define SPI_XFER_INIT_LOCK(_data, _xfer_name)				\
	._xfer_name.lock = Z_SEM_INITIALIZER(_data._xfer_name.lock, 0, 1)

#define SPI_XFER_INIT_SYNC(_data, _xfer_name)				\
	._xfer_name.sync = Z_SEM_INITIALIZER(_data._xfer_name.sync, 0, 1)

static inline bool spi_xfer_configured(struct intel_spi_xfer *xfer,
					  const struct spi_config *config)
{
	return !!(xfer->config == config);
}

static inline bool spi_xfer_is_slave(struct intel_spi_xfer *xfer)
{
	return (xfer->config->operation & SPI_OP_MODE_SLAVE);
}

static inline void spi_xfer_lock(struct intel_spi_xfer *xfer, bool asynchronous,
				 struct k_poll_signal *signal)
{
	k_sem_take(&xfer->lock, K_FOREVER);

#ifdef CONFIG_SPI_ASYNC
	xfer->asynchronous = asynchronous;
	xfer->signal = signal;
#endif /* CONFIG_SPI_ASYNC */
}

static inline void spi_xfer_release(struct intel_spi_xfer *xfer, int status)
{
#ifdef CONFIG_SPI_SLAVE
	if (status >= 0 && (xfer->config->operation & SPI_LOCK_ON)) {
		return;
	}
#endif /* CONFIG_SPI_SLAVE */

#ifdef CONFIG_SPI_ASYNC
	if (!xfer->asynchronous || (status < 0)) {
		k_sem_give(&xfer->lock);
	}
#else
	k_sem_give(&xfer->lock);
#endif /* CONFIG_SPI_ASYNC */
}

static inline int spi_xfer_wait_for_completion(struct intel_spi_xfer *xfer)
{
	int status = 0;
#ifdef CONFIG_SPI_ASYNC
	if (!xfer->asynchronous) {
		k_sem_take(&xfer->sync, K_FOREVER);
		status = xfer->sync_status;
	}
#else
	k_sem_take(&xfer->sync, K_FOREVER);
	status = xfer->sync_status;
#endif /* CONFIG_SPI_ASYNC */

#ifdef CONFIG_SPI_SLAVE
	if (spi_xfer_is_slave(xfer) && !status) {
		return xfer->recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

	return status;
}

static inline void spi_xfer_complete(struct intel_spi_xfer *xfer, int status)
{
#ifdef CONFIG_SPI_ASYNC
	if (!xfer->asynchronous) {
		xfer->sync_status = status;
		k_sem_give(&xfer->sync);
	} else {
		if (xfer->signal) {
#ifdef CONFIG_SPI_SLAVE
			if (spi_xfer_is_slave(xfer) && !status) {
				/* Let's update the status so it tells
				 * about number of received frames.
				 */
				status = xfer->recv_frames;
			}
#endif /* CONFIG_SPI_SLAVE */
			k_poll_signal_raise(xfer->signal, status);
		}

		if (!(xfer->config->operation & SPI_LOCK_ON)) {
			k_sem_give(&xfer->lock);
		}
	}
#else
	xfer->sync_status = status;
	k_sem_give(&xfer->sync);
#endif /* CONFIG_SPI_ASYNC */
}

static inline int spi_xfer_cs_active_value(struct intel_spi_xfer *xfer)
{
	if (xfer->config->operation & SPI_CS_ACTIVE_HIGH) {
		return 1;
	}

	return 0;
}

static inline int spi_xfer_cs_inactive_value(struct intel_spi_xfer *xfer)
{
	if (xfer->config->operation & SPI_CS_ACTIVE_HIGH) {
		return 0;
	}

	return 1;
}

static inline void spi_xfer_cs_configure(struct intel_spi_xfer *xfer)
{
	if (xfer->config->cs && xfer->config->cs->gpio_dev) {
		gpio_pin_configure(xfer->config->cs->gpio_dev,
				   xfer->config->cs->gpio_pin, GPIO_OUTPUT);
		gpio_pin_set(xfer->config->cs->gpio_dev,
			     xfer->config->cs->gpio_pin,
			     spi_xfer_cs_inactive_value(xfer));
	} else {
		LOG_INF("CS control inhibited (no GPIO device)");
	}
}

static inline void _spi_xfer_cs_control(struct intel_spi_xfer *xfer,
					bool on, bool force_off)
{
	if (xfer->config && xfer->config->cs && xfer->config->cs->gpio_dev) {
		if (on) {
			gpio_pin_set(xfer->config->cs->gpio_dev,
				     xfer->config->cs->gpio_pin,
				     spi_xfer_cs_active_value(xfer));
			k_busy_wait(xfer->config->cs->delay);
		} else {
			if (!force_off &&
			    xfer->config->operation & SPI_HOLD_ON_CS) {
				return;
			}

			k_busy_wait(xfer->config->cs->delay);
			gpio_pin_set(xfer->config->cs->gpio_dev,
				     xfer->config->cs->gpio_pin,
				     spi_xfer_cs_inactive_value(xfer));
		}
	}
}

static inline void spi_xfer_cs_control(struct intel_spi_xfer *xfer, bool on)
{
	_spi_xfer_cs_control(xfer, on, false);
}

static inline void spi_xfer_unlock_unconditionally(struct intel_spi_xfer *xfer)
{
	/* Forcing CS to go to inactive status */
	_spi_xfer_cs_control(xfer, false, true);

	if (!k_sem_count_get(&xfer->lock)) {
		k_sem_give(&xfer->lock);
	}
}

static inline
void spi_xfer_buffers_setup(struct intel_spi_xfer *xfer,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs,
			    uint8_t dfs)
{
	LOG_DBG("tx_bufs %p - rx_bufs %p - %u", tx_bufs, rx_bufs, dfs);

	if (tx_bufs) {
		xfer->current_tx = tx_bufs->buffers;
		xfer->tx_count = tx_bufs->count;
		xfer->tx_buf = (const uint8_t *)xfer->current_tx->buf;
		xfer->tx_len = xfer->current_tx->len / dfs;
	} else {
		xfer->current_tx = NULL;
		xfer->tx_count = 0;
		xfer->tx_buf = NULL;
		xfer->tx_len = 0;
	}

	if (rx_bufs) {
		xfer->current_rx = rx_bufs->buffers;
		xfer->rx_count = rx_bufs->count;
		xfer->rx_buf = (uint8_t *)xfer->current_rx->buf;
		xfer->rx_len = xfer->current_rx->len / dfs;
	} else {
		xfer->current_rx = NULL;
		xfer->rx_count = 0;
		xfer->rx_buf = NULL;
		xfer->rx_len = 0;
	}

	xfer->sync_status = 0;

#ifdef CONFIG_SPI_SLAVE
	xfer->recv_frames = 0;
#endif /* CONFIG_SPI_SLAVE */

	LOG_DBG("current_tx %p (%zu), current_rx %p (%zu),"
		" tx buf/len %p/%zu, rx buf/len %p/%zu",
		xfer->current_tx, xfer->tx_count,
		xfer->current_rx, xfer->rx_count,
		xfer->tx_buf, xfer->tx_len, xfer->rx_buf, xfer->rx_len);
}

static ALWAYS_INLINE
void spi_xfer_update_tx(struct intel_spi_xfer *xfer, uint8_t dfs, uint32_t len)
{
	if (!xfer->tx_len) {
		return;
	}

	if (len > xfer->tx_len) {
		LOG_ERR("Update exceeds current buffer");
		return;
	}

	xfer->tx_len -= len;
	if (!xfer->tx_len) {
		xfer->tx_count--;
		if (xfer->tx_count) {
			xfer->current_tx++;
			xfer->tx_buf = (const uint8_t *)xfer->current_tx->buf;
			xfer->tx_len = xfer->current_tx->len / dfs;
		} else {
			xfer->tx_buf = NULL;
		}
	} else if (xfer->tx_buf) {
		xfer->tx_buf += dfs * len;
	}

	LOG_DBG("tx buf/len %p/%zu", xfer->tx_buf, xfer->tx_len);
}

static ALWAYS_INLINE
bool spi_xfer_tx_on(struct intel_spi_xfer *xfer)
{
	return !!(xfer->tx_len);
}

static ALWAYS_INLINE
bool spi_xfer_tx_buf_on(struct intel_spi_xfer *xfer)
{
	return !!(xfer->tx_buf && xfer->tx_len);
}

static ALWAYS_INLINE
void spi_xfer_update_rx(struct intel_spi_xfer *xfer, uint8_t dfs, uint32_t len)
{
#ifdef CONFIG_SPI_SLAVE
	if (spi_xfer_is_slave(xfer)) {
		xfer->recv_frames += len;
	}

#endif /* CONFIG_SPI_SLAVE */

	if (!xfer->rx_len) {
		return;
	}

	if (len > xfer->rx_len) {
		LOG_ERR("Update exceeds current buffer");
		return;
	}

	xfer->rx_len -= len;
	if (!xfer->rx_len) {
		xfer->rx_count--;
		if (xfer->rx_count) {
			xfer->current_rx++;
			xfer->rx_buf = (uint8_t *)xfer->current_rx->buf;
			xfer->rx_len = xfer->current_rx->len / dfs;
		} else {
			xfer->rx_buf = NULL;
		}
	} else if (xfer->rx_buf) {
		xfer->rx_buf += dfs * len;
	}

	LOG_DBG("rx buf/len %p/%zu", xfer->rx_buf, xfer->rx_len);
}

static ALWAYS_INLINE
bool spi_xfer_rx_on(struct intel_spi_xfer *xfer)
{
	return !!(xfer->rx_len);
}

static ALWAYS_INLINE
bool spi_xfer_rx_buf_on(struct intel_spi_xfer *xfer)
{
	return !!(xfer->rx_buf && xfer->rx_len);
}

static inline size_t spi_xfer_longest_current_buf(struct intel_spi_xfer *xfer)
{
	if (!xfer->tx_len) {
		return xfer->rx_len;
	} else if (!xfer->rx_len) {
		return xfer->tx_len;
	} else if (xfer->tx_len < xfer->rx_len) {
		return xfer->tx_len;
	}

	return xfer->rx_len;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SPI_SPI_XFER_H_ */
