/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _IPC_H_
#define _IPC_H_

#include <kernel.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef ipc_new_msg_f
 * @brief Callback  when ipc receiving a new message
 */
typedef int (*ipc_new_msg_f)(const struct device *dev, void *args);

/**
 * @typedef ipc_set_rx_notify_f
 * @brief Callback  when registering a callback
 */
typedef int (*ipc_set_rx_notify_f)(const struct device *dev, ipc_new_msg_f cb);

/**
 * @typedef ipc_read_drbl_f
 * @brief CallbackAPI upon reading an IPC doorbell
 */
typedef int (*ipc_read_drbl_f)(const struct device *dev, uint32_t *drbl);

/**
 * @typedef ipc_read_msg_f
 * @brief CallbackAPI upon reading an IPC message content
 */
typedef int (*ipc_read_msg_f)(const struct device *dev,
			      uint32_t *drbl,
			      uint8_t *msg,
			      uint32_t msg_size);

/**
 * @typedef ipc_send_ack_f
 * @brief  CallbackAPI upon sending an ack to peer after consuming the incoming
 * message
 */
typedef int (*ipc_send_ack_f)(const struct device *dev,
			      uint32_t ack_drbl,
			      uint8_t *ack_msg,
			      uint32_t ack_msg_size);

/**
 * @typedef ipc_write_msg_f
 * @brief Callback API upon sending new message to peer via IPC
 */
typedef int (*ipc_write_msg_f)(const struct device *dev,
			       uint32_t drbl,
			       uint8_t *msg,
			       uint32_t msg_size,
			       uint32_t *ack_drbl,
			       uint8_t *ack_msg,
			       uint32_t ack_msg_size);
/**
 * @typedef ipc_write_msg_f
 * @brief Callback API upon sending msg to peer via IPC
 */
typedef int (*ipc_write_msg_with_timeout_f)(const struct device *dev,
					    uint32_t drbl,
					    uint8_t *msg,
					    uint32_t msg_size,
					    uint32_t *ack_drbl,
					    uint8_t *ack_msg,
					    uint32_t ack_msg_size,
					    int32_t timeout_ms);

__subsystem struct ipc_driver_api {
	ipc_set_rx_notify_f ipc_set_rx_notify;
	ipc_read_drbl_f ipc_read_drbl;
	ipc_read_msg_f ipc_read_msg;
	ipc_send_ack_f ipc_send_ack;
	ipc_write_msg_f ipc_write_msg;
	ipc_write_msg_with_timeout_f ipc_write_msg_with_timeout;
};

static inline int ipc_set_rx_notify(const struct device *dev,
				    ipc_new_msg_f cb)
{
	const struct ipc_driver_api *api = dev->api;

	return api->ipc_set_rx_notify(dev, cb);
}

__syscall int ipc_read_drbl(const struct device *dev, uint32_t *drbl);

static inline int z_impl_ipc_read_drbl(const struct device *dev, uint32_t *drbl)
{
	const struct ipc_driver_api *api =
		(const struct ipc_driver_api *)dev->api;

	if (api->ipc_read_drbl) {
		return api->ipc_read_drbl(dev, drbl);
	}
	return -ENOTSUP;
}

__syscall int ipc_read_msg(const struct device *dev,
			   uint32_t *drbl,
			   uint8_t *msg,
			   uint32_t msg_size);

static inline int z_impl_ipc_read_msg(const struct device *dev,
				      uint32_t *drbl,
				      uint8_t *msg,
				      uint32_t msg_size)
{
	const struct ipc_driver_api *api =
		(const struct ipc_driver_api *)dev->api;

	if (api->ipc_read_msg) {
		return api->ipc_read_msg(dev, drbl, msg, msg_size);
	}
	return -ENOTSUP;
}

__syscall int ipc_send_ack(const struct device *dev,
			   uint32_t ack_drbl,
			   uint8_t *ack_msg,
			   uint32_t ack_msg_size);

static inline int z_impl_ipc_send_ack(const struct device *dev,
				      uint32_t ack_drbl,
				      uint8_t *ack_msg,
				      uint32_t ack_msg_size)
{
	const struct ipc_driver_api *api =
		(const struct ipc_driver_api *)dev->api;

	if (api->ipc_send_ack) {
		return api->ipc_send_ack(dev, ack_drbl, ack_msg, ack_msg_size);
	}
	return -ENOTSUP;
}

__syscall int ipc_write_msg(const struct device *dev,
			    uint32_t drbl,
			    uint8_t *msg,
			    uint32_t msg_size,
			    uint32_t *ack_drbl,
			    uint8_t *ack_msg,
			    uint32_t ack_msg_size);

static inline int z_impl_ipc_write_msg(const struct device *dev,
				       uint32_t drbl,
				       uint8_t *msg,
				       uint32_t msg_size,
				       uint32_t *ack_drbl,
				       uint8_t *ack_msg,
				       uint32_t ack_msg_size)
{
	const struct ipc_driver_api *api =
		(const struct ipc_driver_api *)dev->api;

	if (api->ipc_write_msg) {
		return api->ipc_write_msg(dev, drbl, msg, msg_size, ack_drbl,
					  ack_msg, ack_msg_size);
	}
	return -ENOTSUP;
}

#ifdef __cplusplus
}
#endif

#include <syscalls/ipc.h>

#endif /* _IPC_H_ */
