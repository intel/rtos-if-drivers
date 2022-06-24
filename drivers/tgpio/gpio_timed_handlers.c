/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/gpio-timed.h>
#include <syscall_handler.h>

static inline int z_vrfy_tgpio_port_set_time(const struct device *port,
				      enum tgpio_timer timer, uint32_t sec,
				      uint32_t ns) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, set_time));
	return z_impl_tgpio_port_set_time((const struct device *)port,
		(enum tgpio_timer)timer, (uint32_t)sec, (uint32_t)ns);
}
#include <syscalls/tgpio_port_set_time_mrsh.c>

static inline int z_vrfy_tgpio_port_get_time(const struct device *port,
				      enum tgpio_timer timer, uint32_t *sec,
				      uint32_t *ns) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, get_time));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(sec, sizeof(uint32_t)));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(ns, sizeof(uint32_t)));
	return z_impl_tgpio_port_get_time((const struct device *)port, timer,
		(uint32_t *)sec, (uint32_t *)ns);
}
#include <syscalls/tgpio_port_get_time_mrsh.c>

static inline int z_vrfy_tgpio_port_adjust_time(const struct device *port,
					 enum tgpio_timer timer, int32_t nsec) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, adjust_time));
	return z_impl_tgpio_port_adjust_time((const struct device *)port,
		timer, (int32_t)nsec);
}
#include <syscalls/tgpio_port_adjust_time_mrsh.c>

static inline int z_vrfy_tgpio_port_adjust_frequency(const struct device *port,
					      enum tgpio_timer timer,
					      int32_t ppb) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, adjust_frequency));
	return z_impl_tgpio_port_adjust_frequency((const struct device *)port,
		timer, (int32_t)ppb);
}
#include <syscalls/tgpio_port_adjust_frequency_mrsh.c>

static inline int z_vrfy_tgpio_get_cross_timestamp(const struct device *port,
						 enum tgpio_timer local_clock,
						 enum tgpio_timer ref_clock,
						 struct tgpio_time *local_time,
						 struct tgpio_time *ref_time) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, get_cross_timestamp));
	return z_impl_tgpio_get_cross_timestamp((const struct device *)port,
		local_clock, ref_clock, (struct tgpio_time *)local_time,
		(struct tgpio_time *)ref_time);
}
#include <syscalls/tgpio_get_cross_timestamp_mrsh.c>

static inline int z_vrfy_tgpio_pin_periodic_output(const struct device *port,
					     uint32_t pin,
					     enum tgpio_timer timer,
					     struct tgpio_time *start_time,
					     struct tgpio_time *repeat_interval,
					     uint32_t repeat_count, uint32_t pws) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, set_perout));
	return z_impl_tgpio_pin_periodic_output((const struct device *)port,
		pin, timer, (struct tgpio_time *)start_time,
		(struct tgpio_time *)repeat_interval, repeat_count, pws);
}
#include <syscalls/tgpio_pin_periodic_output_mrsh.c>

static inline int z_vrfy_tgpio_pin_disable(const struct device *port,
					 uint32_t pin) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, pin_disable));
	return z_impl_tgpio_pin_disable((const struct device *)port, pin);
}
#include <syscalls/tgpio_pin_disable_mrsh.c>

static inline int z_vrfy_tgpio_pin_enable(const struct device *port,
					uint32_t pin) {
	Z_OOPS(Z_SYSCALL_DRIVER_TGPIO(port, pin_enable));
	return z_impl_tgpio_pin_enable((const struct device *)port, pin);
}
#include <syscalls/tgpio_pin_enable_mrsh.c>
