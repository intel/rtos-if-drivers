/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_ia_pwm

#include <errno.h>

#include <drivers/pwm.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <sys/util.h>
#include <spinlock.h>
#include "intel/hal_pwm.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif

/* Convenient macro to get the controller instance. */
#define PWM_GET_INSTANCE(dev) \
	(((const struct pwm_intel_config *)dev->config)->instance)

#define GET_PWM_MUTEX(dev) (&((struct pwm_runtime *)(dev->data))->pwm_mutex)

#define MIN_LOW_PERIOD  (1)

struct pwm_intel_config {
	DEVICE_MMIO_ROM;
	intel_instance_t *instance;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

struct pwm_runtime {
	DEVICE_MMIO_RAM;
	pcie_bdf_t pcie_bdf;
	struct k_spinlock s_lock;
};

void pwm_isr(const struct device *dev)
{
	intel_instance_t *instance = PWM_GET_INSTANCE(dev);

	intel_pwm_isr_handler(instance);
}

static int pwm_intel_pin_stop(const struct device *dev, uint32_t pwm)
{
	intel_instance_t *instance = PWM_GET_INSTANCE(dev);

	__ASSERT(pwm < INTEL_PWM_ID_NUM, "");

	intel_pwm_stop(instance, pwm);

	return 0;
}

static int pwm_intel_pin_start(const struct device *dev, uint32_t pwm)
{
	intel_instance_t *instance = PWM_GET_INSTANCE(dev);

	__ASSERT(pwm < INTEL_PWM_ID_NUM, "");

	intel_pwm_start(instance, pwm);

	return 0;
}

static int pwm_intel_pin_set(const struct device *dev, uint32_t pwm,
			     uint32_t period_cycles, uint32_t pulse_cycles,
			     pwm_flags_t flags)
{
	ARG_UNUSED(flags);
	uint32_t ret, high, low, max = UINT32_MAX;
	intel_pwm_config_t cfg = { 0 };
	intel_instance_t *instance = PWM_GET_INSTANCE(dev);
	struct pwm_runtime *data = dev->data;

	__ASSERT(pwm < INTEL_PWM_ID_NUM, "");

	if ((period_cycles == 0) ||
	    (pulse_cycles > period_cycles) ||
	    (period_cycles > max) || (pulse_cycles > max)) {
		return -EINVAL;
	}

	if (pulse_cycles == 0) {
		pwm_intel_pin_stop(dev, pwm);
		return 0;
	}

	high = pulse_cycles;
	low = period_cycles - pulse_cycles;

	/*
	 * low must be more than zero. Otherwise, the PWM pin will be
	 * turned off. Let's make sure low is always more than zero.
	 */
	if (low == 0) {
		high--;
		low = MIN_LOW_PERIOD;
	}

	cfg.mode = INTEL_PWM_MODE_PWM;
	cfg.intr_enable = false;
	cfg.hi_count = high;
	cfg.lo_count = low;

	k_spinlock_key_t key = k_spin_lock(&data->s_lock);

	pwm_intel_pin_stop(dev, pwm);
	ret = intel_pwm_set_config(instance, pwm, cfg);
	if (!ret) {
		pwm_intel_pin_start(dev, pwm);
	}

	k_spin_unlock(&data->s_lock, key);

	return ret;
}

static int pwm_intel_get_cycles_per_sec(const struct device *dev, uint32_t pwm,
					uint64_t *cycles)
{
	if (cycles == NULL) {
		return -EINVAL;
	}

	*cycles = (uint64_t)(100 * 1000000UL);

	return 0;
}

static const struct pwm_driver_api api_funcs = {
	.pin_set = pwm_intel_pin_set,
	.get_cycles_per_sec = pwm_intel_get_cycles_per_sec,
};

static int pwm_init(const struct device *dev)
{
	int ret;
	intel_instance_t *instance = PWM_GET_INSTANCE(dev);
	const struct pwm_intel_config *config = dev->config;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (config->pcie) {
		struct pcie_mbar mbar;
		struct pwm_runtime *data = dev->data;

		data->pcie_bdf = pcie_bdf_lookup(config->pcie_id);
		if (!pcie_probe(data->pcie_bdf, config->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(data->pcie_bdf, 0, &mbar);
		pcie_set_cmd(data->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
		pcie_set_cmd(data->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);
	} else
#endif
	{
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}

	intel_set_base_addr(instance, DEVICE_MMIO_GET(dev));
	ret = intel_pwm_init(instance, NULL, NULL);
	if (ret != INTEL_DRIVER_OK) {
		return -ENODEV;
	}

	return 0;
}

#define PWM_INTEL_DEV_CFG(n)						       \
	static const struct pwm_intel_config pwm_cfg_##n = {		       \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			       \
		.instance = INTEL_PWM_INSTANCE(n),			       \
		.pcie = true,						       \
		.pcie_id = DT_INST_REG_SIZE(n),				       \
	};								       \
									       \
	static struct pwm_runtime pwm_runtime_##n;			       \
	DEVICE_DT_INST_DEFINE(n, &pwm_init, NULL,			       \
			      &pwm_runtime_##n, &pwm_cfg_##n,		       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &api_funcs);				       \

DT_INST_FOREACH_STATUS_OKAY(PWM_INTEL_DEV_CFG)
