/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <stdio.h>
#include <drivers/gpio-timed.h>
#include <syscall_handler.h>
#include <intel/hal_tgpio.h>

#define DT_DRV_COMPAT intel_ia_tgpio

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif

#define TGPIO_GET_INSTANCE(dev)	\
	(((const struct tgpio_config *)dev->config)->instance)
#define DEV_CFG(_dev) \
	((const struct tgpio_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct tgpio_runtime *)(_dev)->data)
#define SEM_GET(dev) (&((struct tgpio_runtime *)(dev->data))->sem)

struct tgpio_config {
	intel_instance_t *instance;

	DEVICE_MMIO_NAMED_ROM(reg_base);

	tgpio_irq_config_func_t tgpio_irq_config_func;
	int pin_mux;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

struct tgpio_runtime {
	DEVICE_MMIO_NAMED_RAM(reg_base);
	pcie_bdf_t pcie_bdf;
	struct k_sem sem;
	tgpio_pin_callback_t cb_list[INTEL_GPIO_PIN_NUM];
};

static inline mm_reg_t regs(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, reg_base);
}

void callback(const void *data, uint32_t status)
{
	unsigned int pin = 0;
	uint64_t ec;
	intel_tgpio_time_t ts;
	struct tgpio_time tstamp;
	const struct device *port = (const struct device *)data;

	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);
	struct tgpio_runtime *rt = DEV_DATA(port);

	while (pin < INTEL_GPIO_PIN_NUM) {
		if ((status & BIT(pin)) && rt->cb_list[pin]) {
			intel_tgpio_get_pindata(instance, pin, &ts, &ec);
			tstamp.sec = ts.sec;
			tstamp.nsec = ts.nsec;
			rt->cb_list[pin](port, pin, tstamp, ec);
		}
		pin++;
	}
}

static inline int tgpio_intel_set_time(const struct device *port,
				       enum tgpio_timer timer, uint32_t sec,
				       uint32_t ns)
{
	int status;
	k_timeout_t flag;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_set_time(instance, timer, sec, ns);
	k_sem_give(SEM_GET(port));
	return status;
}

static inline int tgpio_intel_get_time(const struct device *port,
				       enum tgpio_timer timer, uint32_t *sec,
				       uint32_t *ns)
{
	int status;
	k_timeout_t flag;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_get_time(instance, timer, (uint32_t *)sec,
				      (uint32_t *)ns);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_adjust_time(const struct device *port,
					  enum tgpio_timer timer, int32_t nsec)
{
	int status;
	k_timeout_t flag;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}

	status = intel_tgpio_adjust_time(instance, timer, nsec);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_adjust_frequency(const struct device *port,
					       enum tgpio_timer timer,
					       int32_t ppb)
{
	int status;
	k_timeout_t flag;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_adjust_frequency(instance, timer, ppb);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_get_cross_timestamp(const struct device *port,
						  enum tgpio_timer local_clock,
						  enum tgpio_timer ref_clock,
						  struct tgpio_time *local_time,
						  struct tgpio_time *ref_time)
{
	int status;
	k_timeout_t flag;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);
	intel_tgpio_time_t local, reference;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_get_cross_timestamp(instance, local_clock,
						 ref_clock, &local, &reference);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	local_time->nsec = local.nsec;
	local_time->sec = local.sec;
	ref_time->nsec = reference.nsec;
	ref_time->sec = reference.sec;

	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_pin_enable(const struct device *port, uint32_t pin)
{
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	return intel_tgpio_pin_enable(instance, pin);
}

static inline int tgpio_intel_periodic_output(const struct device *port,
					      uint32_t pin,
					      enum tgpio_timer timer,
					      struct tgpio_time *start_time,
					      struct tgpio_time *repeat_interval,
					      struct more_args *margs)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	intel_tgpio_pin_config_t cfg = { 0 };
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = TGPIO_RISING_EDGE;
	cfg.intr_enable = 0;
	cfg.direction = INTEL_OUTPUT;
	cfg.start_time.nsec = start_time->nsec;
	cfg.start_time.sec = start_time->sec;
	cfg.repeat_interval.nsec = repeat_interval->nsec;
	cfg.repeat_interval.sec = repeat_interval->sec;
	cfg.repeat_count = margs->arg0;
	cfg.pulse_width = margs->arg1;
	rt->cb_list[pin] = NULL;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_intel_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_external_timestamp(const struct device *port,
						 uint32_t pin,
						 enum tgpio_timer timer,
						 uint32_t events_ceiling,
						 enum tgpio_pin_polarity edge,
						 tgpio_pin_callback_t cb)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	intel_tgpio_pin_config_t cfg = { 0 };
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = edge;
	cfg.intr_enable = 1;
	cfg.direction = INTEL_INPUT;
	cfg.events_ceiling = events_ceiling;
	rt->cb_list[pin] = cb;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_intel_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_intel_count_events(const struct device *port,
					   uint32_t pin,
					   enum tgpio_timer timer,
					   struct tgpio_time start_time,
					   struct tgpio_time repeat_interval,
					   enum tgpio_pin_polarity edge,
					   tgpio_pin_callback_t cb)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	intel_tgpio_pin_config_t cfg = { 0 };
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = edge;
	cfg.intr_enable = 1;
	cfg.direction = INTEL_INPUT;
	cfg.start_time.nsec = start_time.nsec;
	cfg.start_time.sec = start_time.sec;
	cfg.repeat_interval.nsec = repeat_interval.nsec;
	cfg.repeat_interval.sec = repeat_interval.sec;
	rt->cb_list[pin] = cb;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = intel_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_intel_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));


	return status;
}

static inline int tgpio_intel_pin_disable(const struct device *port,
					  uint32_t pin)
{
	int status;
	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

	status =  intel_tgpio_pin_disable(instance, pin);

	return status;
}

static const struct tgpio_driver_api api_funcs = {
	.pin_disable = tgpio_intel_pin_disable,
	.pin_enable = tgpio_intel_pin_enable,
	.set_time = tgpio_intel_set_time,
	.get_time = tgpio_intel_get_time,
	.adjust_time = tgpio_intel_adjust_time,
	.adjust_frequency = tgpio_intel_adjust_frequency,
	.get_cross_timestamp = tgpio_intel_get_cross_timestamp,
	.set_perout = tgpio_intel_periodic_output,
	.ext_ts = tgpio_intel_external_timestamp,
	.count_events = tgpio_intel_count_events,
};

static int tgpio_init(const struct device *port)
{
	int status;

	k_sem_init(SEM_GET(port), 1, 1);
	const struct tgpio_config *cfg = port->config;

	intel_instance_t *instance = TGPIO_GET_INSTANCE(port);

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (cfg->pcie) {
		struct pcie_mbar mbar;
		struct tgpio_runtime *rt = port->data;

		rt->pcie_bdf = pcie_bdf_lookup(cfg->pcie_id);
		if (!pcie_probe(rt->pcie_bdf, cfg->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(rt->pcie_bdf, 0, &mbar);
		pcie_set_cmd(rt->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(port), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
	} else
#endif
	{
		DEVICE_MMIO_NAMED_MAP(port, reg_base, K_MEM_CACHE_NONE);
	}

	intel_set_base_addr(instance, regs(port));
	intel_set_pin_mux(instance, cfg->pin_mux);
	cfg->tgpio_irq_config_func(port);

	status = intel_tgpio_init(instance, callback, (void *)port);
	if (status != INTEL_DRIVER_OK) {
		return -ENODEV;
	}

	return status;
}

void intel_tgpio_isr(const struct device *port)
{
	intel_instance_t *inst = TGPIO_GET_INSTANCE(port);

	intel_tgpio_isr_handler(inst);
}

/* TGPIO IRQ handler declaration.  */
#define  TGPIO_IRQ_HANDLER_DECL(n) \
	static void irq_config_tgpio_##n(const struct device *dev)

/* Setting configuration function. */
#define TGPIO_CONFIG_IRQ_HANDLER_SET(n)	\
	.tgpio_irq_config_func = irq_config_tgpio_##n

#define TGPIO_INIT_PCIE0(n)
#define TGPIO_INIT_PCIE1(n) \
	.pcie = true,	    \
	.pcie_id = DT_INST_REG_SIZE(n),
#define TGPIO_INIT_PCIE(n) \
	_CONCAT(TGPIO_INIT_PCIE, DT_INST_ON_BUS(n, pcie)) (n)

#define TGPIO_IRQ_FLAGS_SENSE0(n) 0
#define TGPIO_IRQ_FLAGS_SENSE1(n) DT_INST_IRQ(n, sense)
#define TGPIO_IRQ_FLAGS(n) \
	_CONCAT(TGPIO_IRQ_FLAGS_SENSE, DT_INST_IRQ_HAS_CELL(n, sense)) (n)

/* not PCI(e) */
#define TGPIO_IRQ_CONFIG_PCIE0(n)				       \
	static void irq_config_tgpio_##n(const struct device *port)    \
	{							       \
		ARG_UNUSED(port);				       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    intel_tgpio_isr,			       \
			    DEVICE_DT_INST_GET(n),		       \
			    TGPIO_IRQ_FLAGS(n));		       \
		irq_enable(DT_INST_IRQN(n));			       \
	}

/* PCI(e) with auto IRQ detection */
#define TGPIO_IRQ_CONFIG_PCIE1(n)					     \
	static void irq_config_tgpio_##n(const struct device *port)	     \
	{								     \
		ARG_UNUSED(port);					     \
		BUILD_ASSERT(DT_INST_IRQN(n) == PCIE_IRQ_DETECT,	     \
			     "Only runtime IRQ configuration is supported"); \
		BUILD_ASSERT(IS_ENABLED(CONFIG_DYNAMIC_INTERRUPTS),	     \
			     "TGPIO PCI needs CONFIG_DYNAMIC_INTERRUPTS");   \
		unsigned int irq = pcie_alloc_irq(DT_INST_REG_ADDR(n));	     \
		if (irq == PCIE_CONF_INTR_IRQ_NONE) {			     \
			return;						     \
		}							     \
		irq_connect_dynamic(irq, DT_INST_IRQ(n, priority),	     \
				    (void (*)(const void *))		     \
				    intel_tgpio_isr,			     \
				    DEVICE_DT_INST_GET(n),		     \
				    TGPIO_IRQ_FLAGS(n));		     \
		pcie_irq_enable(DT_INST_REG_ADDR(n), irq);		     \
	}

#define TGPIO_IRQ_CONFIG(n) \
	_CONCAT(TGPIO_IRQ_CONFIG_PCIE, DT_INST_ON_BUS(n, pcie)) (n)

#define TGPIO_INTEL_DEV_CFG_DATA(n)					       \
	TGPIO_IRQ_HANDLER_DECL(n);					       \
	static const struct tgpio_config				       \
		tgpio_##n##_cfg = {					       \
		.instance = INTEL_TGPIO_INSTANCE(),			       \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),	       \
		TGPIO_CONFIG_IRQ_HANDLER_SET(n),			       \
		.pin_mux = DT_INST_PROP(n, pin_mux),			       \
		TGPIO_INIT_PCIE(n)					       \
	};								       \
									       \
	static struct tgpio_runtime tgpio_##n##_runtime;		       \
									       \
	DEVICE_DT_INST_DEFINE(n,					       \
			      &tgpio_init,				       \
			      NULL,					       \
			      &tgpio_##n##_runtime,			       \
			      &tgpio_##n##_cfg,				       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &api_funcs);				       \
	TGPIO_IRQ_CONFIG(n)						       \

DT_INST_FOREACH_STATUS_OKAY(TGPIO_INTEL_DEV_CFG_DATA)
