/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_ia_dma

#include <cache.h>
#include <errno.h>
#include <stdio.h>
#include <kernel.h>
#include <device.h>
#include <string.h>
#include <init.h>
#include <drivers/dma.h>
#include <devicetree.h>

#include <intel/hal_dma.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dma_intel, CONFIG_DMA_LOG_LEVEL);

#define LL_SLAB_COUNT CONFIG_LL_SLAB_COUNT
#define LL_SLAB_ALIGN 8
#define BLK_SLAB_COUNT CONFIG_BLOCK_SLAB_COUNT
#define BLK_SLAB_ALIGN 32
#define GET_MSB(data64) ((uint32_t)(data64 >> 32))
#define GET_LSB(data64) ((uint32_t)(data64))

#if SIZE_MAX == UINT32_MAX
#define UINT64_32(x) (uint32_t)x
#elif SIZE_MAX == UINT64_MAX
#define UINT64_32(x) (uint64_t)x
#endif

K_MEM_SLAB_DEFINE(ll_mem_slab, sizeof(dma_linked_list_item_t), LL_SLAB_COUNT,
		  LL_SLAB_ALIGN);
K_MEM_SLAB_DEFINE(blk_mem_slab, sizeof(struct dma_block_config), BLK_SLAB_COUNT,
		  BLK_SLAB_ALIGN);

static int free_ll_nodes_slab(dma_linked_list_item_t **ll_p_h);
static int free_blk_nodes_slab(struct dma_block_config *p);

struct dma_intel_config_info {
	intel_instance_t *instance; /* Controller instance. */
	intel_dma_t id;
	void (*irq_config)(void);

	DEVICE_MMIO_NAMED_ROM(reg_base);
};

struct dma_intel_driver_data {
	struct k_sem sema[DMA_CHANNEL_NUM];
	struct dma_config dma_configs[DMA_CHANNEL_NUM];
	dma_linked_list_item_t *ll_header[DMA_CHANNEL_NUM];

	DEVICE_MMIO_NAMED_RAM(reg_base);
};

#define DMA_GET_INSTANCE(dev) \
	(((const struct dma_intel_config_info *)dev->config)->instance)
#define DEV_NAME(dev) ((dev)->name)
#define DEV_DATA(dev) ((struct dma_intel_driver_data *const)(dev)->data)
#define DEV_CFG(dev) \
	((const struct dma_intel_config_info *const)(dev)->config)


void dma_isr(const struct device *dev)
{
	intel_instance_t *inst = DMA_GET_INSTANCE(dev);

	intel_dma_isr_handler(inst);
}

static inline mm_reg_t regs(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, reg_base);
}

static inline int dma_source_gather_en(struct dma_block_config *block,
				       uint32_t src, uint32_t dst)
{
	int ret = 0;

	switch (block->source_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		src += block->source_gather_interval;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		src -= block->source_gather_interval;
		break;
	default:
		ret = -1;
		return ret;
	}

	switch (block->dest_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		dst += block->block_size;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		dst -= block->block_size;
		break;
	default:
		ret = -1;
		return ret;
	}
	return 0;
}

static inline int dma_dest_scatter_en(struct dma_block_config *block,
				      uint32_t src, uint32_t dst)
{
	int ret = 0;

	switch (block->dest_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		dst += block->dest_scatter_interval;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		dst -= block->dest_scatter_interval;
		break;
	default:
		ret = -1;
		return ret;
	}

	switch (block->source_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		src += block->block_size;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		src -= block->block_size;
		break;
	default:
		ret = -1;
		return ret;
	}
	return 0;
}

static int dma_scatter_gather_en(struct dma_config *config,
				 struct dma_block_config *block, uint8_t dir,
				 dma_linked_list_item_t *ll_p_h, uint32_t src_w,
				 uint32_t dst_w, uint32_t src_b, uint32_t dst_b)
{
	int ret = 0, i, ll_size = 0;
	uint32_t ctrl_low, src, dst;

	src = GET_LSB(block->source_address);
	dst = GET_LSB(block->dest_address);
	for (i = 0; i < config->block_count; i++) {
		ll_size += sizeof(dma_linked_list_item_t);
		ctrl_low = BUILD_LL_CTRL_REG(dir,
					     src_b, dst_b, src_w, src_w,
					     block->source_addr_adj,
					     block->dest_addr_adj);
		dma_fill_linkedlist(ll_p_h, src, dst, block->block_size,
				    ctrl_low, ll_p_h->next_ll_p);
		LOG_DBG("filling linked list [%d] : %p", i, ll_p_h);
		ll_p_h = (dma_linked_list_item_t *)
			 ((uint64_t)ll_p_h->next_ll_p);
		if (block->source_gather_en) {
			ret = dma_source_gather_en(block, src, dst);
			if (ret) {
				return ret;
			}

			if (block->dest_reload_en) {
				continue;
			}
		}

		if (block->dest_scatter_en) {
			ret = dma_dest_scatter_en(block, src, dst);
			if (ret) {
				return ret;
			}
			if (block->source_reload_en) {
				continue;
			}
		}
	}
	return 0;
}

/*
 * this function will be called when dma transferring is completed
 * or error happened
 */
static void dma_handler(const intel_instance_t *inst, int channel, int event_id,
			void *args)
{
	const struct device *dev = args;
	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);

	ARG_UNUSED(args);

	k_sem_give(&data->sema[channel]);

	free_ll_nodes_slab(&(data->ll_header[channel]));

	/* run user-defined callback */
	if (config->dma_callback) {
		if ((event_id == INTEL_DMA_EVENT_TRANSFER_DONE) &&
		    (config->complete_callback_en)) {
			config->dma_callback(dev, config->user_data,
					     channel, 0);
		} else if (config->error_callback_en) {
			config->dma_callback(dev, config->user_data,
					     channel, event_id);
		}
	}
}

/* map width to certain macros*/
static int width_index(uint32_t num_bytes, uint32_t *index)
{
	switch (num_bytes) {
	case 1:
		*index = DMA_TRANS_WIDTH_8;
		break;
	case 2:
		*index = DMA_TRANS_WIDTH_16;
		break;
	case 4:
		*index = DMA_TRANS_WIDTH_32;
		break;
	case 8:
		*index = DMA_TRANS_WIDTH_64;
		break;
	case 16:
		*index = DMA_TRANS_WIDTH_128;
		break;
	case 32:
		*index = DMA_TRANS_WIDTH_256;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/* map burst size to certain macros*/
static int burst_index(uint32_t num_units, uint32_t *index)
{
	switch (num_units) {
	case 1:
		*index = DMA_BURST_TRANS_LENGTH_1;
		break;
	case 4:
		*index = DMA_BURST_TRANS_LENGTH_4;
		break;
	case 8:
		*index = DMA_BURST_TRANS_LENGTH_8;
		break;
	case 16:
		*index = DMA_BURST_TRANS_LENGTH_16;
		break;
	case 32:
		*index = DMA_BURST_TRANS_LENGTH_32;
		break;
	case 64:
		*index = DMA_BURST_TRANS_LENGTH_64;
		break;
	case 128:
		*index = DMA_BURST_TRANS_LENGTH_128;
		break;
	case 256:
		*index = DMA_BURST_TRANS_LENGTH_256;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static dma_linked_list_item_t *alloc_ll_nodes_slab(int count)
{
	int ret = 0;
	dma_linked_list_item_t *ll_p, *ll_p_h = NULL;

	for (int i = 0; i < count; i++) {
		if (k_is_in_isr()) {
			ret = k_mem_slab_alloc(&ll_mem_slab, (void **)&ll_p,
					       K_NO_WAIT);
		} else {
			ret = k_mem_slab_alloc(&ll_mem_slab,
					       (void **)&ll_p, K_FOREVER);
		}

		if (ret) {
			LOG_ERR("DMA resource is used up, try it later!!!");
			free_ll_nodes_slab(&ll_p);
			return NULL;
		}

		LOG_DBG("linked list [%d] : %p", i, ll_p);
		memset((void *)ll_p, 0x0, sizeof(dma_linked_list_item_t));

		if (UINT64_32(ll_p) >= UINTPTR_MAX) {
			__ASSERT(0, "Does not support 64-bit descriptor address\n");
		}

		ll_p->next_ll_p = (uintptr_t)ll_p_h;
		ll_p_h = ll_p;
	}
	return ll_p_h;
}

static struct dma_block_config *
alloc_and_cpy_blk_nodes_slab(struct dma_block_config *src_blk)
{
	int i = 0, ret = 0;
	struct dma_block_config *blk_p, *blk_p_h = NULL;

	while (src_blk) {
		ret = k_mem_slab_alloc(&blk_mem_slab, (void **)&blk_p, K_FOREVER);
		if (ret) {
			LOG_ERR("Block buffer allocation failed %d", ret);
			free_blk_nodes_slab(blk_p);
			return NULL;
		}

		LOG_DBG("block node [%02d] : %p", i, blk_p);
		memcpy((void *)blk_p, src_blk, sizeof(struct dma_block_config));
		blk_p->next_block = blk_p_h;

		src_blk = src_blk->next_block;
		blk_p_h = blk_p;
		i++;
	}
	return blk_p_h;
}

static int free_ll_nodes_slab(dma_linked_list_item_t **ll_p_h)
{
	int count = 0;
	dma_linked_list_item_t *p = *ll_p_h, *p_next;

	while (p) {
		p_next = (dma_linked_list_item_t *)((uint64_t)p->next_ll_p);
		k_mem_slab_free(&ll_mem_slab, (void **)&p);
		LOG_DBG("linked list [%d] : %p", count, p);
		p = p_next;
		count++;
	}
	*ll_p_h = NULL;
	return count;
}

static int free_blk_nodes_slab(struct dma_block_config *p)
{
	int count = 0;
	struct dma_block_config *p_next;

	while (p) {
		p_next = (struct dma_block_config *)p->next_block;
		k_mem_slab_free(&blk_mem_slab, (void **)&p);
		LOG_DBG("block node [%d] : %p", count, p);
		p = p_next;
		count++;
	}

	return count;
}

static void dma_config_convert(struct dma_config *config, bool *is_src_dram,
			       bool *is_dest_dram, uint8_t *intel_dma_dir)
{
	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case MEMORY_TO_PERIPHERAL:
	case PERIPHERAL_TO_MEMORY:
		*is_src_dram = true;
		*is_dest_dram = true;
		*intel_dma_dir = config->channel_direction;
		break;
	}
}

/* config basic dma */
static int dma_intel_apply_common_config(intel_instance_t *dev, uint32_t channel,
					 struct dma_config *config, uint8_t *dir)
{
	uint8_t direction = MEMORY_TO_MEMORY;
	bool is_src_dram = false, is_dest_dram = false;

	dma_config_convert(config, &is_src_dram, &is_dest_dram, &direction);

	if (dir) {
		*dir = direction;
	}

	/* configure dma transferring direction*/
	intel_dma_control(dev, channel, INTEL_CONFIG_DMA_DIRECTION,
			  direction);

	if (direction == MEMORY_TO_MEMORY) {
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_SR_MEM_TYPE,
				  is_src_dram ? DMA_DRAM_MEM : DMA_SRAM_MEM);
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_DT_MEM_TYPE,
				  is_dest_dram ? DMA_DRAM_MEM : DMA_SRAM_MEM);
	} else if (direction == MEMORY_TO_PERIPHERAL) {
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_HS_DEVICE_ID,
				  config->dma_slot);
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_HS_POLARITY,
				  DMA_HS_POLARITY_HIGH);
		intel_dma_control(dev, channel,
				  INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
				  DMA_HS_PER_TX);
	} else if (direction == PERIPHERAL_TO_MEMORY) {
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_HS_DEVICE_ID,
				  config->dma_slot);
		intel_dma_control(dev, channel, INTEL_CONFIG_DMA_HS_POLARITY,
				  DMA_HS_POLARITY_HIGH);
		intel_dma_control(dev, channel,
				  INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
				  DMA_HS_PER_RX);
	} else {
		return -1;
	}
	return 0;
}

static int dma_intel_apply_ll_config(intel_instance_t *dev, uint32_t channel,
				     struct dma_config *config,
				     dma_linked_list_item_t **ll_p)
{
	int ret = 0, i, ll_size = 0;
	uint8_t dir;
	uint32_t ctrl_low, src, dst;
	uint32_t src_w, dst_w, src_b, dst_b;
	struct dma_block_config *block = config->head_block;

	ret = dma_intel_apply_common_config(dev, channel, config, &dir);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	ret = width_index(config->source_data_size, &src_w);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	ret = width_index(config->dest_data_size, &dst_w);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	ret = burst_index(config->source_burst_length, &src_b);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	ret = burst_index(config->dest_burst_length, &dst_b);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	dma_linked_list_item_t *ll_p_h =
		alloc_ll_nodes_slab(config->block_count);
	if (ll_p_h == NULL) {
		return -1;
	}
	*ll_p = ll_p_h;

	if ((block->source_gather_en == 1) || (block->dest_scatter_en == 1)) {
		/*scatter gather mode*/
		ret = dma_scatter_gather_en(config, block, dir, ll_p_h,
					    src_w, dst_w, src_b, dst_b);
		if (ret) {
			goto INVALID_ARGS;
		}

#if CONFIG_CACHE_MANAGEMENT
		arch_dcache_range((uint32_t *)*ll_p,
				  ll_size + ((uint32_t)(*ll_p)), K_CACHE_WB);
#endif

		return 0;
	}

	/*normal linked list mode */
	for (i = 0; i < config->block_count; i++) {
		ll_size += sizeof(dma_linked_list_item_t);
		src = GET_LSB(block->source_address);
		dst = GET_LSB(block->dest_address);
		ctrl_low = BUILD_LL_CTRL_REG(dir, src_b,
					     dst_b, src_w, src_w,
					     block->source_addr_adj,
					     block->dest_addr_adj);
		dma_fill_linkedlist(ll_p_h, src, dst, block->block_size,
				    ctrl_low, ll_p_h->next_ll_p);
		ll_p_h = (dma_linked_list_item_t *)
			 ((uint64_t)ll_p_h->next_ll_p);
		block = block->next_block;
	}

#if CONFIG_CACHE_MANAGEMENT
	arch_dcache_range((uint32_t *)*ll_p, ll_size + ((uint32_t)(*ll_p)),
			  K_CACHE_WB);
#endif

	return 0;
INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}

static int dma_intel_apply_single_config(intel_instance_t *dev, uint32_t channel,
					 struct dma_config *config)
{
	int ret = 0;
	uint32_t temp;

	ret = dma_intel_apply_common_config(dev, channel, config, NULL);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	/* configurate dma width of source data*/
	ret = width_index(config->source_data_size, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	intel_dma_control(dev, channel, INTEL_CONFIG_DMA_SR_TRANS_WIDTH, temp);

	/* configurate dma width of destination data*/
	ret = width_index(config->dest_data_size, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	intel_dma_control(dev, channel, INTEL_CONFIG_DMA_DT_TRANS_WIDTH, temp);

	/* configurate dma burst size*/
	ret = burst_index(config->source_burst_length, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	intel_dma_control(dev, channel, INTEL_CONFIG_DMA_BURST_LENGTH, temp);
	return 0;

INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}

static int dma_intel_chan_config(const struct device *dev, uint32_t channel,
				 struct dma_config *config)
{
	int ret = -1;

	if ((channel >= DMA_CHANNEL_NUM) || (config == NULL)) {
		goto INVALID_ARGS;
	}

	intel_instance_t *instance = DMA_GET_INSTANCE(dev);
	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	struct dma_config *local_config = &(data->dma_configs[channel]);

	memcpy(local_config, config, sizeof(struct dma_config));
	local_config->head_block =
		alloc_and_cpy_blk_nodes_slab(config->head_block);
	data->ll_header[channel] = NULL;

	/* initialize the dma controller, following the intel api*/
	intel_dma_event_cb_t cb = dma_handler;

	intel_dma_init(instance, (int)channel, cb, (void *)dev);
	return 0;

INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}

static int dma_intel_reload(const struct device *dev, uint32_t channel,
			    uint64_t src, uint64_t dst, size_t size)
{
	if (channel >= DMA_CHANNEL_NUM) {
		LOG_ERR("dma reload failed for invalid args");
		return -EINVAL;
	}

	int ret = 0;
	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);
	struct dma_block_config *block_config = config->head_block;

	if ((config == NULL) || (block_config == NULL)) {
		LOG_ERR("dma reload failed, no config found");
		return -ENOTSUP;
	}
	if ((config->block_count == 1) || (block_config->next_block == NULL)) {
		block_config->source_address = src;
		block_config->dest_address = dst;
		block_config->block_size = size;
	} else {
		LOG_ERR("no reload support for multi-linkedlist mode");
		return -ENOTSUP;
	}
	return ret;
}

static int dma_intel_start(const struct device *dev, uint32_t channel)
{
	if (channel >= DMA_CHANNEL_NUM) {
		LOG_ERR("dma transferring failed for invalid args");
		return -EINVAL;
	}
	int ret;
	dma_linked_list_item_t *ll_p = NULL;
	intel_instance_t *instance = DMA_GET_INSTANCE(dev);
	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);
	struct dma_block_config *block_config = config->head_block;
	uint64_t src_addr, dst_addr;

	if (k_is_in_isr()) {
		ret = k_sem_take(&data->sema[channel], K_NO_WAIT);
		if (ret == -EBUSY) {
			LOG_ERR("dma channel %d is busy!!!", channel);
			return -EBUSY;
		}
	} else {
		k_sem_take(&data->sema[channel], K_FOREVER);
	}

	if (config->block_count == 1) {
		/* call intel start function */
		ret = dma_intel_apply_single_config(instance,
						    channel, config);
		if (ret) {
			goto ERR;
		}
		src_addr = block_config->source_address;
		dst_addr = block_config->dest_address;

		ret = intel_dma_start_transfer(instance, channel,
					       src_addr, dst_addr,
					       block_config->block_size);
	} else {
		ret = dma_intel_apply_ll_config(instance, channel, config,
						&ll_p);
		if (ret) {
			goto ERR;
		}
		LOG_DBG("starting linked list: %p", ll_p);
		data->ll_header[channel] = ll_p;
		ret = intel_dma_start_ll_transfer(instance, channel, ll_p);
	}
	if (ret != INTEL_DRIVER_OK) {
		goto ERR;
	}
	return ret;


ERR:
	LOG_ERR("dma transfer failed");
	k_sem_give(&data->sema[channel]);
	return ret;
}

static int dma_intel_stop(const struct device *dev, uint32_t channel)
{
	if (channel >= DMA_CHANNEL_NUM) {
		LOG_ERR("dma stop failed for invalid args");
		return -EINVAL;
	}

	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);
	intel_instance_t *instance = DMA_GET_INSTANCE(dev);

	LOG_DBG("stoping dma: %p, %d", dev, channel);
	intel_dma_abort_transfer(instance, channel);

	free_ll_nodes_slab(&(data->ll_header[channel]));
	free_blk_nodes_slab(config->head_block);
	config->head_block = NULL;

	return 0;
}

static const struct dma_driver_api dma_intel_api = {
	.config = dma_intel_chan_config,
	.start = dma_intel_start,
	.stop = dma_intel_stop,
	.reload = dma_intel_reload,
	.get_status = NULL
};

static int _impl_dma_intel_init(const struct device *dev)
{
	struct dma_intel_driver_data *const data = DEV_DATA(dev);
	const struct dma_intel_config_info *const dev_cfg = DEV_CFG(dev);
	intel_instance_t *instance = DMA_GET_INSTANCE(dev);

	/* initialize the semaphare for each channel*/
	for (int i = 0; i < DMA_CHANNEL_NUM; i++) {
		k_sem_init(&data->sema[i], 1, 1);
	}
	/*Initial base address in HAL*/
	intel_set_base_addr(instance, regs(dev));
	intel_dma_set_instance(instance, dev_cfg->id);
	return 0;
}

static int dma_intel_init(const struct device *dev)
{
	const struct dma_intel_config_info *const dev_cfg = DEV_CFG(dev);

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);
	_impl_dma_intel_init(dev);
	dev_cfg->irq_config();

	return 0;
}

#define DMA_INTEL_DEV_CFG(n)						       \
	static void dma##n##_irq_config(void);				       \
	static const struct dma_intel_config_info			       \
		dma_intel_config_data_##n = {				       \
		.instance = INTEL_DMA_INSTANCE(),			       \
		.irq_config = dma##n##_irq_config,			       \
		.id = DT_INST_PROP(n, peripheral_id),			       \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),	       \
	};								       \
									       \
	static struct dma_intel_driver_data dma_intel_dev_data_##n;	       \
									       \
	DEVICE_DT_INST_DEFINE(n,					       \
			      dma_intel_init,				       \
			      NULL,					       \
			      &dma_intel_dev_data_##n,			       \
			      &dma_intel_config_data_##n,		       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &dma_intel_api);				       \
	static void dma##n##_irq_config(void)				       \
	{								       \
		IRQ_CONNECT(DT_INST_IRQN(n),				       \
			    DT_INST_IRQ(0, priority), dma_isr,		       \
			    DEVICE_DT_INST_GET(n),			       \
			    DT_INST_IRQ(0, sense));			       \
		irq_enable(DT_INST_IRQN(n));				       \
	}

DT_INST_FOREACH_STATUS_OKAY(DMA_INTEL_DEV_CFG)
