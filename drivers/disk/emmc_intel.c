/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_ia_emmc

#include <cache.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/disk.h>
#include <init.h>
#include <intel/hal_emmc.h>
#include <kernel.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(emmc, CONFIG_EMMC_LOG_LEVEL);

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
BUILD_ASSERT(IS_ENABLED(CONFIG_PCIE), "DT need CONFIG_PCIE");
#include <drivers/pcie/pcie.h>
#endif

struct emmc_config {
	intel_instance_t *instance;

	DEVICE_MMIO_NAMED_ROM(reg_base);
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	bool pcie;
	pcie_id_t pcie_id;
#endif
};

struct emmc_priv_data {
	DEVICE_MMIO_RAM;
	pcie_bdf_t pcie_bdf;
	uint8_t status;
};

#define EMMC_GET_INSTANCE(dev) \
	(((const struct emmc_config *)dev->config)->instance)

static struct k_spinlock disk_io_lock;

static int disk_emmc_intel_access_ioctl(struct disk_info *disk,
					uint8_t cmd, void *buf)
{
	const struct device *dev = disk->dev;
	intel_instance_t *instance = EMMC_GET_INSTANCE(dev);
	uint32_t card_info;
	uint8_t bus_width;

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		card_info = intel_emmc_get_card_info(instance,
						     EMMC_CARD_SECTOR_COUNT);
		*(uint32_t *)buf = card_info;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		card_info = intel_emmc_get_card_info(instance,
						     EMMC_CARD_BLOCK_SIZE);
		*(uint32_t *)buf = card_info;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		card_info = intel_emmc_get_card_info(instance,
						     EMMC_CARD_BLOCK_SIZE);
		*(uint32_t *)buf = card_info;
		break;
	case DISK_IOCTL_LINE_CTRL:
		bus_width = *(uint8_t *)buf;
		__ASSERT(bus_width == 1 || bus_width == 4 || bus_width == 8,
				"Invalid bus width");
		intel_emmc_set_bus_width(instance, bus_width, true);
		break;
	case DISK_IOCTL_CTRL_SYNC:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int disk_emmc_intel_access_write(struct disk_info *disk,
					const uint8_t *buf, uint32_t sector,
					uint32_t count)
{
	__ASSERT(buf != NULL, "");

	const struct device *dev = disk->dev;
	intel_instance_t *instance = EMMC_GET_INSTANCE(dev);
	int ret;

	k_spinlock_key_t key = k_spin_lock(&disk_io_lock);

#if defined(CONFIG_CACHE_MANAGEMENT)
	arch_dcache_range((uint8_t *)buf, sizeof(buf), K_CACHE_WB);
#endif

	ret = intel_emmc_write(instance, (uint8_t *)buf, sector, count);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("eMMC write block failed :%d", ret);
		return -EIO;
	}

	k_spin_unlock(&disk_io_lock, key);

	return 0;
}

static int disk_emmc_intel_access_read(struct disk_info *disk, uint8_t *buf,
				       uint32_t sector, uint32_t count)
{
	__ASSERT(buf != NULL, "");

	const struct device *dev = disk->dev;
	intel_instance_t *instance = EMMC_GET_INSTANCE(dev);
	int ret;

	k_spinlock_key_t key = k_spin_lock(&disk_io_lock);

#if defined(CONFIG_CACHE_MANAGEMENT)
	arch_dcache_range(buf, sizeof(buf), K_CACHE_INVD);
#endif

	ret = intel_emmc_read(instance, buf, sector, count);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("eMMC read block failed :%d", ret);
		return -EIO;
	}

	k_spin_unlock(&disk_io_lock, key);

	return 0;
}

static int disk_emmc_intel_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct emmc_priv_data *emmc_priv = dev->data;

	return emmc_priv->status;
}

static int disk_emmc_intel_access_init(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	intel_instance_t *instance = EMMC_GET_INSTANCE(dev);
	struct emmc_priv_data *emmc_priv = dev->data;
	int ret;

	if (emmc_priv->status == DISK_STATUS_OK) {
		return 0;
	}

	if (emmc_priv->status == DISK_STATUS_NOMEDIA) {
		return -ENODEV;
	}

	ret = intel_emmc_init_card(instance);
	if (ret != INTEL_DRIVER_OK) {
		LOG_ERR("Failed to initialize eMMC card.");
		return -EIO;
	}

	emmc_priv->status = DISK_STATUS_OK;
	return 0;
}

static const struct disk_operations intel_emmc_disk_ops = {
	.init = disk_emmc_intel_access_init,
	.status = disk_emmc_intel_access_status,
	.read = disk_emmc_intel_access_read,
	.write = disk_emmc_intel_access_write,
	.ioctl = disk_emmc_intel_access_ioctl,
};

static struct disk_info intel_emmc_disk = {
	.name = CONFIG_EMMC_VOLUME_NAME,
	.ops = &intel_emmc_disk_ops,
};

static int disk_emmc_intel_init(const struct device *dev)
{
	struct emmc_priv_data *priv = dev->data;
	const struct emmc_config *config = dev->config;

	priv->status = DISK_STATUS_UNINIT;
	intel_emmc_disk.dev = dev;
	intel_instance_t *instance = EMMC_GET_INSTANCE(dev);

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)
	if (config->pcie) {
		struct pcie_mbar mbar;

		priv->pcie_bdf = pcie_bdf_lookup(config->pcie_id);

		if (!pcie_probe(priv->pcie_bdf, config->pcie_id)) {
			return -EINVAL;
		}

		pcie_get_mbar(priv->pcie_bdf, 0, &mbar);
		pcie_set_cmd(priv->pcie_bdf, PCIE_CONF_CMDSTAT_MEM, true);
		device_map(DEVICE_MMIO_RAM_PTR(dev), mbar.phys_addr,
			   mbar.size, K_MEM_CACHE_NONE);
		pcie_set_cmd(priv->pcie_bdf, PCIE_CONF_CMDSTAT_MASTER, true);
	} else
#endif
	{
		DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	}
	intel_set_base_addr(instance, DEVICE_MMIO_GET(dev));

	return disk_access_register(&intel_emmc_disk);
}

#define EMMC_INTEL_DEV_CFG(n)					      \
	static const struct emmc_config				      \
		emmc_config_data_##n = {			      \
		.instance = INTEL_EMMC_INSTANCE(),		      \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)), \
		.pcie = true,					      \
		.pcie_id = DT_INST_REG_SIZE(n)			      \
	};							      \
								      \
	static struct emmc_priv_data emmc_priv_data_##n;	      \
								      \
	DEVICE_DT_INST_DEFINE(n,				      \
			      disk_emmc_intel_init,		      \
			      NULL,				      \
			      &emmc_priv_data_##n,		      \
			      &emmc_config_data_##n,		      \
			      POST_KERNEL,			      \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,     \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(EMMC_INTEL_DEV_CFG)
