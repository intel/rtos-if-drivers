/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME eth_phy_framework
#ifdef CONFIG_ETH_PHY_DEBUG
#define LOG_LEVEL 4
#else
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <kernel.h>
#include <stdbool.h>
#include <drivers/ethernet/phy_ehl.h>

#ifdef CONFIG_ETH_PHY_USE_C22
int phy_write_c22(struct phy_device *phy, uint8_t regnum, uint16_t val)
{
	union mdio_frm_flds frm_fld;
	int retval;

	frm_fld.c22_frm_fld.phyaddr = phy->addr;
	frm_fld.c22_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = phy->mdio_write(phy, frm_fld, val, C22);
	k_sem_give(&phy->phy_lock);

	return retval;
}

int phy_read_c22(struct phy_device *phy, uint8_t regnum, uint16_t *val)
{
	union mdio_frm_flds frm_fld;
	int retval;

	frm_fld.c22_frm_fld.phyaddr = phy->addr;
	frm_fld.c22_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = phy->mdio_read(phy, frm_fld, val, C22);
	k_sem_give(&phy->phy_lock);

	return retval;
}

int phy_modify_c22(struct phy_device *phy, uint8_t regnum,
		   uint16_t mask, uint16_t val)
{
	int retval;
	uint16_t data;

	retval = phy_read_c22(phy, regnum, &data);
	if (retval < 0) {
		return retval;
	}

	return phy_write_c22(phy, regnum, ((data & ~mask) | val));
}

/* MII Compatible PHY functions
 * phy reset and auto-nego implementation is from phy_sam_gmac.c
 */
uint32_t mii_phy_id_get(struct phy_device *phy)
{
	int retval;
	uint16_t phy_reg;
	uint32_t phy_id;

	retval = phy_read_c22(phy, MII_PHYID1R, &phy_reg);
	phy_id = (phy_reg & BIT_MASK(16)) << 16;

	retval |= phy_read_c22(phy, MII_PHYID2R, &phy_reg);
	phy_id |= (phy_reg & BIT_MASK(16));

	return (retval < 0) ? -1 : phy_id;
}

int mii_phy_soft_reset(struct phy_device *phy, bool wait)
{
	int retries = PHY_SOFT_RESET_TIMEOUT_MS / PHY_SOFT_RESET_POLL_MS;
	int retval;
	uint16_t status;

	/* Issue a soft reset */
	retval = phy_modify_c22(phy, MII_BMCR, MII_BMCR_RESET, MII_BMCR_RESET);
	if (retval < 0) {
		return retval;
	}

	if (wait) {
		do {
			if (retries-- == 0) {
				return -ETIMEDOUT;
			}

			k_sleep(K_MSEC(PHY_SOFT_RESET_POLL_MS));

			retval = phy_read_c22(phy, MII_BMCR, &status);
			if (retval < 0) {
				continue;
			}
		} while (status & MII_BMCR_RESET);

	}

	return 0;
}

static inline int mii_phy_get_supported(struct phy_device *phy)
{
	uint16_t bmsr_val, estat_val;
	int retval;

	retval = phy_read_c22(phy, MII_BMSR, &bmsr_val);
	retval |= phy_read_c22(phy, MII_ESTAT, &estat_val);
	if (retval < 0) {
		return retval;
	}

	if (bmsr_val & MII_BMSR_10_FULL) {
		phy->support |= PHY_SUPPORT_10_FULL;
		LOG_DBG("PHY Supported: 10BASE-T, Full Duplex");
	}
	if (bmsr_val & MII_BMSR_10_HALF) {
		phy->support |= PHY_SUPPORT_10_HALF;
		LOG_DBG("PHY Supported: 10BASE-T, Half Duplex");
	}
	if (bmsr_val & MII_BMSR_100BASE_X_FULL) {
		phy->support |= PHY_SUPPORT_100_FULL;
		LOG_DBG("PHY Supported: 100BASE-X, Full Duplex");
	}
	if (bmsr_val & MII_BMSR_100BASE_X_HALF) {
		phy->support |= PHY_SUPPORT_100_HALF;
		LOG_DBG("PHY Supported: 100BASE-X, Half Duplex");
	}
	if (bmsr_val & MII_BMSR_EXTEND_STATUS) {
		if (estat_val & MII_ESTAT_1000BASE_T_FULL) {
			phy->support |= PHY_SUPPORT_1000_FULL;
			LOG_DBG("PHY Supported: 1000BASE-T, Full Duplex");
		}
		if (estat_val & MII_ESTAT_1000BASE_T_HALF) {
			phy->support |= PHY_SUPPORT_1000_HALF;
			LOG_DBG("PHY Supported: 1000BASE-T, Half Duplex");
		}
	}

	return 0;
}

/* To configure PHY link setting
 *
 * param autoneg: turn on/off auto-negotiation [true / false]
 * param link_speed: link speed setting in Mbps
 *                   [e.g. 10, 100, 1000, -1 (use by autoneg)]
 * param full_duplex: duplex mode setting
 *                    [1 (full duplex); 0 (half duplex); -1 (use by autoneg)]
 * param wait: wait for link setting completion [true / false]
 *
 * auto-negotiation ON mode
 * ========================
 * setting link_speed / full_duplex to -1 indicating auto select the best
 * configuration from link negotiation.
 * setting link_speed / full_duplex to a supported value indicating advertise
 * only that value.
 *
 * auto-negotiation OFF mode
 * =========================
 * setting link_speed / full_duplex to -1 is forbidden and will return -EINVAL.
 */
int mii_phy_config_link(struct phy_device *phy, bool autoneg, int32_t link_speed,
			int8_t full_duplex, bool wait)
{
	int retries = PHY_AUTONEG_1000_TIMEOUT_MS / PHY_AUTONEG_POLL_MS;
	int retval = 0;
	uint16_t val = 0;
	uint16_t mask;
	uint32_t advertise = 0;
	uint32_t advertise_1000 = 0;

	/* Obtain PHY supported list and set advertise */
	if (!phy->support) {
		retval = mii_phy_get_supported(phy);
		if (retval < 0) {
			return retval;
		}
	}
	if (phy->support & PHY_SUPPORT_1000_FULL) {
		advertise_1000 |= MII_ADVERTISE_1000_FULL;
	}
	if (phy->support & PHY_SUPPORT_1000_HALF) {
		advertise_1000 |= MII_ADVERTISE_1000_HALF;
	}
	if (phy->support & PHY_SUPPORT_100_FULL) {
		advertise |= MII_ADVERTISE_100_FULL;
	}
	if (phy->support & PHY_SUPPORT_100_HALF) {
		advertise |= MII_ADVERTISE_100_HALF;
	}
	if (phy->support & PHY_SUPPORT_10_FULL) {
		advertise |= MII_ADVERTISE_10_FULL;
	}
	if (phy->support & PHY_SUPPORT_10_HALF) {
		advertise |= MII_ADVERTISE_10_HALF;
	}

	/* Fix to specific link speed & duplex mode */
	if (full_duplex > 0) {
		val = MII_BMCR_DUPLEX_MODE;
		switch (link_speed) {
		case 1000:
			advertise_1000 &= MII_ADVERTISE_1000_FULL;
			advertise = 0;
			val |= MII_BMCR_SPEED_1000;
			break;
		case 100:
			advertise_1000 = 0;
			advertise &= MII_ADVERTISE_100_FULL;
			val |= MII_BMCR_SPEED_100;
			break;
		case 10:
			advertise_1000 = 0;
			advertise &= MII_ADVERTISE_10_FULL;
			val |= MII_BMCR_SPEED_10;
			break;
		default:
			advertise_1000 &= MII_ADVERTISE_1000_FULL;
			advertise &= MII_ADVERTISE_10_FULL |
					MII_ADVERTISE_100_FULL;
			break;
		}
	} else if (full_duplex == 0) {
		switch (link_speed) {
		case 1000:
			advertise_1000 &= MII_ADVERTISE_1000_HALF;
			advertise = 0;
			val = MII_BMCR_SPEED_1000;
			break;
		case 100:
			advertise_1000 = 0;
			advertise &= MII_ADVERTISE_100_HALF;
			val = MII_BMCR_SPEED_100;
			break;
		case 10:
			advertise_1000 = 0;
			advertise &= MII_ADVERTISE_10_HALF;
			val = MII_BMCR_SPEED_10;
			break;
		default:
			advertise_1000 &= MII_ADVERTISE_1000_HALF;
			advertise &= MII_ADVERTISE_10_HALF |
					MII_ADVERTISE_100_HALF;
			break;
		}
	}
	if (!advertise_1000 && !advertise) {
		return -EINVAL;
	}

	if (autoneg) {
		/* Configure 10/100BASE-T advertise */
		mask = (MII_ADVERTISE_10_HALF | MII_ADVERTISE_10_FULL |
			MII_ADVERTISE_100_HALF | MII_ADVERTISE_100_FULL |
			MII_ADVERTISE_100BASE_T4 | MII_ADVERTISE_PAUSE |
			MII_ADVERTISE_ASYM_PAUSE);
		advertise |= MII_ADVERTISE_SEL_IEEE_802_3;
		retval = phy_modify_c22(phy, MII_ANAR, mask, advertise);

		/* Configure 1000BASE-T advertise */
		mask = MII_ADVERTISE_1000_HALF | MII_ADVERTISE_1000_FULL;
		retval |= phy_modify_c22(phy, MII_MSCR, mask, advertise_1000);

		/* Restart auto-nego */
		retval |= phy_modify_c22(phy, MII_BMCR, MII_BMCR_ISOLATE,
					 MII_BMCR_AUTONEG_ENABLE |
					 MII_BMCR_AUTONEG_RESTART);

		/* Wait for the auto-negotiation process to complete */
		if (wait && !retval) {
			do {
				if (retries-- == 0) {
					retval = -ETIMEDOUT;
					LOG_ERR("PHY auto-negotiate "
						"timeout");
					break;
				}

				k_sleep(K_MSEC(PHY_AUTONEG_POLL_MS));

				retval = phy_read_c22(phy, MII_BMSR, &val);
				if (retval < 0) {
					continue;
				}
			} while (!(val & MII_BMSR_AUTONEG_COMPLETE));
		}
	} else {
		if ((link_speed < 0) || (full_duplex < 0)) {
			return -EINVAL;
		}

		/* Configure link speed and duplex in manual mode */
		mask = MII_BMCR_SPEED_MASK | MII_BMCR_AUTONEG_ENABLE |
			MII_BMCR_DUPLEX_MODE;
		retval = phy_modify_c22(phy, MII_BMCR, mask, val);
		if (retval == 0) {
			/* Certain PHY devices need soft reset for new speed
			 * and duplex to take effect.
			 */
			retval = mii_phy_soft_reset(phy, wait);
			if (retval < 0) {
				return retval;
			}
		}

		/* BMCR may revert to default for certain PHY devices after
		 * soft reset. So, write the same speed and duplex again.
		 */
		 retval = phy_modify_c22(phy, MII_BMCR, mask, val);
	}
	if (retval < 0) {
		return retval;
	}

#ifdef CONFIG_ETH_PHY_DEBUG
	/* Print Link Partner advertise speed and duplex */
	LOG_DBG("Link partner advertised: ");
	/* 10/100BASE-T */
	retval = phy_read_c22(phy, MII_ANLPAR, &val);
	if (retval == 0) {
		if (val & MII_ADVERTISE_100_FULL) {
			LOG_DBG("Speed: 100Mbps Duplex: Full");
		}
		if (val & MII_ADVERTISE_100_HALF) {
			LOG_DBG("Speed: 100Mbps Duplex: Half");
		}
		if (val & MII_ADVERTISE_10_FULL) {
			LOG_DBG("Speed: 10Mbps Duplex: Full");
		}
		if (val & MII_ADVERTISE_10_HALF) {
			LOG_DBG("Speed: 10Mbps Duplex: Half");
		}
	}

	/* 1000BASE-T */
	retval |= phy_read_c22(phy, MII_MSSR, &val);
	if (retval == 0) {
		if (val & MII_LP_ADVERTISE_1000_FULL) {
			LOG_DBG("Speed: 1000Mbps Duplex: Full");
		}
		if (val & MII_LP_ADVERTISE_1000_HALF) {
			LOG_DBG("Speed: 1000Mbps Duplex: Half");
		}
	}
#endif /* CONFIG_ETH_PHY_DEBUG */

	return retval;
}
#endif /* CONFIG_ETH_PHY_USE_C22 */

#ifdef CONFIG_ETH_PHY_USE_C45
int phy_write_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum, uint16_t val)
{
	union mdio_frm_flds frm_fld;
	int retval;

	frm_fld.c45_frm_fld.portaddr = phy->addr;
	frm_fld.c45_frm_fld.devaddr = devnum;
	frm_fld.c45_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = phy->mdio_write(phy, frm_fld, val, C45);
	k_sem_give(&phy->phy_lock);

	return retval;
}

int phy_read_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum, uint16_t *val)
{
	union mdio_frm_flds frm_fld;
	int retval;

	frm_fld.c45_frm_fld.portaddr = phy->addr;
	frm_fld.c45_frm_fld.devaddr = devnum;
	frm_fld.c45_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = phy->mdio_read(phy, frm_fld, val, C45);
	k_sem_give(&phy->phy_lock);

	return retval;
}

int phy_modify_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum,
		   uint16_t mask, uint16_t val)
{
	int retval;
	uint16_t data;

	retval = phy_read_c45(phy, devnum, regnum, &data);
	if (retval < 0) {
		return retval;
	}

	return phy_write_c45(phy, devnum, regnum, ((data & ~mask) | val));
}

/* MMD Compatible PHY functions */
uint32_t mmd_phy_id_get(struct phy_device *phy, uint8_t devnum)
{
	int retval;
	uint16_t phy_reg;
	uint32_t phy_id;

	retval = phy_read_c45(phy, devnum, MII_PHYID1R, &phy_reg);
	phy_id = (phy_reg & BIT_MASK(16)) << 16;

	retval |= phy_read_c45(phy, devnum, MII_PHYID2R, &phy_reg);
	phy_id |= (phy_reg & BIT_MASK(16));

	return (retval < 0) ? -1 : phy_id;
}
#endif /* CONFIG_ETH_PHY_USE_C45 */
