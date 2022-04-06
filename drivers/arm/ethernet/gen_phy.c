/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Generic PHY driver.
 */

#define LOG_MODULE_NAME gen_phy
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <kernel.h>
#include <sys/util.h>
#include <net/phy_pse.h>
#include "gen_phy.h"

#define SPEED_1000	1000
#define SPEED_100	100
#define SPEED_10	10

#define DUPLEX_FULL	true
#define DUPLEX_HALF	false

/* NOTE: Please use this function for MMD device number of 1 to 13. This
 * function will return true if there is specific MMD device present and return
 * false if there is no specific MMD device present or C45 is not accessible.
 * Error code is not used here because the purpose of this function is to inform
 * the caller whether the specific MMD device is present or not. Not to inform
 * pass or fail.
 */
static bool gen_phy_mmd_device_present_check(struct phy_device *phy,
					     uint8_t devnum)
{
	uint32_t devices_in_package;
	uint16_t regval;
	int retval;

	/* Read devices in package value from Register x.5 and x.6 (x is MMD
	 * device number).
	 */
	retval = phy_read_c45(phy, devnum, MDIO_DEV_IN_PKG2, &regval);
	if (retval < 0) {
		goto no_dev;
	}

	devices_in_package = (uint32_t)regval << 16;

	retval = phy_read_c45(phy, devnum, MDIO_DEV_IN_PKG1, &regval);
	if (retval < 0) {
		goto no_dev;
	}

	devices_in_package |= (uint32_t)regval;

	/* Based on IEEE 802.3-2018 Table 45-2, Register x.6 bit-0 to bit-12 as
	 * well as Register x.5 bit-14 and bit-15 are reserved and the value
	 * should be 0. So, if all those bits are read as 1, we could consider
	 * the PHY doesn`t have the MMD device supported.
	 */
	if ((devices_in_package & DEV_IN_PKG_RSVD) == DEV_IN_PKG_RSVD) {
		goto no_dev;
	}

	return true;

no_dev:
	return false;
}

int gen_phy_init(struct phy_device *phy)
{
	int phy_id;

	phy_id = mii_phy_id_get(phy);
	if (phy_id < 0) {
		LOG_ERR("PHY Init Failed. Unable to read PHY ID");
		return -ENODEV;
	}

	LOG_INF("PHY Init Success. PHY ID 0x%X", phy_id);

	return 0;
}

static int gen_phy_get_link_partner_adv(struct phy_device *phy)
{
	uint16_t mssr_val, anlpar_val;
	int retval;

	retval = phy_read_c22(phy, MII_MSSR, &mssr_val);
	if (retval < 0) {
		return retval;
	}
	retval = phy_read_c22(phy, MII_ANLPAR, &anlpar_val);
	if (retval < 0) {
		return retval;
	}

	phy->lp_advertise = 0;
	if (mssr_val & MII_LP_ADVERTISE_1000_FULL) {
		phy->lp_advertise |= PHY_SUPPORT_1000_FULL;
		LOG_DBG("Link Partner Advertise Speed: 1000Mbps Duplex: Full");
	}
	if (mssr_val & MII_LP_ADVERTISE_1000_HALF) {
		phy->lp_advertise |= PHY_SUPPORT_1000_HALF;
		LOG_DBG("Link Partner Advertise Speed: 1000Mbps Duplex: Half");
	}
	if (anlpar_val & MII_ADVERTISE_100_FULL) {
		phy->lp_advertise |= PHY_SUPPORT_100_FULL;
		LOG_DBG("Link Partner Advertise Speed: 100Mbps Duplex: Full");
	}
	if (anlpar_val & MII_ADVERTISE_100_HALF) {
		phy->lp_advertise |= PHY_SUPPORT_100_HALF;
		LOG_DBG("Link Partner Advertise Speed: 100Mbps Duplex: Half");
	}
	if (anlpar_val & MII_ADVERTISE_10_FULL) {
		phy->lp_advertise |= PHY_SUPPORT_10_FULL;
		LOG_DBG("Link Partner Advertise Speed: 10Mbps Duplex: Full");
	}
	if (anlpar_val & MII_ADVERTISE_10_HALF) {
		phy->lp_advertise |= PHY_SUPPORT_10_HALF;
		LOG_DBG("Link Partner Advertise Speed: 10Mbps Duplex: Half");
	}

	return 0;
}

static void gen_phy_update_link_autoneg_on(struct phy_device *phy,
					   int32_t *link_speed,
					   int8_t *full_duplex)
{
	uint16_t common = phy->advertise & phy->lp_advertise;

	/* The top most common between PHY advertise and link partner advertise
	 * is chosen to update link_speed and full_duplex. Below checking is
	 * done from the highest priority.
	 */
	if (common & PHY_SUPPORT_1000_FULL) {
		*link_speed = SPEED_1000;
		*full_duplex = DUPLEX_FULL;
	} else if (common & PHY_SUPPORT_1000_HALF) {
		*link_speed = SPEED_1000;
		*full_duplex = DUPLEX_HALF;
	} else if (common & PHY_SUPPORT_100_FULL) {
		*link_speed = SPEED_100;
		*full_duplex = DUPLEX_FULL;
	} else if (common & PHY_SUPPORT_100_HALF) {
		*link_speed = SPEED_100;
		*full_duplex = DUPLEX_HALF;
	} else if (common & PHY_SUPPORT_10_FULL) {
		*link_speed = SPEED_10;
		*full_duplex = DUPLEX_FULL;
	} else if (common & PHY_SUPPORT_10_HALF) {
		*link_speed = SPEED_10;
		*full_duplex = DUPLEX_HALF;
	}
}

static int gen_phy_update_link_autoneg_off(struct phy_device *phy,
					   int32_t *link_speed,
					   int8_t *full_duplex)
{
	int retval;
	uint16_t val;

	retval = phy_read_c22(phy, MII_BMCR, &val);
	if (retval < 0) {
		return retval;
	}

	if (val & MII_BMCR_DUPLEX_MODE) {
		*full_duplex = DUPLEX_FULL;
	} else {
		*full_duplex = DUPLEX_HALF;
	}

	if (val & MII_BMCR_SPEED_1000) {
		*link_speed = SPEED_1000;
	} else if (val & MII_BMCR_SPEED_100) {
		*link_speed = SPEED_100;
	} else {
		*link_speed = SPEED_10;
	}

	return 0;
}

/* NOTE: link, link_speed & full_duplex value are valid only if
 * function call return no error
 */
int gen_phy_read_status(struct phy_device *phy, bool *link, int32_t *link_speed,
			int8_t *full_duplex)
{
	int retval;
	uint16_t val;
	bool autoneg_enable;

	*link = phy->cur_link_sts;

	retval = gen_phy_get_link_partner_adv(phy);
	if (retval < 0) {
		return retval;
	}

	retval = phy_read_c22(phy, MII_BMCR, &val);
	if (retval < 0) {
		return retval;
	}

	autoneg_enable = val & MII_BMCR_AUTONEG_ENABLE ? true : false;

	if (*link) {
		if (autoneg_enable) {
			gen_phy_update_link_autoneg_on(phy,
						       link_speed,
						       full_duplex);
		} else {
			retval = gen_phy_update_link_autoneg_off(phy,
								 link_speed,
								 full_duplex);
			if (retval < 0) {
				return retval;
			}
		}

		LOG_INF("PHY Link Up Speed %dMbps Duplex:%s",
			*link_speed, (*full_duplex ? "Full" : "Half"));
	} else {
		LOG_INF("PHY Link Down");
	}

	return 0;
}

int gen_phy_config_link(struct phy_device *phy, bool autoneg,
			int32_t link_speed, int8_t full_duplex, bool wait)
{
	int retval;

	if (gen_phy_mmd_device_present_check(phy, MDIO_MMD_AN)) {
		/* For future expand to support multi-gigabit Ethernet,
		 * currently disable all advertisement above 1Gbps.
		 */
		retval = phy_write_c45(phy, MDIO_MMD_AN, MDIO_MGBT_AN_CTRL1, 0);
		if (retval < 0) {
			return retval;
		}
	}

	return mii_phy_config_link(phy, autoneg, link_speed, full_duplex, wait);
}

/* NOTE: link_sts_chg  value are valid only if function call
 * return no error.
 */
int gen_phy_link_status(struct phy_device *phy, bool *link_sts_chg)
{
	int retval;
	uint16_t val;
	bool old_link = phy->cur_link_sts;

	retval = phy_read_c22(phy, MII_BMCR, &val);
	if (retval < 0) {
		return retval;
	}

	/* Autoneg is being started, therefore disregard BMSR value and
	 * report link as down.
	 */
	if (val & MII_BMCR_AUTONEG_RESTART) {
		goto done;
	}

	/* Read link status */
	retval = phy_read_c22(phy, MII_BMSR, &val);

	if (retval < 0) {
		return retval;
	}
done:
	phy->cur_link_sts = val & MII_BMSR_LINK_STATUS ? true : false;
	*link_sts_chg = old_link == phy->cur_link_sts ? false : true;

	LOG_DBG("MII BMSR Link Status is %d", phy->cur_link_sts);

	return 0;
}

/* This a dummy function. Gen PHY doesn`t need interrupt */
int gen_phy_dummy_function(struct phy_device *phy)
{
	return 0;
}

int gen_phy_get_tx_latency(struct phy_device *phy, int32_t link_speed,
			   int32_t *latency)
{
	*latency = 0;

	/* Obtain the PHY TX latency from build config */
	switch (link_speed) {
	case 10:
#ifdef CONFIG_ETH_PHY_GEN_PHY_TX_DLY_10
		*latency = CONFIG_ETH_PHY_GEN_PHY_TX_DLY_10;
#endif
		break;
	case 100:
#ifdef CONFIG_ETH_PHY_GEN_PHY_TX_DLY_100
		*latency = CONFIG_ETH_PHY_GEN_PHY_TX_DLY_100;
#endif
		break;
	case 1000:
#ifdef CONFIG_ETH_PHY_GEN_PHY_TX_DLY_1000
		*latency = CONFIG_ETH_PHY_GEN_PHY_TX_DLY_1000;
#endif
		break;
	default:
		LOG_ERR("Unknown Link Speed.");
		return -EINVAL;
	}

	return 0;
}

int gen_phy_get_rx_latency(struct phy_device *phy, int32_t link_speed,
			   int32_t *latency)
{
	*latency = 0;

	/* Obtain the PHY RX latency from build config */
	switch (link_speed) {
	case 10:
#ifdef CONFIG_ETH_PHY_GEN_PHY_RX_DLY_10
		*latency = CONFIG_ETH_PHY_GEN_PHY_RX_DLY_10;
#endif
		break;
	case 100:
#ifdef CONFIG_ETH_PHY_GEN_PHY_RX_DLY_100
		*latency = CONFIG_ETH_PHY_GEN_PHY_RX_DLY_100;
#endif
		break;
	case 1000:
#ifdef CONFIG_ETH_PHY_GEN_PHY_RX_DLY_1000
		*latency = CONFIG_ETH_PHY_GEN_PHY_RX_DLY_1000;
#endif
		break;
	default:
		LOG_ERR("Unknown Link Speed.");
		return -EINVAL;
	}

	return 0;
}
