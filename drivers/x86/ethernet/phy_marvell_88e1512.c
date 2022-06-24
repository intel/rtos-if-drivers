/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Marvell 88E1512 PHY driver.
 */

#define LOG_MODULE_NAME phy_marvell_88e1512
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <kernel.h>
#include <sys/util.h>
#include <drivers/ethernet/phy_ehl.h>
#include "phy_marvell_88e1512.h"

int marvell_88e1512_init(struct phy_device *phy)
{
	int retval;
	int phy_id;

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

	phy_id = mii_phy_id_get(phy);
	if (phy_id != MARVELL_PHY_ID_88E1512) {
		LOG_ERR("Incorrect PHY ID %X", phy_id);
		return -ENODEV;
	}

	/* Power Up the PHY */
	retval = phy_modify_c22(phy, MII_BMCR, MII_BMCR_POWER_DOWN, 0);
	if (retval < 0) {
		return retval;
	}

	if (phy->interface == PHY_INTERFACE_SGMII) {
		/* Change to Page 18 */
		retval = phy_write_c22(phy, REG_PAGE_ADDR, 18);
		if (retval < 0) {
			return retval;
		}

		/* Configure to sgmii copper mode */
		retval |= phy_modify_c22(phy, REG_GENERAL_CTRL_1,
					 REG_GENERAL_CTRL_MODE_MASK,
					 REG_GENERAL_CTRL_1_SGMII_COPPER);

		/* PHY reset */
		retval |= phy_modify_c22(phy, REG_GENERAL_CTRL_1, 0,
					 REG_GENERAL_CTRL_1_RESET);
		if (retval < 0) {
			return retval;
		}
	}

	/* Change to Page 3 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 3);
	if (retval < 0) {
		return retval;
	}

	/* PHY LED config: LED[0] .. Link, LED[1] .. Activity */
	retval = phy_write_c22(phy, REG_LED_FUNC_CTRL, REG_LED_FUNC_CTRL_DEF);
	if (retval < 0) {
		return retval;
	}

	/* Change to Page 2 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 2);
	if (retval < 0) {
		return retval;
	}

	/* Pass as is and do not pad odd nibble preambles in copper receive
	 * packets
	 */
	retval = phy_modify_c22(phy, REG_SPE_MAC_CTRL_1, 0,
				REG_SPE_MAC_CTRL_1_PAD_ODD);
	if (retval < 0) {
		return retval;
	}

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

#ifdef CONFIG_ETH_PHY_88E1512_EEE
	/* EEE init & Advertised EEE */
	retval = phy_modify_c45(phy, MDIO_AN_DEVICE, MDIO_AN_EEE_ADV, 0,
				MDIO_AN_EEE_ADV_SPEED);
	if (retval < 0) {
		return retval;
	}
#else
	/* Force EEE to OFF when EEE config is not selected */
	retval = phy_write_c45(phy, MDIO_AN_DEVICE, MDIO_AN_EEE_ADV, 0);
	if (retval < 0) {
		return retval;
	}
#endif

	mii_phy_soft_reset(phy, false);

	LOG_INF("PHY Init %s", (retval ? "Failed" : "Success"));

	return retval;
}

/* NOTE: link, link_speed & full_duplex value are valid only if
 * function call return no error
 */
int marvell_88e1512_read_status(struct phy_device *phy, bool *link,
				int32_t *link_speed, int8_t *full_duplex)
{
	int retval;
	uint16_t val;

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

	/* Read Copper Status */
	retval = phy_read_c22(phy, REG_COPPER_SPE_STATUS_1, &val);
	if (retval < 0) {
		return retval;
	}

	*link = ((val & REG_COPPER_SPE_STATUS_LINK) == 0 ? false : true);
	if (*link) {
		if (val & REG_COPPER_SPE_STATUS_FULL_DUPLEX) {
			*full_duplex = true;
		} else {
			*full_duplex = false;
		}

		val &= REG_COPPER_SPE_STAUS_SPEED_MASK;
		if (val == REG_COPPER_SPE_STAUS_SPEED_1000) {
			*link_speed = 1000;
		} else if (val == REG_COPPER_SPE_STAUS_SPEED_100) {
			*link_speed = 100;
		} else {
			*link_speed = 10;
		}

		LOG_INF("PHY Link Up Speed %dMbps Duplex:%s",
			*link_speed, (*full_duplex ? "Full" : "Half"));
	} else {
		LOG_INF("PHY Link Down");
	}

	return 0;
}

int marvell_88e1512_config_link(struct phy_device *phy, bool autoneg,
				int32_t link_speed, int8_t full_duplex, bool wait)
{
	int retval;

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

	return mii_phy_config_link(phy, autoneg, link_speed, full_duplex, wait);
}

int marvell_88e1512_enable_interrupt(struct phy_device *phy)
{
	int retval;

	/* Change to Page 3 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 3);
	if (retval < 0) {
		return retval;
	}

	/* Configure interrupt pin and polarity */
	retval = phy_modify_c22(phy, REG_LED_TIMER_CTRL,
				REG_LED_TIMER_CTRL_INT_EN |
				REG_LED_TIMER_CTRL_INT_POL,
				REG_LED_TIMER_CTRL_INT_EN);
	if (retval < 0) {
		return retval;
	}

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

	/* Enable interrupt */
	return phy_write_c22(phy, REG_COPPER_SPE_INT_ENABLE,
			     REG_COPPER_SPE_INT_LINK);
}

/* NOTE: intr_sig value is valid only if function call return no error */
int marvell_88e1512_interrupt_status(struct phy_device *phy, bool *intr_sig)
{
	int retval;
	uint16_t val;

	/* Change to Page 0 */
	retval = phy_write_c22(phy, REG_PAGE_ADDR, 0);
	if (retval < 0) {
		return retval;
	}

	/* Read interrupt status */
	retval = phy_read_c22(phy, REG_COPPER_SPE_INT_STATUS, &val);
	*intr_sig = (val & REG_COPPER_SPE_INT_LINK) ? true : false;

	return retval;
}

int marvell_88e1512_get_tx_latency(struct phy_device *phy, int32_t link_speed,
				   int32_t *latency)
{
	__ASSERT_NO_MSG(phy);

	*latency = 0;
#if defined(CONFIG_ETH_PHY_88E1512) && defined(CONFIG_NET_GPTP)
	/* Obtain the PHY TX latency from build config */
	if (phy->interface == PHY_INTERFACE_SGMII) {
		switch (link_speed) {
		case 10:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_TX_DLY_10;
			break;
		case 100:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_TX_DLY_100;
			break;
		case 1000:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_TX_DLY_1000;
			break;
		default:
			LOG_ERR("Unknown Link Speed.");
			return -EINVAL;
		}
	} else if (phy->interface == PHY_INTERFACE_RGMII) {
		switch (link_speed) {
		case 10:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_TX_DLY_10;
			break;
		case 100:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_TX_DLY_100;
			break;
		case 1000:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_TX_DLY_1000;
			break;
		default:
			LOG_ERR("Unknown Link Speed.");
			return -EINVAL;
		}
	} else {
		LOG_ERR("Unknown Interface.");
		return -EINVAL;
	}
#endif
	return 0;
}

int marvell_88e1512_get_rx_latency(struct phy_device *phy, int32_t link_speed,
				   int32_t *latency)
{
	__ASSERT_NO_MSG(phy);

	*latency = 0;
#if defined(CONFIG_ETH_PHY_88E1512) && defined(CONFIG_NET_GPTP)
	/* Obtain the PHY RX latency from build config */
	if (phy->interface == PHY_INTERFACE_SGMII) {
		switch (link_speed) {
		case 10:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_RX_DLY_10;
			break;
		case 100:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_RX_DLY_100;
			break;
		case 1000:
			*latency = CONFIG_ETH_PHY_88E1512_SGMII_RX_DLY_1000;
			break;
		default:
			LOG_ERR("Unknown Link Speed.");
			return -EINVAL;
		}
	} else if (phy->interface == PHY_INTERFACE_RGMII) {
		switch (link_speed) {
		case 10:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_RX_DLY_10;
			break;
		case 100:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_RX_DLY_100;
			break;
		case 1000:
			*latency = CONFIG_ETH_PHY_88E1512_RGMII_RX_DLY_1000;
			break;
		default:
			LOG_ERR("Unknown Link Speed.");
			return -EINVAL;
		}
	} else {
		LOG_ERR("Unknown Interface.");
		return -EINVAL;
	}
#endif

	return 0;
}
