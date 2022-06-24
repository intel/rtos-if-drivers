/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Phy
 * The media-independent interface (MII) was originally defined as a standard
 * interface to connect a Fast Ethernet (i.e., 100 Mbit/s) media access control
 * (MAC) block to a PHY chip. The MII is standardized by IEEE 802.3u and
 * connects different types of PHYs to MACs.
 * The Management Data Input/Output (MDIO) serial bus is a subset of the MII
 * that is used to transfer management information between MAC and PHY
 */

#ifndef _PHY_H_
#define _PHY_H_

#include <zephyr/types.h>
#include <sys/util.h>
/* TODO: mii.h should be pointed to net/mii.h if the files is ready
 * with all MII defination needed.
 */
#include <net/mii.h>

/**
 * @addtogroup ethernet
 * @{
 */
 #ifdef __cplusplus
extern "C" {
#endif

/* Maximum time to establish a link through auto-negotiation for
 * 10BASE-T, 100BASE-TX is 3.7s, to add an extra margin the timeout
 * is set at 4s.
 * 1000BASE-T is 5.3s, add 0.2s margin
 * http://www.ieee802.org/3/af/public/jan02/brown_1_0102.pdf
 */
#define PHY_AUTONEG_1000_TIMEOUT_MS             5500
#define PHY_AUTONEG_100_TIMEOUT_MS              4000
#define PHY_AUTONEG_POLL_MS                     100

/* Wait up to 0.6s for the reset sequence to finish. According to
 * IEEE 802.3, Section 2, Subsection 22.2.4.1.1 a PHY reset may take
 * up to 0.5 s.
 */
#define PHY_SOFT_RESET_TIMEOUT_MS               600
#define PHY_SOFT_RESET_POLL_MS                  50

/* MDIO read/write mode access indentification */
#define C45             true
#define C22             false

union mdio_frm_flds {
	struct {
		uint8_t phyaddr;
		uint8_t regaddr;
	} c22_frm_fld;
	struct {
		uint8_t portaddr;
		uint8_t devaddr;
		uint16_t regaddr;
	} c45_frm_fld;
};

enum phy_interface {
	PHY_INTERFACE_MII,
	PHY_INTERFACE_GMII,
	PHY_INTERFACE_SGMII,
	PHY_INTERFACE_RGMII,
};

enum phy_support {
	PHY_SUPPORT_10_HALF    = BIT(0),
	PHY_SUPPORT_10_FULL    = BIT(1),
	PHY_SUPPORT_100_HALF   = BIT(2),
	PHY_SUPPORT_100_FULL   = BIT(3),
	PHY_SUPPORT_1000_HALF  = BIT(4),
	PHY_SUPPORT_1000_FULL  = BIT(5),
};

struct phy_device {
	uint8_t addr;
	enum phy_interface interface;
	enum phy_support support;
	enum phy_support advertise;
	enum phy_support lp_advertise;
	bool cur_link_sts;
	struct k_sem phy_lock;
	int (*init)(struct phy_device *phy);
	int (*cfg_link)(struct phy_device *phy, bool autoneg, int32_t link_speed,
			int8_t full_duplex, bool wait);
	int (*read_status)(struct phy_device *phy, bool *link,
			   int32_t *link_speed, int8_t *full_duplex);
	int (*en_intr)(struct phy_device *phy);
	int (*intr_status)(struct phy_device *phy, bool *sts_chg);
	int (*mdio_read)(struct phy_device *phy,
			 union mdio_frm_flds fld_data,
			 uint16_t *data, bool c45);
	int (*mdio_write)(struct phy_device *phy,
			  union mdio_frm_flds fld_data,
			  uint16_t data, bool c45);
	int (*get_tx_latency)(struct phy_device *phy, int32_t link_speed,
			      int32_t *latency);
	int (*get_rx_latency)(struct phy_device *phy, int32_t link_speed,
			      int32_t *latency);
};

#ifdef CONFIG_ETH_PHY_USE_C22
int phy_write_c22(struct phy_device *phy, uint8_t regnum, uint16_t val);
int phy_read_c22(struct phy_device *phy, uint8_t regnum, uint16_t *val);
int phy_modify_c22(struct phy_device *phy, uint8_t regnum, uint16_t mask, uint16_t val);
uint32_t mii_phy_id_get(struct phy_device *phy);
int mii_phy_soft_reset(struct phy_device *phy, bool wait);
int mii_phy_config_link(struct phy_device *phy, bool autoneg, int32_t link_speed,
			int8_t full_duplex, bool wait);
#endif /* CONFIG_ETH_PHY_USE_C22 */

#ifdef CONFIG_ETH_PHY_USE_C45
int phy_write_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum, uint16_t val);
int phy_read_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum, uint16_t *val);
int phy_modify_c45(struct phy_device *phy, uint8_t devnum, uint16_t regnum,
		   uint16_t mask, uint16_t val);
uint32_t mmd_phy_id_get(struct phy_device *phy, uint8_t devnum);
#endif /* CONFIG_ETH_PHY_USE_C45 */

#ifdef __cplusplus
}
#endif
/**
 * @}
 */

#endif /* _PHY_H_ */
