/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Marvell 88E1512 PHY driver.
 */

#ifndef _PHY_MARVELL_88E1512_H_
#define _PHY_MARVELL_88E1512_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MARVELL_PHY_ID_88E1512                  0x01410dd1

/* Refer Datasheet Section 3.2 PHY MDIO Register Description
 * Register is access through Page 0 to Page 18
 * Page 0 is MII compatible register
 * For e.g: To access Page 18 General Control Register 1
 * Need to update Page Address Register(Page Any, Register 22)
 * Page Number to 18
 */
#define REG_PAGE_ADDR                           0x16

/* General Control Register 1 - Page 18, Reg 20 */
#define REG_GENERAL_CTRL_1                      0x14
#define REG_GENERAL_CTRL_MODE_MASK              BIT_MASK(3)
#define REG_GENERAL_CTRL_1_SGMII_COPPER         BIT(0)
#define REG_GENERAL_CTRL_1_RESET                BIT(15)

/* Copper Specific Interrupt Enable Register - Page 0, Reg 18 */
#define REG_COPPER_SPE_INT_ENABLE               0x12

/* Copper Specific Interrupt Status Register - Page 0, Reg 19 */
#define REG_COPPER_SPE_INT_STATUS               0x13
#define REG_COPPER_SPE_INT_LINK                 BIT(10)
#define REG_COPPER_SPE_INT_AUTONEG              BIT(11)
#define REG_COPPER_SPE_INT_DUPLEX               BIT(13)
#define REG_COPPER_SPE_INT_SPEED                BIT(14)
#define REG_COPPER_SPE_INT_DEF                  (REG_COPPER_SPE_INT_LINK | \
						 REG_COPPER_SPE_INT_AUTONEG | \
						 REG_COPPER_SPE_INT_DUPLEX | \
						 REG_COPPER_SPE_INT_SPEED)

/* LED[2:0] Function Control Register - Page 3, Reg 16 */
#define REG_LED_FUNC_CTRL                       0x10
/* LED[1] Control Bit 7:4 0011 = On - Activity, Off - No Activity */
#define REG_LED_FUNC_CTRL_DEF                   (BIT(4) | BIT(5))

/* LED Timer Control Register - Page 3, Reg 18 */
#define REG_LED_TIMER_CTRL                      0x12
/* LED[2] Control Bit 7 = On - Interrupt Function, Off - LED Function */
#define REG_LED_TIMER_CTRL_INT_EN               BIT(7)
#define REG_LED_TIMER_CTRL_INT_POL              BIT(11)

/* MAC Specific Control Register 1 - Page 2, Reg 16 */
#define REG_SPE_MAC_CTRL_1                      0x10
#define REG_SPE_MAC_CTRL_1_PAD_ODD              BIT(6)

/* Copper Specific Status Register 1 - Page 0, Reg 17 */
#define REG_COPPER_SPE_STATUS_1                 0x11
#define REG_COPPER_SPE_STATUS_LINK              BIT(10)
#define REG_COPPER_SPE_STATUS_FULL_DUPLEX       BIT(13)
#define REG_COPPER_SPE_STAUS_SPEED_MASK         (BIT(15) | BIT(14))
#define REG_COPPER_SPE_STAUS_SPEED_1000         BIT(15)
#define REG_COPPER_SPE_STAUS_SPEED_100          BIT(14)

/* MDIO Manageable Device (MMD) Auto-Negotiation */
#define MDIO_AN_DEVICE                          7

/* PHY EEE Advertisement Register - Device 7, Reg 60 */
#define MDIO_AN_EEE_ADV                         60
/* 100M & 1000M link speed */
#define MDIO_AN_EEE_ADV_SPEED                   (BIT(1) | BIT(2))

int marvell_88e1512_init(struct phy_device *phy);

int marvell_88e1512_read_status(struct phy_device *phy, bool *link,
				int32_t *link_speed, int8_t *full_duplex);

int marvell_88e1512_config_link(struct phy_device *phy, bool autoneg,
				int32_t link_speed, int8_t full_duplex, bool wait);

int marvell_88e1512_enable_interrupt(struct phy_device *phy);

int marvell_88e1512_interrupt_status(struct phy_device *phy, bool *intr_sig);

int marvell_88e1512_get_tx_latency(struct phy_device *phy, int32_t link_speed,
				   int32_t *latency);

int marvell_88e1512_get_rx_latency(struct phy_device *phy, int32_t link_speed,
				   int32_t *latency);

#ifdef __cplusplus
}
#endif

#endif /* _PHY_MARVELL_88E1512_H_ */
