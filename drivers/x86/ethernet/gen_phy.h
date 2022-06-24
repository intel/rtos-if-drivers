/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Generic PHY driver.
 */

#ifndef _GEN_PHY_H_
#define _GEN_PHY_H_

#ifdef __cplusplus
extern "C" {
#endif

int gen_phy_init(struct phy_device *phy);

int gen_phy_read_status(struct phy_device *phy, bool *link, int32_t *link_speed,
			int8_t *full_duplex);

int gen_phy_config_link(struct phy_device *phy, bool autoneg,
			int32_t link_speed, int8_t full_duplex, bool wait);

int gen_phy_link_status(struct phy_device *phy, bool *link_sts_chg);

int gen_phy_dummy_function(struct phy_device *phy);

int gen_phy_get_tx_latency(struct phy_device *phy, int32_t link_speed,
			   int32_t *latency);

int gen_phy_get_rx_latency(struct phy_device *phy, int32_t link_speed,
			   int32_t *latency);

#ifdef __cplusplus
}
#endif

#endif /* _GEN_PHY_H_ */
