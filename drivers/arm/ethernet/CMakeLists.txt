# SPDX-License-Identifier: Apache-2.0

zephyr_library_include_directories(
        ${ZEPHYR_BASE}/subsys/net/l2
        ${WEST_TOPDIR}/zephyr-iotg/include/
        )

zephyr_sources_ifdef(CONFIG_ETH_PHY		phy.c)
zephyr_sources_ifdef(CONFIG_ETH_PHY_88E1512	phy_marvell_88e1512.c)
zephyr_sources_ifdef(CONFIG_ETH_PHY_GEN_PHY	gen_phy.c)
zephyr_sources_ifdef(CONFIG_ETH_DWC_EQOS
	eth_dwc_eqos_v5.c
	dw_tsn_lib.c
	)
if(CONFIG_ETH_DWC_EQOS_NETWORK_PROXY)
	zephyr_include_directories(${ZEPHYR_BASE}/subsys/net/ip/)
	zephyr_sources(eth_dwc_eqos_v5_netprox.c)
endif()

zephyr_sources_ifdef(CONFIG_ETH_PSE_VNIC eth_pse_vnic.c)
