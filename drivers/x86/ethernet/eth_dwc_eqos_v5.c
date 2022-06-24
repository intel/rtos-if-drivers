/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME eth_dwc_eqos
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#if defined(CONFIG_ARM)
#define USE_SEDI 1
#endif

#include <soc.h>
#include <device.h>
#include <errno.h>
#include <init.h>
#include <kernel.h>
#include <cache.h>
#include <random/rand32.h>
#include <sys/__assert.h>
#include <net/net_core.h>
#include <net/net_pkt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <drivers/pcie/pcie.h>
#include <net/ethernet.h>
#include <drivers/ethernet/phy_ehl.h>
#include <sys/crc.h>

#ifdef CONFIG_SHARED_IRQ
#include <shared_irq.h>
#endif

#ifdef CONFIG_NET_PKT_TIMESTAMP
#include <net/ptp_time.h>
#endif

#ifdef CONFIG_ETH_DWC_EQOS_PTP
#include <ptp_clock.h>
#include <net/gptp.h>
#endif

#if USE_SEDI
#include "sedi.h"
#else
/*!
 * \enum pse_phyif_type
 * \brief GBE PHY interface type.
 * \ingroup sedi_driver_tsn
 */
enum pse_phyif_type {
        PHY_NOT_CONNECTED,
        PHY_RGMII,
        PHY_SGMII,
        PHY_SGMII_PLUS,
};

/**
 * \brief Get the PSE GbE port PHY interface value from BIOS pass-in data struct
 * \param[in] gbeport PSE GbE port number
 * \return PSE GbE PHY interface type from enum pse_phyif_type
 */
static inline uint32_t pse_gbe_get_phyif(int gbeport)
{
	/* FIXME: get the type using SEDI (for PSE) or devicetree (for IA) */
	return PHY_RGMII;
}

typedef enum {
        NO_SUCH_DEV = -1,
        DEV_PSE_OWNED = 0,
        DEV_LH_OWNED_MSI,
        DEV_LH_OWNED_SB,
        DEV_NO_OWNER
} sedi_dev_ownership_t;

#define sedi_get_dev_ownership(a) DEV_PSE_OWNED

/* FIXME: Get the value from devicetree */
#define ETH_DWC_EQOS_SYS_CLOCK 1000000
#endif

#include "eth_dwc_eqos_v5_priv.h"
#include "eth_dwc_eqos_v5_macros.h"
#include "dw_tsn_lib.h"

#ifdef CONFIG_ETH_PHY_88E1512
#include "phy_marvell_88e1512.h"
#endif

#ifdef CONFIG_ETH_PHY_GEN_PHY
#include "gen_phy.h"
#endif

uint8_t uninitialized_macaddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
/* Statistics Registers and their offsets */
static const struct eth_statistic {
	const char * const key;
	const uint32_t offset;
} statreg[] = {
	{ .key = "Tx_Octet_Count_Good_Bad", .offset = 0x714 },
	{ .key = "Tx_Packet_Count_Good_Bad", .offset = 0x718 },
	{ .key = "Tx_Broadcast_Packets_Good", .offset = 0x71c },
	{ .key = "Tx_Multicast_Packets_Good", .offset = 0x720 },
	{ .key = "Tx_64Octets_Packets_Good_Bad", .offset = 0x724 },
	{ .key = "Tx_65To127Octets_Packets_Good_Bad", .offset = 0x728 },
	{ .key = "Tx_128To255Octets_Packets_Good_Bad", .offset = 0x72c },
	{ .key = "Tx_256To511Octets_Packets_Good_Bad", .offset = 0x730 },
	{ .key = "Tx_512To1023Octets_Packets_Good_Bad", .offset = 0x734 },
	{ .key = "Tx_1024ToMaxOctets_Packets_Good_Bad", .offset = 0x738 },
	{ .key = "Tx_Unicast_Packets_Good_Bad", .offset = 0x73c },
	{ .key = "Tx_Multicast_Packets_Good_Bad", .offset = 0x740 },
	{ .key = "Tx_Broadcast_Packets_Good_Bad", .offset = 0x744 },
	{ .key = "Tx_Underflow_Error_Packets", .offset = 0x748 },
	{ .key = "Tx_Single_Collision_Good_Packets", .offset = 0x74c },
	{ .key = "Tx_Multiple_Collision_Good_Packets", .offset = 0x750 },
	{ .key = "Tx_Deferred_Packets", .offset = 0x754 },
	{ .key = "Tx_Late_Collision_Packets", .offset = 0x758 },
	{ .key = "Tx_Excessive_Collision_Packets", .offset = 0x75c },
	{ .key = "Tx_Carrier_Error_Packets", .offset = 0x760 },
	{ .key = "Tx_Octet_Count_Good", .offset = 0x764 },
	{ .key = "Tx_Packet_Count_Good", .offset = 0x768 },
	{ .key = "Tx_Excessive_Deferral_Error", .offset = 0x76c },
	{ .key = "Tx_Pause_Packets", .offset = 0x770 },
	{ .key = "Tx_VLAN_Packets_Good", .offset = 0x774 },
	{ .key = "Tx_OSize_Packets_Good", .offset = 0x778 },
	{ .key = "Rx_Packets_Count_Good_Bad", .offset = 0x780 },
	{ .key = "Rx_Octet_Count_Good_Bad", .offset = 0x784 },
	{ .key = "Rx_Octet_Count_Good", .offset = 0x788 },
	{ .key = "Rx_Broadcast_Packets_Good", .offset = 0x78c },
	{ .key = "Rx_Multicast_Packets_Good", .offset = 0x790 },
	{ .key = "Rx_CRC_Error_Packets", .offset = 0x794 },
	{ .key = "Rx_Alignment_Error_Packets", .offset = 0x798 },
	{ .key = "Rx_Runt_Error_Packets", .offset = 0x79c },
	{ .key = "Rx_Jabber_Error_Packets", .offset = 0x7a0 },
	{ .key = "Rx_Undersize_Packets_Good", .offset = 0x7a4 },
	{ .key = "Rx_Oversize_Packets_Good", .offset = 0x7a8 },
	{ .key = "Rx_64Octets_Packets_Good_Bad", .offset = 0x7ac },
	{ .key = "Rx_65To127Octets_Packets_Good_Bad", .offset = 0x7b0 },
	{ .key = "Rx_128To255Octets_Packets_Good_Bad", .offset = 0x7b4 },
	{ .key = "Rx_256To511Octets_Packets_Good_Bad", .offset = 0x7b8 },
	{ .key = "Rx_512To1023Octets_Packets_Good_Bad", .offset = 0x7bc },
	{ .key = "Rx_1024ToMaxOctets_Packets_Good_Bad", .offset = 0x7c0 },
	{ .key = "Rx_Unicast_Packets_Good", .offset = 0x7c4 },
	{ .key = "Rx_Length_Error_Packets", .offset = 0x7c8 },
	{ .key = "Rx_Out_Of_Range_Type_Packets", .offset = 0x7cc },
	{ .key = "Rx_Pause_Packets", .offset = 0x7d0 },
	{ .key = "Rx_FIFO_Overflow_Packets", .offset = 0x7d4 },
	{ .key = "Rx_VLAN_Packets_Good_Bad", .offset = 0x7d8 },
	{ .key = "Rx_Watchdog_Error_Packets", .offset = 0x7dc },
	{ .key = "Rx_Receive_Error_Packets", .offset = 0x7e0 },
	{ .key = "Rx_Control_Packets_Good", .offset = 0x7e4 },
	{ .key = "Tx_LPI_USEC_Cntr", .offset = 0x7ec },
	{ .key = "Tx_LPI_Tran_Cntr", .offset = 0x7f0 },
	{ .key = "Rx_LPI_USEC_Cntr", .offset = 0x7f4 },
	{ .key = "Rx_LPI_Tran_Cntr", .offset = 0x7f8 },
	{ .key = "RxIPv4_Good_Packets", .offset = 0x810 },
	{ .key = "RxIPv4_Header_Error_Packets", .offset = 0x814 },
	{ .key = "RxIPv4_No_Payload_Packets", .offset = 0x818 },
	{ .key = "RxIPv4_Fragmented_Packets", .offset = 0x81c },
	{ .key = "RxIPv4_UDP_Checksum_Disabled_Packets", .offset = 0x820 },
	{ .key = "RxIPv6_Good_Packets", .offset = 0x824 },
	{ .key = "RxIPv6_Header_Error_Packets", .offset = 0x828 },
	{ .key = "RxIPv6_No_Payload_Packets", .offset = 0x82c },
	{ .key = "RxUDP_Good_Packets", .offset = 0x830 },
	{ .key = "RxUDP_Error_Packets", .offset = 0x834 },
	{ .key = "RxTCP_Good_", .offset = 0x838 },
	{ .key = "RxTCP_Error_Packets", .offset = 0x83c },
	{ .key = "RxICMP_Good_Packets", .offset = 0x840 },
	{ .key = "RxICMP_Error_Packets", .offset = 0x844 },
	{ .key = "RxIPv4_Good_Octets", .offset = 0x850 },
	{ .key = "RxIPv4_Header_Error_Octets", .offset = 0x854 },
	{ .key = "RxIPv4_No_Payload_Octets", .offset = 0x858 },
	{ .key = "RxIPv4_Fragmented_Octets", .offset = 0x85c },
	{ .key = "RxIPv4_UDP_Checksum_Disable_Octets", .offset = 0x860 },
	{ .key = "RxIPv6_Good_Octets", .offset = 0x864 },
	{ .key = "RxIPv6_Header_Error_Octets", .offset = 0x868 },
	{ .key = "RxIPv6_No_Payload_Octets", .offset = 0x86c },
	{ .key = "RxUDP_Good_Octets", .offset = 0x870 },
	{ .key = "RxUDP_Error_Octets", .offset = 0x874 },
	{ .key = "RxTCP_Good_Octets", .offset = 0x878 },
	{ .key = "RxTCP_Error_Octets", .offset = 0x87c },
	{ .key = "RxICMP_Good_Octets", .offset = 0x880 },
	{ .key = "RxICMP_Error_Octets", .offset = 0x884 },
	{ .key = "Q0_Rx_Packets", .offset = 0 },
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 1
	{ .key = "Q1_Rx_Packets", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_RX_QUEUES > 2
	{ .key = "Q2_Rx_Packets", .offset = 0 },
#endif
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 3
	{ .key = "Q3_Rx_Packets", .offset = 0 },
#endif
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 4
	{ .key = "Q4_Rx_Packets", .offset = 0 },
#endif
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 5
	{ .key = "Q5_Rx_Packets", .offset = 0 },
#endif
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 6
	{ .key = "Q6_Rx_Packets", .offset = 0 },
#endif
#if CONFIG_ETH_DWC_EQOS_RX_QUEUES > 7
	{ .key = "Q7_Rx_Packets", .offset = 0 },
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	{ .key = "Constant_Gate_Control_Error", .offset = 0 },
	{ .key = "Head-Of-Line_Blocking_Queue", .offset = 0 },
	{ .key = "BTR_Error", .offset = 0 },
	{ .key = "BTR_Error_with_BTR_Renewal_Fail", .offset = 0 },
	{ .key = "BTR_Error_Loop", .offset = 0 },
	{ .key = "Q0_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 1
	{ .key = "Q1_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 2
	{ .key = "Q2_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 3
	{ .key = "Q3_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 4
	{ .key = "Q4_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 5
	{ .key = "Q5_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 6
	{ .key = "Q6_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#if  CONFIG_ETH_DWC_EQOS_TX_QUEUES > 7
	{ .key = "Q7_Head-Of-Line_Blocking_Frame_Size", .offset = 0 },
#endif
#endif
	{ .key = "MTL_TX_FIFO_ECC_Correctable_Error_Count", .offset = 0 },
	{ .key = "MTL_TX_FIFO_ECC_Uncorrectable_Error_Count", .offset = 0 },
	{ .key = "MTL_RX_FIFO_ECC_Correctable_Error_Count", .offset = 0 },
	{ .key = "MTL_RX_FIFO_ECC_Uncorrectable_Error_Count", .offset = 0 },
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	{ .key = "MTL_EST_Correctable_Error_Count", .offset = 0 },
	{ .key = "MTL_EST_Uncorrectable_Error_Count", .offset = 0 },
#endif
	{ .key = NULL, .offset = 0 }
};
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
#define MAC_ADDR_OFF_L 0
#define MAC_ADDR_OFF_H 1
#define ETH_TYPE_OFF 3
#define IP_PROTO_OFF 5
#define IP_PROTO_SHIFT 24
#define IPV4_FLAGS_DF 2
#define IPV4_FLAGS_MASK 7
#define IPV4_FRAGMENTOFF_MASK 0xFFF8
#define TCP_UDP_PORT_OFF 9
#define TCP_SYNC_OFF 11
#define IPP_PORT 631
#define LPD_PORT 515
#define TCP_SYN 0x02
#define TCP_SYN_SHIFT 24
#define RXP_DMA_CH_NO(n)  BIT(n)

struct eth_rxp_instruction {
	uint32_t match_data;
	uint32_t match_mask;
	union {
		struct {
			uint8_t accpt_frm          : 1;
			uint8_t rej_frm            : 1;
			uint8_t inv_match          : 1;
			uint8_t next_instr_ctrl    : 1;
			uint8_t reserved           : 4;
			uint8_t frm_off; /* strip off value for 4-bytes align */
			uint8_t ok_idx;
			uint8_t dma_ch;
		};
		uint32_t match_action;
	};
	uint32_t match_reserved;
} static_rxp_rule[] = {
	/* non-ipv4 packets */
	{.match_data = htons(NET_ETH_PTYPE_IP), .match_mask = UINT16_MAX,
	 .accpt_frm = 1, .inv_match = 1, .frm_off = ETH_TYPE_OFF,
	 .dma_ch = RXP_DMA_CH_NO(0)},
	/* ipv4 fragmented packets */
	{.match_data = htons(NET_ETH_PTYPE_IP), .match_mask = UINT16_MAX,
	 .next_instr_ctrl = 1, .frm_off = ETH_TYPE_OFF},
	{.match_data = IPV4_FLAGS_DF,
	 .match_mask = IPV4_FLAGS_MASK,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = 0,
	 .match_mask = IPV4_FRAGMENTOFF_MASK,
	 .rej_frm = 1, .inv_match = 1, .frm_off = IP_PROTO_OFF},
	/* udp/ipv4 packets with destination port 631 */
	{.match_data = IPPROTO_UDP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(IPP_PORT), .match_mask = UINT16_MAX,
	 .accpt_frm = 1, .frm_off = TCP_UDP_PORT_OFF,
	 .dma_ch = RXP_DMA_CH_NO(1)},
	/* udp/ipv4 packets with not destination port 631 */
	{.match_data = IPPROTO_UDP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(IPP_PORT), .match_mask = UINT16_MAX,
	 .accpt_frm = 1, .inv_match = 1, .frm_off = TCP_UDP_PORT_OFF,
	 .dma_ch = RXP_DMA_CH_NO(0)},
	/* non-tcp packets */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .accpt_frm = 1, .inv_match = 1, .frm_off = IP_PROTO_OFF,
	 .dma_ch = RXP_DMA_CH_NO(0)},
	/* tcp/ipv4 packets with destination port 515 and tcp sync */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(LPD_PORT), .match_mask = UINT16_MAX,
	 .next_instr_ctrl = 1, .frm_off = TCP_UDP_PORT_OFF},
	{.match_data = TCP_SYN << TCP_SYN_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .accpt_frm = 1, .frm_off = TCP_SYNC_OFF, .dma_ch = RXP_DMA_CH_NO(1)},
	/* tcp/ipv4 packets with destination port 631 and tcp sync */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(IPP_PORT), .match_mask = UINT16_MAX,
	 .next_instr_ctrl = 1, .frm_off = TCP_UDP_PORT_OFF},
	{.match_data = TCP_SYN << TCP_SYN_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .accpt_frm = 1, .frm_off = TCP_SYNC_OFF, .dma_ch = RXP_DMA_CH_NO(1)},
	/* tcp/ipv4 packets with not destination port 631 */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(IPP_PORT), .match_mask = UINT16_MAX,
	 .accpt_frm = 1, .inv_match = 1, .frm_off = TCP_UDP_PORT_OFF,
	 .dma_ch = RXP_DMA_CH_NO(0)},
	/* tcp/ipv4 packets with destination port 515 and not tcp sync */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(LPD_PORT), .match_mask = UINT16_MAX,
	 .next_instr_ctrl = 1, .frm_off = TCP_UDP_PORT_OFF},
	{.match_data = TCP_SYN << TCP_SYN_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .rej_frm = 1, .inv_match = 1, .frm_off = TCP_SYNC_OFF},
	/* tcp/ipv4 packets with destination port 631 and not tcp sync */
	{.match_data = IPPROTO_TCP << IP_PROTO_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .next_instr_ctrl = 1, .frm_off = IP_PROTO_OFF},
	{.match_data = htons(IPP_PORT), .match_mask = UINT16_MAX,
	 .next_instr_ctrl = 1, .frm_off = TCP_UDP_PORT_OFF},
	{.match_data = TCP_SYN << TCP_SYN_SHIFT,
	 .match_mask = UINT8_MAX << IP_PROTO_SHIFT,
	 .rej_frm = 1, .inv_match = 1, .frm_off = TCP_SYNC_OFF},
	};

/* RX parser entries will be structured in below sequence:
 * 1.) Dynamic entries: MAC address hash filtering
 *                      size = TOTAL_MAC_ADDRS_RXP_ENTRY - 2
 * 2.) Dynamic entries: MAC address perfect filtering
 *                      size = 2
 * 3.) Dynamic entries: VLAN tag filtering
 *                      size = TOTAL_VLAN_RXP_ENTRY
 * 4.) Dynamic entries: ... ... (other add-on e.g. IPv4)
 * 5.) Static entries: static_rxp_rule[]
 *                     size = RXP_STATIC_ENTRIES_COUNT
 */
#define RXP_STATIC_ENTRIES_COUNT ARRAY_SIZE(static_rxp_rule)
#define MAC_SUPRT_MAX_ADDRS_COUNT 7
#define TOTAL_MAC_ADDRS_RXP_ENTRY (MAC_SUPRT_MAX_ADDRS_COUNT * 2)
#ifdef CONFIG_NET_VLAN
#define TOTAL_VLAN_RXP_ENTRY CONFIG_NET_VLAN_COUNT
#else
#define TOTAL_VLAN_RXP_ENTRY 0
#endif /* CONFIG_NET_VLAN */
#define TOTAL_IPV4_RXP_ENTRY 0
#define RXP_DYNAMIC_ENTRIES_COUNT (TOTAL_MAC_ADDRS_RXP_ENTRY + \
				   TOTAL_VLAN_RXP_ENTRY + TOTAL_IPV4_RXP_ENTRY)
#define RXP_TOTAL_ENTRIES_COUNT (RXP_STATIC_ENTRIES_COUNT + \
				 RXP_DYNAMIC_ENTRIES_COUNT)
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */

static inline void dcache_invalidate(uintptr_t addr, uint32_t size)
{
#if defined(CONFIG_CACHE_MANAGEMENT)
#if USE_SEDI
	/* Align address to 32 bytes */
	uint32_t start_addr = addr & (uint32_t)~(ETH_DWC_EQOS_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sedi_core_inv_dcache_by_addr((uint32_t *)start_addr, size_full);
#else
	arch_dcache_range(addr, size, K_CACHE_INVD);
#endif
#endif
}

static inline void dcache_clean(uintptr_t addr, uint32_t size)
{
#if defined(CONFIG_CACHE_MANAGEMENT)
#if USE_SEDI
	/* Align address to 32 bytes */
	uint32_t start_addr = addr & (uint32_t)~(ETH_DWC_EQOS_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sedi_core_clean_dcache_by_addr((uint32_t *)start_addr, size_full);
#else
	arch_dcache_range(addr, size, K_CACHE_WB);
#endif
#endif
}

static void *eth_get_tx_desc(struct eth_runtime *ctxt, int q)
{
	volatile struct eth_tx_desc *pdesc;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *pedesc;
#endif
	int idx;

	k_sem_take(&ctxt->txq_lock[q], K_FOREVER);
	idx = ctxt->tdesc_ring_wr_ptr[q];
	pdesc = &ctxt->tx_desc[q][idx];
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	pedesc = &ctxt->enh_tx_desc[q][idx];
#endif
	idx = (idx + 1) % CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
	if (idx == ctxt->tdesc_ring_rd_ptr[q]) {
		k_sem_give(&ctxt->txq_lock[q]);
		return NULL;
	}
	ctxt->tdesc_ring_wr_ptr[q] = idx;
	k_sem_give(&ctxt->txq_lock[q]);

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	return ctxt->tbs_enabled[q] ? (void *)pedesc : (void *)pdesc;
#else
	return (void *)pdesc;
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
}

static void *eth_put_tx_desc(struct eth_runtime *ctxt, int q)
{
	volatile struct eth_tx_desc *pdesc;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *pedesc;
#endif
	int idx;

	k_sem_take(&ctxt->txq_lock[q], K_FOREVER);
	idx = ctxt->tdesc_ring_rd_ptr[q];
	pdesc = &ctxt->tx_desc[q][idx];
	dcache_invalidate((uintptr_t)pdesc, sizeof(*pdesc));

#ifndef CONFIG_ETH_DWC_EQOS_TBS
	if (!pdesc->tdes3.wb.own) {
		idx++;
		idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
		ctxt->tdesc_ring_rd_ptr[q] = idx;
	} else {
		pdesc = NULL;
	}
#else
	pedesc = &ctxt->enh_tx_desc[q][idx];
	dcache_invalidate((uintptr_t)pedesc, sizeof(*pedesc));

	if ((!pdesc->tdes3.wb.own) && (!pedesc->tdes3.wb.own)) {
		idx++;
		idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
		ctxt->tdesc_ring_rd_ptr[q] = idx;
	} else {
		pdesc = NULL;
		pedesc = NULL;
	}
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
	k_sem_give(&ctxt->txq_lock[q]);

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	return ctxt->tbs_enabled[q] ? (void *)pedesc : (void *)pdesc;
#else
	return (void *)pdesc;
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
}

static int eth_completed_tx_desc_num(struct eth_runtime *ctxt, int q)
{
	int idx = ctxt->tdesc_ring_rd_ptr[q];
	volatile struct eth_tx_desc *pdesc;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *pedesc;
#endif
	int count = 0;

	while (idx != ctxt->tdesc_ring_wr_ptr[q]) {
		pdesc = &ctxt->tx_desc[q][idx];
		dcache_invalidate((uintptr_t)pdesc, sizeof(*pdesc));
#ifdef CONFIG_ETH_DWC_EQOS_TBS
		pedesc = &ctxt->enh_tx_desc[q][idx];
		dcache_invalidate((uintptr_t)pedesc, sizeof(*pedesc));

		if ((pdesc->tdes3.wb.own) || (pedesc->tdes3.wb.own)) {
			break;
		}
#else
		if (pdesc->tdes3.wb.own) {
			break;
		}
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
		idx++;
		idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
		count++;
	}

	return count;
}

static void *eth_get_rx_desc(struct eth_runtime *ctxt, int q)
{
	volatile struct eth_rx_desc *pdesc;
	int idx;

	k_sem_take(&ctxt->rxq_lock[q], K_FOREVER);
	idx = ctxt->rdesc_ring_wr_ptr[q];
	pdesc = &ctxt->rx_desc[q][idx++];
	idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
	if (idx == ctxt->rdesc_ring_rd_ptr[q]) {
		k_sem_give(&ctxt->rxq_lock[q]);
		return NULL;
	}
	ctxt->rdesc_ring_wr_ptr[q] = idx;
	k_sem_give(&ctxt->rxq_lock[q]);

	return (void *)pdesc;
}

static void *eth_put_rx_desc(struct eth_runtime *ctxt, int q)
{
	volatile struct eth_rx_desc *pdesc;
	int idx;

	k_sem_take(&ctxt->rxq_lock[q], K_FOREVER);
	idx = ctxt->rdesc_ring_rd_ptr[q];
	pdesc = &ctxt->rx_desc[q][idx];

	dcache_invalidate((uintptr_t)pdesc, sizeof(*pdesc));

	if (!pdesc->rdes3.wb.own) {
		idx++;
		idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
		ctxt->rdesc_ring_rd_ptr[q] = idx;
	} else {
		pdesc = NULL;
	}
	k_sem_give(&ctxt->rxq_lock[q]);

	return (void *)pdesc;
}

static int eth_completed_rx_desc_num(struct eth_runtime *ctxt, int q)
{
	int idx = ctxt->rdesc_ring_rd_ptr[q];
	int count = 0;

	while (idx != ctxt->rdesc_ring_wr_ptr[q]) {
		dcache_invalidate((uintptr_t)(&ctxt->rx_desc[q][idx]),
				  sizeof(ctxt->rx_desc[q][idx]));

		if (ctxt->rx_desc[q][idx].rdes3.wb.own) {
			break;
		}
		idx++;
		idx %= CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE;
		count++;
	}

#ifdef CONFIG_NET_PKT_TIMESTAMP
	/* Make sure descriptor count in pair */
	count &= ~(uint32_t)1;
#endif /* CONFIG_NET_PKT_TIMESTAMP */

	return count;
}

static inline uint32_t eth_read(mm_reg_t base_addr, uint32_t offset)
{
	return sys_read32(base_addr + offset);
}

static inline void eth_write(mm_reg_t base_addr, uint32_t offset,
			     uint32_t val)
{
	sys_write32(val, base_addr + offset);
}

static int dwc_busy_wait(mm_reg_t base_addr, uint32_t reg_addr, uint32_t reg_bit,
			 char *s, uint32_t retries, bool bit_set_wait)
{
	/* Validating the bit logic by AND-ing with bitmask then '!!' to get
	 * the truth / false result. If bit_set_wait is set, XOR the bit result
	 * with bit_set_wait will get the inverted result, else if bit_set_wait
	 * is not set, XOR-ing will remain the same result.
	 */
	while (!!((eth_read(base_addr, reg_addr) & reg_bit) ^ bit_set_wait)) {
		if (retries-- == 0) {
			LOG_ERR("%s busy wait timeout", s);
			return -ETIMEDOUT;
		}

		k_busy_wait(10 * 1000);
	}

	return 0;
}

static int dwc_mdio_send(struct phy_device *phydev,
			 union mdio_frm_flds fld_data,
			 uint16_t data, bool c45)
{
	struct eth_runtime *context = CONTAINER_OF(phydev, struct eth_runtime,
						   phy_dev);
	uint32_t mdio_addr, mdio_data = 0;
	int retval;

	if (c45) {
		mdio_addr = MAC_MDIO_CLAUSE_45_PHY_EN;
		mdio_addr |= (fld_data.c45_frm_fld.portaddr <<
			      MAC_MDIO_PA_SHIFT) & MAC_MDIO_PA_MASK;
		mdio_addr |= (fld_data.c45_frm_fld.devaddr <<
			      MAC_MDIO_RDA_SHIFT) & MAC_MDIO_RDA_MASK;
		mdio_data = (fld_data.c45_frm_fld.regaddr <<
			     MAC_MDIO_RA_SHIFT) & MAC_MDIO_RA_MASK;
	} else {
		mdio_addr = (fld_data.c22_frm_fld.phyaddr <<
			     MAC_MDIO_PA_SHIFT) & MAC_MDIO_PA_MASK;
		mdio_addr |= (fld_data.c22_frm_fld.regaddr <<
			      MAC_MDIO_RDA_SHIFT) & MAC_MDIO_RDA_MASK;
	}
	mdio_addr |= MAC_MDIO_GMII_BUSY | MAC_MDIO_GMII_OPR_CMD_WRITE |
		     context->mdio_csr_clk;
	mdio_data |= (uint32_t)data;

	retval = dwc_busy_wait(context->base_addr, MAC_MDIO_ADDRESS,
			       MAC_MDIO_GMII_BUSY, "mdio", 100, false);
	if (retval) {
		return retval;
	}
	eth_write(context->base_addr, MAC_MDIO_DATA, mdio_data);
	eth_write(context->base_addr, MAC_MDIO_ADDRESS, mdio_addr);

	return dwc_busy_wait(context->base_addr, MAC_MDIO_ADDRESS,
			     MAC_MDIO_GMII_BUSY, "mdio", 100, false);
}

static int dwc_mdio_read(struct phy_device *phydev,
			 union mdio_frm_flds fld_data,
			 uint16_t *data, bool c45)
{
	struct eth_runtime *context = CONTAINER_OF(phydev, struct eth_runtime,
						   phy_dev);
	uint32_t mdio_addr, mdio_data = 0;
	int retval;

	if (c45) {
		mdio_addr = MAC_MDIO_CLAUSE_45_PHY_EN;
		mdio_addr |= (fld_data.c45_frm_fld.portaddr <<
			      MAC_MDIO_PA_SHIFT) & MAC_MDIO_PA_MASK;
		mdio_addr |= (fld_data.c45_frm_fld.devaddr <<
			      MAC_MDIO_RDA_SHIFT) & MAC_MDIO_RDA_MASK;
		mdio_data = (fld_data.c45_frm_fld.regaddr <<
			     MAC_MDIO_RA_SHIFT) & MAC_MDIO_RA_MASK;
	} else {
		mdio_addr = (fld_data.c22_frm_fld.phyaddr <<
			     MAC_MDIO_PA_SHIFT) & MAC_MDIO_PA_MASK;
		mdio_addr |= (fld_data.c22_frm_fld.regaddr <<
			      MAC_MDIO_RDA_SHIFT) & MAC_MDIO_RDA_MASK;
	}
	mdio_addr |= MAC_MDIO_GMII_BUSY | MAC_MDIO_GMII_OPR_CMD_READ |
		     context->mdio_csr_clk;

	retval = dwc_busy_wait(context->base_addr, MAC_MDIO_ADDRESS,
			       MAC_MDIO_GMII_BUSY, "mdio", 100, false);
	if (retval) {
		return retval;
	}
	eth_write(context->base_addr, MAC_MDIO_DATA, mdio_data);
	eth_write(context->base_addr, MAC_MDIO_ADDRESS, mdio_addr);
	retval = dwc_busy_wait(context->base_addr, MAC_MDIO_ADDRESS,
			       MAC_MDIO_GMII_BUSY, "mdio", 100, false);
	if (retval) {
		return retval;
	}

	*data = eth_read(context->base_addr, MAC_MDIO_DATA);

	return 0;
}

static inline int eth_reset(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t retries = 10;  /* retry up to 100ms (10 x 10ms poll interval) */
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_MODE);
	reg_val |= DMA_MD_SWR;
	eth_write(base_addr, DMA_MODE, reg_val);
	k_sleep(K_MSEC(4)); /* No readback immediately after set reset bit */
	while (eth_read(base_addr, DMA_MODE) & DMA_MD_SWR) {
		if (retries-- == 0) {
			LOG_ERR("MAC reset timeout");
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(10));  /* 10ms polling interval */
	}

	return 0;
}

#ifdef CONFIG_ETH_DWC_EQOS_INTEL_PSE_PLATDATA
int intel_pse_platdata(const struct device *port)
{
	struct eth_runtime *context = port->data;
	int ret = 0;

	switch (pse_gbe_get_phyif(context->port_id)) {
	case PHY_RGMII:
		context->phy_dev.interface = PHY_INTERFACE_RGMII;
		context->use_xpcs = 0;
		break;
	case PHY_SGMII:
	case PHY_SGMII_PLUS:
		context->phy_dev.interface = PHY_INTERFACE_SGMII;
		context->use_xpcs = 1;
		break;
	case PHY_NOT_CONNECTED:
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_INTEL_PSE_PLATDATA */

static inline int eth_get_hw_capabilities(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val, txq, rxq;

#if defined(CONFIG_ETH_DWC_EQOS_QBV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
	struct tsn_hw_cap *tsn_cap;
#endif

	/* Obtain HW TX & RX fifo size */
	reg_val = eth_read(base_addr, MAC_HW_FEATURE1);
	if (!context->txfifosz) {
		context->txfifosz = reg_val & MAC_HW_FEAT1_TXFIFOSZ_MASK;
		context->txfifosz >>= MAC_HW_FEAT1_TXFIFOSZ_SHIFT;
		context->txfifosz = 1 << (context->txfifosz + 7);
	}

	if (!context->rxfifosz) {
		context->rxfifosz = reg_val & MAC_HW_FEAT1_RXFIFOSZ_MASK;
		context->rxfifosz = 1 << (context->rxfifosz + 7);
	}

#ifdef CONFIG_ETH_DWC_EQOS_QAV
	/* Obtain AV feature support */
	if (reg_val & MAC_HW_FEAT1_AVSEL) {
		context->flags |= ETH_DEV_FLAGS_QAV;
	}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */

	/* Obtain HW MAC Hash Table Size */
	context->hashtblsz = (reg_val & MAC_HW_FEAT1_HASHTBLSZ_MASK) >>
				MAC_HW_FEAT1_HASHTBLSZ_SHIFT;

	/* Obtain HW TX & RX checksum offload enablement */
	reg_val = eth_read(base_addr, MAC_HW_FEATURE0);
	if (reg_val & MAC_HW_FEAT0_TXCOESEL) {
		context->flags |= ETH_DEV_FLAGS_TX_CSUM_OFFLOAD;
	}
	if (reg_val & MAC_HW_FEAT0_RXCOESEL) {
		context->flags |= ETH_DEV_FLAGS_RX_CSUM_OFFLOAD;
	}

	/* Obtain HW MAC Perfect Filter Count */
	context->addrcnt = reg_val & MAC_HW_FEAT0_ADDMACADRSEL_MASK;
	context->addrcnt >>= MAC_HW_FEAT0_ADDMACADRSEL_SHIFT;

#ifdef CONFIG_NET_PKT_TIMESTAMP
	if (reg_val & MAC_HW_FEAT0_TSSEL) {
		context->flags |= ETH_DEV_FLAGS_TSSEL;
	}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

#if defined(CONFIG_ETH_DWC_EQOS_QBV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
	dwmac_tsn_init(port);
	dwmac_get_tsn_hwcap(&tsn_cap);
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	if (tsn_cap->est_support) {
		context->flags |= ETH_DEV_FLAGS_QBV;
	}
	if (tsn_cap->gcl_depth < context->estparam.gcl_depth) {
		LOG_WRN("Kconfig setting greater than HW available "
			"gcl depth: %d", tsn_cap->gcl_depth);
		context->estparam.gcl_depth = tsn_cap->gcl_depth;
	}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	if (tsn_cap->fpe_support) {
		context->flags |= ETH_DEV_FLAGS_QBU;
	}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

	/* Obtain HW TX & RX queues count */
	reg_val = eth_read(base_addr, MAC_HW_FEATURE2);
	txq = reg_val & MAC_HW_FEAT2_TXQCNT_MASK;
	txq >>= MAC_HW_FEAT2_TXQCNT_SHIFT;
	if (txq < context->txqnum - 1) {
		LOG_ERR("Kconfig setting greater than HW available "
			"tx queues: %d", txq);
		return -EINVAL;
	}
	rxq = reg_val & MAC_HW_FEAT2_RXQCNT_MASK;
	if (rxq < context->rxqnum - 1) {
		LOG_ERR("Kconfig setting greater than HW available "
			"rx queues: %d", rxq);
		return -EINVAL;
	}

	/* Obtain VLAN entry count */
	reg_val = eth_read(base_addr, MAC_HW_FEATURE3);
	switch (reg_val & MAC_HW_FEAT3_NRVF_MASK) {
	case MAC_HW_FEAT3_NRVF_4:
		context->vlancnt = 4;
		break;
	case MAC_HW_FEAT3_NRVF_8:
		context->vlancnt = 8;
		break;
	case MAC_HW_FEAT3_NRVF_16:
		context->vlancnt = 16;
		break;
	case MAC_HW_FEAT3_NRVF_24:
		context->vlancnt = 24;
		break;
	case MAC_HW_FEAT3_NRVF_32:
		context->vlancnt = 32;
		break;
	default:
		context->vlancnt = 0;
		break;
	}

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	/* Obtain HW TBS support */
	if (reg_val & MAC_HW_FEAT3_TBSSEL) {
		context->flags |= ETH_DEV_FLAGS_TBS;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	if (reg_val & MAC_HW_FEAT3_FRPSEL) {
		/* Obtain RX Parser entry count */
		switch ((reg_val & MAC_HW_FEAT3_FRPES_MASK) >>
			MAC_HW_FEAT3_FRPES_SHIFT) {
		case MAC_HW_FEAT3_FRPES_64ENTR:
			context->frp_entry_sz = 64;
			break;
		case MAC_HW_FEAT3_FRPES_128ENTR:
			context->frp_entry_sz = 128;
			break;
		case MAC_HW_FEAT3_FRPES_256ENTR:
			context->frp_entry_sz = 256;
			break;
		default:
			context->frp_entry_sz = 0;
			break;
		}

		/* Obtain RX Parser buffer size */
		switch ((reg_val & MAC_HW_FEAT3_FRPBS_MASK) >>
			MAC_HW_FEAT3_FRPBS_SHIFT) {
		case MAC_HW_FEAT3_FRPBS_64BYTS:
			context->frp_buf_sz = 64;
			break;
		case MAC_HW_FEAT3_FRPBS_128BYTS:
			context->frp_buf_sz = 128;
			break;
		case MAC_HW_FEAT3_FRPBS_256BYTS:
			context->frp_buf_sz = 256;
			break;
		default:
			context->frp_buf_sz = 0;
			break;
		}
	} else {
		LOG_WRN("HW has no RX parser support");
		context->frp_buf_sz = 0;
		context->frp_entry_sz = 0;
	}

	/* Check the intended use size vs available size */
	if (context->frp_entry_sz < RXP_DYNAMIC_ENTRIES_COUNT) {
		LOG_ERR("Total dynamic entries use for RX parser has exceeded "
			"available size");
		return -EINVAL;
	} else if (context->frp_entry_sz < RXP_TOTAL_ENTRIES_COUNT) {
		LOG_WRN("Total of intended use RX parser's entries has "
			"exceeded available size, perform truncate");
	}
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */

	return 0;
}

static inline void eth_tx_dma_ctrl(struct eth_runtime *ctxt, int queue, bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_TX_CONTROL_CH(queue));
	if (on) {
		reg_val |= DMA_CH_TX_CTRL_ST;
	} else {
		reg_val &= ~DMA_CH_TX_CTRL_ST;
	}
	eth_write(base_addr, DMA_TX_CONTROL_CH(queue), reg_val);
}

static inline void eth_rx_dma_ctrl(struct eth_runtime *ctxt, int queue, bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_RX_CONTROL_CH(queue));
	if (on) {
		reg_val |= DMA_CH_RX_CTRL_SR;
	} else {
		reg_val &= ~DMA_CH_RX_CTRL_SR;
	}
	eth_write(base_addr, DMA_RX_CONTROL_CH(queue), reg_val);
}

static inline void eth_tx_mac_ctrl(struct eth_runtime *ctxt, bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, MAC_CONFIGURATION);
	if (on) {
		reg_val |= MAC_CONF_TE;
	} else {
		reg_val &= ~MAC_CONF_TE;
	}
	eth_write(base_addr, MAC_CONFIGURATION, reg_val);
}

static inline void eth_rx_mac_ctrl(struct eth_runtime *ctxt, bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, MAC_CONFIGURATION);
	if (on) {
		reg_val |= MAC_CONF_RE;
	} else {
		reg_val &= ~MAC_CONF_RE;
	}
	eth_write(base_addr, MAC_CONFIGURATION, reg_val);
}

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
static inline int dwc_rxp_indir_acc(uint32_t base_addr, bool wr_rd, int entry_idx,
				    struct eth_rxp_instruction *rxp_instr)
{
	int i, retval;
	uint32_t reg_val;
	uint32_t *rxp_instr_array = (uint32_t *)rxp_instr;

	for (i = 0; i < MTL_RXP_DWORD_PER_INSTR; i++) {
		if (wr_rd) {
			/* Write RX parser entry dword */
			reg_val = rxp_instr_array[i];
			eth_write(base_addr, MTL_RXP_INDRT_ACC_DATA, reg_val);
		}

		/* Program RX parser indirect access control status */
		reg_val = MTL_RXP_INDACC_CTRLSTS_STARTBUSY;
		if (wr_rd) {
			reg_val |= MTL_RXP_INDACC_CTRLSTS_WRRDN;
		}

		reg_val |= (entry_idx * MTL_RXP_DWORD_PER_INSTR + i) &
				MTL_RXP_INDACC_CTRLSTS_ADDR_MASK;
		eth_write(base_addr, MTL_RXP_INDRT_ACC_CTRL_STATS,
			  reg_val);

		/* Wait for indirect access completion */
		retval = dwc_busy_wait(base_addr, MTL_RXP_INDRT_ACC_CTRL_STATS,
				       MTL_RXP_INDACC_CTRLSTS_STARTBUSY,
				       "RX parser", 10, false);
		if (retval) {
			return retval;
		}

		if (!wr_rd) {
			/* Read RX parser entry dword */
			rxp_instr_array[i] = eth_read(base_addr,
						      MTL_RXP_INDRT_ACC_DATA);
		}
	}

	return 0;
}

static inline int eth_set_static_rx_parser(struct eth_runtime *ctxt)
{
	uint32_t base_addr = ctxt->base_addr;
	int i, retval, rule_num = 0, next_rule_idx;
	int max_statc_entry_num = RXP_STATIC_ENTRIES_COUNT;

	/* Exit if no flexible rx parser support */
	if (ctxt->frp_buf_sz == 0 || ctxt->frp_entry_sz == 0) {
		LOG_ERR("RX parser not configured");
		return -ECANCELED;
	}

	/* Initialize dynamic RX parser entries */
	for (i = 0; i < RXP_DYNAMIC_ENTRIES_COUNT; i++) {
		struct eth_rxp_instruction dynm_rxp_instr = {
							.match_data = 0,
							.match_mask = 0,
							.match_action = 0,
							.match_reserved = 0};

		dynm_rxp_instr.ok_idx = i + 1;
		retval = dwc_rxp_indir_acc(base_addr, true, i,
					   &dynm_rxp_instr);
		if (retval) {
			return retval;
		}
	}

	/* Only program RX parser rules within available entries */
	if (max_statc_entry_num + RXP_DYNAMIC_ENTRIES_COUNT >
	    ctxt->frp_entry_sz) {
		max_statc_entry_num = ctxt->frp_entry_sz
						- RXP_DYNAMIC_ENTRIES_COUNT;
		while (!static_rxp_rule[max_statc_entry_num - 1].accpt_frm &&
		       !static_rxp_rule[max_statc_entry_num - 1].rej_frm) {
			if (--max_statc_entry_num == 0) {
				LOG_ERR("No valid static data entry for "
					"RX parser");
				return 0;
			}
		}
	}

	for (i = 0, next_rule_idx = 0; i < max_statc_entry_num; i++) {
		/* Check data validity */
		uint32_t parser_act = static_rxp_rule[i].match_action &
				   (MTL_RXP_INSTR_ACCPT | MTL_RXP_INSTR_REJT |
				    MTL_RXP_INSTR_NEXT);

		if (parser_act != MTL_RXP_INSTR_NEXT &&
		    parser_act != MTL_RXP_INSTR_REJT &&
		    parser_act != MTL_RXP_INSTR_ACCPT) {
			LOG_ERR("Invalid RX parser data entry %d", i);
			return -EINVAL;
		}

		/* Check buffer limit */
		if (static_rxp_rule[i].frm_off * MTL_RXP_INSTR_FRM_OFF_UNIT >
		    ctxt->frp_buf_sz) {
			LOG_ERR("Frame offset exceeded RX parser buffer limit"
				"(%d)", ctxt->frp_buf_sz);
			return -EINVAL;
		}

		/* Get the next rule entry index */
		if (next_rule_idx <= i) {
			for (next_rule_idx = i;
			     next_rule_idx < max_statc_entry_num;
			     next_rule_idx++) {
				if (static_rxp_rule[next_rule_idx].accpt_frm ||
				    static_rxp_rule[next_rule_idx].rej_frm) {
					next_rule_idx++;
					break;
				}
			}
		}

		/* Assign next rule index */
		static_rxp_rule[i].ok_idx = next_rule_idx
						+ RXP_DYNAMIC_ENTRIES_COUNT;
		/* Program RX parser entry */
		retval = dwc_rxp_indir_acc(base_addr, true,
					   i + RXP_DYNAMIC_ENTRIES_COUNT,
					   &static_rxp_rule[i]);
		if (retval) {
			return retval;
		}

		/* Counting of RX Parser Rules being programmed */
		if (static_rxp_rule[i].accpt_frm ||
		    static_rxp_rule[i].rej_frm) {
			rule_num++;
		}
	}

	LOG_INF("RX parser configured %d rules", rule_num);
	return 0;
}

static int eth_set_dynamic_rx_parser(struct eth_runtime *ctxt,
				     int offset, int length,
				     struct eth_rxp_instruction *dynm_rxp_rule)
{
	uint32_t base_addr = ctxt->base_addr;
	int retval, i;
	uint32_t reg_val;

	/* Disable MAC layer RX */
	eth_rx_mac_ctrl(ctxt, false);
	/* Disable RX parser */
	reg_val = eth_read(base_addr, MTL_OPERATION_MODE);
	reg_val &= ~MTL_OPR_MD_FRPE;
	eth_write(base_addr, MTL_OPERATION_MODE, reg_val);
	/* Wait for RX parser idle */
	retval = dwc_busy_wait(base_addr, MTL_RXP_CTRL_STATS,
			       MTL_RXP_CTRL_STATS_RXPI, "RX parser idle", 10,
			       true);
	if (retval) {
		return retval;
	}

	for (i = offset; i < offset + length; i++) {
		retval = dwc_rxp_indir_acc(base_addr, true, i,
					   &dynm_rxp_rule[i]);
		if (retval) {
			return retval;
		}
	}

	/* Enable back RX parser */
	reg_val |= MTL_OPR_MD_FRPE;
	eth_write(base_addr, MTL_OPERATION_MODE, reg_val);
	/* Enable back the MAC layer RX */
	eth_rx_mac_ctrl(ctxt, true);
	return 0;
}

static int eth_rxp_mac_addr(struct eth_runtime *ctxt, int idx, bool del,
			    uint8_t addr[6])
{
	uint32_t base_addr = ctxt->base_addr;
	int i, retval = 0;
	struct eth_rxp_instruction rxp_mac_addr[TOTAL_MAC_ADDRS_RXP_ENTRY];

	/* Read entry from MAC addr section (1st block) */
	for (i = 0; i < TOTAL_MAC_ADDRS_RXP_ENTRY; i++) {
		retval = dwc_rxp_indir_acc(base_addr, false, i,
					   &rxp_mac_addr[i]);
		if (retval) {
			return retval;
		}
	}

	if (del) {
		/* Search for the entry to delete */
		for (i = 0; i < TOTAL_MAC_ADDRS_RXP_ENTRY; i++) {
			if (rxp_mac_addr[i].match_data == *(uint32_t *)&addr[0] &&
			    rxp_mac_addr[i + 1].match_data == (
			    *(uint32_t *)&addr[4] & UINT16_MAX)) {
				rxp_mac_addr[i].match_data = 0;
				rxp_mac_addr[i].match_mask = 0;
				rxp_mac_addr[i].match_action = 0;
				rxp_mac_addr[i].ok_idx = i + 1;
				rxp_mac_addr[i + 1].match_data = 0;
				rxp_mac_addr[i + 1].match_mask = 0;
				rxp_mac_addr[i + 1].match_action = 0;
				rxp_mac_addr[i + 1].ok_idx = i + 2;
				break;
			}
		}

		/* No entry found then exit */
		if (i >= TOTAL_MAC_ADDRS_RXP_ENTRY) {
			LOG_ERR("Unable to remove, MAC addr (%02X %02X %02X "
				"%02X %02X %02X) not found in RX parser",
				addr[0], addr[1], addr[2], addr[3], addr[4],
				addr[5]);
			return -ENOENT;
		}
	} else {
		/* Replace perfect MAC addr filter */
		if (!idx) {
			i = TOTAL_MAC_ADDRS_RXP_ENTRY - 2;
			/* Perfect filter:- if not match then reject it */
			rxp_mac_addr[i].rej_frm = 1;
			rxp_mac_addr[i].inv_match = 1;
			rxp_mac_addr[i].frm_off = MAC_ADDR_OFF_L;
			rxp_mac_addr[i + 1].rej_frm = 1;
			rxp_mac_addr[i + 1].inv_match = 1;
			rxp_mac_addr[i + 1].frm_off = MAC_ADDR_OFF_H;
		} else {
			/* Look for empty slot */
			for (i = 0; i < TOTAL_MAC_ADDRS_RXP_ENTRY - 2; i += 2) {
				if (!rxp_mac_addr[i].match_mask) {
					break;
				}
			}

			/* Check if exceeded available slot */
			if (i >= TOTAL_MAC_ADDRS_RXP_ENTRY - 2) {
				LOG_ERR("Exceeded RX parser defined available "
					"MAC addr filter entries");
				return -ENOENT;
			}

			/* Hash filter:- if match then accept it */
			rxp_mac_addr[i].next_instr_ctrl = 1;
			rxp_mac_addr[i].frm_off = MAC_ADDR_OFF_L;
			rxp_mac_addr[i].ok_idx = i + 2;
			rxp_mac_addr[i + 1].accpt_frm = 1;
			rxp_mac_addr[i + 1].frm_off = MAC_ADDR_OFF_H;
			rxp_mac_addr[i + 1].ok_idx = i + 1;
			rxp_mac_addr[i + 1].dma_ch = 0;
		}

		/* Assign new MAC addr value */
		rxp_mac_addr[i].match_data = *(uint32_t *)&addr[0];
		rxp_mac_addr[i].match_mask = UINT32_MAX;
		rxp_mac_addr[i + 1].match_data = *(uint32_t *)&addr[4] &
							UINT16_MAX;
		rxp_mac_addr[i + 1].match_mask = UINT16_MAX;
	}

	retval = eth_set_dynamic_rx_parser(ctxt, 0, TOTAL_MAC_ADDRS_RXP_ENTRY,
					   rxp_mac_addr);
	return retval;
}

#ifdef CONFIG_NET_VLAN
static int eth_rxp_vlan(struct eth_runtime *ctxt, bool en, uint16_t tag)
{
	uint32_t base_addr = ctxt->base_addr;
	int i, retval = 0;
	struct eth_rxp_instruction rxp_vlan_tag[TOTAL_VLAN_RXP_ENTRY];

	/* Read entry from VLAN section (2nd block) */
	for (i = 0; i < TOTAL_VLAN_RXP_ENTRY; i++) {
		retval = dwc_rxp_indir_acc(base_addr, false,
					   i + TOTAL_MAC_ADDRS_RXP_ENTRY,
					   &rxp_vlan_tag[i]);
		if (retval) {
			return retval;
		}
	}

	if (en) {  /* Add VLAN tag to filtering */
		/* Look for empty slot */
		for (i = 0; i < TOTAL_VLAN_RXP_ENTRY; i++) {
			if (!rxp_vlan_tag[i].match_mask) {
				break;
			}
		}

		/* Check if exceeded available slot */
		if (i >= TOTAL_VLAN_RXP_ENTRY) {
			LOG_ERR("Exceeded RX parser defined available "
				"VLAN tag filter entries");
			return -ENOENT;
		}

		/* Assign new VLAN tag value */
		rxp_vlan_tag[i].match_data = htons(NET_ETH_PTYPE_VLAN) |
						((uint32_t)htons(tag) <<
						 sizeof(uint16_t));
		rxp_vlan_tag[i].match_mask = UINT16_MAX |
						(NET_VLAN_TAG_UNSPEC <<
						 sizeof(uint16_t));
		rxp_vlan_tag[i].inv_match = 1;
		rxp_vlan_tag[i].rej_frm = 1;
		rxp_vlan_tag[i].frm_off = ETH_TYPE_OFF;
		rxp_vlan_tag[i].ok_idx = TOTAL_MAC_ADDRS_RXP_ENTRY
						+ TOTAL_VLAN_RXP_ENTRY;
		/* Include new entry onto all previous entries */
		if (i != 0) {
			rxp_vlan_tag[i - 1].rej_frm = 0;
			rxp_vlan_tag[i - 1].next_instr_ctrl = 1;
		}
	} else {  /* Remove VLAN tag from filtering */
		/* Look for matching entry */
		for (i = 0; i < TOTAL_VLAN_RXP_ENTRY; i++) {
			if ((rxp_vlan_tag[i].match_data & (UINT16_MAX <<
			    sizeof(uint16_t))) == ((uint32_t)htons(tag) <<
			    sizeof(uint16_t))) {
				break;
			}
		}

		if (i >= TOTAL_VLAN_RXP_ENTRY) {  /* No tag found */
			LOG_ERR("No matching VLAN tag entry in RX parser");
			return -ENOENT;
		} else if (rxp_vlan_tag[i].next_instr_ctrl) {
			/* Delete the middle entry by shifting those behind
			 * entries to front
			 */
			for (; i < TOTAL_VLAN_RXP_ENTRY - 1 &&
			     rxp_vlan_tag[i].next_instr_ctrl; i++) {
				rxp_vlan_tag[i] = rxp_vlan_tag[i + 1];
			}

			/* Empty the last entry  */
			if (i >= TOTAL_VLAN_RXP_ENTRY - 1) {
				rxp_vlan_tag[i].match_data = 0;
				rxp_vlan_tag[i].match_mask = 0;
				rxp_vlan_tag[i].match_action = 0;
				rxp_vlan_tag[i].ok_idx = i + 1;
			}
		} else {  /* Delete the last in the list */
			/* Remove current entry */
			rxp_vlan_tag[i].match_data = 0;
			rxp_vlan_tag[i].match_mask = 0;
			rxp_vlan_tag[i].match_action = 0;
			rxp_vlan_tag[i].ok_idx = i + 1;
			/* Exclude from previous entries */
			if (i != 0) {
				rxp_vlan_tag[i - 1].rej_frm = 1;
				rxp_vlan_tag[i - 1].next_instr_ctrl = 0;
			}
		}
	}

	retval = eth_set_dynamic_rx_parser(ctxt, TOTAL_MAC_ADDRS_RXP_ENTRY,
					   TOTAL_VLAN_RXP_ENTRY,
					   rxp_vlan_tag);
	return retval;
}
#endif /* CONFIG_NET_VLAN */
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */

#ifdef CONFIG_ETH_DWC_EQOS_PTP
static inline void eth_mac_get_egress_latency(struct eth_runtime *ctxt,
					      int32_t *latency)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t val;

	/* Read the egress latency value */
	val = eth_read(base_addr, MAC_TIMESTAMP_EGRESS_LATENCY);
	*latency = (val & MAC_TIMESTAMP_ETLNS_MASK) >>
		    MAC_TIMESTAMP_ETLNS_SHIFT;
	/* CDC sync correction */
	*latency -= ONE_SEC_IN_NANOSEC / ctxt->ptp_clock_rate * 2;
}

static inline void eth_mac_get_ingress_latency(struct eth_runtime *ctxt,
					       int32_t *latency)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t val;

	/* Read the ingress latency value */
	val = eth_read(base_addr, MAC_TIMESTAMP_INGRESS_LATENCY);
	*latency = (val & MAC_TIMESTAMP_ITLNS_MASK) >>
		    MAC_TIMESTAMP_ITLNS_SHIFT;
	/* CDC sync correction */
	*latency += ONE_SEC_IN_NANOSEC / ctxt->ptp_clock_rate * 2;
}
#endif  /* CONFIG_ETH_DWC_EQOS_PTP */

/* Configure MAC link speed and duplex mode */
static void eth_mac_config_mode(struct eth_runtime *context)
{
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, MAC_CONFIGURATION);

	reg_val &= INV_MAC_CONF_SPD;
	switch (context->link_speed) {
	case 100:
		reg_val |= MAC_CONF_SPD_100MHZ;
		break;
	case 1000:
		reg_val |= MAC_CONF_SPD_1000MHZ;
		break;
	case 2500:
		reg_val |= MAC_CONF_SPD_2500MHZ;
		break;
	default:
		reg_val |= MAC_CONF_SPD_10MHZ;
		break;
	}

	switch (context->duplex_mode) {
	case FULL_DUPLX:
		reg_val |= MAC_CONF_DM;
		break;
	default:
		reg_val &= ~MAC_CONF_DM;
		break;
	}

	eth_write(base_addr, MAC_CONFIGURATION, reg_val);
}

static inline int eth_mac_addr_idx(struct eth_runtime *ctxt, uint8_t addr[6])
{
	uint32_t base_addr = ctxt->base_addr;
	int idx = -1;

	for (int i = 1; i <= ctxt->addrcnt; i++) {
		uint32_t words[2];

		words[1] = eth_read(base_addr, MAC_ADDRESS_HIGH(i));
		words[0] = eth_read(base_addr, MAC_ADDRESS_LOW(i));

		if (!memcmp(addr, words, 6)) {
			idx = i;
			break;
		}
	}

	return idx;
}

/* bitwise reverse a byte from BE bit to LE bit or vice versa */
static inline uint8_t byte_bitwise_reverse(uint8_t data)
{
	union {
		struct {
			uint8_t lo : 4;
			uint8_t hi : 4;
		};
		uint8_t byte_nibl;
	} data_nibl;
	const uint8_t nibble_bitrev_tbl[16] = { 0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6,
					     0xE, 0x1, 0x9, 0x5, 0xD, 0x3, 0xB,
					     0x7, 0xF };

	data_nibl.byte_nibl = data;
	return (nibble_bitrev_tbl[data_nibl.lo] << 4 |
		nibble_bitrev_tbl[data_nibl.hi]);
}

static void eth_mac_set_filter(struct eth_runtime *ctxt, uint8_t addr[6], int idx,
			       bool hash, bool remove)
{
	uint32_t base_addr = ctxt->base_addr;

	LOG_DBG("%s MAC filter address 0x%02X%02X%02X%02X%02X%02X",
		remove ? "Unset" : "Set", addr[0], addr[1], addr[2],
		addr[3], addr[4], addr[5]);

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	if (idx && !hash) {
		LOG_ERR("Only one MAC address perfect filtering is supported "
			"in RX Parser functionality");
		return;
	} else if (hash) {
		idx = -1;
	}

	eth_rxp_mac_addr(ctxt, idx, remove, addr);
	return;
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */

	if (idx > ctxt->addrcnt) {
		hash = true;
	} else if (!idx && remove && !hash) {
		idx = eth_mac_addr_idx(ctxt, addr);
		if (idx < 0) {
			hash = true;
		}
	}

	if (!hash) {   /* Perfect filtering */
		uint32_t words[2];

		if (!remove) {
			memcpy(words, addr, 6);
			eth_write(base_addr, MAC_ADDRESS_HIGH(idx),
				  words[1] | MAC_ADDR_HI_AE);
			eth_write(base_addr, MAC_ADDRESS_LOW(idx), words[0]);
			LOG_DBG("Set MAC perfect filter index %d", idx);
		} else {
			uint32_t word;

			word = eth_read(base_addr, MAC_ADDRESS_HIGH(idx));
			eth_write(base_addr, MAC_ADDRESS_HIGH(idx),
				  word & ~MAC_ADDR_HI_AE);
			LOG_DBG("Remove MAC perfect filter index %d", idx);
		}
	} else {       /* Hash filtering */
		uint32_t hash_val, hash_idx, regval;

		/* Calculate Hash value */
		hash_val = byte_bitwise_reverse(crc32_ieee(addr, 6));

		if (ctxt->hashtblsz == MAC_HW_FEAT1_HASHTBLSZ_64) {
			hash_val >>= MAC_HASH_64_VAL_SHIFT;
			hash_idx = hash_val & MAC_HASH_64_REG_SEL_BIT;
		} else if (ctxt->hashtblsz == MAC_HW_FEAT1_HASHTBLSZ_128) {
			hash_val >>= MAC_HASH_128_VAL_SHIFT;
			hash_idx = hash_val & MAC_HASH_128_REG_SEL_BIT;
		} else if (ctxt->hashtblsz == MAC_HW_FEAT1_HASHTBLSZ_256) {
			hash_idx = hash_val & MAC_HASH_256_REG_SEL_BIT;
		} else {
			LOG_ERR("HW support Hash Filter is not available");
			return;
		}

		/* Configure Hash Table */
		regval = eth_read(base_addr, MAC_HASH_TABLE_REG(hash_idx));
		if (!remove) {
			regval |= 1 << (hash_val & MAC_HASH_VAL_MASK);
		} else {
			regval &= ~(1 << (hash_val & MAC_HASH_VAL_MASK));
		}

		eth_write(base_addr, MAC_HASH_TABLE_REG(hash_idx), regval);
		LOG_DBG("%s MAC hash value 0x%08X", remove ? "Remove" : "Set",
			hash_val);
	}
}

static inline int eth_mac_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	int i;

	/* Enable MAC auto pad/crc stripping for IEEE802.3 length packet,
	 * crc stripping for EtherType (Ethernet II) type packet,
	 * and RX checksum offload
	 *
	 * Note: Setting bit IPC would not take effect if HW IP has not
	 *       enabled the checksum offload feature
	 */
	reg_val = MAC_CONF_CST | MAC_CONF_ACS | MAC_CONF_IPC;
	eth_write(base_addr, MAC_CONFIGURATION, reg_val);

	eth_mac_config_mode(context);

	/* Enable MAC RX queues to DCB/General mode */
	reg_val = 0;
	for (i = 0; i < context->rxqnum; i++) {
		reg_val |= MAC_EN_DCB_GEN_RXQ(i);
	}
	eth_write(base_addr, MAC_RXQ_CTRL0, reg_val);

	/* Set the MAC address to device. */
	eth_mac_set_filter(context, context->mac_addr.bytes, 0, false, false);

#ifdef CONFIG_ETH_DWC_EQOS_PTP
	/* Set the PTP multicast MAC address to device. */
	uint8_t ptp_mcast_addr[6] = { 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E };

	eth_mac_set_filter(context, ptp_mcast_addr, 0, true, false);
#endif /* CONFIG_ETH_DWC_EQOS_PTP */

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	if (context->frp_buf_sz != 0 && context->frp_entry_sz != 0) {
		reg_val = MAC_PKT_FLTR_PR;
	} else
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */
	{
		/* Enable MAC HASH & MAC Perfect filtering */
		reg_val = MAC_PKT_FLTR_HPF | MAC_PKT_FLTR_HMC |
				MAC_PKT_FLTR_HUC;
#ifdef CONFIG_NET_VLAN
		/* Enable VLAN Perfect filtering */
		if (context->vlancnt > 0) {
			reg_val |= MAC_PKT_FLTR_VTFE;
		}
#endif /* CONFIG_NET_VLAN */
	}
	eth_write(base_addr, MAC_PACKET_FILTER, reg_val);

#ifdef CONFIG_NET_VLAN
	/* Enable VLAN RX status */
	if (context->vlancnt > 0) {
		reg_val = MAC_VLAN_TAG_CTRL_EVLRXS;
#ifdef CONFIG_ETH_DWC_EQOS_VLAN_STRIP
		/* Enable VLAN tag strip off if filter pass */
		reg_val |= MAC_VLAN_TAG_CTRL_EVLS_IFPASS;
#endif /* CONFIG_ETH_DWC_EQOS_VLAN_STRIP */
		eth_write(base_addr, MAC_VLAN_TAG_CTRL, reg_val);
	}
#endif /* CONFIG_NET_VLAN */

	/* Set Vlan priority for Rx Queue Steering */
	if (context->rxqnum <= 4) {
		eth_write(base_addr, MAC_RXQ_CTRL2, MAC_RXQ_PRIO_CTRL2);
	} else if (context->rxqnum >= 5) {
		eth_write(base_addr, MAC_RXQ_CTRL2, MAC_RXQ_PRIO_CTRL2);
		eth_write(base_addr, MAC_RXQ_CTRL3, MAC_RXQ_PRIO_CTRL3);
	}

	return 0;
}

static inline int eth_mac_eee_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	/* set LPI link status timer to be 1 sec */
	reg_val = (MAC_LPI_TIMERS_CONTROL_LST_DEFAULT &
			MAC_LPI_TIMERS_CONTROL_LST_MASK)
			<< MAC_LPI_TIMERS_CONTROL_LST_SHIFT;
	reg_val |= (MAC_LPI_TIMERS_CONTROL_TWT_DEFAULT &
			MAC_LPI_TIMERS_CONTROL_TWT_MASK);
	eth_write(base_addr, MAC_LPI_TIMERS_CONTROL, reg_val);

	/* 1US Tic Counter */
	reg_val = (MAC_1US_TIC_CNTR_VALUE(ETH_DWC_EQOS_SYS_CLOCK) &
			MAC_1US_TIC_CNTR_MASK);
	eth_write(base_addr, MAC_1US_TIC_CNTR, reg_val);

	/* set LPI Entry Timer */
	reg_val = context->lpi_timer & MAC_LPI_ENTRY_TIMER_START_MASK;
	eth_write(base_addr, MAC_LPI_ENTRY_TIMER, reg_val);

	/* Set LPI Timer Enable & LPI Tx Automate & LPI Enable */
	reg_val = (MAC_LPI_CONTROL_STATE_LPITXA |
			 MAC_LPI_CONTROL_STATE_LPIATE |
				MAC_LPI_CONTROL_STATE_LPIEN);
	eth_write(base_addr, MAC_LPI_CONTROL_STATE, reg_val);

	return 0;
}

static inline int eth_mtl_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	int32_t txqsz, rxqsz;
	int i;

	/* Set MTL TX scheduling algo & RX arbitration algo to
	 * strict priority
	 */
	reg_val = MTL_OPR_MD_SCHALG_SP;
#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	if (!eth_set_static_rx_parser(context)) {
		/* Enable RX parser functionality */
		reg_val |= MTL_OPR_MD_FRPE;
	}
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */
	eth_write(base_addr, MTL_OPERATION_MODE, reg_val);

	/* Configure RX queues and DMA channels mapping */
	reg_val = MTL_RXQ_DMA_Q1_Q5MDMACH_1 | MTL_RXQ_DMA_Q2_Q6MDMACH_2 |
		  MTL_RXQ_DMA_Q3_Q7MDMACH_3;
	eth_write(base_addr, MTL_RXQ_DMA_MAP0, reg_val);
	reg_val = MTL_RXQ_DMA_Q0_Q4MDMACH_4 | MTL_RXQ_DMA_Q1_Q5MDMACH_5 |
		  MTL_RXQ_DMA_Q2_Q6MDMACH_6 | MTL_RXQ_DMA_Q3_Q7MDMACH_7;
	eth_write(base_addr, MTL_RXQ_DMA_MAP1, reg_val);

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	/* Configure TBS Launch Time Expiry Offset & Valid bit */
	if ((CONFIG_ETH_DWC_EQOS_TBS_EXPIRY_OFFSET >= 0) &&
	    (CONFIG_ETH_DWC_EQOS_TBS_EXPIRY_OFFSET < ONE_SEC_IN_NANOSEC)) {
		reg_val = CONFIG_ETH_DWC_EQOS_TBS_EXPIRY_OFFSET &
			  MTL_TBS_CTRL_LEOS_MASK;
		reg_val |= MTL_TBS_CTRL_LEOV;
	} else {
		reg_val = 0;
	}

	eth_write(base_addr, MTL_TBS_CTRL, reg_val);
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

	txqsz = (context->txfifosz / (context->txqnum * MTL_TXQSZ_BLOCK)) - 1;
	txqsz = (txqsz < 0) ? 0 : txqsz;
	rxqsz = (context->rxfifosz / (context->rxqnum * MTL_RXQSZ_BLOCK)) - 1;
	rxqsz = (rxqsz < 0) ? 0 : rxqsz;

	for (i = 0; i < context->txqnum; i++) {
		/* Enable TX store forward and configure TX queue size */
		reg_val = MTL_TXQ_OPR_TSF;
		reg_val |= ((txqsz << MTL_TXQ_OPR_TQS_SHIFT) &
			    MTL_TXQ_OPR_TQS_MASK);
		eth_write(base_addr, MTL_TXQ_OPERATION_MODE(i), reg_val);
	}

	for (i = 0; i < context->rxqnum; i++) {
		/* Enable RX store forward and configure RX queue size */
		reg_val = MTL_RXQ_OPR_RSF;
		reg_val |= ((rxqsz << MTL_RXQ_OPR_RQS_SHIFT) &
			    MTL_RXQ_OPR_RQS_MASK);
		eth_write(base_addr, MTL_RXQ_OPERATION_MODE(i), reg_val);
	}

	reg_val = eth_read(base_addr, MTL_ECC_CTRL);
	/* Enable ECC for MTL Rx/Tx FIFO */
	reg_val |= MTL_ECC_CTRL_MRXEE | MTL_ECC_CTRL_MTXEE;
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	/* When gcl_depth is not equal to 0, it means EST feature is supported.
	 * Thus, enable ECC for MTL EST.
	 */
	if (context->estparam.gcl_depth) {
		reg_val |= MTL_ECC_CTRL_MESTEE;
	}
#endif
	eth_write(base_addr, MTL_ECC_CTRL, reg_val);

	return 0;
}

static inline void eth_dma_tx_intr_ctrl(struct eth_runtime *ctxt, int queue,
					bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_INTR_EN_CH(queue));
	if (on) {
		reg_val |= DMA_CH_INTR_EN_TIE;
	} else {
		reg_val &= ~DMA_CH_INTR_EN_TIE;
	}
	eth_write(base_addr, DMA_INTR_EN_CH(queue), reg_val);
}

static inline void eth_dma_rx_intr_ctrl(struct eth_runtime *ctxt, int queue,
					bool on)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_INTR_EN_CH(queue));
	if (on) {
		reg_val |= DMA_CH_INTR_EN_RIE;
	} else {
		reg_val &= ~DMA_CH_INTR_EN_RIE;
	}
	eth_write(base_addr, DMA_INTR_EN_CH(queue), reg_val);
}

#ifdef CONFIG_ETH_DWC_EQOS_INTR_COALESCE
static inline uint32_t eth_rx_intr_coalesce_value(int queue)
{
	uint32_t val;

	switch (queue) {
#ifdef CONFIG_ETH_DWC_EQOS_RXQ0_COALESCE_TIMER
	case 0:
		val = CONFIG_ETH_DWC_EQOS_RXQ0_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ1_COALESCE_TIMER
	case 1:
		val = CONFIG_ETH_DWC_EQOS_RXQ1_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ2_COALESCE_TIMER
	case 2:
		val = CONFIG_ETH_DWC_EQOS_RXQ2_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ3_COALESCE_TIMER
	case 3:
		val = CONFIG_ETH_DWC_EQOS_RXQ3_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ4_COALESCE_TIMER
	case 4:
		val = CONFIG_ETH_DWC_EQOS_RXQ4_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ5_COALESCE_TIMER
	case 5:
		val = CONFIG_ETH_DWC_EQOS_RXQ5_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ6_COALESCE_TIMER
	case 6:
		val = CONFIG_ETH_DWC_EQOS_RXQ6_COALESCE_TIMER;
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_RXQ7_COALESCE_TIMER
	case 7:
		val = CONFIG_ETH_DWC_EQOS_RXQ7_COALESCE_TIMER;
		break;
#endif
	default:
		val = 0;
		break;
	}

	return val;
}

static inline void eth_dma_rx_intr_wtd(struct eth_runtime *ctxt, int queue)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;
	int rwtu_range;
	int i;

	reg_val = eth_rx_intr_coalesce_value(queue);
	if (!reg_val) {
		return;
	}

	reg_val = (u64_t)reg_val * ETH_DWC_EQOS_SYS_CLOCK / ONE_SEC_IN_MICROSEC;
	if (reg_val < DMA_CH_RX_INTR_WDT_RWTU(0)) {
		reg_val = DMA_CH_RX_INTR_WDT_RWTU(0);
	}

	rwtu_range = (DMA_CH_RX_INTR_WDT_RWTU_MASK >>
			DMA_CH_RX_INTR_WDT_RWTU_SHIFT) + 1;
	for (i = 0; i < rwtu_range; i++) {
		if (reg_val > (DMA_CH_RX_INTR_WDT_RWTU(i) *
		    DMA_CH_RX_INTR_WDT_RWT_MASK)) {
			continue;
		} else {
			if (reg_val % DMA_CH_RX_INTR_WDT_RWTU(i)) {
				reg_val += DMA_CH_RX_INTR_WDT_RWTU(i);
			}
			reg_val /= DMA_CH_RX_INTR_WDT_RWTU(i);
			break;
		}
	}
	/* Limit to maximum allowed value */
	if (i == rwtu_range) {
		reg_val = DMA_CH_RX_INTR_WDT_RWT_MASK;
		i--;
	}

	LOG_INF("Setting RXQ%d coalesce timer: %uus", queue,
		(unsigned int)((u64_t)reg_val * DMA_CH_RX_INTR_WDT_RWTU(i) *
		ONE_SEC_IN_MICROSEC / ETH_DWC_EQOS_SYS_CLOCK));
	reg_val &= DMA_CH_RX_INTR_WDT_RWT_MASK;
	reg_val |= (i << DMA_CH_RX_INTR_WDT_RWTU_SHIFT) &
			DMA_CH_RX_INTR_WDT_RWTU_MASK;
	eth_write(base_addr, DMA_RX_INTR_WDT_CH(queue), reg_val);
}
#endif /* CONFIG_ETH_DWC_EQOS_INTR_COALESCE */

static inline void eth_dma_tx_desc_init(struct eth_runtime *context, int queue)
{
	volatile struct eth_tx_desc *desc = context->tx_desc[queue];

	memset((void *)desc, 0,
	       sizeof(*desc) * CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE);

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *edesc = context->enh_tx_desc[queue];

	memset((void *)edesc, 0,
	       sizeof(*edesc) * CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE);
#endif
}

static inline void eth_dma_rx_desc_init(struct eth_runtime *context, int queue)
{
	volatile struct eth_rx_desc *desc;
	int i;

	for (i = 0; i < (CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE - 1); i++) {
		desc = eth_get_rx_desc(context, queue);

		if (!desc) {
			LOG_ERR("Failed to retrieve RX descriptor.");
			continue;
		}

		desc->rdes0.buf1ap = POINTER_TO_DMA(&context->rx_buf[queue][i][0]);
		desc->rdes1.dword = 0;
		desc->rdes2.dword = 0;
		desc->rdes3.rd.buf1v = 1;
		desc->rdes3.rd.ioc = 1;
#ifdef CONFIG_ETH_DWC_EQOS_INTR_COALESCE
		if (eth_rx_intr_coalesce_value(queue)) {
			desc->rdes3.rd.ioc = 0;
		}
#endif
		desc->rdes3.rd.own = 1;

		dcache_clean((uintptr_t)desc, sizeof(*desc));
	}
}

static inline int eth_dma_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	int i;

	/* Enable Low Power Interface and DMA AXI Burst Length 4, 8 & 16 mode */
	reg_val = DMA_SYSBUS_MD_EN_LPI | DMA_SYSBUS_MD_BLEN16 |
		  DMA_SYSBUS_MD_BLEN8 | DMA_SYSBUS_MD_BLEN4;
	eth_write(base_addr, DMA_SYSBUS_MODE, reg_val);

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	/* Configure TBS Fetch Time Offset & Valid bit */
	if ((CONFIG_ETH_DWC_EQOS_TBS_FETCH_OFFSET >= 0) &&
	    (CONFIG_ETH_DWC_EQOS_TBS_FETCH_OFFSET < ONE_SEC_IN_NANOSEC)) {
		reg_val = CONFIG_ETH_DWC_EQOS_TBS_FETCH_OFFSET &
			  DMA_TBS_CTRL_FTOS_MASK;
		reg_val |= DMA_TBS_CTRL_FTOV;
	} else {
		reg_val = 0;
	}

	eth_write(base_addr, DMA_TBS_CTRL, reg_val);
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

	/* TODO: descriptor address alignment */
	if (POINTER_TO_DMA(&context->tx_desc[0][0]) & 0x0F) {
		LOG_ERR("TX descriptor address alignment error 0x%08lX",
			(long)POINTER_TO_DMA(&context->tx_desc[0][0]));
	}

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	if (POINTER_TO_DMA(&context->enh_tx_desc[0][0]) & 0x0F) {
		LOG_ERR("Enhanced TX descriptor address alignment error 0x%08lX",
			(long)POINTER_TO_DMA(&context->enh_tx_desc[0][0]));
	}
#endif

	if (POINTER_TO_DMA(&context->rx_desc[0][0]) & 0x0F) {
		LOG_ERR("RX descriptor address alignment error 0x%08lX",
			(long)POINTER_TO_DMA(&context->rx_desc[0][0]));
	}

	for (i = 0; i < context->txqnum; i++) {
		/* Initialize TX descriptor ring length */
		eth_write(base_addr, DMA_TXDESC_RING_LENGTH_CH(i),
			  CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE - 1);

		/* Set TX PBL to 32x8 */
		reg_val = 32 << DMA_CH_TX_CTRL_TXPBL_SHIFT;
		reg_val &= DMA_CH_TX_CTRL_TXPBL_MASK;

#ifdef CONFIG_ETH_DWC_EQOS_TBS
		/* Clean up TBS enabled flag if HW not supported */
		if (!(context->flags & ETH_DEV_FLAGS_TBS)) {
			context->tbs_enabled[i] = false;
		}
		/* Set Enhanced Descriptor Mode for TBS */
		if (context->tbs_enabled[i]) {
			reg_val |= DMA_CH_TX_CTRL_EDSE;
		}
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
		eth_write(base_addr, DMA_TX_CONTROL_CH(i), reg_val);

		k_sem_init(&context->txq_lock[i], 1, 1);
		eth_dma_tx_desc_init(context, i);

		/* Initialize TX descriptor ring list address */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
		if (context->tbs_enabled[i]) {
			eth_write(base_addr, DMA_TXDESC_LIST_ADDR_CH(i),
				  POINTER_TO_DMA(&context->enh_tx_desc[i][0]));
		} else
#endif
		{
			eth_write(base_addr, DMA_TXDESC_LIST_ADDR_CH(i),
				  POINTER_TO_DMA(&context->tx_desc[i][0]));
		}

		/* Initialize TX descriptor ring tail pointer */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
		if (context->tbs_enabled[i]) {
			eth_write(base_addr, DMA_TXDESC_TAIL_PTR_CH(i),
				  POINTER_TO_DMA(&context->enh_tx_desc[i][0]));
		} else
#endif
		{
			eth_write(base_addr, DMA_TXDESC_TAIL_PTR_CH(i),
				  POINTER_TO_DMA(&context->tx_desc[i][0]));
		}
	}

	for (i = 0; i < context->rxqnum; i++) {
		/* Initialize RX descriptor ring length */
		eth_write(base_addr, DMA_RXDESC_RING_LENGTH_CH(i),
			  CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE - 1);

		/* Initialize RX descriptor ring list address */
		eth_write(base_addr, DMA_RXDESC_LIST_ADDR_CH(i),
			  POINTER_TO_DMA(&context->rx_desc[i][0]));

		/* Set RX PBL to 32x8 */
		reg_val = 32 << DMA_CH_RX_CTRL_RXPBL_SHIFT;
		reg_val |= ETH_MAX_FRAME_SZ << DMA_CH_RX_CTRL_RBSZ_SHIFT;
		reg_val &= DMA_CH_RX_CTRL_RXPBL_MASK | DMA_CH_RX_CTRL_RBSZ_MASK;
		eth_write(base_addr, DMA_RX_CONTROL_CH(i), reg_val);

		k_sem_init(&context->rxq_lock[i], 1, 1);
		eth_dma_rx_desc_init(context, i);

		/* Initialize RX descriptor ring tail pointer */
		eth_write(base_addr, DMA_RXDESC_TAIL_PTR_CH(i),
			  POINTER_TO_DMA(RX_DESC_TAIL_PTR(context, i)));
	}

	for (i = 0; i < MAX(context->txqnum, context->rxqnum); i++) {
		/* Enable 8x Programmable Burst Length mode */
		reg_val = DMA_CH_CTRL_PBLX8;
		eth_write(base_addr, DMA_CONTROL_CH(i), reg_val);
	}

	return 0;
}

/* TODO: Refactor to use PHY framework model for modphy & xpcs */
/* TODO: Refactor to use mdiobus_lock instead of phy_lock */
static int eth_modphy_gpsr0_chk(const struct device *port, uint16_t valmask,
				uint16_t chkval)
{
	struct eth_runtime *context = port->data;
	struct phy_device *phy = &context->phy_dev;
	union mdio_frm_flds frm_fld;
	uint16_t val;
	int retry = 10;
	int retval = 0;

	do {
		frm_fld.c22_frm_fld.phyaddr = MODPHY_ADDR;
		frm_fld.c22_frm_fld.regaddr = MODPHY_GPSR0;

		k_sem_take(&phy->phy_lock, K_FOREVER);
		retval = dwc_mdio_read(phy, frm_fld, &val, C22);
		k_sem_give(&phy->phy_lock);
		if (retval < 0) {
			return retval;
		}

		if ((val & valmask) == chkval) {
			break;
		}

		k_sleep(K_MSEC(1));  /* 1ms polling interval */
	} while (retry--);

	return (retry < 0) ? -EBUSY : retval;
}

static int eth_modphy_gpcr0_modify(const struct device *port, uint16_t valmask,
				   uint16_t valwrite)
{
	struct eth_runtime *context = port->data;
	struct phy_device *phy = &context->phy_dev;
	union mdio_frm_flds frm_fld;
	uint16_t val;
	int retval = 0;

	frm_fld.c22_frm_fld.phyaddr = MODPHY_ADDR;
	frm_fld.c22_frm_fld.regaddr = MODPHY_GPCR0;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = dwc_mdio_read(phy, frm_fld, &val, C22);
	k_sem_give(&phy->phy_lock);
	if (retval < 0) {
		return retval;
	}

	val &= ~(valmask);
	val |= valwrite;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = dwc_mdio_send(phy, frm_fld, val, C22);
	k_sem_give(&phy->phy_lock);
	if (retval < 0) {
		return retval;
	}

	return retval;
}

static inline int eth_modphy_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	int retval = 0;

	/* Set to turn on SGMII Reference clock */
	retval = eth_modphy_gpcr0_modify(port,
					 MODPHY_GPCR0_PLLREFCLK_REQ,
					 MODPHY_GPCR0_PLLREFCLK_REQ);
	if (retval) {
		return retval;
	}
	/* Verify if clock has turned on */
	retval = eth_modphy_gpsr0_chk(port,
				      MODPHY_GPSR0_PLLREFCLK_VALID,
				      MODPHY_GPSR0_PLLREFCLK_VALID <<
				      MODPHY_GPSR0_PLLREFCLK_SHIFT);
	if (retval) {
		return retval;
	}

	/* Set the modphy lane to out of reset, pclk and power stage to P0 */
	retval = eth_modphy_gpcr0_modify(port, MODPHY_GPCR0_PCLK_RATE_MASK |
					 MODPHY_GPCR0_PWRDWN_SUS_MASK |
					 MODPHY_GPCR0_RSTDATAPATH_INTF,
					 (MODPHY_GPCR0_PCLK_RATE_1G <<
					 MODPHY_GPCR0_PCLK_RATE_SHIFT) |
					 (MODPHY_GPCR0_PWRDWN_SUS_P0 <<
					 MODPHY_GPCR0_PWRDWN_SUS_SHIFT) |
					 MODPHY_GPCR0_RSTDATAPATH_INTF);
	if (retval) {
		return retval;
	}
	/* Verify if modphy has changed to P0 power stage */
	retval = eth_modphy_gpsr0_chk(port, MODPHY_GPSR0_PWRDWN_SUS_SIG_MASK,
				      MODPHY_GPSR0_PWRDWN_SUS_SIG_P0 <<
				      MODPHY_GPSR0_PWRDWN_SUS_SIG_SHIFT);
	if (retval) {
		return retval;
	}

	/* Ungate SGMII PHY RX clock only for PSE gbe */
	if (context->pse_gbe) {
		retval = eth_modphy_gpcr0_modify(port,
						 MODPHY_GPCR0_PHYRXCLK_REQ,
						 MODPHY_GPCR0_PHYRXCLK_REQ);
		if (retval) {
			return retval;
		}
		/* Verify if clock has turned on */
		retval = eth_modphy_gpsr0_chk(port,
					      MODPHY_GPSR0_PHYRXCLK_VALID,
					      MODPHY_GPSR0_PHYRXCLK_VALID <<
					      MODPHY_GPSR0_PHYRXCLK_SHIFT);
		if (retval) {
			return retval;
		}
	}

	return retval;
}

static int eth_xpcs_read(struct eth_runtime *ctxt, uint8_t devnum, uint16_t regnum,
			 uint16_t *val)
{
	struct phy_device *phy = &ctxt->phy_dev;
	union mdio_frm_flds frm_fld;
	int retval = 0;

	frm_fld.c45_frm_fld.portaddr = XPCS_ADDR;
	frm_fld.c45_frm_fld.devaddr = devnum;
	frm_fld.c45_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = dwc_mdio_read(phy, frm_fld, val, C45);
	k_sem_give(&phy->phy_lock);
	return retval;
}

static int eth_xpcs_modify(struct eth_runtime *ctxt, uint8_t devnum,
			   uint16_t regnum, uint16_t mask, uint16_t val)
{
	struct phy_device *phy = &ctxt->phy_dev;
	union mdio_frm_flds frm_fld;
	uint16_t data;
	int retval = 0;

	frm_fld.c45_frm_fld.portaddr = XPCS_ADDR;
	frm_fld.c45_frm_fld.devaddr = devnum;
	frm_fld.c45_frm_fld.regaddr = regnum;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = dwc_mdio_read(phy, frm_fld, &data, C45);
	k_sem_give(&phy->phy_lock);
	if (retval < 0) {
		return retval;
	}

	data &= ~(mask);
	data |= val;

	k_sem_take(&phy->phy_lock, K_FOREVER);
	retval = dwc_mdio_send(phy, frm_fld, data, C45);
	k_sem_give(&phy->phy_lock);
	if (retval < 0) {
		return retval;
	}

	return retval;
}

static inline int eth_xpcs_get_tx_latency(struct eth_runtime *ctxt,
					  int32_t link_speed, int32_t *latency)
{
	ARG_UNUSED(link_speed);
	uint16_t phy_reg, temp, min_delay, max_delay;
	int retval = 0;

	*latency = 0;
	/* Confirm if TX delay value can be read from registers */
	retval = eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
			       SR_MII_TIME_SYNC_ABL, &phy_reg);
	if (!(phy_reg & SR_MII_TIME_SYNC_MII_TX_DLY_ABL)) {
		return -EINVAL;
	}

	/* Read the maximum TX delay value */
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_TX_MAX_DLY_LWR, &temp);
	max_delay = temp;
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_TX_MAX_DLY_UPR, &temp);
	max_delay = (temp << SR_MII_TIME_SYNC_DLY_UPR_SHIFT) | max_delay;

	/* Read the minimum TX delay value */
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_TX_MIN_DLY_LWR, &temp);
	min_delay = temp;
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_TX_MIN_DLY_UPR, &temp);
	min_delay = (temp << SR_MII_TIME_SYNC_DLY_UPR_SHIFT) | min_delay;

	/* Get the mean value for TX latency */
	if (!retval) {
		*latency = (max_delay + min_delay) >> 1;
	}

	return retval;
}

static inline int eth_xpcs_get_rx_latency(struct eth_runtime *ctxt,
					  int32_t link_speed, int32_t *latency)
{
	ARG_UNUSED(link_speed);
	uint16_t phy_reg, temp, min_delay, max_delay;
	int retval = 0;

	*latency = 0;
	/* Confirm if RX delay value can be read from registers */
	retval = eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
			       SR_MII_TIME_SYNC_ABL, &phy_reg);
	if (!(phy_reg & SR_MII_TIME_SYNC_MII_RX_DLY_ABL)) {
		return -EINVAL;
	}

	/* Read the maximum RX delay value */
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_RX_MAX_DLY_LWR, &temp);
	max_delay = temp;
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_RX_MAX_DLY_UPR, &temp);
	max_delay = (temp << SR_MII_TIME_SYNC_DLY_UPR_SHIFT) | max_delay;

	/* Read the minimum RX delay value */
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_RX_MIN_DLY_LWR, &temp);
	min_delay = temp;
	retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
				SR_MII_TIME_SYNC_RX_MIN_DLY_UPR, &temp);
	min_delay = (temp << SR_MII_TIME_SYNC_DLY_UPR_SHIFT) | min_delay;

	/* Get the mean value for RX latency */
	if (!retval) {
		*latency = (max_delay + min_delay) >> 1;
	}

	return retval;
}

static inline int eth_xpcs_init(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	int xpcs_retry, retval = 0;
	uint16_t phy_reg;
	uint16_t mask;

	/* Reset XPCS before initialize the registers */
	mask = SR_MII_CTRL_RST;
	phy_reg = SR_MII_CTRL_RST;
	retval |= eth_xpcs_modify(ctxt, VENDOR_SPECIFIC_MII_MMD,
				  SR_MII_MMD_CTRL_REG, mask, phy_reg);
	/* Check if the XPCS reset has completed */
	phy_reg = 0;
	xpcs_retry = 10;
	do {
		retval |= eth_xpcs_read(ctxt, VENDOR_SPECIFIC_MII_MMD,
					SR_MII_MMD_CTRL_REG, &phy_reg);

		if (!(phy_reg & SR_MII_CTRL_RST)) {
			break;
		}

		k_sleep(K_MSEC(1));
	} while (--xpcs_retry);
	/* Return error timedout if retry count reach zero */
	if (!xpcs_retry) {
		return -ETIMEDOUT;
	}

	/* Enable Pre-emption packet & auto speed mode change after AN */
	mask = VR_MII_DIG_CTRL1_MAC_AUTO_SW | VR_MII_DIG_CTRL1_PRE_EMP;
	phy_reg = VR_MII_DIG_CTRL1_MAC_AUTO_SW | VR_MII_DIG_CTRL1_PRE_EMP;
	retval |= eth_xpcs_modify(ctxt, VENDOR_SPECIFIC_MII_MMD,
				  VR_MII_MMD_DIG_CTRL1_REG, mask, phy_reg);

	/* Enable AN interrupt, SGMII PCS Mode & MAC side SGMII */
	mask = VR_MII_AN_CTRL_TX_CFG | VR_MII_AN_CTRL_PCS_MODE_MASK |
		VR_MII_AN_CTRL_AN_INTR_EN;
	phy_reg = VR_MII_AN_CTRL_TX_CFG_MAC_SIDE_SGMII |
		  VR_MII_AN_CTRL_PCS_MODE_SGMII | VR_MII_AN_CTRL_AN_INTR_EN;
	retval |= eth_xpcs_modify(ctxt, VENDOR_SPECIFIC_MII_MMD,
				  VR_MII_MMD_AN_CTRL_REG, mask, phy_reg);

	/* Enable AN & restart AN */
	mask = SR_MII_CTRL_AN_EN | SR_MII_CTRL_RESTART_AN;
	phy_reg = SR_MII_CTRL_AN_EN | SR_MII_CTRL_RESTART_AN;
	retval |= eth_xpcs_modify(ctxt, VENDOR_SPECIFIC_MII_MMD,
				  SR_MII_MMD_CTRL_REG, mask, phy_reg);

	/* Enable EEE */
	mask = VR_MII_EEE_LTX_EN | VR_MII_EEE_LRX_EN | VR_MII_EEE_TX_QUIET_EN |
		VR_MII_EEE_RX_QUIET_EN | VR_MII_EEE_TX_EN_CTRL |
		VR_MII_EEE_RX_EN_CTRL;
	phy_reg = mask;
	retval |= eth_xpcs_modify(ctxt, VENDOR_SPECIFIC_MII_MMD,
				  VR_MII_EEE_MCTRL0, mask, phy_reg);

	return retval;
}

#if defined(CONFIG_ETH_DWC_EQOS_QAV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
static void eth_tsn_reconfig(struct eth_runtime *ctxt)
{
	uint32_t base_addr = ctxt->base_addr;
#ifdef CONFIG_ETH_DWC_EQOS_QAV
	int i;

	/* Check whether link speed support Qav. */
	if (ctxt->link_speed == 10) {
		LOG_INF("10Mbps link speed does not support "
			    "IEEE 802.1Qav.");
		for (i = 1; i < ctxt->txqnum; i++) {
			if (ctxt->cbsparam[i - 1].enable) {
				dwmac_set_cbs_status(base_addr, i, 0);
				ctxt->cbsparam[i - 1].enable = 0;
				LOG_INF("Qav is disable for TX Queue %d.", i);
			}
		}
	} else {
	/* Change idle slope according to bandwidth and vice versa. */
		for (i = 1; i < ctxt->txqnum; i++) {
			if (ctxt->cbsparam[i - 1].enable) {
				if (ctxt->cbsparam[i - 1].bandwidth) {
					ctxt->cbsparam[i - 1].idle_slope =
						PERCENT_TO_DECI(
						ctxt->cbsparam[i - 1].bandwidth,
						ctxt->link_speed);
				} else {
					if (ctxt->cbsparam[i - 1].idle_slope >
					    ctxt->link_speed) {
						ctxt->cbsparam[i - 1].idle_slope
							= ctxt->link_speed;
					}
					ctxt->cbsparam[i - 1].bandwidth =
					    DECI_TO_PERCENT(
					    ctxt->cbsparam[i - 1].idle_slope,
					    ctxt->link_speed);
				}
#ifdef CONFIG_ETH_DWC_EQOS_QBV
				/* Recalculate idle slope based on oper GCL */
				if (ctxt->estparam.enable) {
					dwmac_cbs_recal_idleslope(base_addr,
					    &ctxt->cbsparam[i - 1].idle_slope,
					    ctxt->link_speed, i, 0);
				}
#endif
				dwmac_set_cbs_idlesend(base_addr,
					ctxt->cbsparam[i - 1].idle_slope,
					ctxt->link_speed, i);
				dwmac_set_cbs_status(base_addr, i, 1);
			}
		}
	}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	if (ctxt->fpeparam.enable) {
		dwmac_fpe_send_mpacket(base_addr, MPACKET_VERIFY);
	}
#endif
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV || CONFIG_ETH_DWC_EQOS_QBU */

#if defined(CONFIG_NET_GPTP) && defined(CONFIG_ETH_DWC_EQOS_PTP)
static void eth_mac_timestamp_correction(struct eth_runtime *ctxt)
{
	struct phy_device *phy = &ctxt->phy_dev;
	int32_t temp_delay, tx_delay, rx_delay;
	uint32_t base_addr = ctxt->base_addr;
	uint32_t neg_rx_delay;

	if (phy->get_tx_latency && phy->get_rx_latency) {
		/* Obtain PHY TX & RX latencies */
		if (phy->get_tx_latency(phy, ctxt->link_speed, &temp_delay)) {
			LOG_ERR("Failed to obtain PHY TX latency");
			return;
		}
		tx_delay = temp_delay;
		if (phy->get_rx_latency(phy, ctxt->link_speed, &temp_delay)) {
			LOG_ERR("Failed to obtain PHY RX latency");
			return;
		}
		rx_delay = temp_delay;

		/* Obtain XPCS TX & RX latency if SGMII mode is configured */
		if (ctxt->use_xpcs) {
			if (eth_xpcs_get_tx_latency(ctxt, ctxt->link_speed,
						    &temp_delay)) {
				LOG_ERR("Failed to obtain XPCS TX latency");
				return;
			}
			tx_delay += temp_delay;
			if (eth_xpcs_get_rx_latency(ctxt, ctxt->link_speed,
						    &temp_delay)) {
				LOG_ERR("Failed to obtain XPCS RX latency");
				return;
			}
			rx_delay += temp_delay;
		}

		/* Obtain MAC ingress & egress latency */
		eth_mac_get_egress_latency(ctxt, &temp_delay);
		tx_delay += temp_delay;
		eth_mac_get_ingress_latency(ctxt, &temp_delay);
		rx_delay += temp_delay;
		neg_rx_delay = (ONE_SEC_IN_NANOSEC - rx_delay) |
				MAC_TIMESTAMP_INGRESS_CORR_NEG_BIT;

		/* Set the correction value into register */
		eth_write(base_addr, MAC_TIMESTAMP_EGRESS_CORR_NANOSEC,
			  tx_delay);
		eth_write(base_addr, MAC_TIMESTAMP_INGRESS_CORR_NANOSEC,
			  neg_rx_delay);
	}
}
#endif  /* CONFIG_NET_GPTP && CONFIG_ETH_DWC_EQOS_PTP */

static void eth_phy_irq(struct k_work *item)
{
	struct eth_runtime *ctxt = CONTAINER_OF(item, struct eth_runtime,
						phy_work);
	struct phy_device *phy = &ctxt->phy_dev;
	bool link;
	int32_t speed;
	int8_t duplex;
	int retval;
	uint32_t reg_val;
	uint32_t base_addr = ctxt->base_addr;
	bool intr_sts_chg = false;

	/* To check link status change.
	 * For PHY interrupt mode, read to clear PHY interrupt status.
	 * For PHY polling mode, check the change of link status.
	 */
	if (!ctxt->phy_dev.intr_status(&ctxt->phy_dev, &intr_sts_chg)) {
		if (!intr_sts_chg) {
			goto complete;
		}
	} else {
		LOG_ERR("Failed to obtain PHY IRQ status");
		goto complete;
	}

	retval = phy->read_status(phy, &link, &speed, &duplex);
	if (retval < 0) {
		LOG_ERR("Unable to obtain link information\n");
		goto complete;
	}

	reg_val = eth_read(base_addr, MAC_LPI_CONTROL_STATE);

	/* PHY link status is UP(okay) */
	if (link) {
		ctxt->link_speed = speed;
		ctxt->duplex_mode = duplex;
		ctxt->link_status = LINK_UP;
		eth_mac_config_mode(ctxt);
		net_eth_carrier_on(ctxt->iface);
		LOG_DBG("PHY link up, link speed %d, duplex %s",
			speed, duplex ? "Full" : "Half");
		reg_val |= MAC_LPI_CONTROL_STATE_PLS;
#if defined(CONFIG_ETH_DWC_EQOS_QAV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
		eth_tsn_reconfig(ctxt);
#endif
#if defined(CONFIG_NET_GPTP) && defined(CONFIG_ETH_DWC_EQOS_PTP)
		eth_mac_timestamp_correction(ctxt);
#endif
	} else {
		ctxt->link_status = LINK_DOWN;
		net_eth_carrier_off(ctxt->iface);
		LOG_DBG("PHY link down");
		reg_val &= ~MAC_LPI_CONTROL_STATE_PLS;
#ifdef CONFIG_ETH_DWC_EQOS_QBU
		if (ctxt->fpeparam.enable) {
			dwmac_fpe_reset_lp_status();
		}
#endif
	}

	eth_write(base_addr, MAC_LPI_CONTROL_STATE, reg_val);

complete:
	return;
}

static inline int eth_phy_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	struct phy_device *phy = &context->phy_dev;
	int retval;

	k_work_init(&context->phy_work, eth_phy_irq);

	retval = phy->init(phy);
	if (retval < 0) {
		return retval;
	}

	retval = phy->en_intr(phy);
	if (retval < 0) {
		return retval;
	}

	phy->support = 0;
	retval = phy->cfg_link(phy, context->autoneg, context->link_speed,
			       context->duplex_mode, false);
	if (retval < 0) {
		return retval;
	}

	return 0;
}

#ifdef CONFIG_ETH_DWC_EQOS_PTP
/* refer eth_mcux.c implementation */
static bool eth_ptp_pkt(struct net_if *iface, struct net_pkt *pkt,
			struct net_buf *frag, bool is_tx)
{
	struct gptp_hdr *hdr;

#if defined(CONFIG_NET_VLAN)
	struct net_eth_vlan_hdr *hdr_vlan;
	struct ethernet_context *eth_ctx;
	bool vlan_enabled = false;

	eth_ctx = net_if_l2_data(iface);
	if (net_eth_is_vlan_enabled(eth_ctx, iface)) {
		hdr_vlan = (struct net_eth_vlan_hdr *)frag->data;
		vlan_enabled = true;

		if (ntohs(hdr_vlan->type) != NET_ETH_PTYPE_PTP) {
			return false;
		}
	} else
#endif
	{
		struct net_eth_hdr *eth_hdr = (struct net_eth_hdr *)frag->data;

		if (ntohs(eth_hdr->type) != NET_ETH_PTYPE_PTP) {
			return false;
		}
	}

	if (is_tx) {
		if (!pkt->frags) {
			return false;
		}

		hdr = (struct gptp_hdr *)net_pkt_data(pkt);

		/* Only submit timestamp back to tx timestamp fifo for sync &
		 * pdelay response packets which has timestamp callback routine
		 * registered.
		 */
		if ((hdr->message_type != GPTP_SYNC_MESSAGE) &&
		    (hdr->message_type != GPTP_PATH_DELAY_RESP_MESSAGE)) {
			return false;
		}

	} else {
#if defined(CONFIG_NET_VLAN)
		if (vlan_enabled) {
			hdr = (struct gptp_hdr *)((uint8_t *)net_pkt_data(pkt) +
			      sizeof(struct net_eth_vlan_hdr));
		} else
#endif
		{
			hdr = (struct gptp_hdr *)((uint8_t *)net_pkt_data(pkt) +
			      sizeof(struct net_eth_hdr));
		}
	}

	net_pkt_set_priority(pkt, NET_PRIORITY_CA);

#ifdef CONFIG_ETH_DWC_EQOS_PTP_DEBUG
	LOG_DBG("PTP packet: ver %d type %d len %d ", hdr->ptp_version,
		hdr->message_type, ntohs(hdr->message_length));
	LOG_DBG("PTP packet: clk %02x%02x%02x%02x%02x%02x%02x%02x",
		hdr->port_id.clk_id[0], hdr->port_id.clk_id[1],
		hdr->port_id.clk_id[2], hdr->port_id.clk_id[3],
		hdr->port_id.clk_id[4], hdr->port_id.clk_id[5],
		hdr->port_id.clk_id[6], hdr->port_id.clk_id[7]);
	LOG_DBG("PTP packet: port %d seq %d", ntohs(hdr->port_id.port_number),
		ntohs(hdr->sequence_id));
#endif

	return true;
}

static const struct device *eth_get_ptp_clock(const struct device *port)
{
	struct eth_runtime *context = port->data;

	return context->ptp_clock;
}
#endif /* CONFIG_ETH_DWC_EQOS_PTP */

#ifdef CONFIG_NET_PKT_TIMESTAMP
static bool eth_rx_hwtstamp(volatile struct eth_rx_desc *next_desc,
			    struct net_ptp_time *timestamp)
{
	bool ret = false;

	/* Refer 2.3.3 DMA Receive Operation in datasheet
	 * If IEEE 1588 timestamping is enabled and the timestamp is available
	 * for the previous packet, the DMA writes the timestamp (if available)
	 * to the RDES0 and RDES1 of current descriptor and sets the CTXT field
	 * (RDES3[30]).
	 *
	 * If IEEE 1588 Timestamp feature is enabled, the DMA stores the
	 * timestamp (if available). The DMA writes the context descriptor
	 * after the last descriptor for the current packet (in the next
	 * available descriptor).
	 */
	if (next_desc->rdes3.wb.ctxt) {
		/* invalid timestamp */
		if (next_desc->rdes0.timestamp_low != 0xffffffff &&
		    next_desc->rdes1.timestamp_high != 0xffffffff) {
			ret = true;
			timestamp->second =
				(uint64_t)next_desc->rdes1.timestamp_high;
			timestamp->nanosecond =
				(uint32_t)next_desc->rdes0.timestamp_low;
		}
	}

	return ret;
}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

static void eth_rx_data(struct eth_runtime *context, int queue, int desc_num)
{
	volatile struct eth_rx_desc *desc;
#ifdef CONFIG_NET_PKT_TIMESTAMP
	volatile struct eth_rx_desc *next_desc;
	struct net_ptp_time timestamp = {
		.second = UINT64_MAX,
		.nanosecond = UINT32_MAX,
	};
#endif /* CONFIG_NET_PKT_TIMESTAMP */
	struct net_if *iface = context->iface;
	struct net_pkt *pkt;
	int i, j, buf_idx, ret;
	uint8_t *rxbuf;
#ifdef CONFIG_NET_VLAN
	uint16_t vlan_tag = NET_VLAN_TAG_UNSPEC;
	enum net_priority prio;
	uint8_t vlan_pcp;
#endif /* CONFIG_NET_VLAN */
	int total_desc = 1;

#ifdef CONFIG_ETH_DWC_EQOS_RX_NAPI
	if (!desc_num) {
		context->rx_napi_count[queue]++;
	} else {
		context->rx_napi_count[queue] = 0;
	}
#endif

	for (i = 0; i < desc_num; i++) {
		buf_idx = context->rdesc_ring_rd_ptr[queue];
		desc = eth_put_rx_desc(context, queue);
		if (!desc) {
			LOG_ERR("Unable to retrieve from RX descriptor.");
			continue;
		}

#ifdef CONFIG_NET_PKT_TIMESTAMP
		next_desc = NULL;
		/* Timestamp Available
		 * When Timestamp is present, RDES1 WB BIT14 indicates that the
		 * timestamp value is available in a context descriptor word 2
		 * (RDES2) and word 1(RDES1).
		 * The context descriptor is written in the next descriptor just
		 * after the last normal descriptor for a packet.
		 */
		if (desc->rdes1.tsa && i + 1 < desc_num) {
			next_desc = eth_put_rx_desc(context, queue);

			if (next_desc) {
				if (!eth_rx_hwtstamp(next_desc, &timestamp)) {
					next_desc = NULL;
				}
			}
		}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

		/* TODO: handling of fragmented rx frame */
		/* TODO: handling error descriptor */

		if (net_if_is_up(iface)) {
			pkt = net_pkt_rx_alloc_with_buffer(iface,
							   desc->rdes3.wb.pl,
							   AF_UNSPEC, 0,
							   NET_BUF_TIMEOUT);
			if (!pkt) {
				LOG_ERR("Failed to obtain RX buffer, "
					"packet dropped");
				goto cleanup;
			}

			rxbuf = (uint8_t *)context->rx_buf[queue][buf_idx];

			dcache_invalidate((uintptr_t)rxbuf, desc->rdes3.wb.pl);

			if (net_pkt_write(pkt, (void *)rxbuf,
					  desc->rdes3.wb.pl)) {
				LOG_ERR("Failed to append RX buffer to "
					"context buffer");
				net_pkt_unref(pkt);
				goto cleanup;
			}

#ifdef CONFIG_NET_PKT_TIMESTAMP
			net_pkt_set_timestamp(pkt, &timestamp);
#endif /* CONFIG_NET_PKT_TIMESTAMP */

#ifdef CONFIG_NET_VLAN
#ifdef CONFIG_ETH_DWC_EQOS_VLAN_STRIP
			if (desc->rdes3.wb.rs0v &&
			    desc->rdes3.wb.lt == RDESC2_VLAN_TYPE &&
			    desc->rdes2.ots) {
				/* Extract tag from descriptor to packet */
				net_pkt_set_vlan_tci(pkt, desc->rdes0.ovt);
			}
#else
			struct net_eth_hdr *hdr = NET_ETH_HDR(pkt);

			if (ntohs(hdr->type) == NET_ETH_PTYPE_VLAN) {
				struct net_eth_vlan_hdr *hdr_vlan =
						(struct net_eth_vlan_hdr *)hdr;

				net_pkt_set_vlan_tci(pkt, ntohs(
						     hdr_vlan->vlan.tci));
			}
#endif /* CONFIG_ETH_DWC_EQOS_VLAN_STRIP */

			/* Set receive vlan priority */
			vlan_pcp = net_pkt_vlan_priority(pkt);
			prio = net_vlan2priority(vlan_pcp);
			net_pkt_set_priority(pkt, prio);

			/* Pass packet back to correct VLAN interface */
			vlan_tag = net_pkt_vlan_tag(pkt);
			iface = net_eth_get_vlan_iface(context->iface,
						       vlan_tag);
			if (!iface) {
				iface = context->iface;
			}
#endif /* CONFIG_NET_VLAN */

#ifdef CONFIG_ETH_DWC_EQOS_PTP
			/* Set packet priority for PTP packet */
			eth_ptp_pkt(iface, pkt, pkt->frags, false);
#endif /* CONFIG_ETH_DWC_EQOS_PTP */

			ret = net_recv_data(iface, pkt);
			if (ret < 0) {
				LOG_ERR("Failed to enqueue frame into RX "
					"queue: %d", ret);
				net_pkt_unref(pkt);
			}

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
			/*Count number of rx packet in each queue*/
			context->eth_stats.rx_pkt_n[queue]++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */
		}

cleanup:
#ifdef CONFIG_NET_PKT_TIMESTAMP
		/* Increase the total clearing descriptor number if
		 * timestamp is available in next_desc
		 */
		if (next_desc) {
			i++;
			total_desc = 2;
		}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

		/* Re-initialize the rx descriptor for continuos receive */
		for (j = 0; j < total_desc; j++) {
			desc = eth_get_rx_desc(context, queue);
			if (!desc) {
				LOG_ERR("Failed to retrieve RX descriptor.");
				continue;
			}

			buf_idx = RX_DESC_INDEX(context, queue, desc);
			desc->rdes0.buf1ap =
			       POINTER_TO_DMA(context->rx_buf[queue][buf_idx]);
			desc->rdes1.dword = 0;
			desc->rdes2.dword = 0;
			desc->rdes3.rd.buf1v = 1;
			desc->rdes3.rd.ioc = 1;
#ifdef CONFIG_ETH_DWC_EQOS_INTR_COALESCE
			if (eth_rx_intr_coalesce_value(queue)) {
				desc->rdes3.rd.ioc = 0;
			}
#endif
			desc->rdes3.rd.own = 1;

			dcache_clean((uintptr_t)desc, sizeof(*desc));
		}

		eth_write(context->base_addr, DMA_RXDESC_TAIL_PTR_CH(queue),
			  POINTER_TO_DMA(RX_DESC_TAIL_PTR(context, queue)));
	}

#ifdef CONFIG_ETH_DWC_EQOS_RX_NAPI
	if (context->rx_napi_count[queue] > ETH_DWC_EQOS_NAPI_WAIT) {
		eth_dma_rx_intr_ctrl(context, queue, true);
	} else {
		k_work_reschedule(&context->rx_irq_work[queue],
				  K_MSEC(CONFIG_ETH_DWC_EQOS_POLL_INTERVAL));
	}
#endif
}

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
ETH_RX_IRQ_QUEUE(7);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
ETH_RX_IRQ_QUEUE(6);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
ETH_RX_IRQ_QUEUE(5);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
ETH_RX_IRQ_QUEUE(4);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
ETH_RX_IRQ_QUEUE(3);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
ETH_RX_IRQ_QUEUE(2);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
ETH_RX_IRQ_QUEUE(1);
#endif

ETH_RX_IRQ_QUEUE(0);

static inline void eth_rx_irq(const struct device *port, int queue)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_INTR_STATUS_CH(queue));

	if (reg_val & DMA_CH_INTR_STS_RI) {
#ifdef CONFIG_ETH_DWC_EQOS_RX_NAPI
		eth_dma_rx_intr_ctrl(context, queue, false);
		context->rx_napi_count[queue] = 0;
#endif
		eth_write(base_addr, DMA_INTR_STATUS_CH(queue),
			  DMA_CH_INTR_STS_RI | DMA_CH_INTR_STS_NIS);
		/* TODO: instead of system workqueue,
		 * a separated workqueue needed for performance ?
		 */
		k_work_schedule(&context->rx_irq_work[queue], K_MSEC(0));
	}
}

static void eth_tx_clean(struct eth_runtime *context, int queue, int desc_num)
{
	volatile struct eth_tx_desc *desc;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *edesc;
#endif
	volatile void *temp_desc;
	struct net_pkt *pkt;
	struct net_buf *frag;
	int i, idx;

	for (i = 0; i < desc_num; i++) {
		temp_desc = eth_put_tx_desc(context, queue);
		if (!temp_desc) {
			LOG_ERR("Unable to retrieve from TX descriptor.");
			continue;
		}

#ifdef CONFIG_ETH_DWC_EQOS_TBS
		if (!context->tbs_enabled[queue]) {
			desc = (struct eth_tx_desc *)temp_desc;
			idx = TX_DESC_INDEX(context, queue, desc);

			edesc = &context->enh_tx_desc[queue][idx];
		} else {
			edesc = (enh_desc_t *)temp_desc;
			idx = TX_DESC_INDEX(context, queue, edesc);

			desc = &context->tx_desc[queue][idx];
		}
#else
		desc = (struct eth_tx_desc *)temp_desc;

		idx = TX_DESC_INDEX(context, queue, desc);
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
		pkt = context->tdesc_pkt[queue][idx];
		frag = context->tdesc_frag[queue][idx];

#ifdef CONFIG_NET_PKT_TIMESTAMP
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	bool timestampstats = desc->tdes3.wb.ttss || edesc->tdes3.wb.ttss;
#else
	bool timestampstats = desc->tdes3.wb.ttss;
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
		if (timestampstats && pkt) {
			struct net_ptp_time timestamp;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
			timestamp.second = context->tbs_enabled[queue] ?
						edesc->tdes1.timestamp_high :
						desc->tdes1.timestamp_high;
			timestamp.nanosecond = context->tbs_enabled[queue] ?
						edesc->tdes0.timestamp_low :
						desc->tdes0.timestamp_low;
#else
			timestamp.second = desc->tdes1.timestamp_high;
			timestamp.nanosecond = desc->tdes0.timestamp_low;
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
			net_pkt_set_timestamp(pkt, &timestamp);

#ifdef CONFIG_ETH_DWC_EQOS_PTP
			/* check is ptp packet */
			if (eth_ptp_pkt(context->iface, pkt, frag, true)) {
				net_if_add_tx_timestamp(pkt);
			}
#endif /* CONFIG_ETH_DWC_EQOS_PTP */
		}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

		desc->tdes0.dword = 0;
		desc->tdes1.dword = 0;
		desc->tdes2.dword = 0;
		desc->tdes3.dword = 0;
		dcache_clean((uintptr_t)desc, sizeof(*desc));

#ifdef CONFIG_ETH_DWC_EQOS_TBS
		edesc->etdes4.dword = 0;
		edesc->etdes5.dword = 0;
		edesc->etdes6.dword = 0;
		edesc->etdes7.dword = 0;
		edesc->tdes0.dword = 0;
		edesc->tdes1.dword = 0;
		edesc->tdes2.dword = 0;
		edesc->tdes3.dword = 0;
		dcache_clean((uint32_t)edesc, sizeof(*edesc));
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

               if (pkt) {
                       /* Freeing the L2 header fragment and the packet */
                       net_pkt_frag_unref(frag);
                       net_pkt_unref(pkt);
               }
	}
}

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
ETH_TX_IRQ_QUEUE(7);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
ETH_TX_IRQ_QUEUE(6);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
ETH_TX_IRQ_QUEUE(5);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
ETH_TX_IRQ_QUEUE(4);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
ETH_TX_IRQ_QUEUE(3);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
ETH_TX_IRQ_QUEUE(2);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
ETH_TX_IRQ_QUEUE(1);
#endif

ETH_TX_IRQ_QUEUE(0);

static inline void eth_tx_irq(const struct device *port, int queue)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, DMA_INTR_STATUS_CH(queue));

	if (reg_val & DMA_CH_INTR_STS_TI) {
		eth_write(base_addr, DMA_INTR_STATUS_CH(queue),
			  DMA_CH_INTR_STS_TI | DMA_CH_INTR_STS_NIS);
		/* TODO: instead of system workqueue,
		 * a separated workqueue needed for performance ?
		 */
		k_work_submit(&context->tx_irq_work[queue]);
	}
}

static inline void eth_ecc_irq(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	/* Clear ECC interrupts */
	reg_val = eth_read(base_addr, MTL_ECC_INTR_STS);
	eth_write(base_addr, MTL_ECC_INTR_STS, reg_val);

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	if (reg_val & MTL_ECC_INTR_STS_TXCES) {
		context->eth_stats.ecc_err_count[ETH_ECC_TXCES]++;
	}

	if (reg_val & MTL_ECC_INTR_STS_TXUES) {
		context->eth_stats.ecc_err_count[ETH_ECC_TXUES]++;
	}

	if (reg_val & MTL_ECC_INTR_STS_RXCES) {
		context->eth_stats.ecc_err_count[ETH_ECC_RXCES]++;
	}

	if (reg_val & MTL_ECC_INTR_STS_RXUES) {
		context->eth_stats.ecc_err_count[ETH_ECC_RXUES]++;
	}

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	if (reg_val & MTL_ECC_INTR_STS_ECES) {
		context->eth_stats.ecc_err_count[ETH_ECC_ECES]++;
	}

	if (reg_val & MTL_ECC_INTR_STS_EUES) {
		context->eth_stats.ecc_err_count[ETH_ECC_EUES]++;
	}
#endif
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */
}

static void eth_dwc_eqos_isr(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;

	/* TODO: handling pmt/eee interrupt */

#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
	k_work_submit(&context->phy_work);
#else
	reg_val = eth_read(base_addr, MAC_INTERRUPT_STATUS);
	if (reg_val & MAC_INTR_STS_PHYIS) {
		k_work_submit(&context->phy_work);
	}
#endif

#if defined(CONFIG_ETH_DWC_EQOS_POLLING_MODE) || \
	!defined(ETH_DWC_EQOS_MULTI_IRQ)
	reg_val = eth_read(base_addr, DMA_INTERRUPT_STATUS);
	for (int i = 0; i < DMA_INTR_STS_CHNL_MAX; i++) {
		if (reg_val & DMA_INTR_STS_DCIS(i)) {
			eth_rx_irq(port, i);
			eth_tx_irq(port, i);
		}
	}
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	reg_val = eth_read(base_addr, MTL_INTERRUPT_STATUS);
	if (reg_val & MTL_INTR_ESTIS) {
		dwmac_est_irq_status(port);
	}
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	reg_val = eth_read(base_addr, MAC_INTERRUPT_STATUS);
	if (reg_val & MAC_INTR_STS_FPEIS) {
		int fpe_state = dwmac_fpe_irq_status(base_addr);

		if ((fpe_state & FPE_STATE_RVER) && context->fpeparam.enable) {
			dwmac_fpe_send_mpacket(base_addr, MPACKET_RESPONSE);
		}
	}
#endif

#if defined(CONFIG_ETH_DWC_EQOS_POLLING_MODE) || \
	!defined(ETH_DWC_EQOS_MULTI_IRQ)
	/* Check ECC interrupt status */
	reg_val = eth_read(base_addr, MTL_SAFETY_INTR_STS);
	if (reg_val & (MTL_SAFETY_INTR_STS_MEUIS | MTL_SAFETY_INTR_STS_MECIS)) {
		eth_ecc_irq(port);
	}
#endif
}

#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
ETH_DWC_EQOS_TX_ISR(7);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
ETH_DWC_EQOS_TX_ISR(6);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
ETH_DWC_EQOS_TX_ISR(5);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
ETH_DWC_EQOS_TX_ISR(4);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
ETH_DWC_EQOS_TX_ISR(3);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
ETH_DWC_EQOS_TX_ISR(2);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
ETH_DWC_EQOS_TX_ISR(1);
#endif

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
ETH_DWC_EQOS_TX_ISR(0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
ETH_DWC_EQOS_RX_ISR(7);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
ETH_DWC_EQOS_RX_ISR(6);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
ETH_DWC_EQOS_RX_ISR(5);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
ETH_DWC_EQOS_RX_ISR(4);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
ETH_DWC_EQOS_RX_ISR(3);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
ETH_DWC_EQOS_RX_ISR(2);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
ETH_DWC_EQOS_RX_ISR(1);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
ETH_DWC_EQOS_RX_ISR(0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */

#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
static void eth_irq_poll(struct k_timer *timer)
{
	struct eth_runtime *context = CONTAINER_OF(timer, struct eth_runtime,
						   polling_timer);
	eth_dwc_eqos_isr(context->port);
}
#endif /* CONFIG_ETH_DWC_EQOS_POLLING_MODE */

#ifdef CONFIG_ETH_PHY_POLLING_MODE
static void eth_phy_poll(struct k_timer *timer)
{
	struct eth_runtime *context = CONTAINER_OF(timer, struct eth_runtime,
						   phy_polling_timer);
	k_work_submit(&context->phy_work);
}
#endif /* CONFIG_ETH_PHY_POLLING_MODE */

static inline int eth_intr_enable(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	int i, total_queues = MAX(context->txqnum, context->rxqnum);

	/* Set interrupt mode */
	reg_val = eth_read(base_addr, DMA_MODE);
	reg_val &= INV_DMA_MD_INTM;
#if !defined(CONFIG_ETH_DWC_EQOS_POLLING_MODE) && \
	defined(ETH_DWC_EQOS_MULTI_IRQ)
	reg_val |= DMA_MD_INTM_MODE1;
#endif
	eth_write(base_addr, DMA_MODE, reg_val);

	for (i = 0; i < total_queues; i++) {
		/* Enable DMA normal interrupt summary, receive watchdog,
		 * TX & RX interrupts.
		 */
		reg_val = DMA_CH_INTR_EN_NIE | DMA_CH_INTR_EN_RWTE;
		eth_write(base_addr, DMA_INTR_EN_CH(i), reg_val);
		if (i < context->txqnum) {
			eth_dma_tx_intr_ctrl(context, i, true);
		}
		if (i < context->rxqnum) {
			eth_dma_rx_intr_ctrl(context, i, true);
#ifdef CONFIG_ETH_DWC_EQOS_INTR_COALESCE
			eth_dma_rx_intr_wtd(context, i);
#endif
		}
	}

	/* Enable PHY interrupt */
	reg_val = MAC_INTR_EN_PHYIE;
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	/* Enable Frame Preemption interrupt */
	reg_val |= MAC_INTR_EN_FPEIE;
#endif
	eth_write(base_addr, MAC_INTERRUPT_ENABLE, reg_val);

#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
	k_timer_init(&context->polling_timer, eth_irq_poll, NULL);
#endif

#ifdef CONFIG_ETH_PHY_POLLING_MODE
	k_timer_init(&context->phy_polling_timer, eth_phy_poll, NULL);
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	dwmac_set_est_intr(base_addr);
#endif

	reg_val = eth_read(base_addr, MTL_ECC_INTR_EN);
	/* Enable ECC correctable error interrupt for MTL Rx/Tx FIFO */
	reg_val |= MTL_ECC_INTR_EN_RXCEIE | MTL_ECC_INTR_EN_TXCEIE;
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	/* When gcl_depth is not equal to 0, it means EST feature is supported.
	 * Thus, enable ECC correctable error interrupt for MTL EST.
	 */
	if (context->estparam.gcl_depth) {
		reg_val |= MTL_ECC_INTR_EN_ECEIE;
	}
#endif
	eth_write(base_addr, MTL_ECC_INTR_EN, reg_val);

	return 0;
}

static inline void eth_irq_work_init(const struct device *port)
{
	struct eth_runtime *context = port->data;

#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	INIT_ETH_TX_IRQ_WORK(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	INIT_ETH_TX_IRQ_WORK(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	INIT_ETH_TX_IRQ_WORK(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	INIT_ETH_TX_IRQ_WORK(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	INIT_ETH_TX_IRQ_WORK(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	INIT_ETH_TX_IRQ_WORK(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	INIT_ETH_TX_IRQ_WORK(1);
#endif
	INIT_ETH_TX_IRQ_WORK(0);

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	INIT_ETH_RX_IRQ_WORK(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	INIT_ETH_RX_IRQ_WORK(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	INIT_ETH_RX_IRQ_WORK(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	INIT_ETH_RX_IRQ_WORK(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	INIT_ETH_RX_IRQ_WORK(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	INIT_ETH_RX_IRQ_WORK(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	INIT_ETH_RX_IRQ_WORK(1);
#endif
	INIT_ETH_RX_IRQ_WORK(0);
}

static inline int eth_mac_stop_transaction(const struct device *port)
{
	struct eth_runtime *context = port->data;
	int i;

	/* Disable MAC layer receive */
	eth_rx_mac_ctrl(context, false);
	for (i = 0; i < context->rxqnum; i++) {
		/* Disable DMA layer receive channels */
		eth_rx_dma_ctrl(context, i, false);
	}

	for (i = 0; i < context->txqnum; i++) {
		/* Disable DMA layer transmit channels */
		eth_tx_dma_ctrl(context, i, false);
	}

	/* Disable MAC layer transmit */
	eth_tx_mac_ctrl(context, false);

#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
	k_timer_stop(&context->polling_timer);
#endif

	return 0;
}

static inline int eth_mac_start_transaction(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	int i;

	for (i = 0; i < context->rxqnum; i++) {
		/* Enable DMA layer receive channels */
		eth_rx_dma_ctrl(context, i, true);
	}

	for (i = 0; i < context->txqnum; i++) {
		/* Enable MTL layer transmit queues */
		reg_val = eth_read(base_addr, MTL_TXQ_OPERATION_MODE(i));
		reg_val &= INV_MTL_TXQ_OPR_TXQEN;
#ifdef CONFIG_ETH_DWC_EQOS_QAV
		reg_val |= MTL_TXQ_OPR_TXQEN_EN_IF_AV;
#else
		reg_val |= MTL_TXQ_OPR_TXQEN_EN;
#endif
		eth_write(base_addr, MTL_TXQ_OPERATION_MODE(i), reg_val);

		/* Enable DMA layer transmit channels */
		eth_tx_dma_ctrl(context, i, true);
	}

	/* Enable MAC layer receive & transmit */
	eth_tx_mac_ctrl(context, true);
	eth_rx_mac_ctrl(context, true);

#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
	k_timer_start(&context->polling_timer,
		      K_MSEC(CONFIG_ETH_DWC_EQOS_POLL_INTERVAL),
		      K_MSEC(CONFIG_ETH_DWC_EQOS_POLL_INTERVAL));
#endif

#ifdef CONFIG_ETH_PHY_POLLING_MODE
	k_timer_start(&context->phy_polling_timer,
		      K_MSEC(CONFIG_ETH_PHY_POLLING_INTERVAL),
		      K_MSEC(CONFIG_ETH_PHY_POLLING_INTERVAL));
#endif

	return 0;
}

static void eth_tx_data(struct eth_runtime *context, int queue, uint8_t *data,
			uint16_t len, size_t total, int first, struct net_pkt *pkt,
			uint64_t txtime)
{
	volatile struct eth_tx_desc *desc;
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t *edesc;
#endif
	volatile void *temp_desc;
	mm_reg_t base_addr = context->base_addr;
	int idx;

	while (!(temp_desc = eth_get_tx_desc(context, queue))) {
		k_sleep(K_MSEC(1)); /* 1ms polling interval */
	}

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	if (context->tbs_enabled[queue]) {
		edesc = (enh_desc_t *)temp_desc;

		edesc->tdes0.buf1ap = data;
		edesc->tdes1.dword = 0;
		edesc->tdes2.rd.hl_bl1 = len;
		edesc->tdes3.rd.fl = total;
		edesc->tdes3.rd.cic = TX_DESC_CHECKSUM_EN;
		/* net_pkt will be passed in only on the last fragment of packet */
		if (pkt) {
			edesc->tdes3.rd.ld = 1;
			edesc->tdes2.rd.ioc = 1;
		}
		if (first) {
			edesc->tdes3.rd.fd = 1;
#ifdef CONFIG_NET_PKT_TIMESTAMP
			if (context->flags & ETH_DEV_FLAGS_TSSEL) {
				edesc->tdes2.rd.ttse_tmwd = 1;
			}
#endif /* CONFIG_NET_PKT_TIMESTAMP */
			if (txtime) {
				edesc->etdes5.lt_ns = (txtime &
						ENH_TX_DESC_LT_NANOSEC_MASK) >>
						ENH_TX_DESC_LT_NANOSEC_SHIFT;
				edesc->etdes4.lt_s = (txtime >>
						ENH_TX_DESC_LT_SEC_SHIFT) &
						ENH_TX_DESC_LT_SEC_MASK;
				edesc->etdes4.ltv = 1;
			}
		}
		edesc->tdes3.rd.own = 1;

		idx = TX_DESC_INDEX(context, queue, edesc);

		dcache_clean((uint32_t)edesc, sizeof(*edesc));
	} else
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
	{
		desc = (struct eth_tx_desc *)temp_desc;

		desc->tdes0.buf1ap = POINTER_TO_DMA(data);
		desc->tdes1.dword = 0;
		desc->tdes2.rd.hl_bl1 = len;
		desc->tdes3.rd.fl = total;
		desc->tdes3.rd.cic = TX_DESC_CHECKSUM_EN;
		/* net_pkt will be passed in only on the last fragment of packet */
		if (pkt) {
			desc->tdes3.rd.ld = 1;
			desc->tdes2.rd.ioc = 1;
		}
		if (first) {
			desc->tdes3.rd.fd = 1;
#ifdef CONFIG_NET_PKT_TIMESTAMP
			if (context->flags & ETH_DEV_FLAGS_TSSEL) {
				desc->tdes2.rd.ttse_tmwd = 1;
			}
#endif /* CONFIG_NET_PKT_TIMESTAMP */
		}
		desc->tdes3.rd.own = 1;

		idx = TX_DESC_INDEX(context, queue, desc);

		dcache_clean((uintptr_t)desc, sizeof(*desc));
	}

	dcache_clean((uintptr_t)data, len);

	/* Store the net_pkt for eth_tx_clean() to retrieve */
	context->tdesc_pkt[queue][idx] = pkt;
	/* ethernet_send() will remove 1st fragment buffer after return from
	 * eth_tx(), store it for retrieving in eth_tx_clean()
	 */
	context->tdesc_frag[queue][idx] = pkt ? pkt->frags : NULL;

	if (pkt) {
		eth_write(base_addr, DMA_TXDESC_TAIL_PTR_CH(queue),
			  POINTER_TO_DMA(TX_DESC_TAIL_PTR(context, queue)));
	}
}

static int eth_tx(const struct device *port, struct net_pkt *pkt)
{
	struct eth_runtime *context = port->data;
	struct net_if *iface = context->iface;
	struct ethernet_context *ctx = net_if_l2_data(iface);
	struct net_buf *frag;
	size_t total_len = net_pkt_get_len(pkt);
	int q = 0;
	uint64_t txtime = 0;

	/* Ensure packet transmission after initialization fully completed
	 * or link is up
	 */
	if (!ctx->is_init || !context->link_status) {
		return -ENETDOWN;
	}

	/* Hold the packet until TX completed */
	net_pkt_ref(pkt);

#if (NET_TC_COUNT > 1)
	q = net_tx_priority2tc(net_pkt_priority(pkt));
	if (q >= context->txqnum) {
		q = context->txqnum - 1;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	if (context->tbs_enabled[q]) {
		txtime = pkt->txtime;
	}
#endif

	for (frag = pkt->frags; frag; frag = frag->frags) {
		if (frag == pkt->frags) {
			/* Let's prevent L2 freeing the header fragment */
			net_pkt_frag_ref(frag);
		}

		eth_tx_data(context, q, frag->data, frag->len, total_len,
			    frag == pkt->frags ? 1 : 0,
			    !frag->frags ? pkt : NULL, txtime);
	}

	return 0;
}

#ifdef CONFIG_NET_PKT_TIMESTAMP
static int eth_mac_set_time(struct eth_runtime *context, uint32_t sec, uint32_t nsec)
{
	int limit = 10;
	uint32_t value;
	mm_reg_t base_addr = context->base_addr;

	eth_write(base_addr, MAC_SYS_TIME_SEC_UPD, sec);
	eth_write(base_addr, MAC_SYS_TIME_NANOSEC_UPD, nsec);
	/* issue command to initialize the system time value */
	value = eth_read(base_addr, MAC_TIMESTAMP_CTRL);
	value |= MAC_TIMESTAMP_CTRL_TSINIT;
	eth_write(base_addr, MAC_TIMESTAMP_CTRL, value);

	/* wait for present system time initialize to complete */
	while (limit--) {
		if (!(eth_read(base_addr, MAC_TIMESTAMP_CTRL) &
		    MAC_TIMESTAMP_CTRL_TSINIT)) {
			break;
		}

		k_sleep(K_MSEC(10));
	}

	if (limit < 0) {
		return -EBUSY;
	}

	return 0;
}

static int eth_mac_config_addend(struct eth_runtime *context, uint32_t addend)
{
	int limit = 10;
	uint32_t value;
	mm_reg_t base_addr = context->base_addr;

	eth_write(base_addr, MAC_TIMESTAMP_ADDEND, addend);
	/* issue command to initialize the system time value */
	value = eth_read(base_addr, MAC_TIMESTAMP_CTRL);
	value |= MAC_TIMESTAMP_CTRL_TSADDREG;
	eth_write(base_addr, MAC_TIMESTAMP_CTRL, value);

	/* wait for present system time initialize to complete */
	while (limit--) {
		if (!(eth_read(base_addr, MAC_TIMESTAMP_CTRL) &
		    MAC_TIMESTAMP_CTRL_TSADDREG)) {
			break;
		}

		k_sleep(K_MSEC(10));
	}

	if (limit < 0) {
		return -EBUSY;
	}

	return 0;
}

static int eth_mac_sys_time_init(const struct device *port)
{
	struct eth_runtime *context = port->data;
	mm_reg_t base_addr = context->base_addr;
	uint32_t reg_val;
	uint32_t sec_inc;
	uint64_t temp = 0;
	int retval = 0;

	if (!(context->flags & ETH_DEV_FLAGS_TSSEL)) {
		return -ENOTSUP;
	}

	/* Select PLL_PTP clock source */
	reg_val = MAC_GPIO_STS_GPO_PTP_200MHZ;
	eth_write(base_addr, MAC_GPIO_STATUS, reg_val);

	/* Enable timestamp for all packet */
	reg_val = (MAC_TIMESTAMP_CTRL_TSENA | MAC_TIMESTAMP_CTRL_TSCFUPDT |
		   MAC_TIMESTAMP_CTRL_TSENALL | MAC_TIMESTAMP_CTRL_TSCTRLSSR);
	eth_write(base_addr, MAC_TIMESTAMP_CTRL, reg_val);

	/* Fine Update PTP clock is 50 MHz (period is 20 ns),
	 * program 20 (0x14) when the System Time Nanoseconds
	 * register has an accuracy of 1 ns
	 */
	sec_inc = (NSEC_PER_SEC / PTP_CLOCK_FINE_UPDATE);
	sec_inc &= MAC_SUB_SECOND_INCR_SSINC_MASK;
	reg_val = sec_inc;
	reg_val <<= MAC_SUB_SECOND_INCR_SSINC_SHIFT;
	eth_write(base_addr, MAC_SUB_SECOND_INCR, reg_val);

	/* The frequency division is the ratio of the reference clock frequency
	 * to the required clock frequency. For example, if the reference clock
	 * (clk_ptp_ref_i) is 62.5 MHz, this ratio is calculated as
	 * 62.5 MHz / 50 MHz = 1.25. Therefore, the default addend value to
	 * be set in the register is 2^32/ 1.25, 0xCCCCCCCC
	 */
	temp = (uint64_t)PTP_CLOCK_FINE_UPDATE << 32;
	context->default_addend = temp / context->ptp_clock_rate;

	retval = eth_mac_config_addend(context, context->default_addend);

	retval |= eth_mac_set_time(context, 0, 0);

	return retval;
}
#endif /* CONFIG_NET_PKT_TIMESTAMP */

#ifdef CONFIG_NET_IPV6
static void eth_mac_mcast_cb(struct net_if *iface,
			     const struct in6_addr *addr,
			     bool is_joined)
{
	const struct device *port = net_if_get_device(iface);
	struct eth_runtime *ctxt = port->data;
	struct net_eth_addr mac_addr;

	net_eth_ipv6_mcast_to_mac_addr(addr, &mac_addr);

	eth_mac_set_filter(ctxt, mac_addr.addr, 0, true, !is_joined);
}
#endif /* CONFIG_NET_IPV6 */

#ifdef CONFIG_NET_PROMISCUOUS_MODE
static inline int eth_mac_set_promiscuous(struct eth_runtime *ctxt,
					  bool promisc_mode)
{
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;

	reg_val = eth_read(base_addr, MAC_PACKET_FILTER);
	if ((promisc_mode && (reg_val & MAC_PKT_FLTR_PR)) ||
	    (!promisc_mode && !(reg_val & MAC_PKT_FLTR_PR))) {
		return -EALREADY;
	}

	LOG_DBG("%s promiscuous mode", promisc_mode ? "Set" : "Unset");
	if (promisc_mode) {
		reg_val |= MAC_PKT_FLTR_PR;
	} else {
#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
		LOG_WRN("Unable to turn off DWC EQoS MAC promiscuous mode. "
			"It is required by flexible receive parser operation");
		return 0;
#else
		reg_val &= ~MAC_PKT_FLTR_PR;
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */
	}

	eth_write(base_addr, MAC_PACKET_FILTER, reg_val);
	return 0;
}
#endif /* CONFIG_NET_PROMISCUOUS_MODE */

#ifdef CONFIG_ETH_DWC_EQOS_QAV
static void eth_cbs_init(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	int i;

	if (ctxt->flags | ETH_DEV_FLAGS_QAV) {
		for (i = 1; i < ctxt->txqnum; i++) {
			dwmac_set_cbs_hilocredit(base_addr, i);
		}
	} else {
		LOG_INF("HW does not support 802.1Qav(CBS).");
	}
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */

#ifdef CONFIG_ETH_DWC_EQOS_QBV
static int eth_est_init(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	struct est_gc_entry *gcl[EST_GCL_BANK_MAX];
	int bank;
	unsigned int current_time;
	int ret = 0;

	if (ctxt->flags & ETH_DEV_FLAGS_QBV) {
		dwmac_set_tsn_feat(TSN_FEAT_ID_EST, 1);
	} else {
		LOG_INF("HW does not support 802.1Qbv(EST).");
		return 0;
	}

	/* allocate memory for gcl*/
	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		gcl[bank] = k_calloc(ctxt->estparam.gcl_depth, sizeof(*gcl));
		if (!gcl[bank]) {
			ret = -ENOMEM;
			break;
		}
		dwmac_set_est_gcb(gcl[bank], bank);
	}
	if (ret) {
		int i;

		for (i = bank - 1; i >= 0; i--) {
			k_free(gcl[bank]);
			dwmac_set_est_gcb(NULL, bank);
		}
		LOG_ERR("Could not allocate memory for GCL.");
		return ret;
	}

	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_EST_TILS,
				&ctxt->estparam.tils);
	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_EST_PTOV,
				&ctxt->estparam.ptov);
	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_EST_CTOV,
				&ctxt->estparam.ctov);

	if (ctxt->estparam.enable) {
		dwmac_set_est_enable(base_addr, 1);
		dwmac_set_est_gcrr_llr(base_addr, ctxt->estparam.gcrr.llr,
				       0, 0);
		for (int i = 0; i < ctxt->estparam.gcrr.llr; i++) {
#ifdef CONFIG_ETH_DWC_EQOS_QBU
			/* Set Bit0 when FPE is set to hold and vice versa */
			if (ctxt->estparam.mode[i] == GCL_MODE_HOLD) {
				ctxt->estparam.gce[i].gates |= FPE_PMAC_BIT;
			} else if (ctxt->estparam.mode[i] ==
				   GCL_MODE_RELEASE) {
				ctxt->estparam.gce[i].gates &= ~FPE_PMAC_BIT;
			}
#endif
			dwmac_set_est_gce(base_addr, &ctxt->estparam.gce[i],
					  i, 0, 0);
		}

		/* Start to exercute GCL after 5 seconds */
		current_time = eth_read(base_addr, MAC_SYS_TIME_SEC);
		ctxt->estparam.gcrr.base_sec = current_time + 5;
		dwmac_set_est_gcrr_times(base_addr, &ctxt->estparam.gcrr, 0, 0);

	}
	return 0;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */

#ifdef CONFIG_ETH_DWC_EQOS_QBU
static int eth_fpe_init(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	struct fpe_config fpec;
	int ret = 0;
	int est_status = 0;

	if (ctxt->flags & ETH_DEV_FLAGS_QBU) {
		dwmac_set_tsn_feat(TSN_FEAT_ID_FPE, 1);
	} else {
		LOG_INF("HW does not support 802.1Qbu(FPE).");
		return 0;
	}

	dwmac_set_fpe_fprq(base_addr, ctxt->fpeparam.fprq);
	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_AFSZ,
				&ctxt->fpeparam.afsz);
	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_HADV,
				&ctxt->fpeparam.hadv);
	dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_RADV,
				&ctxt->fpeparam.radv);

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	if (ctxt->estparam.enable) {
		est_status = 1;
	}
#endif
	fpec.txqpec = ctxt->fpeparam.fpst;
	ret = dwmac_set_fpe_config(base_addr, &fpec, est_status);
	if (ret == 0 && ctxt->fpeparam.enable) {
		ret = dwmac_set_fpe_enable(base_addr, 1);
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

static void generate_mac(uint8_t *mac_addr)
{
	sys_rand_get(mac_addr, 6);

       /* Locally administered, unicast */
       mac_addr[0] |= 0x02;
}

static int eth_initialize_internal(struct net_if *iface)
{
	const struct device *port = net_if_get_device(iface);
	struct eth_runtime *context = port->data;
	const struct eth_config *config = port->config;
	mm_reg_t base_addr = context->base_addr;
	uint32_t addr_hi, addr_lo;
	uint8_t mac_ver;
	int retval;
#ifdef CONFIG_NET_VLAN
	struct ethernet_context *ethctx = net_if_l2_data(iface);

	/* Only the real interface will be HW initialized,
	 * no initialization needed for virtual interface.
	 */
	if (ethctx->is_init) {
		ethernet_init(iface);
		return 0;
	}
#endif /* CONFIG_NET_VLAN */

	/* Set device_busy flag to prevent system enters low power state */
	pm_device_busy_set(port);

	/* Prevent system hang issue if initialization failed */
	net_if_set_link_addr(iface, uninitialized_macaddr,
			     sizeof(uninitialized_macaddr), NET_LINK_ETHERNET);

	context->iface = iface;
	net_if_flag_set(context->iface, NET_IF_NO_AUTO_START);

	/* Obtain platform data at runtime */
	if (context->get_platdata) {
		retval = context->get_platdata(port);
		if (retval < 0) {
			return retval;
		}
	}

#ifdef CONFIG_ETH_DWC_EQOS_PCI
	if (!(context->flags & ETH_DEV_FLAGS_PCI_DETECTED)) {
		return -ENODEV;
	}
#endif

	/* Check IP version matching */
	mac_ver = eth_read(base_addr, MAC_VERSION) & MAC_VERSION_MASK;
	if (mac_ver < 0x50) {
		LOG_ERR("Invalid DWC EQoS MAC IP Version 0x%02X", mac_ver);
		return -ENODEV;
	}
	LOG_INF("DWC EQoS MAC IP Version 0x%02X", mac_ver);

	/* Read the MAC address from the device. */
	addr_hi = eth_read(base_addr, MAC_ADDRESS_HIGH(0)) & 0xFFFF;
	addr_lo = eth_read(base_addr, MAC_ADDRESS_LOW(0))  & 0xFFFFFFFF;

	/* get random MAC addr, if device register has invalid value */
	if ((addr_lo == 0xFFFFFFFF && addr_hi == 0xFFFF) ||
	    (addr_lo == 0 && addr_hi == 0)) {
		generate_mac(context->mac_addr.bytes);

		eth_write(base_addr, MAC_ADDRESS_HIGH(0),
			  addr_hi | MAC_ADDR_HI_AE);
		eth_write(base_addr, MAC_ADDRESS_LOW(0), addr_lo);
	} else {
		context->mac_addr.words[1] = addr_hi;
		context->mac_addr.words[0] = addr_lo;
	}
	LOG_INF("DWC EQoS MAC addr %02X:%02X:%02X:%02X:%02X:%02X",
		context->mac_addr.bytes[0], context->mac_addr.bytes[1],
		context->mac_addr.bytes[2], context->mac_addr.bytes[3],
		context->mac_addr.bytes[4], context->mac_addr.bytes[5]);

	retval = eth_get_hw_capabilities(port);
	if (retval < 0) {
		return retval;
	}

	k_sem_init(&context->phy_dev.phy_lock, 1, 1);
	if (context->use_xpcs) {
		retval = eth_modphy_init(port);
		if (retval < 0) {
			return retval;
		}

		retval = eth_xpcs_init(port);
		if (retval < 0) {
			return retval;
		}
	}

	retval = eth_phy_init(port);
	if (retval < 0) {
		return retval;
	}

	/* software reset */
	retval = eth_reset(port);
	if (retval < 0) {
		return retval;
	}

	retval = eth_dma_init(port);
	if (retval < 0) {
		return retval;
	}

	retval = eth_mtl_init(port);
	if (retval < 0) {
		return retval;
	}

	retval = eth_mac_init(port);
	if (retval < 0) {
		return retval;
	}

	if (context->use_eee && context->lpi_timer) {
		retval = eth_mac_eee_init(port);
		if (retval < 0) {
			return retval;
		}
	}

	config->config_func(port);

	retval = eth_intr_enable(port);
	if (retval < 0) {
		return retval;
	}

	eth_irq_work_init(port);

#ifdef CONFIG_NET_PKT_TIMESTAMP
	retval = eth_mac_sys_time_init(port);
	if (retval == -ENOTSUP) {
		LOG_WRN("Ethernet device not support HW timestamping");
	} else if (retval < 0) {
		return retval;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	retval = eth_est_init(port);
	if (retval < 0) {
		return retval;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QAV
	eth_cbs_init(port);
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QBU
	retval = eth_fpe_init(port);
	if (retval < 0) {
		return retval;
	}
#endif

	retval = eth_mac_start_transaction(port);
	if (retval < 0) {
		return retval;
	}

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	memcpy(context->stats.vendor, statreg, sizeof(statreg));
#endif

	net_if_set_link_addr(context->iface, context->mac_addr.bytes,
			     sizeof(context->mac_addr.bytes),
			     NET_LINK_ETHERNET);
	ethernet_init(context->iface);

#ifdef CONFIG_NET_IPV6
	net_if_mcast_mon_register(&context->mcast_mon, context->iface,
				  eth_mac_mcast_cb);
#endif /* CONFIG_NET_IPV6 */

	return 0;
}

static void eth_initialize(struct net_if *iface)
{
	int r = eth_initialize_internal(iface);

	if (r < 0) {
		LOG_ERR("Could not initialize ethernet device: %d", r);
	}
}

static enum ethernet_hw_caps eth_get_capabilities(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	enum phy_support support = ctxt->phy_dev.support;
	int retval = ETHERNET_AUTO_NEGOTIATION_SET;
	uint32_t base_t_10 = PHY_SUPPORT_10_FULL | PHY_SUPPORT_10_HALF;
	uint32_t base_t_100 = PHY_SUPPORT_100_FULL | PHY_SUPPORT_100_HALF;
	uint32_t base_t_1000 = PHY_SUPPORT_1000_FULL | PHY_SUPPORT_1000_HALF;

	if (support & base_t_10) {
		retval |= ETHERNET_LINK_10BASE_T;
	}
	if (support & base_t_100) {
		retval |= ETHERNET_LINK_100BASE_T;
	}
	if (support & base_t_1000) {
		retval |= ETHERNET_LINK_1000BASE_T;
	}

	if (((support & base_t_10) == base_t_10) ||
	    ((support & base_t_100) == base_t_100) ||
	    ((support & base_t_1000) == base_t_1000)) {
		retval |= ETHERNET_DUPLEX_SET;
	}

	if (ctxt->flags & ETH_DEV_FLAGS_TX_CSUM_OFFLOAD) {
		retval |= ETHERNET_HW_TX_CHKSUM_OFFLOAD;
	}
	if (ctxt->flags & ETH_DEV_FLAGS_RX_CSUM_OFFLOAD) {
		retval |= ETHERNET_HW_RX_CHKSUM_OFFLOAD;
	}

	if (ctxt->addrcnt > 0 && ctxt->hashtblsz > 0) {
		retval |= ETHERNET_HW_FILTERING;
	}

#ifdef CONFIG_ETH_DWC_EQOS_PTP
	if (ctxt->flags & ETH_DEV_FLAGS_TSSEL) {
		retval |= ETHERNET_PTP;
	}
#endif /* CONFIG_ETH_DWC_EQOS_PTP */

#ifdef CONFIG_NET_PROMISCUOUS_MODE
	retval |= ETHERNET_PROMISC_MODE;
#endif

#ifdef CONFIG_NET_VLAN
	if (ctxt->vlancnt > 0) {
		retval |= ETHERNET_HW_VLAN;
#ifdef CONFIG_ETH_DWC_EQOS_VLAN_STRIP
		retval |= ETHERNET_HW_VLAN_TAG_STRIP;
#endif /* CONFIG_ETH_DWC_EQOS_VLAN_STRIP */
	}
#endif /* CONFIG_NET_VLAN */

#ifdef CONFIG_ETH_DWC_EQOS_QAV
	if (ctxt->flags & ETH_DEV_FLAGS_QAV) {
		retval |= ETHERNET_QAV;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	if (ctxt->flags & ETH_DEV_FLAGS_QBV) {
		retval |= ETHERNET_QBV;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_QBU
	if (ctxt->flags & ETH_DEV_FLAGS_QBU) {
		retval |= ETHERNET_QBU;
	}
#endif

#ifdef CONFIG_ETH_DWC_EQOS_TBS
	if (ctxt->flags & ETH_DEV_FLAGS_TBS) {
		retval |= ETHERNET_TXTIME;
	}
#endif

	return retval;
}

#if CONFIG_ETH_DWC_EQOS_QAV
static int eth_set_cbs_param(const struct device *port,
			     enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	int link_speed = ctxt->link_speed;
	int retval = 0;
	int queue;
	int txq;

	queue = config->qav_param.queue_id;
	txq = queue - 1;

	/* Check if the queue number is supported */
	if (queue >= ctxt->txqnum || queue == 0) {
		retval = -ENOTSUP;
		goto error;
	}

	/* Enable bandwidth and status setting when link is down */
	if (ctxt->link_status == LINK_DOWN) {
		switch (config->qav_param.type) {
		case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
			ctxt->cbsparam[txq].bandwidth =
				config->qav_param.delta_bandwidth;
			break;
		case ETHERNET_QAV_PARAM_TYPE_STATUS:
			ctxt->cbsparam[txq].enable = config->qav_param.enabled;
			break;
		default:
			retval = -ENOTSUP;
		}

		return retval;
	}

	/* Check if the link speed is supported */
	if (link_speed != 100 && link_speed != 1000 &&
	    link_speed != 2500) {
		retval = -ENOTSUP;
		goto error;
	}

	switch (config->qav_param.type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		ctxt->cbsparam[txq].enable = config->qav_param.enabled;
		dwmac_set_cbs_status(base_addr, queue,
				     config->qav_param.enabled);
		break;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		if (BPS_TO_MBPS(config->qav_param.idle_slope) > link_speed) {
			retval = -EINVAL;
			break;
		}
		ctxt->cbsparam[txq].idle_slope =
			BPS_TO_MBPS(config->qav_param.idle_slope);
		ctxt->cbsparam[txq].bandwidth =
			DECI_TO_PERCENT(ctxt->cbsparam[txq].idle_slope,
					link_speed);
#ifdef CONFIG_ETH_DWC_EQOS_QBV
		/* Recalculate idle slope based on oper GCL */
		if (ctxt->estparam.enable) {
			dwmac_cbs_recal_idleslope(base_addr,
				&ctxt->cbsparam[txq].idle_slope,
				ctxt->link_speed, queue, 0);
		}
#endif
		dwmac_set_cbs_idlesend(base_addr,
				       ctxt->cbsparam[txq].idle_slope,
				       link_speed, queue);
		break;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		ctxt->cbsparam[txq].bandwidth =
			config->qav_param.delta_bandwidth;
		ctxt->cbsparam[txq].idle_slope =
			PERCENT_TO_DECI(ctxt->cbsparam[txq].bandwidth,
					link_speed);
#ifdef CONFIG_ETH_DWC_EQOS_QBV
		/* Recalculate idle slope based on oper GCL */
		if (ctxt->estparam.enable) {
			dwmac_cbs_recal_idleslope(base_addr,
				&ctxt->cbsparam[txq].idle_slope,
				ctxt->link_speed, queue, 0);
		}
#endif
		dwmac_set_cbs_idlesend(base_addr,
				       ctxt->cbsparam[txq].idle_slope,
				       link_speed, queue);
		break;
	default:
		retval = -ENOTSUP;
	}

error:
	return retval;
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */

#ifdef CONFIG_ETH_DWC_EQOS_QBV
static int eth_set_est_param(const struct device *port,
			     enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	int ret = 0;
	const struct ethernet_qbv_param *est = &config->qbv_param;
	struct est_gc_entry gce = {0};

	switch (est->type) {
	case ETHERNET_QBV_PARAM_TYPE_STATUS:
		ctxt->estparam.enable = est->enabled;
		ret = dwmac_set_est_enable(base_addr, ctxt->estparam.enable);
#ifdef CONFIG_ETH_DWC_EQOS_QAV
		/* Recalculate idle slope based on bandwidth */
		for (int i = 1; i < ctxt->txqnum; i++) {
			if (ctxt->cbsparam[i - 1].enable &&
			    !ctxt->estparam.enable) {
				ctxt->cbsparam[i - 1].idle_slope =
					PERCENT_TO_DECI(
						ctxt->cbsparam[i - 1].bandwidth,
						ctxt->link_speed);
				dwmac_set_cbs_idlesend(base_addr,
					ctxt->cbsparam[i - 1].idle_slope,
					ctxt->link_speed, i);
			}
		}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
		break;
	case ETHERNET_QBV_PARAM_TYPE_GATE_CONTROL_LIST_LEN:
		if (est->gate_control_list_len > ctxt->estparam.gcl_depth) {
			return -EINVAL;
		}
		ctxt->estparam.gcrr.llr = est->gate_control_list_len;
		ret = dwmac_set_est_gcrr_llr(base_addr,
					     ctxt->estparam.gcrr.llr, 0, 0);
		break;
	case ETHERNET_QBV_PARAM_TYPE_GATE_CONTROL_LIST:
		for (int i = ctxt->txqnum; i < NET_TC_TX_COUNT; i++) {
			if (est->gate_control.gate_status[ctxt->txqnum - 1]
				!= est->gate_control.gate_status[i]) {
				return -EINVAL;
			}
		}
		for (int i = 0 ; i < ctxt->txqnum; i++) {
			gce.gates |= est->gate_control.gate_status[i] << i;
		}
		switch (est->gate_control.operation) {
		case ETHERNET_SET_GATE_STATE:
#ifdef CONFIG_ETH_DWC_EQOS_QBU
			if (ctxt->fpeparam.enable) {
				return -EINVAL;
			}
#endif
			break;
#ifdef CONFIG_ETH_DWC_EQOS_QBU
		case ETHERNET_SET_AND_HOLD_MAC_STATE:
			if (!ctxt->fpeparam.enable) {
				return -EINVAL;
			}
			gce.gates |= FPE_PMAC_BIT;
			break;
		case ETHERNET_SET_AND_RELEASE_MAC_STATE:
			if (!ctxt->fpeparam.enable) {
				return -EINVAL;
			}
			gce.gates &= ~FPE_PMAC_BIT;
			break;
#endif /* CONFIG_ETH_DWC_EQOS_QBU */
		default:
			return -EINVAL;
		}
		gce.ti_nsec = est->gate_control.time_interval;
		ret = dwmac_set_est_gce(base_addr, &gce, est->gate_control.row,
					0, 0);
		break;
	case ETHERNET_QBV_PARAM_TYPE_TIME:
		ctxt->estparam.gcrr.base_sec = est->base_time.second;
		ctxt->estparam.gcrr.base_nsec = est->base_time.fract_nsecond;
		ctxt->estparam.gcrr.cycle_sec = est->cycle_time.second;
		ctxt->estparam.gcrr.cycle_nsec = est->cycle_time.nanosecond;
		ctxt->estparam.gcrr.ter_nsec = est->extension_time;
		ret = dwmac_set_est_gcrr_times(base_addr, &ctxt->estparam.gcrr,
					       0, 0);
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */

#ifdef CONFIG_ETH_DWC_EQOS_QBU
static int eth_set_fpe_param(const struct device *port,
			     enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	const struct ethernet_qbu_param *fpe = &config->qbu_param;
	int ret = 0;
	int est_status = 0;
	struct fpe_config fpec;

	switch (fpe->type) {
	case ETHERNET_QBU_PARAM_TYPE_STATUS:
		if (ctxt->fpeparam.enable == fpe->enabled) {
			ret = -EALREADY;
			break;
		}
		ctxt->fpeparam.enable = fpe->enabled;
		ret = dwmac_set_fpe_enable(base_addr, ctxt->fpeparam.enable);
		if (ret == 0 && ctxt->fpeparam.enable) {
			ret = dwmac_fpe_send_mpacket(base_addr, MPACKET_VERIFY);
		}
		break;
	case ETHERNET_QBU_PARAM_TYPE_RELEASE_ADVANCE:
		ctxt->fpeparam.radv = fpe->release_advance;
		ret = dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_RADV,
					      &ctxt->fpeparam.radv);
		break;
	case ETHERNET_QBU_PARAM_TYPE_HOLD_ADVANCE:
		ctxt->fpeparam.hadv = fpe->hold_advance;
		ret = dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_HADV,
					      &ctxt->fpeparam.hadv);
		break;
	case ETHERNET_QBR_PARAM_TYPE_ADDITIONAL_FRAGMENT_SIZE:
		ctxt->fpeparam.afsz = fpe->additional_fragment_size;
		ret = dwmac_set_tsn_hwtunable(base_addr, TSN_HWTUNA_TX_FPE_AFSZ,
					      &ctxt->fpeparam.afsz);
		break;
	case ETHERNET_QBU_PARAM_TYPE_PREEMPTION_STATUS_TABLE:
		for (int i = ctxt->txqnum; i < NET_TC_TX_COUNT; i++) {
			if (fpe->frame_preempt_statuses[ctxt->txqnum - 1]
			    != fpe->frame_preempt_statuses[i]) {
				return -EINVAL;
			}
		}
		ctxt->fpeparam.fpst = 0;
		for (int i = 0 ; i < ctxt->txqnum; i++) {
			if (fpe->frame_preempt_statuses[i] ==
			    ETHERNET_QBU_STATUS_PREEMPTABLE) {
				ctxt->fpeparam.fpst |= BIT(i);
			} else if (fpe->frame_preempt_statuses[i] !=
				   ETHERNET_QBU_STATUS_EXPRESS) {
				return -EINVAL;
			}
		}
		fpec.txqpec = ctxt->fpeparam.fpst;
#ifdef CONFIG_ETH_DWC_EQOS_QBV
		if (ctxt->estparam.enable) {
			est_status = 1;
		}
#endif
		ret = dwmac_set_fpe_config(base_addr, &fpec, est_status);
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

#ifdef CONFIG_ETH_DWC_EQOS_TBS
static int eth_set_tbs_param(const struct device *port,
			     enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	const struct ethernet_txtime_param *tbs = &config->txtime_param;
	int ret = 0;

	switch (tbs->type) {
	case ETHERNET_TXTIME_PARAM_TYPE_ENABLE_QUEUES:
		if ((tbs->queue_id < ctxt->txqnum) &&
		    (ctxt->tbs_enabled[tbs->queue_id] != tbs->enable_txtime)) {
			eth_mac_stop_transaction(port);
			ctxt->tbs_enabled[tbs->queue_id] = tbs->enable_txtime;
			eth_dma_init(port);
			eth_mac_start_transaction(port);
		} else if (tbs->queue_id >= ctxt->txqnum) {
			ret = -ENOTSUP;
		}

		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

static int eth_set_config(const struct device *port, enum ethernet_config_type type,
			  const struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	struct ethernet_context *ctx = net_if_l2_data(ctxt->iface);
	struct phy_device *phy = &ctxt->phy_dev;
	int retval = 0;

	/* Ensure configuration only after initialization completed */
	if (!ctx->is_init) {
		return -ENETDOWN;
	}

	switch (type) {
	case ETHERNET_CONFIG_TYPE_AUTO_NEG:
		if (config->auto_negotiation) {
			ctxt->autoneg = true;
			/* Setting -1 to allow auto select the best
			 * negotiated speed and duplex mode
			 */
			retval = phy->cfg_link(phy, ctxt->autoneg, -1, -1,
					       true);
		} else {
			ctxt->autoneg = false;
			retval = phy->cfg_link(phy, ctxt->autoneg,
					       ctxt->link_speed,
					       ctxt->duplex_mode, true);
		}
		break;
	case ETHERNET_CONFIG_TYPE_LINK:
		if ((config->l.link_10bt && ctxt->link_speed == 10) ||
		    (config->l.link_100bt && ctxt->link_speed == 100) ||
		    (config->l.link_1000bt && ctxt->link_speed == 1000)) {
			return -EALREADY;
		}

		if (config->l.link_1000bt) {
			ctxt->link_speed = 1000;
		} else if (config->l.link_100bt) {
			ctxt->link_speed = 100;
		} else if (config->l.link_10bt) {
			ctxt->link_speed = 10;
		}
		retval = phy->cfg_link(phy, ctxt->autoneg, ctxt->link_speed,
				       ctxt->duplex_mode, true);
		break;
	case ETHERNET_CONFIG_TYPE_DUPLEX:
		if (config->full_duplex == ctxt->duplex_mode) {
			return -EALREADY;
		}

		ctxt->duplex_mode = config->full_duplex;
		retval = phy->cfg_link(phy, ctxt->autoneg, ctxt->link_speed,
				       ctxt->duplex_mode, true);
		break;
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(ctxt->mac_addr.bytes, config->mac_address.addr, 6);
		net_if_set_link_addr(ctxt->iface, ctxt->mac_addr.bytes,
				     sizeof(ctxt->mac_addr.bytes),
				     NET_LINK_ETHERNET);
		eth_mac_set_filter(ctxt, ctxt->mac_addr.bytes, 0, false, false);
		break;
#ifdef CONFIG_NET_PROMISCUOUS_MODE
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		retval = eth_mac_set_promiscuous(ctxt, config->promisc_mode);
		break;
#endif /* CONFIG_NET_PROMISCUOUS_MODE */
#if CONFIG_ETH_DWC_EQOS_QAV
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return eth_set_cbs_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
#if CONFIG_ETH_DWC_EQOS_QBV
	case ETHERNET_CONFIG_TYPE_QBV_PARAM:
		return eth_set_est_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QBV */
#if CONFIG_ETH_DWC_EQOS_QBU
	case ETHERNET_CONFIG_TYPE_QBU_PARAM:
		return eth_set_fpe_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QBU */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	case ETHERNET_CONFIG_TYPE_TXTIME_PARAM:
		return eth_set_tbs_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
	default:
		retval = -ENOTSUP;
		break;
	}

	return retval;
}

#if CONFIG_ETH_DWC_EQOS_QAV
static int eth_get_cbs_param(const struct device *port,
			     enum ethernet_config_type type,
			     struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	int queue = config->qav_param.queue_id;
	uint32_t base_addr = ctxt->base_addr;
	int link_speed = ctxt->link_speed;
	int retval = 0;

	/* Check if the queue number is supported */
	if (queue >= ctxt->txqnum || queue == 0) {
		retval = -ENOTSUP;
		goto error;
	}

	/* Check if the link speed is supported */
	if (link_speed != 100 && link_speed != 1000 &&
	    link_speed != 2500) {
		retval = -ENOTSUP;
		goto error;
	}

	switch (config->qav_param.type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		dwmac_get_cbs_status(base_addr, queue,
				     &config->qav_param.enabled);
		break;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		dwmac_get_cbs_idleband(base_addr,
				       &config->qav_param.idle_slope,
				       link_speed, queue, 1);
		break;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		config->qav_param.delta_bandwidth =
			ctxt->cbsparam[queue - 1].bandwidth;
		break;
	default:
		return -ENOTSUP;
	}

error:
	return retval;
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */

#ifdef CONFIG_ETH_DWC_EQOS_QBV
static int eth_get_est_param(const struct device *port,
			     enum ethernet_config_type type,
			     struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	int ret = 0;
	struct ethernet_qbv_param *est = &config->qbv_param;
	struct est_gc_config *gcc;
	struct est_gc_entry *gce;
	int bank;
	int own = 0;
	int row;

	if (est->state == ETHERNET_QBV_STATE_TYPE_ADMIN) {
		own = 1;
	} else if (est->state == ETHERNET_QBV_STATE_TYPE_OPER) {
		own = 0;
	}

	bank = dwmac_get_est_bank(base_addr, own);

	if (bank < 0) {
		ret = bank;
		return ret;
	}

	dwmac_get_est_gcc(base_addr, &gcc, 0);

	switch (est->type) {
	case ETHERNET_QBV_PARAM_TYPE_STATUS:
		est->enabled = gcc->enable;
		break;
	case ETHERNET_QBV_PARAM_TYPE_GATE_CONTROL_LIST_LEN:
		est->gate_control_list_len = gcc->gcb[bank].gcrr.llr;
		break;
	case ETHERNET_QBV_PARAM_TYPE_GATE_CONTROL_LIST:
		row = est->gate_control.row;
		gce = gcc->gcb[bank].gcl + row;

		if (row >= gcc->gcb[bank].gcrr.llr) {
			return -EINVAL;
		}

		for (int i = 0 ; i < ctxt->txqnum; i++) {
			if (gce->gates & BIT(i)) {
				est->gate_control.gate_status[i] = 1;
			} else {
				est->gate_control.gate_status[i] = 0;
			}
		}
		for (int i = ctxt->txqnum; i < NET_TC_TX_COUNT; i++) {
			est->gate_control.gate_status[i] =
				est->gate_control.gate_status[ctxt->txqnum - 1];
		}

		est->gate_control.time_interval = gce->ti_nsec;
		break;
	case ETHERNET_QBV_PARAM_TYPE_TIME:
		est->base_time.second = gcc->gcb[bank].gcrr.base_sec;
		est->base_time.fract_nsecond = gcc->gcb[bank].gcrr.base_nsec;
		est->cycle_time.second = gcc->gcb[bank].gcrr.cycle_sec;
		est->cycle_time.nanosecond = gcc->gcb[bank].gcrr.cycle_nsec;
		est->extension_time = gcc->gcb[bank].gcrr.ter_nsec;
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */

#ifdef CONFIG_ETH_DWC_EQOS_QBU
static int eth_get_fpe_param(const struct device *port,
			     enum ethernet_config_type type,
			     struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	struct ethernet_qbu_param *fpe = &config->qbu_param;
	int ret = 0;
	struct fpe_config *fpec;
	int afsz = 0;

	ret = dwmac_get_fpe_config(base_addr, &fpec, 0);

	switch (fpe->type) {
	case ETHERNET_QBU_PARAM_TYPE_STATUS:
		fpe->enabled = fpec->enable;
		break;
	case ETHERNET_QBU_PARAM_TYPE_RELEASE_ADVANCE:
		ret = dwmac_get_tsn_hwtunable(TSN_HWTUNA_TX_FPE_RADV,
					      &fpe->release_advance);
		break;
	case ETHERNET_QBU_PARAM_TYPE_HOLD_ADVANCE:
		ret = dwmac_get_tsn_hwtunable(TSN_HWTUNA_TX_FPE_HADV,
					      &fpe->hold_advance);
		break;
	case ETHERNET_QBR_PARAM_TYPE_ADDITIONAL_FRAGMENT_SIZE:
		ret = dwmac_get_tsn_hwtunable(TSN_HWTUNA_TX_FPE_AFSZ,
					      &afsz);
		fpe->additional_fragment_size = afsz;
		break;
	case ETHERNET_QBU_PARAM_TYPE_PREEMPTION_STATUS_TABLE:
		ctxt->fpeparam.fpst = fpec->txqpec;
		for (int i = 0 ; i < ctxt->txqnum; i++) {
			if (ctxt->fpeparam.fpst & BIT(i)) {
				fpe->frame_preempt_statuses[i] =
					ETHERNET_QBU_STATUS_PREEMPTABLE;
			} else {
				fpe->frame_preempt_statuses[i] =
					ETHERNET_QBU_STATUS_EXPRESS;
			}
		}
		for (int i = ctxt->txqnum; i < NET_TC_TX_COUNT; i++) {
			fpe->frame_preempt_statuses[i] =
				fpe->frame_preempt_statuses[ctxt->txqnum - 1];
		}
		break;
	case ETHERNET_QBR_PARAM_TYPE_LINK_PARTNER_STATUS:
		fpe->link_partner_status = fpec->lp_fpe_support;
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

#ifdef CONFIG_ETH_DWC_EQOS_TBS
static int eth_get_tbs_param(const struct device *port,
			     enum ethernet_config_type type,
			     struct ethernet_config *config)
{
	struct eth_runtime *ctxt = port->data;
	struct ethernet_txtime_param *tbs = &config->txtime_param;
	int ret = 0;

	switch (tbs->type) {
	case ETHERNET_TXTIME_PARAM_TYPE_ENABLE_QUEUES:
		if (tbs->queue_id < ctxt->txqnum) {
			tbs->enable_txtime = ctxt->tbs_enabled[tbs->queue_id];
		} else {
			tbs->enable_txtime = 0;
			ret = -ENOTSUP;
		}
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

static int eth_get_config(const struct device *port, enum ethernet_config_type type,
			  struct ethernet_config *config)
{
	switch (type) {
#if CONFIG_ETH_DWC_EQOS_QAV
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return eth_get_cbs_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
#if CONFIG_ETH_DWC_EQOS_QBV
	case ETHERNET_CONFIG_TYPE_QBV_PARAM:
		return eth_get_est_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QBV */
#if CONFIG_ETH_DWC_EQOS_QBU
	case ETHERNET_CONFIG_TYPE_QBU_PARAM:
		return eth_get_fpe_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_QBU */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	case ETHERNET_CONFIG_TYPE_TXTIME_PARAM:
		return eth_get_tbs_param(port, type, config);
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
	default:
		return -ENOTSUP;
	}
}

#ifdef CONFIG_NET_STATISTICS_ETHERNET
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
static void eth_get_stats_vendor(struct eth_runtime *context)
{
	struct net_stats_eth_vendor *v = context->stats.vendor;
	int i = 0, j, m;
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	struct tsn_err_stat *est_stat;
	int k;
#endif

	do {
		v[i].value = eth_read(context->base_addr, statreg[i].offset);
		i++;
	} while (statreg[i].key && statreg[i].offset);

	for (j = 0; j < CONFIG_ETH_DWC_EQOS_RX_QUEUES; j++, i++) {
		v[i].value = context->eth_stats.rx_pkt_n[j];
	}

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	dwmac_get_est_err_stat(&est_stat);
	v[i++].value = est_stat->cgce_n;
	v[i++].value = est_stat->hlbs_q;
	v[i++].value = est_stat->btre_n;
	v[i++].value = est_stat->btre_max_n;
	v[i++].value = est_stat->btrl;
	for (k = 0; k < CONFIG_ETH_DWC_EQOS_TX_QUEUES; k++, i++) {
		v[i].value = est_stat->hlbf_sz[k];
	}
#endif

	for (m = 0; m < ETH_ECC_MAX; m++, i++) {
		v[i].value = context->eth_stats.ecc_err_count[m];
	}
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */

static struct net_stats_eth *eth_get_stats(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	struct net_stats_eth *ret = &ctxt->stats;
	uint32_t base_addr = ctxt->base_addr;

	/* Update stats block */
	ret->bytes.sent = eth_read(base_addr, TX_OCTET_COUNT_GOOD);
	ret->bytes.received = eth_read(base_addr, RX_OCTET_COUNT_GOOD);

	/* Packet count */
	ret->pkts.tx = eth_read(base_addr, TX_PACKET_COUNT_GOOD);
	/* There is no corresponding "rx packet count good" register,
	 * getting the count value by adding from unicast, broadcast,
	 * multicast & control packets count register
	 */
	ret->pkts.rx = eth_read(base_addr, RX_UNICAST_PACKETS_GOOD);
	ret->pkts.rx += eth_read(base_addr, RX_BROADCAST_PACKETS_GOOD);
	ret->pkts.rx += eth_read(base_addr, RX_MULTICAST_PACKETS_GOOD);
	ret->pkts.rx += eth_read(base_addr, RX_CONTROL_PACKETS_GOOD);

	ret->broadcast.tx = eth_read(base_addr, TX_BROADCAST_PACKETS_GOOD);
	ret->broadcast.rx = eth_read(base_addr, RX_BROADCAST_PACKETS_GOOD);
	ret->multicast.tx = eth_read(base_addr, TX_MULTICAST_PACKETS_GOOD);
	ret->multicast.rx = eth_read(base_addr, RX_MULTICAST_PACKETS_GOOD);

	/* Vendor statistics */
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	eth_get_stats_vendor(ctxt);
#endif

	return ret;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

#ifdef CONFIG_NET_VLAN
static int eth_mac_vlan_setup(const struct device *dev, struct net_if *iface,
			      uint16_t tag, bool enable)
{
	struct ethernet_context *ethctx = net_if_l2_data(iface);
	struct eth_runtime *ctxt = dev->data;
	uint32_t base_addr = ctxt->base_addr;
	uint32_t reg_val;
	int retval, idx = -1;

#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	return eth_rxp_vlan(ctxt, enable, tag);
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */

	/* Look for VLAN interface index */
	for (int i = 0; i < CONFIG_NET_VLAN_COUNT; i++) {
		if (ethctx->vlan[i].iface == iface) {
			idx = i;
			break;
		}
	}

	/* Reject if index greater than available slot */
	if ((idx >= ctxt->vlancnt) || (idx < 0)) {
		LOG_ERR("HW VLAN entry index (%d) not in the range 0 to %d",
			idx, ctxt->vlancnt - 1);
		return -ENOENT;
	}

	LOG_DBG("%s VLAN tag (0x%04X) to entry index %d",
		enable ? "Register" : "Remove", tag, idx);
	if (enable) {
		reg_val = (tag & MAC_VLAN_TAG_FILT_VID_MASK) |
				MAC_VLAN_TAG_FILT_VEN | MAC_VLAN_TAG_FILT_ETV;
		eth_write(base_addr, MAC_VLAN_TAG_FILT, reg_val);
	} else {
		eth_write(base_addr, MAC_VLAN_TAG_FILT, 0);
	}

	/* Indirect access to VLAN entries */
	reg_val = eth_read(base_addr, MAC_VLAN_TAG_CTRL);
	reg_val &= ~MAC_VLAN_TAG_CTRL_OFS_MASK & MAC_VLAN_TAG_CTRL_CT_WR_INVMSK;
	reg_val |= (idx << MAC_VLAN_TAG_CTRL_OFS_SHIFT) | MAC_VLAN_TAG_CTRL_OB;
	eth_write(base_addr, MAC_VLAN_TAG_CTRL, reg_val);
	retval = dwc_busy_wait(base_addr, MAC_VLAN_TAG_CTRL,
			       MAC_VLAN_TAG_CTRL_OB, "vlan", 10, false);
	if (retval) {
		return retval;
	}

	return 0;
}
#endif /* CONFIG_NET_VLAN */

static const struct ethernet_api api_funcs = {
	.iface_api.init = eth_initialize,
	.send = eth_tx,
	.get_capabilities = eth_get_capabilities,
	.set_config = eth_set_config,
	.get_config = eth_get_config,
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	.get_stats = eth_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
#ifdef CONFIG_ETH_DWC_EQOS_PTP
	.get_ptp_clock = eth_get_ptp_clock,
#endif /* CONFIG_ETH_DWC_EQOS_PTP */
#ifdef CONFIG_NET_VLAN
	.vlan_setup = eth_mac_vlan_setup,
#endif /* CONFIG_NET_VLAN */
};

#ifdef CONFIG_ETH_DWC_EQOS_MSI
static void eth_dwc_eqos_msi(const struct device *dev, int vector)
{
	struct eth_runtime *context = dev->data;
	int queue_id;
	bool is_tx;

	if (vector == ETH_DWC_EQOS_0_MSI) {
		k_work_submit(&context->phy_work);
		return;
	}

	is_tx = vector & BIT(0);
	queue_id = vector >> 1;

	if (is_tx) {
		k_work_submit(&context->tx_irq_work[queue_id]);
	} else {
		k_work_schedule(&context->rx_irq_work[queue_id], K_MSEC(0));
	}
}

static void eth_doorbell(const void *arg)
{
	const struct eth_msi_param *param = arg;

	LOG_DBG("Interrupt received on vector %u dev %p", param->vector,
		param->dev);

	eth_dwc_eqos_msi(param->dev, param->vector);
}

static bool eth_configure_interrupts(const struct device *dev)
{
	struct eth_runtime *data = dev->data;
	bool ret = false;
	uint8_t n_vectors;
	uint32_t key;
	int i;

	key = irq_lock();

	n_vectors = pcie_msi_vectors_allocate(
					data->bdf,
					CONFIG_ETH_DWC_EQOS_MSI_INT_PRIORITY,
					data->vectors,
					ARRAY_SIZE(data->vectors));
	if (n_vectors == 0) {
		LOG_ERR("Could not allocate %lu MSI vectors",
			ARRAY_SIZE(data->vectors));
		goto out;
	}

	LOG_DBG("Allocated %u vectors", n_vectors);

	for (i = 0; i < n_vectors; i++) {
		data->params[i].dev = dev;
		data->params[i].vector = i;

		if (!pcie_msi_vector_connect(data->bdf,
					     &data->vectors[i],
					     eth_doorbell,
					     &data->params[i], 0)) {
			LOG_ERR("Failed to connect MSI vector %u", i);
			goto out;
		}
	}

	LOG_DBG("%u MSI Vectors connected", n_vectors);

	if (!pcie_msi_enable(data->bdf, data->vectors, n_vectors)) {
		LOG_ERR("Could not enable MSI");
		goto out;
	}

	data->n_vectors = n_vectors;
	ret = true;

	LOG_DBG("MSI configured");
out:
	irq_unlock(key);

	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_MSI */

static int eth_setup(const struct device *port, pcie_id_t pcie_id)
{
	struct eth_runtime *context = port->data;

	context->port = port;
	context->bdf = pcie_bdf_lookup(pcie_id);

	if (context->bdf == PCIE_BDF_NONE ||
	    !pcie_get_mbar(context->bdf, 0, &context->mbar)) {
		LOG_ERR("Cannot get mbar");
		return -ENOENT;
	}

	pcie_set_cmd(context->bdf, PCIE_CONF_CMDSTAT_MEM |
		     PCIE_CONF_CMDSTAT_MASTER, true);

	device_map(&context->base_addr, context->mbar.phys_addr,
		   context->mbar.size, K_MEM_CACHE_NONE);

#ifdef CONFIG_ETH_DWC_EQOS_MSI
	if (!eth_configure_interrupts(port)) {
		return -ENODEV;
	}
#endif
	context->flags |= ETH_DEV_FLAGS_PCI_DETECTED;

	return 0;
}

#ifdef CONFIG_ETH_DWC_EQOS_0
DEVICE_DECLARE(eth_dwc_eqos_0);

static void eth_config_0_irq(const struct device *port)
{
#ifdef CONFIG_ETH_DWC_EQOS_0_IRQ_DIRECT
#if defined(CONFIG_X86_64)
	setup_pcie(port, CONFIG_ETH_DWC_EQOS_0_IRQ_PRI,
		   (void (*)(const void *parameter))eth_dwc_eqos_isr,
		   DEVICE_GET(eth_dwc_eqos_0));
#else
	IRQ_CONNECT(ETH_DWC_EQOS_0_IRQ, CONFIG_ETH_DWC_EQOS_0_IRQ_PRI,
		    eth_dwc_eqos_isr, DEVICE_GET(eth_dwc_eqos_0), 0);
	irq_enable(ETH_DWC_EQOS_0_IRQ);
#endif
#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_DIRECT_IRQ(0, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_DIRECT_IRQ(0, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_DIRECT_IRQ(0, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_DIRECT_IRQ(0, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_DIRECT_IRQ(0, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_DIRECT_IRQ(0, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_DIRECT_IRQ(0, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_DIRECT_IRQ(0, 0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_DIRECT_IRQ(0, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_DIRECT_IRQ(0, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_DIRECT_IRQ(0, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_DIRECT_IRQ(0, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_DIRECT_IRQ(0, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_DIRECT_IRQ(0, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_DIRECT_IRQ(0, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_DIRECT_IRQ(0, 0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */
#elif defined(CONFIG_ETH_DWC_EQOS_0_IRQ_SHARED)
	const struct eth_config *config = port->config->config_info;
	const struct device *shared_irq_dev;

	shared_irq_dev = device_get_binding(config->sharedirq_devname);
	__ASSERT(shared_irq_dev != NULL,
		 "Failed to get eth_dwc_eqos device binding");
	shared_irq_isr_register(shared_irq_dev, (isr_t)eth_dwc_eqos_isr, port);
#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_SHARED_IRQ(0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_SHARED_IRQ(0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */
	shared_irq_enable(shared_irq_dev, port);
#endif /* CONFIG_ETH_DWC_EQOS_0_IRQ_DIRECT */
}

static const struct eth_config eth_config_0 = {
	.config_func            = eth_config_0_irq,

#ifdef CONFIG_ETH_DWC_EQOS_0_IRQ_SHARED
	.sharedirq_devname    = CONFIG_ETH_DWC_EQOS_0_IRQ_SHARED_NAME,
#endif
};

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
static struct net_stats_eth_vendor vendor0[ARRAY_SIZE(statreg)];
#endif

static struct eth_runtime eth_0_runtime = {
	.port_id                = ETH_PORT_0,
/* TODO: gbe type should not be hardcoded. */
	.pse_gbe                = true,
#ifdef CONFIG_ETH_DWC_EQOS_INTEL_PSE_PLATDATA
	.get_platdata            = &intel_pse_platdata,
#else
	.get_platdata            = NULL,
#endif
	.txqnum                 = CONFIG_ETH_DWC_EQOS_TX_QUEUES,
	.rxqnum                 = CONFIG_ETH_DWC_EQOS_RX_QUEUES,
#if defined(ETH_DWC_EQOS_MAX_TX_FIFOSZ) && defined(ETH_DWC_EQOS_MAX_RX_FIFOSZ)
	.txfifosz               = ETH_DWC_EQOS_MAX_TX_FIFOSZ,
	.rxfifosz               = ETH_DWC_EQOS_MAX_RX_FIFOSZ,
#endif
	.base_addr              = ETH_DWC_EQOS_0_BASE_ADDR,
#if defined(CONFIG_ETH_DWC_EQOS_0_IRQ_DIRECT) && defined(CONFIG_ARM)
	.irq_num                = ETH_DWC_EQOS_0_IRQ,
#endif
	.mdio_csr_clk           = ETH_DWC_EQOS_0_CSR_CLOCK_RANGE,
#ifdef CONFIG_NET_PKT_TIMESTAMP
	.ptp_clock_rate         = ETH_DWC_EQOS_0_PTP_CLOCK_RATE,
#endif /* CONFIG_NET_PKT_TIMESTAMP */
#ifdef CONFIG_ETH_DWC_EQOS_0_AUTONEG
	.autoneg                = true,
#else
	.autoneg                = false,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_0_10MHZ)
	.link_speed             = 10,
#elif defined(CONFIG_ETH_DWC_EQOS_0_100MHZ)
	.link_speed             = 100,
#elif defined(CONFIG_ETH_DWC_EQOS_0_1000MHZ)
	.link_speed             = 1000,
#elif defined(CONFIG_ETH_DWC_EQOS_0_2500MHZ)
	.link_speed             = 2500,
#else
	.link_speed             = -1,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_0_FULL_DUPLEX)
	.duplex_mode            = FULL_DUPLX,
#elif defined(CONFIG_ETH_DWC_EQOS_0_HALF_DUPLEX)
	.duplex_mode            = HALF_DUPLX,
#else
	.duplex_mode            = -1,
#endif
#ifdef ETH_DWC_EQOS_0_USE_XPCS
	.use_xpcs               = ETH_DWC_EQOS_0_USE_XPCS,
#else
	.use_xpcs               = 0,
#endif
#if defined CONFIG_ETH_DWC_EQOS_0_EEE
	.use_eee                = true,
	.lpi_timer              = CONFIG_LPI_ENTRY_TIMER_0,
#else
	.use_eee                = false,
	.lpi_timer              = 0,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_0_PCI) && !defined(CONFIG_PCIE)
	.pci_dev.class_type     = ETH_DWC_EQOS_PCI_CLASS,
	.pci_dev.bus            = ETH_DWC_EQOS_0_PCI_BUS,
	.pci_dev.dev            = ETH_DWC_EQOS_0_PCI_DEV,
	.pci_dev.vendor_id      = ETH_DWC_EQOS_0_PCI_VENDOR_ID,
	.pci_dev.device_id      = ETH_DWC_EQOS_0_PCI_DEVICE_ID,
	.pci_dev.function       = ETH_DWC_EQOS_0_PCI_FUNCTION,
	.pci_dev.bar            = ETH_DWC_EQOS_0_PCI_BAR,
#endif
	.phy_dev.addr           = ETH_DWC_EQOS_0_PHY_ADDR,
	.phy_dev.interface      = ETH_DWC_EQOS_0_PHY_IFACE,
#if defined(CONFIG_ETH_DWC_EQOS_0_PHY_88E1512)
	.phy_dev.init           = &marvell_88e1512_init,
	.phy_dev.cfg_link       = &marvell_88e1512_config_link,
	.phy_dev.read_status    = &marvell_88e1512_read_status,
	.phy_dev.en_intr        = &marvell_88e1512_enable_interrupt,
	.phy_dev.intr_status    = &marvell_88e1512_interrupt_status,
	.phy_dev.get_tx_latency = &marvell_88e1512_get_tx_latency,
	.phy_dev.get_rx_latency = &marvell_88e1512_get_rx_latency,
#elif CONFIG_ETH_DWC_EQOS_0_GEN_PHY
	.phy_dev.init           = &gen_phy_init,
	.phy_dev.cfg_link       = &gen_phy_config_link,
	.phy_dev.read_status    = &gen_phy_read_status,
	.phy_dev.en_intr        = &gen_phy_dummy_function,
	.phy_dev.intr_status    = &gen_phy_link_status,
	.phy_dev.get_tx_latency = &gen_phy_get_tx_latency,
	.phy_dev.get_rx_latency = &gen_phy_get_rx_latency,
#endif
	.phy_dev.mdio_read      = &dwc_mdio_read,
	.phy_dev.mdio_write     = &dwc_mdio_send,
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	.stats.vendor = vendor0,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ7_BANDWIDTH
	.cbsparam[6].enable = 1,
	.cbsparam[6].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ7_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ7_IDLESLOPE
	.cbsparam[6].enable = 1,
	.cbsparam[6].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ7_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ6_BANDWIDTH
	.cbsparam[5].enable = 1,
	.cbsparam[5].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ6_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ6_IDLESLOPE
	.cbsparam[5].enable = 1,
	.cbsparam[5].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ6_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ5_BANDWIDTH
	.cbsparam[4].enable = 1,
	.cbsparam[4].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ5_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ5_IDLESLOPE
	.cbsparam[4].enable = 1,
	.cbsparam[4].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ5_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ4_BANDWIDTH
	.cbsparam[3].enable = 1,
	.cbsparam[3].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ4_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ4_IDLESLOPE
	.cbsparam[3].enable = 1,
	.cbsparam[3].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ4_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ3_BANDWIDTH
	.cbsparam[2].enable = 1,
	.cbsparam[2].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ3_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ3_IDLESLOPE
	.cbsparam[2].enable = 1,
	.cbsparam[2].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ3_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ2_BANDWIDTH
	.cbsparam[1].enable = 1,
	.cbsparam[1].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ2_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ2_IDLESLOPE
	.cbsparam[1].enable = 1,
	.cbsparam[1].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ2_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ1_BANDWIDTH
	.cbsparam[0].enable = 1,
	.cbsparam[0].bandwidth = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ1_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QAV_TXQ1_IDLESLOPE
	.cbsparam[0].enable = 1,
	.cbsparam[0].idle_slope = CONFIG_ETH_DWC_EQOS_0_QAV_TXQ1_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV
	.estparam.gcl_depth = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_DEPTH,
	.estparam.tils = CONFIG_ETH_DWC_EQOS_0_QBV_TILS,
	.estparam.ptov = CONFIG_ETH_DWC_EQOS_0_QBV_PTOV,
	.estparam.ctov = CONFIG_ETH_DWC_EQOS_0_QBV_CTOV,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_ENABLE
	.estparam.enable = 1,
	.estparam.gcrr.base_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_BTR,
	.estparam.gcrr.cycle_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_CTR_NS,
	.estparam.gcrr.cycle_sec = CONFIG_ETH_DWC_EQOS_0_QBV_CTR_S,
	.estparam.gcrr.ter_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_TER,
	.estparam.gcrr.llr = CONFIG_ETH_DWC_EQOS_0_QBV_LLR,
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 1)
	.estparam.gce[0].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R0_GS,
	.estparam.gce[0].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R0_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R0_S
	.estparam.mode[0] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R0_R
	.estparam.mode[0] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R0_H
	.estparam.mode[0] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 1 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 2)
	.estparam.gce[1].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R1_GS,
	.estparam.gce[1].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R1_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R1_S
	.estparam.mode[1] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R1_R
	.estparam.mode[1] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R1_H
	.estparam.mode[1] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 2 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 3)
	.estparam.gce[2].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R2_GS,
	.estparam.gce[2].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R2_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R2_S
	.estparam.mode[2] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R2_R
	.estparam.mode[2] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R2_H
	.estparam.mode[2] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 3 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 4)
	.estparam.gce[3].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R3_GS,
	.estparam.gce[3].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R3_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R3_S
	.estparam.mode[3] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R3_R
	.estparam.mode[3] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R3_H
	.estparam.mode[3] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 4 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 5)
	.estparam.gce[4].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R4_GS,
	.estparam.gce[4].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R4_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R4_S
	.estparam.mode[4] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R4_R
	.estparam.mode[4] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R4_H
	.estparam.mode[4] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 5 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 6)
	.estparam.gce[5].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R5_GS,
	.estparam.gce[5].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R5_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R5_S
	.estparam.mode[5] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R5_R
	.estparam.mode[5] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R5_H
	.estparam.mode[5] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 6 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 7)
	.estparam.gce[6].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R6_GS,
	.estparam.gce[6].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R6_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R6_S
	.estparam.mode[6] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R6_R
	.estparam.mode[6] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R6_H
	.estparam.mode[6] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 7 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 8)
	.estparam.gce[7].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R7_GS,
	.estparam.gce[7].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R7_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R7_S
	.estparam.mode[7] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R7_R
	.estparam.mode[7] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R7_H
	.estparam.mode[7] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 8 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 9)
	.estparam.gce[8].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R8_GS,
	.estparam.gce[8].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R8_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R8_S
	.estparam.mode[8] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R8_R
	.estparam.mode[8] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R8_H
	.estparam.mode[8] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 9 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 10)
	.estparam.gce[9].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R9_GS,
	.estparam.gce[9].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R9_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R9_S
	.estparam.mode[9] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R9_R
	.estparam.mode[9] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R9_H
	.estparam.mode[9] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 10 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 11)
	.estparam.gce[10].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R10_GS,
	.estparam.gce[10].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R10_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R10_S
	.estparam.mode[10] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R10_R
	.estparam.mode[10] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R10_H
	.estparam.mode[10] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 11 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 12)
	.estparam.gce[11].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R11_GS,
	.estparam.gce[11].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R11_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R11_S
	.estparam.mode[11] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R11_R
	.estparam.mode[11] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R11_H
	.estparam.mode[11] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 12 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 13)
	.estparam.gce[12].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R12_GS,
	.estparam.gce[12].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R12_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R12_S
	.estparam.mode[12] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R12_R
	.estparam.mode[12] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R12_H
	.estparam.mode[12] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 13 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 14)
	.estparam.gce[13].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R13_GS,
	.estparam.gce[13].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R13_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R13_S
	.estparam.mode[13] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R13_R
	.estparam.mode[13] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R13_H
	.estparam.mode[13] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 14 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 15)
	.estparam.gce[14].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R14_GS,
	.estparam.gce[14].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R14_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R14_S
	.estparam.mode[14] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R14_R
	.estparam.mode[14] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R14_H
	.estparam.mode[14] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 15 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 16)
	.estparam.gce[15].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R15_GS,
	.estparam.gce[15].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R15_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R15_S
	.estparam.mode[15] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R15_R
	.estparam.mode[15] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R15_H
	.estparam.mode[15] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 16 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 17)
	.estparam.gce[16].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R16_GS,
	.estparam.gce[16].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R16_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R16_S
	.estparam.mode[16] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R16_R
	.estparam.mode[16] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R16_H
	.estparam.mode[16] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 17 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 18)
	.estparam.gce[17].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R17_GS,
	.estparam.gce[17].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R17_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R17_S
	.estparam.mode[17] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R17_R
	.estparam.mode[17] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R17_H
	.estparam.mode[17] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 18 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 19)
	.estparam.gce[18].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R18_GS,
	.estparam.gce[18].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R18_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R18_S
	.estparam.mode[18] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R18_R
	.estparam.mode[18] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R18_H
	.estparam.mode[18] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 19 */
#if (CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 20)
	.estparam.gce[19].gates = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R19_GS,
	.estparam.gce[19].ti_nsec = CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R19_TI,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R19_S
	.estparam.mode[19] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R19_R
	.estparam.mode[19] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_QBV_GCL_R19_H
	.estparam.mode[19] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_LLR >= 20 */
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV_ENABLE */
#endif /* CONFIG_ETH_DWC_EQOS_0_QBV */
#ifdef CONFIG_ETH_DWC_EQOS_0_QBU
	.fpeparam.fpst = CONFIG_ETH_DWC_EQOS_0_QBU_ST,
	.fpeparam.hadv = CONFIG_ETH_DWC_EQOS_0_QBU_HA,
	.fpeparam.radv = CONFIG_ETH_DWC_EQOS_0_QBU_RA,
	.fpeparam.afsz = CONFIG_ETH_DWC_EQOS_0_QBU_AFSZ,
	.fpeparam.fprq = CONFIG_ETH_DWC_EQOS_0_QBU_FPRQ,
#ifdef CONFIG_ETH_DWC_EQOS_0_QBU_ENABLE
	.fpeparam.enable = 1,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_0_QBU */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ0_ENABLE
	.tbs_enabled[0] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ0_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ1_ENABLE
	.tbs_enabled[1] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ1_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ2_ENABLE
	.tbs_enabled[2] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ2_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ3_ENABLE
	.tbs_enabled[3] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ3_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ4_ENABLE
	.tbs_enabled[4] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ4_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ5_ENABLE
	.tbs_enabled[5] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ5_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ6_ENABLE
	.tbs_enabled[6] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ6_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_0_TBS_TXQ7_ENABLE
	.tbs_enabled[7] = CONFIG_ETH_DWC_EQOS_0_TBS_TXQ7_ENABLE,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
};

#ifdef CONFIG_ETH_DWC_EQOS_0_PCI
static int eth_0_setup(const struct device *dev)
{
	return eth_setup(dev,
			 PCIE_ID(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_EQOS_0));
}
#else
static int eth_0_setup(const struct device *dev)
{
	return 0;
}
#endif /* CONFIG_ETH_DWC_EQOS_0_PCI */

ETH_NET_DEVICE_INIT(eth_dwc_eqos_0, CONFIG_ETH_DWC_EQOS_0_NAME, eth_0_setup,
		    NULL, &eth_0_runtime, &eth_config_0,
		    CONFIG_ETH_INIT_PRIORITY, &api_funcs, ETH_DWC_EQOS_MTU);
#endif  /* CONFIG_ETH_DWC_EQOS_0 */

#ifdef CONFIG_ETH_DWC_EQOS_1
DEVICE_DECLARE(eth_dwc_eqos_1);

static void eth_config_1_irq(const struct device *port)
{
#ifdef CONFIG_ETH_DWC_EQOS_1_IRQ_DIRECT
#if defined(CONFIG_X86_64)
	setup_pcie(port, CONFIG_ETH_DWC_EQOS_1_IRQ_PRI,
		   (void (*)(const void *parameter))eth_dwc_eqos_isr,
		   DEVICE_GET(eth_dwc_eqos_1));
#else
	IRQ_CONNECT(ETH_DWC_EQOS_1_IRQ, CONFIG_ETH_DWC_EQOS_1_IRQ_PRI,
		    eth_dwc_eqos_isr, DEVICE_GET(eth_dwc_eqos_1), 0);
	irq_enable(ETH_DWC_EQOS_1_IRQ);
#endif
#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_DIRECT_IRQ(1, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_DIRECT_IRQ(1, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_DIRECT_IRQ(1, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_DIRECT_IRQ(1, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_DIRECT_IRQ(1, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_DIRECT_IRQ(1, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_DIRECT_IRQ(1, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_DIRECT_IRQ(1, 0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_DIRECT_IRQ(1, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_DIRECT_IRQ(1, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_DIRECT_IRQ(1, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_DIRECT_IRQ(1, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_DIRECT_IRQ(1, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_DIRECT_IRQ(1, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_DIRECT_IRQ(1, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_DIRECT_IRQ(1, 0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */
#elif defined(CONFIG_ETH_DWC_EQOS_1_IRQ_SHARED)
	const struct eth_config *config = port->config->config_info;
	const struct device *shared_irq_dev;

	shared_irq_dev = device_get_binding(config->sharedirq_devname);
	__ASSERT(shared_irq_dev != NULL,
		 "Failed to get eth_dwc_eqos device binding");
	shared_irq_isr_register(shared_irq_dev, (isr_t)eth_dwc_eqos_isr, port);
#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_SHARED_IRQ(0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_SHARED_IRQ(0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */
	shared_irq_enable(shared_irq_dev, port);
#endif /* CONFIG_ETH_DWC_EQOS_1_IRQ_DIRECT */
}

static const struct eth_config eth_config_1 = {
	.config_func            = eth_config_1_irq,

#ifdef CONFIG_ETH_DWC_EQOS_1_IRQ_SHARED
	.sharedirq_devname    = CONFIG_ETH_DWC_EQOS_1_IRQ_SHARED_NAME,
#endif
};

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
static struct net_stats_eth_vendor vendor1[ARRAY_SIZE(statreg)];
#endif

static struct eth_runtime eth_1_runtime = {
	.port_id                = ETH_PORT_1,
	.pse_gbe                = true,
#ifdef CONFIG_ETH_DWC_EQOS_INTEL_PSE_PLATDATA
	.get_platdata            = &intel_pse_platdata,
#else
	.get_platdata            = NULL,
#endif
	.txqnum                 = CONFIG_ETH_DWC_EQOS_TX_QUEUES,
	.rxqnum                 = CONFIG_ETH_DWC_EQOS_RX_QUEUES,
#if defined(ETH_DWC_EQOS_MAX_TX_FIFOSZ) && defined(ETH_DWC_EQOS_MAX_RX_FIFOSZ)
	.txfifosz               = ETH_DWC_EQOS_MAX_TX_FIFOSZ,
	.rxfifosz               = ETH_DWC_EQOS_MAX_RX_FIFOSZ,
#endif
	.base_addr              = ETH_DWC_EQOS_1_BASE_ADDR,
#if defined(CONFIG_ETH_DWC_EQOS_1_IRQ_DIRECT) && defined(CONFIG_ARM)
	.irq_num                = ETH_DWC_EQOS_1_IRQ,
#endif
	.mdio_csr_clk           = ETH_DWC_EQOS_1_CSR_CLOCK_RANGE,
#ifdef CONFIG_NET_PKT_TIMESTAMP
	.ptp_clock_rate         = ETH_DWC_EQOS_1_PTP_CLOCK_RATE,
#endif /* CONFIG_NET_PKT_TIMESTAMP */
#ifdef CONFIG_ETH_DWC_EQOS_1_AUTONEG
	.autoneg                = true,
#else
	.autoneg                = false,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_1_10MHZ)
	.link_speed             = 10,
#elif defined(CONFIG_ETH_DWC_EQOS_1_100MHZ)
	.link_speed             = 100,
#elif defined(CONFIG_ETH_DWC_EQOS_1_1000MHZ)
	.link_speed             = 1000,
#elif defined(CONFIG_ETH_DWC_EQOS_1_2500MHZ)
	.link_speed             = 2500,
#else
	.link_speed             = -1,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_1_FULL_DUPLEX)
	.duplex_mode            = FULL_DUPLX,
#elif defined(CONFIG_ETH_DWC_EQOS_1_HALF_DUPLEX)
	.duplex_mode            = HALF_DUPLX,
#else
	.duplex_mode            = -1,
#endif
#ifdef ETH_DWC_EQOS_1_USE_XPCS
	.use_xpcs               = ETH_DWC_EQOS_1_USE_XPCS,
#else
	.use_xpcs               = 0,
#endif
#if defined CONFIG_ETH_DWC_EQOS_1_EEE
	.use_eee                = true,
	.lpi_timer              = CONFIG_LPI_ENTRY_TIMER_1,
#else
	.use_eee                = false,
	.lpi_timer              = 0,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_1_PCI) && !defined(CONFIG_PCIE)
	.pci_dev.class_type     = ETH_DWC_EQOS_PCI_CLASS,
	.pci_dev.bus            = ETH_DWC_EQOS_1_PCI_BUS,
	.pci_dev.dev            = ETH_DWC_EQOS_1_PCI_DEV,
	.pci_dev.vendor_id      = ETH_DWC_EQOS_1_PCI_VENDOR_ID,
	.pci_dev.device_id      = ETH_DWC_EQOS_1_PCI_DEVICE_ID,
	.pci_dev.function       = ETH_DWC_EQOS_1_PCI_FUNCTION,
	.pci_dev.bar            = ETH_DWC_EQOS_1_PCI_BAR,
#endif
	.phy_dev.addr           = ETH_DWC_EQOS_1_PHY_ADDR,
	.phy_dev.interface      = ETH_DWC_EQOS_1_PHY_IFACE,
#if defined(CONFIG_ETH_DWC_EQOS_1_PHY_88E1512)
	.phy_dev.init           = &marvell_88e1512_init,
	.phy_dev.cfg_link       = &marvell_88e1512_config_link,
	.phy_dev.read_status    = &marvell_88e1512_read_status,
	.phy_dev.en_intr        = &marvell_88e1512_enable_interrupt,
	.phy_dev.intr_status    = &marvell_88e1512_interrupt_status,
	.phy_dev.get_tx_latency = &marvell_88e1512_get_tx_latency,
	.phy_dev.get_rx_latency = &marvell_88e1512_get_rx_latency,
#elif CONFIG_ETH_DWC_EQOS_1_GEN_PHY
	.phy_dev.init           = &gen_phy_init,
	.phy_dev.cfg_link       = &gen_phy_config_link,
	.phy_dev.read_status    = &gen_phy_read_status,
	.phy_dev.en_intr        = &gen_phy_dummy_function,
	.phy_dev.intr_status    = &gen_phy_link_status,
	.phy_dev.get_tx_latency = &gen_phy_get_tx_latency,
	.phy_dev.get_rx_latency = &gen_phy_get_rx_latency,
#endif
	.phy_dev.mdio_read      = &dwc_mdio_read,
	.phy_dev.mdio_write     = &dwc_mdio_send,
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	.stats.vendor = vendor1,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ7_BANDWIDTH
	.cbsparam[6].enable = 1,
	.cbsparam[6].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ7_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ7_IDLESLOPE
	.cbsparam[6].enable = 1,
	.cbsparam[6].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ7_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ6_BANDWIDTH
	.cbsparam[5].enable = 1,
	.cbsparam[5].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ6_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ6_IDLESLOPE
	.cbsparam[5].enable = 1,
	.cbsparam[5].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ6_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ5_BANDWIDTH
	.cbsparam[4].enable = 1,
	.cbsparam[4].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ5_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ5_IDLESLOPE
	.cbsparam[4].enable = 1,
	.cbsparam[4].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ5_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ4_BANDWIDTH
	.cbsparam[3].enable = 1,
	.cbsparam[3].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ4_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ4_IDLESLOPE
	.cbsparam[3].enable = 1,
	.cbsparam[3].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ4_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ3_BANDWIDTH
	.cbsparam[2].enable = 1,
	.cbsparam[2].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ3_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ3_IDLESLOPE
	.cbsparam[2].enable = 1,
	.cbsparam[2].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ3_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ2_BANDWIDTH
	.cbsparam[1].enable = 1,
	.cbsparam[1].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ2_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ2_IDLESLOPE
	.cbsparam[1].enable = 1,
	.cbsparam[1].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ2_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ1_BANDWIDTH
	.cbsparam[0].enable = 1,
	.cbsparam[0].bandwidth = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ1_BANDWIDTH,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QAV_TXQ1_IDLESLOPE
	.cbsparam[0].enable = 1,
	.cbsparam[0].idle_slope = CONFIG_ETH_DWC_EQOS_1_QAV_TXQ1_IDLESLOPE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV
	.estparam.gcl_depth = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_DEPTH,
	.estparam.tils = CONFIG_ETH_DWC_EQOS_1_QBV_TILS,
	.estparam.ptov = CONFIG_ETH_DWC_EQOS_1_QBV_PTOV,
	.estparam.ctov = CONFIG_ETH_DWC_EQOS_1_QBV_CTOV,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_ENABLE
	.estparam.enable = 1,
	.estparam.gcrr.base_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_BTR,
	.estparam.gcrr.cycle_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_CTR_NS,
	.estparam.gcrr.cycle_sec = CONFIG_ETH_DWC_EQOS_1_QBV_CTR_S,
	.estparam.gcrr.ter_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_TER,
	.estparam.gcrr.llr = CONFIG_ETH_DWC_EQOS_1_QBV_LLR,
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 1)
	.estparam.gce[0].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R0_GS,
	.estparam.gce[0].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R0_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R0_S
	.estparam.mode[0] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R0_R
	.estparam.mode[0] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R0_H
	.estparam.mode[0] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 1 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 2)
	.estparam.gce[1].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R1_GS,
	.estparam.gce[1].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R1_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R1_S
	.estparam.mode[1] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R1_R
	.estparam.mode[1] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R1_H
	.estparam.mode[1] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 2 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 3)
	.estparam.gce[2].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R2_GS,
	.estparam.gce[2].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R2_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R2_S
	.estparam.mode[2] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R2_R
	.estparam.mode[2] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R2_H
	.estparam.mode[2] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 3 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 4)
	.estparam.gce[3].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R3_GS,
	.estparam.gce[3].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R3_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R3_S
	.estparam.mode[3] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R3_R
	.estparam.mode[3] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R3_H
	.estparam.mode[3] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 4 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 5)
	.estparam.gce[4].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R4_GS,
	.estparam.gce[4].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R4_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R4_S
	.estparam.mode[4] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R4_R
	.estparam.mode[4] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R4_H
	.estparam.mode[4] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 5 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 6)
	.estparam.gce[5].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R5_GS,
	.estparam.gce[5].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R5_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R5_S
	.estparam.mode[5] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R5_R
	.estparam.mode[5] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R5_H
	.estparam.mode[5] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 6 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 7)
	.estparam.gce[6].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R6_GS,
	.estparam.gce[6].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R6_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R6_S
	.estparam.mode[6] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R6_R
	.estparam.mode[6] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R6_H
	.estparam.mode[6] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 7 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 8)
	.estparam.gce[7].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R7_GS,
	.estparam.gce[7].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R7_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R7_S
	.estparam.mode[7] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R7_R
	.estparam.mode[7] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R7_H
	.estparam.mode[7] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 8 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 9)
	.estparam.gce[8].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R8_GS,
	.estparam.gce[8].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R8_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R8_S
	.estparam.mode[8] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R8_R
	.estparam.mode[8] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R8_H
	.estparam.mode[8] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 9 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 10)
	.estparam.gce[9].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R9_GS,
	.estparam.gce[9].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R9_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R9_S
	.estparam.mode[9] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R9_R
	.estparam.mode[9] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R9_H
	.estparam.mode[9] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 10 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 11)
	.estparam.gce[10].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R10_GS,
	.estparam.gce[10].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R10_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R10_S
	.estparam.mode[10] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R10_R
	.estparam.mode[10] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R10_H
	.estparam.mode[10] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 11 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 12)
	.estparam.gce[11].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R11_GS,
	.estparam.gce[11].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R11_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R11_S
	.estparam.mode[11] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R11_R
	.estparam.mode[11] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R11_H
	.estparam.mode[11] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 12 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 13)
	.estparam.gce[12].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R12_GS,
	.estparam.gce[12].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R12_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R12_S
	.estparam.mode[12] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R12_R
	.estparam.mode[12] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R12_H
	.estparam.mode[12] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 13 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 14)
	.estparam.gce[13].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R13_GS,
	.estparam.gce[13].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R13_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R13_S
	.estparam.mode[13] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R13_R
	.estparam.mode[13] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R13_H
	.estparam.mode[13] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 14 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 15)
	.estparam.gce[14].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R14_GS,
	.estparam.gce[14].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R14_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R14_S
	.estparam.mode[14] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R14_R
	.estparam.mode[14] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R14_H
	.estparam.mode[14] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 15 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 16)
	.estparam.gce[15].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R15_GS,
	.estparam.gce[15].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R15_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R15_S
	.estparam.mode[15] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R15_R
	.estparam.mode[15] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R15_H
	.estparam.mode[15] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 16 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 17)
	.estparam.gce[16].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R16_GS,
	.estparam.gce[16].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R16_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R16_S
	.estparam.mode[16] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R16_R
	.estparam.mode[16] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R16_H
	.estparam.mode[16] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 17 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 18)
	.estparam.gce[17].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R17_GS,
	.estparam.gce[17].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R17_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R17_S
	.estparam.mode[17] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R17_R
	.estparam.mode[17] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R17_H
	.estparam.mode[17] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 18 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 19)
	.estparam.gce[18].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R18_GS,
	.estparam.gce[18].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R18_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R18_S
	.estparam.mode[18] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R18_R
	.estparam.mode[18] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R18_H
	.estparam.mode[18] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 19 */
#if (CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 20)
	.estparam.gce[19].gates = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R19_GS,
	.estparam.gce[19].ti_nsec = CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R19_TI,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R19_S
	.estparam.mode[19] = GCL_MODE_SET,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R19_R
	.estparam.mode[19] = GCL_MODE_RELEASE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_QBV_GCL_R19_H
	.estparam.mode[19] = GCL_MODE_HOLD,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_LLR >= 20 */
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV_ENABLE */
#endif /* CONFIG_ETH_DWC_EQOS_1_QBV */
#ifdef CONFIG_ETH_DWC_EQOS_1_QBU
	.fpeparam.fpst = CONFIG_ETH_DWC_EQOS_1_QBU_ST,
	.fpeparam.hadv = CONFIG_ETH_DWC_EQOS_1_QBU_HA,
	.fpeparam.radv = CONFIG_ETH_DWC_EQOS_1_QBU_RA,
	.fpeparam.afsz = CONFIG_ETH_DWC_EQOS_1_QBU_AFSZ,
	.fpeparam.fprq = CONFIG_ETH_DWC_EQOS_1_QBU_FPRQ,
#ifdef CONFIG_ETH_DWC_EQOS_1_QBU_ENABLE
	.fpeparam.enable = 1,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_1_QBU */
#ifdef CONFIG_ETH_DWC_EQOS_TBS
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ0_ENABLE
	.tbs_enabled[0] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ0_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ1_ENABLE
	.tbs_enabled[1] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ1_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ2_ENABLE
	.tbs_enabled[2] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ2_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ3_ENABLE
	.tbs_enabled[3] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ3_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ4_ENABLE
	.tbs_enabled[4] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ4_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ5_ENABLE
	.tbs_enabled[5] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ5_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ6_ENABLE
	.tbs_enabled[6] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ6_ENABLE,
#endif
#ifdef CONFIG_ETH_DWC_EQOS_1_TBS_TXQ7_ENABLE
	.tbs_enabled[7] = CONFIG_ETH_DWC_EQOS_1_TBS_TXQ7_ENABLE,
#endif
#endif /* CONFIG_ETH_DWC_EQOS_TBS */
};

#ifdef CONFIG_ETH_DWC_EQOS_1_PCI
static int eth_1_setup(const struct device *dev)
{
	return eth_setup(dev,
			 PCIE_ID(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_EQOS_1));
}
#else
static int eth_1_setup(const struct device *dev)
{
	return 0;
}
#endif /* CONFIG_ETH_DWC_EQOS_1_PCI */

ETH_NET_DEVICE_INIT(eth_dwc_eqos_1, CONFIG_ETH_DWC_EQOS_1_NAME, eth_1_setup,
		    NULL, &eth_1_runtime, &eth_config_1,
		    CONFIG_ETH_INIT_PRIORITY, &api_funcs, ETH_DWC_EQOS_MTU);
#endif  /* CONFIG_ETH_DWC_EQOS_1 */

#ifdef CONFIG_ETH_DWC_EQOS_2
DEVICE_DECLARE(eth_dwc_eqos_2);

static void eth_config_2_irq(const struct device *port)
{
#ifdef CONFIG_ETH_DWC_EQOS_2_IRQ_DIRECT
#if defined(CONFIG_X86_64)
	setup_pcie(port, CONFIG_ETH_DWC_EQOS_2_IRQ_PRI,
		   (void (*)(const void *parameter))eth_dwc_eqos_isr,
		   DEVICE_GET(eth_dwc_eqos_2));
#else
	IRQ_CONNECT(ETH_DWC_EQOS_2_IRQ, CONFIG_ETH_DWC_EQOS_2_IRQ_PRI,
		    eth_dwc_eqos_isr, DEVICE_GET(eth_dwc_eqos_2), 0);
	irq_enable(ETH_DWC_EQOS_2_IRQ);
#endif

#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_DIRECT_IRQ(2, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_DIRECT_IRQ(2, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_DIRECT_IRQ(2, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_DIRECT_IRQ(2, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_DIRECT_IRQ(2, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_DIRECT_IRQ(2, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_DIRECT_IRQ(2, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_DIRECT_IRQ(2, 0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_DIRECT_IRQ(2, 7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_DIRECT_IRQ(2, 6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_DIRECT_IRQ(2, 5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_DIRECT_IRQ(2, 4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_DIRECT_IRQ(2, 3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_DIRECT_IRQ(2, 2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_DIRECT_IRQ(2, 1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_DIRECT_IRQ(2, 0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */

#elif defined(CONFIG_ETH_DWC_EQOS_2_IRQ_SHARED)
	const struct eth_config *config = port->config->config_info;
	const struct device *shared_irq_dev;

	shared_irq_dev = device_get_binding(config->sharedirq_devname);
	__ASSERT(shared_irq_dev != NULL,
		 "Failed to get eth_dwc_eqos device binding");
	shared_irq_isr_register(shared_irq_dev, (isr_t)eth_dwc_eqos_isr, port);
#if defined(ETH_DWC_EQOS_MULTI_IRQ)
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES == 8)
	SETUP_TXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 7)
	SETUP_TXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 6)
	SETUP_TXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 5)
	SETUP_TXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 4)
	SETUP_TXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 3)
	SETUP_TXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 2)
	SETUP_TXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES >= 1)
	SETUP_TXQ_SHARED_IRQ(0);
#endif

#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES == 8)
	SETUP_RXQ_SHARED_IRQ(7);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 7)
	SETUP_RXQ_SHARED_IRQ(6);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 6)
	SETUP_RXQ_SHARED_IRQ(5);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 5)
	SETUP_RXQ_SHARED_IRQ(4);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 4)
	SETUP_RXQ_SHARED_IRQ(3);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 3)
	SETUP_RXQ_SHARED_IRQ(2);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 2)
	SETUP_RXQ_SHARED_IRQ(1);
#endif
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES >= 1)
	SETUP_RXQ_SHARED_IRQ(0);
#endif
#endif /* ETH_DWC_EQOS_MULTI_IRQ */
	shared_irq_enable(shared_irq_dev, port);
#endif /* CONFIG_ETH_DWC_EQOS_2_IRQ_DIRECT */
}

static const struct eth_config eth_config_2 = {
	.config_func            = eth_config_2_irq,
};

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
static struct net_stats_eth_vendor vendor2[ARRAY_SIZE(statreg)];
#endif

static struct eth_runtime eth_2_runtime = {
	.port_id                = ETH_PORT_2,
	.pse_gbe		= false,
	.txqnum                 = CONFIG_ETH_DWC_EQOS_TX_QUEUES,
	.rxqnum                 = CONFIG_ETH_DWC_EQOS_RX_QUEUES,
#if defined(ETH_DWC_EQOS_MAX_TX_FIFOSZ) && defined(ETH_DWC_EQOS_MAX_RX_FIFOSZ)
	.txfifosz               = ETH_DWC_EQOS_MAX_TX_FIFOSZ,
	.rxfifosz               = ETH_DWC_EQOS_MAX_RX_FIFOSZ,
#endif
	.mdio_csr_clk           = ETH_DWC_EQOS_2_CSR_CLOCK_RANGE,
#ifdef CONFIG_NET_PKT_TIMESTAMP
	.ptp_clock_rate         = ETH_DWC_EQOS_2_PTP_CLOCK_RATE,
#endif /* CONFIG_NET_PKT_TIMESTAMP */
#ifdef CONFIG_ETH_DWC_EQOS_2_AUTONEG
	.autoneg                = true,
#else
	.autoneg                = false,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_2_10MHZ)
	.link_speed             = 10,
#elif defined(CONFIG_ETH_DWC_EQOS_2_100MHZ)
	.link_speed             = 100,
#elif defined(CONFIG_ETH_DWC_EQOS_2_1000MHZ)
	.link_speed             = 1000,
#elif defined(CONFIG_ETH_DWC_EQOS_2_2500MHZ)
	.link_speed             = 2500,
#else
	.link_speed             = -1,
#endif
#if defined(CONFIG_ETH_DWC_EQOS_2_FULL_DUPLEX)
	.duplex_mode            = FULL_DUPLX,
#elif defined(CONFIG_ETH_DWC_EQOS_2_HALF_DUPLEX)
	.duplex_mode            = HALF_DUPLX,
#else
	.duplex_mode            = -1,
#endif
	.use_xpcs               = 1,
	.phy_dev.addr           = ETH_DWC_EQOS_2_PHY_ADDR,
	.phy_dev.interface      = ETH_DWC_EQOS_2_PHY_IFACE,
#if defined(CONFIG_ETH_DWC_EQOS_2_PHY_88E1512)
	.phy_dev.init           = &marvell_88e1512_init,
	.phy_dev.cfg_link       = &marvell_88e1512_config_link,
	.phy_dev.read_status    = &marvell_88e1512_read_status,
	.phy_dev.en_intr        = &marvell_88e1512_enable_interrupt,
	.phy_dev.intr_status    = &marvell_88e1512_interrupt_status,
	.phy_dev.get_tx_latency = &marvell_88e1512_get_tx_latency,
	.phy_dev.get_rx_latency = &marvell_88e1512_get_rx_latency,
#elif CONFIG_ETH_DWC_EQOS_2_GEN_PHY
	.phy_dev.init           = &gen_phy_init,
	.phy_dev.cfg_link       = &gen_phy_config_link,
	.phy_dev.read_status    = &gen_phy_read_status,
	.phy_dev.en_intr        = &gen_phy_dummy_function,
	.phy_dev.intr_status    = &gen_phy_link_status,
	.phy_dev.get_tx_latency = &gen_phy_get_tx_latency,
	.phy_dev.get_rx_latency = &gen_phy_get_rx_latency,
#endif
	.phy_dev.mdio_read      = &dwc_mdio_read,
	.phy_dev.mdio_write     = &dwc_mdio_send,
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	.stats.vendor = vendor2,
#endif
};

#ifdef CONFIG_ETH_DWC_EQOS_2_PCI
static int eth_2_setup(const struct device *dev)
{
	return eth_setup(dev, PCIE_ID(PCI_VENDOR_ID_INTEL,
			 PCI_DEVICE_ID_EQOS_EHL_PCH));
}
#else
static int eth_2_setup(const struct device *dev)
{
	return 0;
}
#endif /* CONFIG_ETH_DWC_EQOS_2_PCI */

ETH_NET_DEVICE_INIT(eth_dwc_eqos_2, CONFIG_ETH_DWC_EQOS_2_NAME, eth_2_setup,
		    NULL, &eth_2_runtime, &eth_config_2,
		    CONFIG_ETH_INIT_PRIORITY, &api_funcs, ETH_DWC_EQOS_MTU);
#endif  /* CONFIG_ETH_DWC_EQOS_2 */

#ifdef CONFIG_ETH_DWC_EQOS_PTP
struct ptp_context {
	struct eth_runtime *eth_context;
};

static int dwc_eqos_ptp_clock_set(const struct device *dev,
				  struct net_ptp_time *tm)
{
	struct ptp_context *ptp_context = dev->data;
	struct eth_runtime *context = ptp_context->eth_context;

	eth_mac_set_time(context, tm->second, tm->nanosecond);

	return 0;
}

static int dwc_eqos_ptp_clock_get(const struct device *dev,
				  struct net_ptp_time *tm)
{
	struct ptp_context *ptp_context = dev->data;
	struct eth_runtime *context = ptp_context->eth_context;
	mm_reg_t base_addr = context->base_addr;

	tm->nanosecond = eth_read(base_addr, MAC_SYS_TIME_NANOSEC);
	tm->second = eth_read(base_addr, MAC_SYS_TIME_SEC);

	return 0;
}

static int dwc_eqos_ptp_clock_adjust(const struct device *dev, int increment)
{
	struct ptp_context *ptp_context = dev->data;
	struct eth_runtime *context = ptp_context->eth_context;
	mm_reg_t base_addr = context->base_addr;
	uint32_t sec;
	uint32_t nsec;
	uint32_t value;
	int neg_adj = ((increment < 0) ? 1 : 0);
	int limit = 10;
	int ret;

	if ((increment <= -(int)NSEC_PER_SEC) ||
	    (increment >= (int)NSEC_PER_SEC)) {
		ret = -EINVAL;
	} else {
		sec = increment / NSEC_PER_SEC;
		nsec = increment % NSEC_PER_SEC;

		if (neg_adj) {
			nsec = NSEC_PER_SEC - nsec;
			nsec |= MAC_SYS_TIME_NANOSEC_UPD_ADDSUB;
		}

		eth_write(base_addr, MAC_SYS_TIME_SEC_UPD, sec);
		eth_write(base_addr, MAC_SYS_TIME_NANOSEC_UPD, nsec);

		value = eth_read(base_addr, MAC_TIMESTAMP_CTRL);
		value |= MAC_TIMESTAMP_CTRL_TSUPDT;
		eth_write(base_addr, MAC_TIMESTAMP_CTRL, value);

		/* wait for present system time initialize to complete */
		while (limit--) {
			if (!(eth_read(base_addr, MAC_TIMESTAMP_CTRL) &
			    MAC_TIMESTAMP_CTRL_TSUPDT)) {
				break;
			}

			k_sleep(K_MSEC(10));
		}

		if (limit < 0) {
			ret = -EBUSY;
		}

		ret = 0;
	}
	return ret;
}

static int dwc_eqos_ptp_rate_adjust(const struct device *dev, float ratio)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ratio);
	/* TODO: Adjust the Addend register accordingly when neighbor rate ratio
	 * does not jump.
	 */
	return -ENOTSUP;
}

static const struct ptp_clock_driver_api ptp_api = {
	.set = dwc_eqos_ptp_clock_set,
	.get = dwc_eqos_ptp_clock_get,
	.adjust = dwc_eqos_ptp_clock_adjust,
	.rate_adjust = dwc_eqos_ptp_rate_adjust,
};

#endif /* CONFIG_ETH_DWC_EQOS_PTP */

#if defined(CONFIG_ETH_DWC_EQOS_0_PTP)
static struct ptp_context ptp_context_0;

static int dwc_eqos_ptp_init_0(const struct device *port)
{
	const struct device *eth_dev = DEVICE_GET(eth_dwc_eqos_0);
	struct eth_runtime *context = eth_dev->data;
	struct ptp_context *ptp_context = port->data;

	context->ptp_clock = port;
	ptp_context->eth_context = context;

	return 0;
}

DEVICE_DEFINE(ptp_clock_0, PTP_CLOCK_NAME, dwc_eqos_ptp_init_0, NULL,
	      &ptp_context_0, NULL, POST_KERNEL,
	      CONFIG_APPLICATION_INIT_PRIORITY, &ptp_api);

#endif /* CONFIG_ETH_DWC_EQOS_0_PTP */

#if defined(CONFIG_ETH_DWC_EQOS_1_PTP)
static struct ptp_context ptp_context_1;

static int dwc_eqos_ptp_init_1(const struct device *port)
{
	const struct device *eth_dev = DEVICE_GET(eth_dwc_eqos_1);
	struct eth_runtime *context = eth_dev->data;
	struct ptp_context *ptp_context = port->data;

	context->ptp_clock = port;
	ptp_context->eth_context = context;

	return 0;
}

DEVICE_DEFINE(ptp_clock_1, PTP_CLOCK_NAME, dwc_eqos_ptp_init_1, NULL,
	      &ptp_context_1, NULL, POST_KERNEL,
	      CONFIG_APPLICATION_INIT_PRIORITY, &ptp_api);

#endif /* CONFIG_ETH_DWC_EQOS_1_PTP */

#if defined(CONFIG_ETH_DWC_EQOS_2_PTP)
static struct ptp_context ptp_context_2;

static int dwc_eqos_ptp_init_2(const struct device *port)
{
	const struct device *eth_dev = DEVICE_GET(eth_dwc_eqos_2);
	struct eth_runtime *context = eth_dev->data;
	struct ptp_context *ptp_context = port->data;

	context->ptp_clock = port;
	ptp_context->eth_context = context;

	return 0;
}

DEVICE_DEFINE(ptp_clock_2, PTP_CLOCK_NAME, dwc_eqos_ptp_init_2, NULL,
	      &ptp_context_2, NULL, POST_KERNEL,
	      CONFIG_APPLICATION_INIT_PRIORITY, &ptp_api);

#endif /* CONFIG_ETH_DWC_EQOS_2_PTP */
