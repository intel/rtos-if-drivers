/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW_TSN_LIB_H__
#define __DW_TSN_LIB_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TSN Related MAC Registers
 */
#define MAC_HW_FEAT_FPESEL			BIT(26)
#define MAC_HW_FEAT_ESTWID			0x00300000
#define MAC_HW_FEAT_ESTWID_SHIFT		20
#define MAC_HW_FEAT_ESTDEP			0x000e0000
#define MAC_HW_FEAT_ESTDEP_SHIFT		17
#define MAC_HW_FEAT_ESTSEL			BIT(16)

#define MAC_RXQ_CTRL1			0x000000a4
#define MAC_RXQCTRL_FPRQ_MASK		0x07000000
#define MAC_RXQCTRL_FPRQ_SHIFT		24

#define MAC_FPE_CTRL_STS		0x00000234
#define MAC_FPE_CTRL_STS_TRSP		BIT(19)
#define MAC_FPE_CTRL_STS_TVER		BIT(18)
#define MAC_FPE_CTRL_STS_RRSP		BIT(17)
#define MAC_FPE_CTRL_STS_RVER		BIT(16)
#define MAC_FPE_CTRL_STS_SRSP		BIT(2)
#define MAC_FPE_CTRL_STS_SVER		BIT(1)
#define MAC_FPE_CTRL_STS_EFPE		BIT(0)

/*
 * TSN Related MTL Registers
 */
#define MTL_EST_CTRL				0x00000c50
#define MTL_EST_CTRL_PTOV			0xff000000
#define MTL_EST_CTRL_PTOV_SHIFT			24
#define MTL_EST_CTRL_CTOV			0x00fff000
#define MTL_EST_CTRL_CTOV_SHIFT			12
#define MTL_EST_CTRL_TILS			0x00000700
#define MTL_EST_CTRL_TILS_SHIFT			8
#define MTL_EST_CTRL_SSWL			BIT(1)
#define MTL_EST_CTRL_EEST			BIT(0)

#define MTL_EST_STATUS				0x00000c58
#define MTL_EST_STATUS_BTRL			0x00000f00
#define MTL_EST_STATUS_BTRL_SHIFT		8
#define MTL_EST_STATUS_BTRL_MAX			(0xF << 8)
#define MTL_EST_STATUS_SWOL			BIT(7)
#define MTL_EST_STATUS_SWOL_SHIFT		7
#define MTL_EST_STATUS_CGCE			BIT(4)
#define MTL_EST_STATUS_HLBS			BIT(3)
#define MTL_EST_STATUS_HLBF			BIT(2)
#define MTL_EST_STATUS_BTRE			BIT(1)
#define MTL_EST_STATUS_SWLC			BIT(0)

#define MTL_EST_SCH_ERR				0x00000c60
#define MTL_EST_FRM_SZ_ERR			0x00000c64
#define MTL_EST_FRM_SZ_CAP			0x00000c68
#define MTL_EST_FRM_SZ_CAP_HBFS_MASK		0x00007fff
#define MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT		16
#define MTL_EST_FRM_SZ_CAP_HBFQ_MASK(x)		(x > 4 ? 0x00070000 : \
						 x > 2 ? 0x00030000 : \
						 BIT(16))

#define MTL_EST_INT_EN				0x00000c70
#define MTL_EST_INT_EN_CGCE			BIT(4)
#define MTL_EST_INT_EN_IEHS			BIT(3)
#define MTL_EST_INT_EN_IEHF			BIT(2)
#define MTL_EST_INT_EN_IEBE			BIT(1)
#define MTL_EST_INT_EN_IECC			BIT(0)

#define MTL_EST_GCL_CTRL			0x00000c80
#define MTL_EST_GCL_CTRL_ADDR			0x00000700
#define MTL_EST_GCL_CTRL_ADDR_VAL(addr)		(addr << 8)
#define GCL_CTRL_ADDR_BTR_LO			0x0
#define GCL_CTRL_ADDR_BTR_HI			0x1
#define GCL_CTRL_ADDR_CTR_LO			0x2
#define GCL_CTRL_ADDR_CTR_HI			0x3
#define GCL_CTRL_ADDR_TER			0x4
#define GCL_CTRL_ADDR_LLR			0x5
#define MTL_EST_GCL_CTRL_DBGB1			BIT(5)
#define MTL_EST_GCL_CTRL_DBGM			BIT(4)
#define MTL_EST_GCL_CTRL_GCRR			BIT(2)
#define MTL_EST_GCL_CTRL_R1W0			BIT(1)
#define GCL_OPS_R				BIT(1)
#define GCL_OPS_W				0
#define MTL_EST_GCL_CTRL_SRWO			BIT(0)

#define MTL_EST_GCL_DATA			0x00000c84

#define MTL_FPE_CTRL_STS		0x00000c90
#define MTL_FPE_CTRL_STS_HRS		BIT(28)
#define MTL_FPE_CTRL_STS_HRS_SHIFT	28
#define MTL_FPE_CTRL_STS_PEC		0x0000ff00
#define MTL_FPE_CTRL_STS_PEC_SHIFT	8
#define MTL_FPE_CTRL_STS_AFSZ		0x00000003

#define MTL_FPE_ADVANCE			0x00000c94
#define MTL_FPE_ADVANCE_RADV		0xffff0000
#define MTL_FPE_ADVANCE_RADV_SHIFT	16
#define MTL_FPE_ADVANCE_HADV		0x0000ffff

#define MTL_TXQ_ETS_CONTROL(x)			(0x0D10 + (x * 0x40))
#define MTL_TXQ_ETS_CONTROL_AVALG		BIT(2)
#define MTL_TXQ_QUANTUM_WEIGHT(x)		(0x0D18 + (x * 0x40))
#define MTL_TXQ_SENDSLOPECREDIT(x)		(0x0D1C + (x * 0x40))
#define MTL_TXQ_HICREDIT(x)			(0x0D20 + (x * 0x40))
#define MTL_TXQ_LOCREDIT(x)			(0x0D24 + (x * 0x40))

/*
 * CBS Related Global defines
 */
#define CBS_HICREDIT				0x08000000 /* 16384 bytes */
#define CBS_LOCREDIT				0x18000000 /* -16384 bytes */
#define PERCENT_TO_DECI(x, y)			(((x) * (y)) / 100)
#define DECI_TO_PERCENT(x, y)			(((x) * 100) / (y))
#define MBPS_TO_BPC_100(x)			((x) * 1024 * 4 / 100)
#define MBPS_TO_BPC_1000(x)			((x) * 1024 * 8 / 1000)
#define MBPS_TO_BPC_2500(x)			((x) * 1024 * 32 / 10000)
#define BPC_TO_MBPS_100(x)			((x) * 100 / (1024 * 4))
#define BPC_TO_MBPS_1000(x)			((x) * 1000 / (1024 * 8))
#define BPC_TO_MBPS_2500(x)			((x) * 10000 / (1024 * 32))
#define ROUDING_OFFSET_100			20
#define ROUDING_OFFSET_1000			4
#define ROUDING_OFFSET_2500			1
#define MBPS_TO_BPS(x)				((x) * 1000000)
#define BPS_TO_MBPS(x)				((x) / 1000000)

/*
 * EST Related Global Defines
 */
#define EST_CTR_HI_MAX				0xff	/* 8 bits */
#define EST_PTOV_MAX				0xff	/* 8 bits */
#define EST_CTOV_MAX				0xfff	/* 12 bits */
#define EST_GCL_BANK_MAX			(2) /* Oper & Admin */

/*
 * FPE Related Global Defines
 */
#define FPE_AFSZ_MAX			0x3	/* 2 bits */
#define FPE_ADV_MAX			0xffff	/* 16 bits */
#define FPE_PMAC_BIT			0x01	/* default preemtable TxQ */

/*
 * DWC EQoS MAC IP Version
 */
#define TSN_CORE_VER				0x50

/*
 * Register Read/Write
 */
#define TSN_RD32(__addr)			sys_read32(__addr)
#define TSN_WR32(__val, __addr)			sys_write32(__val, __addr)

/*
 * System Logging
 */
#define TSN_INFO_NA(__msg)	LOG_INF(__msg)
#define TSN_WARN_NA(__msg)	LOG_WRN(__msg)
#define TSN_ERR_NA(__msg)	LOG_ERR(__msg)
#define TSN_INFO(__msg, __arg0, __args...) \
	LOG_INF(__msg, (__arg0), ##__args)
#define TSN_WARN(__msg, __arg0, __args...) \
	LOG_WRN(__msg, (__arg0), ##__args)
#define TSN_ERR(__msg, __arg0, __args...) \
	LOG_ERR(__msg, (__arg0), ##__args)

enum tsn_hwtunable_id {
	TSN_HWTUNA_TX_EST_TILS = 0,
	TSN_HWTUNA_TX_EST_PTOV,
	TSN_HWTUNA_TX_EST_CTOV,
	TSN_HWTUNA_TX_FPE_AFSZ,
	TSN_HWTUNA_TX_FPE_HADV,
	TSN_HWTUNA_TX_FPE_RADV,
	TSN_HWTUNA_MAX,
};

enum tsn_feat_id {
	TSN_FEAT_ID_EST = 0,
	TSN_FEAT_ID_FPE,
	TSN_FEAT_ID_MAX,
};

/*
 * EST Operation Mode
 */
enum gcl_mode_id {
	GCL_MODE_SET = 0,
	GCL_MODE_HOLD,
	GCL_MODE_RELEASE,
};

enum tsn_fpe_irq_state {
	FPE_STATE_TRSP = 1,
	FPE_STATE_TVER = 2,
	FPE_STATE_RRSP = 4,
	FPE_STATE_RVER = 8,
	FPE_STATE_UNKNOWN = 16,
};

enum mpacket_type {
	MPACKET_VERIFY = 0,
	MPACKET_RESPONSE = 1,
};

struct tsn_hw_cap {
	bool est_support;
	bool fpe_support;
	unsigned int txqcnt;		/* Number of Tx Queue */
	unsigned int rxqcnt;		/* Number of Rx Queue */
	unsigned int gcl_depth;		/* Depth of GCL */
	unsigned int ti_wid;		/* width of time interval */
	unsigned int tils_max;		/* Max time interval left shift */
	unsigned int ext_max;		/* Max extension time */
};

struct est_gc_entry {
	unsigned int gates;		/* gate status (open/close)*/
	unsigned int ti_nsec;		/* time interval (nsec) */
};

/*
 * TSN Error Status
 */
struct tsn_err_stat {
	/* Constant gate control error */
	unsigned int cgce_n;
	/* Queue with Head-Of-Line Blocking due to Scheduling */
	unsigned int hlbs_q;
	/* Frame sizes that causes Head-Of-Line Blocking */
	unsigned int hlbf_sz[8];
	/* BTR error */
	unsigned int btre_n;
	/* BTR error with BTR renewal fail */
	unsigned int btre_max_n;
	/* BTR error loop */
	unsigned int btrl;
};

struct est_gcrr {
	unsigned int base_nsec;		/* base time (nsec) */
	unsigned int base_sec;		/* base time (sec) */
	unsigned int cycle_nsec;	/* cycle time (nsec) */
	unsigned int cycle_sec;		/* cycle time (sec)*/
	unsigned int ter_nsec;		/* extension time (nsec) */
	unsigned int llr;		/* gate control list length */
};

/*
 * Gate Control Bank
 */
struct est_gc_bank {
	struct est_gc_entry *gcl;
	struct est_gcrr gcrr;
};

/*
 * Gate Control Configuration
 */
struct est_gc_config {
	struct est_gc_bank gcb[EST_GCL_BANK_MAX];
	bool enable;
};

struct fpe_config {
	unsigned int txqpec;		/* TxQ preemption classification */
	bool enable;			/* Qbu enable */
	bool lp_fpe_support;		/* link partner status */
};

/*
 * CBS Related Parameters
 */
struct cbs_param {
	bool enable;				/* Qav enable */
	uint32_t bandwidth;			/* bandwidth (%) */
	uint32_t idle_slope;			/* idle slope (Mbps) */
};

/*
 * EST Related Parameters
 */
struct est_param {
	bool enable;			/* Qbv enable */
	unsigned int gcl_depth;		/* max number of row in gcl */
	unsigned int tils;		/* time interval left shift */
	unsigned int ptov;		/* PTP time offset value (nsec)*/
	unsigned int ctov;		/* current time offset value (nsec)*/
	struct est_gcrr gcrr;		/* gate control related register */
	struct est_gc_entry gce[20];	/* gate control entry */
	unsigned int mode[20];		/* operation mode (S/H/R) */
};

/*
 * FPE Related Parameters
 */
struct fpe_param {
	bool enable;
	unsigned int fpst;		/* frame preemption status table */
	unsigned int hadv;		/* hold advance */
	unsigned int radv;		/* release advance */
	unsigned int afsz;		/* additional fragment size */
	unsigned int fprq;		/* frame preemption residue queue */
};

/*
 * TSN Functions
 */
#ifdef CONFIG_ETH_DWC_EQOS_QAV
void dwmac_set_cbs_hilocredit(uint32_t base_addr, int queue);
void dwmac_set_cbs_idlesend(uint32_t base_addr, uint32_t idleslope,
			    int32_t linkspeed, int queue);
void dwmac_set_cbs_status(uint32_t base_addr, int queue, bool enable);
void dwmac_get_cbs_status(uint32_t base_addr, int queue, bool *enable);
void dwmac_get_cbs_idleband(uint32_t base_addr, uint32_t *idleband,
			    int32_t linkspeed, int queue, bool idle);
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
#if defined(CONFIG_ETH_DWC_EQOS_QBV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
void dwmac_tsn_init(const struct device *port);
void dwmac_get_tsn_hwcap(struct tsn_hw_cap **tsn_hwcap);
void dwmac_set_tsn_feat(enum tsn_feat_id featid, bool enable);
int dwmac_set_tsn_hwtunable(uint32_t base_addr, enum tsn_hwtunable_id id,
			    const unsigned int *data);
int dwmac_get_tsn_hwtunable(enum tsn_hwtunable_id id, unsigned int *data);
#endif /* CONFIG_ETH_DWC_EQOS_QBV || CONFIG_ETH_DWC_EQOS_QBU */
#ifdef CONFIG_ETH_DWC_EQOS_QBV
void dwmac_set_est_intr(uint32_t base_addr);
void dwmac_set_est_gcb(struct est_gc_entry *gcl, unsigned int bank);
int dwmac_get_est_bank(uint32_t base_addr, unsigned int own);
int dwmac_set_est_gce(uint32_t base_addr,
		      struct est_gc_entry *gce, unsigned int row,
		      unsigned int dbgb, unsigned int dbgm);
int dwmac_get_est_gcrr_llr(uint32_t base_addr, unsigned int *gcl_len,
			   unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_gcrr_llr(uint32_t base_addr, unsigned int gcl_len,
			   unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_gcrr_times(uint32_t base_addr,
			     struct est_gcrr *gcrr,
			     unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_enable(uint32_t base_addr, bool enable);
int dwmac_get_est_gcc(uint32_t base_addr,
		      struct est_gc_config **gcc, bool frmdrv);
int dwmac_est_irq_status(const struct device *port);
int dwmac_get_est_err_stat(struct tsn_err_stat **err_stat);
int dwmac_clr_est_err_stat(uint32_t base_addr);
#endif /* CONFIG_ETH_DWC_EQOS_QBV */
#if defined(CONFIG_ETH_DWC_EQOS_QBV) && defined(CONFIG_ETH_DWC_EQOS_QAV)
int dwmac_cbs_recal_idleslope(uint32_t base_addr, unsigned int *idle_slope,
			      int32_t linkspeed, unsigned int queue, bool swol);
#endif /* CONFIG_ETH_DWC_EQOS_QBV && CONFIG_ETH_DWC_EQOS_QAV */
#ifdef CONFIG_ETH_DWC_EQOS_QBU
void dwmac_set_fpe_fprq(uint32_t base_addr, unsigned int fprq);
int dwmac_set_fpe_config(uint32_t base_addr, struct fpe_config *fpec,
			 uint32_t est_status);
int dwmac_set_fpe_enable(uint32_t base_addr, bool enable);
int dwmac_get_fpe_config(uint32_t base_addr, struct fpe_config **fpec,
			 bool frmdrv);
int dwmac_get_fpe_pmac_sts(uint32_t base_addr, unsigned int *hrs);
int dwmac_fpe_irq_status(uint32_t base_addr);
int dwmac_fpe_send_mpacket(uint32_t base_addr, enum mpacket_type type);
void dwmac_fpe_reset_lp_status(void);
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

#ifdef __cplusplus
}
#endif

#endif /* __DW_TSN_LIB_H__ */
