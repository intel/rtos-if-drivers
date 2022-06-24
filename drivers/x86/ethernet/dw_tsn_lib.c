/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME eth_dw_tsn
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <net/ethernet.h>
#include <drivers/ethernet/phy_ehl.h>

#include "eth_dwc_eqos_v5_priv.h"
#include "dw_tsn_lib.h"

#if defined(CONFIG_ETH_DWC_EQOS_QBV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
static struct tsn_hw_cap dw_tsn_hwcap;
static bool dw_tsn_feat_en[TSN_FEAT_ID_MAX];
static unsigned int dw_tsn_hwtunable[TSN_HWTUNA_MAX];
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBU
static struct fpe_config dw_fpe_config;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBV
static struct est_gc_config dw_est_gc_config;
static struct tsn_err_stat dw_err_stat;

#define ONE_SEC_IN_NANOSEC 1000000000ULL

static unsigned int est_get_gcl_depth(unsigned int hw_cap)
{
	unsigned int depth;
	unsigned int estdep = (hw_cap & MAC_HW_FEAT_ESTDEP) >>
			      MAC_HW_FEAT_ESTDEP_SHIFT;

	switch (estdep) {
	case 1:
		depth = 64;
		break;
	case 2:
		depth = 128;
		break;
	case 3:
		depth = 256;
		break;
	case 4:
		depth = 512;
		break;
	case 5:
		depth = 1024;
		break;
	default:
		depth = 0;
	}

	return depth;
}

static unsigned int est_get_ti_width(unsigned int hw_cap)
{
	unsigned int width;
	unsigned int estwid = (hw_cap & MAC_HW_FEAT_ESTWID) >>
			      MAC_HW_FEAT_ESTWID_SHIFT;

	switch (estwid) {
	case 1:
		width = 16;
		break;
	case 2:
		width = 20;
		break;
	case 3:
		width = 24;
		break;
	default:
		width = 0;
	}

	return width;
}

static int est_poll_srwo(uint32_t base_addr)
{
	/* Poll until the EST GCL Control[SRWO] bit clears.
	 * Total wait = 12 x 50ms ~= 0.6s.
	 */
	unsigned int retries = 12;
	unsigned int value;

	do {
		value = TSN_RD32(base_addr + MTL_EST_GCL_CTRL);
		if (!(value & MTL_EST_GCL_CTRL_SRWO)) {
			return 0;
		}
		k_sleep(K_MSEC(50));
	} while (--retries);

	return -ETIMEDOUT;
}

static int est_set_gcl_addr(uint32_t base_addr, unsigned int addr,
			    unsigned int gcrr, unsigned int rwops,
			    unsigned int dbgb, unsigned int dbgm)
{
	unsigned int value;

	value = MTL_EST_GCL_CTRL_ADDR_VAL(addr) & MTL_EST_GCL_CTRL_ADDR;

	if (dbgm) {
		if (dbgb) {
			value |= MTL_EST_GCL_CTRL_DBGB1;
		}
		value |= MTL_EST_GCL_CTRL_DBGM;
	}

	if (gcrr) {
		value |= MTL_EST_GCL_CTRL_GCRR;
	}

	/* This is the only place SRWO is set and driver polls SRWO
	 * for self-cleared before exit. Therefore, caller should
	 * check return status for possible time out error.
	 */
	value |= (rwops | MTL_EST_GCL_CTRL_SRWO);

	TSN_WR32(value, base_addr + MTL_EST_GCL_CTRL);

	return est_poll_srwo(base_addr);
}

static int est_write_gcl_config(uint32_t base_addr, unsigned int data,
				unsigned int addr, unsigned int gcrr,
				unsigned int dbgb, unsigned int dbgm)
{
	TSN_WR32(data, base_addr + MTL_EST_GCL_DATA);

	return est_set_gcl_addr(base_addr, addr, gcrr, GCL_OPS_W, dbgb, dbgm);
}

static int est_read_gcl_config(uint32_t base_addr, unsigned int *data,
			       unsigned int addr, unsigned int gcrr,
			       unsigned int dbgb, unsigned int dbgm)
{
	int ret;

	ret = est_set_gcl_addr(base_addr, addr, gcrr, GCL_OPS_R, dbgb, dbgm);
	if (ret) {
		goto error;
	}

	*data = TSN_RD32(base_addr + MTL_EST_GCL_DATA);

error:
	return ret;
}

static int est_read_gce(uint32_t base_addr, unsigned int row,
			unsigned int *gates, unsigned int *ti_nsec,
			unsigned int dbgb, unsigned int dbgm)
{
	int ret;
	unsigned int value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int ti_wid = cap->ti_wid;
	unsigned int gates_mask = (1 << cap->txqcnt) - 1;
	unsigned int ti_mask = (1 << ti_wid) - 1;

	ret = est_read_gcl_config(base_addr, &value, row, 0, dbgb, dbgm);
	if (ret) {
		TSN_ERR("Read GCE failed! row=%u", row);
		goto error;
	}
	*ti_nsec = value & ti_mask;
	*gates = (value >> ti_wid) & gates_mask;

error:
	return ret;
}

static unsigned int est_get_gcl_total_intervals_nsec(unsigned int bank,
						     unsigned int gcl_len)
{
	unsigned int row;
	unsigned int nsec = 0;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;

	for (row = 0; row < gcl_len; row++) {
		nsec += gcl->ti_nsec;
		gcl++;
	}

	return nsec;
}

static int est_set_tils(uint32_t base_addr, const unsigned int tils)
{
	unsigned int value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (tils > cap->tils_max) {
		TSN_WARN("EST: invalid tils(%u), max=%u",
			 tils, cap->tils_max);
		ret = -EINVAL;
		goto error;
	}

	/* Ensure that HW is not in the midst of GCL transition */
	value = TSN_RD32(base_addr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	/* MTL_EST_CTRL value has been read earlier, if TILS value
	 * differs, we update here.
	 */
	if (tils != dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_TILS]) {
		value &= ~MTL_EST_CTRL_TILS;
		value |= (tils << MTL_EST_CTRL_TILS_SHIFT);

		TSN_WR32(value, base_addr + MTL_EST_CTRL);
		dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_TILS] = tils;
	}

error:
	return ret;
}

static int est_set_ov(uint32_t base_addr, const unsigned int *ptov,
		      const unsigned int *ctov)
{
	unsigned int value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	value = TSN_RD32(base_addr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	if (ptov) {
		if (*ptov > EST_PTOV_MAX) {
			TSN_WARN("EST: invalid PTOV(%u), max=%u",
				 *ptov, EST_PTOV_MAX);
			ret =  -EINVAL;
			goto error;
		} else if (*ptov !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_PTOV]) {
			value &= ~MTL_EST_CTRL_PTOV;
			value |= (*ptov << MTL_EST_CTRL_PTOV_SHIFT);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_PTOV] = *ptov;
		}
	}

	if (ctov) {
		if (*ctov > EST_CTOV_MAX) {
			TSN_WARN("EST: invalid CTOV(%u), max=%u",
				 *ctov, EST_CTOV_MAX);
			ret = -EINVAL;
			goto error;
		} else if (*ctov != dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_CTOV]) {
			value &= ~MTL_EST_CTRL_CTOV;
			value |= (*ctov << MTL_EST_CTRL_CTOV_SHIFT);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_CTOV] = *ctov;
		}
	}

	TSN_WR32(value, base_addr + MTL_EST_CTRL);

error:
	return ret;
}

#ifdef CONFIG_ETH_DWC_EQOS_QAV
static unsigned long long est_get_all_open_time(unsigned int bank,
						unsigned long long cycle_ns,
						unsigned int queue)
{
	int row;
	unsigned int gcl_len = dw_est_gc_config.gcb[bank].gcrr.llr;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;
	unsigned long long total = 0;
	unsigned long long tti_ns = 0;
	unsigned int gate = 0x1 << queue;

	/* GCL which exceeds the cycle time will be truncated.
	 * So, time interval that exceeds the cycle time will not be
	 * included.
	 */
	for (row = 0; row < gcl_len; row++) {
		tti_ns += gcl->ti_nsec;

		if (gcl->gates & gate) {
			if (tti_ns <= cycle_ns) {
				total += gcl->ti_nsec;
			} else {
				total += gcl->ti_nsec -
					 (tti_ns - cycle_ns);
			}
		}

		gcl++;
	}

	/* The gates without any setting of open/close within
	 * the cycle time are considered as open.
	 */
	if (tti_ns < cycle_ns) {
		total += cycle_ns - tti_ns;
	}

	return total;
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */
#endif /* CONFIG_ETH_DWC_EQOS_QBV */

#ifdef CONFIG_ETH_DWC_EQOS_QBU
static int fpe_set_afsz(uint32_t base_addr, const unsigned int afsz)
{
	unsigned int value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (afsz > FPE_AFSZ_MAX) {
		TSN_WARN_NA("FPE: AFSZ is out-of-bound.");
		ret = -EINVAL;
		goto error;
	}

	if (afsz != dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_AFSZ]) {
		value = TSN_RD32(base_addr + MTL_FPE_CTRL_STS);
		value &= ~MTL_FPE_CTRL_STS_AFSZ;
		value |= afsz;
		TSN_WR32(value, base_addr + MTL_FPE_CTRL_STS);
		dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_AFSZ] = afsz;
	}

error:
	return ret;
}

static int fpe_set_hr_adv(uint32_t base_addr,
			  const unsigned int *hadv,
			  const unsigned int *radv)
{
	unsigned int value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	value = TSN_RD32(base_addr + MTL_FPE_ADVANCE);

	if (hadv) {
		if (*hadv > FPE_ADV_MAX) {
			TSN_WARN("FPE: invalid HADV(%u), max=%u",
				 *hadv, FPE_ADV_MAX);
			ret = -EINVAL;
			goto error;
		} else if (*hadv !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_HADV]) {
			value &= ~MTL_FPE_ADVANCE_HADV;
			value |= (*hadv & MTL_FPE_ADVANCE_HADV);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_HADV] = *hadv;
		}
	}

	if (radv) {
		if (*radv > FPE_ADV_MAX) {
			TSN_WARN("FPE: invalid RADV(%u), max=%u",
				 *radv, FPE_ADV_MAX);
			ret = -EINVAL;
			goto error;
		} else if (*radv !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_RADV]) {
			value &= ~MTL_FPE_ADVANCE_RADV;
			value |= ((*radv << MTL_FPE_ADVANCE_RADV_SHIFT) &
				  MTL_FPE_ADVANCE_RADV);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_RADV] = *radv;
		}
	}

	TSN_WR32(value, base_addr + MTL_FPE_ADVANCE);

error:
	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */

#ifdef CONFIG_ETH_DWC_EQOS_QAV
void dwmac_set_cbs_hilocredit(uint32_t base_addr, int queue)
{
	TSN_WR32(CBS_HICREDIT, base_addr + MTL_TXQ_HICREDIT(queue));
	TSN_WR32(CBS_LOCREDIT, base_addr + MTL_TXQ_LOCREDIT(queue));
}

void dwmac_set_cbs_idlesend(uint32_t base_addr, uint32_t idleslope,
			    int32_t linkspeed, int queue)
{
	uint32_t sendslope;

	sendslope = linkspeed - idleslope;
	TSN_INFO("Set CBS: Queue %u: Idle Slope = %uMbps, Send Slope = "
		 "%uMbps.", queue, idleslope, sendslope);

	if (linkspeed == 100) {
		idleslope = MBPS_TO_BPC_100(idleslope);
		sendslope = MBPS_TO_BPC_100(sendslope);
	} else if (linkspeed == 1000) {
		idleslope = MBPS_TO_BPC_1000(idleslope);
		sendslope = MBPS_TO_BPC_1000(sendslope);
	} else if (linkspeed == 2500) {
		idleslope = MBPS_TO_BPC_2500(idleslope);
		sendslope = MBPS_TO_BPC_2500(sendslope);
	}

	TSN_WR32(idleslope, base_addr + MTL_TXQ_QUANTUM_WEIGHT(queue));
	TSN_WR32(sendslope, base_addr + MTL_TXQ_SENDSLOPECREDIT(queue));
}

void dwmac_set_cbs_status(uint32_t base_addr, int queue, bool enable)
{
	uint32_t value;

	value = TSN_RD32(base_addr + MTL_TXQ_ETS_CONTROL(queue));
	if (enable) {
		value |= MTL_TXQ_ETS_CONTROL_AVALG;
	} else {
		value &= ~MTL_TXQ_ETS_CONTROL_AVALG;
	}
	TSN_WR32(value, base_addr + MTL_TXQ_ETS_CONTROL(queue));
}

void dwmac_get_cbs_status(uint32_t base_addr, int queue, bool *enable)
{
	uint32_t value;

	value = TSN_RD32(base_addr + MTL_TXQ_ETS_CONTROL(queue));
	value &= MTL_TXQ_ETS_CONTROL_AVALG;

	if (value) {
		*enable = 1;
	} else {
		*enable = 0;
	}
}

void dwmac_get_cbs_idleband(uint32_t base_addr, uint32_t *idleband,
			    int32_t linkspeed, int queue, bool idle)
{
	int idleslope;

	idleslope = TSN_RD32(base_addr + MTL_TXQ_QUANTUM_WEIGHT(queue));

	/* offset is added to round the value of idleslope to nearest integer */
	if (linkspeed == 100) {
		idleslope = BPC_TO_MBPS_100(idleslope + ROUDING_OFFSET_100);
	} else if (linkspeed == 1000) {
		idleslope = BPC_TO_MBPS_1000(idleslope + ROUDING_OFFSET_1000);
	} else if (linkspeed == 2500) {
		idleslope = BPC_TO_MBPS_2500(idleslope + ROUDING_OFFSET_2500);
	}

	if (idle) {
		*idleband = MBPS_TO_BPS(idleslope);
	} else {
		*idleband = DECI_TO_PERCENT(idleslope, linkspeed);
	}
}

static void dwmac_tsn_irq(struct k_work *item)
{
	struct eth_runtime *ctxt = CONTAINER_OF(item, struct eth_runtime,
						tsn_work);
	uint32_t base_addr = ctxt->base_addr;

	for (int i = 1; i < ctxt->txqnum; i++) {
		if (ctxt->cbsparam[i - 1].enable) {
			ctxt->cbsparam[i - 1].idle_slope =
				PERCENT_TO_DECI(ctxt->cbsparam[i - 1].bandwidth,
						ctxt->link_speed);
			dwmac_cbs_recal_idleslope(base_addr,
				&ctxt->cbsparam[i - 1].idle_slope,
				ctxt->link_speed, i, 0);
			dwmac_set_cbs_idlesend(base_addr,
				ctxt->cbsparam[i - 1].idle_slope,
				ctxt->link_speed, i);
		}
	}
}
#endif /* CONFIG_ETH_DWC_EQOS_QAV */

#if defined(CONFIG_ETH_DWC_EQOS_QBV) || defined(CONFIG_ETH_DWC_EQOS_QBU)
void dwmac_tsn_init(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int hwid = TSN_RD32(base_addr + MAC_VERSION) &
			    MAC_VERSION_MASK;
	unsigned int hw_cap2 = TSN_RD32(base_addr + MAC_HW_FEATURE2);
	unsigned int hw_cap3 = TSN_RD32(base_addr + MAC_HW_FEATURE3);

	memset(cap, 0, sizeof(*cap));

	if (hwid < TSN_CORE_VER) {
		TSN_WARN_NA("IP v5.00 does not support TSN");
		return;
	}

#ifdef CONFIG_ETH_DWC_EQOS_QBV
	if (!(hw_cap3 & MAC_HW_FEAT_ESTSEL)) {
		TSN_WARN_NA("EST NOT supported");
		cap->est_support = 0;
	} else {
		cap->gcl_depth = est_get_gcl_depth(hw_cap3);
		cap->ti_wid = est_get_ti_width(hw_cap3);

		/* width of the left shift time interval equal to 3 bits */
		cap->tils_max = 7;

		cap->ext_max = (1 << (cap->ti_wid + 7)) - 1;
		cap->txqcnt = ((hw_cap2 & MAC_HW_FEAT2_TXQCNT_MASK) >>
			      MAC_HW_FEAT2_TXQCNT_SHIFT) + 1;
		cap->est_support = 1;

		TSN_INFO("EST: depth=%u, ti_wid=%u, tils_max=7 tqcnt=%u",
			 cap->gcl_depth, cap->ti_wid, cap->txqcnt);

#ifdef CONFIG_ETH_DWC_EQOS_QAV
		k_work_init(&ctxt->tsn_work, dwmac_tsn_irq);
#endif
	}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	if (!(hw_cap3 & MAC_HW_FEAT_FPESEL)) {
		TSN_INFO_NA("FPE NOT supported");
		cap->fpe_support = 0;
	} else {
		TSN_INFO_NA("FPE capable");
		cap->rxqcnt = (hw_cap2 & MAC_HW_FEAT2_RXQCNT_MASK) + 1;
		cap->txqcnt = ((hw_cap2 & MAC_HW_FEAT2_TXQCNT_MASK) >>
			      MAC_HW_FEAT2_TXQCNT_SHIFT) + 1;
		cap->fpe_support = 1;
	}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */
}

void dwmac_get_tsn_hwcap(struct tsn_hw_cap **tsn_hwcap)
{
	*tsn_hwcap = &dw_tsn_hwcap;
}

void dwmac_set_tsn_feat(enum tsn_feat_id featid, bool enable)
{
	if (featid < TSN_FEAT_ID_MAX) {
		dw_tsn_feat_en[featid] = enable;
	}
}

int dwmac_set_tsn_hwtunable(uint32_t base_addr,
			    enum tsn_hwtunable_id id,
			    const unsigned int *data)
{
	int ret = 0;

	switch (id) {
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	case TSN_HWTUNA_TX_EST_TILS:
		ret = est_set_tils(base_addr, *data);
		break;
	case TSN_HWTUNA_TX_EST_PTOV:
		ret = est_set_ov(base_addr, data, NULL);
		break;
	case TSN_HWTUNA_TX_EST_CTOV:
		ret = est_set_ov(base_addr, NULL, data);
		break;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	case TSN_HWTUNA_TX_FPE_AFSZ:
		ret = fpe_set_afsz(base_addr, *data);
		break;
	case TSN_HWTUNA_TX_FPE_HADV:
		ret = fpe_set_hr_adv(base_addr, data, NULL);
		break;
	case TSN_HWTUNA_TX_FPE_RADV:
		ret = fpe_set_hr_adv(base_addr, NULL, data);
		break;
#endif
	default:
		ret = -EINVAL;
	};

	return ret;
}

int dwmac_get_tsn_hwtunable(enum tsn_hwtunable_id id, unsigned int *data)
{
	int ret = 0;

	if (id >= TSN_HWTUNA_MAX) {
		ret = -EINVAL;
		goto error;
	}

	*data = dw_tsn_hwtunable[id];

error:
	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV || CONFIG_ETH_DWC_EQOS_QBU */

#ifdef CONFIG_ETH_DWC_EQOS_QBV
void dwmac_set_est_intr(uint32_t base_addr)
{
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int value;

	if (cap->est_support) {
		value = (MTL_EST_INT_EN_CGCE | MTL_EST_INT_EN_IEHS |
			 MTL_EST_INT_EN_IEHF | MTL_EST_INT_EN_IEBE |
			 MTL_EST_INT_EN_IECC);
		TSN_WR32(value, base_addr + MTL_EST_INT_EN);
	}
}

void dwmac_set_est_gcb(struct est_gc_entry *gcl, unsigned int bank)
{
	if (bank < EST_GCL_BANK_MAX) {
		dw_est_gc_config.gcb[bank].gcl = gcl;
	}
}

int dwmac_get_est_bank(uint32_t base_addr, unsigned int own)
{
	int swol;
	int ret;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	swol = TSN_RD32(base_addr + MTL_EST_STATUS);

	swol = ((swol & MTL_EST_STATUS_SWOL) >>
		MTL_EST_STATUS_SWOL_SHIFT);

	if (own) {
		ret = swol;
	} else {
		ret = (~swol & 0x1);
	}

error:
	return ret;
}

int dwmac_set_est_gce(uint32_t base_addr,
		      struct est_gc_entry *gce, unsigned int row,
		      unsigned int dbgb, unsigned int dbgm)
{
	unsigned int value;
	unsigned int ti_wid;
	unsigned int ti_max;
	unsigned int gates_mask;
	unsigned int bank;
	struct est_gc_entry *gcl;
	int ret = 0;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int gates = gce->gates;
	unsigned int ti_nsec = gce->ti_nsec;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (dbgb >= EST_GCL_BANK_MAX) {
		ret = -EINVAL;
		goto error;
	}

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(base_addr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (!cap->gcl_depth || row > cap->gcl_depth) {
		TSN_WARN("EST: row(%u) > GCL depth(%u)",
			 row, cap->gcl_depth);
		ret = -EINVAL;
		goto error;
	}

	ti_wid = cap->ti_wid;
	ti_max = (1 << ti_wid) - 1;
	if (ti_nsec > ti_max) {
		TSN_WARN("EST: ti_nsec(%u) > upper limit(%u)",
			 ti_nsec, ti_max);
		ret =  -EINVAL;
		goto error;
	}

	gates_mask = (1 << cap->txqcnt) - 1;
	value = ((gates & gates_mask) << ti_wid) | ti_nsec;

	ret = est_write_gcl_config(base_addr, value, row, 0, dbgb, dbgm);
	if (ret) {
		TSN_ERR("EST: GCE write failed: bank=%u row=%u",
			bank, row);
		goto error;
	}

	TSN_INFO("EST: GCE write: dbgm=%u bank=%u row=%u, gc=0x%x",
		 dbgm, bank, row, value);

	/* Since GC write is successful, update GCL copy of the driver */
	gcl = dw_est_gc_config.gcb[bank].gcl + row;
	gcl->gates = gates;
	gcl->ti_nsec = ti_nsec;

error:
	return ret;
}

int dwmac_get_est_gcrr_llr(uint32_t base_addr, unsigned int *gcl_len,
			   unsigned int dbgb, unsigned int dbgm)
{
	unsigned int bank, value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (dbgb >= EST_GCL_BANK_MAX) {
		ret = -EINVAL;
		goto error;
	}

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(base_addr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	ret = est_read_gcl_config(base_addr, &value,
				  GCL_CTRL_ADDR_LLR, 1,
				  dbgb, dbgm);
	if (ret) {
		TSN_ERR("read LLR fail at bank=%u", bank);
			goto error;
	}

	*gcl_len = value;

error:
	return ret;
}

int dwmac_set_est_gcrr_llr(uint32_t base_addr, unsigned int gcl_len,
			   unsigned int dbgb, unsigned int dbgm)
{
	unsigned int bank, value;
	struct est_gcrr *bgcrr;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (dbgb >= EST_GCL_BANK_MAX) {
		ret = -EINVAL;
		goto error;
	}

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(base_addr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (gcl_len > cap->gcl_depth) {
		TSN_WARN("EST: GCL length(%u) > depth(%u)",
			 gcl_len, cap->gcl_depth);
		ret = -EINVAL;
		goto error;
	}

	bgcrr = &dw_est_gc_config.gcb[bank].gcrr;

	if (gcl_len != bgcrr->llr) {
		ret = est_write_gcl_config(base_addr, gcl_len,
					   GCL_CTRL_ADDR_LLR, 1,
					   dbgb, dbgm);
		if (ret) {
			TSN_ERR_NA("EST: GCRR programming failure!");
			goto error;
		}
		bgcrr->llr = gcl_len;
	}

error:
	return ret;
}

int dwmac_set_est_gcrr_times(uint32_t base_addr,
			     struct est_gcrr *gcrr,
			     unsigned int dbgb, unsigned int dbgm)
{
	uint64_t val_ns, sys_ns;
	unsigned int gcl_len, tti_ns, value;
	unsigned int bank;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	struct est_gcrr *bgcrr;
	int ret = 0;
	unsigned int base_sec = gcrr->base_sec;
	unsigned int base_nsec = gcrr->base_nsec;
	unsigned int cycle_sec = gcrr->cycle_sec;
	unsigned int cycle_nsec = gcrr->cycle_nsec;
	unsigned int ext_nsec = gcrr->ter_nsec;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	if (dbgb >= EST_GCL_BANK_MAX) {
		ret = -EINVAL;
		goto error;
	}

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(base_addr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (base_nsec > ONE_SEC_IN_NANOSEC || cycle_nsec > ONE_SEC_IN_NANOSEC) {
		TSN_WARN("EST: base(%u) or cycle(%u) nsec > 1s !",
			 base_nsec, cycle_nsec);
		ret = -EINVAL;
		goto error;
	}

	/* Ensure base time is later than MAC system time */
	val_ns = (uint64_t)base_nsec;
	val_ns += (uint64_t)(base_sec * ONE_SEC_IN_NANOSEC);

	/* Get the MAC system time */
	sys_ns = TSN_RD32(base_addr + MAC_SYS_TIME_NANOSEC);
	sys_ns += TSN_RD32(base_addr + MAC_SYS_TIME_SEC) * ONE_SEC_IN_NANOSEC;

	if (val_ns <= sys_ns) {
		TSN_WARN("EST: base time(%llu) <= system time(%llu)",
			 val_ns, sys_ns);
		ret = -EINVAL;
		goto error;
	}

	if (cycle_sec > EST_CTR_HI_MAX) {
		TSN_WARN("EST: cycle time(%u) > 255 seconds", cycle_sec);
		ret = -EINVAL;
		goto error;
	}

	if (ext_nsec > cap->ext_max) {
		TSN_WARN("EST: invalid time extension(%u), max=%u",
			 ext_nsec, cap->ext_max);
		ret = -EINVAL;
		goto error;
	}

	bgcrr = &dw_est_gc_config.gcb[bank].gcrr;
	gcl_len = bgcrr->llr;

	/* Sanity test on GCL total time intervals against cycle time.
	 * a) For GC length = 1, if its time interval is equal or greater
	 *    than cycle time, it is a constant gate error.
	 * b) If total time interval > cycle time, irregardless of GC
	 *    length, it is not considered an error that GC list is
	 *    truncated. In this case, giving a warning message is
	 *    sufficient.
	 * c) If total time interval < cycle time, irregardless of GC
	 *    length, all GATES are OPEN after the last GC is processed
	 *    until cycle time lapses. This is potentially due to poor
	 *    GCL configuration but is not an error, so we inform user
	 *    about it.
	 */
	tti_ns = est_get_gcl_total_intervals_nsec(bank, gcl_len);
	val_ns = (uint64_t)cycle_nsec;
	val_ns += (uint64_t)(cycle_sec * ONE_SEC_IN_NANOSEC);
	if (gcl_len == 1 && tti_ns >= val_ns) {
		TSN_WARN_NA("EST: Constant gate error!");
		ret = -EINVAL;
		goto error;
	}

	if (tti_ns > val_ns) {
		TSN_WARN_NA("EST: GCL is truncated!");
	}

	if (tti_ns < val_ns) {
		TSN_INFO("EST: All GCs OPEN at %u of %llu-ns cycle",
			 tti_ns, val_ns);
	}

	/* Finally, start programming GCL related registers if the value
	 * differs from the driver copy for efficiency.
	 */

	if (base_nsec != bgcrr->base_nsec) {
		ret |= est_write_gcl_config(base_addr, base_nsec,
					    GCL_CTRL_ADDR_BTR_LO, 1,
					    dbgb, dbgm);
	}

	if (base_sec != bgcrr->base_sec) {
		ret |= est_write_gcl_config(base_addr, base_sec,
					    GCL_CTRL_ADDR_BTR_HI, 1,
					    dbgb, dbgm);
	}

	if (cycle_nsec != bgcrr->cycle_nsec) {
		ret |= est_write_gcl_config(base_addr, cycle_nsec,
					    GCL_CTRL_ADDR_CTR_LO, 1,
					    dbgb, dbgm);
	}

	if (cycle_sec != bgcrr->cycle_sec) {
		ret |= est_write_gcl_config(base_addr, cycle_sec,
					    GCL_CTRL_ADDR_CTR_HI, 1,
					    dbgb, dbgm);
	}

	if (ext_nsec != bgcrr->ter_nsec) {
		ret |= est_write_gcl_config(base_addr, ext_nsec,
					    GCL_CTRL_ADDR_TER, 1,
					    dbgb, dbgm);
	}

	if (ret) {
		TSN_ERR_NA("EST: GCRR programming failure!");
		goto error;
	}

	/* Finally, we are ready to switch SWOL now. */
	value = TSN_RD32(base_addr + MTL_EST_CTRL);
	value |= MTL_EST_CTRL_SSWL;
	TSN_WR32(value, base_addr + MTL_EST_CTRL);

	/* Update driver copy */
	bgcrr->base_sec = base_sec;
	bgcrr->base_nsec = base_nsec;
	bgcrr->cycle_sec = cycle_sec;
	bgcrr->cycle_nsec = cycle_nsec;
	bgcrr->ter_nsec = ext_nsec;

	TSN_INFO_NA("EST: gcrr set successful");

error:
	return ret;
}

int dwmac_set_est_enable(uint32_t base_addr, bool enable)
{
	unsigned int value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	value = TSN_RD32(base_addr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_EEST);
	value |= (enable & MTL_EST_CTRL_EEST);
	TSN_WR32(value, base_addr + MTL_EST_CTRL);
	dw_est_gc_config.enable = enable;

error:
	return ret;
}

int dwmac_get_est_gcc(uint32_t base_addr,
		      struct est_gc_config **gcc, bool frmdrv)
{
	int ret = 0;
	unsigned int bank;
	unsigned int value;
	struct est_gc_config *pgcc;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* Get GC config from driver */
	if (frmdrv) {
		*gcc = &dw_est_gc_config;
		TSN_INFO_NA("EST: read GCL from driver copy done.");
		goto error;
	}

	/* Get GC config from HW */
	pgcc = &dw_est_gc_config;

	value = TSN_RD32(base_addr + MTL_EST_CTRL);
	pgcc->enable = value & MTL_EST_CTRL_EEST;

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		unsigned int llr, row;
		struct est_gc_bank *gcbc = &pgcc->gcb[bank];

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_BTR_LO, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read BTR(low) fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.base_nsec = value;

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_BTR_HI, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read BTR(high) fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.base_sec = value;

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_CTR_LO, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read CTR(low) fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.cycle_nsec = value;

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_CTR_HI, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read CTR(high) fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.cycle_sec = value;

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_TER, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read TER fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.ter_nsec = value;

		ret = est_read_gcl_config(base_addr, &value,
					  GCL_CTRL_ADDR_LLR, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read LLR fail at bank=%u", bank);
			goto error;
		}
		gcbc->gcrr.llr = value;
		llr = value;

		for (row = 0; row < llr; row++) {
			unsigned int gates, ti_nsec;
			struct est_gc_entry *gce = gcbc->gcl + row;

			ret = est_read_gce(base_addr, row, &gates, &ti_nsec,
					   bank, 1);
			if (ret) {
				TSN_ERR("read GCE fail at bank=%u", bank);
				goto error;
			}
			gce->gates = gates;
			gce->ti_nsec = ti_nsec;
		}
	}

	*gcc = pgcc;
	TSN_INFO_NA("EST: read GCL from HW done.");

error:
	return ret;
}

int dwmac_est_irq_status(const struct device *port)
{
	struct eth_runtime *ctxt = port->data;
	uint32_t base_addr = ctxt->base_addr;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	struct tsn_err_stat *err_stat = &dw_err_stat;
	unsigned int txqcnt_mask = (1 << cap->txqcnt) - 1;
	unsigned int status;
	unsigned int value = 0;
	unsigned int feqn = 0;
	unsigned int hbfq = 0;
	unsigned int hbfs = 0;

	status = TSN_RD32(base_addr + MTL_EST_STATUS);

	value = (MTL_EST_STATUS_CGCE | MTL_EST_STATUS_HLBS |
		 MTL_EST_STATUS_HLBF | MTL_EST_STATUS_BTRE |
		 MTL_EST_STATUS_SWLC);

	/* Return if there is no error */
	if (!(status & value)) {
		status = 0;
		goto done;
	}

	/* spin_lock() is not needed here because of BTRE and SWLC
	 * bit will not be altered. Both of the bit will be
	 * polled in dwmac_set_est_gcrr_times()
	 */
	if (status & MTL_EST_STATUS_CGCE) {
		/* Clear Interrupt */
		TSN_WR32(MTL_EST_STATUS_CGCE, base_addr + MTL_EST_STATUS);

		err_stat->cgce_n++;
	}

	if (status & MTL_EST_STATUS_HLBS) {
		value = TSN_RD32(base_addr + MTL_EST_SCH_ERR);
		value &= txqcnt_mask;

		/* Clear Interrupt */
		TSN_WR32(value, base_addr + MTL_EST_SCH_ERR);

		/* Collecting info to shows all the queues that has HLBS */
		/* issue. The only way to clear this is to clear the     */
		/* statistic  */
		err_stat->hlbs_q |= value;
	}

	if (status & MTL_EST_STATUS_HLBF) {
		value = TSN_RD32(base_addr + MTL_EST_FRM_SZ_ERR);
		feqn = value & txqcnt_mask;

		value = TSN_RD32(base_addr + MTL_EST_FRM_SZ_CAP);
		hbfq = (value & MTL_EST_FRM_SZ_CAP_HBFQ_MASK(cap->txqcnt))
			>> MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT;
		hbfs = value & MTL_EST_FRM_SZ_CAP_HBFS_MASK;

		/* Clear Interrupt */
		TSN_WR32(feqn, base_addr + MTL_EST_FRM_SZ_ERR);

		err_stat->hlbf_sz[hbfq] = hbfs;
	}

	if (status & MTL_EST_STATUS_BTRE) {
		if ((status & MTL_EST_STATUS_BTRL) ==
		    MTL_EST_STATUS_BTRL_MAX)
			err_stat->btre_max_n++;
		else
			err_stat->btre_n++;

		err_stat->btrl = (status & MTL_EST_STATUS_BTRL) >>
					MTL_EST_STATUS_BTRL_SHIFT;

		/* Clear Interrupt */
		TSN_WR32(MTL_EST_STATUS_BTRE, base_addr +
			 MTL_EST_STATUS);
	}

	if (status & MTL_EST_STATUS_SWLC) {
		/* Clear Interrupt */
		TSN_WR32(MTL_EST_STATUS_SWLC, base_addr +
			 MTL_EST_STATUS);
		TSN_INFO_NA("SWOL has been switched");

#ifdef CONFIG_ETH_DWC_EQOS_QAV
		/* Recalculate idle slope based on oper GCL */
		k_work_submit(&ctxt->tsn_work);
#endif
	}

done:
	return status;
}

int dwmac_get_est_err_stat(struct tsn_err_stat **err_stat)
{
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	*err_stat = &dw_err_stat;

error:
	return ret;
}

int dwmac_clr_est_err_stat(uint32_t base_addr)
{
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	memset(&dw_err_stat, 0, sizeof(dw_err_stat));

error:
	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV */

#if defined(CONFIG_ETH_DWC_EQOS_QBV) && defined(CONFIG_ETH_DWC_EQOS_QAV)
int dwmac_cbs_recal_idleslope(uint32_t base_addr, unsigned int *idle_slope,
			      int32_t linkspeed, unsigned int queue, bool swol)
{
	unsigned int open_time;
	int hw_bank = dwmac_get_est_bank(base_addr, swol);
	int ret = 0;
	unsigned long long new_idle_slope;
	unsigned long long scaling = 0;

	if (hw_bank < 0) {
		ret = hw_bank;
		goto error;
	}

	unsigned long long cycle_time_ns =
			(dw_est_gc_config.gcb[hw_bank].gcrr.cycle_sec *
			 ONE_SEC_IN_NANOSEC) +
			 dw_est_gc_config.gcb[hw_bank].gcrr.cycle_nsec;

	if (!cycle_time_ns) {
		TSN_WARN_NA("EST: Cycle time is 0.");
		TSN_WARN_NA("CBS idle slope will not be reconfigured.");
		ret = -EINVAL;
		goto error;
	}

	open_time = est_get_all_open_time(hw_bank,
					  cycle_time_ns,
					  queue);

	if (!open_time) {
		TSN_WARN("EST: Total gate open time for queue %d is 0",
			 queue);
		ret = -EINVAL;
		goto error;
	}

	scaling = cycle_time_ns / open_time;

	new_idle_slope = *idle_slope * scaling;
	if (new_idle_slope > linkspeed) {
		new_idle_slope = linkspeed;
	}

	*idle_slope = new_idle_slope;

error:
	return ret;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBV && CONFIG_ETH_DWC_EQOS_QAV */

#ifdef CONFIG_ETH_DWC_EQOS_QBU
void dwmac_set_fpe_fprq(uint32_t base_addr, unsigned int fprq)
{
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int value;

	if (cap->fpe_support && fprq <= cap->rxqcnt) {
		/* Update FPRQ */
		value = TSN_RD32(base_addr + MAC_RXQ_CTRL1);
		value &= ~MAC_RXQCTRL_FPRQ_MASK;
		value |= fprq << MAC_RXQCTRL_FPRQ_SHIFT;
		TSN_WR32(value, base_addr + MAC_RXQ_CTRL1);
	} else {
		TSN_WARN_NA("FPE: FPRQ is out-of-bound.");
	}
}

int dwmac_set_fpe_config(uint32_t base_addr, struct fpe_config *fpec,
			 uint32_t est_status)
{
	unsigned int txqmask, value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* Check PEC is within TxQ range */
	txqmask = (1 << cap->txqcnt) - 1;
	if (fpec->txqpec & ~txqmask) {
		TSN_WARN_NA("FPE: Tx PEC is out-of-bound.");
		ret = -EINVAL;
		goto error;
	}

	/* When EST and FPE are both enabled, TxQ0 is always preemptable
	 * queue. If FPE is enabled, we expect at least lsb is set.
	 * If FPE is not enabled, we also allow PEC = 0.
	 */
	if (fpec->txqpec && !(fpec->txqpec & FPE_PMAC_BIT) && est_status) {
		TSN_WARN_NA("FPE: TxQ0 must not be express queue.");
		ret = -EINVAL;
		goto error;
	}

	/* Field masking not needed as condition checks have been done */
	value = TSN_RD32(base_addr + MTL_FPE_CTRL_STS);
	value &= ~(txqmask << MTL_FPE_CTRL_STS_PEC_SHIFT);
	value |= (fpec->txqpec << MTL_FPE_CTRL_STS_PEC_SHIFT);
	TSN_WR32(value, base_addr + MTL_FPE_CTRL_STS);

	/* Update driver copy */
	dw_fpe_config.txqpec = fpec->txqpec;

error:
	return ret;
}

int dwmac_set_fpe_enable(uint32_t base_addr, bool enable)
{
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	dw_fpe_config.lp_fpe_support = 0;
	dw_fpe_config.enable = enable & MAC_FPE_CTRL_STS_EFPE;

	TSN_WR32((unsigned int)dw_fpe_config.enable,
		 base_addr + MAC_FPE_CTRL_STS);

error:
	return ret;
}

int dwmac_get_fpe_config(uint32_t base_addr, struct fpe_config **fpec,
			 bool frmdrv)
{
	unsigned int value;
	struct fpe_config *pfpec;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* Get FPE config from driver */
	if (frmdrv) {
		*fpec = &dw_fpe_config;
		TSN_INFO_NA("FPE: read config from driver copy done.");
		goto error;
	}

	pfpec = &dw_fpe_config;

	value = TSN_RD32(base_addr + MTL_FPE_CTRL_STS);
	pfpec->txqpec = (value & MTL_FPE_CTRL_STS_PEC) >>
			MTL_FPE_CTRL_STS_PEC_SHIFT;

	value = TSN_RD32(base_addr + MAC_FPE_CTRL_STS);
	pfpec->enable = (bool)(value & MAC_FPE_CTRL_STS_EFPE);

	*fpec = pfpec;
	TSN_INFO_NA("FPE: read config from HW done.");

error:
	return ret;
}

int dwmac_get_fpe_pmac_sts(uint32_t base_addr, unsigned int *hrs)
{
	unsigned int value;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE]) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	value = TSN_RD32(base_addr + MTL_FPE_CTRL_STS);
	*hrs = (value & MTL_FPE_CTRL_STS_HRS) >> MTL_FPE_CTRL_STS_HRS_SHIFT;

	if (hrs) {
		TSN_INFO_NA("FPE: pMAC is in Hold state.");
	} else {
		TSN_INFO_NA("FPE: pMAC is in Release state.");
	}

error:
	return ret;
}

int dwmac_fpe_irq_status(uint32_t base_addr)
{
	unsigned int status;
	int fpe_state = FPE_STATE_UNKNOWN;

	status = TSN_RD32(base_addr + MAC_FPE_CTRL_STS);

	if (status & MAC_FPE_CTRL_STS_TRSP) {
		if (dw_fpe_config.enable) {
			dw_fpe_config.lp_fpe_support = 1;
		}
		TSN_INFO_NA("Respond mPacket is transmitted");
		fpe_state |= FPE_STATE_TRSP;
	}

	if (status & MAC_FPE_CTRL_STS_TVER) {
		TSN_INFO_NA("Verify mPacket is transmitted");
		fpe_state |= FPE_STATE_TVER;
	}

	if (status & MAC_FPE_CTRL_STS_RRSP) {
		dw_fpe_config.lp_fpe_support = 1;
		TSN_INFO_NA("Respond mPacket is received");
		fpe_state |= FPE_STATE_RRSP;
	}

	if (status & MAC_FPE_CTRL_STS_RVER) {
		TSN_INFO_NA("Verify mPacket is received");
		fpe_state |= FPE_STATE_RVER;
	}

	return fpe_state;
}

int dwmac_fpe_send_mpacket(uint32_t base_addr, enum mpacket_type type)
{
	unsigned int value;
	int ret = 0;

	value = TSN_RD32(base_addr + MAC_FPE_CTRL_STS);

	switch (type) {
	case MPACKET_VERIFY:
		dw_fpe_config.lp_fpe_support = 0;
		value &= ~MAC_FPE_CTRL_STS_SRSP;
		value |= MAC_FPE_CTRL_STS_SVER;
		break;
	case MPACKET_RESPONSE:
		value &= ~MAC_FPE_CTRL_STS_SVER;
		value |= MAC_FPE_CTRL_STS_SRSP;
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	TSN_WR32(value, base_addr + MAC_FPE_CTRL_STS);

	return ret;
}

void dwmac_fpe_reset_lp_status(void)
{
	dw_fpe_config.lp_fpe_support = 0;
}
#endif /* CONFIG_ETH_DWC_EQOS_QBU */
