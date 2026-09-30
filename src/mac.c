// SPDX-License-Identifier: GPL-2.0
/*
 * MAC-layer hardware init: data-link-engine (packet buffer) partitioning,
 * host flow control, DMAC/CMAC function blocks.
 */
#include <linux/delay.h>

#include "ax52.h"

/*
 * Packet buffer layout. The 192 KiB shared buffer is split between WDE
 * (64-byte pages, WiFi descriptors) and PLE (128-byte pages, payload).
 * Each quota register holds min[11:0] | max[27:16] pages.
 */
struct dle_layout {
	u16 wde_lnk, ple_lnk;		/* linked (free-list) page counts */
	u8 ple_bound;			/* PLE start, in 8 KiB units */
	u32 wde_qta[4];			/* hif, wcpu, pkt_in, cpu_io */
	u32 ple_qta[11];		/* cma0_tx .. cpu_io */
};

#define QTA(min, max)	((u32)(min) | ((u32)(max) << 16))

/* Firmware-download layout: only H2C and C2H buffers. */
static const struct dle_layout dle_dlfw = {
	.wde_lnk = 0, .ple_lnk = 64, .ple_bound = 8,
	.wde_qta = { 0, QTA(48, 0), 0, 0 },
	.ple_qta = { 0, 0, QTA(16, 16), QTA(48, 48), 0, 0, 0, 0, 0, 0, 0 },
};

/* Normal single-band layout. */
static const struct dle_layout dle_scc = {
	.wde_lnk = 510, .ple_lnk = 496, .ple_bound = 4,
	.wde_qta = { QTA(446, 446), QTA(48, 48), 0, QTA(16, 16) },
	.ple_qta = {
		QTA(147, 147),	/* cma0_tx: band-0 TX payload */
		0,		/* cma1_tx */
		QTA(16, 16),	/* c2h */
		QTA(20, 20),	/* h2c */
		QTA(17, 157),	/* wcpu */
		QTA(13, 13),	/* mpdu_proc */
		QTA(89, 229),	/* cma0_dma: band-0 RX */
		0,		/* cma1_dma */
		QTA(32, 172),	/* bb_rpt */
		QTA(14, 14),	/* wd_rel */
		QTA(8, 24),	/* cpu_io */
	},
};

static const u16 wde_qta_reg[4] = { 0x8C40, 0x8C44, 0x8C4C, 0x8C50 };

static int dle_init(struct ax52_dev *rd, const struct dle_layout *l)
{
	u32 v;
	int i, ret;

	v = rd32(rd, REG_DMAC_FUNC_EN);
	if ((v & (DMAC_FUNC_MAC_FUNC_EN | DMAC_FUNC_DMAC_FUNC_EN)) !=
	    (DMAC_FUNC_MAC_FUNC_EN | DMAC_FUNC_DMAC_FUNC_EN)) {
		ax52_err(rd, "DLE init with DMAC disabled (0x%08x)\n", v);
		return -EIO;
	}

	clr32(rd, REG_DMAC_FUNC_EN, DMAC_FUNC_DLE_WDE_EN | DMAC_FUNC_DLE_PLE_EN);
	set32(rd, REG_DMAC_CLK_EN, DMAC_CLK_DLE_WDE | DMAC_CLK_DLE_PLE);

	v = rd32(rd, REG_WDE_PKTBUF_CFG);
	v &= ~(PKTBUF_PAGE_SEL_MASK | PKTBUF_START_BOUND_MASK | PKTBUF_FREE_PAGE_MASK);
	v |= FIELD_PREP(PKTBUF_PAGE_SEL_MASK, 0) |		/* 64 B */
	     FIELD_PREP(PKTBUF_FREE_PAGE_MASK, l->wde_lnk);
	wr32(rd, REG_WDE_PKTBUF_CFG, v);

	v = rd32(rd, REG_PLE_PKTBUF_CFG);
	v &= ~(PKTBUF_PAGE_SEL_MASK | PKTBUF_START_BOUND_MASK | PKTBUF_FREE_PAGE_MASK);
	v |= FIELD_PREP(PKTBUF_PAGE_SEL_MASK, 1) |		/* 128 B */
	     FIELD_PREP(PKTBUF_START_BOUND_MASK, l->ple_bound) |
	     FIELD_PREP(PKTBUF_FREE_PAGE_MASK, l->ple_lnk);
	wr32(rd, REG_PLE_PKTBUF_CFG, v);

	for (i = 0; i < 4; i++)
		wr32(rd, wde_qta_reg[i], l->wde_qta[i]);
	for (i = 0; i < 11; i++)
		wr32(rd, REG_PLE_QTA0 + 4 * i, l->ple_qta[i]);

	set32(rd, REG_DMAC_FUNC_EN, DMAC_FUNC_DLE_WDE_EN | DMAC_FUNC_DLE_PLE_EN);

	ret = ax52_poll32(rd, REG_WDE_INI_STATUS, 0x3, 0x3, 1, 2000);
	if (!ret)
		ret = ax52_poll32(rd, REG_PLE_INI_STATUS, 0x3, 0x3, 1, 2000);
	if (ret)
		ax52_err(rd, "DLE init timeout (WDE 0x%08x PLE 0x%08x)\n",
			 rd32(rd, REG_WDE_INI_STATUS), rd32(rd, REG_PLE_INI_STATUS));
	return ret;
}

/* Host flow control with only the H2C channel accounted (for FW download). */
static void hfc_init_h2c_only(struct ax52_dev *rd)
{
	u32 v;

	clr32(rd, REG_HCI_FC_CTRL, HCI_FC_EN | HCI_FC_CH12_EN);
	wr32(rd, REG_CH_PAGE_CTRL, 40 << 16);		/* H2C pre-cost 40 pages */
	mask32(rd, REG_HCI_FC_CTRL, HCI_FC_CH12_FULL_COND_MASK, 0);

	v = rd32(rd, REG_HCI_FC_CTRL);
	v &= ~HCI_FC_EN;
	v |= HCI_FC_CH12_EN;
	wr32(rd, REG_HCI_FC_CTRL, v);
}

/* Full host flow control for the normal (single-band) layout. */
static int hfc_init_full(struct ax52_dev *rd)
{
	/* per channel: min[12:0] | max[28:16] pages; channels 0-3, 8, 9 */
	static const struct { u8 ch; u32 val; } ch_cfg[] = {
		{ 0, 0x01550005 }, { 1, 0x01550005 }, { 2, 0x01560004 },
		{ 3, 0x01560004 }, { 8, 0x01560004 }, { 9, 0x01560004 },
	};
	u32 v, g0, g1, pub;
	int i;

	clr32(rd, REG_HCI_FC_CTRL, HCI_FC_EN | HCI_FC_CH12_EN);
	for (i = 0; i < ARRAY_SIZE(ch_cfg); i++)
		wr32(rd, REG_ACH0_PAGE_CTRL + 4 * ch_cfg[i].ch, ch_cfg[i].val);

	wr32(rd, REG_PUB_PAGE_CTRL1, 446);		/* group 0: all 446 pages */
	wr32(rd, REG_WP_PAGE_CTRL2, 0);
	wr32(rd, REG_CH_PAGE_CTRL, (40 << 16) | 2);	/* pre-cost: H2C 40, ch0-11 2 */
	wr32(rd, REG_PUB_PAGE_CTRL2, 446);
	wr32(rd, REG_WP_PAGE_CTRL1, 0);

	v = rd32(rd, REG_HCI_FC_CTRL);
	v &= ~(HCI_FC_MODE_MASK | HCI_FC_WD_FULL_COND_MASK |
	       HCI_FC_CH12_FULL_COND_MASK | HCI_FC_WP_CH07_FULL_COND_MASK |
	       HCI_FC_WP_CH811_FULL_COND_MASK);
	v |= FIELD_PREP(HCI_FC_WD_FULL_COND_MASK, 1);
	wr32(rd, REG_HCI_FC_CTRL, v);

	set32(rd, REG_HCI_FC_CTRL, HCI_FC_EN | HCI_FC_CH12_EN);
	udelay(10);

	g0 = rd32(rd, REG_PUB_PAGE_INFO1) & GENMASK(12, 0);
	g1 = (rd32(rd, REG_PUB_PAGE_INFO1) >> 16) & GENMASK(12, 0);
	pub = rd32(rd, REG_PUB_PAGE_INFO2) & GENMASK(12, 0);
	if (g0 + g1 + pub != 446) {
		ax52_err(rd, "HFC page accounting off: %u + %u + %u\n", g0, g1, pub);
		return -EFAULT;
	}
	return 0;
}

static void enable_bb_rf(struct ax52_dev *rd)
{
	set8(rd, REG_SYS_FUNC_EN, FEN_BBRSTB | FEN_BB_GLB_RSTN);
	mask32(rd, REG_SPS_DIG_ON_CTRL0, SPS_REG_ZCDC_H_MASK, 1);
	set32(rd, REG_WLRF_CTRL, WLRF_AFC_AFEDIG);
	clr32(rd, REG_WLRF_CTRL, WLRF_AFC_AFEDIG);
	set32(rd, REG_WLRF_CTRL, WLRF_AFC_AFEDIG);
	ax52_xsi_write(rd, XSI_WL_RFC_S0, 0xC7, 0xFF);	/* RF path A/B power */
	ax52_xsi_write(rd, XSI_WL_RFC_S1, 0xC7, 0xFF);
	wr8(rd, 0x8040, 0x0E);				/* PHYREG_SET: XYN cycle */
}

static void disable_bb_rf(struct ax52_dev *rd)
{
	u8 v;

	clr32(rd, REG_WLRF_CTRL, WLRF_AFC_AFEDIG);
	clr8(rd, REG_SYS_FUNC_EN, FEN_BBRSTB | FEN_BB_GLB_RSTN);
	if (!ax52_xsi_read(rd, XSI_WL_RFC_S0, &v))
		ax52_xsi_write(rd, XSI_WL_RFC_S0, v & ~0x07, 0xFF);
	if (!ax52_xsi_read(rd, XSI_WL_RFC_S1, &v))
		ax52_xsi_write(rd, XSI_WL_RFC_S1, v & ~0x07, 0xFF);
}

/* Cycle BB/RF power so the PHY tables are applied to a freshly reset PHY. */
void ax52_mac_reset_bb_rf(struct ax52_dev *rd)
{
	disable_bb_rf(rd);
	enable_bb_rf(rd);
}

static void sys_init(struct ax52_dev *rd)
{
	/* full DMAC function set; DLE engines come back with the SCC layout */
	wr32(rd, REG_DMAC_FUNC_EN, 0xFB7D0000);
	wr32(rd, REG_DMAC_CLK_EN, 0x0B1F0000);
	/* CMAC band 0: clocks and functions */
	set32(rd, 0xC004, 0x4000003F);
	set32(rd, 0xC000, 0xF000003F);
	mask32(rd, REG_SPS_DIG_ON_CTRL0, SPS_OCP_L1_MASK, 7);
}

static int dmac_init(struct ax52_dev *rd)
{
	int ret;

	ret = dle_init(rd, &dle_scc);
	if (ret)
		return ret;
	ret = hfc_init_full(rd);
	if (ret)
		return ret;

	/* station scheduler */
	set8(rd, 0x9E10, BIT(0));
	ret = ax52_poll32(rd, 0x9E10, BIT(31), BIT(31), 1, 2000);
	if (ret) {
		ax52_err(rd, "station scheduler init timeout\n");
		return ret;
	}
	set32(rd, 0x9E10, BIT(29));
	clr32(rd, 0x9E10, BIT(28));

	/* MPDU processor: forward rules, append FCS, keep ICV-error frames */
	wr32(rd, 0x9C04, 0x02A95A95);
	wr32(rd, 0x9C14, 0x0000AA55);
	set32(rd, 0x9C00, BIT(0) | BIT(1));
	wr32(rd, 0x9C40, 0x010E05F0);

	/* security engine clocks and enc/dec paths; append ICV/MIC */
	wr32(rd, 0x9D00, (rd32(rd, 0x9D00) | 0x70F) & ~BIT(11));
	set32(rd, 0x9D04, BIT(1) | BIT(0));
	return 0;
}

static int cmac_init(struct ax52_dev *rd)
{
	u32 v;
	u16 v16;
	int ret;

	if (!(rd32(rd, 0xC000) & BIT(30)))
		return -EIO;

	/* scheduler */
	mask32(rd, 0xC33C, GENMASK(6, 0), 0x47);
	set32(rd, 0xC3FC, BIT(1));
	clr32(rd, 0xC340, BIT(5));
	mask32(rd, 0xC338, GENMASK(4, 0), 0x18);

	/* address CAM: enable, full range, clear */
	v = rd32(rd, 0xCE34);
	v |= FIELD_PREP(GENMASK(23, 16), 0x7F) | BIT(8) | BIT(0);
	wr32(rd, 0xCE34, v);
	ret = read_poll_timeout(rd16, v16, !(v16 & BIT(8)), 1, 2000, false,
				rd, 0xCE34);
	if (ret) {
		ax52_err(rd, "address CAM clear timeout\n");
		return ret;
	}

	/* RX filter: every frame type to the host */
	wr32(rd, 0xCE28, 0x55555555);
	wr32(rd, 0xCE24, 0x55555555);
	wr32(rd, 0xCE2C, 0x55555555);
	wr32(rd, 0xCE20, AX52_RX_FLTR_DEFAULT);
	wr16(rd, 0xCE04, 0x007F);

	/* CCA control; no response checks on NAV/CCA */
	v = rd32(rd, 0xC390);
	v |= 0x712100FF;
	v &= ~0x8E1E0100;
	wr32(rd, 0xC390, v);
	clr32(rd, 0xCC00, 0x00380000);
	clr32(rd, 0xCC04, 0x1F800000);

	/* NAV: 25 ms upper bound */
	set32(rd, 0xCC80, BIT(26) | BIT(17) | BIT(16));
	mask32(rd, 0xCC80, GENMASK(15, 8), 0xC4);

	/* spatial reuse off */
	clr8(rd, 0xCE4A, BIT(0));
	set8(rd, 0xCE4B, BIT(0));

	/* TMAC */
	clr32(rd, 0xCC20, BIT(0));
	mask32(rd, 0xCA00, GENMASK(22, 16), 6);
	mask32(rd, 0xCA1C, GENMASK(15, 12), 7);
	mask32(rd, 0xCA1C, GENMASK(11, 8), 7);

	/* TRX protocol: SIFS for CCK/OFDM responses */
	v = rd32(rd, 0xCC04);
	v = (v & ~GENMASK(15, 0)) | 0x0A | (0x11 << 8);
	wr32(rd, 0xCC04, v);
	set32(rd, 0xCCB0, BIT(20));
	mask32(rd, 0xCC08, BIT(9), 0);
	mask32(rd, 0xCC08, GENMASK(11, 10), 2);

	/* RMAC: reset responder BA CAM, timeouts, max MPDU length */
	mask32(rd, 0xCE3C, GENMASK(1, 0), 2);
	ret = read_poll_timeout_atomic(rd32, v, !(v & GENMASK(1, 0)), 1, 1000,
				       false, rd, 0xCE3C);
	if (ret) {
		ax52_err(rd, "BA CAM reset timeout\n");
		return ret;
	}
	set8(rd, 0xCE3C, BIT(2));
	v16 = rd16(rd, 0xCE02);
	v16 = (v16 & ~GENMASK(15, 4)) | (15 << 4) | (32 << 8);
	wr16(rd, 0xCE02, v16);
	mask8(rd, 0xCE00, GENMASK(3, 0), 1);
	mask32(rd, 0xCE20, GENMASK(21, 16), 22);	/* 22 x 512 B */
	clr8(rd, 0xCE04, BIT(4));

	/* CMAC common: TX sub-carrier, RRSR */
	clr32(rd, 0xC088, GENMASK(11, 0));
	mask32(rd, 0xC090, GENMASK(11, 8), 3);

	/* PTCL */
	v = rd32(rd, 0xC624);
	v = (v & ~GENMASK(31, 18)) | (4 << 24) | (1 << 18) | BIT(16);
	wr32(rd, 0xC624, v);
	v = rd32(rd, 0xC6E8);
	v = (v & ~(GENMASK(5, 0) | BIT(6))) | 0x3F;
	wr32(rd, 0xC6E8, v);
	set8(rd, 0xC600, BIT(0) | BIT(1));
	clr8(rd, 0xC600, BIT(2) | BIT(3) | BIT(4));
	mask8(rd, 0xC660, GENMASK(5, 4), 1);
	mask32(rd, 0xC618, GENMASK(19, 0), 0x3FF80);

	/* CMAC DMA: no "full mode" reports */
	clr8(rd, 0xC804, 0x3F);
	return 0;
}

/* Error interrupt masks feeding the firmware's error-recovery reporting. */
static void enable_err_imrs(struct ax52_dev *rd)
{
	static const struct { u16 reg; u32 clr, set; } imr[] = {
		{ 0x9430, 0x00003337, 0x00003327 },
		{ 0x9D1C, 0,          0x00000008 },
		{ 0x9BF4, 0x0000003E, 0 },
		{ 0x9CF4, 0x0000000B, 0 },
		{ 0x9EF0, 0x00000007, 0x00000007 },
		{ 0x9F1C, 0x0000030F, 0x00000101 },
		{ 0x9F2C, 0x0000030F, 0x00000303 },
		{ 0x8C38, 0x070FF0FF, 0x070FF0FF },
		{ 0x9038, 0x070FF0FF, 0x070FF0DF },
		{ 0x9A20, 0,          0x00000001 },
		{ 0x8850, 0xFF0FFFFF, 0x0C000161 },
		{ 0x8854, 0xFF07FFFF, 0x04000062 },
		{ 0x8858, 0x3F031F1F, 0 },
		{ 0x9840, 0x00001111, 0x00001111 },
		{ 0x960C, 0,          0x00000001 },
		{ 0x962C, 0x000000FF, 0 },
		{ 0x963C, 0,          0x00000001 },
		{ 0x966C, 0,          0x00000001 },
		{ 0xC3E8, 0x00000003, 0x00000001 },
		{ 0xC6C0, 0xFFFFFFFF, 0x10800001 },
		{ 0xC800, 0x0080C000, 0x0000C000 },
		{ 0xCEF4, 0x000FF000, 0x000E4000 },
		{ 0xCCEC, 0x00000780, 0x00000780 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(imr); i++) {
		u32 v = rd32(rd, imr[i].reg);

		wr32(rd, imr[i].reg, (v & ~imr[i].clr) | imr[i].set);
	}
	wr32(rd, 0x8520, 0xFFFFFFFF);	/* DMAC_ERR_IMR */
	wr32(rd, 0xC160, 0xFFFFFFFF);	/* CMAC_ERR_IMR */
}

/*
 * MAC bring-up after the firmware is running: BB/RF power, the full DMAC
 * and CMAC function blocks, the normal packet-buffer layout, then release
 * all TX DMA channels.
 */
int ax52_mac_init(struct ax52_dev *rd)
{
	int ret;

	enable_bb_rf(rd);
	sys_init(rd);

	ret = dmac_init(rd);
	if (ret)
		return ret;
	ret = cmac_init(rd);
	if (ret)
		return ret;
	enable_err_imrs(rd);

	/* TX release reports to the host (RPQ ring), 30 per buffer */
	mask32(rd, 0x9408, GENMASK(1, 0), 0);
	set32(rd, 0x9410, 0x0F000000);
	mask32(rd, 0x9414, GENMASK(7, 0), 30);
	mask32(rd, 0x9414, GENMASK(23, 16), 255);

	/* PPDU status reports go to the host */
	mask32(rd, 0x9C18, GENMASK(1, 0), 1);

	ax52_pci_post_init(rd);
	return 0;
}

void ax52_mac_ppdu_status(struct ax52_dev *rd, bool enable)
{
	if (enable)
		wr32(rd, 0xCE40, 0x2B);	/* RPT_EN | MAC_INFO | PLCP_HDR | CRC32 */
	else
		clr32(rd, 0xCE40, BIT(0));
}

void ax52_mac_set_rx_filter(struct ax52_dev *rd, u32 fltr)
{
	u32 v = rd32(rd, 0xCE20);

	wr32(rd, 0xCE20, (v & GENMASK(21, 16)) | (fltr & ~GENMASK(21, 16)));
}

void ax52_mac_set_rts_threshold(struct ax52_dev *rd, u32 thr)
{
	if (thr == (u32)-1) {
		mask16(rd, 0xC614, 0xFF00, 88 >> 5);
		mask16(rd, 0xC614, 0x00FF, 4080 >> 4);
	} else {
		mask16(rd, 0xC614, 0xFF00, 0xFF);
		mask16(rd, 0xC614, 0x00FF, min_t(u32, thr >> 4, 0xFF));
	}
}

/* TX A-MPDU length limit tried by rate adaptation (subframes - 1). */
void ax52_mac_set_agg_limit(struct ax52_dev *rd, u8 lmt)
{
	mask32(rd, 0xC610, GENMASK(23, 16), lmt);
}

/* Wait until the packet engine's TX queues are empty. */
int ax52_mac_wait_txq_empty(struct ax52_dev *rd)
{
	u32 v;

	return read_poll_timeout(rd32, v, (v & 0x07FF079F) == 0x07FF079F,
				 10000, 200000, false, rd, 0x8430);
}

/* Port 0 configuration for a station interface. */
void ax52_mac_port_update(struct ax52_dev *rd, struct ax52_vif *rv,
			  struct ieee80211_vif *vif)
{
	bool link = rv->net_type != 0;
	u16 bcn_int = vif->bss_conf.beacon_int ?: 100;
	u8 dtim = vif->bss_conf.dtim_period;

	if (rd32(rd, 0xC400) & BIT(2)) {	/* port running: stop it first */
		msleep(bcn_int + 1);
		clr32(rd, 0xC400, BIT(2) | BIT(16));
		set32(rd, 0xC400, BIT(5));	/* TSF reset */
		wr32(rd, 0xC434, 0);
	}
	clr32(rd, 0xC400, BIT(1) | BIT(0));
	mask32(rd, 0xC400, GENMASK(11, 10), rv->net_type);
	if (link)
		set32(rd, 0xC400, BIT(13) | BIT(16) | BIT(4) | BIT(3));
	else
		clr32(rd, 0xC400, BIT(13) | BIT(16) | BIT(4) | BIT(3));
	clr32(rd, 0xC400, BIT(12));		/* stations send no beacons */
	mask32(rd, 0xC414, GENMASK(15, 0), bcn_int);
	wr8(rd, 0xC590, 0);
	set8(rd, 0xCA08, BIT(1) | BIT(0));
	mask16(rd, 0xC426, 0xFF00, dtim);
	clr32(rd, 0xC63C, BIT(16) | BIT(0));
	mask32(rd, 0xC404, GENMASK(7, 0), 2);
	mask32(rd, 0xC404, GENMASK(27, 16), 200);
	mask32(rd, 0xC408, GENMASK(27, 16), 0);
	mask16(rd, 0xC40E, GENMASK(11, 0), 5);
	mask16(rd, 0xC412, GENMASK(15, 8), 1);
	mask32(rd, 0xC6A0, GENMASK(5, 0), rv->bss_color);
	clr32(rd, 0xC568, 0x00FFFFFE);
	set32(rd, 0xC400, BIT(2));
	fsleep(20);
	mask32(rd, 0xC40C, GENMASK(11, 0), 160);
	mask32(rd, 0xCE84, GENMASK(10, 0), 0);
}

/* Default per-MACID DMAC/CMAC control tables (direct indirect-memory writes). */
void ax52_mac_macid_tbl_init(struct ax52_dev *rd, u8 mac_id)
{
	static const u32 cctl[8] = {
		0x00000004, 0x400A0004, 0, 0, 0, 0x0E43000B, 0, 0x000B8109,
	};
	int i;

	for (i = 0; i < 4; i++) {
		wr32(rd, REG_FILTER_MODEL_ADDR, 0x18800000 + mac_id * 16 + i * 4);
		wr32(rd, REG_INDIR_ACCESS_ENTRY, 0);
	}
	wr32(rd, REG_FILTER_MODEL_ADDR, 0x18840000 + mac_id * 32);
	for (i = 0; i < 8; i++)
		wr32(rd, REG_INDIR_ACCESS_ENTRY + 4 * i, cctl[i]);
}

/* ------------------------------------------------------------------ coex */

#define LTE_CTRL		0xDAF0
#define LTE_WDATA		0xDAF4
#define LTE_RDATA		0xDAF8
#define LTE_SW_CFG_1		0x38	/* GNT override */
#define LTE_SW_CFG_2		0x3C

#define GNT_WL_ONLY		0x77007700	/* WL sw-high, BT sw-low, both paths */
#define GNT_WL_5G		0x33003300	/* WL sw-high, BT hardware */

#define COEX_SB_ACTIVE		BIT(0)
#define COEX_SB_ON			BIT(1)
#define COEX_SB_WLRFK		BIT(11)
#define COEX_SB_BTLOG		BIT(14)
#define COEX_SB_BT_RFK_RUN		BIT(5)
#define COEX_SB_BT_RFK_REQ		BIT(6)

static int lte_wait_ready(struct ax52_dev *rd)
{
	return ax52_poll8(rd, LTE_CTRL + 3, BIT(5), BIT(5), 50, 50000);
}

int ax52_lte_write(struct ax52_dev *rd, u32 off, u32 val)
{
	int ret = lte_wait_ready(rd);

	if (ret)
		return ret;
	wr32(rd, LTE_WDATA, val);
	wr32(rd, LTE_CTRL, 0xC00F0000 | off);
	return 0;
}

int ax52_lte_read(struct ax52_dev *rd, u32 off, u32 *val)
{
	int ret = lte_wait_ready(rd);

	if (ret)
		return ret;
	wr32(rd, LTE_CTRL, 0x800F0000 | off);
	*val = rd32(rd, LTE_RDATA);
	return 0;
}

/* WL->BT scoreboard: our 24 driver bits, plus the "powered" firmware bit. */
static void coex_set_sb(struct ax52_dev *rd, u32 bits)
{
	u32 fw = (rd32(rd, REG_SCOREBOARD) >> 24) & 0x7E;

	rd->coex_sb = bits;
	if (rd->mac_on)
		fw |= 1;
	wr32(rd, REG_SCOREBOARD, BIT(31) | fw << 24 | (bits & GENMASK(23, 0)));
	fsleep(1000);
}

static void coex_set_ant(struct ax52_dev *rd, u32 gnt)
{
	ax52_lte_write(rd, LTE_SW_CFG_1, gnt);
	set8(rd, REG_SYS_SDIO_CTRL + 3, BIT(2));	/* control path: WL */
	wr16(rd, 0xC67C, 0x0100);			/* PTA: no pollution */
}

/*
 * Put the packet-traffic arbiter (shared with the on-chip BT) into a known
 * state. There is no coex algorithm here: WL simply takes the antennas.
 */
void ax52_coex_init(struct ax52_dev *rd)
{
	static const struct { u8 path; u8 grp; u32 val; } lut[] = {
		{ RF_PATH_A, 0, 0x5FF }, { RF_PATH_B, 0, 0x5FF },
		{ RF_PATH_A, 2, 0x5FF }, { RF_PATH_B, 2, 0x55F },
	};
	u32 v;
	int i;

	set8(rd, REG_GPIO_MUXCFG, BIT(5));		/* BT enable pin */
	set8(rd, 0xDA20, BIT(1));			/* PTA WL TX enable */
	set8(rd, 0xDA35, 0x01);				/* GNT_BT polarity */
	set8(rd, 0xDA40, BIT(2) | BIT(3));
	set8(rd, 0xDA42, 0x01);
	clr8(rd, 0xCC07, 0x02);
	v = rd16(rd, 0xC340);
	wr16(rd, 0xC340, (v | BIT(5)) & ~BIT(9));
	if (!ax52_lte_read(rd, LTE_SW_CFG_2, &v))
		ax52_lte_write(rd, LTE_SW_CFG_2, v & BIT(8));
	mask8(rd, REG_GPIO_MUXCFG, GENMASK(7, 6), 0);	/* RTK BT mode */
	set8(rd, 0xDA4C, BIT(0));
	mask8(rd, 0xDA6C, GENMASK(5, 0), 5);
	v = rd8(rd, REG_GPIO_MUXCFG + 1);
	wr8(rd, REG_GPIO_MUXCFG + 1, (v & ~BIT(2)) | BIT(1));

	/* WL response/beacon frames at high priority */
	set32(rd, 0xDA30, BIT(3));
	set32(rd, 0xDA10, BIT(8));

	/* RF: no GNT debug, TRX mask LUT for the shared antenna */
	ax52_rf_write(rd, RF_PATH_A, 0x02, 0xFFFFF, 0);
	ax52_rf_write(rd, RF_PATH_B, 0x02, 0xFFFFF, 0);
	for (i = 0; i < ARRAY_SIZE(lut); i++) {
		ax52_rf_write(rd, lut[i].path, 0xEF, 0xFFFFF, 0x20000);
		ax52_rf_write(rd, lut[i].path, 0x33, 0xFFFFF, lut[i].grp);
		ax52_rf_write(rd, lut[i].path, 0x3F, 0xFFFFF, lut[i].val);
		ax52_rf_write(rd, lut[i].path, 0xEF, 0xFFFFF, 0);
	}

	wr32(rd, 0xDA2C, 0xF0FFFFFF);			/* break table */
	set32(rd, 0xDA40, BIT(16) | BIT(2));

	coex_set_sb(rd, COEX_SB_ACTIVE | COEX_SB_ON | COEX_SB_BTLOG);
}

/*
 * WL takes the antennas. On 5 GHz BT keeps hardware arbitration of its own
 * 2.4 GHz path; on 2.4 GHz BT loses the shared path while WL is up.
 */
void ax52_coex_wl_only(struct ax52_dev *rd)
{
	coex_set_ant(rd, rd->chan.band == AX52_BAND_5G ? GNT_WL_5G : GNT_WL_ONLY);
}

/* Interface down: hand the RF switch back to BT. */
void ax52_coex_release(struct ax52_dev *rd)
{
	coex_set_sb(rd, 0);
	clr8(rd, REG_SYS_SDIO_CTRL + 3, BIT(2));	/* control path: BT */
	wr16(rd, 0xC67C, 0x0100);
}

void ax52_coex_rfk_begin(struct ax52_dev *rd)
{
	u32 sb;

	/* let a BT-side calibration finish first (bounded) */
	read_poll_timeout(rd32, sb, !(sb & (COEX_SB_BT_RFK_RUN | COEX_SB_BT_RFK_REQ)),
			  1000, 100000, false, rd, REG_SCOREBOARD);
	coex_set_sb(rd, rd->coex_sb | COEX_SB_WLRFK);
	if (ax52_lte_read(rd, LTE_SW_CFG_1, &rd->coex_gnt_saved))
		rd->coex_gnt_saved = GNT_WL_ONLY;
	coex_set_ant(rd, GNT_WL_ONLY);
}

void ax52_coex_rfk_end(struct ax52_dev *rd)
{
	ax52_lte_write(rd, LTE_SW_CFG_1, rd->coex_gnt_saved);
	coex_set_sb(rd, rd->coex_sb & ~COEX_SB_WLRFK);
}

/*
 * MAC setup that must precede firmware download: HCI DMA, the minimal DMAC
 * function set and a packet buffer holding only H2C/C2H.
 */
int ax52_mac_pre_fwdl(struct ax52_dev *rd)
{
	int ret;

	set32(rd, REG_HCI_FUNC_EN, HCI_TXDMA_EN | HCI_RXDMA_EN);

	/* MAC, DMAC, packet buffer and dispatcher only */
	wr32(rd, REG_DMAC_FUNC_EN, 0x60440000);
	wr32(rd, REG_DMAC_CLK_EN, 0x00040000);

	ret = dle_init(rd, &dle_dlfw);
	if (ret)
		return ret;
	hfc_init_h2c_only(rd);

	return ax52_pci_pre_init(rd);
}
