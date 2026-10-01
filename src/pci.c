// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host interface: buffer-descriptor rings, DMA engine bring-up, the
 * firmware-command (H2C) channel, interrupts/NAPI and the frame rings.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>

#include "ax52.h"

/*
 * Per-ring registers. The on-chip BD prefetch RAM (32 entries) is split
 * 5/5/5/5 between the AC rings and 4/4/4 between MGMT, HIQ and FWCMD.
 * bdram = start | max << 8 | min << 16.
 */
static const struct {
	u8 hwch;
	u16 num, idx, bdram, desa;
	u32 bdram_val;
} txring_def[TXQ_NUM] = {
	[TXQ_ACH0]  = {  0, 0x1024, 0x1058, 0x1200, 0x1110, 0x00020500 },
	[TXQ_ACH1]  = {  1, 0x1026, 0x105C, 0x1204, 0x1118, 0x00020505 },
	[TXQ_ACH2]  = {  2, 0x1028, 0x1060, 0x1208, 0x1120, 0x0002050A },
	[TXQ_ACH3]  = {  3, 0x102A, 0x1064, 0x120C, 0x1128, 0x0002050F },
	[TXQ_MGMT]  = {  8, 0x1034, 0x1078, 0x1220, 0x1150, 0x00010414 },
	[TXQ_HIQ]   = {  9, 0x1036, 0x107C, 0x1224, 0x1158, 0x00010418 },
	[TXQ_FWCMD] = { 12, 0x1038, 0x1080, 0x1228, 0x1160, 0x0001041C },
};

static const struct {
	u16 num, idx, desa;
} rxring_def[RXQ_NUM] = {
	[RXQ_DATA] = { REG_RXQ_RXBD_NUM, REG_RXQ_RXBD_IDX, REG_RXQ_RXBD_DESA_L },
	[RXQ_RPQ]  = { REG_RPQ_RXBD_NUM, REG_RPQ_RXBD_IDX, REG_RPQ_RXBD_DESA_L },
};

/* hardware DMA channel -> our ring, for release reports */
static int hwch_to_txq(u8 hwch)
{
	switch (hwch) {
	case 0 ... 3:	return TXQ_ACH0 + hwch;
	case 8:		return TXQ_MGMT;
	case 9:		return TXQ_HIQ;
	default:	return -1;
	}
}

/* release-report queue select -> hardware DMA channel */
static int qsel_to_hwch(u8 qsel)
{
	if (qsel < 0x10)
		return qsel & 3;	/* BE/BK/VI/VO of any WMM set -> ACH0-3 */
	if (qsel == 0x12)
		return 8;
	if (qsel == 0x11)
		return 9;
	return -1;
}

static inline u16 bd_opt_hi(dma_addr_t a)
{
	return FIELD_PREP(BD_OPT_DMA_HI, upper_32_bits(a) & 0xff);
}

static inline u16 ring_dist(u16 from, u16 to)
{
	return (to + TXBD_NUM - from) % TXBD_NUM;
}

/* ------------------------------------------------------ allocation */

static void txring_reset_pages(struct ax52_txring *r)
{
	int i;

	r->nfree = WD_PAGE_NUM;
	for (i = 0; i < WD_PAGE_NUM; i++)
		r->free_page[i] = WD_PAGE_NUM - 1 - i;
}

int ax52_pci_alloc_rings(struct ax52_dev *rd)
{
	struct device *dev = rd->dev;
	int q, i;

	for (q = 0; q < TXQ_NUM; q++) {
		struct ax52_txring *r = &rd->tx[q];

		r->hwch = txring_def[q].hwch;
		r->reg_num = txring_def[q].num;
		r->reg_idx = txring_def[q].idx;
		r->reg_bdram = txring_def[q].bdram;
		r->reg_desa = txring_def[q].desa;
		r->bdram_val = txring_def[q].bdram_val;

		r->bd = dma_alloc_coherent(dev, TXBD_NUM * sizeof(*r->bd),
					   &r->bd_dma, GFP_KERNEL);
		if (!r->bd)
			goto oom;

		if (q == TXQ_FWCMD) {
			r->slot = kcalloc(TXBD_NUM, sizeof(*r->slot), GFP_KERNEL);
			r->slot_dma = kcalloc(TXBD_NUM, sizeof(*r->slot_dma),
					      GFP_KERNEL);
			if (!r->slot || !r->slot_dma)
				goto oom;
			for (i = 0; i < TXBD_NUM; i++) {
				r->slot[i] = dma_alloc_coherent(dev, FWCMD_SLOT_SIZE,
								&r->slot_dma[i],
								GFP_KERNEL);
				if (!r->slot[i])
					goto oom;
			}
			continue;
		}

		r->page = kcalloc(WD_PAGE_NUM, sizeof(*r->page), GFP_KERNEL);
		r->bd2page = kcalloc(TXBD_NUM, sizeof(*r->bd2page), GFP_KERNEL);
		r->free_page = kcalloc(WD_PAGE_NUM, sizeof(*r->free_page),
				       GFP_KERNEL);
		rd->wd[q] = dma_alloc_coherent(dev, WD_PAGE_NUM * WD_PAGE_SIZE,
					       &rd->wd_dma[q], GFP_KERNEL);
		if (!r->page || !r->bd2page || !r->free_page || !rd->wd[q])
			goto oom;
		txring_reset_pages(r);
	}

	for (q = 0; q < RXQ_NUM; q++) {
		struct ax52_rxring *r = &rd->rx[q];

		r->reg_num = rxring_def[q].num;
		r->reg_idx = rxring_def[q].idx;
		r->reg_desa = rxring_def[q].desa;
		r->bd = dma_alloc_coherent(dev, RXBD_NUM * sizeof(*r->bd),
					   &r->bd_dma, GFP_KERNEL);
		if (!r->bd)
			goto oom;

		for (i = 0; i < RXBD_NUM; i++) {
			r->buf[i] = dma_alloc_coherent(dev, RXBUF_SIZE,
						       &r->buf_dma[i], GFP_KERNEL);
			if (!r->buf[i])
				goto oom;
			r->bd[i].len = cpu_to_le16(RXBUF_SIZE);
			r->bd[i].opt = cpu_to_le16(bd_opt_hi(r->buf_dma[i]));
			r->bd[i].dma = cpu_to_le32(lower_32_bits(r->buf_dma[i]));
		}
	}
	return 0;

oom:
	ax52_pci_free_rings(rd);
	return -ENOMEM;
}

void ax52_pci_free_rings(struct ax52_dev *rd)
{
	struct device *dev = rd->dev;
	int q, i;

	for (q = 0; q < RXQ_NUM; q++) {
		struct ax52_rxring *r = &rd->rx[q];

		for (i = 0; i < RXBD_NUM; i++) {
			if (r->buf[i])
				dma_free_coherent(dev, RXBUF_SIZE, r->buf[i],
						  r->buf_dma[i]);
			r->buf[i] = NULL;
		}
		if (r->bd)
			dma_free_coherent(dev, RXBD_NUM * sizeof(*r->bd), r->bd,
					  r->bd_dma);
		r->bd = NULL;
		dev_kfree_skb(r->partial);
		r->partial = NULL;
	}

	for (q = 0; q < TXQ_NUM; q++) {
		struct ax52_txring *r = &rd->tx[q];

		if (r->slot) {
			for (i = 0; i < TXBD_NUM; i++)
				if (r->slot[i])
					dma_free_coherent(dev, FWCMD_SLOT_SIZE,
							  r->slot[i], r->slot_dma[i]);
		}
		kfree(r->slot);
		kfree(r->slot_dma);
		r->slot = NULL;
		r->slot_dma = NULL;

		if (q != TXQ_FWCMD && rd->wd[q]) {
			dma_free_coherent(dev, WD_PAGE_NUM * WD_PAGE_SIZE,
					  rd->wd[q], rd->wd_dma[q]);
			rd->wd[q] = NULL;
		}
		kfree(r->page);
		kfree(r->bd2page);
		kfree(r->free_page);
		r->page = NULL;
		r->bd2page = NULL;
		r->free_page = NULL;

		if (r->bd)
			dma_free_coherent(dev, TXBD_NUM * sizeof(*r->bd), r->bd,
					  r->bd_dma);
		r->bd = NULL;
	}
}

/* Program ring lengths, base addresses and BD-RAM partitioning. */
void ax52_pci_program_rings(struct ax52_dev *rd)
{
	int q;

	for (q = 0; q < TXQ_NUM; q++) {
		struct ax52_txring *r = &rd->tx[q];

		r->wp = 0;
		r->rp = 0;
		wr16(rd, r->reg_num, TXBD_NUM);
		wr32(rd, r->reg_bdram, r->bdram_val);
		wr32(rd, r->reg_desa, lower_32_bits(r->bd_dma));
		wr32(rd, r->reg_desa + 4, upper_32_bits(r->bd_dma));
	}

	for (q = 0; q < RXQ_NUM; q++) {
		struct ax52_rxring *r = &rd->rx[q];

		r->wp = 0;
		wr16(rd, r->reg_num, RXBD_NUM);
		wr32(rd, r->reg_desa, lower_32_bits(r->bd_dma));
		wr32(rd, r->reg_desa + 4, upper_32_bits(r->bd_dma));
	}
}

/* ------------------------------------------------------ PCIe PHY (MDIO) */

static int mdio_access(struct ax52_dev *rd, u8 addr, u8 page, u16 rw)
{
	u16 v;

	wr8(rd, REG_MDIO_CFG, addr & MDIO_ADDR_MASK);
	v = rd16(rd, REG_MDIO_CFG);
	v = (v & ~MDIO_PAGE_MASK) | FIELD_PREP(MDIO_PAGE_MASK, page);
	wr16(rd, REG_MDIO_CFG, v);
	set16(rd, REG_MDIO_CFG, rw);

	return read_poll_timeout(rd16, v, !(v & rw), 10, 2000, false,
				 rd, REG_MDIO_CFG);
}

/* Each speed has its own register bank: page 0/1 (Gen1) or 2/3 (Gen2). */
static u8 mdio_page(u8 addr, bool gen2)
{
	if (gen2)
		return addr < 0x20 ? MDIO_PAGE_G2_LOW : MDIO_PAGE_G2_HIGH;
	return addr < 0x20 ? MDIO_PAGE_G1_LOW : MDIO_PAGE_G1_HIGH;
}

static int mdio_read(struct ax52_dev *rd, u8 addr, bool gen2, u16 *val)
{
	int ret = mdio_access(rd, addr, mdio_page(addr, gen2), MDIO_RFLAG);

	if (!ret)
		*val = rd16(rd, REG_MDIO_RDATA);
	return ret;
}

static int mdio_write(struct ax52_dev *rd, u8 addr, bool gen2, u16 val)
{
	wr16(rd, REG_MDIO_WDATA, val);
	return mdio_access(rd, addr, mdio_page(addr, gen2), MDIO_WFLAG);
}

static int mdio_mask(struct ax52_dev *rd, u8 addr, bool gen2, u16 mask,
		     u16 field)
{
	u16 v;
	int ret = mdio_read(rd, addr, gen2, &v);

	if (ret)
		return ret;
	v = (v & ~mask) | ((field << __ffs(mask)) & mask);
	return mdio_write(rd, addr, gen2, v);
}

/* Disable the PCIe reference-clock auto calibration of the current speed. */
static int refclk_cal_disable(struct ax52_dev *rd)
{
	struct pci_dev *pdev = rd->pdev;
	u8 rate, l1ctrl;
	bool l1_was_on, gen2;
	u16 v;
	int ret;

	pci_read_config_byte(pdev, PCICFG_PHY_RATE, &rate);
	if ((rate & 0x3) != 1 && (rate & 0x3) != 2) {
		/* rtw89 fails the bring-up here; the tweak is not essential */
		ax52_warn(rd, "PCIe link rate code %u, skipping refclk cal tweak\n",
			  rate & 3);
		return 0;
	}
	gen2 = (rate & 0x3) == 2;

	pci_read_config_byte(pdev, PCICFG_L1_CTRL, &l1ctrl);
	l1_was_on = l1ctrl & PCICFG_L1_CTRL_ASPM_L1;
	if (l1_was_on)
		pci_write_config_byte(pdev, PCICFG_L1_CTRL,
				      l1ctrl & ~PCICFG_L1_CTRL_ASPM_L1);

	ret = mdio_read(rd, MDIO_RAC_CTRL_PPR_V1, gen2, &v);
	if (!ret && (v & BAC_CALIB_EN))
		ret = mdio_write(rd, MDIO_RAC_CTRL_PPR_V1, gen2, v & ~BAC_CALIB_EN);

	if (l1_was_on)
		pci_write_config_byte(pdev, PCICFG_L1_CTRL, l1ctrl);
	return ret;
}

/* ------------------------------------------------------ bring-up */

/*
 * Everything the host interface needs before firmware download: PHY
 * tweaks, stop and reset all DMA, program the rings, then re-enable DMA
 * with only the firmware-command channel released.
 */
int ax52_pci_pre_init(struct ax52_dev *rd)
{
	int ret;

	clr32(rd, REG_PCIE_PS_CTRL, PS_L1OFF_PWR_OFF_EN);
	clr32(rd, REG_SYS_PW_CTRL, PW_PSUS_OFF_CAPC_EN);
	set32(rd, REG_SYS_SDIO_CTRL, SDIO_PCIE_DIS_L2_CTRL_LDO_HCI);
	clr32(rd, REG_SYS_SDIO_CTRL, SDIO_PCIE_DIS_WLSUS_AFT_PDN);

	/* these two are always programmed in the Gen1 bank, as rtw89 does */
	mdio_mask(rd, MDIO_RAC_REG_REV2, false, BAC_CMU_EN_DLY_MASK, 1);
	ret = mdio_mask(rd, MDIO_RAC_REG_FLD_0, false, BAC_AUTOK_N_MASK, 3);
	if (ret)
		return ret;
	ret = refclk_cal_disable(rd);
	if (ret)
		return ret;

	set32(rd, REG_HCI_OPT_CTRL, HCI_BIT_WAKE_CTRL);
	clr32(rd, REG_PCIE_EXP_CTRL, EXP_SIC_EN_FORCE_CLKREQ);

	/* LBC watchdog: 2 ms timer, enabled */
	mask32(rd, REG_LBC_WATCHDOG, LBC_TIMER_MASK, 8);
	set32(rd, REG_LBC_WATCHDOG, LBC_FLAG | LBC_EN);

	set32(rd, REG_PCIE_DBG_CTRL, BIT(1) | BIT(0));
	mask32(rd, REG_PCIE_EXP_CTRL, 0x3, 1);
	set32(rd, REG_PCIE_INIT_CFG1, CFG1_TXRST_KEEP_REG | CFG1_RXRST_KEEP_REG);

	/* stop everything and wait for the engines to go idle */
	set32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_WPDMA);
	set32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_PCIEIO);
	clr32(rd, REG_PCIE_INIT_CFG1, CFG1_RXHCI_EN | CFG1_TXHCI_EN);
	ret = ax52_poll32(rd, REG_PCIE_DMA_BUSY1, DMA_BUSY_TXCH_MASK, 0, 10, 100);
	if (!ret)
		ret = ax52_poll32(rd, REG_PCIE_DMA_BUSY1, DMA_BUSY_RXCH_MASK, 0,
				  10, 100);
	if (ret) {
		ax52_err(rd, "DMA did not go idle (busy=0x%08x)\n",
			 rd32(rd, REG_PCIE_DMA_BUSY1));
		return ret;
	}

	set32(rd, REG_TXBD_RWPTR_CLR1, TXBD_CLR_ALL);
	set32(rd, REG_RXBD_RWPTR_CLR, RXBD_CLR_ALL);

	/* packet-mode RX, 2 KiB TX / 128 B RX bursts, 8 outstanding tags */
	clr32(rd, REG_PCIE_INIT_CFG1, CFG1_RXBD_MODE);
	mask32(rd, REG_PCIE_INIT_CFG1, CFG1_MAX_TXDMA_MASK, 7);
	mask32(rd, REG_PCIE_INIT_CFG1, CFG1_MAX_RXDMA_MASK, 3);
	set32(rd, REG_PCIE_INIT_CFG1, CFG1_LATENCY_CONTROL);
	mask32(rd, REG_PCIE_EXP_CTRL, EXP_MAX_TAG_NUM_MASK, 7);
	mask32(rd, REG_PCIE_INIT_CFG2, CFG2_WD_ITVL_IDLE_MASK, 1);
	mask32(rd, REG_PCIE_INIT_CFG2, CFG2_WD_ITVL_ACT_MASK, 1);
	set32(rd, REG_TX_ADDR_INFO_MODE, HOST_ADDR_INFO_8B_SEL);
	clr32(rd, REG_PKTIN_SETTING, PKTIN_WD_ADDR_INFO_LENGTH);

	ax52_pci_program_rings(rd);

	set32(rd, REG_PCIE_INIT_CFG1, CFG1_RST_BDRAM);
	ret = ax52_poll32(rd, REG_PCIE_INIT_CFG1, CFG1_RST_BDRAM, 0, 1, 100);
	if (ret) {
		ax52_err(rd, "BD RAM reset stuck\n");
		return ret;
	}

	/* only the firmware-command channel runs until MAC init is done */
	set32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_TXCH_MASK);
	clr32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_CH12);
	clr32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_PCIEIO);
	set32(rd, REG_PCIE_INIT_CFG1, CFG1_RXHCI_EN | CFG1_TXHCI_EN);
	return 0;
}

/* After MAC init: release all TX channels and the WD-page DMA engine. */
void ax52_pci_post_init(struct ax52_dev *rd)
{
	set32(rd, REG_TX_ADDR_INFO_MODE, HOST_ADDR_INFO_8B_SEL);
	clr32(rd, REG_PKTIN_SETTING, PKTIN_WD_ADDR_INFO_LENGTH);
	clr32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_TXCH_MASK);
	clr32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_WPDMA | DMA_STOP_PCIEIO);
}

/* Quiesce DMA before power-off / teardown. */
void ax52_pci_deinit(struct ax52_dev *rd)
{
	clr32(rd, REG_HCI_OPT_CTRL, HCI_BIT_WAKE_CTRL);
	set32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_TXCH_MASK | DMA_STOP_WPDMA);
	set32(rd, REG_PCIE_DMA_STOP1, DMA_STOP_PCIEIO);
	clr32(rd, REG_PCIE_INIT_CFG1, CFG1_RXHCI_EN | CFG1_TXHCI_EN);
	if (ax52_poll32(rd, REG_PCIE_DMA_BUSY1,
			DMA_BUSY_TXCH_MASK | DMA_BUSY_RXCH_MASK, 0, 10, 1000))
		ax52_warn(rd, "DMA busy at deinit: 0x%08x\n",
			  rd32(rd, REG_PCIE_DMA_BUSY1));
	set32(rd, REG_TXBD_RWPTR_CLR1, TXBD_CLR_ALL);
	set32(rd, REG_RXBD_RWPTR_CLR, RXBD_CLR_ALL);
}

/* ------------------------------------------------------ FWCMD channel */

static u16 txring_avail(const struct ax52_txring *r)
{
	return TXBD_NUM - 1 - ring_dist(r->rp, r->wp);
}

static u16 txring_hw_idx(struct ax52_dev *rd, struct ax52_txring *r)
{
	return rd32_mask(rd, r->reg_idx, BD_HW_IDX_MASK);
}

/*
 * Queue one packet on the firmware-command ring: a 24-byte WD body, an
 * optional header (the 8-byte H2C header), then the payload.
 */
int ax52_fwcmd_tx(struct ax52_dev *rd, const void *hdr, u32 hdr_len,
		  const void *data, u32 len, bool fwdl)
{
	struct ax52_txring *r = &rd->tx[TXQ_FWCMD];
	u32 pkt = hdr_len + len;
	unsigned long timeout;
	__le32 *wd;
	u8 *p;

	if (WARN_ON(24 + pkt > FWCMD_SLOT_SIZE))
		return -EINVAL;
	if (!rd->mac_on)
		return -ENODEV;

	lockdep_assert_held(&rd->h2c_lock);

	/* keep FWCMD_KEEP recently consumed slots untouched (read-ahead) */
	timeout = jiffies + msecs_to_jiffies(100);
	for (;;) {
		r->rp = txring_hw_idx(rd, r);
		if (txring_avail(r) > FWCMD_KEEP)
			break;
		if (time_after(jiffies, timeout)) {
			ax52_err(rd, "FWCMD ring stuck (wp %u rp %u)\n", r->wp, r->rp);
			/* later H2Cs fail at once instead of waiting here too */
			ax52_fw_failed(rd);
			return -EBUSY;
		}
		udelay(10);
	}

	p = r->slot[r->wp];
	wd = (__le32 *)p;
	memset(wd, 0, 24);
	wd[0] = cpu_to_le32(FIELD_PREP(GENMASK(19, 16), r->hwch) |
			    (fwdl ? BIT(20) : 0));
	wd[2] = cpu_to_le32(FIELD_PREP(GENMASK(13, 0), pkt));
	if (hdr_len)
		memcpy(p + 24, hdr, hdr_len);
	memcpy(p + 24 + hdr_len, data, len);

	r->bd[r->wp].len = cpu_to_le16(24 + pkt);
	r->bd[r->wp].opt = cpu_to_le16(TXBD_OPT_LS | bd_opt_hi(r->slot_dma[r->wp]));
	r->bd[r->wp].dma = cpu_to_le32(lower_32_bits(r->slot_dma[r->wp]));

	r->wp = (r->wp + 1) % TXBD_NUM;
	dma_wmb();
	wr16(rd, r->reg_idx, r->wp);
	return 0;
}

/* ------------------------------------------------------ frame TX */

static void page_free(struct ax52_dev *rd, enum ax52_txq q, u16 idx)
{
	struct ax52_txring *r = &rd->tx[q];

	if (WARN_ON_ONCE(r->nfree >= WD_PAGE_NUM))
		return;
	memset(rd->wd[q] + idx * WD_PAGE_SIZE, 0, WD_PAGE_SIZE);
	r->free_page[r->nfree++] = idx;
}

/* Retire BDs the DMA engine has consumed. Called with tx_lock held. */
static void txring_reclaim_bd(struct ax52_dev *rd, enum ax52_txq q)
{
	struct ax52_txring *r = &rd->tx[q];
	u16 hw = txring_hw_idx(rd, r);

	/*
	 * Trust the index only within the BDs in flight: an all-ones read
	 * (device gone) or a ring cleared under us must not retire BDs, and
	 * free their pages, that were never submitted.
	 */
	if (hw >= TXBD_NUM || ring_dist(r->rp, hw) > ring_dist(r->rp, r->wp))
		return;
	while (r->rp != hw) {
		u16 idx = r->bd2page[r->rp];
		struct ax52_wd_page *pg = &r->page[idx];

		pg->bd_busy = false;
		if (!pg->skb)		/* release report already came */
			page_free(rd, q, idx);
		r->rp = (r->rp + 1) % TXBD_NUM;
	}
}

int ax52_pci_tx_avail(struct ax52_dev *rd, enum ax52_txq q)
{
	struct ax52_txring *r = &rd->tx[q];
	int n;

	spin_lock_bh(&rd->tx_lock);
	txring_reclaim_bd(rd, q);
	n = min_t(int, txring_avail(r), r->nfree);
	spin_unlock_bh(&rd->tx_lock);
	return n;
}

int ax52_pci_tx(struct ax52_dev *rd, enum ax52_txq q, const __le32 *wd,
		struct sk_buff *skb)
{
	struct ax52_txring *r = &rd->tx[q];
	struct ax52_wd_page *pg;
	dma_addr_t dma, pdma;
	__le16 *wp;
	u8 *page;
	u16 idx;

	dma = dma_map_single(rd->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(rd->dev, dma))
		return -ENOMEM;

	spin_lock_bh(&rd->tx_lock);
	if (!r->nfree || !txring_avail(r))
		txring_reclaim_bd(rd, q);
	if (!r->nfree || !txring_avail(r)) {
		spin_unlock_bh(&rd->tx_lock);
		dma_unmap_single(rd->dev, dma, skb->len, DMA_TO_DEVICE);
		return -ENOSPC;
	}

	idx = r->free_page[--r->nfree];
	pg = &r->page[idx];
	pg->skb = skb;
	pg->skb_dma = dma;
	pg->ts = jiffies;
	pg->bd_busy = true;

	/* WD body + info, then the page-sequence tag and one address entry */
	page = rd->wd[q] + idx * WD_PAGE_SIZE;
	pdma = rd->wd_dma[q] + idx * WD_PAGE_SIZE;
	memcpy(page, wd, 48);
	wp = (__le16 *)(page + 48);
	wp[0] = cpu_to_le16(idx | BIT(15));
	wp[1] = wp[2] = wp[3] = 0;
	*(__le16 *)(page + 56) = cpu_to_le16(skb->len);
	*(__le16 *)(page + 58) = cpu_to_le16(BIT(15) | bd_opt_hi(dma) | 1);
	*(__le32 *)(page + 60) = cpu_to_le32(lower_32_bits(dma));

	r->bd2page[r->wp] = idx;
	r->bd[r->wp].len = cpu_to_le16(64);
	r->bd[r->wp].opt = cpu_to_le16(TXBD_OPT_LS | bd_opt_hi(pdma));
	r->bd[r->wp].dma = cpu_to_le32(lower_32_bits(pdma));
	r->wp = (r->wp + 1) % TXBD_NUM;
	spin_unlock_bh(&rd->tx_lock);
	return 0;
}

void ax52_pci_tx_kick(struct ax52_dev *rd, enum ax52_txq q)
{
	struct ax52_txring *r = &rd->tx[q];

	spin_lock_bh(&rd->tx_lock);
	dma_wmb();
	wr16(rd, r->reg_idx, r->wp);
	spin_unlock_bh(&rd->tx_lock);
}

#define FLUSH_FETCH_US		20000	/* per ring */

/* Wait until the hardware has fetched every BD of the frame rings. Sleeps. */
void ax52_pci_flush(struct ax52_dev *rd)
{
	int q;
	u16 hw;

	for (q = 0; q < TXQ_FWCMD; q++) {
		struct ax52_txring *r = &rd->tx[q];

		if (read_poll_timeout(txring_hw_idx, hw,
				      hw == READ_ONCE(r->wp) || hw >= TXBD_NUM,
				      50, FLUSH_FETCH_US, false, rd, r))
			ax52_dbg(rd, "flush: ring %d not fetched (hw %u wp %u)\n",
				 q, hw, READ_ONCE(r->wp));
	}
}

/* Complete every frame still owned by the rings (used at stop). */
void ax52_pci_reset(struct ax52_dev *rd)
{
	struct sk_buff_head done;
	struct sk_buff *skb;
	int q, i;

	__skb_queue_head_init(&done);
	spin_lock_bh(&rd->tx_lock);
	for (q = 0; q < TXQ_FWCMD; q++) {
		struct ax52_txring *r = &rd->tx[q];

		for (i = 0; i < WD_PAGE_NUM; i++) {
			struct ax52_wd_page *pg = &r->page[i];

			if (!pg->skb)
				continue;
			dma_unmap_single(rd->dev, pg->skb_dma, pg->skb->len,
					 DMA_TO_DEVICE);
			__skb_queue_tail(&done, pg->skb);
			pg->skb = NULL;
		}
		memset(r->page, 0, WD_PAGE_NUM * sizeof(*r->page));
		memset(rd->wd[q], 0, WD_PAGE_NUM * WD_PAGE_SIZE);
		txring_reset_pages(r);
		r->wp = r->rp = 0;
	}
	spin_unlock_bh(&rd->tx_lock);

	while ((skb = __skb_dequeue(&done)))
		ax52_tx_status(rd, skb, 3);	/* "MAC ID drop" */

	for (q = 0; q < RXQ_NUM; q++) {
		dev_kfree_skb(rd->rx[q].partial);
		rd->rx[q].partial = NULL;
		rd->rx[q].wp = 0;
	}
	__skb_queue_purge(&rd->ppdu_q);	/* radio is down: drop, don't deliver */
}

/* ------------------------------------------------------ release reports */

#define RPP_SEQ		GENMASK(30, 16)
#define RPP_STATUS	GENMASK(15, 13)
#define RPP_QSEL	GENMASK(12, 8)

static void rpp_release(struct ax52_dev *rd, u32 rpp, struct sk_buff_head *done)
{
	u16 seq = FIELD_GET(RPP_SEQ, rpp);
	int hwch = qsel_to_hwch(FIELD_GET(RPP_QSEL, rpp));
	int q = hwch < 0 ? -1 : hwch_to_txq(hwch);
	struct ax52_txring *r;
	struct ax52_wd_page *pg;
	struct sk_buff *skb;

	if (q < 0 || seq >= WD_PAGE_NUM) {
		ax52_dbg(rd, "bogus release report 0x%08x\n", rpp);
		return;
	}
	r = &rd->tx[q];
	pg = &r->page[seq];
	if (pg->bd_busy)
		txring_reclaim_bd(rd, q);

	skb = pg->skb;
	if (!skb)
		return;
	pg->skb = NULL;
	dma_unmap_single(rd->dev, pg->skb_dma, skb->len, DMA_TO_DEVICE);
	/* stash the status in cb scratch space until we drop the lock */
	skb->priority = FIELD_GET(RPP_STATUS, rpp);
	__skb_queue_tail(done, skb);
	if (!pg->bd_busy)
		page_free(rd, q, seq);
}

static int rpq_poll(struct ax52_dev *rd)
{
	struct ax52_rxring *r = &rd->rx[RXQ_RPQ];
	struct sk_buff_head done;
	struct sk_buff *skb;
	u16 hw, cnt;
	int n = 0;

	__skb_queue_head_init(&done);
	spin_lock_bh(&rd->tx_lock);
	hw = rd32_mask(rd, r->reg_idx, BD_HW_IDX_MASK);
	cnt = hw < RXBD_NUM ? (hw + RXBD_NUM - r->wp) % RXBD_NUM : 0;
	while (cnt--) {
		const u8 *buf = r->buf[r->wp];
		u32 info, wlen, off, pkt_len;
		u8 type;

		dma_rmb();
		info = le32_to_cpu(*(const __le32 *)buf);
		wlen = info & GENMASK(13, 0);
		if ((info & (BIT(15) | BIT(14))) == (BIT(15) | BIT(14)) &&
		    wlen > 4 && wlen <= RXBUF_SIZE) {
			off = 4 + ax52_rx_payload_offset(buf + 4, &pkt_len, &type);
			for (; off + 4 <= wlen; off += 4)
				rpp_release(rd, le32_to_cpu(*(const __le32 *)(buf + off)),
					    &done);
		}
		r->wp = (r->wp + 1) % RXBD_NUM;
		n++;
	}
	if (n)
		wr16(rd, r->reg_idx, r->wp);
	spin_unlock_bh(&rd->tx_lock);

	while ((skb = __skb_dequeue(&done)))
		ax52_tx_status(rd, skb, skb->priority);
	if (n)
		ax52_tx_resources_freed(rd);
	return n;
}

/* Hand the release reports already in the ring to mac80211 now (flush). */
void ax52_pci_tx_complete(struct ax52_dev *rd)
{
	rpq_poll(rd);
}

#define TX_ORPHAN_AGE		(2 * HZ)

/*
 * A frame whose BD was fetched long ago, while the packet engine holds no
 * frames, will not get a release report any more: complete it as dropped,
 * so a lost report does not pin its WD page and DMA mapping until the
 * radio stops. Process context, every 2 s.
 */
void ax52_pci_tx_reap(struct ax52_dev *rd)
{
	struct sk_buff_head done;
	struct sk_buff *skb;
	int q, i, n = 0;

	if (!ax52_mac_txq_empty(rd))
		return;
	rpq_poll(rd);	/* reports that did arrive win */

	__skb_queue_head_init(&done);
	spin_lock_bh(&rd->tx_lock);
	for (q = 0; q < TXQ_FWCMD; q++) {
		struct ax52_txring *r = &rd->tx[q];

		txring_reclaim_bd(rd, q);
		for (i = 0; i < WD_PAGE_NUM; i++) {
			struct ax52_wd_page *pg = &r->page[i];

			if (!pg->skb || pg->bd_busy ||
			    time_before(jiffies, pg->ts + TX_ORPHAN_AGE))
				continue;
			dma_unmap_single(rd->dev, pg->skb_dma, pg->skb->len,
					 DMA_TO_DEVICE);
			__skb_queue_tail(&done, pg->skb);
			pg->skb = NULL;
			page_free(rd, q, i);
			n++;
		}
	}
	spin_unlock_bh(&rd->tx_lock);

	if (!n)
		return;
	dev_warn_ratelimited(rd->dev, "%d TX frames got no release report, dropped\n",
			     n);
	while ((skb = __skb_dequeue(&done)))
		ax52_tx_status(rd, skb, 3);	/* "MAC ID drop" */
	ax52_tx_resources_freed(rd);
}

/* ------------------------------------------------------ frame RX */

static int rxq_poll(struct ax52_dev *rd, int budget)
{
	struct ax52_rxring *r = &rd->rx[RXQ_DATA];
	u16 hw, cnt;
	int done = 0;

	hw = rd32_mask(rd, r->reg_idx, BD_HW_IDX_MASK);
	cnt = hw < RXBD_NUM ? (hw + RXBD_NUM - r->wp) % RXBD_NUM : 0;

	while (cnt-- && done < budget) {
		const u8 *buf = r->buf[r->wp];
		u32 info, wlen, off, pkt_len;
		bool fs, ls;
		u8 type;

		dma_rmb();
		info = le32_to_cpu(*(const __le32 *)buf);
		wlen = info & GENMASK(13, 0);
		fs = info & BIT(15);
		ls = info & BIT(14);

		if (wlen < 4 || wlen > RXBUF_SIZE) {	/* never from sane HW */
			dev_kfree_skb_any(r->partial);
			r->partial = NULL;
			goto next;
		}

		if (fs) {
			if (r->partial) {
				dev_kfree_skb_any(r->partial);
				r->partial = NULL;
			}
			off = 4 + ax52_rx_payload_offset(buf + 4, &pkt_len, &type);
			if (off > wlen || !pkt_len || pkt_len > RXBUF_SIZE * 2)
				goto next;
			/* keep the RX descriptor in front: rx.c parses it */
			r->partial = dev_alloc_skb(off - 4 + pkt_len);
			if (!r->partial)
				goto next;
			r->partial_len = off - 4 + pkt_len;
			skb_put_data(r->partial, buf + 4,
				     min(wlen, off + pkt_len) - 4);
		} else if (r->partial) {
			u32 n = min_t(u32, wlen - 4,
				      r->partial_len - r->partial->len);

			skb_put_data(r->partial, buf + 4, n);
		}

		if (ls && r->partial) {
			struct sk_buff *skb = r->partial;

			r->partial = NULL;
			if (skb->len == r->partial_len) {
				ax52_rx_packet(rd, skb);
				done++;
			} else {
				dev_kfree_skb_any(skb);
			}
		}
next:
		r->wp = (r->wp + 1) % RXBD_NUM;
	}
	wr16(rd, r->reg_idx, r->wp);
	return done;
}

/* ------------------------------------------------------ interrupts */

#define IMR00_NORMAL	(HI00_RXDMA | HI00_RXP1DMA | HI00_RPQDMA | \
			 HI00_TXDMA_STUCK | HI00_RXDMA_STUCK | HI00_RDU | \
			 HI00_RPQBD_FULL | HI00_HS0ISR_IND)

static void intr_enable(struct ax52_dev *rd)
{
	wr32(rd, REG_HIMR0, HIMR0_HALT_C2H);
	wr32(rd, REG_PCIE_HIMR00, IMR00_NORMAL);
	wr32(rd, REG_PCIE_HIMR10, HI10_HC10ISR_IND);
}

static void intr_disable(struct ax52_dev *rd)
{
	wr32(rd, REG_HIMR0, 0);
	wr32(rd, REG_PCIE_HIMR00, 0);
	wr32(rd, REG_PCIE_HIMR10, 0);
}

static irqreturn_t ax52_irq(int irq, void *data)
{
	struct ax52_dev *rd = data;
	u32 halt, isr0, isr1;

	spin_lock(&rd->irq_lock);
	if (!rd->running) {
		spin_unlock(&rd->irq_lock);
		return IRQ_NONE;
	}

	isr0 = rd32(rd, REG_PCIE_HISR00);
	if (isr0 == 0xffffffff) {	/* device gone: not ours to handle */
		spin_unlock(&rd->irq_lock);
		return IRQ_NONE;
	}
	isr0 &= IMR00_NORMAL;
	halt = rd32(rd, REG_HISR0) & HIMR0_HALT_C2H;
	isr1 = rd32(rd, REG_PCIE_HISR10) & HI10_HC10ISR_IND;
	if (!(halt | isr0 | isr1)) {	/* another device on a shared line */
		spin_unlock(&rd->irq_lock);
		return IRQ_NONE;
	}

	/* masked until NAPI is done; bits set from here on fire again */
	intr_disable(rd);
	wr32(rd, REG_HISR0, halt);
	wr32(rd, REG_PCIE_HISR00, isr0);
	wr32(rd, REG_PCIE_HISR10, isr1);
	spin_unlock(&rd->irq_lock);

	if (halt) {
		dev_err_ratelimited(rd->dev, "firmware halted (reason 0x%08x, UDM0 0x%08x)\n",
				    rd32(rd, REG_HALT_C2H), rd32(rd, REG_UDM0));
		ax52_fw_failed(rd);
	}
	/* rtw89 does not treat these as fatal either */
	if (isr0 & (HI00_TXDMA_STUCK | HI00_RXDMA_STUCK))
		dev_warn_ratelimited(rd->dev, "DMA stuck, isr 0x%08x\n", isr0);

	napi_schedule(&rd->napi);
	return IRQ_HANDLED;
}

static int ax52_napi_poll(struct napi_struct *napi, int budget)
{
	struct ax52_dev *rd = container_of(napi, struct ax52_dev, napi);
	unsigned long flags;
	bool waiting;
	int done;

	wr32(rd, REG_PCIE_HISR00, HI00_RPQDMA | HI00_RPQBD_FULL);
	rpq_poll(rd);

	wr32(rd, REG_PCIE_HISR00, HI00_RXDMA | HI00_RXP1DMA | HI00_RDU);
	done = rxq_poll(rd, budget);
	waiting = ax52_rx_ppdu_expire(rd);

	if (done < budget && napi_complete_done(napi, done)) {
		spin_lock_irqsave(&rd->irq_lock, flags);
		if (rd->running)
			intr_enable(rd);
		spin_unlock_irqrestore(&rd->irq_lock, flags);
		/* poll again if no interrupt brings the PPDU status they wait for */
		if (waiting)
			mod_timer(&rd->ppdu_timer, jiffies + AX52_PPDU_WAIT);
	}
	return done;
}

static void ppdu_timer_fn(struct timer_list *t)
{
	struct ax52_dev *rd = timer_container_of(rd, t, ppdu_timer);

	if (READ_ONCE(rd->running))
		napi_schedule(&rd->napi);
}

int ax52_pci_irq_init(struct ax52_dev *rd)
{
	int ret;

	rd->napi_dev = alloc_netdev_dummy(0);
	if (!rd->napi_dev)
		return -ENOMEM;
	netif_napi_add(rd->napi_dev, &rd->napi, ax52_napi_poll);
	timer_setup(&rd->ppdu_timer, ppdu_timer_fn, 0);

	ret = pci_alloc_irq_vectors(rd->pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_INTX);
	if (ret < 0)
		goto err_napi;
	ret = request_irq(pci_irq_vector(rd->pdev, 0), ax52_irq, IRQF_SHARED,
			  DRV_NAME, rd);
	if (ret)
		goto err_vec;
	return 0;

err_vec:
	pci_free_irq_vectors(rd->pdev);
err_napi:
	netif_napi_del(&rd->napi);
	free_netdev(rd->napi_dev);
	rd->napi_dev = NULL;
	return ret;
}

void ax52_pci_irq_deinit(struct ax52_dev *rd)
{
	if (!rd->napi_dev)
		return;
	free_irq(pci_irq_vector(rd->pdev, 0), rd);
	timer_shutdown_sync(&rd->ppdu_timer);
	pci_free_irq_vectors(rd->pdev);
	netif_napi_del(&rd->napi);
	free_netdev(rd->napi_dev);
	rd->napi_dev = NULL;
}

void ax52_pci_start(struct ax52_dev *rd)
{
	unsigned long flags;

	napi_enable(&rd->napi);
	spin_lock_irqsave(&rd->irq_lock, flags);
	WRITE_ONCE(rd->running, true);
	intr_enable(rd);
	spin_unlock_irqrestore(&rd->irq_lock, flags);
}

void ax52_pci_stop(struct ax52_dev *rd)
{
	unsigned long flags;

	spin_lock_irqsave(&rd->irq_lock, flags);
	WRITE_ONCE(rd->running, false);
	intr_disable(rd);
	spin_unlock_irqrestore(&rd->irq_lock, flags);
	synchronize_irq(pci_irq_vector(rd->pdev, 0));
	napi_synchronize(&rd->napi);
	napi_disable(&rd->napi);
	/* after napi_disable: a late timer can no longer schedule a poll */
	timer_delete_sync(&rd->ppdu_timer);
}
