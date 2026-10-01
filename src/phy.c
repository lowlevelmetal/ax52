// SPDX-License-Identifier: GPL-2.0
/*
 * Baseband (BB) and RF register access, the BB/RF parameter tables, PHY
 * dynamic-mechanism init and channel switching.
 *
 * BB registers live in a 64 KiB window at BAR offset 0x10000. RF registers
 * are 20 bits wide, one set per path: the "A-die" registers (0x00-0xff) sit
 * behind a serial interface (SWSI) driven through BB registers, the "D-die"
 * ones (address bit 16 set) are mapped straight into the BB window.
 */
#include <linux/delay.h>
#include <linux/slab.h>

#include "phy.h"

#define BB_BASE			0x10000
#define PATH_OFS(p)		((p) << 13)	/* per-path BB blocks: +0x2000 */

/* BB registers (BB-relative) */
#define BB_SWSI_DATA		0x0370
#define   SWSI_ADDR		GENMASK(27, 20)
#define   SWSI_PATH		GENMASK(30, 28)
#define   SWSI_BIT_MASK_EN	BIT(31)
#define BB_SWSI_BIT_MASK	0x0374
#define BB_SWSI_READ_ADDR	0x0378	/* [10:8] path, [7:0] address */
#define BB_SWSI_STS		0x174C
#define   SWSI_W_BUSY		BIT(24)
#define   SWSI_R_BUSY		BIT(25)
#define   SWSI_R_DONE		BIT(26)
#define BB_UPD_CLK_ADC		0x0700
#define   ENABLE_CCK		BIT(5)
#define BB_RSTB_ASYNC		0x0704
#define   RSTB_ASYNC_ALL	BIT(1)
#define BB_MAC_PIN_SEL		0x0734
#define   CH_IDX_SEG0		GENMASK(23, 16)
#define BB_PLCP_HISTOGRAM	0x0738
#define   STS_DIS_TRIG_BY_FAIL	BIT(3)
#define   STS_DIS_TRIG_BY_BRK	BIT(2)
#define BB_PHY_STS_BITMAP	0x073C	/* IE enables, one dword per PPDU kind */
#define BB_PD_CTRL		0x0C3C
#define   PD_HIT_DIS		BIT(9)
#define BB_GNT_BT_WGT		0x0C6C
#define   GNT_BT_WGT_EN		BIT(21)
#define BB_RX_MCS_LIMIT		0x0D18
#define   RXHT_MCS_LIMIT	GENMASK(9, 8)
#define   RXVHT_MCS_LIMIT	GENMASK(22, 21)
#define BB_RXHE			0x0D80
#define   RXHE_USER_MAX		GENMASK(13, 6)
#define   RXHE_MAX_NSS		GENMASK(16, 14)
#define   RXHETB_MAX_NSS	GENMASK(25, 23)
#define BB_S0_HW_SI_DIS		0x1200
#define   HW_SI_DIS_TRIG	GENMASK(30, 28)
#define BB_P0_RFMODE		0x12AC
#define   RFMODE_ORI_TXRX	GENMASK(31, 4)
#define   RFMODE_ORI_RX		GENMASK(23, 12)
#define BB_P0_RFMODE_FTM_RX	0x12B0
#define BB_P0_TSSI_THERMAL	0x1C10
#define   TSSI_THERMAL		GENMASK(29, 24)
#define BB_ADC_FIFO		0x20FC
#define   ADC_FIFO_RST		GENMASK(31, 24)
#define BB_TXFIR		0x2300	/* 8 dwords */
#define BB_RXCCA		0x2344
#define   RXCCA_DIS		BIT(31)
#define BB_RXSC			0x237C
#define   RXSC_CCK_UPPER	BIT(0)
#define BB_RX_RPL_OFST		0x23AC
#define BB_RXSCOBC		0x23B0
#define BB_RXSCOCCK		0x23B4
#define BB_BT_DYN_DC_EST	0x4420
#define BB_ASSIGN_SBD_OPT	0x4440
#define BB_DCFO_WEIGHT		0x4490
#define   DCFO_OPT_EN		BIT(29)	/* in BB_DCFO_OPT */
#define BB_P0_5MDET		0x46F8
#define BB_P1_5MDET		0x47B8
#define   MDET5_TH		GENMASK(5, 0)
#define   MDET5_SB0		BIT(6)
#define   MDET5_SB2		BIT(8)
#define   MDET5_EN		BIT(12)
#define BB_P0_BAND_SEL		0x4738
#define BB_P1_BAND_SEL		0x4AA4
#define   BAND_SEL_2G		BIT(17)
#define   BT_SHARE		BIT(19)
#define   BTG_PATH		BIT(22)
#define BB_SEG0R_PD		0x4860
#define   PD_LOWER_BOUND	GENMASK(10, 6)
#define   PD_SR_EN		BIT(30)
#define BB_P0_RPL1		0x49B0	/* [7:0] RPL bias, [31:8] sub-channel RPL */
#define BB_P0_RPL2		0x49B4
#define BB_P0_RPL3		0x49B8
#define BB_P1_RPL1		0x4A00	/* [7:0] RSSI bias */
#define BB_P1_RPL2		0x4A04
#define BB_P1_RPL3		0x4A08
#define BB_FC0_BW		0x49C0
#define   FC0_BW_INV		GENMASK(6, 0)
#define   ANT_RX_1RCCA_SEG0	GENMASK(17, 14)
#define   ANT_RX_1RCCA_SEG1	GENMASK(21, 18)
#define   ANT_RX_BT_SEG0	GENMASK(25, 22)
#define   FC0_BW_SET		GENMASK(31, 30)
#define BB_CHBW_MOD		0x49C4
#define   ANT_RX_SEG0		GENMASK(3, 0)
#define   CHBW_PRICH		GENMASK(11, 8)
#define   CHBW_SBW		GENMASK(13, 12)
#define   CHBW_BT_SHARE		BIT(14)
#define BB_BMODE_PDTH		0x4B64
#define   BMODE_PDTH_LOWER	GENMASK(31, 24)
#define BB_BMODE_PDTH_EN	0x4B74
#define   BMODE_PDTH_LIMIT_EN	BIT(30)
#define BB_P0_TSSI_TRK		0x5818
#define   TSSI_TRK_EN		BIT(30)
#define BB_P0_TSSI_AVG		0x5820
#define   TSSI_EN		BIT(31)
#define BB_P0_TXPW_RSTB		0x58DC
#define   TXPW_RSTB_MANON	BIT(30)
#define   TXPW_RSTB_TSSI	BIT(31)

/* MAC registers */
#define REG_WMAC_RFMOD		0xC010
#define REG_TX_SUB_CARRIER	0xC088
#define REG_TXRATE_CHK		0xC628
#define   TXRATE_CHECK_CCK	BIT(0)
#define   TXRATE_RTS_OFDM6	BIT(1)
#define   TXRATE_BAND_2G	BIT(4)
#define   PWR_UL_CFO_MASK	GENMASK(2, 0)	/* in REG_PWR_UL_CTRL2 */
#define REG_PWR_MACID_LMT	0xD36C	/* 128 dwords */

/* RF registers */
#define RF_MASK			0xFFFFF
#define RF_INV			0xFFFFFFFF	/* failed read */
#define RF_DDIE			BIT(16)		/* direct-mapped register */
#define RR_CFGCH		0x18
#define   RR_CFGCH_CH		GENMASK(7, 0)
#define   RR_CFGCH_BAND0	GENMASK(9, 8)
#define   RR_CFGCH_BW		GENMASK(11, 10)
#define   RR_CFGCH_BW2		BIT(12)
#define   RR_CFGCH_BCN		BIT(13)
#define   RR_CFGCH_TRX_AH	BIT(14)
#define   RR_CFGCH_POW_LCK	BIT(15)
#define   RR_CFGCH_BAND1	GENMASK(17, 16)
#define RR_LUTWA		0x33
#define RR_LUTWD0		0x3F
#define RR_TM			0x42
#define   RR_TM_VAL		GENMASK(6, 1)
#define   RR_TM_TRIG		BIT(19)
#define RR_TM2			0x43
#define   RR_TM2_OFF		GENMASK(19, 16)
#define RR_BIASA		0x60
#define   RR_BIASA_TXG		GENMASK(15, 12)
#define   RR_BIASA_TXA		GENMASK(19, 16)
#define RR_LDO			0xB1
#define   RR_LDO_SEL		GENMASK(8, 6)
#define RR_LPF			0xB7
#define   RR_LPF_BUSY		BIT(8)
#define RR_LCKST		0xCF
#define   RR_LCKST_BIN		BIT(0)
#define RR_LUTWE2		0xEE
#define   RR_LUTWE2_RTXBW	BIT(2)

/* sub-band -> BB gain table band / efuse RX gain offset index */
static const u8 sb_gain_band[AX52_SB_NUM] = {
	[AX52_SB_2G] = 0, [AX52_SB_5G_1] = 1, [AX52_SB_5G_3] = 2, [AX52_SB_5G_4] = 3,
};
static const u8 sb_ofst_band[AX52_SB_NUM] = {
	[AX52_SB_2G] = 1, [AX52_SB_5G_1] = 2, [AX52_SB_5G_3] = 3, [AX52_SB_5G_4] = 4,
};

/* ------------------------------------------------------------ BB access */

u32 ax52_bb_read(struct ax52_dev *rd, u32 addr)
{
	return rd32(rd, BB_BASE + addr);
}

void ax52_bb_write(struct ax52_dev *rd, u32 addr, u32 val)
{
	wr32(rd, BB_BASE + addr, val);
}

u32 ax52_bb_read_mask(struct ax52_dev *rd, u32 addr, u32 mask)
{
	return rd32_mask(rd, BB_BASE + addr, mask);
}

void ax52_bb_write_mask(struct ax52_dev *rd, u32 addr, u32 mask, u32 val)
{
	mask32(rd, BB_BASE + addr, mask, val);
}

static inline void bb_mask(struct ax52_dev *rd, u32 a, u32 mask, u32 v)
{
	mask32(rd, BB_BASE + a, mask, v);
}

static inline void bb_set(struct ax52_dev *rd, u32 a, u32 bits)
{
	set32(rd, BB_BASE + a, bits);
}

static inline void bb_clr(struct ax52_dev *rd, u32 a, u32 bits)
{
	clr32(rd, BB_BASE + a, bits);
}

/* ------------------------------------------------------------ RF access */

static u32 rf_direct_addr(u8 path, u32 addr)
{
	return (path == RF_PATH_A ? 0xE000 : 0xF000) + ((addr & 0xFF) << 2);
}

static int swsi_wait_idle(struct ax52_dev *rd)
{
	u32 v;

	return read_poll_timeout_atomic(ax52_bb_read, v,
					!(v & (SWSI_W_BUSY | SWSI_R_BUSY)),
					1, 30, false, rd, BB_SWSI_STS);
}

/* Returns 0xffffffff if the register cannot be read. */
u32 ax52_rf_read(struct ax52_dev *rd, u8 path, u32 addr, u32 mask)
{
	u32 v;

	if (path >= RF_PATH_NUM) {
		ax52_err(rd, "RF read from invalid path %u\n", path);
		return RF_INV;
	}
	mask &= RF_MASK;
	if (addr & RF_DDIE)
		return ax52_bb_read_mask(rd, rf_direct_addr(path, addr), mask);

	if (swsi_wait_idle(rd)) {
		ax52_err(rd, "RF read: serial interface busy\n");
		return RF_INV;
	}
	bb_mask(rd, BB_SWSI_READ_ADDR, GENMASK(10, 0), (path << 8) | (addr & 0xFF));
	udelay(2);
	if (read_poll_timeout_atomic(ax52_bb_read, v, v & SWSI_R_DONE, 1, 30,
				     false, rd, BB_SWSI_STS)) {
		ax52_err(rd, "RF read of %u/0x%02x timed out\n", path, addr);
		return RF_INV;
	}
	return (v & mask) >> __ffs(mask);
}

void ax52_rf_write(struct ax52_dev *rd, u8 path, u32 addr, u32 mask, u32 val)
{
	u32 cmd;

	if (path >= RF_PATH_NUM) {
		ax52_err(rd, "RF write to invalid path %u\n", path);
		return;
	}
	mask &= RF_MASK;
	if (addr & RF_DDIE) {
		ax52_bb_write_mask(rd, rf_direct_addr(path, addr), mask, val);
		udelay(1);
		return;
	}

	if (swsi_wait_idle(rd)) {
		ax52_err(rd, "RF write: serial interface busy\n");
		return;
	}
	cmd = FIELD_PREP(SWSI_PATH, path) | FIELD_PREP(SWSI_ADDR, addr & 0xFF);
	val &= RF_MASK;
	if (mask != RF_MASK) {
		/* the interface merges the field into the register itself */
		bb_mask(rd, BB_SWSI_BIT_MASK, RF_MASK, mask);
		val = (val << __ffs(mask)) & RF_MASK;
		cmd |= SWSI_BIT_MASK_EN;
	}
	ax52_bb_write(rd, BB_SWSI_DATA, cmd | val);
}

/* ------------------------------------------------------ parameter tables */

/* Each entry is {addr, data}; addr[31:28] may carry a condition instead. */
#define TBL_COND(a)		((a) >> 28)
#define TBL_TARGET(a)		((a) & GENMASK(27, 0))	/* rfe[23:16] cv[7:0] */
#define TBL_RFE(t)		(((t) >> 16) & 0xFF)
#define TBL_CV(t)		((t) & 0xFF)
#define TBL_HEADLINE		0xF
#define TBL_IF			0x8
#define TBL_ELIF		0x9
#define TBL_ELSE		0xA
#define TBL_END			0xB
#define TBL_CHECK		0x4
#define TBL_ANY			0xFF

static u32 tbl_addr(const struct ax52_reg2_tbl *t, u32 i)
{
	return le32_to_cpu(t->pairs[2 * i]);
}

/*
 * Leading "headline" entries name the (RFE type, chip cut) variants a table
 * carries. Ours is the exact match, else our RFE with any cut, else the
 * newest cut listed for our RFE, else the newest cut for any RFE.
 */
static int tbl_select(struct ax52_dev *rd, const struct ax52_reg2_tbl *t,
		      u32 *nhead, u32 *target)
{
	u8 rfe = rd->efuse.rfe_type, cv_max = 0;
	u32 want[2] = { rfe << 16 | rd->cv, rfe << 16 | TBL_ANY };
	u32 n, i, round, pick = U32_MAX;

	for (n = 0; n < t->n && TBL_COND(tbl_addr(t, n)) == TBL_HEADLINE; n++)
		;
	*nhead = n;
	*target = 0;
	if (!n)
		return 0;

	for (round = 0; round < 2; round++)
		for (i = 0; i < n; i++)
			if (TBL_TARGET(tbl_addr(t, i)) == want[round]) {
				*target = want[round];
				return 0;
			}

	for (round = 0; round < 2 && pick == U32_MAX; round++)
		for (i = 0; i < n; i++) {
			u32 tg = TBL_TARGET(tbl_addr(t, i));

			if (TBL_RFE(tg) == (round ? TBL_ANY : rfe) &&
			    TBL_CV(tg) >= cv_max) {
				cv_max = TBL_CV(tg);
				pick = i;
			}
		}
	if (pick == U32_MAX)
		return -EINVAL;
	*target = TBL_TARGET(tbl_addr(t, pick));
	return 0;
}

/*
 * Walk a table, passing the entries of our variant to @apply. Conditional
 * blocks are IF/ELIF <target> + CHECK ... END; an ELSE arm is never taken
 * (every variant must be listed) and aborts the load.
 */
static int tbl_apply(struct ax52_dev *rd, const struct ax52_reg2_tbl *t,
		     void (*apply)(struct ax52_dev *rd, u32 addr, u32 data,
				   void *ctx),
		     void *ctx)
{
	bool matched = true, found = false;
	u32 nhead, cfg, target = 0, i;

	if (tbl_select(rd, t, &nhead, &cfg)) {
		ax52_err(rd, "PHY table has no variant for RFE %u cut %u\n",
			 rd->efuse.rfe_type, rd->cv);
		return -EINVAL;
	}

	for (i = nhead; i < t->n; i++) {
		u32 addr = tbl_addr(t, i);

		switch (TBL_COND(addr)) {
		case TBL_IF:
		case TBL_ELIF:
			target = TBL_TARGET(addr);
			break;
		case TBL_CHECK:
			matched = !found && target == cfg;
			found |= matched;
			break;
		case TBL_ELSE:
			if (!found) {
				ax52_err(rd, "PHY table: no branch for our variant at entry %u\n",
					 i);
				return -EINVAL;
			}
			matched = false;
			break;
		case TBL_END:
			matched = true;
			found = false;
			break;
		default:
			if (matched)
				apply(rd, addr, le32_to_cpu(t->pairs[2 * i + 1]), ctx);
		}
	}
	return 0;
}

#define BB_SIZE			0x10000		/* BB register window */

static void bb_tbl_write(struct ax52_dev *rd, u32 addr, u32 data, void *ctx)
{
	/* addresses 0xF9..0xFE are delays */
	static const u32 delay_us[] = { 1, 5, 50, 1000, 5000, 50000 };

	if (addr >= 0xF9 && addr <= 0xFE) {
		fsleep(delay_us[addr - 0xF9]);
	} else if (addr >= BB_SIZE || (addr & 3)) {
		/* the table comes from a file: never write outside the BB */
		dev_warn_once(rd->dev, "BB table entry for 0x%x ignored\n", addr);
	} else if (data != 0xBABECAFE) {	/* "leave unchanged" marker */
		ax52_bb_write(rd, addr, data);
	}
}

/*
 * BB gain table: nothing is written, the values are applied per channel.
 * addr = kind[31:24] gain band[23:16] path[15:8] type[7:0], where for RPL
 * offsets type = bandwidth[7:4] first sub-channel[3:0].
 */
static void bb_gain_parse(struct ax52_dev *rd, u32 addr, u32 data, void *ctx)
{
	struct ax52_bb_gain *g = ctx;
	u8 type = addr & 0xFF, path = (addr >> 8) & 0xFF;
	u8 gb = (addr >> 16) & 0xFF, bw = type >> 4, sc = type & 0xF;
	int i;

	if (gb >= BB_GAIN_BAND_NUM || path >= RF_PATH_NUM)
		return;
	if (addr >= 0xF9 && addr <= 0xFE) {
		ax52_warn(rd, "BB gain table with a delay entry\n");
		return;
	}

	switch (addr >> 24) {
	case 0:		/* LNA / TIA gain error */
		if (type == 0)
			for (i = 0; i < 4; i++, data >>= 8)
				g->lna[gb][path][i] = data;
		else if (type == 1)
			for (i = 4; i < 7; i++, data >>= 8)
				g->lna[gb][path][i] = data;
		else if (type == 2)
			for (i = 0; i < 2; i++, data >>= 8)
				g->tia[gb][path][i] = data;
		break;
	case 1:		/* received-power-level offsets per (sub-)channel width */
		if (bw == 0)
			g->rpl_20[gb][path] = data;
		else if (bw == 1 && sc == 0)
			g->rpl_40[gb][path][0] = data;
		else if (bw == 1 && sc == 1)
			for (i = 1; i < 3; i++, data >>= 8)
				g->rpl_40[gb][path][i] = data;
		else if (bw == 2 && sc == 0)
			g->rpl_80[gb][path][0] = data;
		else if (bw == 2 && sc == 1)
			for (i = 1; i < 5; i++, data >>= 8)
				g->rpl_80[gb][path][i] = data;
		else if (bw == 2 && sc == 9)
			for (i = 9; i < 11; i++, data >>= 8)
				g->rpl_80[gb][path][i] = data;
		break;
	case 2 ... 4:	/* LNA bypass, op1dB, external-FEM data: unused here */
		break;
	default:
		ax52_warn(rd, "BB gain entry of unknown kind 0x%08x\n", addr);
	}
}

#define RF_H2C_MAX_WORDS	1500	/* firmware takes 3 pages of 500 */

struct rf_tbl_ctx {
	u8 path;
	u32 n;
	u32 *words;
};

static void rf_tbl_write(struct ax52_dev *rd, u32 addr, u32 data, void *ctx)
{
	struct rf_tbl_ctx *c = ctx;

	if (addr == 0xFE) {
		fsleep(50000);
		return;
	}
	ax52_rf_write(rd, c->path, addr, RF_MASK, data);

	/* the firmware keeps its own image of the D-die registers */
	if (addr < 0x100)
		return;
	if (c->n < RF_H2C_MAX_WORDS)
		c->words[c->n] = addr << 20 | data;
	c->n++;
}

static int phy_load_rf(struct ax52_dev *rd)
{
	struct rf_tbl_ctx c;
	int ret = 0;

	c.words = kmalloc_array(RF_H2C_MAX_WORDS, sizeof(u32), GFP_KERNEL);
	if (!c.words)
		return -ENOMEM;

	for (c.path = RF_PATH_A; c.path < RF_PATH_NUM; c.path++) {
		c.n = 0;
		ret = tbl_apply(rd, &rd->fw.radio[c.path], rf_tbl_write, &c);
		if (ret)
			break;
		if (c.n > RF_H2C_MAX_WORDS) {
			ax52_warn(rd, "RF path %u: %u D-die entries, firmware copy truncated\n",
				  c.path, c.n);
			c.n = RF_H2C_MAX_WORDS;
		}
		if (c.n && ax52_h2c_rf_reg(rd, c.path, c.words, c.n))
			ax52_warn(rd, "RF path %u: register copy to firmware failed\n",
				  c.path);
	}
	kfree(c.words);
	return ret;
}

/* ------------------------------------------------------------ BB reset */

static void phy_si_dis(struct ax52_dev *rd, u32 v)
{
	bb_mask(rd, BB_S0_HW_SI_DIS, HW_SI_DIS_TRIG, v);
	bb_mask(rd, BB_S0_HW_SI_DIS + PATH_OFS(1), HW_SI_DIS_TRIG, v);
}

/* Hold (or release) TX power control and TSSI tracking on both paths. */
static void phy_txpwr_hold(struct ax52_dev *rd, bool hold)
{
	u8 path;

	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		bb_mask(rd, BB_P0_TXPW_RSTB + PATH_OFS(path), TXPW_RSTB_MANON, hold);
		bb_mask(rd, BB_P0_TSSI_TRK + PATH_OFS(path), TSSI_TRK_EN, hold);
	}
}

/* Pulse the BB asynchronous reset with the RF serial engines stopped. */
static void phy_bb_reset_all(struct ax52_dev *rd)
{
	phy_si_dis(rd, 7);
	fsleep(1);
	bb_set(rd, BB_RSTB_ASYNC, RSTB_ASYNC_ALL);
	bb_clr(rd, BB_RSTB_ASYNC, RSTB_ASYNC_ALL);
	phy_si_dis(rd, 0);
	bb_set(rd, BB_RSTB_ASYNC, RSTB_ASYNC_ALL);
}

void ax52_phy_bb_reset(struct ax52_dev *rd)
{
	phy_txpwr_hold(rd, true);
	phy_bb_reset_all(rd);
	phy_txpwr_hold(rd, false);
}

/* ------------------------------------------------------ RX gain settings */

/* logical efuse RX gain offsets (A: high nibble, B: low): CCK, 2G OFDM, 5G L/M/H */
static const u16 ef_rx_gain[5] = { 0x2D6, 0x2D4, 0x2D8, 0x2DA, 0x2DC };
/* physical efuse RX gain compensation per sub-band (low nibble) */
static const u16 ef_gain_comp[RF_PATH_NUM][AX52_SB_NUM] = {
	{ 0x5BB, 0x5BA, 0, 0x5B9, 0x5B8 },
	{ 0x590, 0x58F, 0, 0x58E, 0x58D },
};

static void phy_efuse_gain(struct ax52_dev *rd)
{
	struct ax52_phy *p = rd->phy_priv;
	u8 v, path;
	int i;

	p->gain_ofst_valid = false;
	for (i = 0; i < ARRAY_SIZE(ef_rx_gain); i++) {
		v = rd->efuse.log[ef_rx_gain[i]];
		p->gain_ofst[RF_PATH_A][i] = sign_extend32(v >> 4, 3);
		p->gain_ofst[RF_PATH_B][i] = sign_extend32(v & 0xF, 3);
		p->gain_ofst_valid |= v != 0xFF;
	}

	p->gain_comp_valid = false;
	for (path = RF_PATH_A; path < RF_PATH_NUM; path++)
		for (i = 0; i < AX52_SB_NUM; i++) {
			if (!ef_gain_comp[path][i])
				continue;
			v = rd->efuse.phycap[ef_gain_comp[path][i] - EFUSE_PHYCAP_ADDR];
			p->gain_comp[path][i] = sign_extend32(v & 0xF, 3);
			p->gain_comp_valid |= v != 0xFF;
		}
}

/* AGC gain tables: LNA 0-6, TIA 0-1 in three registers per band and path */
static const u16 gain_regs[2][RF_PATH_NUM][3] = {
	{ { 0x4678, 0x467C, 0x4680 }, { 0x475C, 0x4760, 0x4764 } },	/* 2 GHz */
	{ { 0x45DC, 0x4660, 0x4664 }, { 0x4740, 0x4744, 0x4748 } },	/* 5 GHz */
};

static void phy_set_gain_error(struct ax52_dev *rd, const struct ax52_chan *c,
			       u8 path)
{
	struct ax52_phy *p = rd->phy_priv;
	const u16 *reg = gain_regs[c->band][path];
	const s8 *lna = p->gain.lna[sb_gain_band[c->subband]][path];
	const s8 *tia = p->gain.tia[sb_gain_band[c->subband]][path];

	bb_mask(rd, reg[0], GENMASK(31, 16), (u8)lna[0] | (u8)lna[1] << 8);
	ax52_bb_write(rd, reg[1], (u8)lna[2] | (u8)lna[3] << 8 |
				   (u8)lna[4] << 16 | (u32)(u8)lna[5] << 24);
	bb_mask(rd, reg[2], 0xFFFF00FF,
		(u8)lna[6] | (u8)tia[0] << 16 | (u32)(u8)tia[1] << 24);
}

/* Per-device RX gain calibration from efuse, for RSSI accuracy. */
static void phy_set_gain_offset(struct ax52_dev *rd, u8 subband)
{
	static const u16 comp_reg[RF_PATH_NUM] = { 0x4ACC, 0x4AD8 };
	static const u16 rssi_ofst_reg[RF_PATH_NUM] = { 0x4694, 0x4778 };
	struct ax52_phy *p = rd->phy_priv;
	u8 ob = sb_ofst_band[subband], path;
	int ofdm, cck;

	if (p->gain_comp_valid)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++)
			bb_mask(rd, comp_reg[path], GENMASK(7, 0),
				clamp(p->gain_comp[path][subband] * 4, S8_MIN, S8_MAX));

	if (!p->gain_ofst_valid)
		return;

	for (path = RF_PATH_A; path < RF_PATH_NUM; path++)
		bb_mask(rd, rssi_ofst_reg[path], GENMASK(23, 16),
			clamp(p->gain_ofst[path][ob] * 4 - (p->offset_base >> 2),
			      S8_MIN, S8_MAX));

	ofdm = -p->gain_ofst[RF_PATH_A][ob];
	cck = -p->gain_ofst[RF_PATH_A][0];
	bb_mask(rd, BB_P0_RPL1, GENMASK(7, 0),
		clamp(ofdm * 16 + p->offset_base, S8_MIN, S8_MAX));
	bb_mask(rd, BB_P1_RPL1, GENMASK(7, 0),
		clamp(ofdm * 16 + p->rssi_base, S8_MIN, S8_MAX));
	if (subband == AX52_SB_2G)
		bb_mask(rd, BB_RX_RPL_OFST, GENMASK(6, 0),
			clamp(cck * 8 + (p->offset_base >> 1), -64, 63));
}

static u32 rpl_avg(s8 a, s8 b)
{
	return (u8)((a + b) >> 1);
}

/* RX power-level compensation per sub-channel, averaged over both paths. */
static void phy_set_rxsc_rpl_comp(struct ax52_dev *rd, u8 subband)
{
	struct ax52_bb_gain *g = &((struct ax52_phy *)rd->phy_priv)->gain;
	u8 gb = sb_gain_band[subband];
	const s8 *a40 = g->rpl_40[gb][RF_PATH_A], *b40 = g->rpl_40[gb][RF_PATH_B];
	const s8 *a80 = g->rpl_80[gb][RF_PATH_A], *b80 = g->rpl_80[gb][RF_PATH_B];
	u32 rpl1, rpl2, rpl3;

	rpl1 = rpl_avg(g->rpl_20[gb][RF_PATH_A], g->rpl_20[gb][RF_PATH_B]) |
	       rpl_avg(a40[0], b40[0]) << 8 | rpl_avg(a40[1], b40[1]) << 16;
	rpl2 = rpl_avg(a40[2], b40[2]) | rpl_avg(a80[0], b80[0]) << 8 |
	       rpl_avg(a80[1], b80[1]) << 16 | rpl_avg(a80[10], b80[10]) << 24;
	rpl3 = rpl_avg(a80[2], b80[2]) | rpl_avg(a80[3], b80[3]) << 8 |
	       rpl_avg(a80[4], b80[4]) << 16 | rpl_avg(a80[9], b80[9]) << 24;

	bb_mask(rd, BB_P0_RPL1, GENMASK(31, 8), rpl1);
	bb_mask(rd, BB_P1_RPL1, GENMASK(31, 8), rpl1);
	ax52_bb_write(rd, BB_P0_RPL2, rpl2);
	ax52_bb_write(rd, BB_P1_RPL2, rpl2);
	ax52_bb_write(rd, BB_P0_RPL3, rpl3);
	ax52_bb_write(rd, BB_P1_RPL3, rpl3);
}

/* ------------------------------------------------- antenna / RX paths */

/*
 * RFE 1 wires Bluetooth to the 2.4 GHz front end of path B ("BTG"): let the
 * AGC account for the shared LNA and for BT grants.
 */
static void phy_ctrl_btg(struct ax52_dev *rd, bool en)
{
	bb_mask(rd, BB_P0_BAND_SEL, BT_SHARE, en);
	bb_clr(rd, BB_P0_BAND_SEL, BTG_PATH);
	bb_mask(rd, 0x476C, GENMASK(31, 24), en ? 0x20 : 0x1A);	/* B: LNA6 op1dB */
	bb_mask(rd, 0x4778, GENMASK(7, 0), en ? 0x30 : 0x2A);	/* B: TIA0 LNA6 op1dB */
	bb_mask(rd, BB_P1_BAND_SEL, BT_SHARE, en);
	bb_mask(rd, BB_P1_BAND_SEL, BTG_PATH, en);
	bb_mask(rd, 0x0980, GENMASK(20, 17), en ? 0 : 0xC);	/* PMAC grant, path B */
	bb_mask(rd, BB_CHBW_MOD, CHBW_BT_SHARE, en);
	bb_mask(rd, BB_FC0_BW, ANT_RX_BT_SEG0, en ? 2 : 0);
	bb_set(rd, BB_BT_DYN_DC_EST, BIT(31));
	bb_mask(rd, BB_GNT_BT_WGT, GNT_BT_WGT_EN, en);
}

/* Receive on both paths, up to 2 spatial streams; TX is driven by the MAC. */
static void phy_cfg_txrx_path(struct ax52_dev *rd)
{
	const struct ax52_chan *c = &rd->chan;
	u8 path;

	bb_mask(rd, BB_CHBW_MOD, ANT_RX_SEG0, 3);
	bb_mask(rd, BB_FC0_BW, ANT_RX_1RCCA_SEG0, 3);
	bb_mask(rd, BB_FC0_BW, ANT_RX_1RCCA_SEG1, 3);
	bb_mask(rd, BB_RX_MCS_LIMIT, RXHT_MCS_LIMIT, 1);
	bb_mask(rd, BB_RX_MCS_LIMIT, RXVHT_MCS_LIMIT, 1);
	bb_mask(rd, BB_RXHE, RXHE_USER_MAX, 4);
	bb_mask(rd, BB_RXHE, RXHE_MAX_NSS, 1);
	bb_mask(rd, BB_RXHE, RXHETB_MAX_NSS, 1);

	phy_set_gain_offset(rd, c->subband);
	phy_ctrl_btg(rd, c->band == AX52_BAND_2G);

	/* restart path B TX power control */
	bb_mask(rd, BB_P0_TXPW_RSTB + PATH_OFS(1), TXPW_RSTB_MANON | TXPW_RSTB_TSSI, 1);
	bb_mask(rd, BB_P0_TXPW_RSTB + PATH_OFS(1), TXPW_RSTB_MANON | TXPW_RSTB_TSSI, 3);

	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		bb_mask(rd, BB_P0_RFMODE + PATH_OFS(path), RFMODE_ORI_TXRX, 0x1233312);
		bb_mask(rd, BB_P0_RFMODE_FTM_RX + PATH_OFS(path), GENMASK(11, 0), 0x333);
	}
	bb_mask(rd, 0x09A4, GENMASK(4, 2), 0);	/* TX from the MAC, not the PMAC */
}

/* ------------------------------------------------ dynamic mechanisms */

static void phy_bb_sethw(struct ax52_dev *rd)
{
	struct ax52_phy *p = rd->phy_priv;
	int i;

	/* no sounding responses without an NDP */
	bb_clr(rd, 0x0D7C, BIT(1));
	bb_clr(rd, 0x2D7C, BIT(1));

	/* no per-station TX power limits */
	for (i = 0; i < 128; i++)
		wr32(rd, REG_PWR_MACID_LMT + 4 * i, 0);

	/* RPL / RSSI bias as left by the BB table, base for the efuse offsets */
	p->offset_base = ax52_bb_read_mask(rd, BB_P0_RPL1, GENMASK(7, 0));
	p->rssi_base = ax52_bb_read_mask(rd, BB_P1_RPL1, GENMASK(7, 0));
}

/* PPDU kinds with a PHY-status IE bitmap register (kind 9 has none). */
enum {
	STS_HE_MU = 6, STS_VHT_MU = 7, STS_RSVD = 9, STS_TRIG = 10,
	STS_CCK = 11, STS_HT = 13, STS_KINDS = 16,
};
#define STS_IE_CMN_OFDM		BIT(1)		/* channel, RSSI, SNR, CFO */
#define STS_IE_EXT_PATH		GENMASK(7, 4)	/* per-path extensions */
#define STS_IE_DL_MU		BIT(13)
#define STS_IE_FD_USER0		BIT(20)

/*
 * Per-PPDU PHY status: the common IE for every kind of received PPDU, no
 * per-path extensions, no reports for failed searches or aborted receptions.
 */
static void phy_physts_init(struct ax52_dev *rd)
{
	u32 reg, v;
	int i;

	bb_set(rd, BB_PLCP_HISTOGRAM, STS_DIS_TRIG_BY_FAIL | STS_DIS_TRIG_BY_BRK);
	for (i = 0; i < STS_KINDS; i++) {
		if (i == STS_RSVD)
			continue;
		reg = BB_PHY_STS_BITMAP + 4 * (i < STS_RSVD ? i : i - 1);
		v = ax52_bb_read(rd, reg);
		if (i == STS_HE_MU || i == STS_VHT_MU) {
			v |= STS_IE_DL_MU;
		} else if (i == STS_TRIG) {
			v |= STS_IE_DL_MU | STS_IE_CMN_OFDM;
		} else if (i >= STS_CCK) {
			v &= ~STS_IE_EXT_PATH;
			if (i == STS_CCK)
				v |= STS_IE_CMN_OFDM;
			else if (i >= STS_HT)
				v |= STS_IE_FD_USER0;
		}
		ax52_bb_write(rd, reg, v);
	}
}

/*
 * DIG reset state: packet-detection lower bounds off (most sensitive), the
 * CCK bound parked at -128 dBm, secondary-20 AGC not following the primary.
 */
static void phy_dig_init(struct ax52_dev *rd)
{
	static const u16 sdagc[] = { 0x46E8, 0x46EC, 0x47A8, 0x47AC };
	int i;

	bb_mask(rd, BB_SEG0R_PD, PD_LOWER_BOUND, 0);
	bb_clr(rd, BB_SEG0R_PD, PD_SR_EN);
	bb_clr(rd, BB_BMODE_PDTH_EN, BMODE_PDTH_LIMIT_EN);
	bb_mask(rd, BB_BMODE_PDTH, BMODE_PDTH_LOWER, 0x80);
	for (i = 0; i < ARRAY_SIZE(sdagc); i++)
		bb_clr(rd, sdagc[i], BIT(5));
}

/*
 * While associated, ignore preambles well below the AP's level (fewer false
 * alarms): the lower bounds sit "under" dB below the AP's beacon RSSI, in
 * units of dBm + 110. This is the vendor DIG without its false-alarm
 * feedback, which would need the CCA / false-alarm counters. Scanning and
 * connecting stay fully sensitive.
 */
static void phy_dig(struct ax52_dev *rd)
{
	int dbm = 0, rssi, under, ofdm, cck;

	if (rd->vif && rd->rvif.assoc && !rd->scanning)
		dbm = ieee80211_ave_rssi(rd->vif, -1);
	if (!dbm) {
		phy_dig_init(rd);
		return;
	}

	rssi = clamp(dbm + 110, 0, 110);
	under = 16 + 7 + 3 * rd->chan.bw;
	ofdm = clamp(rssi, 8 + under, 70 + under);
	cck = max(rssi - under, -18);
	bb_mask(rd, BB_SEG0R_PD, PD_LOWER_BOUND, (ofdm - under - 8) >> 1);
	bb_set(rd, BB_SEG0R_PD, PD_SR_EN);
	bb_set(rd, BB_BMODE_PDTH_EN, BMODE_PDTH_LIMIT_EN);
	bb_mask(rd, BB_BMODE_PDTH, BMODE_PDTH_LOWER, cck - 110);	/* dBm */
}

/* Crystal trim from efuse; hardware CFO compensation on RX and UL TX. */
static int phy_cfo_init(struct ax52_dev *rd)
{
	u8 xcap = rd->efuse.xtal_cap & 0x7F;
	int ret;

	ret = ax52_xsi_write(rd, XSI_XTAL_SC_XO, xcap, 0xFF) ?:
	      ax52_xsi_write(rd, XSI_XTAL_SC_XI, xcap, 0xFF);
	if (ret)
		return ret;
	bb_set(rd, BB_DCFO_OPT, DCFO_OPT_EN);
	bb_mask(rd, BB_DCFO_WEIGHT, GENMASK(27, 24), 8);
	mask32(rd, REG_PWR_UL_CTRL2, PWR_UL_CFO_MASK, 6);
	return 0;
}

/* Per-unit thermal-meter and PA-bias trims from the physical efuse. */
static void phy_power_trim(struct ax52_dev *rd)
{
	static const u16 thm_addr[RF_PATH_NUM] = { 0x5DF, 0x5DC };
	static const u16 pab_addr[RF_PATH_NUM] = { 0x5DE, 0x5DB };
	const u8 *pc = rd->efuse.phycap;
	u8 thm[RF_PATH_NUM], pab[RF_PATH_NUM], path;

	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		thm[path] = pc[thm_addr[path] - EFUSE_PHYCAP_ADDR];
		pab[path] = pc[pab_addr[path] - EFUSE_PHYCAP_ADDR];
	}

	if (thm[RF_PATH_A] != 0xFF || thm[RF_PATH_B] != 0xFF)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++)
			ax52_rf_write(rd, path, RR_TM2, RR_TM2_OFF,
				      (thm[path] & 1) << 3 | (thm[path] & 0x1F) >> 1);

	if (pab[RF_PATH_A] != 0xFF || pab[RF_PATH_B] != 0xFF)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
			ax52_rf_write(rd, path, RR_BIASA, RR_BIASA_TXG, pab[path] & 0xF);
			ax52_rf_write(rd, path, RR_BIASA, RR_BIASA_TXA, pab[path] >> 4);
		}
}

/* ------------------------------------------------------------ init */

int ax52_phy_alloc(struct ax52_dev *rd)
{
	struct ax52_phy *p = kvzalloc(sizeof(*p), GFP_KERNEL);	/* ~43 KiB */

	rd->phy_priv = p;
	return p ? 0 : -ENOMEM;
}

void ax52_phy_free(struct ax52_dev *rd)
{
	kvfree(rd->phy_priv);
	rd->phy_priv = NULL;
}

int ax52_phy_init(struct ax52_dev *rd)
{
	/* the radio tables leave the synthesizer on 2.4 GHz channel 1 */
	static const struct ax52_chan chan_default = {
		.band = AX52_BAND_2G, .bw = AX52_BW_20, .ch = 1, .pri_ch = 1,
		.freq = 2412, .pri_freq = 2412, .subband = AX52_SB_2G,
	};
	struct ax52_phy *p = rd->phy_priv;
	struct ax52_fw *fw = &rd->fw;
	int ret;

	if (!fw->bb.pairs || !fw->bb_gain.pairs || !fw->radio[RF_PATH_A].pairs ||
	    !fw->radio[RF_PATH_B].pairs) {
		ax52_err(rd, "firmware file lacks the BB/RF parameter tables\n");
		return -ENOENT;
	}

	rd->chan = chan_default;
	phy_efuse_gain(rd);

	ret = tbl_apply(rd, &fw->bb, bb_tbl_write, NULL);
	if (ret)
		return ret;
	ax52_txpwr_init_unit(rd);
	memset(&p->gain, 0, sizeof(p->gain));
	ret = tbl_apply(rd, &fw->bb_gain, bb_gain_parse, &p->gain);
	if (ret)
		return ret;
	ax52_phy_bb_reset(rd);

	ret = phy_load_rf(rd);
	if (ret)
		return ret;

	phy_bb_sethw(rd);
	phy_physts_init(rd);
	phy_dig_init(rd);
	ret = phy_cfo_init(rd);
	if (ret)
		return ret;
	bb_mask(rd, 0x0C70, GENMASK(25, 20), 0x29);	/* EDCCA: TX-collision T2R start */

	ax52_txpwr_load(rd);
	ax52_txpwr_set_ref(rd);
	phy_power_trim(rd);
	phy_cfg_txrx_path(rd);
	return 0;
}

/* ------------------------------------------------------------ channel */

static int phy_5g_subband(int ch)
{
	if (ch >= 36 && ch <= 64)
		return AX52_SB_5G_1;
	if (ch >= 100 && ch <= 144)
		return AX52_SB_5G_3;
	if (ch >= 149 && ch <= 177)
		return AX52_SB_5G_4;
	return -EINVAL;
}

static int phy_chan_from_chandef(const struct cfg80211_chan_def *cd,
				 struct ax52_chan *c)
{
	int span, lo;

	memset(c, 0, sizeof(*c));
	if (!cd->chan)
		return -EINVAL;

	switch (cd->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		c->bw = AX52_BW_20;
		span = 0;
		break;
	case NL80211_CHAN_WIDTH_40:
		c->bw = AX52_BW_40;
		span = 2;
		break;
	case NL80211_CHAN_WIDTH_80:
		c->bw = AX52_BW_80;
		span = 6;
		break;
	default:
		return -EINVAL;
	}

	c->pri_freq = cd->chan->center_freq;
	c->freq = cd->center_freq1;
	c->pri_ch = ieee80211_frequency_to_channel(c->pri_freq);
	c->ch = ieee80211_frequency_to_channel(c->freq);

	/* all 20 MHz sub-channels must lie within one sub-band */
	switch (cd->chan->band) {
	case NL80211_BAND_2GHZ:
		c->band = AX52_BAND_2G;
		c->subband = AX52_SB_2G;
		if (c->bw == AX52_BW_80 || c->ch - span < 1 || c->ch + span > 14)
			return -EINVAL;
		return 0;
	case NL80211_BAND_5GHZ:
		c->band = AX52_BAND_5G;
		lo = phy_5g_subband(c->ch - span);
		if (lo < 0 || lo != phy_5g_subband(c->ch + span))
			return -EINVAL;
		c->subband = lo;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * Index of the primary sub-channel of width @dbw within the channel:
 * 0 = the whole channel; 1/2 = upper/lower 20 (40) or inner upper/lower
 * 20 (80); 3/4 = outer upper/lower 20 (80); 9/10 = upper/lower 40 (80).
 */
static u8 phy_txsc(const struct ax52_chan *c, u8 dbw)
{
	bool upper = c->pri_ch > c->ch;

	if (c->bw == dbw || c->bw == AX52_BW_20)
		return 0;
	if (c->bw == AX52_BW_40)
		return upper ? 1 : 2;
	if (dbw == AX52_BW_20)
		return upper ? (c->pri_ch - c->ch) >> 1 : ((c->ch - c->pri_ch) >> 1) + 1;
	return upper ? 9 : 10;
}

/* Quiesce TX, PPDU status, TX power loops, ADC and BB for reprogramming. */
static void phy_switch_enter(struct ax52_dev *rd)
{
	int ret = ax52_mac_sch_tx_en(rd, false);

	if (ret)
		ax52_warn(rd, "cannot pause TX for the channel switch (%d)\n", ret);
	ax52_mac_ppdu_status(rd, false);
	phy_txpwr_hold(rd, true);
	bb_mask(rd, BB_ADC_FIFO, ADC_FIFO_RST, 0xF);
	fsleep(40);
	bb_set(rd, BB_RXCCA, RXCCA_DIS);
	bb_set(rd, BB_PD_CTRL, PD_HIT_DIS);
	phy_si_dis(rd, 7);
	fsleep(1);
	bb_clr(rd, BB_RSTB_ASYNC, RSTB_ASYNC_ALL);
}

static void phy_switch_exit(struct ax52_dev *rd, const struct ax52_chan *c)
{
	int ret;

	ax52_mac_ppdu_status(rd, true);
	bb_mask(rd, BB_ADC_FIFO, ADC_FIFO_RST, 0);
	phy_txpwr_hold(rd, false);
	phy_si_dis(rd, 0);
	bb_set(rd, BB_RSTB_ASYNC, RSTB_ASYNC_ALL);
	if (c->band == AX52_BAND_2G)	/* CCK CCA stays off on 5 GHz */
		bb_clr(rd, BB_RXCCA, RXCCA_DIS);
	bb_clr(rd, BB_PD_CTRL, PD_HIT_DIS);

	ret = ax52_mac_sch_tx_en(rd, true);
	if (ret)
		ax52_warn(rd, "cannot resume TX after the channel switch (%d)\n", ret);
}

static void phy_set_channel_mac(struct ax52_dev *rd, const struct ax52_chan *c)
{
	mask8(rd, REG_WMAC_RFMOD, GENMASK(1, 0), c->bw);
	wr32(rd, REG_TX_SUB_CARRIER, phy_txsc(c, AX52_BW_20) |
				     phy_txsc(c, AX52_BW_40) << 4);
	/* on 5 GHz: refuse CCK rates, RTS at OFDM 6M at least */
	if (c->band == AX52_BAND_5G) {
		clr8(rd, REG_TXRATE_CHK, TXRATE_BAND_2G);
		set8(rd, REG_TXRATE_CHK, TXRATE_CHECK_CCK | TXRATE_RTS_OFDM6);
	} else {
		set8(rd, REG_TXRATE_CHK, TXRATE_BAND_2G);
		clr8(rd, REG_TXRATE_CHK, TXRATE_CHECK_CCK | TXRATE_RTS_OFDM6);
	}
}

/* CCK sub-carrier offset thresholds, by primary channel 1-14 */
static const u32 sco_barker[14] = {
	0x1CFEA, 0x1D0E1, 0x1D1D7, 0x1D2CD, 0x1D3C3, 0x1D4B9, 0x1D5B0,
	0x1D6A6, 0x1D79C, 0x1D892, 0x1D988, 0x1DA7F, 0x1DB75, 0x1DDC4,
};
static const u32 sco_cck[14] = {
	0x27DE3, 0x27F35, 0x28088, 0x281DA, 0x2832D, 0x2847F, 0x285D2,
	0x28724, 0x28877, 0x289C9, 0x28B1C, 0x28C6E, 0x28DC1, 0x290ED,
};

/* CCK TX filter taps; channel 14 has its own (Japan) mask */
static const u32 cck_fir[2][8] = {
	{ 0x3D23FF, 0x29B354, 0x0FC1C8, 0xFDB053, 0xF86F9A, 0xFAEF92, 0xFE5FCC, 0xFFDFF5 },
	{ 0x3B13FF, 0x1C42DE, 0xFDB0AD, 0xF60F6E, 0xFD8F92, 0x02D011, 0x01C02C, 0xFFF00A },
};

/* Sampling-clock-offset compensation for the centre channel (validated). */
static u8 phy_sco_comp(u8 ch)
{
	if (ch == 1)
		return 109;
	if (ch <= 6)
		return 108;
	if (ch <= 10)
		return 107;
	if (ch <= 14)
		return 106;
	if (ch <= 38)
		return 51;
	if (ch <= 58)
		return 50;
	if (ch <= 64)
		return 49;
	if (ch <= 102)
		return 48;
	if (ch <= 126)
		return 47;
	if (ch <= 151)
		return 46;
	return 45;
}

/* Primary channel as reported back in PHY status: base index[7:4], offset[3:0]. */
static u8 phy_chan_idx(const struct ax52_chan *c)
{
	static const u8 base5g[] = { 36, 100, 132, 149 };	/* indices 2..5 */
	int i;

	if (c->band == AX52_BAND_2G)
		return c->pri_ch;
	for (i = ARRAY_SIZE(base5g) - 1; i > 0 && c->pri_ch < base5g[i]; i--)
		;
	return (i + 2) << 4 | (c->pri_ch - base5g[i]) >> 1;
}

/* Mask the 5 MHz band edge for 40 MHz and for edge-primary 80 MHz. */
static void phy_set_5m_mask(struct ax52_dev *rd, const struct ax52_chan *c,
			    u8 pri_idx)
{
	static const u16 reg[RF_PATH_NUM] = { BB_P0_5MDET, BB_P1_5MDET };
	bool en = c->bw == AX52_BW_40 || (c->bw == AX52_BW_80 && pri_idx >= 3);
	bool low = pri_idx == (c->bw == AX52_BW_40 ? 2 : 4);
	u8 path;

	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		if (!en) {
			bb_clr(rd, reg[path], MDET5_EN);
			continue;
		}
		bb_mask(rd, reg[path], MDET5_TH, 4);
		bb_set(rd, reg[path], MDET5_EN);
		bb_mask(rd, reg[path], MDET5_SB2, !low);
		bb_mask(rd, reg[path], MDET5_SB0, low);
	}
	bb_mask(rd, BB_ASSIGN_SBD_OPT, BIT(31), en);
}

static void phy_set_channel_bb(struct ax52_dev *rd, const struct ax52_chan *c)
{
	bool is_2g = c->band == AX52_BAND_2G;
	u8 pri_idx = phy_txsc(c, AX52_BW_20), path;
	u32 rx_path;
	int i;

	if (is_2g) {
		bb_mask(rd, BB_RXSCOBC, GENMASK(18, 0), sco_barker[c->pri_ch - 1]);
		bb_mask(rd, BB_RXSCOCCK, GENMASK(18, 0), sco_cck[c->pri_ch - 1]);
	}

	bb_mask(rd, BB_P0_BAND_SEL, BAND_SEL_2G, is_2g);
	bb_mask(rd, BB_P1_BAND_SEL, BAND_SEL_2G, is_2g);
	bb_mask(rd, BB_FC0_BW, FC0_BW_INV, phy_sco_comp(c->ch));
	for (i = 0; i < ARRAY_SIZE(cck_fir[0]); i++)
		bb_mask(rd, BB_TXFIR + 4 * i, GENMASK(23, 0), cck_fir[c->ch == 14][i]);

	phy_set_gain_error(rd, c, RF_PATH_A);
	phy_set_gain_error(rd, c, RF_PATH_B);
	phy_set_gain_offset(rd, c->subband);
	phy_set_rxsc_rpl_comp(rd, c->subband);

	/* bandwidth and primary sub-channel */
	rx_path = ax52_bb_read_mask(rd, BB_CHBW_MOD, ANT_RX_SEG0);
	bb_mask(rd, BB_FC0_BW, FC0_BW_SET, c->bw);
	bb_mask(rd, BB_CHBW_MOD, CHBW_SBW, 0);
	bb_mask(rd, BB_CHBW_MOD, CHBW_PRICH, pri_idx);
	bb_mask(rd, BB_P0_RFMODE, RFMODE_ORI_RX, 0x333);
	bb_mask(rd, BB_P0_RFMODE + PATH_OFS(1), RFMODE_ORI_RX, 0x333);
	if (c->bw == AX52_BW_40)
		bb_mask(rd, BB_RXSC, RXSC_CCK_UPPER, pri_idx == 1);
	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		/* ADC at full rate for 20/40/80 MHz */
		bb_mask(rd, 0xC0EC + (path << 8), GENMASK(14, 13), 0);
		bb_mask(rd, 0xC0E4 + (path << 8), GENMASK(5, 4), 2);
	}
	if (rx_path == BIT(RF_PATH_A))
		bb_mask(rd, BB_P0_RFMODE + PATH_OFS(1), RFMODE_ORI_RX, 0x111);
	else if (rx_path == BIT(RF_PATH_B))
		bb_mask(rd, BB_P0_RFMODE, RFMODE_ORI_RX, 0x111);

	/* CCK only on 2.4 GHz */
	bb_mask(rd, BB_UPD_CLK_ADC, ENABLE_CCK, is_2g);
	bb_mask(rd, BB_RXCCA, RXCCA_DIS, !is_2g);

	/* BT shares path B at 2.4 GHz only (coexistence would redo this) */
	if (is_2g) {
		phy_ctrl_btg(rd, true);
	} else {
		bb_clr(rd, BB_P0_BAND_SEL, BT_SHARE | BTG_PATH);
		bb_clr(rd, BB_P1_BAND_SEL, BT_SHARE | BTG_PATH);
		bb_clr(rd, BB_CHBW_MOD, CHBW_BT_SHARE);
		bb_mask(rd, BB_FC0_BW, ANT_RX_BT_SEG0, 0);
		bb_clr(rd, BB_BT_DYN_DC_EST, BIT(31));
		bb_clr(rd, BB_GNT_BT_WGT, GNT_BT_WGT_EN);
	}

	bb_mask(rd, BB_MAC_PIN_SEL, CH_IDX_SEG0, phy_chan_idx(c));
	phy_set_5m_mask(rd, c, pri_idx);
	/* fully sensitive until the tracker has the AP's RSSI on this channel */
	phy_dig_init(rd);
	phy_bb_reset_all(rd);
}

/*
 * Path A's A-die channel word starts the synthesizer: write it with the LDO
 * selector switched and wait for the loop filter, then (still with TX
 * paused) verify lock. The lock check and its recovery live in rfk.c.
 */
static void phy_rf18_write_s0(struct ax52_dev *rd, u32 v)
{
	u32 bak = ax52_rf_read(rd, RF_PATH_A, RR_LDO, RF_MASK), busy;
	int timeout;

	ax52_rf_write(rd, RF_PATH_A, RR_LDO, RR_LDO_SEL, 1);
	ax52_rf_write(rd, RF_PATH_A, RR_CFGCH, RF_MASK, v);
	timeout = read_poll_timeout_atomic(ax52_rf_read, busy, !busy, 1, 1000,
					   false, rd, RF_PATH_A, RR_LPF,
					   RR_LPF_BUSY);
	if (timeout)
		ax52_dbg(rd, "RF synthesizer still busy after the channel write\n");
	ax52_rf_write(rd, RF_PATH_A, RR_LDO, RF_MASK, bak);
	if (!timeout)
		ax52_rfk_lck_check(rd);
}

static void phy_set_channel_rf(struct ax52_dev *rd, const struct ax52_chan *c)
{
	static const u8 rf_bw[] = { [AX52_BW_20] = 3, [AX52_BW_40] = 2, [AX52_BW_80] = 1 };
	static const u8 rxbb_bw[] = { [AX52_BW_20] = 0x1B, [AX52_BW_40] = 0x13,
				      [AX52_BW_80] = 0x0B };
	/* each path has an A-die and a D-die copy of the channel word */
	static const u32 cfgch[] = { RR_CFGCH, RF_DDIE | RR_CFGCH };
	u8 die, path;
	u32 v;

	for (die = 0; die < ARRAY_SIZE(cfgch); die++)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
			v = ax52_rf_read(rd, path, cfgch[die], RF_MASK);
			if (v == RF_INV)
				continue;
			v &= ~(RR_CFGCH_BAND1 | RR_CFGCH_POW_LCK | RR_CFGCH_TRX_AH |
			       RR_CFGCH_BCN | RR_CFGCH_BW2 | RR_CFGCH_BAND0 | RR_CFGCH_CH);
			v |= FIELD_PREP(RR_CFGCH_CH, c->ch) | RR_CFGCH_BW2;
			if (c->band == AX52_BAND_5G)
				v |= FIELD_PREP(RR_CFGCH_BAND1, 1) | FIELD_PREP(RR_CFGCH_BAND0, 1);
			if (die == 0 && path == RF_PATH_A)
				phy_rf18_write_s0(rd, v);
			else
				ax52_rf_write(rd, path, cfgch[die], RF_MASK, v);
			/* re-arm the lock-state latch */
			ax52_rf_write(rd, path, RR_LCKST, RR_LCKST_BIN, 0);
			ax52_rf_write(rd, path, RR_LCKST, RR_LCKST_BIN, 1);
		}

	for (die = 0; die < ARRAY_SIZE(cfgch); die++)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
			v = ax52_rf_read(rd, path, cfgch[die], RF_MASK);
			if (v == RF_INV)
				continue;
			v &= ~(RR_CFGCH_BW | RR_CFGCH_POW_LCK | RR_CFGCH_TRX_AH |
			       RR_CFGCH_BCN | RR_CFGCH_BW2);
			v |= FIELD_PREP(RR_CFGCH_BW, rf_bw[c->bw]) | RR_CFGCH_BW2;
			ax52_rf_write(rd, path, cfgch[die], RF_MASK, v);
		}

	/* RX baseband filter, through the LUT write port */
	for (path = RF_PATH_A; path < RF_PATH_NUM; path++) {
		ax52_rf_write(rd, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 1);
		ax52_rf_write(rd, path, RR_LUTWA, GENMASK(4, 0), 0x12);
		ax52_rf_write(rd, path, RR_LUTWD0, GENMASK(5, 0), rxbb_bw[c->bw]);
		ax52_rf_write(rd, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 0);
	}
}

int ax52_phy_set_channel(struct ax52_dev *rd,
			 const struct cfg80211_chan_def *chandef)
{
	struct ax52_chan c;

	if (phy_chan_from_chandef(chandef, &c)) {
		ax52_err(rd, "unsupported channel: %u MHz, width %d\n",
			 chandef->center_freq1, chandef->width);
		return -EINVAL;
	}
	rd->chan = c;

	phy_switch_enter(rd);
	phy_set_channel_mac(rd, &c);
	phy_set_channel_bb(rd, &c);
	phy_set_channel_rf(rd, &c);
	ax52_txpwr_apply(rd);
	phy_switch_exit(rd, &c);

	ax52_rfk_channel(rd);
	return 0;
}

void ax52_phy_set_txpwr(struct ax52_dev *rd)
{
	if (rd->mac_on && rd->chan.ch)
		ax52_txpwr_apply(rd);
}

/* ------------------------------------------------------------ tracking */

/*
 * Energy detection stays at the BB-table -62 dBm and the crystal at its
 * efuse trim: tracking those needs per-packet CFO and noise statistics.
 */
void ax52_phy_track(struct ax52_dev *rd)
{
	phy_dig(rd);
}

/* Raw 6-bit thermal-meter code of @path; 0 if it cannot be read. */
u8 ax52_phy_thermal(struct ax52_dev *rd, u8 path)
{
	u32 v;

	if (path >= RF_PATH_NUM)
		return 0;

	/* with TSSI running, use its own sample instead of re-triggering */
	if (ax52_bb_read_mask(rd, BB_P0_TSSI_AVG + PATH_OFS(path), TSSI_EN))
		return ax52_bb_read_mask(rd, BB_P0_TSSI_THERMAL + PATH_OFS(path),
					  TSSI_THERMAL);

	ax52_rf_write(rd, path, RR_TM, RR_TM_TRIG, 1);
	ax52_rf_write(rd, path, RR_TM, RR_TM_TRIG, 0);
	ax52_rf_write(rd, path, RR_TM, RR_TM_TRIG, 1);
	fsleep(200);
	v = ax52_rf_read(rd, path, RR_TM, RR_TM_VAL);
	return v == RF_INV ? 0 : v;
}
