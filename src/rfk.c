// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side RF calibration ("RFK") of the RTL8852B analog front end.
 *
 * Power-on: NCTL microcode, RC filter (RCK), DAC/ADC DC and DAC MSB (DACK),
 * RX DC offset (RX DCK). Per association: RX DCK, TX/RX IQ imbalance and LO
 * leakage (IQK), closed-loop TX power (TSSI, aligned with over-the-air PMAC
 * packets) and PA pre-distortion (DPK). A band switch re-arms TSSI from
 * stored results and every 2 s the DPD gain follows the thermal meter.
 *
 * IQK and DPK are microcode on the NCTL/KIP engine; the host prepares RF/BB
 * state, fires one-shot commands and reads reports. Nothing here reloads a
 * result into the chip by itself: a failed calibration leaves the engine's
 * neutral defaults (identity IQ correction, DPD off).
 *
 * All entry points run in process context with the wiphy mutex held.
 */
#include <linux/average.h>
#include <linux/delay.h>

#include "rfk.h"

DECLARE_EWMA(thermal, 4, 4)

#define TSSI_CH_NUM		67	/* 14 + 15 + 23 + 15 channel slots */
#define RFK_BAND_NONE		0xff

#define DPK_TXAGC_INIT		0x38
#define DPK_TXAGC_MIN		0x2e
#define DPK_TXAGC_MAX		0x3f
#define DPK_TXAGC_FAIL		0xff
#define DPK_PWSF		0x78	/* power scaling factor at calibration */

/* IQK one-shot operations (NCTL command bits [11:8]) */
enum {
	IQK_LOK_COARSE = 0x1,
	IQK_LOK_FINE = 0x2,
	IQK_LOK_VBUF = 0x3,
	IQK_TXK = 0x8,		/* + bandwidth */
	IQK_RXK = 0xb,		/* + bandwidth */
	IQK_RESTORE = 0xe,
};

/* DPK one-shot operations */
enum {
	DPK_LBK_RXIQK = 0x06,
	DPK_SYNC = 0x10,
	DPK_MDPK_IDL = 0x11,
	DPK_GAIN_LOSS = 0x13,
	DPK_RXAGC = 0x15,
	DPK_KIP_PRESET = 0x16,
	DPK_TXAGC = 0x19,
};

struct rfk_dpk {
	bool ok;		/* DPD running on this path */
	u8 band, bw, ch;	/* channel it was calibrated on */
	u8 ther;		/* thermal code at calibration */
};

struct ax52_rfk {
	bool nctl_ok;
	bool tssi_on[RF_PATH_NUM];
	u8 last_band;		/* band of the previous channel */
	u8 dpk_gs;		/* DPD gain scale: 0x5b, 0x7f if BB backs off */
	struct ewma_thermal thermal[RF_PATH_NUM];
	struct rfk_dpk dpk[RF_PATH_NUM];

	/* factory TSSI data from efuse */
	s8 tssi_cck[RF_PATH_NUM][6];
	s8 tssi_mcs[RF_PATH_NUM][19];
	s8 tssi_trim[RF_PATH_NUM][8];

	/* TSSI alignment results: latest per band, and per channel */
	bool align_done[RF_PATH_NUM][TSSI_BANDS];
	u32 align[RF_PATH_NUM][TSSI_BANDS][4];
	bool align_ch_done[RF_PATH_NUM][TSSI_CH_NUM];
	u32 align_ch[RF_PATH_NUM][TSSI_CH_NUM][4];
};

/* Registers saved around IQK and DPK. */
static const u16 rfk_bak_bb_regs[] = { 0x2344, 0x5800, 0x7800 };
static const u32 rfk_bak_rf_regs[] = {
	0xde, 0xdf, 0x8b, 0x90, 0x97, 0x85, 0x1e, 0x00, 0x02, 0x05, 0x10005,
};

#define RFK_BAK_BB		ARRAY_SIZE(rfk_bak_bb_regs)
#define RFK_BAK_RF		ARRAY_SIZE(rfk_bak_rf_regs)

/* TSSI alignment words, in BB address order (path B at +0x2000) */
static const u16 tssi_align_regs[4] = { 0x5630, 0x5634, 0x563c, 0x5640 };

static char path_name(u8 path)
{
	return 'A' + path;
}

/* ------------------------------------------------------------- helpers */

static void rfk_apply(struct ax52_dev *rd, const struct rfk_tbl *t, u8 path)
{
	const struct rfk_reg *r;

	for (r = t->regs; r < t->regs + t->n; r++) {
		switch (r->op) {
		case RFK_OP_BB:
			ax52_bb_write_mask(rd, r->addr, r->mask, r->val);
			break;
		case RFK_OP_BBP:
			ax52_bb_write_mask(rd, BB_P(path, r->addr), r->mask, r->val);
			break;
		case RFK_OP_DELAY:
			udelay(r->val);
			break;
		}
	}
}

static int rfk_poll_bb(struct ax52_dev *rd, u32 addr, u32 mask, u32 want,
		       u32 timeout_us)
{
	u32 v;

	return read_poll_timeout(ax52_bb_read_mask, v, v == want, 10,
				 timeout_us, false, rd, addr, mask);
}

static void rfk_backup_bb(struct ax52_dev *rd, u32 *bak)
{
	int i;

	for (i = 0; i < RFK_BAK_BB; i++)
		bak[i] = ax52_bb_read(rd, rfk_bak_bb_regs[i]);
}

static void rfk_restore_bb(struct ax52_dev *rd, const u32 *bak)
{
	int i;

	for (i = 0; i < RFK_BAK_BB; i++)
		ax52_bb_write(rd, rfk_bak_bb_regs[i], bak[i]);
}

static void rfk_backup_rf(struct ax52_dev *rd, u8 path, u32 *bak)
{
	int i;

	for (i = 0; i < RFK_BAK_RF; i++)
		bak[i] = ax52_rf_read(rd, path, rfk_bak_rf_regs[i], RFREG_MASK);
}

static void rfk_restore_rf(struct ax52_dev *rd, u8 path, const u32 *bak)
{
	int i;

	for (i = 0; i < RFK_BAK_RF; i++)
		ax52_rf_write(rd, path, rfk_bak_rf_regs[i], RFREG_MASK, bak[i]);
}

/* Hand the RF of @path to software (false) or back to the baseband. */
static void rfk_rf_by_bb(struct ax52_dev *rd, u8 path, bool bybb)
{
	ax52_rf_write(rd, path, RF_RSV1, BIT(0), bybb);
	ax52_rf_write(rd, path, RF_BBDC, BIT(0), bybb);
}

/* Let any frame in flight finish: wait until no path is in TX mode. */
static void rfk_wait_rx_mode(struct ax52_dev *rd)
{
	u32 mode;
	u8 p;

	for (p = 0; p < RF_PATH_NUM; p++)
		if (read_poll_timeout(ax52_rf_read, mode, mode != MODE_TX, 10,
				      5000, false, rd, p, RF_MOD, RF_MOD_MODE))
			ax52_dbg(rd, "rfk: path %c stuck in TX\n", path_name(p));
}

/* Calibrations that use the air: BT handshake, then stop MAC TX. */
static void rfk_begin(struct ax52_dev *rd)
{
	ax52_coex_rfk_begin(rd);
	ax52_mac_sch_tx_en(rd, false);
	rfk_wait_rx_mode(rd);
}

static void rfk_end(struct ax52_dev *rd)
{
	ax52_mac_sch_tx_en(rd, true);
	ax52_coex_rfk_end(rd);
}

/* Latch a new RX DC offset estimate in the RF (no readback exists). */
static void rfk_rx_dck_kick(struct ax52_dev *rd, u8 path)
{
	ax52_rf_write(rd, path, RF_DCK1, GENMASK(3, 0), 0);
	ax52_rf_write(rd, path, RF_DCK, BIT(0), 0);
	ax52_rf_write(rd, path, RF_DCK, BIT(0), 1);
	fsleep(1000);
}

/* Raw thermal code through the RF meter (disturbs TSSI; not while it runs). */
static u8 rfk_thermal_rf(struct ax52_dev *rd, u8 path)
{
	ax52_rf_write(rd, path, RF_TM, BIT(19), 1);
	ax52_rf_write(rd, path, RF_TM, BIT(19), 0);
	ax52_rf_write(rd, path, RF_TM, BIT(19), 1);
	fsleep(200);
	return ax52_rf_read(rd, path, RF_TM, GENMASK(6, 1));
}

static u8 rfk_thermal(struct ax52_dev *rd, u8 path)
{
	struct ax52_rfk *rk = rd->rfk_priv;

	/* with TSSI running, use the sample its engine already took */
	if (rk->tssi_on[path])
		return ax52_bb_read_mask(rd, BB_P(path, 0x1c10), GENMASK(29, 24));
	return rfk_thermal_rf(rd, path);
}

static void rfk_thermal_update(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 p, th;

	for (p = 0; p < RF_PATH_NUM; p++) {
		th = rfk_thermal(rd, p);
		if (th)
			ewma_thermal_add(&rk->thermal[p], th);
	}
}

/* ------------------------------------------------------------ NCTL */

static u32 nctl_probe(struct ax52_dev *rd)
{
	ax52_bb_write(rd, NCTL_ALIVE, 0x4);
	udelay(1);
	return ax52_bb_read(rd, NCTL_ALIVE);
}

/* Bring up the calibration engine and load its microcode/defaults. */
static void rfk_nctl_init(struct ax52_dev *rd)
{
	const struct ax52_reg2_tbl *t = &rd->fw.nctl;
	struct ax52_rfk *rk = rd->rfk_priv;
	u32 i, v, addr;

	/* IQK/DPK clock and reset release */
	ax52_bb_write_mask(rd, 0x0c60, GENMASK(1, 0), 3);
	ax52_bb_write_mask(rd, 0x0c6c, BIT(0), 1);
	ax52_bb_write_mask(rd, 0x58ac, BIT(27), 1);
	ax52_bb_write_mask(rd, 0x78ac, BIT(27), 1);
	ax52_bb_write(rd, NCTL_CFG, 0x8);

	if (read_poll_timeout(nctl_probe, v, v == 0x4, 10, 1000, false, rd))
		ax52_err(rd, "NCTL calibration engine does not respond\n");

	rk->nctl_ok = t->pairs && t->n;
	if (!rk->nctl_ok) {
		ax52_warn(rd, "firmware has no NCTL table: IQK and DPK disabled\n");
		return;
	}

	/* plain {addr, data} writes; nothing outside the engine's window */
	for (i = 0; i < t->n; i++) {
		addr = le32_to_cpu(t->pairs[2 * i]);
		if (addr < 0x8000 || addr > 0xbffc)
			continue;
		ax52_bb_write(rd, addr, le32_to_cpu(t->pairs[2 * i + 1]));
	}
}

/*
 * One NCTL command: bits [11:8] operation, bit (4 + path) path select. IQK
 * reports failure in NCTL_RPT; DPK results come from report pages instead.
 */
static u32 nctl_cmd(u8 path, u8 op)
{
	return (op << 8) | BIT(4 + path) | 0x9;
}

static bool nctl_iqk_run(struct ax52_dev *rd, u8 path, u8 op)
{
	bool fail = true;
	int ret;

	ax52_bb_write(rd, NCTL_CFG, nctl_cmd(path, op));
	udelay(1);
	ret = rfk_poll_bb(rd, NCTL_DONE, GENMASK(7, 0), 0x55, 8200);
	fsleep(200);
	if (!ret)
		fail = ax52_bb_read_mask(rd, NCTL_RPT, BIT(26));
	ax52_bb_write_mask(rd, NCTL_N1, GENMASK(7, 0), 0);
	return fail;
}

static bool iqk_one_shot(struct ax52_dev *rd, u8 path, u8 op, bool rfctm)
{
	bool fail;

	/* the path-A register hands both paths' RF to the engine */
	ax52_bb_write_mask(rd, BB_P0_RFCTM, BIT(29), rfctm);
	fail = nctl_iqk_run(rd, path, op);
	ax52_bb_write_mask(rd, BB_P0_RFCTM, BIT(29), 0);
	return fail;
}

static void dpk_one_shot(struct ax52_dev *rd, u8 path, u8 op)
{
	ax52_bb_write(rd, NCTL_CFG, nctl_cmd(path, op));
	if (rfk_poll_bb(rd, NCTL_DONE, GENMASK(7, 0), 0x55, 20000))
		ax52_dbg(rd, "DPK %c: op 0x%02x timed out\n", path_name(path), op);
	udelay(1);
	/* wait for the KIP to go idle (report page 3) */
	ax52_bb_write(rd, KIP_RPT_SEL, 0x00030000);
	if (rfk_poll_bb(rd, KIP_RPT, GENMASK(15, 0), 0x8000, 2000))
		ax52_dbg(rd, "DPK %c: KIP busy after op 0x%02x\n",
			 path_name(path), op);
	ax52_bb_write_mask(rd, NCTL_N1, GENMASK(7, 0), 0);
}

/* DPK operations that drive the RF themselves. */
static void dpk_one_shot_rf(struct ax52_dev *rd, u8 path, u8 op)
{
	ax52_bb_write_mask(rd, BB_P0_RFCTM, BIT(29), 1);
	dpk_one_shot(rd, path, op);
	ax52_bb_write_mask(rd, BB_P0_RFCTM, BIT(29), 0);
}

/* ------------------------------------------------------------- RCK */

/* Tune the RF baseband filter RC corner. */
static void rfk_rck(struct ax52_dev *rd, u8 path)
{
	u32 rsv1, v;

	rsv1 = ax52_rf_read(rd, path, RF_RSV1, RFREG_MASK);
	ax52_rf_write(rd, path, RF_RSV1, BIT(0), 0);
	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, MODE_RX);

	ax52_rf_write(rd, path, RF_RCKC, RFREG_MASK, 0x00240);
	if (read_poll_timeout_atomic(ax52_rf_read, v, v, 2, 30, false,
				     rd, path, RF_RCKS, BIT(3)))
		ax52_dbg(rd, "RCK %c: no done flag\n", path_name(path));

	/* the 5-bit result replaces the whole register (clears the kick) */
	v = ax52_rf_read(rd, path, RF_RCKC, GENMASK(14, 10));
	ax52_rf_write(rd, path, RF_RCKC, RFREG_MASK, v);
	ax52_rf_write(rd, path, RF_RSV1, RFREG_MASK, rsv1);
}

/* ------------------------------------------------------------ DACK */

/* Average the ADC DC of @path on the BB debug port (diagnostic only). */
static void dack_adc_dc_probe(struct ax52_dev *rd, u8 path)
{
	s32 re = 0, im = 0;
	u32 v;
	int i;

	ax52_bb_write_mask(rd, 0x20f4, BIT(24), 0);
	ax52_bb_write_mask(rd, 0x20f8, BIT(31), 1);
	ax52_bb_write_mask(rd, 0x20f0, GENMASK(23, 16), 1);
	ax52_bb_write_mask(rd, 0x20f0, GENMASK(11, 8), 2);
	ax52_bb_write_mask(rd, 0x20f0, GENMASK(3, 0), 0);
	ax52_bb_write_mask(rd, 0x20f0, GENMASK(7, 6), 2 + path);

	for (i = 0; i < 100; i++) {
		v = ax52_bb_read(rd, 0x1730);
		re += sign_extend32(FIELD_GET(GENMASK(23, 12), v), 11);
		im += sign_extend32(FIELD_GET(GENMASK(11, 0), v), 11);
	}
	ax52_dbg(rd, "DACK %c: ADC DC %d/%d\n", path_name(path), re / 100,
		 im / 100);
}

/* D-die RC calibration; one instance serves both paths. */
static void dack_drck(struct ax52_dev *rd)
{
	u32 v;

	ax52_bb_write_mask(rd, 0xc0cc, BIT(6), 1);
	if (rfk_poll_bb(rd, 0xc0d0, BIT(3), 1, 10000))
		ax52_warn(rd, "DRCK timed out\n");
	ax52_bb_write_mask(rd, 0xc0cc, BIT(6), 0);
	ax52_bb_write_mask(rd, 0xc094, BIT(9), 1);
	udelay(1);
	ax52_bb_write_mask(rd, 0xc094, BIT(9), 0);
	v = ax52_bb_read_mask(rd, 0xc0d0, GENMASK(19, 15));
	ax52_bb_write_mask(rd, 0xc0cc, BIT(9), 0);
	ax52_bb_write_mask(rd, 0xc0cc, GENMASK(4, 0), v);
}

/* ADC DC calibration of one path (AFE block of path B at +0x100). */
static void dack_addck_path(struct ax52_dev *rd, u8 path)
{
	u32 afe = path << 8;

	ax52_bb_write_mask(rd, BB_P(path, 0x12b8), BIT(30), 1);
	ax52_bb_write_mask(rd, 0x032c, BIT(30), 0);	/* ADC clock off */
	ax52_bb_write_mask(rd, 0x032c, BIT(22), 0);	/* filter reset */
	ax52_bb_write_mask(rd, 0x032c, BIT(22), 1);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(27, 24), 0xf);
	ax52_bb_write_mask(rd, 0x032c, BIT(16), 0);
	ax52_bb_write_mask(rd, 0xc0d4 + afe, BIT(1), 1);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(27, 24), 0x3);
	dack_adc_dc_probe(rd, path);

	ax52_bb_write_mask(rd, 0xc0f4 + afe, BIT(11), 1);
	ax52_bb_write_mask(rd, 0xc0f4 + afe, BIT(11), 0);
	udelay(1);
	ax52_bb_write_mask(rd, 0xc0f4 + afe, GENMASK(9, 8), 1);
	if (rfk_poll_bb(rd, 0xc0fc + afe, BIT(0), 1, 10000))
		ax52_warn(rd, "ADDCK %c timed out\n", path_name(path));
	dack_adc_dc_probe(rd, path);

	ax52_bb_write_mask(rd, 0xc0d4 + afe, BIT(1), 0);
	ax52_bb_write_mask(rd, 0x032c, BIT(16), 1);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(27, 24), 0xc);
	ax52_bb_write_mask(rd, 0x032c, BIT(30), 1);
	ax52_bb_write_mask(rd, BB_P(path, 0x12b8), BIT(30), 0);
}

/* Freeze the ADC DC results as manual compensation values. */
static void dack_addck_hold(struct ax52_dev *rd)
{
	u32 d[RF_PATH_NUM][2];
	u8 p;

	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write_mask(rd, 0xc0f4 + (p << 8), GENMASK(9, 8), 0);
		d[p][0] = ax52_bb_read_mask(rd, 0xc0fc + (p << 8), GENMASK(19, 10));
		d[p][1] = ax52_bb_read_mask(rd, 0xc0fc + (p << 8), GENMASK(9, 0));
	}
	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write_mask(rd, 0xc0f0 + (p << 8), GENMASK(25, 16), d[p][0]);
		ax52_bb_write_mask(rd, 0xc0f4 + (p << 8), GENMASK(3, 0), d[p][1] >> 6);
		ax52_bb_write_mask(rd, 0xc0f0 + (p << 8), GENMASK(31, 26),
				   d[p][1] & 0x3f);
		ax52_bb_write_mask(rd, 0xc0f4 + (p << 8), GENMASK(5, 4), 3);
		ax52_dbg(rd, "DACK %c: ADDCK 0x%x/0x%x\n", path_name(p),
			 d[p][0], d[p][1]);
	}
}

static bool dack_step_done(struct ax52_dev *rd, u8 path, bool msbk)
{
	u32 afe = path << 8;
	bool i, q;

	if (msbk) {
		i = ax52_bb_read_mask(rd, 0xc040 + afe, BIT(31));
		q = ax52_bb_read_mask(rd, 0xc064 + afe, BIT(31));
	} else {
		i = ax52_bb_read_mask(rd, 0xc05c + afe, BIT(2));
		q = ax52_bb_read_mask(rd, 0xc080 + afe, BIT(2));
	}
	/* the vendor flow accepts either I or Q on path B */
	return path == RF_PATH_A ? i && q : i || q;
}

static void dack_wait(struct ax52_dev *rd, u8 path, bool msbk)
{
	bool done;

	if (read_poll_timeout(dack_step_done, done, done, 10, 10000, false,
			      rd, path, msbk))
		ax52_warn(rd, "DACK %c: %s timed out\n", path_name(path),
			  msbk ? "MSBK" : "DADCK");
}

/* Path-B DAC loop-back check; it leaves the ANAPAR state that follows. */
static void dack_dadc_check_b(struct ax52_dev *rd)
{
	ax52_bb_write_mask(rd, 0x032c, BIT(30), 0);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(27, 24), 0xf);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(27, 24), 0x3);
	ax52_bb_write_mask(rd, 0x032c, BIT(16), 0);
	ax52_bb_write_mask(rd, 0x32dc, BIT(0), 1);
	ax52_bb_write_mask(rd, 0x32e8, BIT(2), 1);
	ax52_rf_write(rd, RF_PATH_B, RF_RXBB2, BIT(13), 1);
	dack_adc_dc_probe(rd, RF_PATH_B);
	ax52_bb_write_mask(rd, 0x32dc, BIT(0), 0);
	ax52_bb_write_mask(rd, 0x32e8, BIT(2), 0);
	ax52_rf_write(rd, RF_PATH_B, RF_RXBB2, BIT(13), 0);
	ax52_bb_write_mask(rd, 0x032c, BIT(16), 1);
}

/* Read back the DAC calibration results (diagnostic only). */
static void dack_readback(struct ax52_dev *rd, u8 path)
{
	u32 afe = path << 8;
	u8 msbk[2][16];
	int i;

	ax52_bb_write_mask(rd, BB_P(path, 0x12b8), BIT(30), 1);
	for (i = 0; i < 16; i++) {
		ax52_bb_write_mask(rd, 0xc000 + afe, GENMASK(4, 1), i);
		msbk[0][i] = ax52_bb_read_mask(rd, 0xc05c + afe, GENMASK(31, 24));
		ax52_bb_write_mask(rd, 0xc020 + afe, GENMASK(4, 1), i);
		msbk[1][i] = ax52_bb_read_mask(rd, 0xc080 + afe, GENMASK(31, 24));
	}
	ax52_dbg(rd, "DACK %c: MSBK %16ph / %16ph, bias 0x%x/0x%x, DADCK 0x%x/0x%x\n",
		 path_name(path), msbk[0], msbk[1],
		  ax52_bb_read_mask(rd, 0xc048 + afe, GENMASK(11, 2)),
		  ax52_bb_read_mask(rd, 0xc06c + afe, GENMASK(11, 2)),
		  ax52_bb_read_mask(rd, 0xc060 + afe, GENMASK(31, 24)),
		  ax52_bb_read_mask(rd, 0xc084 + afe, GENMASK(31, 24)));
	ax52_bb_write_mask(rd, BB_P(path, 0x12b8), BIT(30), 0);
}

static void dack_path(struct ax52_dev *rd, u8 path)
{
	bool a = path == RF_PATH_A;

	rfk_apply(rd, a ? &ax52_rfk_dack_s0_msbk : &ax52_rfk_dack_s1_msbk, path);
	dack_wait(rd, path, true);
	rfk_apply(rd, a ? &ax52_rfk_dack_s0_dadck : &ax52_rfk_dack_s1_dadck, path);
	dack_wait(rd, path, false);
	rfk_apply(rd, a ? &ax52_rfk_dack_s0_done : &ax52_rfk_dack_s1_done, path);
	if (!a)
		dack_dadc_check_b(rd);
	dack_readback(rd, path);
}

static void rfk_dack(struct ax52_dev *rd)
{
	u32 mod[RF_PATH_NUM];
	u8 p;

	ax52_coex_rfk_begin(rd);

	for (p = 0; p < RF_PATH_NUM; p++)
		mod[p] = ax52_rf_read(rd, p, RF_MOD, RFREG_MASK);

	wr32(rd, 0x8040, 0xf);			/* PHYREG_SET */
	rfk_apply(rd, &ax52_rfk_afe_init, 0);
	dack_drck(rd);

	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_rf_write(rd, p, RF_RSV1, BIT(0), 0);
	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_rf_write(rd, p, RF_MOD, RFREG_MASK, 0x337e1);

	ax52_bb_write_mask(rd, 0xc0f4, GENMASK(5, 4), 0);
	ax52_bb_write_mask(rd, 0xc1d4, GENMASK(5, 4), 0);
	dack_addck_path(rd, RF_PATH_A);
	dack_addck_path(rd, RF_PATH_B);
	dack_addck_hold(rd);

	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_rf_write(rd, p, RF_MODOPT, RFREG_MASK, 0);
	dack_path(rd, RF_PATH_A);
	dack_path(rd, RF_PATH_B);

	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_rf_write(rd, p, RF_MOD, RFREG_MASK, mod[p]);
	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_rf_write(rd, p, RF_RSV1, BIT(0), 1);

	ax52_coex_rfk_end(rd);
}

/* ---------------------------------------------------------- RX DCK */

static void rfk_rx_dck(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u32 rsv1, fine;
	u8 p;

	rfk_begin(rd);
	for (p = 0; p < RF_PATH_NUM; p++) {
		rsv1 = ax52_rf_read(rd, p, RF_RSV1, RFREG_MASK);
		fine = ax52_rf_read(rd, p, RF_DCK, BIT(1));
		if (rk->tssi_on[p])
			ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(30), 1);

		ax52_rf_write(rd, p, RF_RSV1, BIT(0), 0);
		ax52_rf_write(rd, p, RF_DCK, BIT(1), 0);
		ax52_rf_write(rd, p, RF_MOD, RF_MOD_MODE, MODE_RX);
		rfk_rx_dck_kick(rd, p);
		ax52_rf_write(rd, p, RF_DCK, BIT(1), fine);
		ax52_rf_write(rd, p, RF_RSV1, RFREG_MASK, rsv1);

		if (rk->tssi_on[p])
			ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(30), 0);
	}
	rfk_end(rd);
}

/* ------------------------------------------------------------- IQK */

/* {power range, track range, BB gain, tone power} per TXK group */
static const u8 iqk_txk_grp[2][4][4] = {
	[AX52_BAND_2G] = { { 0, 4, 0x08, 0x09 }, { 0, 4, 0x0e, 0x12 },
			    { 0, 6, 0x06, 0x1b }, { 0, 6, 0x0e, 0x24 } },
	[AX52_BAND_5G] = { { 0, 3, 0x08, 0x12 }, { 0, 3, 0x0e, 0x12 },
			    { 0, 6, 0x06, 0x12 }, { 0, 6, 0x0e, 0x1b } },
};

/* {RX gain, attenuator 2, attenuator 1} per RXK group */
static const u16 iqk_rxk_grp[2][4][3] = {
	[AX52_BAND_2G] = { { 0x212, 0x00, 3 }, { 0x21c, 0x00, 3 },
			    { 0x350, 0x28, 2 }, { 0x360, 0x5f, 1 } },
	[AX52_BAND_5G] = { { 0x190, 0x0f, 3 }, { 0x198, 0x0f, 1 },
			    { 0x350, 0x3f, 0 }, { 0x352, 0x7f, 0 } },
};

static void iqk_preset(struct ax52_dev *rd, u8 path)
{
	/* single-channel operation always uses coefficient bank 0 */
	ax52_bb_write_mask(rd, KIP_P(path, KIP_COEF_SEL), BIT(0), 0);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_CFIR_LUT), BIT(3), 0);
	rfk_rf_by_bb(rd, path, false);
	ax52_bb_write(rd, NCTL_RPT, 0x00000080);
	ax52_bb_write(rd, KIP_SYSCFG, 0x81ff010a);
}

/* ADC clocking for the TX (LOK/TXK) or RX (RXK) phase. */
static void iqk_adc_clk(struct ax52_dev *rd, bool rx)
{
	u32 div = rd->chan.bw == AX52_BW_80 ? 2 : 1;

	ax52_bb_write_mask(rd, 0x12b8, BIT(30), 1);
	ax52_bb_write_mask(rd, 0x32b8, BIT(30), 1);
	udelay(1);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(31, 24), rx ? 0x0f : 0x1f);
	udelay(1);
	ax52_bb_write_mask(rd, 0x030c, GENMASK(31, 24), rx ? 0x03 : 0x13);
	ax52_bb_write_mask(rd, 0x032c, GENMASK(31, 16), rx ? 0xa001 : 0x0001);
	udelay(1);
	ax52_bb_write_mask(rd, 0x032c, GENMASK(31, 16), rx ? 0xa041 : 0x0041);
	if (!rx)
		return;

	ax52_bb_write_mask(rd, 0x12a0, GENMASK(18, 16), div);
	ax52_bb_write_mask(rd, 0x12a0, BIT(19), 1);
	ax52_bb_write_mask(rd, 0x32a0, GENMASK(18, 16), div);
	ax52_bb_write_mask(rd, 0x32a0, BIT(19), 1);
	ax52_bb_write_mask(rd, 0x0700, BIT(24), 1);
	ax52_bb_write_mask(rd, 0x0700, GENMASK(26, 25), div - 1);
}

/* RF setup for LO leakage and TX IQ calibration. */
static void iqk_txk_rf_setup(struct ax52_dev *rd, u8 path, u8 ibias)
{
	bool is5g = rd->chan.band == AX52_BAND_5G;

	/* LOK current bias in the LUT */
	ax52_rf_write(rd, path, RF_LUTWE, RFREG_MASK, 0x2);
	ax52_rf_write(rd, path, RF_LUTWA, RFREG_MASK, is5g);
	ax52_rf_write(rd, path, RF_LUTWD0, RFREG_MASK, ibias);
	ax52_rf_write(rd, path, RF_LUTWE, RFREG_MASK, 0x0);
	ax52_rf_write(rd, path, RF_TXVBUF, BIT(5), 1);

	if (is5g) {
		ax52_rf_write(rd, path, RF_XGLNA2, GENMASK(1, 0), 0);
		ax52_rf_write(rd, path, RF_BIASA, GENMASK(2, 0), 1);
	} else {
		ax52_rf_write(rd, path, RF_XALNA2, GENMASK(9, 8), 0);
		ax52_rf_write(rd, path, RF_TXG1, BIT(19), 0);
		ax52_rf_write(rd, path, RF_TXG1, BIT(11), 0);
		ax52_rf_write(rd, path, RF_TXG2, BIT(11), 1);
	}
	ax52_rf_write(rd, path, RF_TXGA, GENMASK(4, 0), 0);
	ax52_rf_write(rd, path, RF_LUTWE, BIT(2), 1);
	ax52_rf_write(rd, path, RF_LUTWA, GENMASK(7, 0), is5g ? 0x80 : 0x00);
	ax52_rf_write(rd, path, RF_MOD, GENMASK(19, 4), 0x403e);
	udelay(1);
}

static void iqk_lok_shot(struct ax52_dev *rd, u8 path, u8 op, u8 gain, u32 itqt)
{
	ax52_rf_write(rd, path, RF_TXIG, GENMASK(16, 12), gain);
	ax52_bb_write(rd, KIP_P(path, KIP_IQP), itqt);
	iqk_one_shot(rd, path, op, true);
}

/* LO leakage: coarse and fine DAC search, each followed by a buffer pass. */
static bool iqk_lok(struct ax52_dev *rd, u8 path)
{
	u32 v, i, q;

	ax52_bb_write_mask(rd, NCTL_TONE, GENMASK(11, 0), 0x021);
	ax52_rf_write(rd, path, RF_TXIG, GENMASK(1, 0), 0);
	ax52_rf_write(rd, path, RF_TXIG, GENMASK(6, 4),
		      rd->chan.band == AX52_BAND_5G ? 4 : 6);

	iqk_lok_shot(rd, path, IQK_LOK_COARSE, 0, 0x09);
	iqk_lok_shot(rd, path, IQK_LOK_VBUF, 0x12, 0x24);
	ax52_rf_write(rd, path, RF_TXIG, GENMASK(16, 12), 0);
	ax52_bb_write(rd, KIP_P(path, KIP_IQP), 0x09);
	ax52_bb_write_mask(rd, NCTL_TONE, GENMASK(11, 0), 0x021);
	iqk_one_shot(rd, path, IQK_LOK_FINE, true);
	iqk_lok_shot(rd, path, IQK_LOK_VBUF, 0x12, 0x24);

	/* judged by the resulting DAC codes, not by the engine's flags */
	v = ax52_rf_read(rd, path, RF_TXMO, RFREG_MASK);
	i = FIELD_GET(GENMASK(19, 15), v);
	q = FIELD_GET(GENMASK(14, 10), v);
	if (i < 0x2 || i > 0x1d || q < 0x2 || q > 0x1d)
		return true;
	v = ax52_rf_read(rd, path, RF_LOKVB, RFREG_MASK);
	i = FIELD_GET(GENMASK(19, 14), v);
	q = FIELD_GET(GENMASK(9, 4), v);
	return i < 0x2 || i > 0x3d || q < 0x2 || q > 0x3d;
}

/* Wide-band TX IQ calibration over four gain groups. */
static bool iqk_txk(struct ax52_dev *rd, u8 path)
{
	const u8 (*grp)[4] = iqk_txk_grp[rd->chan.band];
	u32 lut = KIP_P(path, KIP_CFIR_LUT);
	bool fail = false;
	u8 gp;

	for (gp = 0; gp < 4; gp++) {
		ax52_rf_write(rd, path, RF_TXIG, GENMASK(1, 0), grp[gp][0]);
		ax52_rf_write(rd, path, RF_TXIG, GENMASK(6, 4), grp[gp][1]);
		ax52_rf_write(rd, path, RF_TXIG, GENMASK(16, 12), grp[gp][2]);
		ax52_bb_write(rd, KIP_P(path, KIP_IQP), grp[gp][3]);
		ax52_bb_write_mask(rd, lut, BIT(8), 1);
		ax52_bb_write_mask(rd, lut, BIT(4), 1);	/* TX table */
		ax52_bb_write_mask(rd, lut, BIT(2), 0);
		ax52_bb_write_mask(rd, lut, GENMASK(1, 0), gp);
		ax52_bb_write_mask(rd, NCTL_N1, GENMASK(7, 0), 0);
		fail |= iqk_one_shot(rd, path, IQK_TXK + rd->chan.bw, false);
	}
	/* use the wide-band filter result, or nothing if any group failed */
	ax52_bb_write_mask(rd, KIP_P(path, KIP_IQK_RES), GENMASK(11, 8),
			   fail ? 0 : 5);
	return fail;
}

/* Wide-band RX IQ calibration over four gain groups. */
static bool iqk_rxk(struct ax52_dev *rd, u8 path)
{
	const u16 (*grp)[3] = iqk_rxk_grp[rd->chan.band];
	bool is5g = rd->chan.band == AX52_BAND_5G;
	u32 lut = KIP_P(path, KIP_CFIR_LUT);
	bool fail = false;
	u8 gp;

	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, 0xc);
	ax52_rf_write(rd, path, RF_RXK, is5g ? BIT(7) : BIT(8), 1);
	ax52_rf_write(rd, path, RF_RSV4, RFREG_MASK,
		      ax52_rf_read(rd, path, RF_CFGCH, RFREG_MASK));

	for (gp = 0; gp < 4; gp++) {
		ax52_rf_write(rd, path, RF_MOD, GENMASK(13, 4), grp[gp][0]);
		if (is5g) {
			ax52_rf_write(rd, path, RF_RXA2, GENMASK(6, 0), grp[gp][1]);
			ax52_rf_write(rd, path, RF_RXA2, GENMASK(8, 7), grp[gp][2]);
		} else {
			ax52_rf_write(rd, path, RF_RXBB, GENMASK(16, 10), grp[gp][1]);
			ax52_rf_write(rd, path, RF_RXBB, GENMASK(9, 8), grp[gp][2]);
		}
		ax52_bb_write_mask(rd, lut, BIT(8), 1);
		ax52_bb_write_mask(rd, lut, BIT(4), 0);	/* RX table */
		ax52_bb_write_mask(rd, lut, GENMASK(2, 0), gp);
		fail |= iqk_one_shot(rd, path, IQK_RXK + rd->chan.bw, true);
	}
	/* only the 5 GHz select is cleared, as in the vendor flow */
	ax52_rf_write(rd, path, RF_RXK, BIT(7), 0);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_IQK_RES), GENMASK(3, 0),
			   fail ? 0 : 5);
	return fail;
}

/*
 * Commit the single-tap words (identity; bit1 set when the filter result is
 * not used) and return the engine and RF to normal operation.
 */
static void iqk_restore(struct ax52_dev *rd, u8 path, bool tx_fail, bool rx_fail)
{
	ax52_bb_write(rd, KIP_P(path, KIP_TXIQC), tx_fail ? 0x40000002 : 0x40000000);
	ax52_bb_write(rd, KIP_P(path, KIP_RXIQC), rx_fail ? 0x40000002 : 0x40000000);
	if (nctl_iqk_run(rd, path, IQK_RESTORE))
		ax52_dbg(rd, "IQK %c: restore reported failure\n", path_name(path));

	ax52_bb_write(rd, NCTL_RPT, 0);
	ax52_bb_write(rd, KIP_SYSCFG, 0x80000000);
	ax52_bb_write_mask(rd, KIP_CFIR_SYS, BIT(28), 0);
	ax52_bb_write_mask(rd, KIP_IQRSN, BIT(28), 0);
	ax52_bb_write_mask(rd, KIP_IQRSN, BIT(16), 0);
	ax52_rf_write(rd, path, RF_LUTWE, BIT(2), 0);
	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, MODE_RX);
	rfk_rf_by_bb(rd, path, true);
}

static void iqk_path(struct ax52_dev *rd, u8 path)
{
	u32 bb[RFK_BAK_BB], rf[RFK_BAK_RF];
	bool lok_fail = true, tx_fail, rx_fail;
	u8 ibias;

	rfk_backup_bb(rd, bb);
	rfk_backup_rf(rd, path, rf);
	rfk_apply(rd, &ax52_rfk_iqk_bb_set, path);
	iqk_preset(rd, path);

	iqk_adc_clk(rd, false);
	/* LOK: retry with increasing bias; a failure is not fatal */
	for (ibias = 1; ibias <= 3 && lok_fail; ibias++) {
		iqk_txk_rf_setup(rd, path, ibias);
		lok_fail = iqk_lok(rd, path);
	}
	tx_fail = iqk_txk(rd, path);

	iqk_adc_clk(rd, true);
	rx_fail = iqk_rxk(rd, path);

	iqk_restore(rd, path, tx_fail, rx_fail);
	rfk_apply(rd, &ax52_rfk_iqk_bb_restore, path);
	rfk_restore_bb(rd, bb);
	rfk_restore_rf(rd, path, rf);

	ax52_dbg(rd, "IQK %c ch %u: LOK %s, TXK %s, RXK %s\n", path_name(path),
		 rd->chan.ch, lok_fail ? "fail" : "ok", tx_fail ? "fail" : "ok",
		  rx_fail ? "fail" : "ok");
}

static void rfk_iqk(struct ax52_dev *rd)
{
	rfk_begin(rd);
	iqk_path(rd, RF_PATH_A);
	iqk_path(rd, RF_PATH_B);
	rfk_end(rd);
}

/* ------------------------------------------------------------- DPK */

static void dpk_set_txagc(struct ax52_dev *rd, u8 path, u8 txagc)
{
	ax52_rf_write(rd, path, RF_TXAGC, RFREG_MASK, txagc);
	dpk_one_shot_rf(rd, path, DPK_TXAGC);
}

/* Lower the TX gain by @loss steps within the DPK window; returns it. */
static u8 dpk_adjust_txagc(struct ax52_dev *rd, u8 path, int loss)
{
	int txagc = (int)ax52_rf_read(rd, path, RF_TXAGC, GENMASK(7, 0)) - loss;

	txagc = clamp(txagc, DPK_TXAGC_MIN, DPK_TXAGC_MAX);
	dpk_set_txagc(rd, path, txagc);
	return txagc;
}

static void dpk_rf_setup(struct ax52_dev *rd, u8 path)
{
	bool is5g = rd->chan.band == AX52_BAND_5G;

	ax52_rf_write(rd, path, RF_MOD, RFREG_MASK, 0x50220);
	if (is5g)
		ax52_rf_write(rd, path, RF_RXA2, GENMASK(15, 9), 0x5);
	else
		ax52_rf_write(rd, path, RF_RXBB, GENMASK(7, 0), 0xf2);
	ax52_rf_write(rd, path, RF_LUTDBG, BIT(12), 1);
	ax52_rf_write(rd, path, RF_TIA, BIT(8), 1);
	if (is5g) {
		ax52_rf_write(rd, path, RF_RXA_LNA, RFREG_MASK, 0x920fc);
		ax52_rf_write(rd, path, RF_XALNA2, RFREG_MASK, 0x002c0);
		ax52_rf_write(rd, path, RF_IQGEN, RFREG_MASK, 0x38800);
	}
	ax52_rf_write(rd, path, RF_RCKD, BIT(2), 1);
	ax52_rf_write(rd, path, RF_BTC, GENMASK(14, 12), rd->chan.bw + 1);
	ax52_rf_write(rd, path, RF_BTC, GENMASK(11, 10), 0);
}

/* 80 MHz: calibrate the loop-back RX IQ path used by the DPK capture. */
static void dpk_lbk_rxiqk(struct ax52_dev *rd, u8 path)
{
	u32 rxbb = ax52_rf_read(rd, path, RF_MOD, RF_MOD_RXBB);

	ax52_bb_write_mask(rd, KIP_MDPK_RXDCK, BIT(31), 1);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_IQK_RES), GENMASK(3, 0), 0);
	ax52_rf_write(rd, path, RF_RSV4, RFREG_MASK,
		      ax52_rf_read(rd, path, RF_CFGCH, RFREG_MASK));
	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, 0xd);
	ax52_rf_write(rd, path, RF_RXK, BIT(5), 1);
	ax52_rf_write(rd, path, RF_TXIQK, GENMASK(6, 0),
		      rxbb >= 0x11 ? 0x13 : rxbb <= 0xa ? 0x00 : 0x05);
	ax52_rf_write(rd, path, RF_XGLNA2, GENMASK(1, 0), 0);
	ax52_rf_write(rd, path, RF_RXKPLL, BIT(19), 0);
	ax52_rf_write(rd, path, RF_RXKPLL, RFREG_MASK, 0x80014);
	fsleep(70);

	ax52_bb_write_mask(rd, NCTL_TONE, GENMASK(27, 16), 0x025);
	dpk_one_shot_rf(rd, path, DPK_LBK_RXIQK);

	ax52_rf_write(rd, path, RF_RXK, BIT(5), 0);
	ax52_bb_write_mask(rd, KIP_MDPK_RXDCK, BIT(31), 0);
	ax52_bb_write_mask(rd, KIP_KPATH_CFG, GENMASK(21, 20), 0);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_LOAD_COEF), BIT(1), 1);
	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, 0x5);
}

/* Capture sync check: false when correlation or DC are out of range. */
static bool dpk_sync(struct ax52_dev *rd, u8 path)
{
	u32 corr, v;
	int dc_i, dc_q;

	dpk_one_shot(rd, path, DPK_SYNC);
	ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(21, 16), 0);
	corr = ax52_bb_read_mask(rd, KIP_RPT, GENMASK(15, 8));
	ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(21, 16), 9);
	v = ax52_bb_read(rd, KIP_RPT);
	dc_i = abs(sign_extend32(FIELD_GET(GENMASK(27, 16), v), 11));
	dc_q = abs(sign_extend32(FIELD_GET(GENMASK(11, 0), v), 11));

	if (dc_i > 200 || dc_q > 200 || corr < 170) {
		ax52_dbg(rd, "DPK %c: sync failed (corr %u, DC %d/%d)\n",
			 path_name(path), corr, dc_i, dc_q);
		return false;
	}
	return true;
}

/* RX BB gain step for a digital gain report. */
static int dpk_dgain_step(u32 dgain)
{
	static const u16 bnd[] = {
		0xbf1, 0xaa5, 0x97d, 0x875, 0x789, 0x6b7, 0x5fc, 0x556,
		0x4c1, 0x43d, 0x3c7, 0x35e, 0x2ac, 0x262, 0x220,
	};
	static const s8 step[] = { 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7 };
	int i;

	for (i = 0; i < ARRAY_SIZE(bnd); i++)
		if (dgain >= bnd[i])
			return step[i];
	return -8;
}

static u32 dpk_pas_power(struct ax52_dev *rd)
{
	u32 v = ax52_bb_read(rd, KIP_RPT);
	u32 i = abs(sign_extend32(v >> 16, 11));
	u32 q = abs(sign_extend32(v & 0xffff, 11));

	return i * i + q * q;
}

/* PA AM/AM: is the first sample's power >= 1.6x the last one's? */
static bool dpk_pas_compressed(struct ax52_dev *rd)
{
	u32 first, last;

	ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(23, 16), 0x06);
	ax52_bb_write_mask(rd, KIP_DPK_CFG2, BIT(14), 0);
	ax52_bb_write_mask(rd, KIP_DPK_CFG3, GENMASK(23, 16), 0x08);
	ax52_bb_write_mask(rd, KIP_DPK_CFG3, GENMASK(31, 24), 0x00);
	first = dpk_pas_power(rd);
	ax52_bb_write_mask(rd, KIP_DPK_CFG3, GENMASK(31, 24), 0x1f);
	last = dpk_pas_power(rd);
	return first >= last * 8 / 5;
}

static u8 dpk_gain_loss(struct ax52_dev *rd, u8 path)
{
	ax52_bb_write_mask(rd, KIP_P(path, KIP_DPD_CH0), GENMASK(31, 24), 0x90);
	dpk_one_shot(rd, path, DPK_GAIN_LOSS);
	ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(21, 16), 6);
	ax52_bb_write_mask(rd, KIP_DPK_CFG2, BIT(14), 1);
	return ax52_bb_read_mask(rd, KIP_RPT, GENMASK(7, 4));
}

/*
 * Find the RX gain that fits the capture and the TX gain at which the PA
 * shows the target compression. Returns the TX AGC, or DPK_TXAGC_FAIL.
 */
static u8 dpk_agc(struct ax52_dev *rd, u8 path)
{
	enum { SYNC, RX_GAIN, GAIN_LOSS, TOO_HIGH, TOO_LOW, SET_TX } step = SYNC;
	u8 txagc = DPK_TXAGC_INIT, gl = 0;
	bool limited = false, done = false;
	int rxbb, ofs, adj = 0, guard = 200;
	u32 dgain = 0;

	while (!done && adj < 6 && guard--) {
		switch (step) {
		case SYNC:
			if (!dpk_sync(rd, path))
				return DPK_TXAGC_FAIL;
			ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(21, 16), 0);
			dgain = ax52_bb_read_mask(rd, KIP_RPT, GENMASK(27, 16));
			step = limited ? GAIN_LOSS : RX_GAIN;
			break;
		case RX_GAIN:
			ofs = dpk_dgain_step(dgain);
			rxbb = (int)ax52_rf_read(rd, path, RF_MOD, RF_MOD_RXBB) + ofs;
			if (rxbb < 0 || rxbb > 0x1f) {
				rxbb = clamp(rxbb, 0, 0x1f);
				limited = true;
			}
			ax52_rf_write(rd, path, RF_MOD, RF_MOD_RXBB, rxbb);
			if (ofs || !adj) {
				if (rd->chan.bw == AX52_BW_80) {
					dpk_lbk_rxiqk(rd, path);
				} else {
					/* bypass RX IQ correction */
					ax52_bb_write_mask(rd, KIP_P(path, KIP_RXIQC), BIT(2), 1);
					ax52_bb_write_mask(rd, KIP_P(path, KIP_RXIQC), BIT(0), 1);
				}
			}
			step = dgain > 1922 || dgain < 342 ? SYNC : GAIN_LOSS;
			adj++;
			break;
		case GAIN_LOSS:
			gl = dpk_gain_loss(rd, path);
			if ((!gl && dpk_pas_compressed(rd)) || gl >= 7)
				step = TOO_HIGH;
			else if (!gl)
				step = TOO_LOW;
			else
				step = SET_TX;
			break;
		case TOO_HIGH:
			if (txagc == DPK_TXAGC_MIN)
				done = true;
			else
				txagc = dpk_adjust_txagc(rd, path, 3);
			step = GAIN_LOSS;
			adj++;
			break;
		case TOO_LOW:
			if (txagc == DPK_TXAGC_MAX)
				done = true;
			else
				txagc = dpk_adjust_txagc(rd, path, -2);
			step = GAIN_LOSS;
			adj++;
			break;
		case SET_TX:
			txagc = dpk_adjust_txagc(rd, path, gl);
			done = true;
			break;
		}
	}
	return txagc;
}

static u8 dpk_order(struct ax52_dev *rd)
{
	return 0x3 >> ax52_bb_read_mask(rd, KIP_LDL_NORM, GENMASK(1, 0));
}

/* Identify the PA model: order 2 for 5 GHz below 80 MHz, else order 0. */
static void dpk_idl(struct ax52_dev *rd, u8 path)
{
	bool ord2 = rd->chan.band == AX52_BAND_5G && rd->chan.bw != AX52_BW_80;

	ax52_bb_write_mask(rd, KIP_LDL_NORM, GENMASK(1, 0), ord2 ? 2 : 0);
	ax52_bb_write_mask(rd, KIP_LDL_NORM, GENMASK(12, 8), ord2 ? 0 : 3);
	ax52_bb_write_mask(rd, KIP_MDPK_SYNC, GENMASK(31, 28), ord2 ? 0 : 1);
	dpk_one_shot(rd, path, DPK_MDPK_IDL);
}

static void dpk_fill(struct ax52_dev *rd, u8 path, u8 txagc)
{
	struct ax52_rfk *rk = rd->rfk_priv;

	ax52_bb_write_mask(rd, KIP_P(path, KIP_COEF_SEL), BIT(8), 0);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_TXAGC_RFK), GENMASK(13, 8), txagc);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_DPD_BND), GENMASK(24, 16), DPK_PWSF);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_LOAD_COEF), BIT(16), 1);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_LOAD_COEF), BIT(16), 0);
	ax52_bb_write(rd, KIP_P(path, KIP_DPD_CFG),
		      rk->dpk_gs == 0x7f ? 0x007f7f7f : 0x005b5b5b);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_DPD_CFG), GENMASK(26, 25),
			   dpk_order(rd));
	ax52_bb_write(rd, KIP_P(path, KIP_DPD_V1), 0);
	ax52_bb_write_mask(rd, KIP_MDPK_SYNC, BIT(31), 0);
}

static void dpk_onoff(struct ax52_dev *rd, u8 path, bool on)
{
	ax52_bb_write_mask(rd, KIP_P(path, KIP_DPD_CFG), GENMASK(31, 24),
			   dpk_order(rd) << 1 | on);
}

static bool dpk_path(struct ax52_dev *rd, u8 path)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	struct rfk_dpk *d = &rk->dpk[path];
	u8 bw = rd->chan.bw;
	u8 txagc;

	rfk_rf_by_bb(rd, path, false);
	/* KIP power and clock */
	ax52_bb_write(rd, NCTL_RPT, 0x00000080);
	ax52_bb_write(rd, KIP_SYSCFG, 0x807f030a);
	ax52_bb_write(rd, KIP_P(path, KIP_CFIR_SYS), 0xce000a08);

	dpk_set_txagc(rd, path, DPK_TXAGC_INIT);
	dpk_rf_setup(rd, path);
	ax52_rf_write(rd, path, RF_RXBB2, GENMASK(11, 10), 3);
	rfk_rx_dck_kick(rd, path);

	/* test pattern bandwidth */
	ax52_bb_write_mask(rd, 0x806c, GENMASK(2, 1),
			   bw == AX52_BW_80 ? 0 : bw == AX52_BW_40 ? 2 : 1);
	dpk_one_shot(rd, path, DPK_KIP_PRESET);

	ax52_bb_write_mask(rd, KIP_MOD, GENMASK(19, 0),
			   ax52_rf_read(rd, path, RF_MOD, RFREG_MASK));
	dpk_one_shot_rf(rd, path, DPK_RXAGC);
	ax52_bb_write_mask(rd, KIP_RPT_SEL, GENMASK(19, 16), 8);
	ax52_bb_write_mask(rd, KIP_P(path, KIP_DPD_CH0), GENMASK(31, 24), 0x90);

	txagc = dpk_agc(rd, path);
	if (txagc == DPK_TXAGC_FAIL)
		return false;

	d->ther = rfk_thermal_rf(rd, path);
	dpk_idl(rd, path);
	ax52_rf_write(rd, path, RF_MOD, RF_MOD_MODE, MODE_RX);
	dpk_fill(rd, path, txagc);
	ax52_dbg(rd, "DPK %c ch %u: TX AGC 0x%x, thermal %u\n",
		 path_name(path), rd->chan.ch, txagc, d->ther);
	return true;
}

static void rfk_dpk(struct ax52_dev *rd)
{
	static const u16 kip_regs[] = { KIP_RXIQC, KIP_IQK_RES, KIP_CFIR_SYS };
	struct ax52_rfk *rk = rd->rfk_priv;
	u32 kip[RF_PATH_NUM][ARRAY_SIZE(kip_regs)];
	u32 bb[RFK_BAK_BB], rf[RF_PATH_NUM][RFK_BAK_RF];
	bool bw80 = rd->chan.bw == AX52_BW_80;
	u8 p, i;

	rfk_begin(rd);
	rfk_backup_bb(rd, bb);
	for (p = 0; p < RF_PATH_NUM; p++) {
		for (i = 0; i < ARRAY_SIZE(kip_regs); i++)
			kip[p][i] = ax52_bb_read(rd, KIP_P(p, kip_regs[i]));
		rfk_backup_rf(rd, p, rf[p]);
		rk->dpk[p] = (struct rfk_dpk) {
			.band = rd->chan.band, .bw = rd->chan.bw, .ch = rd->chan.ch,
		};
		if (rk->tssi_on[p])
			ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(30), 1);
	}

	rfk_apply(rd, &ax52_rfk_dpk_bb_set, 0);
	if (bw80) {
		ax52_bb_write_mask(rd, 0xc0d8, BIT(13), 1);
		ax52_bb_write_mask(rd, 0xc1d8, BIT(13), 1);
	}

	/* a failed path keeps DPD off, which is also the power-on state */
	for (p = 0; p < RF_PATH_NUM; p++) {
		rk->dpk[p].ok = dpk_path(rd, p);
		dpk_onoff(rd, p, rk->dpk[p].ok);
	}

	rfk_apply(rd, &ax52_rfk_dpk_bb_restore, 0);
	if (bw80) {
		ax52_bb_write_mask(rd, 0xc0d8, BIT(13), 0);
		ax52_bb_write_mask(rd, 0xc1d8, BIT(13), 0);
	}
	rfk_restore_bb(rd, bb);

	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write(rd, NCTL_RPT, 0);
		ax52_bb_write(rd, KIP_SYSCFG, 0x80000000);
		/* B-cut and later: TX AGC offsets applied in hardware */
		if (rd->cv > 0)
			ax52_bb_write_mask(rd, KIP_P(p, KIP_DPD_COM), BIT(15), 1);
		for (i = 0; i < ARRAY_SIZE(kip_regs); i++)
			ax52_bb_write(rd, KIP_P(p, kip_regs[i]), kip[p][i]);
		rfk_restore_rf(rd, p, rf[p]);
		if (rk->tssi_on[p])
			ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(30), 0);
	}
	rfk_end(rd);
}

/* Read the BB back-off the tables applied and pick the DPD gain scale. */
static void rfk_dpd_backoff(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u32 bkof = ax52_bb_read_mask(rd, 0x44a0, GENMASK(16, 12));
	u32 scale = ax52_bb_read_mask(rd, 0x44a0, GENMASK(6, 0));
	u8 p;

	if (bkof + scale < 44) {
		rk->dpk_gs = 0x5b;
		return;
	}
	/* the BB already backs off: run the DPD at unity gain */
	rk->dpk_gs = 0x7f;
	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_bb_write_mask(rd, KIP_P(p, KIP_DPD_CFG), GENMASK(22, 0), 0x7f7f7f);
}

/* Follow temperature with the DPD power scaling factor. */
static void rfk_dpk_track(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u32 txagc_rf, pwsf;
	s8 delta, ini;
	u8 p, cur;

	for (p = 0; p < RF_PATH_NUM; p++) {
		const struct rfk_dpk *d = &rk->dpk[p];

		if (!d->ok)
			continue;

		cur = ewma_thermal_read(&rk->thermal[p]);
		delta = cur ? d->ther - cur : 0;
		/* thermal code steps to pwsf steps */
		delta = d->band == AX52_BAND_2G ? delta * 3 / 2 : delta * 5 / 2;

		txagc_rf = ax52_bb_read_mask(rd, BB_P(p, 0x1c60), GENMASK(5, 0));
		if (rk->tssi_on[p]) {
			s8 bb = ax52_bb_read_mask(rd, BB_P(p, 0x1c60), GENMASK(23, 16));
			u32 tp = ax52_bb_read_mask(rd, BB_P(p, 0x1c04), GENMASK(2, 0));
			s8 ofst = ax52_bb_read_mask(rd, BB_P(p, 0x1c60), GENMASK(31, 24));

			/* with hardware TX AGC offsets the BB offset is already in */
			if (ax52_bb_read_mask(rd, KIP_P(p, KIP_DPD_COM), BIT(15)))
				ofst = 0;
			ini = txagc_rf && cur ? ofst + delta : 0;
			pwsf = DPK_PWSF + ini;
			if (!ax52_bb_read_mask(rd, BB_P(p, 0x58d4), GENMASK(31, 28)))
				pwsf += tp - bb;
		} else {
			pwsf = DPK_PWSF + delta;
		}

		if (ax52_bb_read_mask(rd, KIP_DPK_TRK, BIT(31)) || !txagc_rf)
			continue;
		ax52_bb_write_mask(rd, KIP_P(p, KIP_DPD_BND), GENMASK(8, 0), pwsf & 0x1ff);
		ax52_bb_write_mask(rd, KIP_P(p, KIP_DPD_BND), GENMASK(24, 16), pwsf & 0x1ff);
	}
}

/* ------------------------------------------------------------ TSSI */

static int tssi_band(const struct ax52_chan *c)
{
	if (c->band == AX52_BAND_2G)
		return 0;
	if (c->ch >= 149)
		return 3;
	if (c->ch >= 100)
		return 2;
	return 1;
}

static int tssi_ch_idx(u8 ch)
{
	if (ch >= 1 && ch <= 14)
		return ch - 1;
	if (ch >= 36 && ch <= 64)
		return (ch - 36) / 2 + 14;
	if (ch >= 100 && ch <= 144)
		return (ch - 100) / 2 + 29;
	if (ch >= 149 && ch <= 177)
		return (ch - 149) / 2 + 52;
	return -1;
}

static void tssi_efuse_parse(struct ax52_dev *rd)
{
	static const u16 base[RF_PATH_NUM] = { 0x210, 0x23a };
	static const u16 trim[RF_PATH_NUM] = { 0x5d6, 0x5ab };
	struct ax52_rfk *rk = rd->rfk_priv;
	const u8 *log = rd->efuse.log;
	bool trim_pg = false;
	u8 p, i;

	for (p = 0; p < RF_PATH_NUM; p++) {
		for (i = 0; i < 6; i++)
			rk->tssi_cck[p][i] = log[base[p] + i];
		for (i = 0; i < 5; i++)		/* 2 GHz, BW40 1S */
			rk->tssi_mcs[p][i] = log[base[p] + 6 + i];
		for (i = 0; i < 14; i++)	/* 5 GHz, BW40 1S */
			rk->tssi_mcs[p][5 + i] = log[base[p] + 0x12 + i];
		/* per-unit trims, stored downwards in the physical area */
		for (i = 0; i < 8; i++) {
			rk->tssi_trim[p][i] =
				rd->efuse.phycap[trim[p] - i - EFUSE_PHYCAP_ADDR];
			trim_pg |= (u8)rk->tssi_trim[p][i] != 0xff;
		}
	}
	if (!trim_pg)
		memset(rk->tssi_trim, 0, sizeof(rk->tssi_trim));
}

static int tssi_cck_group(u8 ch)
{
	if (ch == 14)
		return 5;
	return ch <= 13 ? ch / 3 : 0;
}

static int tssi_trim_group(u8 ch)
{
	static const u8 lo[] = { 1, 9, 36, 52, 100, 116, 132, 149 };
	static const u8 hi[] = { 8, 14, 48, 64, 112, 128, 144, 177 };
	int i;

	for (i = 0; i < ARRAY_SIZE(lo); i++)
		if (ch >= lo[i] && ch <= hi[i])
			return i;
	return 0;
}

/* OFDM DE: channel groups of 5, channels between two groups average them. */
static s8 tssi_ofdm_de(const s8 *mcs, u8 ch)
{
	static const u8 start[] = { 36, 100, 149 }, groups[] = { 4, 6, 4 };
	int s, g = 5, k, r;

	if (ch <= 14)
		return mcs[ch / 3];
	for (s = 0; s < ARRAY_SIZE(start); g += groups[s], s++) {
		if (ch < start[s] || ch >= start[s] + 8 * groups[s])
			continue;
		k = (ch - start[s]) / 8;
		r = (ch - start[s]) % 8;
		if (r <= 4)
			return mcs[g + k];
		if (k < groups[s] - 1)
			return (mcs[g + k] + mcs[g + k + 1]) / 2;
		break;
	}
	return mcs[0];
}

/* Factory TSSI power offsets ("DE") plus per-unit trim, per rate class. */
static void tssi_efuse_de(struct ax52_dev *rd)
{
	static const u16 ofdm_regs[] = { 0x5838, 0x5840, 0x5848, 0x5850, 0x5828, 0x5830 };
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 ch = rd->chan.ch;
	u32 v;
	s8 trim;
	u8 p, i;

	for (p = 0; p < RF_PATH_NUM; p++) {
		trim = rk->tssi_trim[p][tssi_trim_group(ch)];
		v = (rk->tssi_cck[p][tssi_cck_group(ch)] + trim) & 0x3ff;
		ax52_bb_write_mask(rd, BB_P(p, 0x5858), GENMASK(21, 12), v);
		ax52_bb_write_mask(rd, BB_P(p, 0x5860), GENMASK(21, 12), v);

		v = (tssi_ofdm_de(rk->tssi_mcs[p], ch) + trim) & 0x3ff;
		for (i = 0; i < ARRAY_SIZE(ofdm_regs); i++)
			ax52_bb_write_mask(rd, BB_P(p, ofdm_regs[i]), GENMASK(21, 12), v);
	}
}

static void tssi_disable(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 p;

	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_EN), BIT(31), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), GENMASK(28, 27), 1);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_MV_AVG), BIT(14), 1);
		rk->tssi_on[p] = false;
	}
}

static void tssi_enable(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 p;

	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write_mask(rd, BB_P(p, 0x5814), BIT(11), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_MV_AVG), GENMASK(19, 11), 0x010);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_MV_AVG), BIT(14), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_EN), BIT(31), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_EN), BIT(31), 1);
		ax52_rf_write(rd, p, RF_TXGA_V1, BIT(7), 1);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), GENMASK(28, 27), 3);
		/* default TX AGC offset, then strobe it in */
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), GENMASK(7, 0), 0xc0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(28), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(28), 1);
		rk->tssi_on[p] = true;
	}
}

static void tssi_sys(struct ax52_dev *rd, u8 path)
{
	bool is2g = rd->chan.band == AX52_BAND_2G;

	ax52_rf_write(rd, path, RF_TXPOW, is2g ? BIT(1) : BIT(8), 1);
	rfk_apply(rd, &ax52_rfk_tssi_sys, path);
	rfk_apply(rd, is2g ? &ax52_rfk_tssi_sys_2g : &ax52_rfk_tssi_sys_5g, path);
}

/*
 * Thermal-offset table the TSSI hardware indexes with the 6-bit (two's
 * complement) difference to the factory thermal code.
 */
static void tssi_thermal_table(struct ax52_dev *rd, u8 path, int band)
{
	const s8 *up = ax52_rfk_tssi_swing[path][band][0];
	const s8 *down = ax52_rfk_tssi_swing[path][band][1];
	u8 th = rd->efuse.thermal[path];
	u32 v;
	int i, j, k;
	s8 o;

	ax52_bb_write_mask(rd, BB_P(path, BB_TSSI_TMETER), BIT(16), 0);
	ax52_bb_write_mask(rd, BB_P(path, BB_TSSI_TMETER), BIT(24), 1);
	/* no factory value: centre the meter and apply no offsets */
	ax52_bb_write_mask(rd, BB_P(path, BB_TSSI_TMETER), GENMASK(15, 10),
			   th == 0xff ? 32 : th);
	ax52_bb_write_mask(rd, BB_P(path, BB_P0_RFCTM), GENMASK(25, 20),
			   th == 0xff ? 32 : th);

	for (i = 0; i < 64; i += 4) {
		v = 0;
		for (j = 0; j < 4 && th != 0xff; j++) {
			k = i + j;
			o = k < 32 ? -down[min(k, TSSI_SWING_NUM - 1)] :
				     up[min(64 - k, TSSI_SWING_NUM - 1)];
			v |= (u32)(u8)o << (8 * j);
		}
		ax52_bb_write(rd, BB_P(path, 0x5c00) + i, v);
	}

	ax52_bb_write_mask(rd, BB_P(path, BB_P0_RFCTM), BIT(26), 1);
	ax52_bb_write_mask(rd, BB_P(path, BB_P0_RFCTM), BIT(26), 0);
}

static void tssi_dac_gain(struct ax52_dev *rd, u8 path)
{
	int i;

	ax52_bb_write_mask(rd, BB_P(path, 0x58b0), GENMASK(11, 0), 0);
	ax52_bb_write_mask(rd, BB_P(path, 0x58b0), BIT(11), 1);
	for (i = 0; i <= 0xc0; i += 4)
		ax52_bb_write(rd, BB_P(path, 0x5a00) + i, 0);
}

static void tssi_align_default(struct ax52_dev *rd, u8 path, int band)
{
	const u32 *v = ax52_rfk_tssi_align_def[path][band];

	ax52_bb_write_mask(rd, BB_P(path, 0x5604), BIT(31), 1);
	ax52_bb_write_mask(rd, BB_P(path, 0x5600), GENMASK(29, 0), 0x3f2d2721);
	ax52_bb_write_mask(rd, BB_P(path, 0x5604), GENMASK(21, 0), 0x010101);
	ax52_bb_write_mask(rd, BB_P(path, 0x5630), GENMASK(29, 0), v[0]);
	ax52_bb_write_mask(rd, BB_P(path, 0x5634), GENMASK(29, 0), v[1]);
	ax52_bb_write_mask(rd, BB_P(path, 0x5638), GENMASK(19, 0), 0);
	ax52_bb_write_mask(rd, BB_P(path, 0x563c), GENMASK(29, 0), v[2]);
	ax52_bb_write_mask(rd, BB_P(path, 0x5640), GENMASK(29, 0), v[3]);
	ax52_bb_write_mask(rd, BB_P(path, 0x5644), GENMASK(19, 0), 0);
}

static void tssi_align_write(struct ax52_dev *rd, u8 path, const u32 *v)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tssi_align_regs); i++)
		ax52_bb_write(rd, BB_P(path, tssi_align_regs[i]), v[i]);
}

/* Re-apply the last alignment measured in the current band, if any. */
static void tssi_align_reload(struct ax52_dev *rd, u8 path)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	int band = tssi_band(&rd->chan);

	if (rk->align_done[path][band])
		tssi_align_write(rd, path, rk->align[path][band]);
}

/* ------------------------------------ PMAC test transmission for TSSI */

#define PMAC_PATH_KEEP		-1	/* leave TX/RX path setup alone */

/* BT-shared-antenna RX setup that the PMAC RX path configuration touches. */
static void pmac_btg(struct ax52_dev *rd, bool en)
{
	ax52_bb_write_mask(rd, 0x4738, BIT(19), en);
	ax52_bb_write_mask(rd, 0x4738, BIT(22), 0);
	ax52_bb_write_mask(rd, 0x476c, GENMASK(31, 24), en ? 0x20 : 0x1a);
	ax52_bb_write_mask(rd, 0x4778, GENMASK(7, 0), en ? 0x30 : 0x2a);
	ax52_bb_write_mask(rd, 0x4aa4, BIT(19), en);
	ax52_bb_write_mask(rd, 0x4aa4, BIT(22), en);
	ax52_bb_write_mask(rd, 0x0980, GENMASK(20, 17), en ? 0 : 0xc);
	ax52_bb_write_mask(rd, 0x49c4, BIT(14), en);
	ax52_bb_write_mask(rd, 0x49c0, GENMASK(25, 22), en ? 2 : 0);
	ax52_bb_write_mask(rd, 0x4420, BIT(31), 1);
	ax52_bb_write_mask(rd, 0x0c6c, BIT(21), en);
}

/*
 * RX path for the test transmission. The per-subband RX gain offsets that
 * the vendor code also re-applies here are left as phy.c set them.
 */
static void pmac_rx_path(struct ax52_dev *rd, int path)
{
	u32 rst = path == RF_PATH_A ? 0x58dc : 0x78dc;

	if (path != PMAC_PATH_KEEP) {
		ax52_bb_write_mask(rd, 0x49c4, GENMASK(3, 0), BIT(path));
		ax52_bb_write_mask(rd, 0x49c0, GENMASK(17, 14), BIT(path));
		ax52_bb_write_mask(rd, 0x49c0, GENMASK(21, 18), BIT(path));
		ax52_bb_write_mask(rd, 0x0d18, GENMASK(9, 8), 0);
		ax52_bb_write_mask(rd, 0x0d18, GENMASK(22, 21), 0);
		ax52_bb_write_mask(rd, 0x0d80, GENMASK(13, 6), 4);
		ax52_bb_write_mask(rd, 0x0d80, GENMASK(16, 14), 0);
		ax52_bb_write_mask(rd, 0x0d80, GENMASK(25, 23), 0);
	}
	pmac_btg(rd, rd->chan.band == AX52_BAND_2G && path == RF_PATH_B);
	/* TX power control reset pulse */
	ax52_bb_write_mask(rd, rst, GENMASK(31, 30), 1);
	ax52_bb_write_mask(rd, rst, GENMASK(31, 30), 3);
}

/* Send 100 HT20 MCS7 packets, one per 5000 us, at power code @pwr. */
static void pmac_tx_start(struct ax52_dev *rd, int path, u32 pwr)
{
	rfk_apply(rd, &ax52_rfk_pmac_ht20_mcs7, 0);

	ax52_bb_write_mask(rd, 0x09a4, GENMASK(4, 2), 7);	/* PMAC drives TX */
	if (path != PMAC_PATH_KEEP) {
		ax52_bb_write_mask(rd, 0x458c, GENMASK(31, 28), BIT(path));
		ax52_bb_write_mask(rd, 0x45b4, GENMASK(20, 17), 0);
	}
	pmac_rx_path(rd, path);
	ax52_bb_write_mask(rd, 0x09a4, BIT(16), 1);
	ax52_bb_write_mask(rd, 0x4594, GENMASK(30, 22), pwr);

	ax52_bb_write_mask(rd, 0x0980, BIT(0), 1);
	ax52_bb_write_mask(rd, 0x0980, BIT(16), 1);
	ax52_bb_write_mask(rd, 0x0988, GENMASK(11, 0), 0x3f);
	ax52_bb_write_mask(rd, 0x0704, BIT(1), 0);
	ax52_bb_write_mask(rd, 0x0c3c, BIT(9), 1);		/* no packet detect */
	ax52_bb_write_mask(rd, 0x2344, BIT(31), 1);		/* no CCA */
	ax52_bb_write_mask(rd, 0x0704, BIT(1), 1);

	ax52_bb_write_mask(rd, 0x09c4, BIT(4), 1);
	ax52_bb_write_mask(rd, 0x09c4, GENMASK(31, 8), 5000);
	ax52_bb_write(rd, 0x09c8, 100);
	ax52_bb_write_mask(rd, 0x09c0, BIT(0), 1);
	ax52_bb_write_mask(rd, 0x09c0, BIT(0), 0);
}

static void pmac_tx_stop(struct ax52_dev *rd)
{
	ax52_bb_write_mask(rd, 0x09c4, BIT(4), 0);
	ax52_bb_write_mask(rd, 0x0c3c, BIT(9), 0);
	if (rd->chan.band == AX52_BAND_2G)
		ax52_bb_write_mask(rd, 0x2344, BIT(31), 0);
}

/* Full dwords restored after the measurement. */
static const u16 tssi_meas_regs[] = {
	0x5820, 0x7820, 0x4978, 0x58e4, 0x78e4, 0x49c0, 0x0d18, 0x0d80,
	/*
	 * BT-shared-antenna RX setup: the vendor flow leaves it as the last
	 * PMAC RX path left it; we put back what phy.c/coex had set.
	 */
	0x4738, 0x476c, 0x4778, 0x4aa4, 0x0c6c, 0x49c4, 0x0980,
};

struct tssi_meas_bak {
	u32 regs[ARRAY_SIZE(tssi_meas_regs)];
	u32 rfmode[4];		/* 0x12ac, 0x12b0, 0x32ac, 0x32b0 */
	u32 tx_path, rx_path, tx_pwr;
};

static const u16 tssi_rfmode_regs[4] = { 0x12ac, 0x12b0, 0x32ac, 0x32b0 };

static void tssi_meas_save(struct ax52_dev *rd, struct tssi_meas_bak *b)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tssi_meas_regs); i++)
		b->regs[i] = ax52_bb_read(rd, tssi_meas_regs[i]);
	for (i = 0; i < ARRAY_SIZE(tssi_rfmode_regs); i++)
		b->rfmode[i] = ax52_bb_read(rd, tssi_rfmode_regs[i]);
	b->tx_path = ax52_bb_read_mask(rd, 0x458c, GENMASK(31, 28));
	b->rx_path = ax52_bb_read_mask(rd, 0x49c4, GENMASK(3, 0));
	b->tx_pwr = ax52_bb_read_mask(rd, 0x4594, GENMASK(30, 22));
}

static void tssi_meas_restore(struct ax52_dev *rd, const struct tssi_meas_bak *b)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tssi_meas_regs); i++)
		ax52_bb_write(rd, tssi_meas_regs[i], b->regs[i]);

	ax52_bb_write_mask(rd, 0x458c, GENMASK(31, 28), b->tx_path);
	ax52_bb_write_mask(rd, 0x45b4, GENMASK(20, 17), b->tx_path == 3 ? 4 : 0);
	ax52_bb_write_mask(rd, 0x49c4, GENMASK(3, 0), b->rx_path);
	ax52_bb_write_mask(rd, 0x09a4, BIT(16), 1);
	for (i = 0; i < ARRAY_SIZE(tssi_rfmode_regs); i++)
		ax52_bb_write(rd, tssi_rfmode_regs[i], b->rfmode[i]);
	ax52_bb_write_mask(rd, 0x4594, GENMASK(30, 22), b->tx_pwr);

	/* back to MAC-driven TX */
	ax52_bb_write_mask(rd, 0x0980, BIT(0), 0);
	ax52_bb_write_mask(rd, 0x0980, BIT(16), 0);
	ax52_bb_write_mask(rd, 0x0988, GENMASK(11, 0), 0);
	ax52_bb_write_mask(rd, 0x0994, GENMASK(7, 4), 0);
	ax52_bb_write_mask(rd, 0x09a4, BIT(10), 0);
	ax52_bb_write_mask(rd, 0x09a4, GENMASK(4, 2), 0);
	ax52_bb_write_mask(rd, 0x09a4, BIT(16), 0);
}

/*
 * Measure the TSSI code word at two PMAC powers and shift the alignment
 * words so the measured slope matches the commanded one.
 */
static bool tssi_align_measure(struct ax52_dev *rd, u8 path, u32 *out)
{
	static const int power[2] = { 48, 20 };
	u32 rpt = BB_P(path, BB_TSSI_CW_RPT), alim = BB_P(path, 0x5630);
	struct tssi_meas_bak bak;
	s32 cw[2], d1, o1, o2, o3, diff;
	u32 v;
	bool ok = true;
	int j;

	tssi_meas_save(rd, &bak);
	ax52_bb_write_mask(rd, 0x5820, GENMASK(15, 12), 8);
	ax52_bb_write_mask(rd, 0x7820, GENMASK(15, 12), 8);
	ax52_bb_write_mask(rd, 0x58e4, GENMASK(13, 11), 2);
	ax52_bb_write_mask(rd, 0x78e4, GENMASK(13, 11), 2);

	for (j = 0; j < 2 && ok; j++) {
		ax52_bb_write_mask(rd, BB_P(path, BB_TSSI_EN), BIT(31), 0);
		ax52_bb_write_mask(rd, BB_P(path, BB_TSSI_EN), BIT(31), 1);
		pmac_tx_start(rd, j ? PMAC_PATH_KEEP : path, power[j]);
		ok = !read_poll_timeout(ax52_bb_read_mask, v, v, 30, 3000, false,
					rd, rpt, BIT(16));
		if (ok)
			cw[j] = ax52_bb_read_mask(rd, rpt, GENMASK(8, 0));
		pmac_tx_stop(rd);
	}

	if (ok) {
		d1 = sign_extend32(ax52_bb_read_mask(rd, alim, GENMASK(29, 20)), 8);
		o1 = cw[0] - (power[0] - power[1]) * 2 - cw[1] + d1;
		diff = o1 - d1;
		o2 = sign_extend32(ax52_bb_read_mask(rd, alim, GENMASK(19, 10)), 8) + diff;
		o3 = sign_extend32(ax52_bb_read_mask(rd, alim, GENMASK(9, 0)), 8) + diff;
		v = FIELD_PREP(GENMASK(29, 20), o1 & 0x3ff) |
		    FIELD_PREP(GENMASK(19, 10), o2 & 0x3ff) | (o3 & 0x3ff);
		ax52_bb_write_mask(rd, alim, GENMASK(29, 0), v);
		ax52_bb_write_mask(rd, BB_P(path, 0x563c), GENMASK(29, 0), v);
		for (j = 0; j < ARRAY_SIZE(tssi_align_regs); j++)
			out[j] = ax52_bb_read(rd, BB_P(path, tssi_align_regs[j]));
		ax52_dbg(rd, "TSSI %c ch %u: CW %d/%d, alignment 0x%08x\n",
			 path_name(path), rd->chan.ch, cw[0], cw[1], v);
	} else {
		ax52_dbg(rd, "TSSI %c ch %u: no report, default alignment kept\n",
			 path_name(path), rd->chan.ch);
	}

	tssi_meas_restore(rd, &bak);
	return ok;
}

static void tssi_align(struct ax52_dev *rd, u8 path)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	int band = tssi_band(&rd->chan), idx = tssi_ch_idx(rd->chan.ch);
	u32 v[ARRAY_SIZE(tssi_align_regs)];
	bool ok;

	/* measured before on this channel: the result is still valid */
	if (idx >= 0 && rk->align_ch_done[path][idx]) {
		tssi_align_write(rd, path, rk->align_ch[path][idx]);
		return;
	}

	rfk_begin(rd);
	ok = tssi_align_measure(rd, path, v);
	rfk_end(rd);
	if (!ok)
		return;

	rk->align_done[path][band] = true;
	memcpy(rk->align[path][band], v, sizeof(v));
	if (idx >= 0) {
		rk->align_ch_done[path][idx] = true;
		memcpy(rk->align_ch[path][idx], v, sizeof(v));
	}
}

/* Full TSSI setup on the current channel, with PMAC alignment. */
static void rfk_tssi(struct ax52_dev *rd)
{
	bool is2g = rd->chan.band == AX52_BAND_2G;
	int band = tssi_band(&rd->chan);
	u8 p;

	tssi_disable(rd);
	for (p = 0; p < RF_PATH_NUM; p++) {
		tssi_sys(rd, p);
		rfk_apply(rd, &ax52_rfk_tssi_txpwr, p);
		rfk_apply(rd, &ax52_rfk_tssi_he_tb, p);
		rfk_apply(rd, &ax52_rfk_tssi_dck, p);
		tssi_thermal_table(rd, p, band);
		tssi_dac_gain(rd, p);
		rfk_apply(rd, is2g ? &ax52_rfk_tssi_slope_2g :
				     &ax52_rfk_tssi_slope_5g, p);
		tssi_align_default(rd, p, band);
		rfk_apply(rd, &ax52_rfk_tssi_slope_en, p);
		tssi_align(rd, p);
	}
	tssi_enable(rd);
	tssi_efuse_de(rd);
}

/* Band switch: re-arm TSSI from stored (or default) alignment, no TX. */
static void rfk_tssi_band(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	int band = tssi_band(&rd->chan);
	u8 p;

	tssi_disable(rd);
	for (p = 0; p < RF_PATH_NUM; p++) {
		tssi_sys(rd, p);
		tssi_thermal_table(rd, p, band);
		if (rk->align_done[p][band])
			tssi_align_write(rd, p, rk->align[p][band]);
		else
			tssi_align_default(rd, p, band);
	}
	tssi_enable(rd);
	tssi_efuse_de(rd);
}

/* ------------------------------------------------ synthesizer lock */

/* Write the path-A channel word and wait for the LO to settle. */
static void rfk_set_s0_rf18(struct ax52_dev *rd, u32 val)
{
	u32 ldo, busy;

	ldo = ax52_rf_read(rd, RF_PATH_A, RF_LDO, RFREG_MASK);
	ax52_rf_write(rd, RF_PATH_A, RF_LDO, GENMASK(8, 6), 1);
	ax52_rf_write(rd, RF_PATH_A, RF_CFGCH, RFREG_MASK, val);
	if (read_poll_timeout_atomic(ax52_rf_read, busy, !busy, 1, 1000, false,
				     rd, RF_PATH_A, RF_LPF, BIT(8)))
		ax52_dbg(rd, "LCK: synthesizer busy\n");
	ax52_rf_write(rd, RF_PATH_A, RF_LDO, RFREG_MASK, ldo);
}

static void rfk_lck_retrigger(struct ax52_dev *rd)
{
	ax52_rf_write(rd, RF_PATH_A, RF_LCK_TRG, BIT(8), 1);
	rfk_set_s0_rf18(rd, ax52_rf_read(rd, RF_PATH_A, RF_CFGCH, RFREG_MASK));
	ax52_rf_write(rd, RF_PATH_A, RF_LCK_TRG, BIT(8), 0);
}

static bool rfk_synth_locked(struct ax52_dev *rd)
{
	return ax52_rf_read(rd, RF_PATH_A, RF_SYNFB, BIT(15));
}

/*
 * Escalating recovery when the shared synthesizer did not lock. Called by
 * phy.c right after the path-A channel write, with TX still paused.
 */
void ax52_rfk_lck_check(struct ax52_dev *rd)
{
	u32 v;

	if (!rfk_synth_locked(rd)) {
		/* reset the multi-modulus divider */
		ax52_rf_write(rd, RF_PATH_A, RF_MMD, BIT(8), 1);
		ax52_rf_write(rd, RF_PATH_A, RF_MMD, BIT(6), 0);
		ax52_rf_write(rd, RF_PATH_A, RF_MMD, BIT(6), 1);
		ax52_rf_write(rd, RF_PATH_A, RF_MMD, BIT(8), 0);
	}
	fsleep(10);

	if (!rfk_synth_locked(rd))
		rfk_lck_retrigger(rd);

	if (!rfk_synth_locked(rd)) {
		/* power-cycle the synthesizer */
		v = ax52_rf_read(rd, RF_PATH_A, RF_POW, RFREG_MASK);
		ax52_rf_write(rd, RF_PATH_A, RF_POW, RFREG_MASK, v);
		v = ax52_rf_read(rd, RF_PATH_A, RF_SX, RFREG_MASK);
		ax52_rf_write(rd, RF_PATH_A, RF_SX, RFREG_MASK, v);
		ax52_rf_write(rd, RF_PATH_A, RF_SYNLUT, BIT(4), 1);
		ax52_rf_write(rd, RF_PATH_A, RF_POW, GENMASK(3, 2), 0);
		ax52_rf_write(rd, RF_PATH_A, RF_POW, GENMASK(3, 2), 3);
		ax52_rf_write(rd, RF_PATH_A, RF_SYNLUT, BIT(4), 0);
		rfk_lck_retrigger(rd);

		if (!rfk_synth_locked(rd))
			ax52_warn(rd, "synthesizer not locked on channel %u\n",
				  rd->chan.ch);
	}
}

/* ------------------------------------------------------ contract */

int ax52_rfk_alloc(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = kzalloc(sizeof(*rk), GFP_KERNEL);

	if (!rk)
		return -ENOMEM;
	rd->rfk_priv = rk;
	return 0;
}

void ax52_rfk_free(struct ax52_dev *rd)
{
	kfree(rd->rfk_priv);
	rd->rfk_priv = NULL;
}

int ax52_rfk_init(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 p;

	/* the chip was just powered up; alignment results stay valid */
	rk->last_band = RFK_BAND_NONE;
	memset(rk->tssi_on, 0, sizeof(rk->tssi_on));
	memset(rk->dpk, 0, sizeof(rk->dpk));
	tssi_efuse_parse(rd);

	for (p = 0; p < RF_PATH_NUM; p++)
		ewma_thermal_init(&rk->thermal[p]);
	rfk_thermal_update(rd);

	rfk_nctl_init(rd);
	rfk_dpd_backoff(rd);
	rfk_rck(rd, RF_PATH_A);
	rfk_rck(rd, RF_PATH_B);
	rfk_dack(rd);
	rfk_rx_dck(rd);
	return 0;
}

void ax52_rfk_channel(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;

	/* first channel or band switch: TSSI for the new band, no TX */
	if (rk->last_band != rd->chan.band) {
		rk->last_band = rd->chan.band;
		rfk_tssi_band(rd);
	}
}

void ax52_rfk_sta_connect(struct ax52_dev *rd)
{
	struct ax52_rfk *rk = rd->rfk_priv;

	rfk_rx_dck(rd);
	if (rk->nctl_ok)
		rfk_iqk(rd);
	rfk_tssi(rd);
	if (rk->nctl_ok)
		rfk_dpk(rd);
}

void ax52_rfk_scan(struct ax52_dev *rd, bool start)
{
	struct ax52_rfk *rk = rd->rfk_priv;
	u8 p;

	if (start) {
		if (!rk->tssi_on[RF_PATH_A] && !rk->tssi_on[RF_PATH_B])
			rfk_tssi(rd);
		return;
	}

	/* scan end: default TX AGC offset, stored alignment of this band */
	for (p = 0; p < RF_PATH_NUM; p++)
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), GENMASK(7, 0), 0xc0);
	for (p = 0; p < RF_PATH_NUM; p++) {
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(28), 0);
		ax52_bb_write_mask(rd, BB_P(p, BB_TSSI_TRK), BIT(28), 1);
	}
	for (p = 0; p < RF_PATH_NUM; p++)
		tssi_align_reload(rd, p);
}

/* Not called while scanning. */
void ax52_rfk_track(struct ax52_dev *rd)
{
	rfk_thermal_update(rd);
	rfk_dpk_track(rd);
}
