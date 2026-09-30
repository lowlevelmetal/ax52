// SPDX-License-Identifier: GPL-2.0
/*
 * TX power: per-rate target powers, regulatory limits (per rate section and
 * per HE resource unit), TX spectral shaping and the BB reference power.
 *
 * The tables come from the firmware file, in 1/4 dB; the MAC power
 * registers take 1/2 dB. A missing table falls back to a fixed, conservative
 * power rather than to the hardware maximum.
 */
#include <linux/unaligned.h>

#include "phy.h"

/* MAC TX-power registers */
#define REG_PWR_RATE_CTRL	0xD200
#define   PWR_REF_MASK		GENMASK(27, 10)
#define REG_PWR_RATE_OFST	0xD204	/* [19:0]: 4-bit offsets per rate section */
#define REG_PWR_COEXT_CTRL	0xD220
#define REG_PWR_UL_CTRL0	0xD240
#define REG_PWR_UL_TB_CTRL	0xD288
#define   PWR_UL_TB_EN		BIT(31)
#define REG_PWR_UL_TB_1T	0xD28C
#define REG_PWR_UL_TB_2T	0xD290
#define   PWR_UL_TB_OFST	GENMASK(4, 0)
#define REG_PWR_BY_RATE		0xD2C0	/* 44 bytes */
#define REG_PWR_LMT		0xD2EC	/* 1TX page, then 2TX page */
#define REG_PWR_RU_LMT		0xD33C	/* same */
#define PWR_BYR_LEN		44
#define PWR_LMT_PAGE		40
#define PWR_RU_LMT_PAGE		24

/* BB */
#define BB_TXFIR		0x2300	/* 8 dwords: CCK TX filter */
#define   TXSHAPE_TRI		GENMASK(25, 24)	/* in BB_DCFO_OPT */
#define BB_P0_TXPWR_REF		0x5804	/* OFDM; CCK at +4, path B at +0x2000 */
#define   TXPWR_REF_MASK	GENMASK(26, 0)

/*
 * Reference power word: TSSI offset 0x12C - 16 dBm = 0xAC, power code
 * 0x27 << 3 (0 dBm base), reference 0; the same for both paths as there is
 * no antenna-gain or SAR difference between them.
 */
#define TXPWR_REF_CW		(FIELD_PREP(GENMASK(26, 18), 0xAC) | \
				 FIELD_PREP(GENMASK(17, 9), 0x27 << 3))

#define TXPWR_MAC_MAX		63		/* 31.5 dBm, register range */
#define TXPWR_FALLBACK		(10 * 4)	/* 10 dBm, if a table is missing */

/* regulation indices used by the tables */
#define REGD_WW			0
#define REGD_ETSI		1
#define REGD_NA			4
#define REGD_UK			14

enum { RS_CCK, RS_OFDM, RS_MCS, RS_HEDCM, RS_OFFSET };

/* Table entry layouts. Entries may be shorter, or longer if zero-padded. */
struct byr_ent {
	u8 band;
	u8 nss;
	u8 rs;
	u8 shf;
	u8 len;
	__le32 data;
	u8 bw;
	u8 ofdma;
} __packed;

struct lmt_ent {
	u8 bw;
	u8 ntx;
	u8 rs;
	u8 bf;
	u8 regd;
	u8 ch;
	s8 v;
} __packed;

struct lmt_ru_ent {
	u8 ru;
	u8 ntx;
	u8 regd;
	u8 ch;
	s8 v;
} __packed;

struct shape_ent {
	u8 band;
	u8 rs;
	u8 regd;
	u8 v;
} __packed;

struct elem_table {
	const u8 *data;
	u32 n;
	u8 sz;
};

/* Table-type element: head[3] = entry size, head[4..7] = entry count. */
static bool elem_table(struct ax52_dev *rd, const struct ax52_fw_elem *e,
		       struct elem_table *t)
{
	if (!e)
		return false;
	t->data = e->data;
	t->sz = e->head[3];
	t->n = get_unaligned_le32(e->head + 4);
	if (!t->sz || (u64)t->n * t->sz > e->size) {
		ax52_warn(rd, "malformed firmware element %u\n", e->id);
		return false;
	}
	return true;
}

/* Copy entry @i into @ent; unknown trailing fields must be zero. */
static bool elem_ent(const struct elem_table *t, u32 i, void *ent, size_t len)
{
	const u8 *p = t->data + i * t->sz;

	memset(ent, 0, len);
	memcpy(ent, p, min_t(size_t, len, t->sz));
	return t->sz <= len || !memchr_inv(p + len, 0, t->sz - len);
}

/* ------------------------------------------------------------ loading */

static bool txpwr_load_byrate(struct ax52_dev *rd, struct ax52_txpwr *t)
{
	struct elem_table s;
	struct byr_ent e;
	struct ax52_byrate *b;
	u32 i, data;
	u8 j, max;
	s8 *dst;

	if (!elem_table(rd, ax52_fw_txpwr_elem(rd, FW_ELEM_TXPWR_BYRATE), &s))
		return false;

	for (i = 0; i < s.n; i++) {
		/* only the 20 MHz, non-OFDMA rows are used by this chip */
		if (!elem_ent(&s, i, &e, sizeof(e)) || e.band > AX52_BAND_5G || e.bw)
			continue;
		b = &t->byr[e.band];
		switch (e.rs) {
		case RS_CCK:
			dst = b->cck;
			max = ARRAY_SIZE(b->cck);
			break;
		case RS_OFDM:
			dst = b->ofdm;
			max = ARRAY_SIZE(b->ofdm);
			break;
		case RS_MCS:
			if (e.nss >= ARRAY_SIZE(b->mcs) || e.ofdma)
				continue;
			dst = b->mcs[e.nss];
			max = ARRAY_SIZE(b->mcs[0]);
			break;
		case RS_HEDCM:
			if (e.nss >= ARRAY_SIZE(b->hedcm) || e.ofdma)
				continue;
			dst = b->hedcm[e.nss];
			max = ARRAY_SIZE(b->hedcm[0]);
			break;
		case RS_OFFSET:
			dst = b->offset;
			max = ARRAY_SIZE(b->offset);
			break;
		default:
			continue;
		}
		if (e.shf + e.len > max)
			continue;
		data = le32_to_cpu(e.data);
		for (j = 0; j < e.len; j++, data >>= 8)
			dst[e.shf + j] = data;
	}
	return true;
}

static bool txpwr_load_lmt(struct ax52_dev *rd, struct ax52_txpwr *t, u8 band)
{
	u32 id = band == AX52_BAND_2G ? FW_ELEM_TXPWR_LMT_2GHZ : FW_ELEM_TXPWR_LMT_5GHZ;
	struct elem_table s;
	struct lmt_ent e;
	u32 i;

	if (!elem_table(rd, ax52_fw_txpwr_elem(rd, id), &s))
		return false;

	for (i = 0; i < s.n; i++) {
		if (!elem_ent(&s, i, &e, sizeof(e)) || e.ntx >= 2 || e.rs > RS_MCS ||
		    e.bf >= 2 || e.regd >= TXPWR_REGD_NUM)
			continue;
		if (band == AX52_BAND_2G && e.bw < ARRAY_SIZE(t->lmt_2g) &&
		    e.ch < TXPWR_2G_CH_NUM)
			t->lmt_2g[e.bw][e.ntx][e.rs][e.bf][e.regd][e.ch] = e.v;
		else if (band == AX52_BAND_5G && e.bw < ARRAY_SIZE(t->lmt_5g) &&
			 e.ch < TXPWR_5G_CH_NUM)
			t->lmt_5g[e.bw][e.ntx][e.rs][e.bf][e.regd][e.ch] = e.v;
	}
	return true;
}

static bool txpwr_load_lmt_ru(struct ax52_dev *rd, struct ax52_txpwr *t, u8 band)
{
	u32 id = band == AX52_BAND_2G ? FW_ELEM_TXPWR_LMT_RU_2GHZ :
					 FW_ELEM_TXPWR_LMT_RU_5GHZ;
	struct elem_table s;
	struct lmt_ru_ent e;
	u32 i;

	if (!elem_table(rd, ax52_fw_txpwr_elem(rd, id), &s))
		return false;

	for (i = 0; i < s.n; i++) {
		if (!elem_ent(&s, i, &e, sizeof(e)) || e.ru >= TXPWR_RU_NUM ||
		    e.ntx >= 2 || e.regd >= TXPWR_REGD_NUM)
			continue;
		if (band == AX52_BAND_2G && e.ch < TXPWR_2G_CH_NUM)
			t->lmt_ru_2g[e.ru][e.ntx][e.regd][e.ch] = e.v;
		else if (band == AX52_BAND_5G && e.ch < TXPWR_5G_CH_NUM)
			t->lmt_ru_5g[e.ru][e.ntx][e.regd][e.ch] = e.v;
	}
	return true;
}

/* Without the table every regulation gets the plain filters, like worldwide. */
static void txpwr_load_shape(struct ax52_dev *rd, struct ax52_txpwr *t)
{
	struct elem_table s;
	struct shape_ent e;
	u32 i;

	if (!elem_table(rd, ax52_fw_txpwr_elem(rd, FW_ELEM_TX_SHAPE_LMT), &s))
		return;

	for (i = 0; i < s.n; i++)
		if (elem_ent(&s, i, &e, sizeof(e)) && e.band <= AX52_BAND_5G &&
		    e.rs <= RS_OFDM && e.regd < TXPWR_REGD_NUM)
			t->shape[e.band][e.rs][e.regd] = e.v;
}

void ax52_txpwr_load(struct ax52_dev *rd)
{
	struct ax52_txpwr *t = &((struct ax52_phy *)rd->phy_priv)->txpwr;
	u8 band;

	if (t->loaded)
		return;

	t->have_byr = txpwr_load_byrate(rd, t);
	for (band = AX52_BAND_2G; band <= AX52_BAND_5G; band++) {
		t->have_lmt[band] = txpwr_load_lmt(rd, t, band);
		t->have_lmt_ru[band] = txpwr_load_lmt_ru(rd, t, band);
	}
	txpwr_load_shape(rd, t);
	t->loaded = true;

	if (!t->have_byr || !t->have_lmt[0] || !t->have_lmt[1] ||
	    !t->have_lmt_ru[0] || !t->have_lmt_ru[1])
		ax52_warn(rd, "TX power tables incomplete (by-rate %d, limits %d/%d, RU limits %d/%d): %d dBm used instead\n",
			  t->have_byr, t->have_lmt[0], t->have_lmt[1],
			   t->have_lmt_ru[0], t->have_lmt_ru[1], TXPWR_FALLBACK / 4);
	if (!ax52_fw_elem(rd, FW_ELEM_REGD))
		ax52_warn(rd, "no country table in the firmware file: worldwide TX power limits\n");
}

/* ------------------------------------------------------------ lookups */

/*
 * Regulation of the current country for @band; entries are {alpha2[2],
 * rule 2 GHz, rule 5 GHz, rule 6 GHz, feature map}. "00" or an unknown
 * country is worldwide.
 */
static u8 txpwr_regd(struct ax52_dev *rd, u8 band)
{
	struct elem_table s;
	u32 i;
	u8 r;

	if (!elem_table(rd, ax52_fw_elem(rd, FW_ELEM_REGD), &s) || s.sz < 2)
		return REGD_WW;

	for (i = 0; i < s.n; i++) {
		const u8 *e = s.data + i * s.sz;

		if (e[0] != rd->regd_alpha2[0] || e[1] != rd->regd_alpha2[1])
			continue;
		r = 2 + band < s.sz ? e[2 + band] : REGD_NA;
		if (r >= TXPWR_REGD_NUM)
			r = REGD_NA;
		return r == REGD_UK ? REGD_ETSI : r;	/* UK follows ETSI */
	}
	return REGD_WW;
}

static int txpwr_ch_idx(u8 band, u8 ch)
{
	if (band == AX52_BAND_2G)
		return ch >= 1 && ch <= 14 ? ch - 1 : -1;
	if (ch >= 36 && ch <= 64)
		return (ch - 36) / 2;
	if (ch >= 100 && ch <= 144)
		return (ch - 100) / 2 + 15;
	if (ch >= 149 && ch <= 177)
		return (ch - 149) / 2 + 38;
	return -1;
}

struct lmt_ctx {
	const struct ax52_txpwr *t;
	u8 band;
	u8 ntx;
	u8 regd;
};

/* A value of 0 means "not listed for this regulation": use worldwide. */
static s8 lmt_to_mac(s8 v, s8 ww)
{
	return min_t(int, (v ?: ww) >> 1, TXPWR_MAC_MAX);
}

/* Limit for a rate section on the 20/40/80 MHz channel centred at @ch. */
static s8 txpwr_limit(const struct lmt_ctx *x, u8 bw, u8 rs, u8 bf, u8 ch)
{
	const struct ax52_txpwr *t = x->t;
	int ci = txpwr_ch_idx(x->band, ch);

	if (!t->have_lmt[x->band] || ci < 0)
		return TXPWR_FALLBACK >> 1;
	if (x->band == AX52_BAND_2G)
		return lmt_to_mac(t->lmt_2g[bw][x->ntx][rs][bf][x->regd][ci],
				  t->lmt_2g[bw][x->ntx][rs][bf][REGD_WW][ci]);
	return lmt_to_mac(t->lmt_5g[bw][x->ntx][rs][bf][x->regd][ci],
			  t->lmt_5g[bw][x->ntx][rs][bf][REGD_WW][ci]);
}

static s8 txpwr_limit_ru(const struct lmt_ctx *x, u8 ru, u8 ch)
{
	const struct ax52_txpwr *t = x->t;
	int ci = txpwr_ch_idx(x->band, ch);

	if (!t->have_lmt_ru[x->band] || ci < 0)
		return TXPWR_FALLBACK >> 1;
	if (x->band == AX52_BAND_2G)
		return lmt_to_mac(t->lmt_ru_2g[ru][x->ntx][x->regd][ci],
				  t->lmt_ru_2g[ru][x->ntx][REGD_WW][ci]);
	return lmt_to_mac(t->lmt_ru_5g[ru][x->ntx][x->regd][ci],
			  t->lmt_ru_5g[ru][x->ntx][REGD_WW][ci]);
}

/* 20 MHz sub-channels of a channel, as offsets from its centre */
static const s8 sub20[][4] = {
	[AX52_BW_20] = { 0 },
	[AX52_BW_40] = { -2, 2 },
	[AX52_BW_80] = { -6, -2, 2, 6 },
};

/*
 * One limit page: {non-BF, BF} pairs for CCK 20/40, OFDM, MCS 20 MHz x8,
 * MCS 40 MHz x4, MCS 80 MHz x2, MCS 160, MCS 40 at 0.5 and at 2.5 offsets.
 */
static void txpwr_fill_lmt(const struct lmt_ctx *x, const struct ax52_chan *c,
			   s8 *pg)
{
	enum { CCK20 = 0, CCK40 = 2, OFDM = 4, MCS20 = 6, MCS40 = 22, MCS80 = 30,
	       MCS40_0P5 = 36 };
	u8 ch = c->ch, bf, i;

	memset(pg, 0, PWR_LMT_PAGE);
	for (bf = 0; bf < 2; bf++) {
		for (i = 0; i < (1 << c->bw); i++)
			pg[MCS20 + 2 * i + bf] = txpwr_limit(x, AX52_BW_20, RS_MCS, bf,
							     ch + sub20[c->bw][i]);
		switch (c->bw) {
		case AX52_BW_20:
			pg[CCK20 + bf] = txpwr_limit(x, AX52_BW_20, RS_CCK, bf, ch);
			pg[CCK40 + bf] = txpwr_limit(x, AX52_BW_40, RS_CCK, bf, ch);
			pg[OFDM + bf] = txpwr_limit(x, AX52_BW_20, RS_OFDM, bf, ch);
			break;
		case AX52_BW_40:
			pg[CCK20 + bf] = txpwr_limit(x, AX52_BW_20, RS_CCK, bf, ch - 2);
			pg[CCK40 + bf] = txpwr_limit(x, AX52_BW_40, RS_CCK, bf, ch);
			pg[OFDM + bf] = txpwr_limit(x, AX52_BW_20, RS_OFDM, bf, c->pri_ch);
			pg[MCS40 + bf] = txpwr_limit(x, AX52_BW_40, RS_MCS, bf, ch);
			break;
		case AX52_BW_80:
			pg[OFDM + bf] = txpwr_limit(x, AX52_BW_20, RS_OFDM, bf, c->pri_ch);
			pg[MCS40 + bf] = txpwr_limit(x, AX52_BW_40, RS_MCS, bf, ch - 4);
			pg[MCS40 + 2 + bf] = txpwr_limit(x, AX52_BW_40, RS_MCS, bf, ch + 4);
			pg[MCS80 + bf] = txpwr_limit(x, AX52_BW_80, RS_MCS, bf, ch);
			pg[MCS40_0P5 + bf] = min(pg[MCS40 + bf], pg[MCS40 + 2 + bf]);
			break;
		}
	}
}

/* One RU limit page: RU26 x8, RU52 x8, RU106 x8 (one per 20 MHz sub-channel). */
static void txpwr_fill_lmt_ru(const struct lmt_ctx *x, const struct ax52_chan *c,
			      s8 *pg)
{
	u8 ru, i;

	memset(pg, 0, PWR_RU_LMT_PAGE);
	for (ru = 0; ru < TXPWR_RU_NUM; ru++)
		for (i = 0; i < (1 << c->bw); i++)
			pg[8 * ru + i] = txpwr_limit_ru(x, ru, c->ch + sub20[c->bw][i]);
}

/* ------------------------------------------------------------ programming */

static void txpwr_write(struct ax52_dev *rd, u32 reg, const s8 *v, int len)
{
	int i;

	for (i = 0; i < len; i += 4)
		wr32(rd, reg + i, (u8)v[i] | (u8)v[i + 1] << 8 |
				  (u8)v[i + 2] << 16 | (u32)(u8)v[i + 3] << 24);
}

/*
 * Target power per rate: CCK 1-11M (from the 2.4 GHz rows on either band),
 * OFDM 6-54M, then per NSS: MCS 0-11 and HE-DCM MCS 0/1/3/4.
 */
static void txpwr_set_byrate(struct ax52_dev *rd, const struct ax52_txpwr *t,
			     u8 band)
{
	const struct ax52_byrate *b = &t->byr[band];
	s8 v[PWR_BYR_LEN];
	int i, nss;

	memcpy(v, t->byr[AX52_BAND_2G].cck, 4);
	memcpy(v + 4, b->ofdm, 8);
	for (nss = 0; nss < 2; nss++) {
		memcpy(v + 12 + 16 * nss, b->mcs[nss], 12);
		memcpy(v + 24 + 16 * nss, b->hedcm[nss], 4);
	}
	for (i = 0; i < PWR_BYR_LEN; i++)
		v[i] = (t->have_byr ? v[i] : TXPWR_FALLBACK) >> 1;
	txpwr_write(rd, REG_PWR_BY_RATE, v, PWR_BYR_LEN);
}

/* Rate-section offsets, 4-bit fields: HE, VHT, HT, OFDM, CCK. */
static void txpwr_set_offset(struct ax52_dev *rd, const struct ax52_txpwr *t,
			     u8 band)
{
	u32 val = 0;
	int i;

	for (i = 0; i < 5; i++)
		val |= ((t->byr[band].offset[i] >> 1) & 0xF) << (4 * i);
	mask32(rd, REG_PWR_RATE_OFST, GENMASK(19, 0), val);
}

static const u32 cck_dfir[3][8] = {
	{ 0x023D23FF, 0x0029B354, 0x000FC1C8, 0x00FDB053,	/* flat */
	  0x00F86F9A, 0x06FAEF92, 0x00FE5FCC, 0x00FFDFF5 },
	{ 0x023D83FF, 0x002C636A, 0x0013F204, 0x00008090,	/* sharp */
	  0x00F87FB0, 0x06F99F83, 0x00FDBFBA, 0x00003FF5 },
	{ 0x023B13FF, 0x001C42DE, 0x00FDB0AD, 0x00F60F6E,	/* channel 14 */
	  0x00FD8F92, 0x0602D011, 0x0001C02C, 0x00FFF00A },
};

/* Spectral shaping required by the regulation: CCK filter, OFDM window. */
static void txpwr_set_shape(struct ax52_dev *rd, const struct ax52_txpwr *t,
			    const struct ax52_chan *c, u8 regd)
{
	int i, k;

	if (c->band == AX52_BAND_2G) {
		k = c->ch == 14 ? 2 : t->shape[c->band][RS_CCK][regd] ? 1 : 0;
		for (i = 0; i < ARRAY_SIZE(cck_dfir[0]); i++)
			ax52_bb_write(rd, BB_TXFIR + 4 * i, cck_dfir[k][i]);
	}
	ax52_bb_write_mask(rd, BB_DCFO_OPT, TXSHAPE_TRI, t->shape[c->band][RS_OFDM][regd]);
}

static void txpwr_set_limits(struct ax52_dev *rd, const struct ax52_txpwr *t,
			     const struct ax52_chan *c, u8 regd)
{
	struct lmt_ctx x = { .t = t, .band = c->band, .regd = regd };
	s8 pg[PWR_LMT_PAGE];

	for (x.ntx = 0; x.ntx < 2; x.ntx++) {
		txpwr_fill_lmt(&x, c, pg);
		txpwr_write(rd, REG_PWR_LMT + x.ntx * PWR_LMT_PAGE, pg, PWR_LMT_PAGE);
	}
	for (x.ntx = 0; x.ntx < 2; x.ntx++) {
		txpwr_fill_lmt_ru(&x, c, pg);
		txpwr_write(rd, REG_PWR_RU_LMT + x.ntx * PWR_RU_LMT_PAGE, pg,
			    PWR_RU_LMT_PAGE);
	}
}

void ax52_txpwr_init_unit(struct ax52_dev *rd)
{
	wr32(rd, REG_PWR_UL_CTRL2, 0x07763333);
	wr32(rd, REG_PWR_COEXT_CTRL, 0x01EBF000);
	wr32(rd, REG_PWR_UL_CTRL0, 0x0002F8FF);
	/* HE trigger-based uplink: offset 0 for 1TX, -3 for 2TX */
	set32(rd, REG_PWR_UL_TB_CTRL, PWR_UL_TB_EN);
	mask32(rd, REG_PWR_UL_TB_1T, PWR_UL_TB_OFST, 0);
	mask32(rd, REG_PWR_UL_TB_2T, PWR_UL_TB_OFST, 0x1D);
}

void ax52_txpwr_set_ref(struct ax52_dev *rd)
{
	u8 cck, path;

	mask32(rd, REG_PWR_RATE_CTRL, PWR_REF_MASK, 0);
	for (cck = 0; cck < 2; cck++)
		for (path = RF_PATH_A; path < RF_PATH_NUM; path++)
			ax52_bb_write_mask(rd, BB_P0_TXPWR_REF + 4 * cck + (path << 13),
					   TXPWR_REF_MASK, TXPWR_REF_CW);
}

void ax52_txpwr_apply(struct ax52_dev *rd)
{
	const struct ax52_txpwr *t = &((struct ax52_phy *)rd->phy_priv)->txpwr;
	const struct ax52_chan *c = &rd->chan;
	u8 regd = txpwr_regd(rd, c->band);

	txpwr_set_byrate(rd, t, c->band);
	txpwr_set_offset(rd, t, c->band);
	txpwr_set_shape(rd, t, c, regd);
	txpwr_set_limits(rd, t, c, regd);
	ax52_txpwr_set_ref(rd);
}
