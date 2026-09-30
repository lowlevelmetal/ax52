/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Private state shared by phy.c (BB/RF access, tables, channel) and
 * phy_txpwr.c (TX power).
 */
#ifndef AX52_PHY_H
#define AX52_PHY_H

#include "ax52.h"

/* registers used by both files */
#define BB_DCFO_OPT		0x4494	/* CFO options, OFDM TX shaping */
#define REG_PWR_UL_CTRL2	0xD248	/* MAC TX power unit / CFO compensation */

/* ------------------------------------------------------------ TX power */

#define TXPWR_REGD_NUM		16	/* regulation index space of the tables */
#define TXPWR_2G_CH_NUM		14
#define TXPWR_5G_CH_NUM		53
#define TXPWR_RU_NUM		3	/* RU26, RU52, RU106 */

/* By-rate target powers of one band, 1/4 dB (20 MHz, non-OFDMA rows). */
struct ax52_byrate {
	s8 cck[4];
	s8 ofdm[8];
	s8 mcs[2][16];		/* [nss] HT/VHT/HE MCS */
	s8 hedcm[2][4];		/* [nss] HE DCM MCS 0, 1, 3, 4 */
	s8 offset[8];		/* rate-section offsets: HE, VHT, HT, OFDM, CCK */
};

/* TX-power tables from the firmware file, 1/4 dB. */
struct ax52_txpwr {
	bool loaded;
	bool have_byr;
	bool have_lmt[2], have_lmt_ru[2];	/* [band] */

	struct ax52_byrate byr[2];		/* [band] */
	/* [bw][ntx][rate section][beamformed][regulation][channel index] */
	s8 lmt_2g[2][2][3][2][TXPWR_REGD_NUM][TXPWR_2G_CH_NUM];
	s8 lmt_5g[3][2][3][2][TXPWR_REGD_NUM][TXPWR_5G_CH_NUM];
	/* [RU size][ntx][regulation][channel index] */
	s8 lmt_ru_2g[TXPWR_RU_NUM][2][TXPWR_REGD_NUM][TXPWR_2G_CH_NUM];
	s8 lmt_ru_5g[TXPWR_RU_NUM][2][TXPWR_REGD_NUM][TXPWR_5G_CH_NUM];
	u8 shape[2][2][TXPWR_REGD_NUM];		/* [band][CCK, OFDM][regulation] */
};

/* ------------------------------------------------------------ phy.c */

#define BB_GAIN_BAND_NUM	4	/* 2G, 5G low, 5G mid, 5G high */

/* BB gain table (RAM only), [gain band][path]; applied per channel. */
struct ax52_bb_gain {
	s8 lna[BB_GAIN_BAND_NUM][RF_PATH_NUM][7];
	s8 tia[BB_GAIN_BAND_NUM][RF_PATH_NUM][2];
	s8 rpl_20[BB_GAIN_BAND_NUM][RF_PATH_NUM];
	s8 rpl_40[BB_GAIN_BAND_NUM][RF_PATH_NUM][3];	/* full, sub 1, 2 */
	s8 rpl_80[BB_GAIN_BAND_NUM][RF_PATH_NUM][11];	/* full, 20M 1-4, 40M 9-10 */
};

struct ax52_phy {
	struct ax52_bb_gain gain;

	/* efuse RX gain calibration */
	bool gain_ofst_valid, gain_comp_valid;
	s8 gain_ofst[RF_PATH_NUM][5];	/* CCK, 2G OFDM, 5G low/mid/high */
	s8 gain_comp[RF_PATH_NUM][AX52_SB_NUM];
	s8 offset_base, rssi_base;	/* RPL / RSSI bias left by the BB table */

	struct ax52_txpwr txpwr;
};

/* ------------------------------------------------------------ phy_txpwr.c */

void ax52_txpwr_load(struct ax52_dev *rd);	/* parse FW tables (once) */
void ax52_txpwr_init_unit(struct ax52_dev *rd);
void ax52_txpwr_set_ref(struct ax52_dev *rd);
void ax52_txpwr_apply(struct ax52_dev *rd);	/* for rd->chan */

#endif
