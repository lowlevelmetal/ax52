/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RF calibration: private register map and the register sequences shared
 * between rfk.c and rfk_table.c. BB addresses are BB-relative; RF addresses
 * with bit 16 set use the direct ("D-die") window, the others the serial bus.
 */
#ifndef AX52_RFK_H
#define AX52_RFK_H

#include "ax52.h"

/* Per-path BB blocks: path B lives 0x2000 above path A. */
#define BB_P(path, a)		((a) + ((path) << 13))

/* ------------------------------------------- NCTL/KIP calibration engine */
#define NCTL_CFG		0x8000	/* one-shot command word, idle 0x8 */
#define NCTL_RPT		0x8008	/*   bit26: IQK one-shot failed */
#define NCTL_N1			0x8010
#define NCTL_TONE		0x802c	/*   [11:0] TX tone, [27:16] RX tone */
#define NCTL_ALIVE		0x8080
#define NCTL_DONE		0xbff8	/*   [7:0] == 0x55: one-shot finished */
#define KIP_MDPK_SYNC		0x8070
#define KIP_MDPK_RXDCK		0x8074
#define KIP_MOD			0x8078	/*   copy of RF 0x00 for DPK RX AGC */
#define KIP_SYSCFG		0x8088
#define KIP_LDL_NORM		0x80a0	/*   [1:0] MDPD model order */
#define KIP_DPK_CFG2		0x80bc
#define KIP_DPK_CFG3		0x80c0
#define KIP_KPATH_CFG		0x80d0
#define KIP_RPT_SEL		0x80d4	/*   [21:16] report page */
#define KIP_DPK_TRK		0x80f0	/*   bit31: DPK tracking disabled */
#define KIP_RPT			0x80fc	/* report window */
#define KIP_IQRSN		0x8220
/* per-path KIP registers, path B 0x100 above path A */
#define KIP_P(path, a)		((a) + ((path) << 8))
#define KIP_COEF_SEL		0x8104	/*   bit0 IQC bank, bit8 MDPD bank */
#define KIP_CFIR_SYS		0x8120
#define KIP_IQK_RES		0x8124	/*   [11:8] TX / [3:0] RX CFIR select */
#define KIP_TXIQC		0x8138
#define KIP_RXIQC		0x813c
#define KIP_CFIR_LUT		0x8154
#define KIP_DPD_V1		0x81a0
#define KIP_DPD_CH0		0x81ac	/*   [31:24] DPD table select */
#define KIP_DPD_BND		0x81b4	/*   [8:0], [24:16] power scaling factor */
#define KIP_DPD_CFG		0x81bc	/*   [22:0] gain, [26:25] order, bit24 on */
#define KIP_TXAGC_RFK		0x81c4
#define KIP_DPD_COM		0x81c8	/*   bit15: HW TX AGC offset mode */
#define KIP_IQP			0x81cc
#define KIP_LOAD_COEF		0x81dc

/* ------------------------------------------------------ other BB registers */
#define BB_P0_RFCTM		0x5864	/* b29: RF under calibration control */
#define BB_TSSI_TMETER		0x5810	/* per path */
#define BB_TSSI_TRK		0x5818	/* per path; b30 pauses tracking */
#define BB_TSSI_EN		0x5820	/* per path; b31 enable/re-arm */
#define BB_TSSI_MV_AVG		0x58e4	/* per path */
#define BB_TSSI_CW_RPT		0x1c18	/* per path; b16 ready, [8:0] code word */

/* ----------------------------------------------------------- RF registers */
#define RFREG_MASK		GENMASK(19, 0)
#define RF_MOD			0x00
#define   RF_MOD_MODE		GENMASK(19, 16)
#define   RF_MOD_RXBB		GENMASK(9, 5)
#define     MODE_TX		2
#define     MODE_RX		3
#define RF_MODOPT		0x01
#define RF_RSV1			0x05	/* bit0: 1 = RF driven by BB */
#define RF_LOKVB		0x0a
#define RF_TXIG			0x11
#define RF_CFGCH		0x18
#define RF_BTC			0x1a
#define RF_RCKC			0x1b
#define RF_RCKS			0x1c
#define RF_RXKPLL		0x1e
#define RF_RSV4			0x1f
#define RF_RXK			0x20
#define RF_LUTWA		0x33
#define RF_LUTWD0		0x3f
#define RF_TM			0x42	/* thermal meter */
#define RF_TXG1			0x51
#define RF_TXG2			0x52
#define RF_TXGA			0x55
#define RF_TXMO			0x58
#define RF_BIASA		0x60
#define RF_TXVBUF		0x7c
#define RF_TXPOW		0x7f
#define RF_RXBB			0x83
#define RF_XGLNA2		0x85
#define RF_RXA_LNA		0x8b
#define RF_RXA2			0x8c
#define RF_RXBB2		0x8f
#define RF_XALNA2		0x90
#define RF_DCK			0x92
#define RF_DCK1			0x93
#define RF_IQGEN		0x97
#define RF_TXIQK		0x98
#define RF_TIA			0x9e
#define RF_POW			0xa0
#define RF_SX			0xaf
#define RF_LDO			0xb1
#define RF_LPF			0xb7
#define RF_SYNFB		0xc5	/* b15: synthesizer locked */
#define RF_LCK_TRG		0xd3
#define RF_MMD			0xd5
#define RF_SYNLUT		0xdd
#define RF_RCKD			0xde
#define RF_LUTDBG		0xdf
#define RF_LUTWE		0xef
#define RF_TXAGC		0x10001
#define RF_BBDC			0x10005	/* bit0: D-die equivalent of RF_RSV1 */
#define RF_TXGA_V1		0x10055

/* ------------------------------------------------------ register sequences */
enum rfk_op {
	RFK_OP_BB,		/* BB masked write */
	RFK_OP_BBP,		/* BB masked write, relocated for path B */
	RFK_OP_DELAY,		/* udelay(val) */
};

struct rfk_reg {
	u16 op;
	u16 addr;
	u32 mask;
	u32 val;		/* field value, not shifted */
};

struct rfk_tbl {
	const struct rfk_reg *regs;
	u32 n;
};

#define RFK_TBL(r)		{ .regs = (r), .n = ARRAY_SIZE(r) }

extern const struct rfk_tbl ax52_rfk_afe_init;
extern const struct rfk_tbl ax52_rfk_dack_s0_msbk;
extern const struct rfk_tbl ax52_rfk_dack_s0_dadck;
extern const struct rfk_tbl ax52_rfk_dack_s0_done;
extern const struct rfk_tbl ax52_rfk_dack_s1_msbk;
extern const struct rfk_tbl ax52_rfk_dack_s1_dadck;
extern const struct rfk_tbl ax52_rfk_dack_s1_done;
extern const struct rfk_tbl ax52_rfk_iqk_bb_set;
extern const struct rfk_tbl ax52_rfk_iqk_bb_restore;
extern const struct rfk_tbl ax52_rfk_dpk_bb_set;
extern const struct rfk_tbl ax52_rfk_dpk_bb_restore;
extern const struct rfk_tbl ax52_rfk_tssi_sys;
extern const struct rfk_tbl ax52_rfk_tssi_sys_2g;
extern const struct rfk_tbl ax52_rfk_tssi_sys_5g;
extern const struct rfk_tbl ax52_rfk_tssi_txpwr;
extern const struct rfk_tbl ax52_rfk_tssi_he_tb;
extern const struct rfk_tbl ax52_rfk_tssi_dck;
extern const struct rfk_tbl ax52_rfk_tssi_slope_2g;
extern const struct rfk_tbl ax52_rfk_tssi_slope_5g;
extern const struct rfk_tbl ax52_rfk_tssi_slope_en;
extern const struct rfk_tbl ax52_rfk_pmac_ht20_mcs7;

/* TSSI bands: 2 GHz, 5 GHz ch 36-64, ch 100-144, ch 149-177 */
#define TSSI_BANDS		4
#define TSSI_SWING_NUM		30

/* Default alignment words for BB 0x5630, 0x5634, 0x563c, 0x5640 (+path). */
extern const u32 ax52_rfk_tssi_align_def[RF_PATH_NUM][TSSI_BANDS][4];

/*
 * Thermal power-tracking steps, [path][band][0 = warmer, 1 = colder][delta],
 * programmed into the TSSI hardware's thermal-offset table.
 */
extern const s8 ax52_rfk_tssi_swing[RF_PATH_NUM][TSSI_BANDS][2][TSSI_SWING_NUM];

#endif
