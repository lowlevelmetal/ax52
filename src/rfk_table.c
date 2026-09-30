// SPDX-License-Identifier: GPL-2.0
/*
 * RF calibration register sequences. These are hardware facts transcribed
 * from the vendor calibration flow (rtw89 rtw8852b_rfk_table.c and the PMAC
 * PLCP table of rtw8852b_common.c); per-path TSSI sequences are stored once
 * with BBP entries relocated for path B.
 */
#include "rfk.h"

#define BBW(a, m, v)	{ .op = RFK_OP_BB, .addr = (a), .mask = (m), .val = (v) }
#define BBP(a, m, v)	{ .op = RFK_OP_BBP, .addr = (a), .mask = (m), .val = (v) }
#define DLY(us)		{ .op = RFK_OP_DELAY, .val = (us) }

/* AFE (DAC/ADC) analog configuration for both paths */
static const struct rfk_reg afe_init_regs[] = {
	BBW(0xc0d4, GENMASK(31, 0), 0x4486888c),
	BBW(0xc0d8, GENMASK(31, 0), 0xc6ba10e0),
	BBW(0xc0dc, GENMASK(31, 0), 0x30c52868),
	BBW(0xc0e0, GENMASK(31, 0), 0x05008128),
	BBW(0xc0e4, GENMASK(31, 0), 0x0000272b),
	BBW(0xc1d4, GENMASK(31, 0), 0x4486888c),
	BBW(0xc1d8, GENMASK(31, 0), 0xc6ba10e0),
	BBW(0xc1dc, GENMASK(31, 0), 0x30c52868),
	BBW(0xc1e0, GENMASK(31, 0), 0x05008128),
	BBW(0xc1e4, GENMASK(31, 0), 0x0000272b),
};

const struct rfk_tbl ax52_rfk_afe_init = RFK_TBL(afe_init_regs);

/* DACK path A: MSB calibration kick */
static const struct rfk_reg dack_s0_msbk_regs[] = {
	BBW(0x12a0, BIT(15), 0x1),
	BBW(0x12a0, GENMASK(14, 12), 0x3),
	BBW(0x12b8, BIT(30), 0x1),
	BBW(0x030c, BIT(28), 0x1),
	BBW(0x032c, BIT(31), 0x0),
	BBW(0xc0d8, BIT(16), 0x1),
	BBW(0xc0dc, GENMASK(27, 26), 0x3),
	BBW(0xc004, BIT(30), 0x0),
	BBW(0xc024, BIT(30), 0x0),
	BBW(0xc004, GENMASK(29, 20), 0x30),
	BBW(0xc004, GENMASK(31, 30), 0x0),
	BBW(0xc004, BIT(17), 0x1),
	BBW(0xc024, BIT(17), 0x1),
	BBW(0xc00c, BIT(2), 0x0),
	BBW(0xc02c, BIT(2), 0x0),
	BBW(0xc004, BIT(0), 0x1),
	BBW(0xc024, BIT(0), 0x1),
	DLY(1),
};

const struct rfk_tbl ax52_rfk_dack_s0_msbk = RFK_TBL(dack_s0_msbk_regs);

/* DACK path A: DAC DC calibration kick */
static const struct rfk_reg dack_s0_dadck_regs[] = {
	BBW(0xc0dc, GENMASK(27, 26), 0x0),
	BBW(0xc00c, BIT(2), 0x1),
	BBW(0xc02c, BIT(2), 0x1),
};

const struct rfk_tbl ax52_rfk_dack_s0_dadck = RFK_TBL(dack_s0_dadck_regs);

/* DACK path A: leave calibration mode */
static const struct rfk_reg dack_s0_done_regs[] = {
	BBW(0xc004, BIT(0), 0x0),
	BBW(0xc024, BIT(0), 0x0),
	BBW(0xc0d8, BIT(16), 0x0),
	BBW(0x12a0, BIT(15), 0x0),
	BBW(0x12a0, GENMASK(14, 12), 0x7),
};

const struct rfk_tbl ax52_rfk_dack_s0_done = RFK_TBL(dack_s0_done_regs);

/* DACK path B: MSB calibration kick */
static const struct rfk_reg dack_s1_msbk_regs[] = {
	BBW(0x32a0, BIT(15), 0x1),
	BBW(0x32a0, GENMASK(14, 12), 0x3),
	BBW(0x32b8, BIT(30), 0x1),
	BBW(0x030c, BIT(28), 0x1),
	BBW(0x032c, BIT(31), 0x0),
	BBW(0xc1d8, BIT(16), 0x1),
	BBW(0xc1dc, GENMASK(27, 26), 0x3),
	BBW(0xc104, BIT(30), 0x0),
	BBW(0xc124, BIT(30), 0x0),
	BBW(0xc104, GENMASK(29, 20), 0x30),
	BBW(0xc104, GENMASK(31, 30), 0x0),
	BBW(0xc104, BIT(17), 0x1),
	BBW(0xc124, BIT(17), 0x1),
	BBW(0xc10c, BIT(2), 0x0),
	BBW(0xc12c, BIT(2), 0x0),
	BBW(0xc104, BIT(0), 0x1),
	BBW(0xc124, BIT(0), 0x1),
	DLY(1),
};

const struct rfk_tbl ax52_rfk_dack_s1_msbk = RFK_TBL(dack_s1_msbk_regs);

/* DACK path B: DAC DC calibration kick */
static const struct rfk_reg dack_s1_dadck_regs[] = {
	BBW(0xc1dc, GENMASK(27, 26), 0x0),
	BBW(0xc10c, BIT(2), 0x1),
	BBW(0xc12c, BIT(2), 0x1),
	DLY(1),
};

const struct rfk_tbl ax52_rfk_dack_s1_dadck = RFK_TBL(dack_s1_dadck_regs);

/* DACK path B: leave calibration mode */
static const struct rfk_reg dack_s1_done_regs[] = {
	BBW(0xc104, BIT(0), 0x0),
	BBW(0xc124, BIT(0), 0x0),
	BBW(0xc1d8, BIT(16), 0x0),
	BBW(0x32a0, BIT(15), 0x0),
	BBW(0x32a0, GENMASK(14, 12), 0x7),
};

const struct rfk_tbl ax52_rfk_dack_s1_done = RFK_TBL(dack_s1_done_regs);

/* IQK: BB/AFE into calibration mode (both paths) */
static const struct rfk_reg iqk_bb_set_regs[] = {
	BBW(0x20fc, GENMASK(31, 16), 0x303),
	BBW(0x5864, GENMASK(28, 27), 0x3),
	BBW(0x7864, GENMASK(28, 27), 0x3),
	BBW(0x12b8, BIT(30), 0x1),
	BBW(0x32b8, BIT(30), 0x1),
	BBW(0x030c, GENMASK(31, 24), 0x13),
	BBW(0x032c, GENMASK(31, 16), 0x41),
	BBW(0x12b8, BIT(28), 0x1),
	BBW(0x58c8, BIT(24), 0x1),
	BBW(0x78c8, BIT(24), 0x1),
	BBW(0x5864, GENMASK(31, 30), 0x3),
	BBW(0x7864, GENMASK(31, 30), 0x3),
	BBW(0x2008, GENMASK(24, 0), 0x1ffffff),
	BBW(0x0c1c, BIT(2), 0x1),
	BBW(0x0700, BIT(27), 0x1),
	BBW(0x0c70, GENMASK(9, 0), 0x3ff),
	BBW(0x0c60, GENMASK(1, 0), 0x3),
	BBW(0x0c6c, BIT(0), 0x1),
	BBW(0x58ac, BIT(27), 0x1),
	BBW(0x78ac, BIT(27), 0x1),
	BBW(0x0c3c, BIT(9), 0x1),
	BBW(0x2344, BIT(31), 0x1),
	BBW(0x4490, BIT(31), 0x1),
	BBW(0x12a0, GENMASK(14, 12), 0x7),
	BBW(0x12a0, BIT(15), 0x1),
	BBW(0x12a0, GENMASK(18, 16), 0x3),
	BBW(0x12a0, BIT(19), 0x1),
	BBW(0x32a0, GENMASK(18, 16), 0x3),
	BBW(0x32a0, BIT(19), 0x1),
	BBW(0x0700, BIT(24), 0x1),
	BBW(0x0700, GENMASK(26, 25), 0x2),
	BBW(0x20fc, GENMASK(31, 16), 0x3333),
};

const struct rfk_tbl ax52_rfk_iqk_bb_set = RFK_TBL(iqk_bb_set_regs);

/* IQK: BB/AFE back to normal operation */
static const struct rfk_reg iqk_bb_restore_regs[] = {
	BBW(0x20fc, GENMASK(31, 16), 0x303),
	BBW(0x12b8, BIT(30), 0x0),
	BBW(0x32b8, BIT(30), 0x0),
	BBW(0x5864, GENMASK(31, 30), 0x0),
	BBW(0x7864, GENMASK(31, 30), 0x0),
	BBW(0x2008, GENMASK(24, 0), 0x0),
	BBW(0x0c1c, BIT(2), 0x0),
	BBW(0x0700, BIT(27), 0x0),
	BBW(0x0c70, GENMASK(4, 0), 0x3),
	BBW(0x0c70, GENMASK(9, 5), 0x3),
	BBW(0x12a0, GENMASK(19, 12), 0x0),
	BBW(0x32a0, GENMASK(19, 12), 0x0),
	BBW(0x0700, GENMASK(26, 24), 0x0),
	BBW(0x20fc, GENMASK(31, 16), 0x0),
	BBW(0x58c8, BIT(24), 0x0),
	BBW(0x78c8, BIT(24), 0x0),
	BBW(0x0c3c, BIT(9), 0x0),
	BBW(0x2344, BIT(31), 0x0),
};

const struct rfk_tbl ax52_rfk_iqk_bb_restore = RFK_TBL(iqk_bb_restore_regs);

/* DPK: BB/AFE into calibration mode (both paths) */
static const struct rfk_reg dpk_bb_set_regs[] = {
	BBW(0x20fc, GENMASK(31, 16), 0x303),
	BBW(0x12b8, BIT(30), 0x1),
	BBW(0x32b8, BIT(30), 0x1),
	BBW(0x030c, GENMASK(31, 24), 0x13),
	BBW(0x032c, GENMASK(31, 16), 0x41),
	BBW(0x12b8, BIT(28), 0x1),
	BBW(0x58c8, BIT(24), 0x1),
	BBW(0x78c8, BIT(24), 0x1),
	BBW(0x5864, GENMASK(31, 30), 0x3),
	BBW(0x7864, GENMASK(31, 30), 0x3),
	BBW(0x2008, GENMASK(24, 0), 0x1ffffff),
	BBW(0x0c1c, BIT(2), 0x1),
	BBW(0x0700, BIT(27), 0x1),
	BBW(0x0c70, GENMASK(9, 0), 0x3ff),
	BBW(0x0c60, GENMASK(1, 0), 0x3),
	BBW(0x0c6c, BIT(0), 0x1),
	BBW(0x58ac, BIT(27), 0x1),
	BBW(0x78ac, BIT(27), 0x1),
	BBW(0x0c3c, BIT(9), 0x1),
	BBW(0x2344, BIT(31), 0x1),
	BBW(0x4490, BIT(31), 0x1),
	BBW(0x12a0, GENMASK(19, 12), 0xbf),
	BBW(0x32a0, GENMASK(19, 16), 0xb),
	BBW(0x0700, GENMASK(26, 24), 0x5),
	BBW(0x20fc, GENMASK(31, 16), 0x3333),
	BBW(0x580c, BIT(15), 0x1),
	BBW(0x5800, GENMASK(15, 0), 0x0),
	BBW(0x780c, BIT(15), 0x1),
	BBW(0x7800, GENMASK(15, 0), 0x0),
};

const struct rfk_tbl ax52_rfk_dpk_bb_set = RFK_TBL(dpk_bb_set_regs);

/* DPK: BB/AFE back to normal operation */
static const struct rfk_reg dpk_bb_restore_regs[] = {
	BBW(0x20fc, GENMASK(31, 16), 0x303),
	BBW(0x12b8, BIT(30), 0x0),
	BBW(0x32b8, BIT(30), 0x0),
	BBW(0x5864, GENMASK(31, 30), 0x0),
	BBW(0x7864, GENMASK(31, 30), 0x0),
	BBW(0x2008, GENMASK(24, 0), 0x0),
	BBW(0x0c1c, BIT(2), 0x0),
	BBW(0x0700, BIT(27), 0x0),
	BBW(0x0c70, GENMASK(9, 0), 0x63),
	BBW(0x12a0, GENMASK(19, 12), 0x0),
	BBW(0x32a0, GENMASK(19, 12), 0x0),
	BBW(0x0700, GENMASK(26, 24), 0x0),
	BBW(0x5864, BIT(29), 0x0),
	BBW(0x7864, BIT(29), 0x0),
	BBW(0x20fc, GENMASK(31, 16), 0x0),
	BBW(0x58c8, BIT(24), 0x0),
	BBW(0x78c8, BIT(24), 0x0),
	BBW(0x0c3c, BIT(9), 0x0),
	BBW(0x580c, BIT(15), 0x0),
	BBW(0x58e4, GENMASK(28, 27), 0x1),
	BBW(0x58e4, GENMASK(28, 27), 0x2),
	BBW(0x780c, BIT(15), 0x0),
	BBW(0x78e4, GENMASK(28, 27), 0x1),
	BBW(0x78e4, GENMASK(28, 27), 0x2),
};

const struct rfk_tbl ax52_rfk_dpk_bb_restore = RFK_TBL(dpk_bb_restore_regs);

/* TSSI: shared system setup */
static const struct rfk_reg tssi_sys_regs[] = {
	BBW(0x12a8, GENMASK(3, 0), 0x5),
	BBW(0x32a8, GENMASK(3, 0), 0x5),
	BBW(0x12bc, GENMASK(19, 4), 0x5555),
	BBW(0x32bc, GENMASK(19, 4), 0x5555),
	BBW(0x0300, GENMASK(31, 24), 0x16),
	BBW(0x0304, GENMASK(7, 0), 0x19),
	BBW(0x0314, GENMASK(31, 16), 0x2041),
	BBW(0x0318, GENMASK(31, 0), 0x00002041),
	BBW(0x0318, GENMASK(31, 0), 0x20012041),
	BBW(0x0020, GENMASK(14, 13), 0x3),
	BBW(0x0024, GENMASK(14, 13), 0x3),
	BBW(0x0704, GENMASK(31, 16), 0x601e),
	BBW(0x2704, GENMASK(31, 16), 0x601e),
	BBW(0x0700, GENMASK(31, 28), 0x4),
	BBW(0x2700, GENMASK(31, 28), 0x4),
	BBW(0x0650, GENMASK(29, 26), 0x0),
	BBW(0x2650, GENMASK(29, 26), 0x0),
};

const struct rfk_tbl ax52_rfk_tssi_sys = RFK_TBL(tssi_sys_regs);

/* TSSI: per-path system setup, 2 GHz */
static const struct rfk_reg tssi_sys_2g_regs[] = {
	BBP(0x120c, GENMASK(7, 0), 0x33),
	BBP(0x12c0, GENMASK(27, 20), 0x33),
	BBP(0x58f8, BIT(30), 0x1),
	BBW(0x0304, GENMASK(15, 8), 0x1e),
};

const struct rfk_tbl ax52_rfk_tssi_sys_2g = RFK_TBL(tssi_sys_2g_regs);

/* TSSI: per-path system setup, 5 GHz */
static const struct rfk_reg tssi_sys_5g_regs[] = {
	BBP(0x120c, GENMASK(7, 0), 0x44),
	BBP(0x12c0, GENMASK(27, 20), 0x44),
	BBP(0x58f8, BIT(30), 0x0),
	BBW(0x0304, GENMASK(15, 8), 0x1d),
};

const struct rfk_tbl ax52_rfk_tssi_sys_5g = RFK_TBL(tssi_sys_5g_regs);

/* TSSI: TX power control engine init */
static const struct rfk_reg tssi_txpwr_regs[] = {
	BBW(0x566c, BIT(12), 0x0),
	BBP(0x5800, GENMASK(31, 0), 0x003f807f),
	BBP(0x580c, GENMASK(6, 0), 0x40),
	BBP(0x580c, GENMASK(27, 8), 0x40),
	BBP(0x5810, GENMASK(31, 0), 0x59010000),
	BBP(0x5814, GENMASK(24, 0), 0x2d000),
	BBP(0x5814, GENMASK(31, 27), 0x0),
	BBP(0x5818, GENMASK(31, 0), 0x002c1800),
	BBP(0x581c, GENMASK(29, 0), 0x1dc80280),
	BBP(0x5820, GENMASK(31, 0), 0x00002080),
	BBP(0x580c, BIT(28), 0x1),
	BBP(0x580c, BIT(30), 0x1),
	BBP(0x5834, GENMASK(29, 0), 0x115f2),
	BBP(0x5838, GENMASK(30, 0), 0x121),
	BBP(0x5854, GENMASK(29, 0), 0x115f2),
	BBP(0x5858, GENMASK(30, 0), 0x121),
	BBP(0x5860, BIT(31), 0x0),
	BBP(0x5864, GENMASK(26, 0), 0x801ff),
	BBP(0x5898, GENMASK(31, 0), 0x00000000),
	BBP(0x589c, GENMASK(31, 0), 0x00000000),
	BBP(0x58a4, GENMASK(7, 0), 0x16),
	BBP(0x58b0, GENMASK(31, 0), 0x00000000),
	BBP(0x58b4, GENMASK(30, 0), 0xa002000),
	BBP(0x58b8, GENMASK(30, 0), 0x7628),
	BBP(0x58bc, GENMASK(26, 0), 0x7a7807f),
	BBP(0x58c0, GENMASK(31, 17), 0x3f),
	BBP(0x58c4, GENMASK(31, 0), 0x0003ffff),
	BBP(0x58c8, GENMASK(23, 0), 0x0),
	BBP(0x58c8, GENMASK(31, 28), 0x0),
	BBP(0x58cc, GENMASK(31, 0), 0x00000000),
	BBP(0x58d0, GENMASK(26, 0), 0x2008101),
	BBP(0x58d4, GENMASK(7, 0), 0x0),
	BBP(0x58d4, GENMASK(17, 9), 0xff),
	BBP(0x58d4, GENMASK(26, 18), 0x100),
	BBP(0x58d8, GENMASK(31, 0), 0x8008016c),
	BBP(0x58dc, GENMASK(16, 0), 0x807f),
	BBP(0x58dc, GENMASK(31, 20), 0x800),
	BBP(0x58f0, GENMASK(17, 0), 0x1ff),
	BBP(0x58f4, GENMASK(19, 0), 0x0),
};

const struct rfk_tbl ax52_rfk_tssi_txpwr = RFK_TBL(tssi_txpwr_regs);

/* TSSI: HE trigger-based PPDU settings */
static const struct rfk_reg tssi_he_tb_regs[] = {
	BBP(0x58a0, GENMASK(31, 0), 0x000000fe),
	BBP(0x58e4, GENMASK(6, 0), 0x1f),
};

const struct rfk_tbl ax52_rfk_tssi_he_tb = RFK_TBL(tssi_he_tb_regs);

/* TSSI: DC settings */
static const struct rfk_reg tssi_dck_regs[] = {
	BBP(0x580c, GENMASK(27, 16), 0x0),
	BBP(0x5814, GENMASK(21, 12), 0xef),
	BBP(0x5814, GENMASK(28, 27), 0x0),
};

const struct rfk_tbl ax52_rfk_tssi_dck = RFK_TBL(tssi_dck_regs);

/* TSSI: slope calibration defaults, 2 GHz */
static const struct rfk_reg tssi_slope_2g_regs[] = {
	BBP(0x5608, GENMASK(26, 0), 0x801008),
	BBP(0x560c, GENMASK(26, 0), 0x201020),
	BBP(0x5610, GENMASK(26, 0), 0x201008),
	BBP(0x5614, GENMASK(26, 0), 0x804008),
	BBP(0x5618, GENMASK(26, 0), 0x201008),
	BBP(0x561c, GENMASK(8, 0), 0x8),
	BBP(0x561c, GENMASK(31, 16), 0x808),
	BBP(0x5620, GENMASK(31, 0), 0x08081e28),
	BBP(0x5624, GENMASK(31, 0), 0x08080808),
	BBP(0x5628, GENMASK(31, 0), 0x08081e28),
	BBP(0x562c, GENMASK(15, 0), 0x808),
	BBP(0x581c, BIT(20), 0x1),
};

const struct rfk_tbl ax52_rfk_tssi_slope_2g = RFK_TBL(tssi_slope_2g_regs);

/* TSSI: slope calibration defaults, 5 GHz */
static const struct rfk_reg tssi_slope_5g_regs[] = {
	BBP(0x5608, GENMASK(26, 0), 0x201008),
	BBP(0x560c, GENMASK(26, 0), 0x201020),
	BBP(0x5610, GENMASK(26, 0), 0x201008),
	BBP(0x5614, GENMASK(26, 0), 0x201008),
	BBP(0x5618, GENMASK(26, 0), 0x201008),
	BBP(0x561c, GENMASK(8, 0), 0x8),
	BBP(0x561c, GENMASK(31, 16), 0x808),
	BBP(0x5620, GENMASK(31, 0), 0x08081e08),
	BBP(0x5624, GENMASK(31, 0), 0x08080808),
	BBP(0x5628, GENMASK(31, 0), 0x08080808),
	BBP(0x562c, GENMASK(15, 0), 0x808),
	BBP(0x581c, BIT(20), 0x1),
};

const struct rfk_tbl ax52_rfk_tssi_slope_5g = RFK_TBL(tssi_slope_5g_regs);

/* TSSI: enable slope compensation */
static const struct rfk_reg tssi_slope_en_regs[] = {
	BBP(0x5814, BIT(11), 0x1),
	BBP(0x581c, BIT(29), 0x1),
	BBP(0x5814, BIT(29), 0x1),
};

const struct rfk_tbl ax52_rfk_tssi_slope_en = RFK_TBL(tssi_slope_en_regs);

/* PMAC PLCP setup for HT20 MCS7 test packets (TSSI alignment) */
static const struct rfk_reg pmac_ht20_mcs7_regs[] = {
	BBW(0x4580, GENMASK(15, 0), 0x0),
	BBW(0x4580, GENMASK(31, 16), 0x0),
	BBW(0x4584, GENMASK(15, 0), 0x0),
	BBW(0x4584, GENMASK(31, 16), 0x0),
	BBW(0x4580, GENMASK(15, 0), 0x1),
	BBW(0x4578, GENMASK(23, 0), 0x2018b),
	BBW(0x4570, GENMASK(25, 0), 0x7),
	BBW(0x4574, GENMASK(25, 0), 0x32407),
	BBW(0x45b8, BIT(4), 0x0),
	BBW(0x45b8, BIT(8), 0x0),
	BBW(0x45b8, BIT(7), 0x0),
	BBW(0x45b8, BIT(3), 0x0),
	BBW(0x45a0, GENMASK(15, 8), 0x0),
	BBW(0x45a0, GENMASK(31, 24), 0x1),
	BBW(0x45a4, GENMASK(15, 8), 0x2),
	BBW(0x45a4, GENMASK(31, 24), 0x3),
	BBW(0x45b8, BIT(5), 0x0),
	BBW(0x4568, GENMASK(31, 29), 0x0),
	BBW(0x45b8, BIT(1), 0x1),
	BBW(0x456c, GENMASK(31, 29), 0x0),
	BBW(0x45b4, GENMASK(14, 13), 0x0),
	BBW(0x45b4, GENMASK(12, 11), 0x1),
	BBW(0x45b8, BIT(6), 0x0),
	BBW(0x45b8, BIT(2), 0x0),
	BBW(0x45b8, BIT(9), 0x0),
	BBW(0x4598, GENMASK(31, 27), 0x0),
	BBW(0x45b8, BIT(20), 0x0),
	BBW(0x45a8, GENMASK(11, 6), 0x0),
	BBW(0x45b8, BIT(21), 0x0),
	BBW(0x45b0, GENMASK(5, 3), 0x0),
	BBW(0x45b0, GENMASK(8, 6), 0x0),
	BBW(0x45a0, GENMASK(7, 0), 0x0),
	BBW(0x45b8, BIT(22), 0x0),
	BBW(0x4590, GENMASK(10, 0), 0x0),
	BBW(0x45b0, GENMASK(11, 9), 0x0),
	BBW(0x45ac, GENMASK(4, 0), 0x0),
	BBW(0x45b8, BIT(23), 0x0),
	BBW(0x45a8, GENMASK(17, 12), 0x0),
	BBW(0x45b8, BIT(24), 0x0),
	BBW(0x45b0, GENMASK(14, 12), 0x0),
	BBW(0x45b0, GENMASK(17, 15), 0x0),
	BBW(0x45a0, GENMASK(23, 16), 0x0),
	BBW(0x45b8, BIT(25), 0x0),
	BBW(0x4590, GENMASK(21, 11), 0x0),
	BBW(0x45b0, GENMASK(20, 18), 0x0),
	BBW(0x45ac, GENMASK(9, 5), 0x0),
	BBW(0x45b8, BIT(26), 0x0),
	BBW(0x45a8, GENMASK(23, 18), 0x0),
	BBW(0x45b8, BIT(27), 0x0),
	BBW(0x45b0, GENMASK(23, 21), 0x0),
	BBW(0x45b0, GENMASK(26, 24), 0x0),
	BBW(0x45a4, GENMASK(7, 0), 0x0),
	BBW(0x45b8, BIT(28), 0x0),
	BBW(0x4594, GENMASK(10, 0), 0x0),
	BBW(0x45b0, GENMASK(29, 27), 0x0),
	BBW(0x45ac, GENMASK(14, 10), 0x0),
	BBW(0x45b8, BIT(29), 0x0),
	BBW(0x45a8, GENMASK(29, 24), 0x0),
	BBW(0x45b8, BIT(30), 0x0),
	BBW(0x45b4, GENMASK(2, 0), 0x0),
	BBW(0x45b4, GENMASK(5, 3), 0x0),
	BBW(0x45a4, GENMASK(23, 16), 0x0),
	BBW(0x45b8, BIT(31), 0x0),
	BBW(0x4594, GENMASK(21, 11), 0x0),
	BBW(0x45b4, GENMASK(8, 6), 0x0),
	BBW(0x4598, GENMASK(31, 27), 0x0),
	BBW(0x45b8, BIT(20), 0x0),
	BBW(0x45a8, GENMASK(11, 6), 0x7),
	BBW(0x45b8, BIT(21), 0x0),
	BBW(0x45b0, GENMASK(5, 3), 0x0),
	BBW(0x45b0, GENMASK(8, 6), 0x0),
	BBW(0x45a0, GENMASK(7, 0), 0x0),
	BBW(0x45b4, GENMASK(26, 25), 0x0),
	BBW(0x45b0, GENMASK(2, 0), 0x0),
	BBW(0x45b8, BIT(19), 0x0),
	BBW(0x45a8, GENMASK(5, 0), 0x0),
	BBW(0x457c, GENMASK(31, 21), 0x1),
	BBW(0x4530, GENMASK(31, 0), 0x00000000),
	BBW(0x4588, GENMASK(13, 0), 0x0),
	BBW(0x4598, GENMASK(8, 0), 0x0),
	BBW(0x4534, GENMASK(31, 0), 0x00000000),
	BBW(0x4538, GENMASK(31, 0), 0x00000000),
	BBW(0x453c, GENMASK(31, 0), 0x00000000),
	BBW(0x4588, GENMASK(27, 14), 0x0),
	BBW(0x4598, GENMASK(17, 9), 0x0),
	BBW(0x4540, GENMASK(31, 0), 0x00000000),
	BBW(0x4544, GENMASK(31, 0), 0x00000000),
	BBW(0x4548, GENMASK(31, 0), 0x00000000),
	BBW(0x458c, GENMASK(13, 0), 0x0),
	BBW(0x4598, GENMASK(26, 18), 0x0),
	BBW(0x454c, GENMASK(31, 0), 0x00000000),
	BBW(0x4550, GENMASK(31, 0), 0x00000000),
	BBW(0x4554, GENMASK(31, 0), 0x00000000),
	BBW(0x458c, GENMASK(27, 14), 0x0),
	BBW(0x459c, GENMASK(8, 0), 0x0),
	BBW(0x4558, GENMASK(31, 0), 0x00000000),
	BBW(0x455c, GENMASK(31, 0), 0x00000000),
	BBW(0x4530, GENMASK(31, 0), 0x4e790001),
	BBW(0x4588, GENMASK(13, 0), 0x0),
	BBW(0x4598, GENMASK(8, 0), 0x1),
	BBW(0x4534, GENMASK(31, 0), 0x00000000),
	BBW(0x4538, GENMASK(31, 0), 0x0000004b),
	BBW(0x45ac, GENMASK(29, 27), 0x7),
	BBW(0x4588, GENMASK(31, 28), 0x0),
	BBW(0x459c, GENMASK(30, 25), 0x0),
	BBW(0x45b8, BIT(18), 0x0),
	BBW(0x45b8, BIT(17), 0x0),
	BBW(0x4590, GENMASK(31, 22), 0x0),
	BBW(0x45b8, BIT(14), 0x0),
	BBW(0x4578, GENMASK(31, 24), 0x0),
	BBW(0x45b8, BIT(10), 0x0),
	BBW(0x45b8, BIT(11), 0x0),
	BBW(0x45b8, BIT(12), 0x0),
	BBW(0x45b8, BIT(13), 0x0),
	BBW(0x45b4, GENMASK(16, 15), 0x0),
	BBW(0x45ac, GENMASK(26, 23), 0x0),
	BBW(0x45b4, GENMASK(10, 9), 0x2),
	BBW(0x459c, GENMASK(16, 9), 0x80),
	BBW(0x45ac, GENMASK(18, 15), 0x3),
	BBW(0x459c, GENMASK(24, 17), 0x1),
};

const struct rfk_tbl ax52_rfk_pmac_ht20_mcs7 = RFK_TBL(pmac_ht20_mcs7_regs);

const u32 ax52_rfk_tssi_align_def[RF_PATH_NUM][TSSI_BANDS][4] = {
	{
		{ 0x01ef27af, 0x00000075, 0x017f13ae, 0x0000006e },
		{ 0x016037e7, 0x0000006f, 0x00000000, 0x00000000 },
		{ 0x01f053f1, 0x00000070, 0x00000000, 0x00000000 },
		{ 0x01c047ee, 0x00000070, 0x00000000, 0x00000000 },
	},
	{
		{ 0x01ff2bb5, 0x00000078, 0x018f2bb0, 0x00000072 },
		{ 0x009003da, 0x00000069, 0x00000000, 0x00000000 },
		{ 0x013027e6, 0x00000069, 0x00000000, 0x00000000 },
		{ 0x009003da, 0x00000069, 0x00000000, 0x00000000 },
	},
};

const s8 ax52_rfk_tssi_swing[RF_PATH_NUM][TSSI_BANDS][2][TSSI_SWING_NUM] = {
	{
		{
			{ 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3,
			  3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		},
		{
			{ 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4,
			  4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7 },
			{ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
			  2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4 },
		},
		{
			{ 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4,
			  5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9 },
			{ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
			  2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3 },
		},
		{
			{ 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5,
			  5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9 },
			{ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
			  2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3 },
		},
	},
	{
		{
			{ 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3,
			  3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
			  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2 },
		},
		{
			{ 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
			  5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8 },
			{ 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4,
			  4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8 },
		},
		{
			{ 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4,
			  4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8 },
			{ 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
			  5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8 },
		},
		{
			{ 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
			  5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9 },
			{ 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6,
			  6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12 },
		},
	},
};
