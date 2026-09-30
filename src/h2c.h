/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Firmware command identifiers for callers of ax52_h2c_send() and
 * ax52_h2creg_msg(), plus the RA-report accessor.
 */
#ifndef AX52_H2C_H
#define AX52_H2C_H

#include "ax52.h"

/* H2C / C2H categories */
#define H2C_CAT_MAC		1
#define H2C_CAT_OUTSRC		2	/* PHY, RF, BT-coex */

/*
 * Register mailbox, word 0: [6:0] function, [11:8] message length in
 * dwords including word 0. Replies use the same header.
 */
#define H2CREG_HDR_FUNC		GENMASK(6, 0)
#define H2CREG_HDR_LEN		GENMASK(11, 8)
#define H2CREG_HDR(func, ndw)	(FIELD_PREP(H2CREG_HDR_FUNC, func) | \
				 FIELD_PREP(H2CREG_HDR_LEN, ndw))
#define H2CREG_GET_FEATURE	3	/* 1 dword, [23:16] part 0 */
#define H2CREG_SCH_TX_EN	5
#define C2HREG_PHY_CAP		3
#define C2HREG_TX_PAUSE_RPT	4

#endif
