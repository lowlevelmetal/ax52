// SPDX-License-Identifier: GPL-2.0
/*
 * Host <-> firmware command protocol.
 *
 * Packet H2Cs go out on the FWCMD ring as [WD body][8-byte header][payload];
 * the scheduler TX pause and the PHY capability query use a 16-byte register
 * mailbox instead. C2H events arrive on the RX ring. Payloads are
 * little-endian dwords, laid out for a single station vif (the AP shares
 * its mac_id).
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#include "h2c.h"

static bool fw_log;
module_param(fw_log, bool, 0444);
MODULE_PARM_DESC(fw_log, "Enable firmware debug log events (printed with dev_dbg)");

/* H2C header. The C2H header shares the w0 layout and the w1 length. */
#define HDR_LEN			8
#define HDR0_CAT		GENMASK(1, 0)
#define HDR0_CLASS		GENMASK(7, 2)
#define HDR0_FUNC		GENMASK(15, 8)
#define HDR0_SEQ		GENMASK(31, 24)
#define HDR1_LEN		GENMASK(13, 0)	/* including the header */
#define HDR1_REC_ACK		BIT(14)
#define HDR1_DONE_ACK		BIT(15)

/* MAC category */
#define CL_FW_INFO		0x00
#define   FUNC_LOG_CFG		0x00
#define CL_FR_EXCHG		0x05
#define   FUNC_CCTLINFO_UD	0x02
#define CL_ADDR_CAM		0x06
#define   FUNC_ADDR_CAM_UPD	0x00
#define CL_MEDIA_RPT		0x08
#define   FUNC_JOININFO		0x00
#define   FUNC_ROLE_MAINTAIN	0x04
#define CL_FW_OFLD		0x09
#define   FUNC_MACID_PAUSE	0x08
#define   FUNC_USR_EDCA		0x0F
#define   FUNC_OFLD_CFG		0x14
#define CL_SEC_CAM		0x0A
#define   FUNC_SEC_UPD		0x01
#define CL_BA_CAM		0x0C
#define   FUNC_BA_CAM		0x00
/* OUTSRC category */
#define CL_RA			0x01
#define   FUNC_RA_MACIDCFG	0x00
#define CL_RF_REG_A		0x08	/* path B = 0x09; func = page */
#define CL_RF_FW_NOTIFY		0x0A
#define   FUNC_RF_GET_MCCCH	0x02

/* C2H events we handle */
#define C2H_TYPE(cat, cls, func)	((cat) << 14 | (cls) << 8 | (func))
#define C2H_TYPES		BIT(16)
#define C2H_REC_ACK		C2H_TYPE(H2C_CAT_MAC, 0, 0)
#define C2H_DONE_ACK		C2H_TYPE(H2C_CAT_MAC, 0, 1)
#define C2H_LOG			C2H_TYPE(H2C_CAT_MAC, 0, 2)
#define C2H_RA_RPT		C2H_TYPE(H2C_CAT_OUTSRC, 1, 0)

#define DACK_W2_CAT		GENMASK(1, 0)
#define DACK_W2_CLASS		GENMASK(7, 2)
#define DACK_W2_FUNC		GENMASK(15, 8)
#define DACK_W2_RET		GENMASK(23, 16)
#define DACK_W2_SEQ		GENMASK(31, 24)

#define RA_RPT_W2_MACID		GENMASK(15, 0)
#define RA_RPT_W3_RATE		GENMASK(6, 0)
#define RA_RPT_W3_MODE		GENMASK(9, 8)
#define RA_RPT_W3_GILTF		GENMASK(12, 10)
#define RA_RPT_W3_BW		GENMASK(14, 13)
#define RA_RATE_HT_MCS		GENMASK(4, 0)
#define RA_RATE_MCS		GENMASK(3, 0)
#define RA_RATE_NSS		GENMASK(6, 4)	/* NSS - 1 */
#define RA_RPT_LEGACY		0
#define RA_RPT_HT		1
#define RA_RPT_VHT		2
#define RA_RPT_HE		3

/* HE GI/LTF codes (RA report, RA H2C) */
#define GILTF_2XHE16		2
#define GILTF_2XHE08		3
#define GILTF_1XHE16		4
#define GILTF_1XHE08		5

/* registers */
#define UDM1_CNT		0x01F5
#define   UDM1_H2CREG_CNT	GENMASK(3, 0)
#define   UDM1_C2HREG_CNT	GENMASK(7, 4)
#define CMAC_FUNC_EN		0xC000
#define   CMAC_EN		BIT(30)
#define CTN_TXEN		0xC348	/* 16-bit band-0 scheduler TX enable */

#define SCH_W0_EN		GENMASK(31, 16)
#define SCH_W1_MASK		GENMASK(15, 0)	/* [16]: band */

/* station constants */
#define NET_NO_LINK		0
#define NET_INFRA		2
#define WIFI_ROLE_STATION	1
#define ROLE_CREATE		0
#define ROLE_REMOVE		1
#define TX_NSS			2
#define HW_RATE_CCK1		0x00
#define HW_RATE_OFDM6		0x04

#define BA_CAM_NUM		2	/* static entries; more are made by HW */
#define BA_TID_NONE		0xff

struct ax52_h2c {
	struct mutex lock;	/* mailbox, scheduler pause, BA CAM */
	u8 sch_depth;		/* nested scheduler pauses */
	u16 sch_saved;		/* CTN_TXEN at the outermost pause */
	u8 ba_tid[BA_CAM_NUM];
	unsigned long *c2h_seen;

	/* ra_bw/ra_nss and rd->ra: NAPI writes rd->ra, rc_update is atomic */
	spinlock_t ra_lock;
	u8 ra_bw, ra_nss;	/* last values given to the FW RA */
};

static void h2c_reset(struct ax52_dev *rd)
{
	struct ax52_h2c *p = rd->h2c_priv;

	mutex_lock(&p->lock);
	p->sch_depth = 0;
	memset(p->ba_tid, BA_TID_NONE, sizeof(p->ba_tid));
	mutex_unlock(&p->lock);

	spin_lock_bh(&p->ra_lock);
	p->ra_bw = U8_MAX;
	p->ra_nss = U8_MAX;
	rd->ra.valid = false;
	spin_unlock_bh(&p->ra_lock);
}

int ax52_h2c_alloc(struct ax52_dev *rd)
{
	struct ax52_h2c *p = kzalloc(sizeof(*p), GFP_KERNEL);

	if (!p)
		return -ENOMEM;
	p->c2h_seen = bitmap_zalloc(C2H_TYPES, GFP_KERNEL);
	if (!p->c2h_seen) {
		kfree(p);
		return -ENOMEM;
	}
	mutex_init(&p->lock);
	spin_lock_init(&p->ra_lock);
	rd->h2c_priv = p;
	h2c_reset(rd);
	return 0;
}

void ax52_h2c_free(struct ax52_dev *rd)
{
	struct ax52_h2c *p = rd->h2c_priv;

	if (!p)
		return;
	mutex_destroy(&p->lock);
	bitmap_free(p->c2h_seen);
	kfree(p);
	rd->h2c_priv = NULL;
}

int ax52_h2c_send(struct ax52_dev *rd, u8 cat, u8 cls, u8 func,
		  bool rack, bool dack, const void *payload, u32 len)
{
	__le32 hdr[2];
	int ret;

	if (!rd->fw_ready)
		return -ENODEV;

	spin_lock_bh(&rd->h2c_lock);
	/* AX firmware wants a receive ack requested on every 4th command */
	if (!(rd->h2c_seq % 4))
		rack = true;
	hdr[0] = cpu_to_le32(FIELD_PREP(HDR0_CAT, cat) |
			     FIELD_PREP(HDR0_CLASS, cls) |
			     FIELD_PREP(HDR0_FUNC, func) |
			     FIELD_PREP(HDR0_SEQ, rd->h2c_seq));
	hdr[1] = cpu_to_le32(FIELD_PREP(HDR1_LEN, HDR_LEN + len) |
			     (rack ? HDR1_REC_ACK : 0) |
			     (dack ? HDR1_DONE_ACK : 0));
	ret = ax52_fwcmd_tx(rd, hdr, sizeof(hdr), payload, len, false);
	if (!ret)
		rd->h2c_seq++;
	spin_unlock_bh(&rd->h2c_lock);

	if (ret)
		ax52_err(rd, "H2C %u/%u/%u not sent: %d\n", cat, cls, func, ret);
	return ret;
}

/* ------------------------------------------------------ register mailbox */

/* The host counts mailbox messages in each direction in a nibble of 0x1F5. */
static void udm1_count(struct ax52_dev *rd, u8 mask)
{
	u8 v = rd8(rd, UDM1_CNT);

	wr8(rd, UDM1_CNT, (v & ~mask) | ((v + (1 << __ffs(mask))) & mask));
}

static void c2hreg_read(struct ax52_dev *rd, u32 c2h[4])
{
	int i;

	for (i = 0; i < 4; i++)
		c2h[i] = rd32(rd, REG_C2HREG_DATA0 + 4 * i);
	wr8(rd, REG_C2HREG_CTRL, 0);
	udm1_count(rd, UDM1_C2HREG_CNT);
}

static int h2creg_msg(struct ax52_dev *rd, const u32 h2c[4], u32 c2h[4])
{
	u32 stale[4];
	int i, ret;
	u8 v;

	if (!rd->fw_ready)
		return -ENODEV;

	ret = read_poll_timeout(rd8, v, !v, 1000, 5000, false, rd,
				REG_H2CREG_CTRL);
	if (ret) {
		ax52_warn(rd, "firmware does not take register H2Cs\n");
		return ret;
	}
	/* a late answer to an earlier message that timed out */
	if (c2h && rd8(rd, REG_C2HREG_CTRL)) {
		c2hreg_read(rd, stale);
		ax52_dbg(rd, "dropped stale register C2H 0x%08x\n", stale[0]);
	}

	for (i = 0; i < 4; i++)
		wr32(rd, REG_H2CREG_DATA0 + 4 * i, h2c[i]);
	udm1_count(rd, UDM1_H2CREG_CNT);
	wr8(rd, REG_H2CREG_CTRL, 1);
	if (!c2h)
		return 0;

	ret = read_poll_timeout(rd8, v, v, 20, 1000000, false, rd,
				REG_C2HREG_CTRL);
	if (ret) {
		ax52_warn(rd, "no register C2H for H2C 0x%08x\n", h2c[0]);
		return ret;
	}
	c2hreg_read(rd, c2h);
	return 0;
}

int ax52_h2creg_msg(struct ax52_dev *rd, const u32 h2c[4], u32 c2h[4])
{
	struct ax52_h2c *p = rd->h2c_priv;
	int ret;

	mutex_lock(&p->lock);
	ret = h2creg_msg(rd, h2c, c2h);
	mutex_unlock(&p->lock);
	return ret;
}

/*
 * Once the firmware runs it owns CTN_TXEN and must be asked through the
 * mailbox; if it does not answer, write the register so TX cannot stay
 * stuck in either state.
 */
static int sch_tx_write(struct ax52_dev *rd, u16 val)
{
	u32 h2c[4] = {}, c2h[4], v;
	int ret;

	v = rd32(rd, CMAC_FUNC_EN);
	if (v == 0xdeadbeef || v == 0xeaeaeaea || !(v & CMAC_EN))
		return -EIO;

	if (rd->fw_ready) {
		h2c[0] = H2CREG_HDR(H2CREG_SCH_TX_EN, 2) |
			 FIELD_PREP(SCH_W0_EN, val);
		h2c[1] = FIELD_PREP(SCH_W1_MASK, 0xffff);
		ret = h2creg_msg(rd, h2c, c2h);
		if (!ret && FIELD_GET(H2CREG_HDR_FUNC, c2h[0]) ==
			    C2HREG_TX_PAUSE_RPT)
			return 0;
		ax52_warn(rd, "TX scheduler 0x%04x not acked by firmware (%d), writing it directly\n",
			  val, ret ?: -EPROTO);
	}
	wr16(rd, CTN_TXEN, val);
	return 0;
}

/* Pauses nest: only the outermost pause/resume touches the hardware. */
int ax52_mac_sch_tx_en(struct ax52_dev *rd, bool enable)
{
	struct ax52_h2c *p = rd->h2c_priv;
	int ret = 0;

	if (!rd->mac_on)
		return -ENODEV;

	mutex_lock(&p->lock);
	if (!enable) {
		if (!p->sch_depth) {
			p->sch_saved = rd16(rd, CTN_TXEN);
			ret = sch_tx_write(rd, 0);
		}
		if (!ret)
			p->sch_depth++;
	} else if (p->sch_depth && !--p->sch_depth) {
		ret = sch_tx_write(rd, p->sch_saved);
	}
	mutex_unlock(&p->lock);
	return ret;
}

/* ------------------------------------------------------------------ C2H */

static void c2h_done_ack(struct ax52_dev *rd, const u8 *c2h, u32 len)
{
	u8 cat, cls, func, seq, err;
	u32 w2;

	if (len < 12)
		return;
	w2 = get_unaligned_le32(c2h + 8);
	err = FIELD_GET(DACK_W2_RET, w2);
	if (!err)
		return;
	cat = FIELD_GET(DACK_W2_CAT, w2);
	cls = FIELD_GET(DACK_W2_CLASS, w2);
	func = FIELD_GET(DACK_W2_FUNC, w2);
	seq = FIELD_GET(DACK_W2_SEQ, w2);
	dev_warn_ratelimited(rd->dev, "H2C %u/%u/%u (seq %u) failed in firmware: %u\n",
			     cat, cls, func, seq, err);
}

/* Formatted records need the firmware's LOGFMT strings; print them raw. */
static void c2h_log(struct ax52_dev *rd, const u8 *p, u32 len)
{
	if (len >= 12 && get_unaligned_le16(p) == 0xA5A5) {
		ax52_dbg(rd, "fw log: fmt %u file %u line %u: %*ph\n",
			 get_unaligned_le32(p + 4), p[8],
			  get_unaligned_le16(p + 9),
			  (int)min_t(u32, len - 12, 64), p + 12);
		return;
	}
	while (len && (p[len - 1] == '\n' || !p[len - 1]))
		len--;
	ax52_dbg(rd, "fw log: %.*s\n", (int)len, p);
}

static const u16 legacy_bitrate[] = {	/* 100 kbps */
	10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540,
};

static const u8 rate_info_bw[] = {
	RATE_INFO_BW_20, RATE_INFO_BW_40, RATE_INFO_BW_80, RATE_INFO_BW_160,
};

static void c2h_ra_report(struct ax52_dev *rd, const u8 *c2h, u32 len)
{
	struct ax52_h2c *p = rd->h2c_priv;
	struct rate_info ri = {};
	u8 rate, giltf;
	u32 w3;

	if (len < 16)
		return;
	if (FIELD_GET(RA_RPT_W2_MACID, get_unaligned_le32(c2h + 8)) != rd->rvif.mac_id)
		return;
	w3 = get_unaligned_le32(c2h + 12);
	rate = FIELD_GET(RA_RPT_W3_RATE, w3);
	giltf = FIELD_GET(RA_RPT_W3_GILTF, w3);

	switch (FIELD_GET(RA_RPT_W3_MODE, w3)) {
	case RA_RPT_LEGACY:
		if (rate >= ARRAY_SIZE(legacy_bitrate))
			return;
		ri.legacy = legacy_bitrate[rate];
		break;
	case RA_RPT_HT:
		ri.flags = RATE_INFO_FLAGS_MCS;
		ri.mcs = FIELD_GET(RA_RATE_HT_MCS, rate);
		if (giltf)
			ri.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case RA_RPT_VHT:
		ri.flags = RATE_INFO_FLAGS_VHT_MCS;
		ri.mcs = FIELD_GET(RA_RATE_MCS, rate);
		ri.nss = FIELD_GET(RA_RATE_NSS, rate) + 1;
		if (ri.mcs > 9)
			return;
		if (giltf)
			ri.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case RA_RPT_HE:
		ri.flags = RATE_INFO_FLAGS_HE_MCS;
		ri.mcs = FIELD_GET(RA_RATE_MCS, rate);
		ri.nss = FIELD_GET(RA_RATE_NSS, rate) + 1;
		if (ri.mcs > 11)
			return;
		if (giltf == GILTF_2XHE08 || giltf == GILTF_1XHE08)
			ri.he_gi = NL80211_RATE_INFO_HE_GI_0_8;
		else if (giltf == GILTF_2XHE16 || giltf == GILTF_1XHE16)
			ri.he_gi = NL80211_RATE_INFO_HE_GI_1_6;
		else
			ri.he_gi = NL80211_RATE_INFO_HE_GI_3_2;
		break;
	}
	ri.bw = rate_info_bw[FIELD_GET(RA_RPT_W3_BW, w3)];

	spin_lock(&p->ra_lock);
	rd->ra.txrate = ri;
	rd->ra.valid = true;
	spin_unlock(&p->ra_lock);
}

bool ax52_h2c_get_txrate(struct ax52_dev *rd, struct rate_info *txrate)
{
	struct ax52_h2c *p = rd->h2c_priv;
	bool valid;

	spin_lock_bh(&p->ra_lock);
	valid = rd->ra.valid;
	if (valid)
		*txrate = rd->ra.txrate;
	spin_unlock_bh(&p->ra_lock);
	return valid;
}

void ax52_c2h_handle(struct ax52_dev *rd, const u8 *c2h, u32 len)
{
	struct ax52_h2c *p = rd->h2c_priv;
	u32 w0, hlen, type;

	if (len < HDR_LEN)
		return;
	w0 = get_unaligned_le32(c2h);
	hlen = FIELD_GET(HDR1_LEN, get_unaligned_le32(c2h + 4));
	type = C2H_TYPE(FIELD_GET(HDR0_CAT, w0), FIELD_GET(HDR0_CLASS, w0),
			FIELD_GET(HDR0_FUNC, w0));
	if (hlen < HDR_LEN || hlen > len)
		hlen = len;

	switch (type) {
	case C2H_REC_ACK:		/* nothing waits for these */
		break;
	case C2H_DONE_ACK:
		c2h_done_ack(rd, c2h, hlen);
		break;
	case C2H_LOG:
		c2h_log(rd, c2h + HDR_LEN, hlen - HDR_LEN);
		break;
	case C2H_RA_RPT:
		c2h_ra_report(rd, c2h, hlen);
		break;
	default:
		if (!test_and_set_bit(type, p->c2h_seen))
			ax52_dbg(rd, "unhandled C2H %lu/%lu/%lu, %u bytes\n",
				 FIELD_GET(HDR0_CAT, w0), FIELD_GET(HDR0_CLASS, w0),
				  FIELD_GET(HDR0_FUNC, w0), hlen);
	}
}

/* -------------------------------------------------------------- bring-up */

#define LOG_W0_LEVEL		GENMASK(7, 0)
#define LOG_W0_PATH		GENMASK(15, 8)
#define LOG_LEVEL_LOUD		4
#define LOG_PATH_C2H		BIT(1)
/* components INIT, TASK, PS, ERROR, MLO, SCAN */
#define LOG_COMPS		(BIT(1) | BIT(2) | BIT(11) | BIT(12) | BIT(26) | BIT(28))

/* Right after MAC init (fresh firmware): offload config, then log config. */
int ax52_h2c_post_mac_init(struct ax52_dev *rd)
{
	static const u8 ofld_cfg[8] = { 0x09, 0, 0, 0, 0x5e, 0, 0, 0 };
	__le32 log[3];
	int ret;

	h2c_reset(rd);

	ret = ax52_h2c_send(rd, H2C_CAT_MAC, CL_FW_OFLD, FUNC_OFLD_CFG,
			    false, true, ofld_cfg, sizeof(ofld_cfg));
	if (ret)
		return ret;

	log[0] = cpu_to_le32(FIELD_PREP(LOG_W0_LEVEL, LOG_LEVEL_LOUD) |
			     FIELD_PREP(LOG_W0_PATH, LOG_PATH_C2H));
	log[1] = cpu_to_le32(fw_log ? LOG_COMPS : 0);
	log[2] = 0;
	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_FW_INFO, FUNC_LOG_CFG,
			      false, false, log, sizeof(log));
}

#define RF_PAGE_DW		500
#define RF_PAGES		3

/* Mirror of the radio table entries with address >= 0x100: (addr << 20) | data. */
int ax52_h2c_rf_reg(struct ax52_dev *rd, u8 path, const u32 *words, u32 n)
{
	__le32 *buf;
	u32 i, cnt;
	u8 page;
	int ret = 0;

	if (path >= RF_PATH_NUM)
		return -EINVAL;
	if (n > RF_PAGE_DW * RF_PAGES) {
		ax52_warn(rd, "RF path %u: firmware takes %u of %u table entries\n",
			  path, RF_PAGE_DW * RF_PAGES, n);
		n = RF_PAGE_DW * RF_PAGES;
	}

	buf = kmalloc_array(RF_PAGE_DW, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	for (page = 0; !ret && n; page++, words += cnt, n -= cnt) {
		cnt = min_t(u32, n, RF_PAGE_DW);
		for (i = 0; i < cnt; i++)
			buf[i] = cpu_to_le32(words[i]);
		ret = ax52_h2c_send(rd, H2C_CAT_OUTSRC, CL_RF_REG_A + path, page,
				    false, false, buf, cnt * sizeof(*buf));
	}
	kfree(buf);
	return ret;
}

/*
 * Which RFK result bank belongs to which channel. Single channel: bank 0
 * holds the current channel, bank 1 is unused.
 */
int ax52_h2c_rf_ntfy_mcc(struct ax52_dev *rd)
{
	__le32 ch = cpu_to_le32(rd->chan.ch);
	__le32 h[5] = { ch, ch, 0, 0, ch };

	return ax52_h2c_send(rd, H2C_CAT_OUTSRC, CL_RF_FW_NOTIFY,
			      FUNC_RF_GET_MCCCH, false, false, h, sizeof(h));
}

/* ------------------------------------------------------------ roles/link */

#define ROLE_W0_MACID		GENMASK(7, 0)
#define ROLE_W0_UPD_MODE	GENMASK(12, 10)
#define ROLE_W0_WIFI_ROLE	GENMASK(16, 13)
#define ROLE_W0_PORT		GENMASK(21, 19)

/* self role CLIENT (0), band 0 */
int ax52_h2c_role_maintain(struct ax52_dev *rd, struct ax52_vif *rv,
			   bool remove)
{
	__le32 w = le32_encode_bits(rv->mac_id, ROLE_W0_MACID) |
		   le32_encode_bits(remove ? ROLE_REMOVE : ROLE_CREATE,
				    ROLE_W0_UPD_MODE) |
		   le32_encode_bits(WIFI_ROLE_STATION, ROLE_W0_WIFI_ROLE) |
		   le32_encode_bits(rv->port, ROLE_W0_PORT);

	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_MEDIA_RPT, FUNC_ROLE_MAINTAIN,
			      false, true, &w, sizeof(w));
}

#define JOIN_W0_MACID		GENMASK(7, 0)
#define JOIN_W0_OP		BIT(8)		/* 1 = disconnect */
#define JOIN_W0_TGR		BIT(12)
#define JOIN_W0_PORT		GENMASK(23, 21)
#define JOIN_W0_NET_TYPE	GENMASK(25, 24)
#define JOIN_W0_WIFI_ROLE	GENMASK(29, 26)

/* disconnect=true also serves add_interface (no link yet). Band 0, WMM 0. */
int ax52_h2c_join_info(struct ax52_dev *rd, struct ax52_vif *rv,
		       bool disconnect)
{
	bool he = !disconnect && rd->vif && rd->vif->bss_conf.he_support;
	__le32 w = le32_encode_bits(rv->mac_id, JOIN_W0_MACID) |
		   le32_encode_bits(disconnect, JOIN_W0_OP) |
		   le32_encode_bits(he, JOIN_W0_TGR) |
		   le32_encode_bits(rv->port, JOIN_W0_PORT) |
		   le32_encode_bits(disconnect ? NET_NO_LINK : NET_INFRA,
				    JOIN_W0_NET_TYPE) |
		   le32_encode_bits(WIFI_ROLE_STATION, JOIN_W0_WIFI_ROLE);

	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_MEDIA_RPT, FUNC_JOININFO,
			      false, true, &w, sizeof(w));
}

/* Address CAM entry with its embedded BSSID CAM entry (AX "v0" format). */
struct h2c_addr_cam {
	__le32 w0;
	__le32 w1;
	__le32 w2;
	__le32 w3;
	u8 sma[ETH_ALEN];	/* w4, w5[15:0] */
	u8 tma[ETH_ALEN];	/* w5[31:16], w6 */
	__le32 w7;
	__le32 w8;
	__le32 w9;
	__le32 w10;		/* w10, w11: security slots */
	__le32 w11;
	__le32 w12;
	__le16 w13;
	u8 bssid[ETH_ALEN];	/* w13[31:16], w14 */
} __packed;
static_assert(sizeof(struct h2c_addr_cam) == 60);

#define ACAM_W1_IDX		GENMASK(7, 0)
#define ACAM_W1_LEN		GENMASK(23, 16)
#define ACAM_W2_VALID		BIT(0)
#define ACAM_W2_NET_TYPE	GENMASK(2, 1)
#define ACAM_W2_SMA_HASH	GENMASK(23, 16)
#define ACAM_W2_TMA_HASH	GENMASK(31, 24)
#define ACAM_W3_BSSID_IDX	GENMASK(5, 0)
#define ACAM_W8_MACID		GENMASK(7, 0)
#define ACAM_W8_PORT_INT	GENMASK(10, 8)
#define ACAM_W8_TSF_SYNC	GENMASK(13, 11)
#define ACAM_W8_TF_TRS		BIT(14)
#define ACAM_W9_AID		GENMASK(11, 0)
#define ACAM_W9_SEC_ENT_MODE	GENMASK(17, 16)
#define ACAM_W9_KEYID_SHIFT	18	/* 2 bits per slot */
#define ACAM_W10_SEC_VALID	GENMASK(7, 0)
#define ACAM_W12_IDX		GENMASK(7, 0)
#define ACAM_W12_LEN		GENMASK(23, 16)
#define ACAM_W13_VALID		BIT(0)
#define ACAM_W13_BSSID_MASK	GENMASK(7, 2)	/* one bit per BSSID byte */
#define ACAM_W13_BSS_COLOR	GENMASK(13, 8)
#define ADDR_CAM_ENT_SIZE	0x40
#define BSSID_CAM_ENT_SIZE	8
#define SEC_ENT_MODE_NORMAL	2

static u8 addr_hash(const u8 *a)
{
	return a[0] ^ a[1] ^ a[2] ^ a[3] ^ a[4] ^ a[5];
}

/*
 * SMA = our address, TMA = the AP (sta) or the BSSID, BSSID CAM = rv->bssid.
 * Uses rv->net_type, aid and bss_color as they are now. vif == NULL
 * invalidates both entries (remove_interface).
 */
int ax52_h2c_addr_cam(struct ax52_dev *rd, struct ax52_vif *rv,
		      struct ieee80211_vif *vif, struct ieee80211_sta *sta)
{
	const u8 *tma = sta ? sta->addr : rv->bssid;
	bool infra = rv->net_type == NET_INFRA;
	bool valid = vif;
	struct h2c_addr_cam h = {};
	int i;

	h.w1 = le32_encode_bits(rv->addr_cam_idx, ACAM_W1_IDX) |
	       le32_encode_bits(ADDR_CAM_ENT_SIZE, ACAM_W1_LEN);
	h.w2 = le32_encode_bits(valid, ACAM_W2_VALID) |
	       le32_encode_bits(rv->net_type, ACAM_W2_NET_TYPE) |
	       le32_encode_bits(addr_hash(rv->addr), ACAM_W2_SMA_HASH) |
	       le32_encode_bits(addr_hash(tma), ACAM_W2_TMA_HASH);
	h.w3 = le32_encode_bits(rv->bssid_cam_idx, ACAM_W3_BSSID_IDX);
	memcpy(h.sma, rv->addr, ETH_ALEN);
	memcpy(h.tma, tma, ETH_ALEN);
	h.w8 = le32_encode_bits(rv->mac_id, ACAM_W8_MACID) |
	       le32_encode_bits(rv->port, ACAM_W8_PORT_INT) |
	       le32_encode_bits(rv->port, ACAM_W8_TSF_SYNC) |
	       le32_encode_bits(valid && infra && vif->bss_conf.he_support,
				ACAM_W8_TF_TRS);
	h.w9 = le32_encode_bits(infra ? rv->aid : 0, ACAM_W9_AID) |
	       le32_encode_bits(SEC_ENT_MODE_NORMAL, ACAM_W9_SEC_ENT_MODE);
	/* key slots: valid bitmap, key IDs, security-CAM indices */
	for (i = 0; valid && i < ARRAY_SIZE(rv->sec_ent); i++) {
		if (!(rv->sec_ent_map & BIT(i)))
			continue;
		h.w9 |= cpu_to_le32((rv->sec_ent_keyid[i] & 3) <<
				    (ACAM_W9_KEYID_SHIFT + 2 * i));
		if (i < 3)
			h.w10 |= cpu_to_le32(rv->sec_ent[i] << (8 + 8 * i));
		else
			h.w11 |= cpu_to_le32(rv->sec_ent[i] << (8 * (i - 3)));
	}
	if (valid)
		h.w10 |= le32_encode_bits(rv->sec_ent_map, ACAM_W10_SEC_VALID);
	h.w12 = le32_encode_bits(rv->bssid_cam_idx, ACAM_W12_IDX) |
		le32_encode_bits(BSSID_CAM_ENT_SIZE, ACAM_W12_LEN);
	h.w13 = le16_encode_bits(valid, ACAM_W13_VALID) |
		le16_encode_bits(valid && vif->bss_conf.nontransmitted ?
				 0x1f : 0x3f, ACAM_W13_BSSID_MASK) |
		le16_encode_bits(rv->bss_color, ACAM_W13_BSS_COLOR);
	memcpy(h.bssid, rv->bssid, ETH_ALEN);

	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_ADDR_CAM, FUNC_ADDR_CAM_UPD,
			      false, true, &h, sizeof(h));
}

/*
 * Security CAM entry: [7:0] index, [15:8] offset, [23:16] length (20), then
 * [3:0] type, [4] second half of a 256-bit key, and the 16 key bytes.
 */
#define SCAM_W0_IDX		GENMASK(7, 0)
#define SCAM_W0_LEN		GENMASK(23, 16)
#define SCAM_W1_TYPE		GENMASK(3, 0)
#define SCAM_W1_EXT_KEY		BIT(4)
#define SEC_CAM_ENT_LEN		20

int ax52_h2c_sec_cam(struct ax52_dev *rd, u8 idx, u8 type, const u8 *key,
		     u8 keylen)
{
	int half, ret;

	for (half = 0; half * 16 < keylen; half++) {
		__le32 h[6] = {};

		h[0] = le32_encode_bits(idx + half, SCAM_W0_IDX) |
		       le32_encode_bits(SEC_CAM_ENT_LEN, SCAM_W0_LEN);
		h[1] = le32_encode_bits(type, SCAM_W1_TYPE) |
		       le32_encode_bits(half, SCAM_W1_EXT_KEY);
		memcpy(&h[2], key + half * 16, min_t(u8, 16, keylen - half * 16));
		ret = ax52_h2c_send(rd, H2C_CAT_MAC, CL_SEC_CAM, FUNC_SEC_UPD,
				    true, false, h, sizeof(h));
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * CMAC control table: dword 0 selects the mac_id, dwords 1..8 are values and
 * dwords 9..16 their write masks; the firmware updates masked fields only.
 */
#define CCTL_DW			17
#define CCTL_W0_MACID		GENMASK(6, 0)
#define CCTL_W0_WRITE		BIT(7)

static void cctl_set(__le32 *t, int dw, u32 mask, u32 val)
{
	t[dw] |= cpu_to_le32((val << __ffs(mask)) & mask);
	t[dw + 8] |= cpu_to_le32(mask);
}

static int cctl_send(struct ax52_dev *rd, __le32 *t, u8 mac_id)
{
	t[0] = cpu_to_le32(FIELD_PREP(CCTL_W0_MACID, mac_id) | CCTL_W0_WRITE);
	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_FR_EXCHG, FUNC_CCTLINFO_UD,
			      false, true, t, CCTL_DW * sizeof(*t));
}

int ax52_h2c_default_cmac_tbl(struct ax52_dev *rd, u8 mac_id)
{
	__le32 t[CCTL_DW] = {};

	cctl_set(t, 5, GENMASK(11, 9), 0);	/* TXPWR_MODE */
	cctl_set(t, 6, GENMASK(19, 16), 3);	/* NTX_PATH_EN: A and B */
	cctl_set(t, 6, GENMASK(21, 20), 0);	/* PATH_MAP_A */
	cctl_set(t, 6, GENMASK(23, 22), 1);	/* PATH_MAP_B */
	cctl_set(t, 6, GENMASK(27, 24), 0);	/* PATH_MAP_C, PATH_MAP_D */
	cctl_set(t, 6, GENMASK(31, 28), 0);	/* ANTSEL_A..D */
	cctl_set(t, 1, BIT(21), 0);		/* MGQ_RPT_EN (USB only) */
	cctl_set(t, 7, GENMASK(19, 18), 0);	/* DOPPLER_CTRL */
	cctl_set(t, 7, GENMASK(27, 24), 0);	/* TXPWR_TOLERENCE */
	return cctl_send(rd, t, mac_id);
}

#define PE_0US			0
#define PE_8US			1
#define PE_16US			2

/* HE packet extension the AP needs, per bandwidth 20/40/80/160. */
static void he_pkt_padding(const struct ieee80211_link_sta *ls, u8 pads[4])
{
	const u8 *phy = ls->he_cap.he_cap_elem.phy_cap_info;
	const u8 *ppe = ls->he_cap.ppe_thres;
	u8 nss = min_t(u8, max_t(u8, ls->rx_nss, 1), TX_NSS) - 1;
	u8 ru, sh, ppe16, ppe8;
	u16 bits;
	u32 n;
	int i;

	if (!(phy[6] & IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT)) {
		memset(pads, FIELD_GET(IEEE80211_HE_PHY_CAP9_NOMINAL_PKT_PADDING_MASK,
				       phy[9]), 4);
		return;
	}

	/* {PPET16, PPET8} pairs of 3 bits per NSS and RU size, from bit 7 */
	ru = FIELD_GET(IEEE80211_PPE_THRES_RU_INDEX_BITMASK_MASK, ppe[0]);
	n = 7 + hweight8(ru) * IEEE80211_PPE_THRES_INFO_PPET_SIZE * 2 * nss;
	for (i = 0; i < 4; i++) {
		if (!(ru & BIT(i))) {
			pads[i] = PE_8US;
			continue;
		}
		bits = get_unaligned_le16(ppe + n / 8);
		sh = n % 8;
		n += IEEE80211_PPE_THRES_INFO_PPET_SIZE * 2;
		ppe16 = (bits >> sh) & IEEE80211_PPE_THRES_NSS_MASK;
		ppe8 = (bits >> (sh + IEEE80211_PPE_THRES_INFO_PPET_SIZE)) &
		       IEEE80211_PPE_THRES_NSS_MASK;
		if (ppe16 != 7 && ppe8 == 7)
			pads[i] = PE_16US;
		else if (ppe8 != 7)
			pads[i] = PE_8US;
		else
			pads[i] = PE_0US;
	}
}

/* Sent at assoc and again at disconnect for the AP (sta may be NULL). */
int ax52_h2c_assoc_cmac_tbl(struct ax52_dev *rd, struct ax52_vif *rv,
			    struct ieee80211_vif *vif, struct ieee80211_sta *sta)
{
	struct ieee80211_link_sta *ls = sta ? &sta->deflink : NULL;
	__le32 t[CCTL_DW] = {};
	u8 pads[4] = {};

	if (ls && ls->he_cap.has_he)
		he_pkt_padding(ls, pads);

	cctl_set(t, 1, BIT(25), 1);		/* DISRTSFB */
	cctl_set(t, 1, BIT(26), 1);		/* DISDATAFB: FW RA picks rates */
	cctl_set(t, 2, GENMASK(31, 28),		/* RTS_RTY_LOWEST_RATE */
		 rd->chan.band == AX52_BAND_2G ? HW_RATE_CCK1 : HW_RATE_OFDM6);
	cctl_set(t, 2, BIT(11), 0);		/* RTS_TXCNT_LMT_SEL */
	cctl_set(t, 3, BIT(6), 0);		/* DATA_TXCNT_LMT_SEL */
	cctl_set(t, 7, BIT(17),			/* ULDL */
		 vif->type == NL80211_IFTYPE_STATION);
	cctl_set(t, 5, GENMASK(2, 0), rv->port);	/* MULTI_PORT_ID */
	cctl_set(t, 7, GENMASK(21, 20), pads[0]);	/* NOMINAL_PKT_PADDING */
	cctl_set(t, 7, GENMASK(23, 22), pads[1]);	/* ... 40 MHz */
	cctl_set(t, 7, GENMASK(31, 30), pads[2]);	/* ... 80 MHz */
	cctl_set(t, 8, GENMASK(29, 28), pads[3]);	/* ... 160 MHz */
	if (ls)
		cctl_set(t, 6, BIT(13), ls->he_cap.has_he); /* BSR_QUEUE_SIZE_FORMAT */
	return cctl_send(rd, t, rv->mac_id);
}

int ax52_h2c_macid_pause(struct ax52_dev *rd, u8 mac_id, bool pause)
{
	struct {
		__le32 pause[4];
		__le32 mask[4];
	} __packed h = {};
	__le32 bit = cpu_to_le32(BIT(mac_id % 32));

	if (mac_id >= 128)
		return -EINVAL;
	h.mask[mac_id / 32] = bit;
	if (pause)
		h.pause[mac_id / 32] = bit;
	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_FW_OFLD, FUNC_MACID_PAUSE,
			      true, false, &h, sizeof(h));
}

#define EDCA_W0_AC		GENMASK(6, 5)	/* SEL, BAND, WMM: 0 */
#define EDCA_W1_AIFS		GENMASK(7, 0)	/* microseconds */
#define EDCA_W1_CWMIN		GENMASK(11, 8)	/* exponent */
#define EDCA_W1_CWMAX		GENMASK(15, 12)
#define EDCA_W1_TXOP		GENMASK(26, 16)	/* 32 us units */

static const u8 ac_to_fw[IEEE80211_NUM_ACS] = {
	[IEEE80211_AC_BE] = 0,
	[IEEE80211_AC_BK] = 1,
	[IEEE80211_AC_VI] = 2,
	[IEEE80211_AC_VO] = 3,
};

/* @ac is the mac80211 AC. Resend all four when the slot time changes. */
int ax52_h2c_edca(struct ax52_dev *rd, struct ax52_vif *rv, u8 ac,
		  const struct ieee80211_tx_queue_params *p)
{
	u32 slot = rd->vif && rd->vif->bss_conf.use_short_slot ? 9 : 20;
	u32 sifs = rd->chan.band == AX52_BAND_2G ? 10 : 16;
	__le32 h[3];

	if (ac >= IEEE80211_NUM_ACS)
		return -EINVAL;
	h[0] = le32_encode_bits(ac_to_fw[ac], EDCA_W0_AC);
	h[1] = le32_encode_bits(min(p->aifs * slot + sifs, 255U), EDCA_W1_AIFS) |
	       le32_encode_bits(ilog2(p->cw_min + 1), EDCA_W1_CWMIN) |
	       le32_encode_bits(ilog2(p->cw_max + 1), EDCA_W1_CWMAX) |
	       le32_encode_bits(p->txop, EDCA_W1_TXOP);
	h[2] = 0;
	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_FW_OFLD, FUNC_USR_EDCA,
			      false, true, h, sizeof(h));
}

/* ------------------------------------------------------ rate adaptation */

#define RA_W0_MODE		GENMASK(5, 1)
#define RA_W0_BW_CAP		GENMASK(7, 6)
#define RA_W0_MACID		GENMASK(15, 8)
#define RA_W0_DCM		BIT(16)
#define RA_W0_ER		BIT(17)
#define RA_W0_INIT_RATE_LV	GENMASK(19, 18)
#define RA_W0_UPD_ALL		BIT(20)
#define RA_W0_SGI		BIT(21)
#define RA_W0_LDPC		BIT(22)
#define RA_W0_STBC		BIT(23)
#define RA_W0_SS_NUM		GENMASK(26, 24)
#define RA_W0_UPD_BW_NSS_MASK	BIT(30)
#define RA_W0_UPD_MASK		BIT(31)
#define RA_W2_MASK_HI		GENMASK(30, 0)
#define RA_W2_BFEE_CSI_CTL	BIT(31)
#define RA_W3_CSI_RATE_EN	BIT(8)
#define RA_W3_FIX_GILTF_EN	BIT(11)
#define RA_W3_FIX_GILTF		GENMASK(14, 12)
#define RA_W3_CSI_MCS_SS_IDX	GENMASK(23, 16)
#define RA_W3_CSI_MODE		GENMASK(25, 24)
#define RA_W3_CSI_BW		GENMASK(31, 29)

#define RA_MODE_CCK		BIT(0)
#define RA_MODE_OFDM		BIT(1)
#define RA_MODE_HT		BIT(2)
#define RA_MODE_VHT		BIT(3)
#define RA_MODE_HE		BIT(4)

/*
 * Rate mask: [3:0] CCK, [11:4] OFDM, then 12 bits per spatial stream from
 * bit 12 (HT MCS0-7, VHT MCS0-9, HE MCS0-11).
 */
#define RA_MASK_CCK		GENMASK_ULL(3, 0)
#define RA_MASK_OFDM		GENMASK_ULL(11, 4)
#define RA_MASK_SUBCCK		0x5ULL		/* 1M, 5.5M */
#define RA_MASK_SUBOFDM		0x10ULL		/* 6M */
#define RA_MASK_HT_2SS		(GENMASK_ULL(19, 12) | GENMASK_ULL(31, 24))
#define RA_MASK_VHT_2SS		(GENMASK_ULL(21, 12) | GENMASK_ULL(33, 24))
#define RA_MASK_HE_2SS		(GENMASK_ULL(23, 12) | GENMASK_ULL(35, 24))

/* VHT/HE MCS map: 2 bits per NSS, 2 = up to @top, 1 = @top - @gap, ... */
static u64 ra_mcs_map_mask(u16 map, u8 top, u8 gap)
{
	u64 mask = 0;
	int nss;

	for (nss = 0; nss < 4; nss++, map >>= 2) {
		if ((map & 3) == IEEE80211_VHT_MCS_NOT_SUPPORTED)
			continue;
		mask |= GENMASK_ULL(top - gap * (2 - (map & 3)), 0) << (12 + 12 * nss);
	}
	return mask;
}

/* Signal in dBm + 110; 0 while mac80211 has no beacon average yet. */
static u8 ra_rssi(struct ieee80211_vif *vif)
{
	int dbm = ieee80211_ave_rssi(vif, -1);

	return dbm < 0 ? clamp(dbm + 110, 0, 110) : 0;
}

/* With a strong signal, drop the lowest rates from the candidates. */
static u64 ra_rssi_floor(u8 rssi)
{
	static const u8 lv_th[] = { 33, 47, 51, 55, 59, 63, 103 };
	static const u64 floor[] = {
		~0ULL, 0xfffffffffffffff0ULL, 0xffffffffffffefe0ULL,
		0xffffffffffffcfc0ULL, 0xffffffffffff8f80ULL,
		0xffffffffffff0f00ULL,
	};
	int lv;

	for (lv = 0; lv < ARRAY_SIZE(lv_th); lv++)
		if (rssi < lv_th[lv])
			break;
	if (lv == ARRAY_SIZE(lv_th))
		lv = 0;
	return floor[min_t(int, lv, ARRAY_SIZE(floor) - 1)];
}

static bool sta_is_beamformer(const struct ieee80211_link_sta *ls)
{
	const u8 *he = ls->he_cap.he_cap_elem.phy_cap_info;

	return (ls->vht_cap.cap & (IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE |
				   IEEE80211_VHT_CAP_MU_BEAMFORMER_CAPABLE)) ||
	       (he[3] & IEEE80211_HE_PHY_CAP3_SU_BEAMFORMER) ||
	       (he[4] & IEEE80211_HE_PHY_CAP4_MU_BEAMFORMER);
}

/*
 * Hand the AP to the firmware rate adaptation: update=false at assoc (full
 * setup, beamforming CSI parameters), true for rc_update and the periodic
 * 2 s refresh (new mask; BW/NSS flagged only when they changed).
 */
int ax52_h2c_ra(struct ax52_dev *rd, struct ax52_vif *rv,
		struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		 bool update)
{
	struct ax52_h2c *p = rd->h2c_priv;
	struct ieee80211_bss_conf *bss = &vif->bss_conf;
	struct ieee80211_link_sta *ls;
	u8 mode = 0, csi_mode = RA_RPT_LEGACY, bw, nss, rssi, lv = 0;
	bool he, sgi, stbc = false, ldpc = false, dcm, er, csi, upd_bw_nss;
	const u8 *hephy;
	u64 mask = 0, bak, cap = 0;
	__le32 h[4];

	if (!sta)
		return -EINVAL;
	ls = &sta->deflink;
	hephy = ls->he_cap.he_cap_elem.phy_cap_info;
	he = ls->he_cap.has_he;

	if (he) {
		mode |= RA_MODE_HE;
		csi_mode = RA_RPT_HE;
		/* MCS 11/9/7; 8852B stops at 80 MHz */
		mask |= ra_mcs_map_mask(le16_to_cpu(ls->he_cap.he_mcs_nss_supp.rx_mcs_80),
					11, 2);
		cap = RA_MASK_HE_2SS;
		stbc = hephy[2] & IEEE80211_HE_PHY_CAP2_STBC_RX_UNDER_80MHZ;
		ldpc = hephy[1] & IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD;
	} else if (ls->vht_cap.vht_supported) {
		mode |= RA_MODE_VHT;
		csi_mode = RA_RPT_VHT;
		/* MCS 9/8/7, or 8/7/6 at 20 MHz */
		mask |= ra_mcs_map_mask(le16_to_cpu(ls->vht_cap.vht_mcs.rx_mcs_map),
					ls->bandwidth == IEEE80211_STA_RX_BW_20 ? 8 : 9, 1);
		cap = RA_MASK_VHT_2SS;
		stbc = ls->vht_cap.cap & IEEE80211_VHT_CAP_RXSTBC_MASK;
		ldpc = ls->vht_cap.cap & IEEE80211_VHT_CAP_RXLDPC;
	} else if (ls->ht_cap.ht_supported) {
		const u8 *rx = ls->ht_cap.mcs.rx_mask;

		mode |= RA_MODE_HT;
		csi_mode = RA_RPT_HT;
		mask |= (u64)rx[0] << 12 | (u64)rx[1] << 24 |
			(u64)rx[2] << 36 | (u64)rx[3] << 48;
		cap = RA_MASK_HT_2SS;
		stbc = ls->ht_cap.cap & IEEE80211_HT_CAP_RX_STBC;
		ldpc = ls->ht_cap.cap & IEEE80211_HT_CAP_LDPC_CODING;
	}

	if (rd->chan.band == AX52_BAND_2G) {
		u32 r = ls->supp_rates[NL80211_BAND_2GHZ];

		mask |= r;
		if (r & 0xf)
			mode |= RA_MODE_CCK;
		if (r & 0xff0)
			mode |= RA_MODE_OFDM;
	} else {
		mask |= (u64)ls->supp_rates[NL80211_BAND_5GHZ] << 4;
		mode |= RA_MODE_OFDM;
	}

	bak = mask;
	if (cap) {
		if (mode & RA_MODE_OFDM)
			cap |= RA_MASK_SUBOFDM;
		if (mode & RA_MODE_CCK)
			cap |= RA_MASK_SUBCCK;
		mask &= cap;
	} else if (mode & RA_MODE_OFDM) {
		mask &= RA_MASK_OFDM | RA_MASK_SUBCCK;
	}
	rssi = ra_rssi(vif);
	if (mode != RA_MODE_CCK)
		mask &= ra_rssi_floor(rssi);
	/* never leave the firmware without a rate to pick */
	if (!(mask & ~(RA_MASK_CCK | RA_MASK_OFDM)))
		mask |= bak & ~(RA_MASK_CCK | RA_MASK_OFDM);
	if (!mask)
		mask = bak & (RA_MASK_CCK | RA_MASK_OFDM);

	switch (ls->bandwidth) {
	case IEEE80211_STA_RX_BW_20:
		bw = AX52_BW_20;
		sgi = ls->ht_cap.ht_supported &&
		      (ls->ht_cap.cap & IEEE80211_HT_CAP_SGI_20);
		break;
	case IEEE80211_STA_RX_BW_40:
		bw = AX52_BW_40;
		sgi = ls->ht_cap.ht_supported &&
		      (ls->ht_cap.cap & IEEE80211_HT_CAP_SGI_40);
		break;
	default:
		bw = AX52_BW_80;
		sgi = ls->vht_cap.vht_supported &&
		      (ls->vht_cap.cap & IEEE80211_VHT_CAP_SHORT_GI_80);
		break;
	}

	dcm = hephy[3] & IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_16_QAM;
	er = bss->he_support &&
	     !(bss->he_oper.params & IEEE80211_HE_OPERATION_ER_SU_DISABLE);
	nss = min_t(u8, max_t(u8, ls->rx_nss, 1), TX_NSS) - 1;
	csi = !update && sta_is_beamformer(ls);
	if (!update)
		lv = rssi > 40 ? 1 : rssi > 20 ? 2 : rssi > 1 ? 3 : 0;

	/* no mutex here: link_sta_rc_update runs under rcu_read_lock */
	spin_lock_bh(&p->ra_lock);
	upd_bw_nss = update && (bw != p->ra_bw || nss != p->ra_nss);
	p->ra_bw = bw;
	p->ra_nss = nss;
	if (!update)
		rd->ra.valid = false;
	spin_unlock_bh(&p->ra_lock);

	h[0] = le32_encode_bits(mode, RA_W0_MODE) |
	       le32_encode_bits(bw, RA_W0_BW_CAP) |
	       le32_encode_bits(rv->mac_id, RA_W0_MACID) |
	       le32_encode_bits(dcm, RA_W0_DCM) |
	       le32_encode_bits(er, RA_W0_ER) |
	       le32_encode_bits(lv, RA_W0_INIT_RATE_LV) |
	       le32_encode_bits(!update, RA_W0_UPD_ALL) |
	       le32_encode_bits(sgi, RA_W0_SGI) |
	       le32_encode_bits(ldpc, RA_W0_LDPC) |
	       le32_encode_bits(stbc, RA_W0_STBC) |
	       le32_encode_bits(nss, RA_W0_SS_NUM) |
	       le32_encode_bits(upd_bw_nss, RA_W0_UPD_BW_NSS_MASK) |
	       le32_encode_bits(update, RA_W0_UPD_MASK);
	h[1] = cpu_to_le32(lower_32_bits(mask));
	h[2] = le32_encode_bits(upper_32_bits(mask), RA_W2_MASK_HI) |
	       le32_encode_bits(csi, RA_W2_BFEE_CSI_CTL);
	h[3] = le32_encode_bits(he, RA_W3_FIX_GILTF_EN) |
	       le32_encode_bits(he ? GILTF_2XHE08 : 0, RA_W3_FIX_GILTF);
	if (csi)	/* band 0, FW-chosen CSI rate from MCS5 */
		h[3] |= le32_encode_bits(1, RA_W3_CSI_RATE_EN) |
			le32_encode_bits(5, RA_W3_CSI_MCS_SS_IDX) |
			le32_encode_bits(csi_mode, RA_W3_CSI_MODE) |
			le32_encode_bits(bw, RA_W3_CSI_BW);
	return ax52_h2c_send(rd, H2C_CAT_OUTSRC, CL_RA, FUNC_RA_MACIDCFG,
			      false, false, h, sizeof(h));
}

/* ------------------------------------------------------------- RX BA CAM */

#define BA_W0_VALID		BIT(0)
#define BA_W0_INIT_REQ		BIT(1)		/* HW takes the SSN */
#define BA_W0_ENTRY		GENMASK(3, 2)
#define BA_W0_TID		GENMASK(7, 4)
#define BA_W0_MACID		GENMASK(15, 8)
#define BA_W0_BMAP_SIZE		GENMASK(19, 16)	/* 0: 64, 4: 256 */
#define BA_W0_SSN		GENMASK(31, 20)

/* Static entry for @tid; TID 0 and 5 may take one over from another TID. */
static int ba_cam_get(struct ax52_h2c *p, u8 tid)
{
	int i;

	for (i = 0; i < BA_CAM_NUM; i++)
		if (p->ba_tid[i] == BA_TID_NONE)
			goto found;
	if (tid != 0 && tid != 5)
		return -ENOSPC;
	for (i = 0; i < BA_CAM_NUM; i++)
		if (p->ba_tid[i] != 0 && p->ba_tid[i] != 5)
			goto found;
	return -ENOSPC;
found:
	p->ba_tid[i] = tid;
	return i;
}

static int ba_cam_put(struct ax52_h2c *p, u8 tid)
{
	int i;

	for (i = 0; i < BA_CAM_NUM; i++) {
		if (p->ba_tid[i] == tid) {
			p->ba_tid[i] = BA_TID_NONE;
			return i;
		}
	}
	return -ENOENT;
}

/*
 * RX_START (valid) / RX_STOP. Without a static entry nothing is sent: the
 * hardware then builds a dynamic BA CAM entry itself.
 */
int ax52_h2c_ba_cam(struct ax52_dev *rd, struct ax52_vif *rv, u8 tid,
		    u16 ssn, u16 buf_size, bool valid)
{
	struct ax52_h2c *p = rd->h2c_priv;
	__le32 h[2] = {};
	int entry;

	mutex_lock(&p->lock);
	entry = valid ? ba_cam_get(p, tid) : ba_cam_put(p, tid);
	mutex_unlock(&p->lock);
	if (entry < 0)
		return 0;

	h[0] = le32_encode_bits(entry, BA_W0_ENTRY) |
	       le32_encode_bits(rv->mac_id, BA_W0_MACID);
	if (valid)
		h[0] |= le32_encode_bits(1, BA_W0_VALID) |
			le32_encode_bits(1, BA_W0_INIT_REQ) |
			le32_encode_bits(tid, BA_W0_TID) |
			le32_encode_bits(buf_size > 64 ? 4 : 0, BA_W0_BMAP_SIZE) |
			le32_encode_bits(ssn, BA_W0_SSN);
	return ax52_h2c_send(rd, H2C_CAT_MAC, CL_BA_CAM, FUNC_BA_CAM,
			      false, true, h, sizeof(h));
}
