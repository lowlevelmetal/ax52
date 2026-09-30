// SPDX-License-Identifier: GPL-2.0
/*
 * Frame reception: RX descriptor parsing, rate/status translation for
 * mac80211, and pairing of MPDUs with the PPDU-status reports that carry
 * their signal strength.
 */
#include "ax52.h"

/* RX descriptor, dword 0 */
#define RXD0_PKT_LEN		GENMASK(13, 0)
#define RXD0_SHIFT		GENMASK(15, 14)
#define RXD0_MAC_INFO_VLD	BIT(23)
#define RXD0_RPKT_TYPE		GENMASK(27, 24)
#define RXD0_DRV_INFO_SIZE	GENMASK(30, 28)
#define RXD0_LONG		BIT(31)
/* dword 1 */
#define RXD1_PPDU_CNT		GENMASK(6, 4)
#define RXD1_DATA_RATE		GENMASK(24, 16)
#define RXD1_GI_LTF		GENMASK(27, 25)
#define RXD1_BW			GENMASK(31, 30)
/* dword 3 */
#define RXD3_A1_MATCH		BIT(0)
#define RXD3_SW_DEC		BIT(1)
#define RXD3_HW_DEC		BIT(2)
#define RXD3_AMPDU		BIT(3)
#define RXD3_CRC32_ERR		BIT(9)
#define RXD3_ICV_ERR		BIT(10)
/* dword 4 (long descriptor only) */
#define RXD4_TYPE		GENMASK(1, 0)
/* dword 7 (long descriptor only) */
#define RXD7_SEC_TYPE		GENMASK(20, 17)

enum rpkt_type {
	RPKT_WIFI = 0,
	RPKT_PPDU_STAT = 1,
	RPKT_C2H = 10,
};

#define PPDU_Q_MAX		64

u32 ax52_rx_payload_offset(const u8 *desc, u32 *pkt_len, u8 *type)
{
	u32 dw0 = le32_to_cpu(*(const __le32 *)desc);

	*pkt_len = FIELD_GET(RXD0_PKT_LEN, dw0);
	*type = FIELD_GET(RXD0_RPKT_TYPE, dw0);
	return (dw0 & RXD0_LONG ? 32 : 16) +
	       FIELD_GET(RXD0_DRV_INFO_SIZE, dw0) * 8 +
	       FIELD_GET(RXD0_SHIFT, dw0) * 2;
}

/* ------------------------------------------------------ rate decoding */

static const u8 he_gi_map[8] = {
	[0] = NL80211_RATE_INFO_HE_GI_3_2, [1] = NL80211_RATE_INFO_HE_GI_0_8,
	[2] = NL80211_RATE_INFO_HE_GI_1_6, [3] = NL80211_RATE_INFO_HE_GI_0_8,
	[4] = NL80211_RATE_INFO_HE_GI_1_6, [5] = NL80211_RATE_INFO_HE_GI_0_8,
	[6] = NL80211_RATE_INFO_HE_GI_3_2, [7] = NL80211_RATE_INFO_HE_GI_3_2,
};

static void fill_rate(struct ieee80211_rx_status *st, u16 rate, u8 gi, u8 bw,
		      bool is_5g)
{
	static const u8 bw_map[4] = {
		RATE_INFO_BW_20, RATE_INFO_BW_40, RATE_INFO_BW_80, RATE_INFO_BW_160,
	};

	st->bw = bw_map[bw & 3];
	switch (rate >> 7) {
	case 0:		/* legacy: 0-3 CCK, 4-11 OFDM */
		st->encoding = RX_ENC_LEGACY;
		st->rate_idx = rate & 0xf;
		if (is_5g)
			st->rate_idx = st->rate_idx >= 4 ? st->rate_idx - 4 : 0;
		break;
	case 1:
		st->encoding = RX_ENC_HT;
		st->rate_idx = rate & 0x1f;
		if (gi)
			st->enc_flags |= RX_ENC_FLAG_SHORT_GI;
		break;
	case 2:
		st->encoding = RX_ENC_VHT;
		st->rate_idx = rate & 0xf;
		st->nss = ((rate >> 4) & 7) + 1;
		if (gi)
			st->enc_flags |= RX_ENC_FLAG_SHORT_GI;
		break;
	case 3:
		st->encoding = RX_ENC_HE;
		st->rate_idx = rate & 0xf;
		st->nss = ((rate >> 4) & 7) + 1;
		st->he_gi = he_gi_map[gi & 7];
		break;
	}
}

/* ------------------------------------------------------ PPDU status */

struct ppdu_info {
	u8 cnt;
	u16 rate;
	u8 bw;
	s8 rssi[2];		/* dBm per path */
	u8 ch_idx;		/* IE01 channel index, 0 if absent */
};

static s8 raw_to_dbm(u8 raw)
{
	return (s8)(raw >> 1) - 110;
}

/* PHY-status IE sizes in bytes; 0xff = variable (length field), 0 = stop */
static const u8 ie_len[32] = {
	16, 32, 24, 24, 8, 8, 8, 8, 0xff, 8, 0xff, 176,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 16, 24, 0xff, 0xff, 0xff, 0,
	24, 24, 24, 24, 32, 32, 32, 32,
};

static bool parse_phy_sts(const u8 *p, u32 len, struct ppdu_info *pi)
{
	u32 w0, w1, off, sts_len;
	u8 rssi_avg, rssi[2], rpl;
	bool have_rpl = false;

	if (len < 8)
		return false;
	w0 = le32_to_cpu(*(const __le32 *)p);
	w1 = le32_to_cpu(*(const __le32 *)(p + 4));
	sts_len = ((w0 >> 8) & 0xff) * 8;
	if (!(w0 & BIT(7)) || sts_len > len)
		return false;

	rssi_avg = w0 >> 24;
	rssi[0] = w1 & 0xff;
	rssi[1] = (w1 >> 8) & 0xff;
	rpl = rssi_avg;

	/* HDR_2_EN: a second 8-byte header precedes the IEs */
	for (off = (w0 & BIT(5)) ? 16 : 8; off + 4 <= sts_len;) {
		u32 ie = le32_to_cpu(*(const __le32 *)(p + off));
		u8 type = ie & 0x1f, n = ie_len[type];
		u32 sz = n == 0xff ? ((ie >> 5) & 0x7f) * 8 : n;

		if (!sz || off + sz > sts_len)
			break;
		if (type == 0) {		/* CCK common */
			rpl = ((ie >> 7) & 0x1ff) >> 1;
			have_rpl = true;
		} else if (type == 1) {		/* OFDM common */
			if (!have_rpl)
				rpl = (ie >> 8) & 0xff;
			pi->ch_idx = (ie >> 16) & 0xff;
			have_rpl = true;
		}
		off += sz;
	}

	/* per-path RSSI is relative to the average; rebase on the RPL */
	pi->rssi[0] = raw_to_dbm(rssi[0] + (u8)(rpl - rssi_avg));
	pi->rssi[1] = raw_to_dbm(rssi[1] + (u8)(rpl - rssi_avg));
	return true;
}

static u16 ch_idx_to_freq(u8 idx, enum nl80211_band *band)
{
	static const u8 base5[4] = { 36, 100, 132, 149 };
	u8 b = idx >> 4, o = idx & 0xf;

	if (b == 0 && o) {
		*band = NL80211_BAND_2GHZ;
		return ieee80211_channel_to_frequency(o, *band);
	}
	if (b >= 2 && b <= 5) {
		*band = NL80211_BAND_5GHZ;
		return ieee80211_channel_to_frequency(base5[b - 2] + 2 * o, *band);
	}
	return 0;
}

static void deliver(struct ax52_dev *rd, struct sk_buff *skb)
{
	struct ieee80211_rx_status *st = IEEE80211_SKB_RXCB(skb);

	if (!st->signal)
		st->flag |= RX_FLAG_NO_SIGNAL_VAL;
	ieee80211_rx_napi(rd->hw, NULL, skb, &rd->napi);
}

static void flush_ppdu(struct ax52_dev *rd)
{
	struct sk_buff *skb;

	while ((skb = __skb_dequeue(&rd->ppdu_q)))
		deliver(rd, skb);
}

/* A PPDU status arrived: give its signal to the queued MPDUs it describes. */
static void rx_ppdu_status(struct ax52_dev *rd, const u8 *desc,
			   const u8 *p, u32 len)
{
	u32 dw0 = le32_to_cpu(*(const __le32 *)desc);
	u32 dw1 = le32_to_cpu(*(const __le32 *)(desc + 4));
	struct ppdu_info pi = {
		.cnt = FIELD_GET(RXD1_PPDU_CNT, dw1),
		.rate = FIELD_GET(RXD1_DATA_RATE, dw1),
		.bw = FIELD_GET(RXD1_BW, dw1),
	};
	struct ieee80211_rx_status ref = {};
	struct sk_buff *skb;
	bool ok = false;

	/* a malformed report still releases the MPDUs waiting for it */
	if (dw0 & RXD0_MAC_INFO_VLD) {
		u32 w0, w1, skip;
		u8 usr;

		if (len < 8)
			goto release;
		w0 = le32_to_cpu(*(const __le32 *)p);
		w1 = le32_to_cpu(*(const __le32 *)(p + 4));
		usr = w0 & 0xf;
		if (usr > 4)
			goto release;
		skip = 8 + usr * 4 + (usr & 1) * 4 +
		       ((w0 & BIT(29)) ? 96 : 0) + ((w1 >> 16) & 0xff) * 8;
		if (skip > len)
			goto release;
		p += skip;
		len -= skip;
	}
	ok = parse_phy_sts(p, len, &pi);

release:
	fill_rate(&ref, pi.rate, 0, pi.bw, rd->chan.band == AX52_BAND_5G);
	while ((skb = __skb_dequeue(&rd->ppdu_q))) {
		struct ieee80211_rx_status *st = IEEE80211_SKB_RXCB(skb);

		if (ok && rd->ppdu_cnt == pi.cnt && st->encoding == ref.encoding &&
		    st->rate_idx == ref.rate_idx && st->bw == ref.bw) {
			enum nl80211_band band;
			u16 freq = ch_idx_to_freq(pi.ch_idx, &band);

			st->signal = max(pi.rssi[0], pi.rssi[1]);
			st->chains = BIT(0) | BIT(1);
			st->chain_signal[0] = pi.rssi[0];
			st->chain_signal[1] = pi.rssi[1];
			if (freq) {
				/* legacy rate_idx indexes the band's rate table */
				if (band != st->band && st->encoding == RX_ENC_LEGACY)
					st->rate_idx = band == NL80211_BAND_5GHZ ?
						(st->rate_idx >= 4 ? st->rate_idx - 4 : 0) :
						st->rate_idx + 4;
				st->freq = freq;
				st->band = band;
			}
		}
		deliver(rd, skb);
	}
}

/*
 * AX hardware-decryption quirk: inside an RX BA session the engine can hand
 * up an MPDU whose PN is newer than a later-sequenced one. Delivering it
 * would make mac80211's replay check drop the rest, so drop it instead.
 */
static bool rx_pn_valid(struct ax52_dev *rd, struct sk_buff *skb, u32 dw3,
			u8 sec_type)
{
	struct ieee80211_hdr *hdr = (void *)skb->data;
	struct ax52_tid_rx *t;
	const u8 *iv;
	int hdrlen;
	s64 pn;
	u16 sn;

	if (!ieee80211_is_data_qos(hdr->frame_control) ||
	    (dw3 & (RXD3_HW_DEC | RXD3_A1_MATCH)) != (RXD3_HW_DEC | RXD3_A1_MATCH) ||
	    sec_type < AX52_SEC_CCMP128 || sec_type > AX52_SEC_GCMP256)
		return true;

	t = &rd->tid_rx[ieee80211_get_tid(hdr)];
	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	if (!READ_ONCE(t->started) || skb->len < hdrlen + 8)
		return true;

	iv = skb->data + hdrlen;	/* CCMP/GCMP header: PN0 PN1 rsv kid PN2..PN5 */
	pn = (s64)iv[0] | (s64)iv[1] << 8 | (s64)iv[4] << 16 |
	     (s64)iv[5] << 24 | (s64)iv[6] << 32 | (s64)iv[7] << 40;
	sn = ieee80211_get_sn(hdr);
	if (pn > t->last_pn) {
		if (t->last_pn != -1LL && ieee80211_sn_less(sn, t->last_sn))
			return false;
		t->last_sn = sn;
		t->last_pn = pn;
	}
	return true;
}

/* ------------------------------------------------------ dispatch */

/* @skb starts at the RX descriptor. Runs in NAPI context. */
void ax52_rx_packet(struct ax52_dev *rd, struct sk_buff *skb)
{
	const u8 *desc = skb->data;
	u32 dw0 = le32_to_cpu(*(const __le32 *)desc);
	u32 dw1 = le32_to_cpu(*(const __le32 *)(desc + 4));
	u32 dw3 = le32_to_cpu(*(const __le32 *)(desc + 12));
	bool is_long = dw0 & RXD0_LONG;
	struct ieee80211_rx_status *st;
	u32 pkt_len, off;
	u8 type, cnt;

	off = ax52_rx_payload_offset(desc, &pkt_len, &type);
	if (off + pkt_len > skb->len) {
		dev_kfree_skb_any(skb);
		return;
	}

	switch (type) {
	case RPKT_C2H:
		ax52_c2h_handle(rd, skb->data + off, pkt_len);
		dev_kfree_skb_any(skb);
		return;
	case RPKT_PPDU_STAT:
		rx_ppdu_status(rd, desc, skb->data + off, pkt_len);
		dev_kfree_skb_any(skb);
		return;
	case RPKT_WIFI:
		break;
	default:
		dev_kfree_skb_any(skb);
		return;
	}

	if (!rd->chandef.chan || pkt_len < 10) {
		dev_kfree_skb_any(skb);
		return;
	}

	/* frames of a new PPDU: the previous one's status is not coming */
	cnt = FIELD_GET(RXD1_PPDU_CNT, dw1);
	if (cnt != rd->ppdu_cnt) {
		flush_ppdu(rd);
		rd->ppdu_cnt = cnt;
	}

	skb_pull(skb, off);
	skb_trim(skb, pkt_len);

	if (is_long &&
	    !rx_pn_valid(rd, skb, dw3,
			 FIELD_GET(RXD7_SEC_TYPE,
				   le32_to_cpu(*(const __le32 *)(desc + 28))))) {
		dev_kfree_skb_any(skb);
		return;
	}

	st = IEEE80211_SKB_RXCB(skb);
	memset(st, 0, sizeof(*st));
	st->freq = rd->chandef.chan->center_freq;
	st->band = rd->chandef.chan->band;
	fill_rate(st, FIELD_GET(RXD1_DATA_RATE, dw1), FIELD_GET(RXD1_GI_LTF, dw1),
		  FIELD_GET(RXD1_BW, dw1), st->band == NL80211_BAND_5GHZ);
	if (dw3 & (RXD3_CRC32_ERR | RXD3_ICV_ERR))
		st->flag |= RX_FLAG_FAILED_FCS_CRC;
	if ((dw3 & RXD3_HW_DEC) && !(dw3 & (RXD3_SW_DEC | RXD3_ICV_ERR)))
		st->flag |= RX_FLAG_DECRYPTED;
	if (dw3 & RXD3_AMPDU) {
		st->flag |= RX_FLAG_AMPDU_DETAILS;
		st->ampdu_reference = cnt;
	}

	/* management and data frames wait for their PPDU status (signal) */
	if (is_long && skb_queue_len(&rd->ppdu_q) < PPDU_Q_MAX) {
		u8 ftype = FIELD_GET(RXD4_TYPE,
				     le32_to_cpu(*(const __le32 *)(desc + 16)));

		if (ftype == 0 || ftype == 2) {
			__skb_queue_tail(&rd->ppdu_q, skb);
			return;
		}
	}
	deliver(rd, skb);
}
