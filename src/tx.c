// SPDX-License-Identifier: GPL-2.0
/*
 * Frame transmission: WiFi-descriptor (WD) construction, queue selection,
 * the mac80211 TXQ scheduler and TX status reporting.
 */
#include "ax52.h"

/* WD body (dwords 0-5) */
#define WD0_HW_SSN_MODE		GENMASK(1, 0)
#define WD0_HW_SSN_SEL		GENMASK(3, 2)
#define WD0_WD_PAGE		BIT(7)
#define WD0_HDR_LLC_LEN		GENMASK(15, 11)
#define WD0_CH_DMA		GENMASK(19, 16)
#define WD0_WD_INFO_EN		BIT(22)
#define WD2_TXPKT_SIZE		GENMASK(13, 0)
#define WD2_QSEL		GENMASK(22, 17)
#define WD2_TID_IND		BIT(23)
#define WD2_MACID		GENMASK(30, 24)
#define WD3_SW_SEQ		GENMASK(11, 0)
#define WD3_AGG_EN		BIT(12)
#define WD3_BK			BIT(13)
/* WD info (dwords 6-11) */
#define WI0_DISDATAFB		BIT(10)
#define WI0_DATA_LDPC		BIT(11)
#define WI0_DATA_STBC		BIT(12)
#define WI0_DATA_RATE		GENMASK(24, 16)
#define WI0_USE_RATE		BIT(30)
#define WI1_MAX_AGGNUM		GENMASK(7, 0)
#define WI1_RTY_LOWEST_RATE	GENMASK(24, 16)
#define WI2_SEC_CAM_IDX		GENMASK(7, 0)
#define WI2_SEC_HW_ENC		BIT(8)
#define WI2_SEC_TYPE		GENMASK(12, 9)
#define WI2_AMPDU_DENSITY	GENMASK(20, 18)
#define WI4_RTS_EN		BIT(27)
#define WI4_HW_RTS_EN		BIT(31)

#define QSEL_B0_MGMT		0x12
#define RATE_CCK1		0x00
#define RATE_OFDM6		0x04

/* TID -> (queue select, ring). The second TID of each AC pair is flagged. */
static const u8 tid_qsel[8] = { 0, 1, 1, 0, 2, 2, 3, 3 };

static enum ax52_txq tid_to_txq(u8 tid)
{
	return TXQ_ACH0 + tid_qsel[tid & 7];
}

static bool tid_indicate(u8 tid)
{
	return tid == 2 || tid == 3 || tid == 5 || tid == 7;
}

/* Hardware rate code of the lowest bit set in a rate bitmap of @band. */
static u8 lowest_rate(struct ax52_dev *rd, enum nl80211_band band, u32 bitmap,
		      u8 fallback)
{
	struct ieee80211_supported_band *sb = rd->hw->wiphy->bands[band];

	if (!bitmap || !sb || __ffs(bitmap) >= sb->n_bitrates)
		return fallback;
	return sb->bitrates[__ffs(bitmap)].hw_value;
}

static bool is_mgmt_class(struct ieee80211_hdr *hdr)
{
	return ieee80211_is_mgmt(hdr->frame_control) ||
	       ieee80211_is_nullfunc(hdr->frame_control) ||
	       ieee80211_is_qos_nullfunc(hdr->frame_control);
}

/*
 * Build the 48-byte WD (body + info) for one frame and pick its ring.
 * Returns the ring index.
 */
static enum ax52_txq build_wd(struct ax52_dev *rd, struct sk_buff *skb,
			      struct ieee80211_sta *sta, __le32 *wd)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr = (void *)skb->data;
	enum nl80211_band band = ax52_hw_band(rd);
	bool is_5g = band == NL80211_BAND_5GHZ;
	u8 base = (is_5g || (info->flags & IEEE80211_TX_CTL_NO_CCK_RATE)) ?
		  RATE_OFDM6 : RATE_CCK1;
	u16 seq = le16_to_cpu(hdr->seq_ctrl) >> 4;
	u8 mac_id = rd->rvif.mac_id;
	enum ax52_txq q;
	u32 w0, w2, w3 = FIELD_PREP(WD3_SW_SEQ, seq);
	u32 i0 = 0, i1 = 0, i2 = 0;

	memset(wd, 0, 48);

	if (is_mgmt_class(hdr)) {
		struct ieee80211_vif *vif = info->control.vif;
		u8 rate = base;

		/*
		 * Lowest basic rate of the BSS once associated. The bitmap
		 * indexes the BSS band's rate table, so it does not apply to
		 * frames sent on a scan channel of the other band.
		 */
		if (vif && vif->cfg.assoc && vif->bss_conf.basic_rates &&
		    ax52_bss_band(rd, vif) == band) {
			u32 basic = vif->bss_conf.basic_rates;

			/* 2.4 GHz table entries 0-3 are the CCK rates */
			if (!is_5g && (info->flags & IEEE80211_TX_CTL_NO_CCK_RATE))
				basic &= ~0xF;
			rate = lowest_rate(rd, band, basic, base);
		}

		q = TXQ_MGMT;
		w0 = FIELD_PREP(WD0_HW_SSN_SEL, 1) | FIELD_PREP(WD0_HW_SSN_MODE, 1);
		w2 = FIELD_PREP(WD2_QSEL, QSEL_B0_MGMT);
		i0 = WI0_USE_RATE | WI0_DISDATAFB | FIELD_PREP(WI0_DATA_RATE, rate);
	} else {
		u8 tid = skb->priority & 7;
		bool eapol = info->control.flags & IEEE80211_TX_CTRL_PORT_CTRL_PROTO;

		q = tid_to_txq(tid);
		w0 = FIELD_PREP(WD0_HDR_LLC_LEN, ieee80211_hdrlen(hdr->frame_control) / 2);
		w2 = FIELD_PREP(WD2_QSEL, tid_qsel[tid]) |
		     (tid_indicate(tid) ? WD2_TID_IND : 0);

		if (sta) {
			struct ieee80211_link_sta *ls = &sta->deflink;
			enum nl80211_band bss = ax52_bss_band(rd, info->control.vif);

			i1 |= FIELD_PREP(WI1_RTY_LOWEST_RATE,
					 lowest_rate(rd, bss, ls->supp_rates[bss], base));
			if ((ls->he_cap.has_he &&
			     (ls->he_cap.he_cap_elem.phy_cap_info[1] &
			      IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD)) ||
			    (ls->vht_cap.vht_supported &&
			     (ls->vht_cap.cap & IEEE80211_VHT_CAP_RXLDPC)) ||
			    (ls->ht_cap.cap & IEEE80211_HT_CAP_LDPC_CODING))
				i0 |= WI0_DATA_LDPC;
			if ((ls->vht_cap.vht_supported &&
			     (ls->vht_cap.cap & IEEE80211_VHT_CAP_RXSTBC_MASK)) ||
			    (ls->ht_cap.cap & IEEE80211_HT_CAP_RX_STBC))
				i0 |= WI0_DATA_STBC;

			if ((info->flags & IEEE80211_TX_CTL_AMPDU) && !eapol) {
				u8 agg = rd->agg_num[tid] ?:
					 (4 << ls->ht_cap.ampdu_factor);

				w3 |= WD3_AGG_EN;
				i1 |= FIELD_PREP(WI1_MAX_AGGNUM, agg - 1);
				i2 |= FIELD_PREP(WI2_AMPDU_DENSITY,
						 ls->ht_cap.ampdu_density);
			}
		} else {
			i1 |= FIELD_PREP(WI1_RTY_LOWEST_RATE, base);
		}
		if (eapol)
			w3 |= WD3_BK;

		/* hardware key: mac80211 wrote the IV, the engine adds the MIC */
		if (info->control.hw_key) {
			u8 idx = info->control.hw_key->hw_key_idx;

			i2 |= WI2_SEC_HW_ENC | FIELD_PREP(WI2_SEC_CAM_IDX, idx) |
			      FIELD_PREP(WI2_SEC_TYPE, rd->sec_cam_type[idx]);
		}
	}

	w0 |= WD0_WD_INFO_EN | WD0_WD_PAGE |
	      FIELD_PREP(WD0_CH_DMA, rd->tx[q].hwch);
	w2 |= FIELD_PREP(WD2_MACID, mac_id) |
	      FIELD_PREP(WD2_TXPKT_SIZE, skb->len);

	wd[0] = cpu_to_le32(w0);
	wd[2] = cpu_to_le32(w2);
	wd[3] = cpu_to_le32(w3);
	wd[6] = cpu_to_le32(i0);
	wd[7] = cpu_to_le32(i1);
	wd[8] = cpu_to_le32(i2);
	wd[10] = cpu_to_le32(WI4_HW_RTS_EN |
			     (is_multicast_ether_addr(hdr->addr1) ? 0 : WI4_RTS_EN));
	return q;
}

/* Queue one frame on its ring. Returns the ring used, or <0 (skb freed). */
static int tx_one(struct ax52_dev *rd, struct ieee80211_sta *sta,
		  struct sk_buff *skb)
{
	__le32 wd[12];
	enum ax52_txq q;
	int ret;

	if (skb_linearize(skb)) {
		ieee80211_free_txskb(rd->hw, skb);
		return -ENOMEM;
	}
	q = build_wd(rd, skb, sta, wd);
	ret = ax52_pci_tx(rd, q, wd, skb);
	if (ret) {
		ieee80211_free_txskb(rd->hw, skb);
		return ret;
	}
	return q;
}

void ax52_op_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
		struct sk_buff *skb)
{
	struct ax52_dev *rd = hw->priv;
	int q;

	if (!READ_ONCE(rd->running)) {
		ieee80211_free_txskb(hw, skb);
		return;
	}
	q = tx_one(rd, control->sta, skb);
	if (q >= 0)
		ax52_pci_tx_kick(rd, q);
}

/* ------------------------------------------------------ TXQ scheduling */

/* mac80211 access category -> our ring */
static const enum ax52_txq ac_txq[IEEE80211_NUM_ACS] = {
	[IEEE80211_AC_VO] = TXQ_ACH3,
	[IEEE80211_AC_VI] = TXQ_ACH2,
	[IEEE80211_AC_BE] = TXQ_ACH0,
	[IEEE80211_AC_BK] = TXQ_ACH1,
};

/*
 * TX BA sessions are started by the driver, once per TID. After the session
 * ends (DELBA, inactivity) or could not be started (e.g. MFP before the
 * handshake), the TID may try again after BA_RETRY_DELAY; mac80211 spaces
 * out repeated ADDBA failures itself.
 */
#define BA_RETRY_DELAY		(2 * HZ)

void ax52_tx_ba_retry_later(struct ax52_dev *rd, u8 tid)
{
	WRITE_ONCE(rd->ba_retry_at[tid], jiffies + BA_RETRY_DELAY);
	smp_mb__before_atomic();	/* the holdoff is visible first */
	clear_bit(tid, &rd->ba_tried);
}

static void maybe_start_ba(struct ax52_dev *rd, struct ieee80211_txq *txq)
{
	if (!txq->sta || txq->tid >= IEEE80211_NUM_TIDS ||
	    !txq->sta->deflink.ht_cap.ht_supported ||
	    test_bit(txq->tid, &rd->ba_tried) ||
	    time_before(jiffies, READ_ONCE(rd->ba_retry_at[txq->tid])) ||
	    test_and_set_bit(txq->tid, &rd->ba_tried))
		return;
	set_bit(txq->tid, &rd->ba_pending);
	queue_work(rd->txq_wq, &rd->ba_work);
}

/* Forget all TX BA state (new interface or link). */
void ax52_tx_ba_reset(struct ax52_dev *rd)
{
	rd->ba_tried = 0;
	rd->ba_pending = 0;
	memset(rd->ba_retry_at, 0, sizeof(rd->ba_retry_at));
	memset(rd->agg_num, 0, sizeof(rd->agg_num));
}

static void txq_work_fn(struct work_struct *w)
{
	struct ax52_dev *rd = container_of(to_delayed_work(w), struct ax52_dev,
					    txq_work);
	struct ieee80211_hw *hw = rd->hw;
	unsigned long kick = 0;
	bool more = false;
	int ac, q;

	if (!READ_ONCE(rd->running))
		return;

	/* dequeued frames carry RCU-protected pointers until handed over */
	rcu_read_lock();
	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
		struct ieee80211_txq *txq;

		ieee80211_txq_schedule_start(hw, ac);
		while ((txq = ieee80211_next_txq(hw, ac))) {
			int avail = ax52_pci_tx_avail(rd, ac_txq[ac]);
			struct sk_buff *skb;
			int n = 0;

			while (n < avail && (skb = ieee80211_tx_dequeue_ni(hw, txq))) {
				q = tx_one(rd, txq->sta, skb);
				if (q >= 0)
					kick |= BIT(q);
				n++;
			}
			if (n >= avail)
				more = true;
			if (n && txq->sta)
				maybe_start_ba(rd, txq);
			ieee80211_return_txq(hw, txq, false);
		}
		ieee80211_txq_schedule_end(hw, ac);
	}
	rcu_read_unlock();

	for_each_set_bit(q, &kick, TXQ_NUM)
		ax52_pci_tx_kick(rd, q);

	WRITE_ONCE(rd->tx_starved, more);
	if (more)
		queue_delayed_work(rd->txq_wq, &rd->txq_work, 1);
}

static void ba_work_fn(struct work_struct *w)
{
	struct ax52_dev *rd = container_of(w, struct ax52_dev, ba_work);
	struct ieee80211_vif *vif = READ_ONCE(rd->vif);
	struct ieee80211_sta *sta = NULL;
	int tid, ret;

	rcu_read_lock();
	if (vif && vif->cfg.assoc)
		sta = ieee80211_find_sta(vif, vif->bss_conf.bssid);
	for (tid = 0; tid < IEEE80211_NUM_TIDS; tid++) {
		if (!test_and_clear_bit(tid, &rd->ba_pending))
			continue;
		ret = sta ? ieee80211_start_tx_ba_session(sta, tid, 0) : -ENOENT;
		/* -EAGAIN: a session already exists or is being set up */
		if (ret && ret != -EAGAIN)
			ax52_tx_ba_retry_later(rd, tid);
	}
	rcu_read_unlock();
}

void ax52_op_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	struct ax52_dev *rd = hw->priv;

	queue_delayed_work(rd->txq_wq, &rd->txq_work, 0);
}

/* Release reports freed ring space: restart a starved scheduler. */
void ax52_tx_resources_freed(struct ax52_dev *rd)
{
	if (READ_ONCE(rd->tx_starved))
		mod_delayed_work(rd->txq_wq, &rd->txq_work, 0);
}

/* Report a completed frame. @status: 0 acked, 1 retry limit, 2 lifetime, 3 dropped. */
void ax52_tx_status(struct ax52_dev *rd, struct sk_buff *skb, u8 status)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	ieee80211_tx_info_clear_status(info);
	if (info->flags & IEEE80211_TX_CTL_NO_ACK)
		info->flags |= IEEE80211_TX_STAT_NOACK_TRANSMITTED;
	else if (status == 0)
		info->flags |= IEEE80211_TX_STAT_ACK;
	ieee80211_tx_status_ni(rd->hw, skb);
}

void ax52_tx_init(struct ax52_dev *rd)
{
	INIT_DELAYED_WORK(&rd->txq_work, txq_work_fn);
	INIT_WORK(&rd->ba_work, ba_work_fn);
}
