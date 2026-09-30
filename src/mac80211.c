// SPDX-License-Identifier: GPL-2.0
/*
 * mac80211 glue: capability advertisement and the ieee80211_ops of a
 * single-interface station driver. Every op except tx/wake_tx_queue runs
 * with the wiphy mutex held.
 */
#include <linux/module.h>

#include "ax52.h"

#define CHAN2G(n, f) { .band = NL80211_BAND_2GHZ, .center_freq = (f), \
		      .hw_value = (n), .max_power = 20 }
#define CHAN5G(n)    { .band = NL80211_BAND_5GHZ, .center_freq = 5000 + 5 * (n), \
		      .hw_value = (n), .max_power = 20 }

static const struct ieee80211_channel chan_2g[] = {
	CHAN2G(1, 2412), CHAN2G(2, 2417), CHAN2G(3, 2422), CHAN2G(4, 2427),
	CHAN2G(5, 2432), CHAN2G(6, 2437), CHAN2G(7, 2442), CHAN2G(8, 2447),
	CHAN2G(9, 2452), CHAN2G(10, 2457), CHAN2G(11, 2462), CHAN2G(12, 2467),
	CHAN2G(13, 2472), CHAN2G(14, 2484),
};

static const struct ieee80211_channel chan_5g[] = {
	CHAN5G(36), CHAN5G(40), CHAN5G(44), CHAN5G(48), CHAN5G(52), CHAN5G(56),
	CHAN5G(60), CHAN5G(64), CHAN5G(100), CHAN5G(104), CHAN5G(108),
	CHAN5G(112), CHAN5G(116), CHAN5G(120), CHAN5G(124), CHAN5G(128),
	CHAN5G(132), CHAN5G(136), CHAN5G(140), CHAN5G(144), CHAN5G(149),
	CHAN5G(153), CHAN5G(157), CHAN5G(161),
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165,
	  .max_power = 20, .flags = IEEE80211_CHAN_NO_HT40MINUS },
	CHAN5G(169), CHAN5G(173), CHAN5G(177),
};

/* hw_value is the hardware rate code */
static const struct ieee80211_rate rates[] = {
	{ .bitrate = 10,  .hw_value = 0x00 },
	{ .bitrate = 20,  .hw_value = 0x01 },
	{ .bitrate = 55,  .hw_value = 0x02 },
	{ .bitrate = 110, .hw_value = 0x03 },
	{ .bitrate = 60,  .hw_value = 0x04 },
	{ .bitrate = 90,  .hw_value = 0x05 },
	{ .bitrate = 120, .hw_value = 0x06 },
	{ .bitrate = 180, .hw_value = 0x07 },
	{ .bitrate = 240, .hw_value = 0x08 },
	{ .bitrate = 360, .hw_value = 0x09 },
	{ .bitrate = 480, .hw_value = 0x0A },
	{ .bitrate = 540, .hw_value = 0x0B },
};

static void init_ht_cap(struct ieee80211_sta_ht_cap *ht)
{
	ht->ht_supported = true;
	ht->cap = IEEE80211_HT_CAP_LDPC_CODING | IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		  IEEE80211_HT_CAP_SGI_20 | IEEE80211_HT_CAP_SGI_40 |
		  IEEE80211_HT_CAP_TX_STBC | (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT) |
		  IEEE80211_HT_CAP_MAX_AMSDU | IEEE80211_HT_CAP_DSSSCCK40;
	ht->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht->ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
	ht->mcs.rx_mask[0] = 0xff;
	ht->mcs.rx_mask[1] = 0xff;
	ht->mcs.rx_mask[4] = 0x01;
	ht->mcs.rx_highest = cpu_to_le16(300);
	ht->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
}

static void init_vht_cap(struct ieee80211_sta_vht_cap *vht)
{
	/* 2 streams of MCS 0-9; no beamformee until sounding is set up */
	vht->vht_supported = true;
	vht->cap = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 |
		   IEEE80211_VHT_CAP_RXLDPC | IEEE80211_VHT_CAP_SHORT_GI_80 |
		   IEEE80211_VHT_CAP_TXSTBC | IEEE80211_VHT_CAP_RXSTBC_1 |
		   IEEE80211_VHT_CAP_HTC_VHT |
		   (7 << IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT);
	vht->vht_mcs.rx_mcs_map = cpu_to_le16(0xfffa);
	vht->vht_mcs.tx_mcs_map = cpu_to_le16(0xfffa);
	vht->vht_mcs.rx_highest = cpu_to_le16(867);
	vht->vht_mcs.tx_highest = cpu_to_le16(867);
}

static void init_he_cap(struct ieee80211_sband_iftype_data *d, bool is_5g)
{
	static const u8 mac_cap[6] = { 0x01, 0x08, 0x0A, 0x10, 0x60, 0x80 };
	/* rtw89's values with the beamformee/feedback bits (PHY 4-7) cleared */
	static const u8 phy_cap[11] = {
		0x02, 0x70, 0x1E, 0x1F, 0x00, 0x00, 0x20, 0x06, 0x91, 0xBD, 0x00,
	};
	struct ieee80211_sta_he_cap *he = &d->he_cap;

	d->types_mask = BIT(NL80211_IFTYPE_STATION);
	he->has_he = true;
	memcpy(he->he_cap_elem.mac_cap_info, mac_cap, sizeof(mac_cap));
	memcpy(he->he_cap_elem.phy_cap_info, phy_cap, sizeof(phy_cap));
	if (is_5g)
		he->he_cap_elem.phy_cap_info[0] =
			IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G;
	he->he_mcs_nss_supp.rx_mcs_80 = cpu_to_le16(0xfffa);
	he->he_mcs_nss_supp.tx_mcs_80 = cpu_to_le16(0xfffa);
	he->he_mcs_nss_supp.rx_mcs_160 = cpu_to_le16(0xffff);
	he->he_mcs_nss_supp.tx_mcs_160 = cpu_to_le16(0xffff);
	he->he_mcs_nss_supp.rx_mcs_80p80 = cpu_to_le16(0xffff);
	he->he_mcs_nss_supp.tx_mcs_80p80 = cpu_to_le16(0xffff);
}

static int init_bands(struct ax52_dev *rd)
{
	struct device *dev = rd->dev;
	struct ieee80211_supported_band *b2, *b5;
	struct ieee80211_sband_iftype_data *he2, *he5;

	b2 = devm_kzalloc(dev, sizeof(*b2), GFP_KERNEL);
	b5 = devm_kzalloc(dev, sizeof(*b5), GFP_KERNEL);
	he2 = devm_kzalloc(dev, sizeof(*he2), GFP_KERNEL);
	he5 = devm_kzalloc(dev, sizeof(*he5), GFP_KERNEL);
	if (!b2 || !b5 || !he2 || !he5)
		return -ENOMEM;

	b2->band = NL80211_BAND_2GHZ;
	b2->channels = devm_kmemdup(dev, chan_2g, sizeof(chan_2g), GFP_KERNEL);
	b2->n_channels = ARRAY_SIZE(chan_2g);
	b2->bitrates = devm_kmemdup(dev, rates, sizeof(rates), GFP_KERNEL);
	b2->n_bitrates = ARRAY_SIZE(rates);
	init_ht_cap(&b2->ht_cap);
	init_he_cap(he2, false);
	_ieee80211_set_sband_iftype_data(b2, he2, 1);

	b5->band = NL80211_BAND_5GHZ;
	b5->channels = devm_kmemdup(dev, chan_5g, sizeof(chan_5g), GFP_KERNEL);
	b5->n_channels = ARRAY_SIZE(chan_5g);
	b5->bitrates = devm_kmemdup(dev, rates + 4, sizeof(rates) - 4 * sizeof(rates[0]),
				    GFP_KERNEL);
	b5->n_bitrates = ARRAY_SIZE(rates) - 4;
	init_ht_cap(&b5->ht_cap);
	init_vht_cap(&b5->vht_cap);
	init_he_cap(he5, true);
	_ieee80211_set_sband_iftype_data(b5, he5, 1);

	if (!b2->channels || !b2->bitrates || !b5->channels || !b5->bitrates)
		return -ENOMEM;

	rd->hw->wiphy->bands[NL80211_BAND_2GHZ] = b2;
	rd->hw->wiphy->bands[NL80211_BAND_5GHZ] = b5;
	return 0;
}

/* ------------------------------------------------------------------ ops */

static int ax52_op_start(struct ieee80211_hw *hw)
{
	return ax52_chip_start(hw->priv);
}

static void ax52_op_stop(struct ieee80211_hw *hw, bool suspend)
{
	ax52_chip_stop(hw->priv);
}

static int ax52_op_add_interface(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif)
{
	struct ax52_dev *rd = hw->priv;
	struct ax52_vif *rv = &rd->rvif;
	int ret;

	if (rd->vif || vif->type != NL80211_IFTYPE_STATION || vif->p2p)
		return -EOPNOTSUPP;

	memset(rv, 0, sizeof(*rv));
	bitmap_zero(rd->sec_cam_map, AX52_SEC_CAM_NUM);
	memset(rd->tid_rx, 0, sizeof(rd->tid_rx));
	ether_addr_copy(rv->addr, vif->addr);
	rd->vif = vif;

	ax52_mac_port_update(rd, rv, vif);
	ax52_mac_macid_tbl_init(rd, rv->mac_id);
	ret = ax52_h2c_macid_pause(rd, rv->mac_id, false) ?:
	      ax52_h2c_role_maintain(rd, rv, false) ?:
	      ax52_h2c_join_info(rd, rv, true) ?:
	      ax52_h2c_addr_cam(rd, rv, vif, NULL) ?:
	      ax52_h2c_default_cmac_tbl(rd, rv->mac_id);
	if (ret) {
		ax52_err(rd, "interface setup failed: %d\n", ret);
		rd->vif = NULL;
	}
	return ret;
}

static void ax52_op_remove_interface(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	struct ax52_dev *rd = hw->priv;

	if (rd->vif != vif)
		return;
	rd->vif = NULL;
	cancel_work_sync(&rd->ba_work);		/* it dereferences the vif */
	ax52_h2c_role_maintain(rd, &rd->rvif, true);
	ax52_h2c_addr_cam(rd, &rd->rvif, NULL, NULL);	/* invalidate entries */
}

static int ax52_op_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct ax52_dev *rd = hw->priv;
	int ret;

	if (!(changed & IEEE80211_CONF_CHANGE_CHANNEL))
		return 0;

	ret = ax52_phy_set_channel(rd, &hw->conf.chandef);
	if (ret) {
		ax52_err(rd, "set channel %u MHz failed: %d\n",
			 hw->conf.chandef.chan->center_freq, ret);
		return ret;
	}
	rd->chandef = hw->conf.chandef;
	ax52_coex_wl_only(rd);
	return 0;
}

#define FLTR_A1_MATCH		BIT(1)
#define FLTR_A_BC		BIT(2)
#define FLTR_A_MC		BIT(3)
#define FLTR_A_UC_CAM_MATCH	BIT(4)
#define FLTR_A_BC_CAM_MATCH	BIT(5)
#define FLTR_A_BCN_CHK_EN	BIT(7)
#define FLTR_A_CRC32_ERR	BIT(11)

static void ax52_op_configure_filter(struct ieee80211_hw *hw,
				     unsigned int changed, unsigned int *total,
				      u64 multicast)
{
	struct ax52_dev *rd = hw->priv;
	u32 f = AX52_RX_FLTR_DEFAULT;

	*total &= FIF_ALLMULTI | FIF_OTHER_BSS | FIF_FCSFAIL |
		  FIF_BCN_PRBRESP_PROMISC | FIF_PROBE_REQ;

	if (*total & FIF_ALLMULTI)
		f &= ~FLTR_A_MC;
	if (*total & FIF_FCSFAIL)
		f |= FLTR_A_CRC32_ERR;
	if (*total & FIF_OTHER_BSS)
		f &= ~FLTR_A1_MATCH;
	if (*total & FIF_BCN_PRBRESP_PROMISC)
		f &= ~(FLTR_A_BCN_CHK_EN | FLTR_A_BC | FLTR_A1_MATCH);
	if (*total & FIF_PROBE_REQ)
		f &= ~(FLTR_A_BC_CAM_MATCH | FLTR_A_UC_CAM_MATCH);

	rd->rx_fltr = f;
	if (rd->running)
		ax52_mac_set_rx_filter(rd, f);
}

static struct ieee80211_sta *ap_sta(struct ieee80211_vif *vif)
{
	return ieee80211_find_sta(vif, vif->bss_conf.bssid);
}

static void send_edca(struct ax52_dev *rd)
{
	int ac;

	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++)
		if (rd->edca_valid & BIT(ac))
			ax52_h2c_edca(rd, &rd->rvif, ac, &rd->edca[ac]);
}

static void ax52_op_bss_info_changed(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				      struct ieee80211_bss_conf *info,
				      u64 changed)
{
	struct ax52_dev *rd = hw->priv;
	struct ax52_vif *rv = &rd->rvif;

	if (rd->vif != vif)
		return;

	if (changed & BSS_CHANGED_BSSID) {
		ether_addr_copy(rv->bssid, info->bssid ?: (const u8 *)"\0\0\0\0\0\0");
		ax52_h2c_addr_cam(rd, rv, vif, NULL);
	}

	if ((changed & BSS_CHANGED_ASSOC) && vif->cfg.assoc) {
		struct ieee80211_sta *sta;

		rv->assoc = true;
		rv->aid = vif->cfg.aid;
		rv->net_type = 2;
		rv->bss_color = info->he_bss_color.enabled ?
				info->he_bss_color.color : 0;

		/* the AP entry cannot go away while we hold the wiphy mutex */
		rcu_read_lock();
		sta = ap_sta(vif);
		rcu_read_unlock();
		if (sta) {
			ax52_h2c_assoc_cmac_tbl(rd, rv, vif, sta);
			ax52_h2c_join_info(rd, rv, false);
			ax52_h2c_addr_cam(rd, rv, vif, sta);
			ax52_h2c_ra(rd, rv, vif, sta, false);
		} else {
			ax52_warn(rd, "associated but no AP station entry\n");
		}
		ax52_mac_port_update(rd, rv, vif);
	}

	if (changed & BSS_CHANGED_ERP_SLOT)
		send_edca(rd);
}

static int ax52_op_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			   unsigned int link_id, u16 ac,
			    const struct ieee80211_tx_queue_params *params)
{
	struct ax52_dev *rd = hw->priv;

	if (ac >= IEEE80211_NUM_ACS)
		return -EINVAL;
	rd->edca[ac] = *params;
	rd->edca_valid |= BIT(ac);
	return ax52_h2c_edca(rd, &rd->rvif, ac, params);
}

static int ax52_op_sta_state(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			     struct ieee80211_sta *sta,
			      enum ieee80211_sta_state old,
			      enum ieee80211_sta_state new)
{
	struct ax52_dev *rd = hw->priv;
	struct ax52_vif *rv = &rd->rvif;

	if (rd->vif != vif)
		return 0;

	if (old == IEEE80211_STA_NOTEXIST && new == IEEE80211_STA_NONE) {
		/* calibrate for the channel we are about to associate on */
		ax52_rfk_sta_connect(rd);
		ax52_h2c_rf_ntfy_mcc(rd);
	} else if (old == IEEE80211_STA_AUTH && new == IEEE80211_STA_NONE) {
		rv->assoc = false;
		rv->aid = 0;
		rv->net_type = 0;
		rd->ba_tried = 0;
		rd->ba_pending = 0;
		memset(rd->agg_num, 0, sizeof(rd->agg_num));
		ax52_mac_set_agg_limit(rd, 0x3F);
		ax52_h2c_assoc_cmac_tbl(rd, rv, vif, sta);
		ax52_h2c_join_info(rd, rv, true);
		ax52_h2c_addr_cam(rd, rv, vif, sta);
	}
	return 0;
}

/* ------------------------------------------------------------ crypto */

static bool swcrypto;
module_param(swcrypto, bool, 0644);
MODULE_PARM_DESC(swcrypto, "Keep all keys in software (applies to keys set afterwards)");

static u8 cipher_to_sec_type(u32 cipher)
{
	switch (cipher) {
	case WLAN_CIPHER_SUITE_CCMP:		return AX52_SEC_CCMP128;
	case WLAN_CIPHER_SUITE_CCMP_256:	return AX52_SEC_CCMP256;
	case WLAN_CIPHER_SUITE_GCMP:		return AX52_SEC_GCMP128;
	case WLAN_CIPHER_SUITE_GCMP_256:	return AX52_SEC_GCMP256;
	default:				return AX52_SEC_NONE;
	}
}

/* Free security-CAM index (two consecutive ones for 256-bit keys). */
static int sec_cam_alloc(struct ax52_dev *rd, bool ext)
{
	int i;

	for (i = 0; i < AX52_SEC_CAM_NUM - ext; i++) {
		if (test_bit(i, rd->sec_cam_map) ||
		    (ext && test_bit(i + 1, rd->sec_cam_map)))
			continue;
		set_bit(i, rd->sec_cam_map);
		if (ext)
			set_bit(i + 1, rd->sec_cam_map);
		return i;
	}
	return -EBUSY;
}

static void sec_cam_free(struct ax52_dev *rd, u8 idx)
{
	bool ext = rd->sec_cam_type[idx] == AX52_SEC_CCMP256 ||
		   rd->sec_cam_type[idx] == AX52_SEC_GCMP256;

	clear_bit(idx, rd->sec_cam_map);
	if (ext)
		clear_bit(idx + 1, rd->sec_cam_map);
	rd->sec_cam_type[idx] = AX52_SEC_NONE;
}

/* Address-CAM key slot: 0-1 pairwise, 2-4 group (BIP slots 5-6 unused). */
static int addr_cam_slot(struct ax52_vif *rv, bool pairwise)
{
	int i, first = pairwise ? 0 : 2, last = pairwise ? 1 : 4;

	for (i = first; i <= last; i++)
		if (!(rv->sec_ent_map & BIT(i)))
			return i;
	return -EBUSY;
}

static void reset_tid_rx(struct ax52_dev *rd)
{
	int tid;

	for (tid = 0; tid < IEEE80211_NUM_TIDS; tid++) {
		rd->tid_rx[tid].last_pn = -1LL;
		rd->tid_rx[tid].last_sn = IEEE80211_SN_MASK;
	}
}

/*
 * CCMP/GCMP keys are handled by the security engine (mac80211 still builds
 * the IV and encrypts management frames). Everything else stays in software.
 */
static int ax52_op_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			    struct ieee80211_key_conf *key)
{
	struct ax52_dev *rd = hw->priv;
	struct ax52_vif *rv = &rd->rvif;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	u8 type = cipher_to_sec_type(key->cipher);
	int idx, slot, i, ret;

	if (rd->vif != vif)
		return -EOPNOTSUPP;

	if (cmd == DISABLE_KEY) {
		if (!test_bit(key->hw_key_idx, rd->sec_cam_map))
			return 0;
		/* nothing queued may still reference the key */
		flush_delayed_work(&rd->txq_work);
		ax52_pci_flush(rd);
		ax52_mac_wait_txq_empty(rd);
		for (i = 0; i < ARRAY_SIZE(rv->sec_ent); i++)
			if ((rv->sec_ent_map & BIT(i)) &&
			    rv->sec_ent[i] == key->hw_key_idx)
				rv->sec_ent_map &= ~BIT(i);
		ret = ax52_h2c_addr_cam(rd, rv, vif, sta);
		sec_cam_free(rd, key->hw_key_idx);
		return ret;
	}

	if (swcrypto || type == AX52_SEC_NONE || key->keylen > 32)
		return -EOPNOTSUPP;

	/* no free slot or entry: fall back to software for this key */
	slot = addr_cam_slot(rv, pairwise);
	if (slot < 0)
		return -EOPNOTSUPP;
	idx = sec_cam_alloc(rd, key->keylen > 16);
	if (idx < 0)
		return -EOPNOTSUPP;
	rd->sec_cam_type[idx] = type;

	ret = ax52_h2c_sec_cam(rd, idx, type, key->key, key->keylen);
	if (!ret) {
		rv->sec_ent[slot] = idx;
		rv->sec_ent_keyid[slot] = key->keyidx;
		rv->sec_ent_map |= BIT(slot);
		ret = ax52_h2c_addr_cam(rd, rv, vif, sta);
		if (ret)
			rv->sec_ent_map &= ~BIT(slot);
	}
	if (ret) {
		sec_cam_free(rd, idx);
		return ret;
	}

	key->hw_key_idx = idx;
	key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV | IEEE80211_KEY_FLAG_SW_MGMT_TX;
	ax52_info(rd, "hardware key: cipher %08x, %s, keyid %u -> sec CAM %d, slot %d\n",
		  key->cipher, pairwise ? "pairwise" : "group", key->keyidx,
		   idx, slot);
	if (pairwise)
		reset_tid_rx(rd);
	return 0;
}

static void ax52_op_link_sta_rc_update(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif,
					struct ieee80211_link_sta *link_sta,
					u32 changed)
{
	struct ax52_dev *rd = hw->priv;

	if (rd->vif == vif && rd->rvif.assoc)
		ax52_h2c_ra(rd, &rd->rvif, vif, link_sta->sta, true);
}

static void recalc_agg_limit(struct ax52_dev *rd)
{
	u8 lmt = 0xff;
	int tid;

	for (tid = 0; tid < IEEE80211_NUM_TIDS; tid++)
		if (rd->agg_num[tid])
			lmt = min_t(u8, lmt, rd->agg_num[tid] - 1);
	ax52_mac_set_agg_limit(rd, lmt == 0xff ? 0x3F : lmt);
}

static int ax52_op_ampdu_action(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				 struct ieee80211_ampdu_params *p)
{
	struct ax52_dev *rd = hw->priv;

	switch (p->action) {
	case IEEE80211_AMPDU_TX_START:
		return IEEE80211_AMPDU_TX_START_IMMEDIATE;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		rd->agg_num[p->tid] = clamp_t(u16, p->buf_size, 1, 255);
		recalc_agg_limit(rd);
		return 0;
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		rd->agg_num[p->tid] = 0;
		recalc_agg_limit(rd);
		ieee80211_stop_tx_ba_cb_irqsafe(vif, p->sta->addr, p->tid);
		return 0;
	case IEEE80211_AMPDU_RX_START:
		/* static responder entry; hardware falls back to dynamic ones */
		ax52_h2c_ba_cam(rd, &rd->rvif, p->tid, p->ssn, p->buf_size, true);
		rd->tid_rx[p->tid].last_pn = -1LL;
		rd->tid_rx[p->tid].last_sn = IEEE80211_SN_MASK;
		WRITE_ONCE(rd->tid_rx[p->tid].started, true);
		return 0;
	case IEEE80211_AMPDU_RX_STOP:
		WRITE_ONCE(rd->tid_rx[p->tid].started, false);
		ax52_h2c_ba_cam(rd, &rd->rvif, p->tid, 0, 0, false);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static void ax52_op_sw_scan_start(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif, const u8 *mac)
{
	struct ax52_dev *rd = hw->priv;

	rd->scanning = true;
	ax52_rfk_scan(rd, true);
}

static void ax52_op_sw_scan_complete(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	struct ax52_dev *rd = hw->priv;

	ax52_rfk_scan(rd, false);
	rd->scanning = false;
}

static void ax52_op_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  u32 queues, bool drop)
{
	struct ax52_dev *rd = hw->priv;

	if (!rd->running)
		return;
	flush_delayed_work(&rd->txq_work);
	ax52_pci_flush(rd);
	if (ax52_mac_wait_txq_empty(rd))
		ax52_dbg(rd, "TX queues not empty after flush\n");
}

static int ax52_op_set_rts_threshold(struct ieee80211_hw *hw, int radio_idx,
				     u32 value)
{
	struct ax52_dev *rd = hw->priv;

	if (rd->running)
		ax52_mac_set_rts_threshold(rd, value);
	return 0;
}

static void ax52_op_sta_statistics(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				    struct ieee80211_sta *sta,
				    struct station_info *sinfo)
{
	struct ax52_dev *rd = hw->priv;

	if (ax52_h2c_get_txrate(rd, &sinfo->txrate))
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
}

static const struct ieee80211_ops ax52_ops = {
	.tx			= ax52_op_tx,
	.wake_tx_queue		= ax52_op_wake_tx_queue,
	.start			= ax52_op_start,
	.stop			= ax52_op_stop,
	.add_interface		= ax52_op_add_interface,
	.remove_interface	= ax52_op_remove_interface,
	.config			= ax52_op_config,
	.configure_filter	= ax52_op_configure_filter,
	.bss_info_changed	= ax52_op_bss_info_changed,
	.conf_tx		= ax52_op_conf_tx,
	.sta_state		= ax52_op_sta_state,
	.link_sta_rc_update	= ax52_op_link_sta_rc_update,
	.ampdu_action		= ax52_op_ampdu_action,
	.set_key		= ax52_op_set_key,
	.sw_scan_start		= ax52_op_sw_scan_start,
	.sw_scan_complete	= ax52_op_sw_scan_complete,
	.flush			= ax52_op_flush,
	.set_rts_threshold	= ax52_op_set_rts_threshold,
	.sta_statistics		= ax52_op_sta_statistics,
	.add_chanctx		= ieee80211_emulate_add_chanctx,
	.remove_chanctx		= ieee80211_emulate_remove_chanctx,
	.change_chanctx		= ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx	= ieee80211_emulate_switch_vif_chanctx,
};

/* ------------------------------------------------------------ regulatory */

static void ax52_reg_notifier(struct wiphy *wiphy,
			      struct regulatory_request *req)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct ax52_dev *rd = hw->priv;

	/* TX power limits follow the country; re-apply under the wiphy lock */
	memcpy(rd->regd_alpha2, req->alpha2, 2);
	rd->regd_alpha2[2] = 0;
	wiphy_work_queue(wiphy, &rd->regd_work);
}

static void regd_work_fn(struct wiphy *wiphy, struct wiphy_work *w)
{
	struct ax52_dev *rd = container_of(w, struct ax52_dev, regd_work);

	if (rd->running)
		ax52_phy_set_txpwr(rd);
}

/* ---------------------------------------------------------- periodic work */

static void track_work_fn(struct wiphy *wiphy, struct wiphy_work *w)
{
	struct ax52_dev *rd = container_of(w, struct ax52_dev, track_work.work);

	if (!rd->running)
		return;
	ax52_phy_track(rd);
	if (!rd->scanning)
		ax52_rfk_track(rd);

	/* keep FW rate adaptation fed with the current RSSI */
	if (rd->vif && rd->rvif.assoc) {
		struct ieee80211_sta *sta;

		rcu_read_lock();
		sta = ap_sta(rd->vif);
		rcu_read_unlock();
		if (sta)
			ax52_h2c_ra(rd, &rd->rvif, rd->vif, sta, true);
	}
	wiphy_delayed_work_queue(wiphy, &rd->track_work, 2 * HZ);
}

void ax52_track_start(struct ax52_dev *rd)
{
	wiphy_delayed_work_queue(rd->hw->wiphy, &rd->track_work, 2 * HZ);
}

void ax52_track_stop(struct ax52_dev *rd)
{
	wiphy_delayed_work_cancel(rd->hw->wiphy, &rd->track_work);
}

/* ------------------------------------------------------------ allocation */

struct ax52_dev *ax52_alloc_hw(struct device *dev)
{
	struct ieee80211_hw *hw;
	struct ax52_dev *rd;

	hw = ieee80211_alloc_hw(sizeof(*rd), &ax52_ops);
	if (!hw)
		return NULL;
	rd = hw->priv;
	rd->hw = hw;
	SET_IEEE80211_DEV(hw, dev);
	wiphy_delayed_work_init(&rd->track_work, track_work_fn);
	wiphy_work_init(&rd->regd_work, regd_work_fn);
	strscpy(rd->regd_alpha2, "00", sizeof(rd->regd_alpha2));
	rd->rx_fltr = AX52_RX_FLTR_DEFAULT;
	return rd;
}

void ax52_free_hw(struct ax52_dev *rd)
{
	ieee80211_free_hw(rd->hw);
}

int ax52_register_hw(struct ax52_dev *rd)
{
	struct ieee80211_hw *hw = rd->hw;
	int ret;

	SET_IEEE80211_PERM_ADDR(hw, rd->efuse.addr);
	hw->queues = IEEE80211_NUM_ACS;
	hw->max_rx_aggregation_subframes = 64;
	hw->max_tx_aggregation_subframes = 128;

	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, RX_INCLUDES_FCS);
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	ieee80211_hw_set(hw, SUPPORT_FAST_XMIT);

	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	hw->wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;
	hw->wiphy->reg_notifier = ax52_reg_notifier;

	ret = init_bands(rd);
	if (ret)
		return ret;
	return ieee80211_register_hw(hw);
}

void ax52_unregister_hw(struct ax52_dev *rd)
{
	ieee80211_unregister_hw(rd->hw);
	wiphy_work_cancel(rd->hw->wiphy, &rd->regd_work);
}
