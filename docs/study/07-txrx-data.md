# 07 — 802.11 frame data path (TX/RX descriptors, PHY status, aggregation, security, TX status) — RTL8852BE

Scope: everything that goes *into* a TX DMA buffer and comes *out of* an RX DMA buffer above the
PCIe ring/BD layer (Track 01 owns TXBD/RXBD rings, ring registers, interrupts, NAPI).
All paths resolved for RTL8852B, PCIe, CV_B. Source root below is
`reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/` (all `file:line` citations are relative to it).

Conventions: `BIT(n)`, `[hi:lo]` = inclusive bit range of a little-endian 32-bit dword.
"dwN" = dword N (byte offset 4·N) of the structure.

---------------------------------------------------------------------------------------------------

## 0. Resolved indirections for 8852B

| Generic hook | 8852B resolves to | Source |
|---|---|---|
| `chip->ops->fill_txdesc` | `rtw89_core_fill_txdesc` (**"v0" layout**: 24-B body + 24-B info) | rtw8852b.c:913, core.c:1600-1622 |
| `chip->ops->fill_txdesc_fwcmd` | `rtw89_core_fill_txdesc` (same v0 builder) | rtw8852b.c:914 |
| `chip->ops->get_ch_dma[PCIE]` | `rtw89_core_get_ch_dma` (1:1 AC mapping) | rtw8852b.c:915-917, core.c:782-817 |
| `chip->ops->query_rxdesc` | `rtw89_core_query_rxdesc` (AX RX desc: 16-B short / 32-B long) | rtw8852b.c:912, core.c:4070-4124 |
| `chip->ops->query_ppdu` | `__rtw8852bx_query_ppdu` | rtw8852b.c:902, rtw8852b_common.c:1970-1986 |
| `chip->ops->convert_rpl_to_rssi` | `__rtw8852bx_convert_rpl_to_rssi` | rtw8852b.c:903, rtw8852b_common.c:1988-1999 |
| `chip->ops->phy_rpt_to_rssi` | NULL (no per-MPDU PHY report on AX) | rtw8852b.c:904 |
| `chip->ops->h2c_ba_cam` | `rtw89_fw_h2c_ba_cam` (BACAM_V0) | rtw8852b.c:930, fw.c:2545-2620 |
| `chip->ops->h2c_ampdu_cmac_tbl` | NULL (no H2C on TX BA start/stop) | rtw8852b.c:925 |
| `chip->ops->h2c_dctl_sec_cam` | NULL | rtw8852b.c:922 |
| `chip->txwd_body_size` / `txwd_info_size` / `h2c_desc_size` | 24 / 24 / 24 bytes | rtw8852b.c:1080-1082, core.h:1092-1130 |
| `chip->hw_sec_hdr` / `hw_mgmt_tx_encrypt` / `hw_tkip_crypto` | false / false / false | rtw8852b.c:1031-1033 |
| `chip->max_tx_agg_num` / `max_rx_agg_num` / `max_amsdu_limit` | 128 / 64 / 5000 | rtw8852b.c:975-979 |
| `chip->bacam_num` / `bacam_dynamic_num` / `bacam_ver` | 2 / 4 / RTW89_BACAM_V0 | rtw8852b.c:1041-1043 |
| `chip->scam_num` (sec CAM entries) / `acam_num` / `bcam_num` | 128 / 128 / 10 | rtw8852b.c:1038-1040 |
| `chip->ppdu_max_usr` | 4 | rtw8852b.c:1045 |
| `chip->rx_freq_from_ie` / `cfo_src_fd` | true / true | rtw8852b.c:1030, 1091 |
| `pci_info->fill_txaddr_info` | `rtw89_pci_fill_txaddr_info` (8-byte addr_info, 1 entry) | rtw8852be.c:62, pci.c:1438-1455 |
| `pci_info->parse_rpp` / `rpp_fmt_size` | `rtw89_pci_parse_rpp` / 4 bytes | rtw8852be.c:33,63, pci.c:572-582 |
| `pci_info->tx_dma_ch_mask` (unavailable channels) | ACH4..ACH7, CH10, CH11 | rtw8852be.c:54-56 |
| `hci.tx_rpt_enabled` | **false** on PCIe (USB-only feature) | usb.c:1228 |
| `phy_def` | `rtw89_phy_gen_ax` (`physt_gen`=0, BB CR base 0x10000) | phy.c:8969-8992 |

WD checksum: **not used** on 8852B. The only checksum field in the tree is the BE (Wi-Fi 7)
`BE_TXD_BODY4_TXDESC_CHECKSUM` / `BE_TXD_BODY0_CHK_EN` (txrx.h:159, 211); nothing writes it.

---------------------------------------------------------------------------------------------------

## 1. TX descriptor ("WD" = WiFi Descriptor)

### 1.1 Per-frame TX flow (data / mgmt), PCIe

1. mac80211 hands the skb via `.tx` (non-TXQ frames) or the driver pulls it from a TXQ
   (`.wake_tx_queue` → work → `ieee80211_tx_dequeue_ni`) (mac80211.c:19-55, core.c:4706-4735).
2. `rtw89_core_tx_write()` picks the vif/sta link and builds a `rtw89_tx_desc_info` via
   `rtw89_core_tx_update_desc_info()` (core.c:1227-1290, 1377-1441).
3. HCI layer (`rtw89_pci_tx_write`, pci.c:1637-1678): take lock, require ≥1 free TXBD, dequeue a
   free 128-byte **WD page** (512 pages per channel, pre-zeroed), `rtw89_pci_txwd_submit`:
   - DMA-map the 802.11 frame (skb->data, skb->len) (pci.c:1514).
   - write `wp_info` and `addr_info` after the WD, then `fill_txdesc` into the start of the page
     (pci.c:1523-1541).
   - put the page on the channel's busy list; TXBD = {len = WD bytes, LS, DMA of page} (pci.c:1619-1627).
4. Kick the channel (write host index) — once per frame from `.tx`, once per TXQ batch from the
   scheduler (core.c:1300-1307, 4820-4821).
5. HW transmits; on completion it posts a 4-byte **release report (RPP)** into the RPQ RX ring
   naming the WD page (seq) and a TX status (§7). The driver reports status to mac80211 and frees the
   page once both the RPP was seen and the TXBD index passed it (pci.c:501-570).

H2C / firmware-download commands take a different, simpler path (§1.6): the WD body is
`skb_push`ed in front of the command and the TXBD points straight at it on DMA channel 12.

### 1.2 Layout of one WD page (data and mgmt frames)

WD pages are `dma_alloc_coherent` memory, 128 B each, 512 per TX channel (pci.h:1124-1125,
pci.c:3535-3584), and are **memset to 0 whenever they are returned to the free list**
(pci.h:1720-1728). So any dword the builder does not write is 0.

| Offset | Size | Content | Source |
|---|---|---|---|
| 0x00 | 24 | `txwd_body` dw0..dw5 (v0) | core.h:1092-1099 |
| 0x18 | 24 | `txwd_info` dw0..dw5 — present only when body dw0 `WD_INFO_EN`=1 (always 1 for data/mgmt) | core.h:1123-1130 |
| 0x30 | 8 | `wp_info`: seq0 (u16) = WD page index (0..511) \| BIT(15) VALID; seq1..seq3 = 0 | pci.h:1473-1480, pci.c:1527-1531 |
| 0x38 | 8 | `addr_info` #0: u16 length = skb->len; u16 option = BIT(15) MSDU_LS \| ((dma>>32)&0xFF)<<6 \| NUM(=1 in bits[5:0]); u32 dma = low 32 bits of frame DMA address | pci.h:1482-1491, pci.c:1438-1455 |
| total | **64** | → TXBD length = 64 (40 if WD info disabled) | pci.c:1539, 1621-1625 |

The 802.11 frame itself is NOT in the WD page; it is a separate DMA buffer referenced by addr_info.
The frame is a full 802.11 MPDU (header + body, **no FCS**; HW appends FCS; with HW crypto, no MIC:
HW appends MIC/ICV, see §6).

### 1.3 `txwd_body` (8852B v0) — all dwords

| dw | Bits | Field | Written by v0? | Meaning / value source |
|---|---|---|---|---|
| 0 | [31:24] | WP_OFFSET | yes | `wp_offset` (unit 8 B; security-header offset for HW IV gen). Always **0** on 8852B (only set when `hw_sec_hdr`, core.c:698-708) |
| 0 | 23 | MORE_DATA | no (0) | PS more-data bit (unused) |
| 0 | 22 | WD_INFO_EN | yes | 1 = `txwd_info` follows (data & mgmt: 1; H2C: 0) |
| 0 | 21 | — | — | reserved |
| 0 | 20 | FW_DL | yes | 1 only for firmware-download chunks sent on CH12 |
| 0 | [19:16] | CHANNEL_DMA | yes | DMA channel number (0-3 ACH, 8 B0MG, 9 B0HI, 12 H2C) — must match the ring used |
| 0 | [15:11] | HDR_LLC_LEN | yes | 802.11 header length / 2 (data frames only; 0 for mgmt/H2C) |
| 0 | 10 | STF_MODE | no (0) | |
| 0 | [9:8] | — | — | |
| 0 | 7 | WD_PAGE | yes | 1 = WD lives in a WD page (all data/mgmt); 0 for H2C/FWDL |
| 0 | 6 | — | — | |
| 0 | 5 | HW_AMSDU | no (0) | HW A-MSDU aggregation not used |
| 0 | [3:2] | HW_SSN_SEL | yes | HW sequence-number counter select: mgmt=**1**, data=0 |
| 0 | [1:0] | HW_SSN_MODE (EN_HWSEQ) | yes | mgmt=**1** (HW assigns seq), data=0 (use header seq) |
| 1 | [31:26] | ADDR_INFO_NUM | **no** (0) | only written by v1 builder (8852C) |
| 1 | [31:16] | PAYLOAD_ID | no | |
| 1 | [5:4] | SEC_KEYID | **no** (0) | v1 only |
| 1 | [3:0] | SEC_TYPE | **no** (0) | v1 only; 8852B carries sec type in info dw2 |
| 2 | 31 | — | | |
| 2 | [30:24] | MACID | yes | 7-bit mac_id (§2.2) |
| 2 | 23 | TID_INDICATE | yes | 1 for TID 2,3,5,7 (second TID of the AC pair), else 0 (txrx.h:873-890) |
| 2 | [22:17] | QSEL | yes | queue select (§2.1) |
| 2 | [16:14] | — | | |
| 2 | [13:0] | TXPKT_SIZE | yes | = skb->len at submit (header+body, incl. IV if present, excl. FCS/MIC); max 16383 |
| 3 | 13 | BK | yes | "break aggregation": set for EAPOL frames, and when the HTC/A-ctrl state flips vs previous frame |
| 3 | 12 | AGG_EN | yes | 1 = frame may be aggregated in an A-MPDU |
| 3 | [11:0] | SW_SEQ | yes | 802.11 sequence number copied from the frame header (`seq_ctrl>>4`) (core.c:1255, 1267) |
| 4 | [31:24] / [23:16] | SEC_IV_L1 / L0 | **no** (0) | only when `hw_sec_hdr` (false) |
| 5 | [31:0] | SEC_IV_H5..H2 | **no** (0) | idem |

Builder: core.c:1443-1455 (dw0), 1478-1486 (dw2), 1488-1495 (dw3); v0 writes only dw0, dw2, dw3
(core.c:1607-1609). Field masks: txrx.h:68-107.

### 1.4 `txwd_info` (8852B v0) — all dwords (present when WD_INFO_EN=1)

| dw | Bits | Field | Value / source |
|---|---|---|---|
| 0 | 31 | — | |
| 0 | 30 | USE_RATE | 1 = use DATA_RATE/BW/GI below, bypass FW rate adaptation. mgmt/nullfunc=1, data=0, injected=1 |
| 0 | [29:28] | DATA_BW | 0=20,1=40,2=80,3=160 MHz (only meaningful with USE_RATE; rtw89 leaves 0 except injection) |
| 0 | [27:25] | GI_LTF | HT/VHT: 0=LGI,1=SGI; HE: `enum rtw89_gi_ltf` (§4.8) |
| 0 | [24:16] | DATA_RATE | 9-bit HW rate code (§1.7) |
| 0 | 15 | DATA_ER | **not written on v0** (v1 only) → 0 |
| 0 | [14:13] | — | |
| 0 | 12 | DATA_STBC | data: peer `ra.stbc_cap`; mgmt: 0 |
| 0 | 11 | DATA_LDPC | data: peer `ra.ldpc_cap`; mgmt: 0 |
| 0 | 10 | DISDATAFB | 1 = disable rate fallback on retries (mgmt/injected=1, data=0) |
| 0 | 9 | — | |
| 0 | 8 | DATA_BW_ER | not written on v0 → 0 |
| 0 | [6:4] | MULTIPORT_ID | HW port; only for HIQ (AP DTIM-buffered) frames; otherwise 0 |
| 1 | 31 | DATA_TXCNT_LMT_SEL | 1 = use DATA_TXCNT_LMT; only with USB tx_rpt → **0 on PCIe** |
| 1 | [30:25] | DATA_TXCNT_LMT | retry limit (USB: 8) → 0 on PCIe |
| 1 | [24:16] | DATA_RTY_LOWEST_RATE | lowest rate for fallback, data only: `lowest + ffs(peer supp_rates[band])` (core.c:1043-1083, 1117) |
| 1 | 14 | A_CTRL_BSR | 1 when driver inserted an HE HT-Control field (§8.5) |
| 1 | [7:0] | MAX_AGGNUM | A-MPDU max subframes − 1 (§5.2) |
| 2 | [20:18] | AMPDU_DENSITY | peer HT `ampdu_density` (802.11 MPDU start-spacing code 0..7) |
| 2 | [12:9] | SEC_TYPE | `enum rtw89_sec_key_type` (§6.2) when HW encryption requested |
| 2 | 8 | SEC_HW_ENC | 1 = HW encrypts with key at SEC_CAM_IDX |
| 2 | [7:0] | SEC_CAM_IDX | security CAM entry index (= mac80211 `hw_key_idx`) |
| 3 | 10 | SPE_RPT | request FW TX report (USB tx_rpt only) → **0 on PCIe** |
| 4 | 31 | HW_RTS_EN | always 1 (HW decides RTS/CTS by length/time threshold, set by MAC init track) |
| 4 | 27 | RTS_EN | `!is_bmc` (1 for unicast addr1, 0 for bcast/mcast) |
| 4 | [3:0] | SW_DEFINE | 4-bit TX-report SN (USB only) → 0 on PCIe |
| 5 | — | — | never written (0) |

Builders: core.c:1525-1537, 1551-1598; masks txrx.h:117-151.

Fields that exist in other generations but **do not exist / are not used** in the 8852B v0 WD:
checksum, CTS2SELF, CCA_RTS, SIFS_TX, LSIG_TXOP_EN, DATA_DCM, NO_ACK, BMC bit, A4_HDR, LIFETIME_SEL,
FORCE_TXOP, SPE_PKT/NULL_0/NULL_1 (all BE-only, txrx.h:153-342). DATA_ER/DATA_BW_ER exist in
info dw0 but only the v1 builder writes them. `lsig_txop` for AX is configured per-port in the
ADDR CAM entry (cam.c:890), not in the WD.

### 1.5 Field values per frame type (what rtw89 actually writes on 8852B PCIe)

Classification (core.c:586-597): **MGMT** = any management frame OR any (QoS-)null-function
frame; **DATA** = everything else from mac80211; **FWCMD** = H2C.

| Field | DATA (normal) | DATA EAPOL | MGMT / nullfunc | H2C (FWCMD) | FW download |
|---|---|---|---|---|---|
| WD_INFO_EN | 1 | 1 | 1 | 0 | 0 |
| WD_PAGE | 1 | 1 | 1 | 0 | 0 |
| FW_DL | 0 | 0 | 0 | 0 | 1 |
| CHANNEL_DMA | 0..3 (§2.1) | 0..3 | 8 (9 if HIQ) | 12 | 12 |
| QSEL | 0..3 by TID | by TID | 0x12 B0_MGMT (0x11 if HIQ) | 0 | 0 |
| TID_INDICATE | by TID | by TID | 0 | 0 | 0 |
| MACID | sta/vif macid | idem | idem | 0 | 0 |
| HDR_LLC_LEN | hdrlen/2 (e.g. QoS 26→13, +HTC 30→15, non-QoS 24→12) | idem | 0 | 0 | 0 |
| HW_SSN_SEL / MODE | 0 / 0 | 0 / 0 | 1 / 1 | 0 | 0 |
| SW_SEQ | header seq | header seq | header seq (HW overrides) | 0 | 0 |
| TXPKT_SIZE | skb->len | skb->len | skb->len | H2C hdr+payload length (before WD push) | chunk length |
| AGG_EN | 1 if `IEEE80211_TX_CTL_AMPDU` | 0 | 0 | 0 | 0 |
| BK | 0 (1 on HTC-state change) | 1 | 0 | 0 | 0 |
| USE_RATE / DISDATAFB | 0 / 0 | 0 / 0 | 1 / 1 | — | — |
| DATA_RATE | 0 | 0 | mgmt rate (below) | — | — |
| DATA_RTY_LOWEST_RATE | lowest peer rate | same | 0 | — | — |
| DATA_STBC / DATA_LDPC | peer caps | peer caps | 0 | — | — |
| MAX_AGGNUM / AMPDU_DENSITY | if AGG_EN | 0 | 0 | — | — |
| SEC_* | if HW key (§6) | if HW key | never (hw_mgmt_tx_encrypt=false) | — | — |
| RTS_EN / HW_RTS_EN | !bmc / 1 | !bmc / 1 | !bmc / 1 | — | — |

Mgmt rate (core.c:711-749): `lowest` = OFDM6 (0x04) if `IEEE80211_TX_CTL_NO_CCK_RATE` or P2P
vif, else CCK1 (0x00) on 2.4 GHz, OFDM6 on 5 GHz. If the vif has `basic_rates` AND a sta is attached:
`rate = lowest + ffs(basic_rates)` (bitmap index into the sband bitrate table, which on 5 GHz starts at
6 Mbps, hence the OFDM6 base); otherwise `rate = lowest`. DATA_BW=0 (20 MHz), GI_LTF=0.

Injected frames (`IEEE80211_TX_CTL_INJECTED`, monitor only): USE_RATE=1, DISDATAFB=1, BW from
`control.rates[0].flags` (40→1, 80→2, 160→3), SGI→GI_LTF=1, rate from VHT/HT/legacy index
(core.c:1189-1225). Optional.

Worked examples (little-endian dword values):

* QoS data, TID 0, macid 0, 1534-byte MPDU, seq 0x123, 5 GHz, no crypto, not aggregated:
  body dw0 = `0x00406880` (WD_INFO_EN | CH 0 | HDR_LLC_LEN 13 | WD_PAGE); dw2 = `0x000005FE`;
  dw3 = `0x00000123`; info dw0 = 0 (+0x1000/0x800 if STBC/LDPC cap); dw1 = `0x00040000`
  (lowest retry rate OFDM6); dw2 = 0; dw4 = `0x88000000`.
* Same but inside a BA session with buf_size 64, density 5, CCMP-128 key in sec CAM 3:
  body dw3 |= `0x1000`; info dw1 |= `0x3F`; info dw2 = (5<<18) | (6<<9) | BIT(8) | 3 = `0x00140D03`.
* Unicast auth frame, 30 bytes, 5 GHz, OFDM6: body dw0 = `0x00480085` (WD_INFO_EN | CH 8 | WD_PAGE |
  SSN_SEL 1 | SSN_MODE 1); dw2 = `0x0024001E` (QSEL 0x12, size 30); info dw0 = `0x40040400`
  (USE_RATE | rate 0x04 | DISDATAFB); info dw4 = `0x88000000` (broadcast probe-req: `0x80000000`).
* H2C of L bytes (8-byte H2C header included): body dw0 = `0x000C0000`, dw2 = L, all else 0;
  FW-download chunk: dw0 = `0x001C0000`.

### 1.6 H2C / FWDL WD (CH12)

`rtw89_h2c_tx()` (core.c:1336-1375): `desc_info` = {pkt_size = skb->len, fw_dl, is_bmc=0,
wd_page=0, ch_dma = 12} (core.c:904-913, 1241-1246). Needs ≥1 free CH12 TXBD (else -ENOSPC), then
`rtw89_pci_fwcmd_submit` (pci.c:1551-1588): push 24 bytes in front of the skb, zero them, fill the
v0 body, DMA-map the whole thing, TXBD = {len = 24 + L, LS, dma}, kick CH12. No WD page, no RPP:
CH12 buffers are released by TXBD-index reclaim, keeping the newest 8 (`RTW89_PCI_MULTITAG`)
buffers mapped (pci.c:100-144, pci.h:1131). `rtw89_pci_tx_write` refuses any non-FWCMD frame on CH12
and any FWCMD on another channel (pci.c:1646-1653). H2C header format belongs to the FW track.

### 1.7 HW rate code (shared by TX DATA_RATE, RX desc data_rate, PPDU rate) — AX encoding

9-bit value (core.h:341-473, txrx.h:10-66):

| bits [8:7] mode | Meaning | Low bits |
|---|---|---|
| 0 (0x000-0x00B) | legacy | [3:0] index: 0..3 = CCK 1/2/5.5/11, 4..11 = OFDM 6/9/12/18/24/36/48/54 |
| 1 (0x080-0x09F) | HT | [4:0] MCS 0..31 ([4:3] = NSS−1) |
| 2 (0x100-0x139) | VHT | [6:4] NSS−1, [3:0] MCS 0..9 → `0x100 + (nss−1)·0x10 + mcs` |
| 3 (0x180-0x1BB) | HE | [6:4] NSS−1, [3:0] MCS 0..11 → `0x180 + (nss−1)·0x10 + mcs` |

Legacy index ↔ bitrate (100 kbps units) table: 10,20,55,110,60,90,120,180,240,360,480,540
(core.c:152-165).

---------------------------------------------------------------------------------------------------

## 2. Queue mapping and mac_id

### 2.1 TID → QSEL → DMA channel (band 0, 8852BE)

rtw89 maps by **`skb->priority & 7` (TID)**, not by mac80211's queue index (core.c:1096-1099,
txrx.h:833-852, core.c:782-817). `hw->queues` = 4 (core.c:7378).

| TID | QSEL (value) | TID_INDICATE | DMA channel | mac80211 AC |
|---|---|---|---|---|
| 0 | BE_0 (0x00) | 0 | ACH0 (0) | BE |
| 3 | BE_0 (0x00) | 1 | ACH0 (0) | BE |
| 1 | BK_0 (0x01) | 0 | ACH1 (1) | BK |
| 2 | BK_0 (0x01) | 1 | ACH1 (1) | BK |
| 4 | VI_0 (0x02) | 0 | ACH2 (2) | VI |
| 5 | VI_0 (0x02) | 1 | ACH2 (2) | VI |
| 6 | VO_0 (0x03) | 0 | ACH3 (3) | VO |
| 7 | VO_0 (0x03) | 1 | ACH3 (3) | VO |
| mgmt / nullfunc / QoS-null | B0_MGMT (0x12) | 0 | CH8 (8) | — |
| `IEEE80211_TX_CTL_SEND_AFTER_DTIM` (AP only) | B0_HI (0x11) | — | CH9 (9) | — |
| H2C / FWDL | (0) | — | CH12 (12) | — |

Full QSEL enum (txrx.h:798-831): BE/BK/VI/VO for WMM set 0..3 = 0x00..0x0F; B0_BCN 0x10, B0_HI 0x11,
B0_MGMT 0x12, B0_NOPS 0x13, B0_MGMT_FAST 0x14; B1_* 0x18..0x1C. `rtw89_core_get_ch_dma` maps
BE_n→ACH0, BK_n→ACH1, VI_n→ACH2, VO_n→ACH3, B0_MGMT→CH8, B0_HI→CH9, B1_MGMT→CH10, B1_HI→CH11,
anything else → warn + ACH0. Beacons (B0_BCN) are never DMA'd by the driver; they go to FW via the
"update beacon" H2C (AP only).

Channels that exist on 8852BE: ACH0..ACH3, CH8, CH9, CH12 (the pci_info mask removes ACH4-7,
CH10, CH11; rtw8852be.c:54-56; also `chip->dma_ch_mask`, rtw8852b.c:1106-1108). RX rings: RXQ (0)
carries WIFI/PPDU-status/C2H packets, RPQ (1) carries release reports (txrx.h:789-796).

A **station-mode minimal driver** only needs ACH0-3 (data), CH8 (mgmt + nullfunc) and CH12 (H2C).

### 2.2 mac_id assignment

* The HW has 128 mac_ids (`RTW89_MAX_MAC_ID_NUM`, core.h:5349). Allocation = first free bit in a
  bitmap (core.c:6678-6695).
* `add_interface`: vif gets the first free mac_id (→ **0** for the first vif) and the first free HW
  port (→ port 0) (mac80211.c:189-199, core.c:6702-6735).
* Station mode: the AP's `ieee80211_sta` **reuses the vif's mac_id** ("for station mode, assign the
  mac_id from itself", mac80211.c:501-505). Only AP-mode clients / TDLS peers get a separate mac_id.
* Per-frame (core.c:751-769): if a sta is attached → sta's mac_id, else the vif's mac_id. So in STA
  mode every frame (unicast, broadcast probe requests, EAPOL, mgmt) carries MACID = vif mac_id.
  There is **no dedicated broadcast mac_id** on this path.
* The same mac_id indexes the FW CMAC/RA tables, the ADDR CAM entry (vif's entry is used for the
  AP in STA mode, core.h:7409-7419), release reports and RX descriptors (`mac_id` in RX long desc,
  RA report macid).

---------------------------------------------------------------------------------------------------

## 3. RX descriptor

### 3.1 RX buffer layout (RXQ ring, packet mode)

Each RX DMA buffer (`RTW89_PCI_RX_BUF_SIZE` = 11454+40+4 = 11498 B, pci.h:1128) contains:

| Offset | Content |
|---|---|
| 0 | 4-byte `rxbd_info` (FS BIT15, LS BIT14, write size [13:0], tag [28:16]) — Track 01 (pci.h:1535-1542) |
| 4 | RX descriptor: 16 B (short) or 32 B (long, dw0 BIT31) |
| 4 + rxd_len | `drv_info_size`·8 bytes of driver info (skipped) and `shift`·2 bytes (see quirk) |
| 4 + rxd_len + drv_info·8 + shift·2 | payload (802.11 frame incl. 4-byte FCS / PPDU status / C2H / RPP list) |

`rtw89_pci_rxbd_deliver_skbs` (pci.c:321-411): on FS, parse RX desc at offset 4, allocate a new
skb of `pkt_size` (+radiotap headroom in monitor mode, core.h:7950-7965) and copy
`bd_len − payload_offset` bytes (or `pkt_size` if that does not fit and FS&&LS); on LS call
`rtw89_core_rx()`. Multi-segment packets (FS without LS) are appended from offset 4 of later buffers.

### 3.2 AX RX descriptor fields (8852B)

Short descriptor = dw0..dw3 (16 B); long = dw0..dw7 (32 B). Masks: txrx.h:344-428.
"Parsed" = read by `rtw89_core_query_rxdesc` (core.c:4070-4124).

| dw | Bits | Field | Parsed | Notes |
|---|---|---|---|---|
| 0 | [13:0] | RPKT_LEN (pkt_len) | yes | payload length (WIFI: MPDU incl. FCS) |
| 0 | [15:14] | SHIFT | **no** | unit 2 B; rtw89 never reads it on AX (treated as 0) |
| 0 | [21:16] | WL_HD_IV_LEN | no | header+IV length (header-conversion feature) |
| 0 | 22 | BB_SEL | **no** | PHY index (always treated as PHY0) |
| 0 | 23 | MAC_INFO_VLD | yes | PPDU status: MAC-info block present |
| 0 | [27:24] | RPKT_TYPE | yes | §3.3 |
| 0 | [30:28] | DRV_INFO_SIZE | yes | unit 8 B |
| 0 | 31 | LONG_RXD | yes | 1 = 32-byte descriptor |
| 1 | [3:0] | PPDU_TYPE | yes | `enum rtw89_rx_ppdu_type`: 0 LCCK,1 SCCK,2 OFDM,3 HT,4 HTGF,5 VHT_SU,6 VHT_MU,7 HE_SU,8 HE_ERSU,9 HE_MU,10 HE_TB,15 unknown (core.h:960-973) |
| 1 | [6:4] | PPDU_CNT | yes | 3-bit per-PPDU counter; links MPDUs with their PPDU status |
| 1 | 7 | SR_EN | yes | spatial reuse |
| 1 | [15:8] | USER_ID | yes | MU user index |
| 1 | [24:16] | RX_DATARATE | yes | HW rate code (§1.7) |
| 1 | [27:25] | RX_GI_LTF | yes | §4.8 |
| 1 | 28 | NON_SRG_PPDU | no | |
| 1 | 29 | INTER_PPDU | no | |
| 1 | [31:30] | BW | yes | 0=20,1=40,2=80,3=160 (8852C uses [31:29]; 8852B uses [31:30], core.c:4085-4088) |
| 2 | [31:0] | FREERUN_CNT | yes | free-running counter → `mactime` |
| 3 | 0 | A1_MATCH | yes | addr1 matched |
| 3 | 1 | SW_DEC | yes | protected frame left for SW decryption |
| 3 | 2 | HW_DEC | yes | HW decrypted |
| 3 | 3 | AMPDU | yes | MPDU was in an A-MPDU |
| 3 | 4 | AMPDU_END_PKT | no | |
| 3 | 5 | AMSDU | no | |
| 3 | 6 | AMSDU_CUT | no | |
| 3 | 7 | LAST_MSDU | no | |
| 3 | 8 | BYPASS | no | |
| 3 | 9 | CRC32_ERR | yes | |
| 3 | 10 | ICV_ERR | yes | |
| 3 | 11/12/13 | MAGIC/UNICAST/PATTERN_WAKE | no | WoWLAN |
| 3 | [15:14] | GET_CH_INFO | no | |
| 3 | [20:16] | PATTERN_IDX | no | |
| 3 | [23:21] | TARGET_IDC | no | |
| 3 | 24 | CHKSUM_OFFLOAD_EN | no | |
| 3 | 25 | WITH_LLC | no | |
| 3 | 26 | RX_STATISTICS | no | |
| 4 | [1:0] | TYPE (frame_type) | yes | 0 mgmt, 1 ctrl, 2 data, 3 rsvd (core.h:3448-3453) |
| 4 | 2/3/4/5/6/7 | MC / BC / MD / MF / PWR / QOS | no | |
| 4 | [11:8] | TID | no | |
| 4 | 12/13/14 | EOSP / HTC / QNULL | no | |
| 4 | [27:16] | SEQ | no | |
| 4 | [31:28] | FRAG | no | |
| 5 | [7:0] | SEC_CAM_IDX | yes | |
| 5 | [15:8] | ADDR_CAM (id) | yes | |
| 5 | [23:16] | MAC_ID | yes | mac_id of the matched ADDR CAM entry |
| 5 | [27:24] | RX_PL_ID | yes | |
| 5 | 28 | ADDR_CAM_VLD | yes | |
| 5 | 29 | ADDR_FWD_EN | no | |
| 5 | 30 | RX_PL_MATCH | no | |
| 6 | [31:0] | MAC_ADDR[31:0] | no | |
| 7 | [15:0] | MAC_ADDR[47:32] | no | |
| 7 | 16 | SMART_ANT | no | |
| 7 | [20:17] | SEC_TYPE | yes | `enum rtw89_sec_key_type` |
| 7 | 21 | HDR_CNV | no | |
| 7 | [26:22] | HDR_OFFSET | no | |
| 7 | 27 / 28 | BIP_KEYID / BIP_ENC | no | |

Derived: payload offset = 4 + (16|32) + drv_info_size·8 + shift·2 (core.c:4103-4109, pci.c:376).

**Quirks a new driver should NOT copy blindly:**
* SHIFT and BB_SEL are never parsed on AX (core.c:4070-4124 has no read of AX_RXD_SHIFT_MASK;
  `desc_info->shift` is only set by the BE parsers at core.c:4141, 4216). Parse SHIFT and honour it;
  expect 0.
* The PCIe RXQ `rtw89_rx_desc_info` (`rx_ring->diliver_desc`) is **persistent and never zeroed**
  (pci.c:324, pci.h:1611); fields that exist only in the long descriptor (frame_type, mac_id, sec_type,
  …) keep stale values when a short descriptor arrives. Clear them per packet.

### 3.3 RPKT_TYPE enum and handling

`enum rtw89_core_rx_type` (core.h:222-238); dispatch in `rtw89_core_rx` / `rtw89_core_rx_process_report`
(core.c:4490-4502, 4051-4068):

| Value | Name | Arrives on | rtw89 handling |
|---|---|---|---|
| 0 | WIFI | RXQ | 802.11 MPDU → §4.5 / mac80211 |
| 1 | PPDU_STAT | RXQ | PHY/PPDU status → §4 |
| 2 | CHAN_INFO | — | dropped (debug print) |
| 3 | BB_SCOPE | — | dropped |
| 4 | F2P_TXCMD | — | dropped |
| 5 | SS2FW | — | dropped |
| 6 | TX_REPORT | — | dropped |
| 7 | TX_REL_HOST | RPQ | release reports; the RPQ path does not look at the type at all — every RPQ buffer is parsed as a list of 4-byte RPP entries (pci.c:644-695) |
| 8 | DFS_REPORT | — | dropped |
| 9 | TX_REL_CPU | — | dropped |
| 10 | C2H | RXQ | firmware event → `rtw89_fw_c2h_irqsafe` (FW track; RA report in §7.6) |
| 11 | CSI | — | dropped |
| 12 | CQI | — | dropped |
| 13 | H2C | — | dropped |
| 14 | FWDL | — | dropped |

There is no "TX_ACCU" type in rtw89. In all cases the RX descriptor (and drv_info/shift) is stripped
before the handler sees the skb.

---------------------------------------------------------------------------------------------------

## 4. PPDU status / PHY status

### 4.1 Enabling (done at `rtw89_core_start`, core.c:6607; and toggled around channel switches)

1. MAC PPDU status report (`rtw89_mac_cfg_ppdu_status_ax`, mac.c:6288-6310):
   - `R_AX_PPDU_STAT` **0xCE40** = `0x0000002B` = RPT_EN BIT0 | APP_MAC_INFO_RPT BIT1 |
     APP_PLCP_HDR_RPT BIT3 | PPDU_STAT_RPT_CRC32 BIT5. (Not set: APP_RX_CNT_RPT BIT2, RPT_A1M BIT4,
     reg.h:3378-3386.) Band-1 copy at +0x2000 (not used).
   - `R_AX_HW_RPT_FWD` **0x9C18** bits[1:0] = 1 (`RTW89_PRPT_DEST_HOST`; 2 = WLCPU) (reg.h:1830-1833).
   - Disabled (write-clear BIT0) during `set_channel_help(enter)` and re-enabled after
     (rtw8852b.c:682-690), and during WoWLAN (mac.c:7360/7383).
   - `cfg_phy_rpt` is NULL for AX (mac.c:7476).
2. PHY status IE selection (`__rtw89_physts_parsing_init`, phy.c:7085-7135, called from
   `rtw89_phy_dm_init`, phy.c:8172). BB registers, absolute MMIO = 0x10000 + offset (phy.h:730-793,
   phy.c:8970):
   - `R_PLCP_HISTOGRAM` 0x0738 (→0x10738): set BIT3 `B_STS_DIS_TRIG_BY_FAIL` and BIT2
     `B_STS_DIS_TRIG_BY_BRK` (no reports for failed/broken PPDUs) (phy.c:7054-7072, reg.h:8854-8858).
   - Per-PPDU-class IE bitmap registers at 0x073C + 4·k (k skips RSVD_9) (phy.c:6995-7021,
     reg.h:8859-8877). Read-modify-write of the BB-table default:

     | class (enum `rtw89_phy_status_bitmap`, phy.h:274-294) | reg (+0x10000) | modification (normal mode) |
     |---|---|---|
     | 0 TD_SEARCH_FAIL … 5 DL_MU_SPOOFING, 8 UL_TB_SPOOFING | 0x073C,0x0740,0x0744,0x0748,0x074C,0x0750,0x075C | unchanged |
     | 6 HE_MU / 7 VHT_MU | 0x0754 / 0x0758 | `|= BIT(13)` (IE13 DL_MU_DEF) |
     | 10 TRIG_BASE_PPDU | 0x0760 | `|= BIT(13) | BIT(1)` |
     | 11 CCK_PKT | 0x0764 | `&= ~0xF0` (drop IE4-7 per-path ext), `|= BIT(1)` (IE01) |
     | 12 LEGACY_OFDM_PKT | 0x0768 | `&= ~0xF0` |
     | 13 HT / 14 VHT / 15 HE | 0x076C / 0x0770 / 0x0774 | `&= ~0xF0`, `|= BIT(20)` (IE20) |
     | 16 EHT | 0x0788 | skipped on AX |

     Monitor mode additionally ORs BIT(9)|BIT(10) into the MU classes and BIT(9) into VHT/HE.
   The resulting absolute bitmaps depend on BB-table defaults (not visible in rtw89 source).

### 4.2 PPDU status packet layout (RPKT_TYPE 1, payload after RX desc)

Handled by `rtw89_core_rx_process_ppdu_sts` (core.c:4022-4049):

1. If RX desc `MAC_INFO_VLD`: MAC-info block (`struct rtw89_rxinfo`, txrx.h:430-458;
   core.c:1902-1977):
   - w0: USR_NUM [3:0], FW_DEFINE [15:8], LSIG_LEN [27:16], IS_TO_SELF 28, RX_CNT_VLD 29, LONG_RXD [31:30]
   - w1: SERVICE [15:0], PLCP_LEN [23:16] (unit 8 B)
   - `USR_NUM` × 4-byte user words: MAC_ID_VALID 0, DATA 1, CTRL 2, MGMT 3, BCN 4, MACID [15:8].
     USR_NUM > 4 → invalid. First user with MAC_ID_VALID gives `has_data`/`has_bcn`
     (AX does NOT take mac_id from here).
   - +4 pad if USR_NUM is odd (8-byte alignment); +96 B RX counters if RX_CNT_VLD (not enabled);
     +PLCP_LEN·8 B PLCP header (enabled by APP_PLCP_HDR_RPT).
2. PHY status (rest of packet). Header `rtw89_phy_sts_hdr` (8 B, txrx.h:460-473):
   - w0: IE_MAP [4:0] (= PPDU class, enum `rtw89_phy_status_bitmap`), HDR_2_EN 5 (0 on AX), VALID 7,
     LEN [15:8] (unit 8 B; on AX equals the whole PHY-status length incl. header), RSSI_AVG [31:24]
   - w1: RSSI_A [7:0], RSSI_B [15:8], RSSI_C [23:16], RSSI_D [31:24]
   - Checks: VALID must be 1 and LEN·8 must equal the remaining bytes (core.c:2248-2271).
   - Only IE_MAP ≥ 11 (CCK_PKT..HE_PKT) is parsed in normal mode (core.c:2281-2284).
3. IEs follow back-to-back. IE header = low bits of the IE's first dword: TYPE [4:0], LEN [11:5]
   (unit 8 B, used only for variable-length IEs) (txrx.h:482-487, core.c:2046-2061). AX length table
   (bytes; V = variable) (phy.c:8973-8975):

   | IE | 0 | 1 | 2 | 3 | 4-7 | 8 | 9 | 10 | 11 | 12-17 | 18 | 19 | 20-22 | 23 | 24-27 | 28-31 |
   |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
   | len | 16 | 32 | 24 | 24 | 8 | V | 8 | V | 176 | V | 16 | 24 | V | 0 | 24 | 32 |

   A zero length or overrun aborts parsing (core.c:2290-2311). IE names: phy.h:235-272.

### 4.3 IEs used and their fields

Only A1-matched ("to_self" = PPDU-status RX desc dw3 A1_MATCH) reports are parsed, except IE01
which is always parsed (channel index for scanning) (core.c:2193-2200).

* **IE00 CMN_CCK** (16 B; txrx.h:594-602; core.c:2124-2137): w0 RPL [15:7] (9 bits) → `rpl_avg = RPL>>1`;
  w3 RX_PATH_EN [31:28].
* **IE01 CMN_OFDM** (32 B; txrx.h:620-640; core.c:2079-2122):
  - w0: RSSI_AVG_FD [15:8], CH_IDX [23:16], RX_PATH_EN [31:28]
  - w1: FD_CFO [19:8], PREMB_CFO [31:20] — signed S(12,2); 8852B uses **FD_CFO** (`cfo_src_fd`)
  - w2: AVG_SNR [5:0], EVM_MAX [15:8], EVM_MIN [23:16], SU 27, LDPC 28, STBC 30
  - w3: BF 8
  - CH_IDX/LDPC/STBC/BF always taken; the rest (rpl_avg ← RSSI_AVG_FD, SU, SNR, EVM, CFO) only if
    rate ≥ OFDM6 and to_self. CFO is accumulated per mac_id for the CFO-tracking DM
    (`rtw89_phy_cfo_parse`, phy.c:5389-5403). SNR/EVM only feed per-sta EWMA stats.
* **IE09 FTR_0** (SIG-A, 8 B) and **IE10 PLCP_EXT** (SIG-B, var): pointers kept for radiotap in
  monitor mode only (txrx.h:661-767, core.c:2158-2182).

### 4.4 RSSI math (8852B)

1. Header gives `rssi_avg` and per-path `rssi[A..D]`; IE00/IE01 give `rpl_avg`.
2. `__rtw8852bx_convert_rpl_to_rssi` (rtw8852b_common.c:1988-1999): `delta = (u8)(rpl_avg − rssi_avg)`;
   `rssi[A] += delta; rssi[B] += delta` (u8 arithmetic); `rssi_avg = rpl_avg`.
3. Raw → dBm: `dBm = (s8)(raw >> 1) − 110` (`RTW89_RSSI_RAW_TO_DBM`, core.h:56-58), i.e. raw is
   (dBm+110)·2.
4. `__rtw8852bx_query_ppdu` (rtw8852b_common.c:1970-1986), applied to each queued MPDU that matches:
   `signal = dBm(max(rssi[A], rssi[B]))` if not already set; `chains = BIT(0)|BIT(1)`;
   `chain_signal[p] = dBm(rssi[p])` for p = 0,1; if the PHY status parsed OK and CH_IDX≠0, override
   `freq`/`band` from CH_IDX.
5. CH_IDX decode (phy.c:8497-8566): base = bits[7:4], offset = bits[3:0]; base 0 → 2.4 GHz,
   ch = offset; base 2..5 → 5 GHz, ch = {36,100,132,149}[base−2] + 2·offset; base 7..14 → 6 GHz.

Caveat: when a report is not to_self, IE01 skips `rpl_avg`, so step 2 uses rpl_avg = 0 and shifts
all path RSSIs by −rssi_avg (mod 256). rtw89 still applies such values to queued frames that match
(see open questions).

### 4.5 Correlating PPDU status with MPDUs (per band, `rtw89_core_rx`, core.c:4490-4522)

State: `curr_rx_ppdu_cnt[phy]` (init 0xFF) and a per-PHY skb queue (core.c:6370-6378, core.h:6038-6041).

1. WIFI packet arrives with RX-desc PPDU_CNT = c.
2. If c ≠ curr: deliver every queued MPDU to mac80211 **without** PHY info (they get
   `RX_FLAG_NO_SIGNAL_VAL`), then curr = c (core.c:4440-4453, 4504-4507).
3. Fill `ieee80211_rx_status` from the RX desc (§4.6).
4. If long descriptor AND frame_type ∈ {mgmt, data} (`PPDU_FILTER_BITMAP`, phy.h:143-147):
   queue the skb; else (control frames, short desc) deliver immediately (no signal).
5. PPDU_STAT packet arrives (after the MPDUs of its PPDU): parse MAC info + PHY status (§4.2-4.4),
   update per-sta RSSI EWMA for the sta whose mac_id == RX-desc mac_id (to_self only,
   core.c:1999-2044), then for **every** queued skb: if `curr == PPDU-status desc PPDU_CNT` and the
   queued frame's rate_idx, HE GI and bw equal those decoded from the PPDU-status descriptor's
   data_rate/gi_ltf/bw, call `query_ppdu` (§4.4 step 4); deliver it regardless (core.c:3999-4020,
   2400-2434). The PPDU-status skb is freed.

Consequences: queued MPDUs wait for the next PPDU status or the next MPDU of a different PPDU;
MPDUs whose status never comes are delivered with no signal.

### 4.6 `ieee80211_rx_status` fill (core.c:4326-4414, 3972-3997)

| Field | Value |
|---|---|
| `freq`, `band` | operating chandef of CHANCTX_0 primary channel; during FW HW-scan the scan channel; overridden by IE01 CH_IDX (matched frames) and, during scanning, by the DS-param/HT-op channel parsed from beacon/probe-resp IEs (`rx_freq_from_ie`, core.c:3785-3826) |
| `boottime_ns` | `ktime_get_boottime_ns()` for beacons/probe responses |
| `flag` RX_FLAG_FAILED_FCS_CRC | if ICV_ERR or CRC32_ERR |
| `flag` RX_FLAG_DECRYPTED | if HW_DEC && !(SW_DEC \|\| ICV_ERR) |
| `flag` RX_FLAG_AMPDU_DETAILS + `ampdu_reference` = PPDU_CNT | if AMPDU bit |
| `bw` | RX desc BW → RATE_INFO_BW_20/40/80/160 (core.h:7325-7335) |
| `encoding`, `rate_idx`, `nss` | from data_rate (§1.7): LEGACY idx; HT mcs [4:0]; VHT/HE mcs [3:0] + nss [6:4]+1 |
| `enc_flags` RX_ENC_FLAG_SHORT_GI | HT/VHT with gi_ltf ≠ 0 |
| `he_gi` | always filled from gi_ltf (§4.8), also used for PPDU matching |
| `flag` RX_FLAG_MACTIME_START, `mactime` | FREERUN_CNT (dw2) |
| `signal`, `chains`, `chain_signal[]` | only for PPDU-matched frames (§4.4); if signal==0 → RX_FLAG_NO_SIGNAL_VAL (core.c:3779-3783) |
| `rate_idx` fix-up | non-2.4 GHz legacy: `rate_idx −= 4` (5 GHz sband starts at 6 Mbps); CCK index on 5 GHz → 0 (core.c:3166-3182) |
| `enc_flags` BF/LDPC/STBC, radiotap HE/VHT | monitor mode only (core.c:3185-3201, 3755-3777) |

Not used: RX_FLAG_IV_STRIPPED, RX_FLAG_MMIC_STRIPPED, RX_FLAG_ICV_STRIPPED, RX_FLAG_PN_VALIDATED,
RX_FLAG_MMIC_ERROR, RX_FLAG_AMSDU_MORE. Frames go up with `ieee80211_rx_napi(hw, NULL, skb, napi)`
under `local_bh_disable` (core.c:3993-3995). HW appends FCS (MAC init sets `R_AX_MPDU_PROC` 0x9C00
BIT0 APPEND_FCS, mac.c:2470-2471) → `IEEE80211_HW_RX_INCLUDES_FCS` (core.c:7393).

### 4.7 Other RX-path side effects (optional)

* Per-vif stats, beacon TSF tracking, `rtw89_fw_h2c_rssi_offload` for each own-BSS beacon when the
  FW has BEACON_FILTER (8852B FW ≥ 0.29.29.7, fw.c:875; fw.c:5426-5470, core.c:3031-3131).
* AP-mode PS-poll/U-APSD trigger (core.c:4455-4488) — AP only.
* 8852B has no antenna diversity (only 1-RF-path chips, mac.c:3260-3265).

### 4.8 GI/LTF encoding (RX desc RX_GI_LTF, TX GI_LTF, RA report GILTF)

`enum rtw89_gi_ltf` (core.h:3438-3446) and mapping (core.c:2338-2358):

| Value | Name | HE GI reported |
|---|---|---|
| 0 | LGI_4XHE32 | 3.2 µs |
| 1 | SGI_4XHE08 | 0.8 µs |
| 2 | 2XHE16 | 1.6 µs |
| 3 | 2XHE08 | 0.8 µs |
| 4 | 1XHE16 | 1.6 µs |
| 5 | 1XHE08 | 0.8 µs |
| other | — | 3.2 µs (+warn) |

For HT/VHT the same field is 0 = long GI, non-zero = short GI.

---------------------------------------------------------------------------------------------------

## 5. Aggregation

### 5.1 RX A-MPDU

* **Reordering is done by mac80211**: rtw89 does not set `IEEE80211_HW_SUPPORTS_REORDERING_BUFFER`
  (flags at core.c:7388-7402). `hw->max_rx_aggregation_subframes` = 64 (core.c:7379).
* HW generates BlockAcks from its responder **BA CAM**. rtw89 programs a static entry via H2C on
  `IEEE80211_AMPDU_RX_START` and clears it on RX_STOP (mac80211.c:1062-1069). If no static entry is
  free, it silently skips: "hardware can create dynamic BA CAM automatically" (fw.c:2563-2571).
  8852B: 2 static + 4 dynamic entries (rtw8852b.c:1041-1042). When the 2 static entries are full,
  TID 0 or 5 may evict a non-0/5 entry (core.c:5522-5566).
* BA CAM H2C (`rtw89_fw_h2c_ba_cam`, fw.c:2545-2620): H2C cat MAC (1), class BA_CAM (0xC),
  func 0, REC_ACK 0, DONE_ACK 1, payload 8 bytes (fw.h:1920-1935, 4619, 4736-4737):

  | Word | Bits | Field | RX_START | RX_STOP |
  |---|---|---|---|---|
  | w0 | 0 | VALID | 1 | 0 |
  | w0 | 1 | INIT_REQ | 1 ("HW sets the SSN") | 0 |
  | w0 | [3:2] | ENTRY_IDX | static entry 0..1 | same entry |
  | w0 | [7:4] | TID | tid | 0 |
  | w0 | [15:8] | MACID | sta mac_id | sta mac_id |
  | w0 | [19:16] | BMAP_SIZE | 0 if buf_size ≤ 64, 4 if > 64 | 0 |
  | w0 | [31:20] | SSN | params->ssn | 0 |
  | w1 | all | UID/STD_EN/BAND/ENTRY_IDX_V1 | 0 (BACAM_V0_EXT only) | 0 |

* MAC init resets the BA CAM: `R_AX_RESPBA_CAM_CTRL` **0xCE3C** bits[1:0] = 2 (reset all), poll
  bits[1:0] == 0 every 1 µs up to 1000 µs, then set BIT2 `B_AX_SSN_SEL` (mac.c:2904-2917, 2940-2941,
  reg.h:3371-3375).
* RX_START also arms the PN/SN workaround of §6.5 (core.c:3869-3883).

### 5.2 TX A-MPDU

* Session setup is driver-initiated: for each frame dequeued from a sta TXQ, `rtw89_core_txq_check_agg`
  (core.c:4670-4704): EAPOL (`ETH_P_PAE`) → tear down BA on that TID and forbid it for 4 s
  (`RTW89_FORBID_BA_TIMER`, core.h:53; core.c:4645-4668, 4863-4875); if the TXQ is already AMPDU →
  set `IEEE80211_TX_CTL_AMPDU`; else queue a work item that calls
  `ieee80211_start_tx_ba_session(sta, tid, 0)`; -EINVAL marks the TXQ `BLOCK_BA` permanently
  (core.c:4564-4601).
* `ampdu_action` (mac80211.c:1026-1076):
  - TX_START → return `IEEE80211_AMPDU_TX_START_IMMEDIATE` (no H2C, no callback needed).
  - TX_OPERATIONAL → set TXQ AMPDU flag, `ampdu_params[tid] = {agg_num = buf_size, amsdu}`,
    set tid in `ampdu_map`, recompute the global agg limit.
  - TX_STOP_CONT / STOP_FLUSH / STOP_FLUSH_CONT → clear flags, `ieee80211_stop_tx_ba_cb_irqsafe()`,
    recompute agg limit. (`h2c_ampdu_cmac_tbl` is NULL on 8852B → no H2C.)
* Global limit (`rtw89_phy_ra_recalc_agg_limit`, phy.c:778-803): `R_AX_AMPDU_AGG_LIMIT` **0xC610**
  bits[23:16] (`B_AX_RA_TRY_RATE_AGG_LMT`) = min over all sessions of (clamp(agg_num,1,256) − 1),
  or 0x3F if no session (reg.h:2561-2565, mac.c:7457-7458).
* Per-frame WD (core.c:599-640): when `IEEE80211_TX_CTL_AMPDU` and not EAPOL: AGG_EN=1,
  AMPDU_DENSITY = peer `ht_cap.ampdu_density`, MAX_AGGNUM = (agg_num ? agg_num : 4<<ampdu_factor) − 1.
  EAPOL → BK=1, no AGG_EN.
* HW builds the A-MPDU from consecutive AGG_EN WDs of the same mac_id/TID queue; BK breaks it.
* `hw->max_tx_aggregation_subframes` = 128 (core.c:7380).

### 5.3 A-MSDU

* TX: **mac80211 software A-MSDU** (`IEEE80211_HW_TX_AMSDU` + `SUPPORT_FAST_XMIT` +
  `SUPPORTS_AMSDU_IN_AMPDU`, core.c:7394-7396). HW_AMSDU WD bit is never set. mac80211 requires the
  driver to bound A-MSDU size via `link_sta->agg.max_rc_amsdu_len` (mac80211.h:2994-2998); rtw89
  sets it from each FW RA report: 1 (off) if bitrate < 55 Mbps or MCS ≤ 2, 1200 if < 180 Mbps,
  2600 if < 400, 3500 if < 700, else `max_amsdu_limit` = 5000 (phy.c:25-51, 3342-3348), then
  `ieee80211_sta_recalc_aggregates`.
* Optional TX "aggregation wait" (core.c:4747-4787): under high TX traffic, holds a single-frame
  TXQ for up to `max_agg_wait = max_rc_amsdu_len/1500 − 1` scheduler rounds to let A-MSDUs build.
* RX: A-MSDUs are delivered whole (up to 11454 B, `max_vht_mpdu_cap` 11454, rtw8852b.c:976);
  AMSDU/AMSDU_CUT/LAST_MSDU desc bits are ignored; mac80211 de-aggregates. MAC init writes
  `R_AX_CUT_AMSDU_CTRL` 0x9C40 = 0x010E05F0 (mac.c:2472, reg.h:1835-1836; bit meaning undocumented).

---------------------------------------------------------------------------------------------------

## 6. Security

### 6.1 What rtw89 does on 8852B

`rtw89_cam_sec_key_add` (cam.c:465-535):

| Cipher | 8852B | Key flags set for mac80211 |
|---|---|---|
| WEP40/104 | HW (types 1/2) | GENERATE_IV |
| TKIP | **-EOPNOTSUPP → mac80211 SW crypto** (`hw_tkip_crypto`=false) | — |
| CCMP-128 | HW type 6 | GENERATE_IV, SW_MGMT_TX |
| CCMP-256 | HW type 7 (2 CAM entries) | GENERATE_IV, SW_MGMT_TX |
| GCMP-128 | HW type 8 | GENERATE_IV, SW_MGMT_TX |
| GCMP-256 | HW type 9 (2 CAM entries) | GENERATE_IV, SW_MGMT_TX |
| AES-CMAC (BIP) | installed as type 10 BIP_CCMP128 | GENERATE_IV |
| others | -EOPNOTSUPP → SW | — |

`set_key` returns -EOPNOTSUPP unchanged to mac80211, which then does that key in software
(mac80211.c:999-1009). DISABLE_KEY first flushes TXQ work, DMA rings and MAC queues
(mac80211.c:1010-1019).

* `GENERATE_IV` (because `hw_sec_hdr`=false): mac80211 writes the IV/PN header into the frame; HW only
  encrypts payload and appends MIC/ICV (MAC init: `R_AX_SEC_MPDU_PROC` 0x9D04 |= APPEND_ICV BIT1 |
  APPEND_MIC BIT0, mac.c:2505-2510).
* `SW_MGMT_TX`: robust mgmt frames are encrypted by mac80211 (mgmt WDs never carry SEC fields).
* Security engine init (mac.c:2479-2517): `R_AX_SEC_ENG_CTRL` 0x9D00 |= CLK_EN_CGCMP BIT10 |
  CLK_EN_WAPI BIT9 | CLK_EN_WEP_TKIP BIT8 | MC_DEC BIT3 | BC_DEC BIT2 | SEC_RX_DEC BIT1 |
  SEC_TX_ENC BIT0 (= 0x70F), clear TX_PARTIAL_MODE BIT11; UC_MGNT_DEC BIT4 is **not** set on 8852B,
  so unicast protected mgmt frames arrive encrypted and mac80211 decrypts them in SW even with HW keys.

### 6.2 TX descriptor fields with HW crypto (core.c:642-709, 1114-1115)

For data frames with `info->control.hw_key`: info dw2 SEC_TYPE [12:9] = key type
(0 none, 1 WEP40, 2 WEP104, 3 TKIP, 4 WAPI, 5 GCMSMS4, 6 CCMP128, 7 CCMP256, 8 GCMP128, 9 GCMP256,
10 BIP_CCMP128; core.h:311-323), SEC_HW_ENC BIT8 = 1, SEC_CAM_IDX [7:0] = `hw_key_idx`. No IV in the
WD (body dw4/dw5 and WP_OFFSET stay 0). TXPKT_SIZE includes the mac80211-built IV header but not the
MIC. HDR_LLC_LEN (header length) is what the engine uses to find the payload start.

### 6.3 Key installation (for reference)

1. Sec CAM entry via H2C cat MAC(1) class SEC_CAM (0xA) func SEC_UPD (1), REC_ACK 1, payload 24 B
   (cam.c:11-100; fw.h:496-544, 4599, 4732-4733): dw0 IDX [7:0], OFFSET [15:8], LEN [23:16] (=20);
   dw1 TYPE [3:0], EXT_KEY 4, SPP_MODE 5; dw2..dw5 key bytes 0..15 (LE). 256-bit keys use two
   consecutive entries, the second carrying key bytes 16..31 with EXT_KEY=1.
2. Attach to the ADDR CAM entry of the vif (STA mode) and resend the ADDR CAM H2C (cam.c:248-291,
   905-925, cam.h:81-103): w9 SEC_ENT_MODE [17:16] (default 2 = NORMAL: slots 0-1 unicast, 2-4 group,
   5-6 BIP; cam.c:672), SEC_ENTn_KEYID 2 bits each from [19:18]; w10 SEC_ENT_VALID [7:0] bitmap and
   SEC_ENT0..2 bytes; w11 SEC_ENT3..6 bytes.
   (ADDR CAM H2C format itself belongs to the CAM/FW track.)

### 6.4 RX with HW crypto

RX desc HW_DEC=1 → `RX_FLAG_DECRYPTED` (unless SW_DEC or ICV_ERR). rtw89 sets no *_STRIPPED
flags, so the IV header and MIC/ICV bytes stay in the frame and mac80211 strips them and performs
the PN replay check from the IV (mac80211.h:1540-1549). ICV errors are forwarded to the host
(`R_AX_MPDU_PROC` BIT1 A_ICV_ERR) and flagged FAILED_FCS_CRC so mac80211 drops them.

### 6.5 AX PN/SN workaround (only with HW decryption)

`rtw89_core_skb_pn_valid` (core.c:3906-3970): for AX chips, HW-decrypted, A1-matched QoS data on a
TID with an RX BA session, CCMP/GCMP: parse PN from the IV; if PN > last PN but SN < last SN (and a
last PN exists), **drop** the frame; otherwise record (SN, PN). State reset on RX_START and on any
pairwise key install (core.c:3863-3904, mac80211.c:1007-1008). Not needed with SW crypto.

### 6.6 Minimal approach: software crypto

Returning -EOPNOTSUPP from `set_key` for every cipher (or not implementing `set_key`) is valid and is
exactly what rtw89 already does for TKIP on this chip:

* TX: WD SEC_TYPE/SEC_HW_ENC/SEC_CAM_IDX = 0. mac80211 builds IV, encrypts, appends MIC; TXPKT_SIZE
  = full protected MPDU. Nothing else in the WD changes (HDR_LLC_LEN may stay = hdrlen/2).
* RX: no key in the ADDR/SEC CAM → HW does not decrypt; frames arrive with HW_DEC=0 (SW_DEC likely 1)
  and must be passed up without RX_FLAG_DECRYPTED; mac80211 decrypts and checks PN.
  Evidence that HW forwards undecryptable protected frames: 8852B TKIP and 8852B unicast PMF frames
  are decrypted by mac80211 today.
* Keep `MFP_CAPABLE` (mac80211 does BIP in SW). No extra HW flag is required for SW crypto.
* Performance: every MPDU is encrypted/decrypted on the CPU (AES-NI makes CCMP/GCMP cheap, but it
  is still per-byte work). Additionally, mac80211 fast-xmit and fast-rx do not handle keys that are
  not uploaded to HW (mac80211 internals, not in this tree), so TX A-MSDU (which needs fast-xmit)
  is effectively disabled and all frames take the slow path → expect noticeably lower peak
  throughput and higher CPU than rtw89. HW crypto can be added later without changing the RX/TX
  descriptor handling beyond §6.2/§6.4.

---------------------------------------------------------------------------------------------------

## 7. TX status / ACK reporting

### 7.1 Release report (RPP) on PCIe

MAC init configures host release reports (`set_host_rpr_ax`, mac.c:4009-4027):
`R_AX_WDRLS_CFG` 0x9408 bits[1:0] = 0 (RPR mode POH, PCIe); `R_AX_RLSRPT0_CFG0` 0x9410 |= 0x0F000000
(FLTR_MAP [27:24]); `R_AX_RLSRPT0_CFG1` 0x9414 bits[7:0] = 30 (reports aggregated per RPQ buffer),
bits[23:16] = 255 (timeout) (reg.h:1605-1617).

Each RPQ RX buffer = rxbd_info + RX desc + N × 4-byte RPP (pci.c:644-695). RPP dword
(pci.h:1505-1513):

| Bits | Field |
|---|---|
| 31 | POLLUTED (ignored by rtw89) |
| [30:16] | SEQ = WD page index (0..511) from wp_info seq0 |
| [15:13] | TX_STATUS: 0 DONE, 1 RETRY_LIMIT, 2 LIFE_TIME, 3 MACID_DROP (core.h:3645-3648) |
| [12:8] | QSEL → DMA channel via `get_ch_dma` (pci.c:572-582) |
| [7:0] | MACID |

Processing (pci.c:596-625, 538-570, 501-520): reject txch ≥ 13 or masked channels and seq ≥ 512;
take WD page `seq` of that channel; if the page is still on the busy list, first reclaim TXBDs
(read HW index) — warn if still busy (not in low-power mode); for each skb on the page: DMA-unmap,
report status; return the page to the free list (zeroed) only when it is off the busy list. The
TXBD-reclaim side likewise frees a page only if its skb queue is empty (RPP already seen). RPQ is
serviced first in NAPI poll (pci.c:4513-4539), and also opportunistically when TX resources run out
(pci.c:1266-1318).

On stop/reset, every page still holding skbs is completed with MACID_DROP (pci.c:627-642).

### 7.2 Mapping to mac80211 (`rtw89_pci_tx_status`, pci.c:460-499)

1. If the skb belongs to a driver "tx wait" (§7.3), complete that and stop (skb not given to mac80211).
2. `ieee80211_tx_info_clear_status()` (rate `count`s become 0; no retry information is reported).
3. `IEEE80211_TX_CTL_NO_ACK` → set `IEEE80211_TX_STAT_NOACK_TRANSMITTED`.
4. TX_STATUS == 0 → `IEEE80211_TX_STAT_ACK`; 1/2/3 → no ACK (counters only; other values warn).
5. `ieee80211_tx_status_ni(hw, skb)` for every mac80211 frame (data and mgmt).

`IEEE80211_HW_REPORTS_TX_ACK_STATUS` is set (core.c:7391). No per-frame TX rate/retry is reported;
rate comes from the RA report (§7.6).

### 7.3 Driver "tx wait" (null-data with ACK wait)

Used for driver-generated QoS/non-QoS null frames (ROC start/end, after HW-scan, MCC;
core.c:4919-4984, 5016, 5075, 7016). A `rtw89_tx_wait_info` {completion, skb, tx_done} is attached via
RCU pointer in the skb's `driver_data` (core.h:3660-3673); `rtw89_core_tx_kick_off_and_wait`
(core.c:1309-1334) waits `timeout` ms (30 ms for scan/ROC). On RPP: `tx_done = (status==DONE)`,
`complete_all` (core.h:7973-7988). Return 0 = ACKed, -EAGAIN = not ACKed, -ETIMEDOUT = no RPP yet
(wait parked on a list and freed by a work 500 ms later). Not needed if mac80211 SW scan is used
(mac80211 sends its own PS nullfunc through `.tx` and gets status via §7.2).

### 7.4 USB-only FW TX report (not used on PCIe)

With `hci.tx_rpt_enabled` (USB only): WD info dw3 SPE_RPT=1, dw4 SW_DEFINE = 4-bit SN, dw1
TXCNT_LMT_SEL=1/TXCNT_LMT=8, and CMAC table MGQ_RPT_EN; status arrives as a C2H (mac.h:1784-1830,
fw.c:3592). PCIe leaves all of these 0.

### 7.5 Which frames need ACK status

With `REPORTS_TX_ACK_STATUS`, mac80211 requests status (`IEEE80211_TX_CTL_REQ_TX_STATUS`) for
auth / assoc / reassoc requests (fast failure detection instead of fixed timeouts), the nullfunc /
probe used by connection monitoring, `NL80211_CMD_FRAME` mgmt TX with an ACK cookie
(e.g. SAE, action frames), and control-port (EAPOL) TX status. Since every WD page gets an RPP on
PCIe, reporting status for *all* frames is free and is what rtw89 does. Without the flag mac80211
still works (timeout-based), so it is optional but recommended.

### 7.6 FW rate-adaptation report → `sta_statistics`

`HAS_RATE_CONTROL` is set (core.c:7389): FW does rate adaptation; data frames go out with USE_RATE=0.
FW periodically sends C2H category OUTSRC (2), class RA (1), func STS_RPT (0)
(fw.h:172-175, phy.h:160-168, 201-205; dispatch phy.c:3396-3404, 4082-4114). Payload after the 8-byte
C2H header (fw.h:3963-3977):

| Word | Bits | Field (AX) |
|---|---|---|
| w2 | [15:0] | MACID |
| w2 | [23:16] / 31 | RETRY_RATIO / MCSNSS bit7 — BE only |
| w3 | [6:0] | MCSNSS: legacy idx; HT MCS in [4:0] (8852B FW is not OLD_HT_RA_FORMAT); VHT/HE: MCS [3:0], NSS−1 [6:4] |
| w3 | [9:8] | MD_SEL: 0 legacy, 1 HT, 2 VHT, 3 HE |
| w3 | [12:10] | GILTF (§4.8; HT/VHT non-zero = SGI) |
| w3 | [14:13] | BW (0..3) |

`__rtw89_phy_c2h_ra_rpt_iter` (phy.c:3229-3349) converts it to `struct rate_info` (legacy bitrate
table, MCS/VHT_MCS/HE_MCS flags, nss, HE GI, bw), stores `ra_report` {txrate, bit_rate, hw_rate,
`might_fallback_legacy` = mcs ≤ 2} and updates `max_rc_amsdu_len` (§5.3). `.sta_statistics` returns
`sinfo->txrate` + `NL80211_STA_INFO_TX_BITRATE` (mac80211.c:1092-1106). RX rate needs nothing:
mac80211 derives it from `rx_status`. (The RA H2C that enables FW RA per mac_id is the FW/PHY track.)

---------------------------------------------------------------------------------------------------

## 8. mac80211 TX entry, scheduling, flow control

### 8.1 Entry points

* `.tx` (mac80211.c:19-46): frames mac80211 does not queue in TXQs (non-bufferable mgmt such as
  probe req, auth, assoc, deauth, most action frames; PS nullfunc). Write + kick immediately; **no
  resource pre-check** — if no TXBD/WD page is free, `rtw89_pci_tx_write` fails with -ENOSPC and the
  frame is freed with `ieee80211_free_txskb`. Frames arriving while the vif is off-channel (ROC) are
  parked in a per-sta queue (optional).
* `.wake_tx_queue` (mac80211.c:48-55): `ieee80211_schedule_txq()` + queue `txq_work` on a
  WQ_UNBOUND|WQ_HIGHPRI workqueue (core.c:6903).

### 8.2 `txq_work` (core.c:4789-4861)

For ac = 0..3: `ieee80211_txq_schedule_start(ac)`; for each `ieee80211_next_txq`:
1. skip (return with requeue) if vif off-channel;
2. `tx_resource` = `check_and_reclaim_tx_resource(ch_dma of TID)` = min(free TXBDs, free WD pages)
   after reclaiming (RPQ processing + TXBD index read) if either is 0 (pci.c:1266-1330);
3. `frame_cnt` from `ieee80211_txq_get_depth`; optional agg-wait (§5.3);
4. push `min(frame_cnt, tx_resource)` frames (`ieee80211_tx_dequeue_ni` → check_agg → tx_write);
5. `ieee80211_return_txq(hw, txq, force)`; kick the channel once if any frame was written;
6. if the resource bound was hit (`frame_cnt == tx_resource`) set `reinvoke`.
After all ACs, if `reinvoke`, re-run the work 1 jiffy later. There is **no** stop/wake of mac80211
queues on ring-full and TX completion does not re-trigger the work; back-pressure is purely
"push what fits, poll again next jiffy".

### 8.3 Sequence numbers

* Data: mac80211 assigns seq_ctrl (per-TID for QoS); HW uses it (HW_SSN_MODE=0); SW_SEQ mirrors it.
* Mgmt/nullfunc: HW_SSN_SEL=1, HW_SSN_MODE=1 → HW overwrites seq from its HW counter 1
  (`RTW89_MGMT_HW_SSN_SEL/SEQ_MODE`, core.h:1285-1287; same selection is used for FW-sent beacons,
  fw.c:4710-4711), keeping FW- and host-originated mgmt frames on one counter.

### 8.4 HE HT-Control insertion (optional, core.c:915-1041)

For HE peers, QoS data, not EAPOL/ARP/DHCP/ICMP, non-AP vif, headroom ≥ 4, and RA report not near
legacy: insert a 4-byte HT-Control after the QoS field (hdr 26→30), set FC Order bit, set QoS-ctrl
bit 4, HTC = per-sta template (2.4 GHz 40 MHz: OM control disabling UL-MU) or HE variant CAS
(`0x3 | 6<<2`); `pkt_size += 4`; A_CTRL_BSR=1. Needs `hw->extra_tx_headroom` = 4 (core.c:7365-7377).
BK is set whenever the A-ctrl state differs from the previous frame of the vif. A minimal driver can
skip this entirely (A_CTRL_BSR=0, BK only for EAPOL).

### 8.5 Power-save-related TX bits that a minimal (PS-off) driver can ignore

* `rtw89_core_tx_wake` → `rtw89_mac_notify_wake` only when in low-power mode (FW TX_WAKE feature)
  (core.c:1158-1187).
* WD MORE_DATA, HIQ/MULTIPORT_ID, `SEND_AFTER_DTIM` → B0_HI/CH9 (AP only).
* `rtw89_core_rx_pkt_hdl` PS-poll/U-APSD (AP only).
* `SUPPORTS_PS`, `SUPPORTS_DYNAMIC_PS` hw flags (don't set them if PS is off), `rtw89_leave_ps_mode`
  calls in ops, BTC special-packet notifications (EAPOL/ARP/DHCP/ICMP → coex work, core.c:1120-1156).

### 8.6 Flush

`.flush` (mac80211.c:1122-1136): per non-CH12 channel poll `TXBD_HW_IDX` (idx register [27:16]) ==
host wp, 60 × 1 µs (pci.c:1388-1436); then poll `R_AX_DLE_EMPTY0` **0x8430** until
`(val & 0x07FF079F) == 0x07FF079F` every 10 ms up to 200 ms (mac.c:2388-2412, 6351-6363,
reg.h:629-649). With drop and FW support, a FW "pkt drop" H2C per vif is used instead.

---------------------------------------------------------------------------------------------------

## 9. Minimal station-mode checklist

Mandatory:
1. WD v0 builder exactly as §1.3/§1.4 with the per-type values of §1.5 (zero the whole WD first).
2. wp_info/addr_info + TXBD as §1.2; H2C WD as §1.6.
3. Queue map §2.1 (ACH0-3, CH8, CH12), MACID = vif mac_id (§2.2).
4. RX desc parser §3.2 (+SHIFT), dispatch §3.3 (WIFI, PPDU_STAT, C2H; RPQ as RPP list).
5. RPP handling + `ieee80211_tx_status_ni` for every frame (§7.1-7.2); WD page recycling rule.
6. `rx_status`: freq/band, rate/encoding/nss/bw/GI, FCS flag, DECRYPTED (if HW crypto), MACTIME,
   and **signal via PPDU status** (§4) — without it every frame is NO_SIGNAL_VAL and cfg80211 BSS
   selection has no RSSI.
7. hw flags: SIGNAL_DBM, HAS_RATE_CONTROL, RX_INCLUDES_FCS, AMPDU_AGGREGATION,
   REPORTS_TX_ACK_STATUS, MFP_CAPABLE; `hw->queues`=4.
8. ampdu_action: TX_START → IMMEDIATE, TX_STOP_* → `ieee80211_stop_tx_ba_cb_irqsafe`,
   TX_OPERATIONAL → record buf_size; RX_START/STOP → return 0 (BA CAM H2C optional).
9. Driver-initiated `ieee80211_start_tx_ba_session` (mac80211 does not start TX BA by itself).

Optional / later: HW crypto (§6.1-6.5), TX A-MSDU (needs fast-xmit → HW crypto, plus
`max_rc_amsdu_len` from RA report), RA report → `sta_statistics`, HTC insertion, agg-wait,
static BA CAM H2C, agg-limit register, rx_freq_from_ie, radiotap/monitor, tx-wait nullfunc,
RSSI/CFO/EVM EWMAs and CFO tracking.

---------------------------------------------------------------------------------------------------

## 10. Open questions / uncertainties

1. **A1_MATCH on PPDU-status descriptors for broadcast frames**: rtw89 parses PHY-status RSSI
   (`rpl_avg`) only when `to_self`. If A1_MATCH is 0 for beacons/probe responses from other BSSes,
   the rpl→rssi conversion (§4.4) produces wrong per-path RSSI for scan results. Behaviour must be
   verified on hardware (log raw header RSSI vs converted values while scanning).
2. Whether PPDU-status packets use short or long RX descriptors (rtw89 reads `mac_id` from the
   persistent, possibly stale, long-desc field). Log LONG_RXD for RPKT_TYPE 1.
3. Semantics of TX_STATUS=0 for frames with `IEEE80211_TX_CTL_NO_ACK` / broadcast (assumed "sent").
4. `SW_DEC` exact meaning (assumed: protected frame not decrypted by HW) and whether HW ever drops
   protected frames with no matching key (evidence says no).
5. `R_AX_CUT_AMSDU_CTRL` = 0x010E05F0 bit meanings (rtw89 treats RX A-MSDUs as whole frames, so cut
   is presumably disabled).
6. Whether the HW actually honours DATA_STBC/DATA_LDPC when USE_RATE=0 (rtw89 sets them anyway).
7. `IEEE80211_TX_CTL_NO_CCK_RATE` on 2.4 GHz with rtw89's "OFDM6 + ffs(basic_rates)" formula can pick
   a higher rate than intended (index base mismatch); a new driver should map the lowest *OFDM* basic
   rate explicitly.
8. RLSRPT0 timeout unit (255) and FLTR_MAP meaning are not documented in the source.
9. RX desc SHIFT is never parsed by rtw89 on AX; assumed 0 on this configuration.
