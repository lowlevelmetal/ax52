# 06 — RF calibration (RFK) for RTL8852B (PCIe, CV_B, RFE 1)

Track 06 of the specification. Scope: everything rtw89 does to calibrate the RTL8852B analog front end.
That covers RCK, DACK (with AFE init, DRCK and ADDCK), RX DCK, IQK (LOK/TXK/RXK), DPK
(with DPD back-off init and DPK tracking), TSSI (full, scan and scan-end variants, PMAC
alignment, efuse DE and the thermal table), and the LCK/synth-lock check done during
channel set. It also covers when each one runs, the NCTL/KIP engine, the coex/FW handshake
and thermal readout.

Source root: `reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/`.
Citation shorthands used below:

| short | file |
|---|---|
| `rfk.c` | `rtw8852b_rfk.c` |
| `tbl.c` | `rtw8852b_rfk_table.c` |
| `8852b.c` | `rtw8852b.c` |
| `cmn.c` / `cmn.h` | `rtw8852b_common.c` / `.h` |
| `table.c` | `rtw8852b_table.c` (BB/RF/NCTL/txpwr tables) |
| `phy.c`, `core.c`, `coex.c`, `mac.c`, `fw.c`, `reg.h` | same names |

---------------------------------------------------------------------------------------------

## 0. Conventions and register access (read this first)

### 0.1 Address spaces

* **BB ("phy") register `BB 0xNNNN`**: the MMIO BAR offset is `0x10000 + 0xNNNN`. rtw89's
  `rtw89_phy_{read,write}32*` add `phy_def->cr_base = 0x10000` for all AX chips
  (phy.h:714-801, phy.c:8970). Every address in `rfk.c`/`tbl.c` passed to
  `rtw89_phy_*` or through an `RTW89_DECL_RFK_WM` table entry is a BB address.
  Example: NCTL command register `BB 0x8000` is at BAR+0x18000.
* **MAC register `MAC 0xNNNN`**: raw BAR offset (`rtw89_write32`). RFK only touches a few
  of these: `0x8040` (R_AX_PHYREG_SET), `0x00AC` (scoreboard), `0x0073` (ctrl path),
  `0xDAF0/0xDAF4` (LTE indirect, for GNT), `0xC348` (CTN_TXEN, via FW).
* **RF register `RF[p] 0xAA`**: 20-bit registers (`RFREG_MASK = 0xfffff`, core.h:45) per
  path p (A = 0 = "S0", B = 1 = "S1"). 8852B uses `read_rf = rtw89_phy_read_rf_v1` and
  `write_rf = rtw89_phy_write_rf_v1` (8852b.c chip ops). The accessor depends on bit 16 of
  the address (`RTW89_RF_ADDR_ADSEL_MASK = 0x10000`, phy.c:984-998, 1190-1204):
  * **bit16 = 1 ("D-die"/direct)**, e.g. `0x10001`, `0x10005`, `0x10018`, `0x10055`:
    direct BB window. BB address = `rf_base_addr[p] + (addr & 0xff) * 4`, with
    `rf_base_addr = {0xe000, 0xf000}` (8852b.c:994). So `RF[A] 0x10001` is `BB 0xe004`,
    which is BAR `0x1e004`. Reads are masked `read32 & mask`. Writes are RMW on that dword,
    followed by `udelay(1)` (phy.c:928-947, 1128-1151).
  * **bit16 = 0 ("A-die", serial SWSI)**, e.g. `0x00`, `0x18`, `0x92`:
    * Write (phy.c:1153-1188): poll until `BB 0x174c` bit24 (W busy) and bit25 (R busy)
      are both 0 (1 µs step, 30 µs timeout). If mask != 0xfffff, write
      `BB 0x374[19:0] = mask` and pre-shift the data by `ffs(mask)`. Then write
      `BB 0x370 = (mask_en<<31) | (path<<28) | ((addr&0xff)<<20) | (data & 0xfffff)`.
      The hardware performs the masked write.
    * Read (phy.c:950-982): same busy poll. Write `BB 0x378[10:0] = (path<<8) | addr`,
      `udelay(2)`, then poll `BB 0x174c` bit26 (data done; 1 µs / 30 µs). The data is in
      `BB 0x174c[19:0]` (masked with the caller's mask and shifted down).
* **Path-B BB offsets**: for most per-path BB blocks rtw89 derives the path-B address
  arithmetically:
  * `+0x2000` (`path << 13`): 0x12xx→0x32xx, 0x1cxx→0x3cxx, 0x56xx/0x58xx/0x5cxx→0x76xx/0x78xx/0x7cxx.
  * `+0x100` (`path << 8`): KIP block 0x81xx→0x82xx.
  * `+0x100`: AFE/DACK block 0xc0xx→0xc1xx.

  There are exceptions, called out where they occur.
* Every `write32_mask` in rtw89 is a read-modify-write, even with mask 0xffffffff
  (core.h:7093-7104). A plain write is equivalent unless the register has read side
  effects; none are known here.
* **CV**: the reference system is CV = 1 = `CHIP_CBV` (core.h:199-200: CHIP_CAV = 0, CHIP_CBV = 1).
  The only CV-dependent RFK step is in DPK KIP restore (§3.7.9).
* **DBCC** is never enabled for 8852B station use, so `_kpath()` always returns
  `RF_AB = 3` (both paths) (rfk.c:277-293). Every "if kpath == RF_A/RF_B" branch is dead.
  Every "non-dbcc path01" branch is live.
* **RFK channel table index `table_idx`**: always 0 for single-channel operation. It is
  computed by `rtw8852b_mcc_get_ch_info` → `rtw89_rfk_chan_lookup` (rfk.c:4168-4193,
  phy.c:8309-8336). A slot is picked that is not used by any active channel, and the
  active channel is already the new one, so slot 0 is always chosen. It only selects
  IQK coefficient bank 0 or 1 (§3.6.2). DPK uses `cur_idx = 0` because reload is disabled
  (rfk.c:2506-2507, 3801).

### 0.2 NCTL / KIP calibration engine

IQK and DPK run as microcode on an on-chip engine: "NCTL" (the controller) and "KIP" (the
calibration datapath). It lives in BB 0x8000–0xa7ff. The driver only configures RF/BB
state, issues one-shot commands and reads reports.

**Initialisation**: `rtw89_phy_init_rf_nctl`, called from `rtw89_phy_dm_init` just before
`rfk_init` (phy.c:8163-8189, 2065-2078). It must happen after BB/RF table load.
1. Pre-init (`rtw89_phy_preinit_rf_nctl_ax`, phy.c:2041-2063). All addresses are BB:
   1. `0x0c60 |= 0x3` (R_IOQ_IQK_DPK: IQK/DPK clock and reset).
   2. `0x0c6c |= 0x1` (R_GNT_BT_WGT_EN).
   3. `0x58ac |= BIT(27)`, `0x78ac |= BIT(27)` (R_P0/P1_PATH_RST).
   4. For 8852B: `0x0c60 |= 0x2` again (redundant).
   5. `0x8000 = 0x00000008` (R_NCTL_CFG idle value).
   6. Poll NCTL alive: repeat { write `0x8080 = 0x4`, `udelay(1)`, read `0x8080` } until
      the read returns `0x4`. Interval 10 µs, timeout 1 ms. On timeout: "failed to poll
      nctl block" (error only).
2. Load the NCTL table as plain `(addr, data)` 32-bit BB writes (`rtw89_phy_config_bb_reg`).
   Source: `elm_info->rf_nctl` (the **"NCTL" element of rtw8852b_fw-2.bin**, which is
   present on the reference system), else the static `rtw89_8852b_phy_nctl_table` (table.c:13251-14572,
   8852b.c:1002).
   * The static fallback has 1320 writes spanning 0x8000–0x8cff, 0x8d00–0x94ff, 0x9f04–0x9f2c
     and 0xa200–0xa7ff.
   * It starts `0x8000=0x8, 0x8008=0, 0x8004=0xf0862966, …` and ends
     `0x8080=4, 0x8080=0, 0x8088=0`.
   * It also sets the "no calibration" defaults that apply if IQK/DPK are never run:
     `0x8138 = 0x813c = 0x8238 = 0x823c = 0x40000000` (TX/RX IQC for A/B),
     `0x81bc = 0x81c0 = 0x005b5b5b` (DPD config, bit24 = DPD enable = 0),
     `0x81ac = 0x81b0 = 0x003f2e2e`, `0x81b4 = 0x81b8 = 0x00600060`, `0x81dc = 0x2`.
   * `nctl_post_table` is NULL for 8852B.

**Key NCTL/KIP registers** (all BB):

| addr | rtw89 name | use |
|---|---|---|
| 0x8000 | R_NCTL_CFG | command word (trigger); idle 0x8 |
| 0x8008 | R_NCTL_RPT | bit26 `B_NCTL_RPT_FLG` = IQK fail flag; written 0x80 at IQK/DPK preset, 0 at restore |
| 0x8010 | R_NCTL_N1 | [7:0] cleared to 0 after every one-shot |
| 0x802c | R_IQK_DIF4 | [11:0] TX tone/"TXT", [27:16] RX tone/"RXT" |
| 0x8070 | R_MDPK_SYNC | [31:28] MAN, bit31 SEL |
| 0x8074 | R_MDPK_RX_DCK | bit31 EN (LBK RXIQK) |
| 0x8078 | R_KIP_MOD | [19:0] copy of RF 0x00 for DPK RXAGC |
| 0x8080 | — | NCTL alive probe (write 4, read back 4) |
| 0x8088 | R_KIP_SYSCFG | 0x81ff010a (IQK), 0x807f030a (DPK), 0x80000000 (idle/restore) |
| 0x80a0 | R_LDL_NORM | [1:0] MDPD order, [12:8] PN |
| 0x80bc / 0x80c0 | R_DPK_CFG2 / 3 | bit14 ST; [23:16], [31:24] PAS report selectors |
| 0x80d0 | R_KPATH_CFG | [21:20] |
| 0x80d4 | R_KIP_RPT1 | [21:16] (or [19:16]) report page select |
| 0x80f0 | R_DPK_TRK | bit31 DPK-track disable |
| 0x80fc | R_RPT_COM | report data window |
| 0x8104 (+0x100) | R_COEF_SEL | bit0 IQC bank, bit8 MDPD bank |
| 0x8120 (+0x100) | R_CFIR_SYS | 0xce000a08 = KIP power/clock on for DPK |
| 0x8124 (+0x100) | R_IQK_RES | [3:0] RX CFIR select, [11:8] TX CFIR select (5 = use WB result, 0 = none) |
| 0x8138 (+0x100) | R_TXIQC | TX IQ correction (0x40000000 = identity) |
| 0x813c (+0x100) | R_RXIQC | RX IQ correction; bit0/bit2 = bypass bits used by DPK |
| 0x8154 (+0x100) | R_CFIR_LUT | bit8 SEL, bit4 SET (1 = TX, 0 = RX), bit3 G3 (= bank idx), bit2 G2, [1:0]/[2:0] group |
| 0x81a0 (+0x100) | R_DPD_V1 | cleared at DPK fill |
| 0x81ac (+0x100) | R_DPD_CH0 | [31:24] DPK table select |
| 0x81b4 (+0x100, +4·kidx) | R_DPD_BND | [8:0] / [24:16] power scaling factor ("pwsf") |
| 0x81bc (+0x100, +4·kidx) | R_DPD_CH0A | [22:0] gain scale, [26:25] order, bit24 DPD enable |
| 0x81c4 (+0x100) | R_TXAGC_RFK | [13:8] DPK TX AGC (gain 1, kidx 0) |
| 0x81c8 (+0x100) | R_DPD_COM | bit15 "HW TX AGC offset mode" |
| 0x81cc (+0x100) | R_KIP_IQP | IQK tone power / "itqt" |
| 0x81dc (+0x100) | R_LOAD_COEF | bit1 DI, bit16 MDPD load pulse |
| 0x8220 | R_IQRSN | bit28 K1, bit16 K2 |
| 0x9fe0 / 0x9fe4 / 0x9fe8 | R_IQKINF / R_IQKCH / R_IQKINF2 | informational scratch (IQK version, band/bw/ch, fail bits, counters). Nothing reads them back functionally. Optional. |
| 0xbff8 | — | [7:0] == 0x55 means one-shot done |

**One-shot protocol (IQK)**: `_iqk_one_shot` + `_iqk_check_cal`, rfk.c:815-868, 253-275.
1. Optionally set `BB 0x5864 bit29` (B_P0_RFCTM_EN) per op (table below). This is always
   the path-0 register, even for path B.
2. Write `BB 0x8000 = cmd` (the code computes `base` and writes `base + 1`, so the low
   nibble is always 0x9).
3. `udelay(1)`.
4. Poll `BB 0xbff8[7:0] == 0x55`, 1 µs step, **8200 µs** timeout.
5. `udelay(200)`.
6. If there was no timeout: `fail = BB 0x8008 bit26`. On timeout, fail stays true.
7. `BB 0x8010[7:0] = 0`.
8. `BB 0x5864 bit29 = 0`.

**IQK command words** (path A / path B). Layout: `[11:8] = op`, `bit(4+path) = path`,
`[3:0] = 0x9`.

| op | ID | cmd A / B | 0x5864 b29 before | extra |
|---|---|---|---|---|
| FLOK coarse | 0x1 | 0x119 / 0x129 | 1 | |
| FLOK fine | 0x2 | 0x219 / 0x229 | 1 | |
| FLOK vbuffer | 0x3 | 0x319 / 0x329 | 1 | |
| NB TXK | 0x4 | 0x419 / 0x429 | 0 | 0x802c[11:0]=0x011 (unused: `is_nbiqk=false`) |
| RXAGC | 0x5 | 0x519 / 0x52b | unchanged | unused on 8852B |
| NB RXK | 0x6 | 0x619 / 0x629 | 1 | 0x802c[27:16]=0x011 (unused) |
| WB TXK | 0x8+bw | 20M 0x819/0x829, 40M 0x919/0x929, 80M 0xa19/0xa29 | 0 | |
| WB RXK | 0xb+bw | 20M 0xb19/0xb29, 40M 0xc19/0xc29, 80M 0xd19/0xd29 | 1 | |
| IQK restore | 0xe | 0xe19 / 0xe29 | — | written directly (rfk.c:1448) |

(bw: 0 = 20, 1 = 40, 2 = 80 MHz, the `RTW89_CHANNEL_WIDTH_*` values.)

**One-shot protocol (DPK)**: `_dpk_one_shot`, rfk.c:1702-1742.
1. `BB 0x8000 = (id << 8) | (path ? 0x29 : 0x19)`.
2. Poll `BB 0xbff8[7:0] == 0x55`, 1 µs step, **20000 µs** timeout.
3. `udelay(1)`.
4. `BB 0x80d4 = 0x00030000` (report page 3).
5. Poll `BB 0x80fc[15:0] == 0x8000`, 1 µs step, 2000 µs timeout (KIP idle).
6. `BB 0x8010[7:0] = 0`.

On timeout rtw89 only logs. There is no fail flag for DPK; success is judged from reports.

DPK IDs used by 8852B (rfk.c:35-55): LBK_RXIQK 0x06, SYNC 0x10, MDPK_IDL 0x11, GAIN_LOSS 0x13,
DPK_RXAGC 0x15, KIP_PRESET 0x16, DPK_TXAGC 0x19. Defined but unused: MDPK_MPA 0x12,
GAIN_CAL 0x14, KIP_RESTORE 0x17, D_* 0x28–0x31.

### 0.3 Common helpers used by several calibrations

* **Backup lists** (rfk.c:113-116, 177-233):
  * BB `{0x2344, 0x5800, 0x7800}` (full dwords).
  * RF (per path, full 20 bits) `{0xde, 0xdf, 0x8b, 0x90, 0x97, 0x85, 0x1e, 0x00, 0x02, 0x05, 0x10005}`.
  * Restore writes them back in the same order.
* **RF direct control**: `RF[p] 0x05 bit0` (RR_RSV1_RST) selects 1 = RF driven by BB,
  0 = software/RFK direct control. `RF[p] 0x10005 bit0` (RR_BBDC_SEL) is the D-die
  equivalent (rfk.c:235-251). RFK code clears these at start and sets them back to 1 at
  the end.
* **Wait for RX mode**: `_wait_rx_mode` (rfk.c:1569-1585). For each path, poll
  `RF[p] 0x00[19:16] != 2` (not TX), 2 µs step, 5000 µs timeout. Logged only.
  `_tmac_tx_pause(…, true)` is the same thing; `(…, false)` is a no-op (rfk.c:1587-1594).
* **Stop/resume MAC TX**: `rtw89_chip_stop_sch_tx(phy, &tx_en, RTW89_SCH_TX_SEL_ALL)` →
  `rtw89_mac_stop_sch_tx` (mac.c:3403-3441).
  * It saves `MAC 0xC348[15:0]` (R_AX_CTN_TXEN) and sets its enable bits (mask 0xffff) to 0.
  * Once FW is running (`RTW89_FLAG_FW_RDY`) the change is made by FW through an
    **H2C-register message**: func `SCH_TX_EN` = 5, `w0[31:16] = tx_en`,
    `w1[15:0] = mask`, `w1 bit16 = band`. The driver expects C2H-reg reply
    `TX_PAUSE_RPT` = 4 (mac.c:3339-3383, fw.h:145-164).
  * Resume writes back the saved value with mask 0xffff.
  * The H2C-reg mechanism is described in the FW track.

---------------------------------------------------------------------------------------------

## 1. When each calibration runs (8852B, exact order)

`chip_ops` (8852b.c:888-899): `rfk_hw_init = NULL`, `rfk_init = rtw8852b_rfk_init`,
`rfk_init_late = NULL`, `rfk_channel = rtw8852b_rfk_channel`,
`rfk_band_changed = rtw8852b_rfk_band_changed`, `rfk_scan = rtw8852b_rfk_scan`,
`rfk_track = rtw8852b_rfk_track`, `get_thermal = rtw8852bx_get_thermal`.

### 1.1 Power-on / interface start (`rtw89_core_start` → `rtw89_phy_dm_init`)

Order, from core.c:6565-6604 and phy.c:8163-8189:

1. MAC init, FW download, BB table, **RF radio tables** (`rtw89_phy_init_rf_reg`, core.c:6600).
2. `rtw89_btc_ntfy_init` (coex init; core.c:6602).
3. `rtw89_phy_dm_init`:
   1. `rtw89_phy_stat_init`: EWMA thermal init plus one thermal read per path. This is the
      RF-trigger method, since TSSI is not yet on.
   2. `bb_sethw` and other DM init (not RFK).
   3. `rtw89_phy_init_rf_nctl` (§0.2): **NCTL pre-init and NCTL microcode table**.
   4. **`rtw8852b_rfk_init`** (8852b.c:697-709):
      1. `is_tssi_mode[A] = is_tssi_mode[B] = false`; clear the RFK-MCC table.
      2. `rtw8852b_dpk_init` → `_set_dpd_backoff(PHY0)` (§3.7.1).
      3. `rtw8852b_rck` → `_rck(A)`, then `_rck(B)` (§3.4).
      4. `rtw8852b_dack(CHANCTX_0)` → coex `DACK START`, `_dac_cal`, coex `DACK STOP` (§3.1-3.3).
      5. `rtw8852b_rx_dck(PHY0, CHANCTX_0)` (§3.5). It runs on whatever channel the RF
         tables left programmed. The default chandef is used for coex band info only.
   5. `set_txpwr_ctrl`, then **`power_trim`**: thermal-meter trim into RF 0x43 and PA-bias
      trim into RF 0x60 (§6.3). This happens *after* rfk_init.
   6. `cfg_txrx_path`.
4. Later in core_start: `rfk_init_late` (NULL) and `btc_ntfy_radio_state(WL_ON)`
   (core.c:6624-6625).

### 1.2 Every channel set (`__rtw89_set_channel`, core.c:532-561)

1. `set_channel_help(enter)` (8852b.c:675-695). It stops MAC TX, disables PPDU status,
   pauses TSSI (`tssi_cont_en(false)`: `BB 0x58dc/0x78dc bit30 = 1` and
   `BB 0x5818/0x7818 bit30 = 1`), holds the ADC FIFO in reset (`BB 0x20fc[31:24] = 0xf`,
   R_ADC_FIFO/B_ADC_FIFO_RST, reg.h:9263-9264; the IQK/DPK tables' `0x20fc[31:16]` writes
   touch the same reset field) and resets the BB.
2. `set_channel` → mac, bb, then `rtw8852b_set_channel_rf`. This includes the
   **LCK / synth lock check** on path A (§3.9).
3. `set_txpwr`.
4. `set_channel_help(exit)`: reverses step 1. TSSI tracking is re-enabled by writing 0 to
   those two bits (8852b.c:638-651).
5. **If the entity was inactive (first set_channel) or the band changed (2G↔5G):**
   `btc_ntfy_switch_band`, then **`rfk_band_changed` → `rtw8852b_tssi_scan`** (§3.8.3).
6. If a pure monitor vif exists: `rfk_channel` (full per-channel RFK, below).

No IQK/DPK/RX-DCK runs on a plain channel switch. With FW scan offload (8852B FW ≥ 0.29.29.0
has `SCAN_OFFLOAD`, fw.c:874) the FW changes channels during scan by itself and the driver
does not even run `__rtw89_set_channel` per scan channel.

### 1.3 Per-channel RFK (`rfk_channel`)

Callers:
* Station: `rtw89_core_sta_link_add` when mac80211 moves the AP's sta NOTEXIST→NONE
  (before auth), for `NL80211_IFTYPE_STATION` (core.c:5678-5686, mac80211.c:488-532).
* AP start (mac80211.c:859).
* Monitor-only channel set (core.c:560).
* MLO link switch paths (only for `WITH_RFK_PRE_NOTIFY` FW, never on 8852B).

For AX chips `rtw89_chip_rfk_channel` directly calls the op (core.c:489-513).
`rtw8852b_rfk_channel` (8852b.c:711-729) does, in order:
1. `rtw8852b_mcc_get_ch_info(phy)`: sets `rfk_mcc.ch/band[idx]` and `table_idx` (= 0).
2. `rtw89_btc_ntfy_conn_rfk(true)` (a coex flag only).
3. **`rtw8852b_rx_dck`** (§3.5).
4. **`rtw8852b_iqk`** (§3.6).
5. `rtw89_btc_ntfy_preserve_bt_time(30)`: `fsleep(30 ms)` only if BT A2DP exists (coex.c:11887-11900).
6. **`rtw8852b_tssi(phy, hwtx_en = true)`** (§3.8.1). This includes PMAC over-the-air TX
   for alignment.
7. `preserve_bt_time(30)` again.
8. **`rtw8852b_dpk`** (§3.7).
9. `rtw89_btc_ntfy_conn_rfk(false)`.
10. **`rtw89_fw_h2c_rf_ntfy_mcc`**: H2C informing FW of the RFK channel slots (§5.3).

### 1.4 Scan start / end (`rfk_scan`)

Called from `rtw89_core_scan_start` / `rtw89_core_scan_complete` (core.c:6980, 7009). This
covers both SW and HW (FW-offload) scans. `rtw8852b_wifi_scan_notify` → `rtw8852b_tssi_default_txagc`
(rfk.c:3891-3941):
* **Scan start**: if neither path is in TSSI mode, run the full `rtw8852b_tssi(hwtx_en=true)`
  on the current channel. Otherwise nothing. Normally a no-op, because band-change TSSI
  already enabled TSSI.
* **Scan end**: §3.8.4. Reset the TSSI TX-AGC offsets to 0xc0 and re-apply the stored
  per-band alignment.

### 1.5 Periodic (`rfk_track`)

`rtw89_track_work` runs every `round_jiffies_relative(2 s)` (core.h:51, core.c:5424-5465).
It returns early when `rtwdev->scanning`. In order it calls:
1. `rtw89_phy_stat_track` → thermal update: `get_thermal` per path → EWMA (§6).
2. …DIG etc…
3. `rtw89_core_rfk_track` (skipped in MCC mode, core.c:5268-5277) → `rtw8852b_rfk_track`
   → **`_dpk_track`** only (8852b.c:746-749, §3.7.10).

On 8852B there is **no periodic TSSI software track**: TSSI thermal compensation is done
in hardware from the thermal-offset table programmed in §3.8.2. There is **no thermal-triggered
LCK and no IQK re-run**. Unlike 8852C, 8852B has no `_lck_track`.

### 1.6 Summary

| calibration | init | channel set | band change | rfk_channel | scan start | scan end | 2 s track |
|---|---|---|---|---|---|---|---|
| DPD back-off init | ✔ | | | | | | |
| RCK (A,B) | ✔ | | | | | | |
| DACK (+AFE init, DRCK, ADDCK) | ✔ | | | | | | |
| RX DCK | ✔ | | | ✔ (1st) | | | |
| LCK / synth-lock check | | ✔ (path A) | | | | | |
| IQK (LOK+TXK+RXK, A then B) | | | | ✔ (2nd) | | | |
| TSSI full (tables + PMAC alignment) | | | | ✔ (3rd) | only if TSSI off | | |
| TSSI "scan" (subset, stored/default alignment) | | | ✔ | | | | |
| TSSI default TX AGC + stored alignment | | | | | | ✔ | |
| DPK (A,B) | | | | ✔ (4th) | | | |
| DPK track | | | | | | | ✔ |
| thermal read | ✔ (stat init) | | | (inside DPK) | | | ✔ |
| thermal trim / PA-bias trim | ✔ (after rfk_init) | | | | | | |

---------------------------------------------------------------------------------------------

## 2. RFK tables (`rtw8852b_rfk_table.c`): format and inventory

### 2.1 Format

Each table is an array of `struct rtw89_reg5_def {u8 flag; u8 path; u32 addr; u32 mask; u32 data;}`
(core.h:4215), wrapped in `struct rtw89_rfk_tbl {defs, size}` (phy.h:917-975).
`rtw89_rfk_parser` walks the entries in order (phy.c:8339-8386):

| flag | value | action |
|---|---|---|
| `RTW89_RFK_F_WRF` | 0 | `write_rf(path, addr, mask, data)` |
| `RTW89_RFK_F_WM` | 1 | BB `write32_mask(addr, mask, data)` (data is the unshifted field value) |
| `RTW89_RFK_F_WS` | 2 | BB `write32_set(addr, mask)` |
| `RTW89_RFK_F_WC` | 3 | BB `write32_clr(addr, mask)` |
| `RTW89_RFK_F_DELAY` | 4 | `udelay(data)` |

8852B tables only use WM, WRF and DELAY. There is no WS or WC entry. Port them as
`{op, path, addr, mask, value}` data. Path-B tables are the path-A tables shifted by
`+0x2000` or `+0x100` with identical values. This was verified programmatically. The single
exception is `tssi_dac_gain_b`, which lacks the first A entry (`0x58b0 bit10 = 1`). That
entry is immediately overwritten by `0x58b0[11:0] = 0` anyway, so the net effect is identical.

### 2.2 Inventory

| table (tbl.c lines) | n | when applied | content |
|---|---|---|---|
| `afe_init` (7-18) | 10 | DACK start | AFE analog config for both paths (listed in §3.1) |
| `check_addc_a/_b` (22-40) | 6 | ADDCK debug check | debug-port select so ADC DC can be read at 0x1730 |
| `check_dadc_en_a/_b` (44-64) | 7 | DACK S1 debug check (B only) | DADC loopback enable (includes one RF write) |
| `check_dadc_dis_a/_b` (68-82) | 4 | same | DADC loopback disable |
| `dack_s0_1/2/3`, `dack_s1_1/2/3` (86-165) | 18/3/5, 18/4/5 | DACK | MSBK and DADCK trigger/cleanup (§3.3) |
| `dpk_afe` (169-199) | 29 | DPK start | BB/AFE into calibration mode (§3.7.3) |
| `dpk_afe_restore` (203-228) | 24 | DPK end | undo |
| `dpk_kip` (232-235) | 2 | DPK end, per path | `0x8008 = 0`, `0x8088 = 0x80000000` |
| `tssi_sys` (239-257) | 17 | TSSI full and scan | TSSI system config (shared) |
| `tssi_sys_{a,b}_{2g,5g}` (261-293) | 4 each | TSSI full and scan | per path/band |
| `tssi_init_txpwr_{a,b}` (297-381) | 39 each | TSSI full only | TSSI/TX-power engine init |
| `tssi_init_txpwr_he_tb_{a,b}` (385-395) | 2 each | TSSI full only | HE-TB |
| `tssi_dck_{a,b}` (399-411) | 3 each | TSSI full only | TSSI DC |
| `tssi_dac_gain_{a,b}` (415-524) | 52/51 | TSSI full only | zero the DAC-gain table 0x5a00–0x5ac0 |
| `tssi_slope_{a,b}_{2g,5g}` (528-592) | 12 each | TSSI full only | slope cal ("org") |
| `tssi_align_{a,b}_{2g,5g1,5g2,5g3}_all` (596-767) | 9 each | TSSI full and scan (when no stored alignment) | default alignment |
| `tssi_align_*_part` | 4 each | **never used on 8852B** (always called with `all = true`) | — |
| `tssi_slope_defs_{a,b}` (780-792) | 3 each | TSSI full only | enable slope |

Full contents of the calibration-critical tables are in the algorithm sections below and in
Appendix A. The TSSI tables are summarised in Appendix A with enough detail to port them.
Copy the exact values from tbl.c by script: they are pure data.

---------------------------------------------------------------------------------------------

## 3. Algorithms

### 3.1 AFE init (first part of DACK)

`_afe_init` (rfk.c:371-376):
1. **MAC** `0x8040` (R_AX_PHYREG_SET) `= 0x0000000f` as a 32-bit write. `enable_bb_rf`
   earlier wrote 8-bit 0x0E here (cmn.c:2031).
2. `afe_init` table (tbl.c:7-18). All entries are full-dword BB writes:

| BB | value | | BB | value |
|---|---|---|---|---|
| 0xc0d4 | 0x4486888c | | 0xc1d4 | 0x4486888c |
| 0xc0d8 | 0xc6ba10e0 | | 0xc1d8 | 0xc6ba10e0 |
| 0xc0dc | 0x30c52868 | | 0xc1dc | 0x30c52868 |
| 0xc0e0 | 0x05008128 | | 0xc1e0 | 0x05008128 |
| 0xc0e4 | 0x0000272b | | 0xc1e4 | 0x0000272b |

This is configuration rather than calibration. It is **mandatory** for a working AFE,
because nothing else writes these values.

### 3.2 DRCK (D-die RC calibration) and ADDCK (ADC DC calibration)

**DRCK** `_drck` (rfk.c:378-402). BB registers:
1. `0xc0cc bit6 = 1` (kick).
2. Poll `0xc0d0 bit3 == 1`, 1 µs step, 10 ms timeout. Logged on timeout.
3. `0xc0cc bit6 = 0`.
4. `0xc094 bit9 = 1`, `udelay(1)`, `0xc094 bit9 = 0` (latch).
5. `rck_d = 0xc0d0[19:15]`.
6. `0xc0cc bit9 = 0`.
7. `0xc0cc[4:0] = rck_d`.

Only one instance exists (0xc0xx). It serves both paths.

**ADDCK** `_addck` (rfk.c:511-586). BB registers throughout. `0x030c` and `0x032c` are shared
"ANAPAR" registers.

*S0 (path A):*
1. `0xc0f4[5:4] = 0` (manual-mode off).
2. `0xc1d4[5:4] = 0`. Note: this is the path-1 register even in the S0 section.
3. `0x12b8 bit30 = 1` (path-0 debug/NRBW).
4. `0x032c bit30 = 0` (ADC clock off).
5. `0x032c bit22 = 0`, then `= 1` (filter reset).
6. `0x030c[27:24] = 0xf`.
7. `0x032c bit16 = 0`.
8. `0xc0d4 bit1 = 1`.
9. `0x030c[27:24] = 0x3`.
10. `_check_addc(A)` (debug: table `check_addc_a`, then average 100 reads of `0x1730`).
    Result only printed. Side effect: debug-port select regs 0x20f0/0x20f4/0x20f8.
11. `0xc0f4 bit11 = 1`, then `0` (trigger); `udelay(1)`.
12. `0xc0f4[9:8] = 1`.
13. Poll `0xc0fc bit0 == 1`, 1 µs step, 10 ms timeout. On timeout set `addck_timeout[0]` (log only).
14. `_check_addc(A)` again (debug).
15. `0xc0d4 bit1 = 0`; `0x032c bit16 = 1`; `0x030c[27:24] = 0xc`; `0x032c bit30 = 1`; `0x12b8 bit30 = 0`.

*S1 (path B)*: same shape.
1. `0x32b8 bit30 = 1`.
2. `0x032c bit30 = 0`; `0x032c bit22` 0→1; `0x030c[27:24] = 0xf`; `0x032c bit16 = 0`.
3. `0xc1d4 bit1 = 1`; `0x030c[27:24] = 3`.
4. `_check_addc(B)`.
5. `0xc1f4 bit11` 1→0; `udelay(1)`; `0xc1f4[9:8] = 1`.
6. Poll `0xc1fc bit0` (1 µs, 10 ms).
7. `_check_addc(B)`.
8. `0xc1d4 bit1 = 0`; `0x032c bit16 = 1`; `0x030c[27:24] = 0xc`; `0x032c bit30 = 1`; `0x32b8 bit30 = 0`.

**ADDCK backup and reload** (rfk.c:404-432). The HW result is read back and forced as a
manual value:
1. `0xc0f4[9:8] = 0`; `d00 = 0xc0fc[19:10]`, `d01 = 0xc0fc[9:0]`.
2. `0xc1f4[9:8] = 0`; `d10 = 0xc1fc[19:10]`, `d11 = 0xc1fc[9:0]`.
3. S0 reload: `0xc0f0[25:16] = d00`; `0xc0f4[3:0] = d01 >> 6`; `0xc0f0[31:26] = d01 & 0x3f`;
   `0xc0f4[5:4] = 3` (manual on).
4. S1 reload: `0xc1f0[25:16] = d10`; `0xc1f4[3:0] = d11 >> 6`; `0xc1f0[31:26] = d11 & 0x3f`;
   `0xc1f4[5:4] = 3`.

### 3.3 DACK (DAC DC / MSB calibration): `_dac_cal`, rfk.c:756-790

Wrapper `rtw8852b_dack` (rfk.c:3748-3755): coex `WL RFK START (type DACK)` → `_dac_cal` → coex STOP.
No TX pause, because it runs at init.

Sequence:
1. Save `rf0_0 = RF[A] 0x00`, `rf1_0 = RF[B] 0x00`.
2. `_afe_init` (§3.1).
3. `_drck` (§3.2).
4. `RF[A,B] 0x05 bit0 = 0`; `RF[A,B] 0x00 = 0x337e1` (full 20 bits).
5. `_addck`, `_addck_backup`, `_addck_reload` (§3.2).
6. `RF[A,B] 0x01 = 0x00000` (RR_MODOPT).
7. **`_dack_s0`** (rfk.c:616-648):
   1. Apply `dack_s0_1` (table below). It ends with `udelay(1)`.
   2. Poll "part1 done": `0xc040 bit31 && 0xc064 bit31` (both 1). 1 µs step, 10 ms timeout.
      Timeout sets `msbk_timeout[0]` (log only).
   3. Apply `dack_s0_2`.
   4. Poll "part2 done": `0xc05c bit2 && 0xc080 bit2`. 1 µs step, 10 ms timeout.
      Timeout sets `dadck_timeout[0]`.
   5. Apply `dack_s0_3`.
   6. `_dack_backup_s0` (debug readback, rfk.c:434-459):
      * `0x12b8 bit30 = 1`.
      * For i = 0..15: `0xc000[4:1] = i` → `msbk[0][0][i] = 0xc05c[31:24]`;
        `0xc020[4:1] = i` → `msbk[0][1][i] = 0xc080[31:24]`.
      * `biask = 0xc048[11:2]`, `0xc06c[11:2]`; `dadck = 0xc060[31:24]`, `0xc084[31:24]`.
   7. `0x12b8 bit30 = 0`.
8. **`_dack_s1`** (rfk.c:665-698): same with `dack_s1_*`.
   * Done conditions use **OR**, unlike S0: part1 `0xc140 bit31 || 0xc164 bit31`;
     part2 `0xc15c bit2 || 0xc180 bit2`.
   * Then `_check_dadc(B)`: table `check_dadc_en_b`, 100-sample 0x1730 average (debug),
     table `check_dadc_dis_b`. This has lasting side effects: `0x032c bit30 = 0`,
     `0x030c[27:24] = 3`, `0x032c bit16 = 1`. Replicate it.
   * Then backup (`0xc100/0xc120` select, reads at `0xc15c/0xc180/0xc148/0xc16c/0xc160/0xc184`),
     then `0x32b8 bit30 = 0`.
9. Restore `RF[A] 0x00 = rf0_0`, `RF[B] 0x00 = rf1_0`; `RF[A,B] 0x05 bit0 = 1`.

There is no retry and no fallback. Timeouts are only recorded. The backed-up MSBK, bias and
DADCK values are **only dumped for debugging**; 8852B never reloads them.

DACK tables (BB, WM entries):

| step | entries in order (addr, mask, value) |
|---|---|
| s0_1 | 0x12a0 b15=1; 0x12a0[14:12]=3; 0x12b8 b30=1; 0x030c b28=1; 0x032c b31=0; 0xc0d8 b16=1; 0xc0dc[27:26]=3; 0xc004 b30=0; 0xc024 b30=0; 0xc004[29:20]=0x30; 0xc004[31:30]=0; 0xc004 b17=1; 0xc024 b17=1; 0xc00c b2=0; 0xc02c b2=0; 0xc004 b0=1; 0xc024 b0=1; delay 1 µs |
| s0_2 | 0xc0dc[27:26]=0; 0xc00c b2=1; 0xc02c b2=1 |
| s0_3 | 0xc004 b0=0; 0xc024 b0=0; 0xc0d8 b16=0; 0x12a0 b15=0; 0x12a0[14:12]=7 |
| s1_1 | 0x32a0 b15=1; 0x32a0[14:12]=3; 0x32b8 b30=1; 0x030c b28=1; 0x032c b31=0; 0xc1d8 b16=1; 0xc1dc[27:26]=3; 0xc104 b30=0; 0xc124 b30=0; 0xc104[29:20]=0x30; 0xc104[31:30]=0; 0xc104 b17=1; 0xc124 b17=1; 0xc10c b2=0; 0xc12c b2=0; 0xc104 b0=1; 0xc124 b0=1; delay 1 µs |
| s1_2 | 0xc1dc[27:26]=0; 0xc10c b2=1; 0xc12c b2=1; delay 1 µs (s0_2 has no delay) |
| s1_3 | 0xc104 b0=0; 0xc124 b0=0; 0xc1d8 b16=0; 0x32a0 b15=0; 0x32a0[14:12]=7 |

The debug tables are:
* `check_addc_{a,b}`: `0x20f4 b24=0`, `0x20f8 b31=1`, `0x20f0[23:16]=1`, `[11:8]=2`,
  `[3:0]=0`, `[7:6]=2 (A) / 3 (B)`.
* `check_dadc_en_{a,b}`: `0x032c b30=0`, `0x030c[27:24]=0xf` then `=3`, `0x032c b16=0`,
  `0x12dc/0x32dc b0=1`, `0x12e8/0x32e8 b2=1`, `RF[A/B] 0x8f b13=1`.
* `check_dadc_dis_{a,b}`: `0x12dc/0x32dc b0=0`, `0x12e8/0x32e8 b2=0`, `RF 0x8f b13=0`,
  `0x032c b16=1`.

### 3.4 RCK (RF RC-filter calibration): `_rck`, rfk.c:336-369

Runs per path (A then B) at init, with no coex notification.
1. `save5 = RF[p] 0x05` (full).
2. `RF[p] 0x05 bit0 = 0`.
3. `RF[p] 0x00[19:16] = 3` (RX mode).
4. Trigger: `RF[p] 0x1b = 0x00240` (full).
5. Poll `RF[p] 0x1c bit3 != 0`, 2 µs step, **30 µs** timeout. The return code is only logged.
6. `rck_val = RF[p] 0x1b[14:10]`.
7. `RF[p] 0x1b = rck_val` as a *full 20-bit write of the unshifted 5-bit value*. This is
   exactly what rtw89 does (also on 8852A/C); it clears the trigger bits.
8. `RF[p] 0x05 = save5`.

There is no fallback. If the poll times out, whatever is in 0x1b[14:10] is used.

### 3.5 RX DCK (RX DC-offset calibration)

Wrapper `rtw8852b_rx_dck` (rfk.c:3774-3788):
1. Coex `RXDCK START` (polls BT for up to 100 ms, §5).
2. Stop sch TX (all).
3. `_wait_rx_mode`.
4. `_rx_dck`.
5. Resume TX.
6. Coex STOP.

`_rx_dck` (rfk.c:304-334) for path = A, then B:
1. `save5 = RF[p] 0x05`; `tune = RF[p] 0x92 bit1` (RR_DCK_FINE).
2. If TSSI mode on p: `BB 0x5818 (+0x2000·p) bit30 = 1` (pause TSSI tracking).
3. `RF[p] 0x05 bit0 = 0`; `RF[p] 0x92 bit1 = 0`; `RF[p] 0x00[19:16] = 3` (RX).
4. `_set_rx_dck` (rfk.c:295-302): `RF[p] 0x93[3:0] = 0`; `RF[p] 0x92 bit0 = 0`, then `= 1`
   (trigger level); `mdelay(1)`.
5. `RF[p] 0x92 bit1 = tune`; `RF[p] 0x05 = save5`.
6. If TSSI mode: `BB 0x5818 bit30 = 0`.

There is no completion poll and no readback. The result stays inside the RF block.

### 3.6 IQK (TX/RX IQ imbalance plus LO leakage)

Wrapper `rtw8852b_iqk` (rfk.c:3757-3772):
1. Coex `IQK START`.
2. Stop sch TX.
3. `_wait_rx_mode(RF_AB)`.
4. `_iqk_init`: `BB 0x9fe0 = 0`. On first call it also clears the state: `is_nbiqk = false`
   (wide-band IQK), `iqk_times = 0` (rfk.c:1538-1567).
5. `_iqk`: `_doiqk(A)`, then `_doiqk(B)` (rfk.c:1628-1647).
6. Resume TX.
7. Coex STOP.

#### 3.6.1 `_doiqk(path)` (rfk.c:1596-1626)

1. Coex `IQK ONESHOT_START`.
2. `iqk_times++`. `_iqk_get_ch_info` records band, bw and ch and writes the info regs
   (all optional/informational):
   * `0x9fe0[31:24] = 0x2a` (IQK version).
   * `0x9fe4[(16p)+3:(16p)] = band` (0 = 2G, 1 = 5G), `[+7:+4] = bw`, `[+15:+8] = ch`.
   * Reads `BB 0x35c[11:10]`. The "syn1to2" flag is informational.
3. Backup the BB list and the RF list for *this path only* (§0.3).
4. **`_iqk_macbb_setting`**: apply the 32-entry `set_nondbcc_path01` sequence (rfk.c:121-154).
   All BB WM:

   ```
   0x20fc[31:16]=0x0303; 0x5864[28:27]=3; 0x7864[28:27]=3; 0x12b8 b30=1; 0x32b8 b30=1;
   0x030c[31:24]=0x13; 0x032c[31:16]=0x0041; 0x12b8 b28=1; 0x58c8 b24=1; 0x78c8 b24=1;
   0x5864[31:30]=3; 0x7864[31:30]=3; 0x2008[24:0]=0x1ffffff; 0x0c1c b2=1; 0x0700 b27=1;
   0x0c70[9:0]=0x3ff; 0x0c60[1:0]=3; 0x0c6c b0=1; 0x58ac b27=1; 0x78ac b27=1; 0x0c3c b9=1;
   0x2344 b31=1; 0x4490 b31=1; 0x12a0[14:12]=7; 0x12a0 b15=1; 0x12a0[18:16]=3; 0x12a0 b19=1;
   0x32a0[18:16]=3; 0x32a0 b19=1; 0x0700 b24=1; 0x0700[26:25]=2; 0x20fc[31:16]=0x3333
   ```

   (Register tables are OK to copy; this is a numeric transcription.)
5. **`_iqk_preset`** (rfk.c:1493-1512):
   1. `0x8104(+0x100p) bit0 = table_idx (0)`; `0x8154(+0x100p) bit3 = table_idx`.
   2. `RF[p] 0x05 bit0 = 0`; `RF[p] 0x10005 bit0 = 0`.
   3. `0x8008 = 0x00000080`; `0x8088 = 0x81ff010a`.
6. **`_iqk_by_path`** (rfk.c:1347-1384):
   1. `_iqk_txclk_setting` (rfk.c:1303-1314):
      * `0x12b8 b30 = 1`, `0x32b8 b30 = 1`, `udelay(1)`.
      * `0x030c[31:24] = 0x1f`, `udelay(1)`, `0x030c[31:24] = 0x13`.
      * `0x032c[31:16] = 0x0001`, `udelay(1)`, `0x032c[31:16] = 0x0041`.
   2. **LOK** (LO-leakage). Up to **3 tries** with `ibias = 1, 2, 3`. Stop at the first try
      that passes:
      1. `_lok_res_table(ibias)` (rfk.c:1127-1145):
         * `RF 0xef = 0x2`.
         * `RF 0x33 = 0 (2G) / 1 (5G)`.
         * `RF 0x3f = ibias`.
         * `RF 0xef = 0`.
         * `RF 0x7c bit5 = 1`.
      2. `_iqk_txk_setting` (rfk.c:1273-1301):
         * 2G: `RF 0x90[9:8] = 0`; `RF 0x51 bit19 = 0`; `RF 0x51 bit11 = 0`;
           `RF 0x52 bit11 = 1`; `RF 0x55[4:0] = 0`; `RF 0xef bit2 = 1`;
           `RF 0x33[7:0] = 0x00`; `RF 0x00[19:4] = 0x403e`; `udelay(1)`.
         * 5G: `RF 0x85[1:0] = 0`; `RF 0x60[2:0] = 1`; `RF 0x55[4:0] = 0`;
           `RF 0xef bit2 = 1`; `RF 0x33[7:0] = 0x80`; `RF 0x00[19:4] = 0x403e`; `udelay(1)`.
      3. `_iqk_lok` (rfk.c:1191-1271):
         1. `BB 0x802c[11:0] = 0x021`.
         2. `RF 0x11[1:0] = 0`, `RF 0x11[6:4] = 6 (2G) / 4 (5G)`, `RF 0x11[16:12] = 0`.
         3. `BB 0x81cc(+p) = 0x9`; one-shot **FLOK coarse**. The fail result is recorded.
         4. `RF 0x11[16:12] = 0x12`; `BB 0x81cc = 0x24`; one-shot **FLOK vbuffer**.
         5. `RF 0x11[16:12] = 0`; `BB 0x81cc = 0x9`; `BB 0x802c[11:0] = 0x021`;
            one-shot **FLOK fine**. The fail result is recorded.
         6. `RF 0x11[16:12] = 0x12`; `BB 0x81cc = 0x24`; one-shot **FLOK vbuffer**.
         7. `_lok_finetune_check` (rfk.c:1147-1189). This decides pass/fail; the one-shot
            flags are informational only:
            * `RF 0x58`: `core_i = [19:15]`, `core_q = [14:10]`. Fail if either is `< 2` or `> 0x1d`.
            * `RF 0x0a`: `vbuf_i = [19:14]`, `vbuf_q = [9:4]`. Fail if either is `< 2` or `> 0x3d`.
            * The raw values are kept (`lok_idac`, `lok_vbuf`) but never re-applied.

      If all 3 tries fail, rtw89 logs and **continues** with whatever LOK is in the RF.
   3. **TXK, wide-band** `_txk_group_sel` (rfk.c:1016-1077). For group `gp = 0..3`:
      1. `RF 0x11[1:0] = pwr_range[gp]`, `RF 0x11[6:4] = track_range[gp]`,
         `RF 0x11[16:12] = gain_bb[gp]`; `BB 0x81cc(+p) = itqt[gp]`.
      2. `BB 0x8154(+p)`: bit8 = 1, bit4 = 1 (TX), bit2 = 0, `[1:0] = gp`.
      3. `BB 0x8010[7:0] = 0`.
      4. One-shot **TXK**.
      5. `BB 0x9fe0 bit(8 + gp + 4p) = fail` (info).

      Afterwards:
      * **Any fail:** `nb_txcfir = 0x40000002`, `BB 0x8124(+p)[11:8] = 0`.
      * **Else:** `nb_txcfir = 0x40000000`, `[11:8] = 5` ("use WB CFIR").

      Per-group constants (rfk.c:98-111):

      | | gp0 | gp1 | gp2 | gp3 |
      |---|---|---|---|---|
      | 2G pwr_range / track_range / gain_bb / itqt | 0/4/0x08/0x09 | 0/4/0x0e/0x12 | 0/6/0x06/0x1b | 0/6/0x0e/0x24 |
      | 5G pwr_range / track_range / gain_bb / itqt | 0/3/0x08/0x12 | 0/3/0x0e/0x12 | 0/6/0x06/0x12 | 0/6/0x0e/0x1b |

   4. `_iqk_rxclk_setting` (rfk.c:977-1014):
      * `0x12b8 b30 = 1`, `0x32b8 b30 = 1`, `udelay(1)`.
      * `0x030c[31:24] = 0x0f`, `udelay(1)`, `= 0x03`.
      * `0x032c[31:16] = 0xa001`, `udelay(1)`, `= 0xa041`.
      * `0x12a0[18:16] = X`, `0x12a0 b19 = 1`, `0x32a0[18:16] = X`, `0x32a0 b19 = 1`.
      * `0x0700 b24 = 1`, `0x0700[26:25] = Y`.
      * **80 MHz: X = 2, Y = 1. 20/40 MHz: X = 1, Y = 0** (ADC clock rate).
   5. `_iqk_rxk_setting` (rfk.c:792-813): `RF 0x00[19:16] = 0xc`; `RF 0x20 bit8 = 1` (2G)
      or `RF 0x20 bit7 = 1` (5G); `RF 0x1f = RF 0x18` (copy the channel config).
   6. **RXK, wide-band** `_rxk_group_sel` (rfk.c:870-926). For `gp = 0..3`:
      1. 2G: `RF 0x00[13:4] = g_rxgain[gp]`, `RF 0x83[16:10] = g_attc2[gp]`,
         `RF 0x83[9:8] = g_attc1[gp]`.
      2. 5G: `RF 0x00[13:4] = a_rxgain[gp]`, `RF 0x8c[6:0] = a_attc2[gp]`,
         `RF 0x8c[8:7] = a_attc1[gp]`.
      3. `BB 0x8154(+p)`: bit8 = 1, bit4 = 0 (RX), `[2:0] = gp`.
      4. One-shot **RXK**.
      5. `BB 0x9fe0 bit(16 + gp + 4p) = fail`.

      Afterwards:
      * `RF 0x20 bit7 = 0`. Only SEL5G is cleared: in 2G, bit8 stays 1, and RF 0x20 is
        not in the RF backup list.
      * **Any fail:** `nb_rxcfir = 0x40000002`, `BB 0x8124(+p)[3:0] = 0`.
      * **Else:** `nb_rxcfir = 0x40000000`, `[3:0] = 5`.

      Constants:

      | | gp0 | gp1 | gp2 | gp3 |
      |---|---|---|---|---|
      | 2G rxgain / attc2 / attc1 | 0x212/0x00/3 | 0x21c/0x00/3 | 0x350/0x28/2 | 0x360/0x5f/1 |
      | 5G rxgain / attc2 / attc1 | 0x190/0x0f/3 | 0x198/0x0f/1 | 0x350/0x3f/0 | 0x352/0x7f/0 |

   7. `_iqk_info_iqk`: informational writes to 0x9fe0/0x9fe8, plus snapshots of
      `0x8124/0x8138/0x813c`. Optional.
7. **`_iqk_restore`** (rfk.c:1439-1465):
   1. `BB 0x8138(+p) = nb_txcfir`; `BB 0x813c(+p) = nb_rxcfir`.
   2. `BB 0x8000 = 0xe19 + (p << 4)`, then `_iqk_check_cal` (poll 0xbff8 == 0x55, etc.).
   3. `0x8010[7:0] = 0`; `0x8008 = 0`; `0x8088 = 0x80000000`.
   4. `0x8120 bit28 = 0` (path-0 register for both paths); `0x8220 bit28 = 0`; `0x8220 bit16 = 0`.
   5. `RF[p] 0xef bit2 = 0` (written twice); `RF[p] 0x00[19:16] = 3`;
      `RF[p] 0x05 bit0 = 1`; `RF[p] 0x10005 bit0 = 1`.
8. **`_iqk_afebb_restore`**: 18-entry `restore_nondbcc_path01` (rfk.c:156-175):

   ```
   0x20fc[31:16]=0x0303; 0x12b8 b30=0; 0x32b8 b30=0; 0x5864[31:30]=0; 0x7864[31:30]=0;
   0x2008[24:0]=0; 0x0c1c b2=0; 0x0700 b27=0; 0x0c70[4:0]=3; 0x0c70[9:5]=3; 0x12a0[19:12]=0;
   0x32a0[19:12]=0; 0x0700[26:24]=0; 0x20fc[31:16]=0x0000; 0x58c8 b24=0; 0x78c8 b24=0;
   0x0c3c b9=0; 0x2344 b31=0
   ```
9. Restore the BB list, then the RF list for this path.
10. Coex `IQK ONESHOT_STOP`.

#### 3.6.2 IQK result storage, meaning and fallback

The microcode writes the per-group CFIR coefficients into KIP memory. The driver never reads
them. `BB 0x8124[11:8]/[3:0] = 5` tells the TX/RX correction path to use the WB CFIR
result. `0x8138/0x813c` hold the single-tap coefficient word.

* `0x40000000` is the identity/default. The NCTL table initialises 0x8138/0x813c/0x8238/0x823c
  to this value.
* bit1 is set (`0x40000002`) on failure together with `0x8124` select = 0.
* In NB mode (unused) bit1 is ORed onto the measured coefficient.

**Interpretation, not stated in the source:** bit1 means "use the single-tap coefficient
in 0x8138/0x813c instead of the CFIR table". On failure the result is therefore identity
correction, i.e. no worse than no IQK. The coefficient bank is 0, selected by
`0x8104 bit0` and `0x8154 bit3`.

**Retry:** only the LOK stage retries (3 tries with increasing ibias). TXK and RXK are
single attempts. There is no IQK retry at a higher level.

### 3.7 DPK (digital pre-distortion calibration) and DPD back-off

Wrapper `rtw8852b_dpk` (rfk.c:3790-3806):
1. Coex `DPK START`.
2. Stop sch TX.
3. `_wait_rx_mode`.
4. `is_dpk_enable = true`, `is_dpk_reload_en = false`.
5. `_dpk()`.
6. Resume TX.
7. Coex STOP.

`_dpk` (rfk.c:2574-2586):
* If an external PA is flagged for the band (`fem.epa_2g/5g`), DPD is forced off.
  **This never happens on 8852B**: only 8852A sets `fem.epa_*` (rtw8852a.c:1463-1470).
* Otherwise `_dpk_cal_select(RF_AB)`.

#### 3.7.1 DPD back-off init (`rtw8852b_dpk_init` → `_set_dpd_backoff`, rfk.c:2692-2717)

Runs at rfk_init:
1. `ofdm_bkof = BB 0x44a0[16:12]`, `tx_scale = BB 0x44a0[6:0]`. These are set by the BB table.
2. **If `ofdm_bkof + tx_scale >= 44`:** `dpk_gs = 0x7f` and `BB 0x81bc[22:0] = 0x7f7f7f`,
   `BB 0x82bc[22:0] = 0x7f7f7f` ("move DPD back-off to BB").
3. **Else:** `dpk_gs = 0x5b`.

#### 3.7.2 `_dpk_cal_select` (rfk.c:2484-2537), reload disabled

1. `cur_idx[A] = cur_idx[B] = 0`.
2. Backup the BB list.
3. For each path:
   1. Backup KIP `{0x813c, 0x8124, 0x8120} + 0x100p`.
   2. Backup the RF list.
   3. Record band/ch/bw.
   4. If TSSI mode: pause TSSI (`BB 0x5818(+0x2000p) bit30 = 1`).
4. `_dpk_bb_afe_setting` (§3.7.3).
5. For p = A, B: `fail = _dpk_main(p, gain = 1)`, then `_dpk_onoff(p, off = fail)`.
6. `_dpk_bb_afe_restore`, then restore the BB list.
7. For each path:
   1. `_dpk_kip_restore(p)` (§3.7.9).
   2. Reload the 3 KIP regs.
   3. Restore the RF list.
   4. If TSSI mode, resume TSSI (`bit30 = 0`).

#### 3.7.3 BB/AFE setting and restore

`dpk_afe` table (tbl.c:169-199). All BB:

```
0x20fc[31:16]=0x0303; 0x12b8 b30=1; 0x32b8 b30=1; 0x030c[31:24]=0x13; 0x032c[31:16]=0x0041;
0x12b8 b28=1; 0x58c8 b24=1; 0x78c8 b24=1; 0x5864[31:30]=3; 0x7864[31:30]=3;
0x2008[24:0]=0x1ffffff; 0x0c1c b2=1; 0x0700 b27=1; 0x0c70[9:0]=0x3ff; 0x0c60[1:0]=3;
0x0c6c b0=1; 0x58ac b27=1; 0x78ac b27=1; 0x0c3c b9=1; 0x2344 b31=1; 0x4490 b31=1;
0x12a0[19:12]=0xbf; 0x32a0[19:16]=0xb; 0x0700[26:24]=5; 0x20fc[31:16]=0x3333;
0x580c b15=1; 0x5800[15:0]=0; 0x780c b15=1; 0x7800[15:0]=0
```

For 80 MHz, additionally `BB 0xc0d8 bit13 = 1`, `BB 0xc1d8 bit13 = 1`.

`dpk_afe_restore` (tbl.c:203-228):

```
0x20fc[31:16]=0x0303; 0x12b8 b30=0; 0x32b8 b30=0; 0x5864[31:30]=0; 0x7864[31:30]=0;
0x2008[24:0]=0; 0x0c1c b2=0; 0x0700 b27=0; 0x0c70[9:0]=0x63; 0x12a0[19:12]=0; 0x32a0[19:12]=0;
0x0700[26:24]=0; 0x5864 b29=0; 0x7864 b29=0; 0x20fc[31:16]=0; 0x58c8 b24=0; 0x78c8 b24=0;
0x0c3c b9=0; 0x580c b15=0; 0x58e4[28:27]=1 then =2; 0x780c b15=0; 0x78e4[28:27]=1 then =2
```

For 80 MHz, additionally clear the two 0xc0d8/0xc1d8 bit13 bits.

#### 3.7.4 `_dpk_main(path p, gain = 1, kidx = 0)` (rfk.c:2435-2482)

1. `RF[p] 0x05 bit0 = 0`; `RF[p] 0x10005 bit0 = 0` (direct control).
2. KIP power/clock on: `BB 0x8008 = 0x80`; `BB 0x8088 = 0x807f030a`; `BB 0x8120(+p) = 0xce000a08`.
3. Set TX AGC 0x38:
   1. `RF[p] 0x10001 = 0x38` (full; D-die TX AGC).
   2. `BB 0x5864 bit29 = 1`.
   3. One-shot **DPK_TXAGC (0x19)**.
   4. `BB 0x5864 bit29 = 0`.
4. RF setting (rfk.c:1892-1921):
   * 2G: `RF 0x00 = 0x50220`; `RF 0x83[7:0] = 0xf2`; `RF 0xdf bit12 = 1`; `RF 0x9e bit8 = 1`.
   * 5G: `RF 0x00 = 0x50220`; `RF 0x8c[15:9] = 5`; `RF 0xdf bit12 = 1`; `RF 0x9e bit8 = 1`;
     `RF 0x8b = 0x920fc`; `RF 0x90 = 0x002c0`; `RF 0x97 = 0x38800`.
   * Then `RF 0xde bit2 = 1`; `RF 0x1a[14:12] = bw + 1`; `RF 0x1a[11:10] = 0`.
5. DPK RX DCK:
   1. `RF 0x8f[11:10] = 3`.
   2. `RF 0x93[3:0] = 0`; `RF 0x92 bit0` 0→1; `mdelay(1)`.
6. KIP preset:
   1. `BB 0x806c[2:1]` = 0 (80M) / 2 (40M) / 1 (20M) (test-pattern BW).
   2. One-shot **KIP_PRESET (0x16)**.
7. Set RX AGC:
   1. `BB 0x8078[19:0] = RF[p] 0x00`.
   2. `0x5864 bit29 = 1`; one-shot **DPK_RXAGC (0x15)**; `0x5864 bit29 = 0`.
   3. `BB 0x80d4[19:16] = 8`.
8. Table select: `BB 0x81ac(+p)[31:24] = 0x80 + kidx·0x20 + gain·0x10`, which is **0x90**.
9. `txagc = _dpk_agc(init 0x38)` (§3.7.5).
10. **If `txagc == 0xff`: FAIL** (sync failed).
11. Otherwise:
    1. Read the thermal into `ther_dpk` (RF 0x42 method, §6.1).
    2. `_dpk_idl_mpa` (§3.7.6).
    3. `RF[p] 0x00[19:16] = 3`.
    4. `_dpk_fill_result` (§3.7.7).
12. `path_ok = !fail`.

#### 3.7.5 `_dpk_agc` state machine (rfk.c:2208-2325)

Start `step = SYNC_DGAIN`, `txagc = 0x38`. Loop while `!goout && agc_cnt < 6 && limit-- > 0`
(limit 200).

* **SYNC_DGAIN**:
  1. One-shot **SYNC (0x10)**. Sync check (rfk.c:1974-2014):
     1. Clear `BB 0x80d4[21:16]`; `corr_idx = 0x80fc[7:0]`, `corr_val = 0x80fc[15:8]`.
     2. `BB 0x80d4[21:16] = 9`; `dc_i = |sext12(0x80fc[27:16])|`, `dc_q = |sext12(0x80fc[11:0])|`.
     3. **Fail** if `dc_i > 200 || dc_q > 200 || corr_val < 170`. Then return `txagc = 0xff`.
  2. DGain read: `BB 0x80d4[21:16] = 0`; `dgain = 0x80fc[27:16]`.
  3. Next step: GAIN_LOSS_IDX if `limited_rxbb`, else GAIN_ADJ.
* **GAIN_ADJ**:
  1. `rxbb = RF[p] 0x00[9:5]`. `off = map(dgain)` using the thresholds below.
  2. `rxbb = clamp(rxbb + off, 0, 0x1f)`. Clamping sets `limited_rxbb`. Write it back to `RF 0x00[9:5]`.
  3. If `off != 0` or this is the first pass:
     * bw < 80: bypass the RX CFIR (`BB 0x813c(+p) bit2 = 1, bit0 = 1`).
     * 80 MHz: **LBK RXIQK** (§3.7.8).
  4. Next step: SYNC_DGAIN again if `dgain > 1922 (0x782)` or `dgain < 342 (0x156)`,
     else GAIN_LOSS_IDX. `agc_cnt++`.

  Dgain mapping (rfk.c:2037-2083):

  | dgain ≥ | 0xbf1 | 0xaa5 | 0x97d | 0x875 | 0x789 | 0x6b7 | 0x5fc | 0x556 | 0x4c1 | 0x43d | 0x3c7 | 0x35e | 0x2ac | 0x262 | 0x220 | below |
  |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
  | off | +6 | +6 | +5 | +4 | +3 | +2 | +1 | 0 | −1 | −2 | −3 | −4 | −5 | −6 | −7 | −8 |

* **GAIN_LOSS_IDX**:
  1. Table select 0x90; one-shot **GAIN_LOSS (0x13)**.
  2. Read: `BB 0x80d4[21:16] = 6`; `BB 0x80bc bit14 = 1`; `gl = 0x80fc[7:4]`.
  3. PAS check, when `gl == 0` (rfk.c:2167-2206):
     1. `0x80d4[23:16] = 0x06`; `0x80bc bit14 = 0`; `0x80c0[23:16] = 0x08`.
     2. `0x80c0[31:24] = 0x00` → `v1 = (|sext12(0x80fc[31:16])|, |sext12(0x80fc[15:0])|)`.
     3. `0x80c0[31:24] = 0x1f` → `v2` the same way.
     4. The PAS check is true if `|v1|² ≥ |v2|²·8/5`.
  4. Next step: GL_GT if `(gl == 0 && PAS) || gl ≥ 7`; GL_LT if `gl == 0`; else SET_TX_GAIN.
* **GL_GT** (too much compression):
  * If `txagc == 0x2e`: exit.
  * Else `txagc = set_offset(+3)`: new = `clamp(RF[p] 0x10001 − 3, 0x2e, 0x3f)`, then
    re-run the DPK_TXAGC one-shot as in step 3 of §3.7.4.
  * Then back to GAIN_LOSS_IDX. `agc_cnt++`.
* **GL_LT** (too little):
  * If `txagc == 0x3f`: exit.
  * Else `set_offset(−2)` (i.e. +2).
  * Then back to GAIN_LOSS_IDX. `agc_cnt++`.
* **SET_TX_GAIN**: `set_offset(gl)` (txagc −= gl, clamped 0x2e..0x3f); exit.

The return value is the final txagc (0x2e..0x3f), or 0xff on sync failure.

#### 3.7.6 IDL (model identification, rfk.c:2327-2367)

* 5G at bw < 80 → order 2:
  * `BB 0x80a0[1:0] = 2`; clear `0x80a0[12:8]`; clear `0x8070[31:28]`.
* Everything else (all 2G, and 5G 80 MHz) → order 0:
  * `0x80a0[1:0] = 0`; `0x80a0[12:8] = 3`; `0x8070[31:28] = 1`.

Then one-shot **MDPK_IDL (0x11)**.

Order encoding for later writes: `ord = 3 >> 0x80a0[1:0]`, so order 0 → 3, 1 → 1, 2 → 0.

#### 3.7.7 Fill result (rfk.c:2369-2406)

1. `BB 0x8104(+p) bit8 = kidx (0)`.
2. `BB 0x81c4(+p)[13:8] = txagc`.
3. `BB 0x81b4(+p)[24:16] = pwsf = 0x78`.
4. `BB 0x81dc(+p) bit16 = 1`, then `0` (load MDPD coefficients).
5. `BB 0x81bc(+p) = 0x007f7f7f` if `dpk_gs == 0x7f`, else `0x005b5b5b`.
6. `BB 0x81bc(+p)[26:25] = ord`.
7. `BB 0x81a0(+p) = 0`; `BB 0x8070 bit31 = 0`.

**On/off** (`_dpk_onoff`, rfk.c:1688-1700):
`BB 0x81bc(+p)[31:24] = (ord << 1) | en`, with `en = is_dpk_enable && !fail && path_ok`.
**On failure, DPD is simply disabled for that path** (bit24 = 0), which is also the NCTL
default. There is no retry.

#### 3.7.8 LBK RXIQK (80 MHz only, rfk.c:1832-1874)

1. `rxbb = RF 0x00[9:5]`.
2. `BB 0x8074 bit31 = 1`; `BB 0x8124(+p)[3:0] = 0`.
3. `RF 0x1f = RF 0x18`; `RF 0x00[19:16] = 0xd`; `RF 0x20 bit5 = 1`.
4. `RF 0x98[6:0]` = 0x13 if rxbb ≥ 0x11, 0x00 if rxbb ≤ 0xa, else 0x05.
5. `RF 0x85[1:0] = 0`; `RF 0x1e bit19 = 0`; `RF 0x1e = 0x80014`; `udelay(70)`.
6. `BB 0x5864 bit29 = 1`; `BB 0x802c[27:16] = 0x025`; one-shot **LBK_RXIQK (0x06)**.
7. `0x5864 bit29 = 0`; `RF 0x20 bit5 = 0`; `0x8074 bit31 = 0`; `0x80d0[21:20] = 0`;
   `0x81dc(+p) bit1 = 1`; `RF 0x00[19:16] = 5`.

#### 3.7.9 KIP restore (rfk.c:1821-1830)

1. Table `dpk_kip`: `BB 0x8008 = 0`, `BB 0x8088 = 0x80000000`.
2. **CV > CAV (true for this CBV chip):** `BB 0x81c8(+p) bit15 = 1` ("HW TX AGC offset mode").

#### 3.7.10 DPK track (every 2 s, rfk.c:2588-2690)

For each path p (kidx = 0):
1. `cur = EWMA thermal[p]` (§6). If the channel was calibrated (`bp.ch != 0`) and `cur != 0`:
   `d = ther_dpk − cur`.
2. Scale: `d = d·3/2` for 2G, `d·5/2` for 5G (s8 integer arithmetic).
3. `txagc_rf = BB 0x1c60(+0x2000p)[5:0]`.
4. **TSSI mode:**
   1. Read `0x1c60[23:16]` (txagc_bb, s8) and `0x1c04(+0x2000p)[2:0]` (txagc_bb_tp).
   2. `ofst = 0x1c60[31:24]`. If `BB 0x81c8(+p) bit15 == 1` (always, on CBV) then `ofst = 0`.
   3. `ini = ofst + d` if `txagc_rf && cur`.
   4. If `BB 0x58d4(+0x2000p)[31:28] == 0`: `pwsf = 0x78 + txagc_bb_tp − txagc_bb + ini`;
      else `pwsf = 0x78 + ini`.
5. **Non-TSSI:** `pwsf = (0x78 + d) & 0x1ff`.
6. If `BB 0x80f0 bit31 == 0` and `txagc_rf != 0`: write `BB 0x81b4(+p)[8:0] = pwsf`
   and `[24:16] = pwsf`.

### 3.8 TSSI (closed-loop TX power control)

State flag `is_tssi_mode[p]` is set by `_tssi_enable` and cleared by `_tssi_disable`/rfk_init.
It changes the thermal readout method (§6) and triggers the TSSI pause in RX DCK and DPK.

Efuse inputs (cmn.c:175-204, 281-313; offsets computed from `struct rtw8852bx_efuse`, cmn.h:29-80):

| data | location |
|---|---|
| `tssi_cck[p][0..5]` | logical efuse A 0x210–0x215, B 0x23a–0x23f |
| `tssi_mcs[p][0..4]` (2G) | A 0x216–0x21a, B 0x240–0x244 |
| `tssi_mcs[p][5..18]` (5G) | A 0x222–0x22f, B 0x24c–0x259 |
| `thermal[p]` (TSSI base) | logical A 0x2d0, B 0x2d1 (0xff = unprogrammed) |
| `tssi_trim[p][0..7]` | **physical** (phycap) A 0x5d6 down to 0x5cf, B 0x5ab down to 0x5a4 (decreasing). All 0xff → treated as 0 |

#### 3.8.1 Full TSSI `rtw8852b_tssi(phy, hwtx_en)` (rfk.c:3813-3850)

1. Coex `IQK ONESHOT_START`. At rfk_channel time the WL-RFK state is already STOP, so this
   is rejected, i.e. a no-op. **TSSI therefore runs without WL-RFK coex protection** (§5).
2. `_tssi_disable` (rfk.c:3093-3104). For A and B:
   * `BB 0x5820/0x7820 bit31 = 0` (TSSI enable).
   * `0x5818/0x7818[28:27] = 1`.
   * `0x58e4/0x78e4 bit14 = 1` (moving-average clear).
   * `is_tssi_mode = false`.
3. For p = A, then B:
   1. RF: `RF[p] 0x7f bit1 = 1` (2G) or `bit8 = 1` (5G).
   2. Tables `tssi_sys`, then `tssi_sys_{p}_{band}`.
   3. Tables `tssi_init_txpwr_{p}`, `tssi_init_txpwr_he_tb_{p}`, `tssi_dck_{p}`.
   4. Thermal-meter table `_tssi_set_tmeter_tbl` (§3.8.2).
   5. Tables `tssi_dac_gain_{p}`, `tssi_slope_{p}_{band}`, `tssi_align_{p}_{band}_all`
      (band: 2G = ch 1–14; 5G1 = 36–64; 5G2 = 100–144; 5G3 = 149–177), `tssi_slope_defs_{p}`.
   6. Stop sch TX; `_wait_rx_mode`; if `hwtx_en`, run `_tssi_alimentk(p)` (§3.8.5); resume TX.
4. `_tssi_enable` (rfk.c:3041-3091). For p = A, B (path B at +0x2000):
   1. `BB 0x5814 bit11 = 0` (tracking not bypassed).
   2. `BB 0x58e4[19:11] = 0x010` (moving average).
   3. `BB 0x58e4 bit14 = 0`.
   4. `BB 0x5820 bit31 = 0`, then `1`.
   5. `RF[p] 0x10055 bit7 = 1` (D-die TSSI tracking enable).
   6. `BB 0x5818[28:27] = 3`; `BB 0x5818[7:0] = 0xc0` (TX AGC offset default);
      `BB 0x5818 bit28 = 0`, then `1`.
   7. `is_tssi_mode[p] = true`.
5. `_tssi_set_efuse_to_de` (§3.8.6).
6. Coex `ONESHOT_STOP` (no-op).

#### 3.8.2 TSSI thermal-offset table (rfk.c:2773-2928)

Select the swing tables by subband: 2G → `2g{a,b}_{p,n}`; 5G ch 36–64 → `5g*[0]`;
100–144 → `[1]`; 149–177 → `[2]` (data in Appendix B, static in table.c:14601-14651).
The 8852B code uses these **static driver tables, not the FW "PWR_TRK" element**; only
8852C/8852BT use `elm_info.txpwr_trk`.

For path A (B identical at 0x7810/0x7864/0x7c00):
1. `BB 0x5810 bit16 = 0`; `BB 0x5810 bit24 = 1`.
2. **If efuse `thermal[A] == 0xff`:** `0x5810[15:10] = 32`; `0x5864[25:20] = 32`;
   `BB 0x5c00..0x5c3c` (16 dwords) = 0.
3. **Else:** `0x5810[15:10] = thermal`; `0x5864[25:20] = thermal`. Build a 64-byte s8 table T:
   * `T[j] = −N[min(j, 29)]` for j = 0..31;
   * `T[j] = P[min(64 − j, 29)]` for j = 32..63;

   where N and P are the `_n` and `_p` 30-entry arrays. Write it little-endian, 4 bytes per
   dword, to `BB 0x5c00 + j` (so byte T[j] sits at byte offset j).
   Index j appears to be the 6-bit two's-complement thermal delta. That is an inference;
   the table itself is what the source defines.
4. `BB 0x5864 bit26 = 1`, then `0` (ready strobe).

#### 3.8.3 TSSI on band change `rtw8852b_tssi_scan` (rfk.c:3852-3889)

1. `_tssi_disable`.
2. For p = A, B:
   1. RF 0x7f setting (as in §3.8.1).
   2. `tssi_sys` + band table.
   3. Thermal table.
   4. If `alignment_done[p][band]`: `_tssi_alimentk_done` (write the stored
      `alignment_value[p][band][0..3]` to `0x5630/0x5634/0x563c/0x5640 (+0x2000p)`).
      Else apply `tssi_align_{p}_{band}_all`.
3. `_tssi_enable`; `_tssi_set_efuse_to_de`.

It does **not** apply `init_txpwr`, `he_tb`, `dck`, `dac_gain` or `slope`. On the very first
set_channel TSSI is therefore enabled with whatever those registers hold from the BB table,
until the first rfk_channel.

#### 3.8.4 Scan end (rfk.c:3891-3931)

1. `BB 0x5818[7:0] = 0xc0`, `BB 0x7818[7:0] = 0xc0`.
2. `0x5818 bit28` 0→1, `0x7818 bit28` 0→1.
3. `_tssi_alimentk_done(A)`, `_tssi_alimentk_done(B)`: re-apply the stored alignment if one
   exists for the current band.

#### 3.8.5 TSSI alignment with PMAC hardware TX `_tssi_alimentk` (rfk.c:3559-3733)

This really transmits on the air: PMAC HT20 MCS7 packets on the current channel.

1. `ch_idx`: 2G `ch − 1`; 36–64 `(ch − 36)/2 + 14`; 100–144 `(ch − 100)/2 + 29`;
   149–177 `(ch − 149)/2 + 52`.
   If this `(path, ch_idx)` was aligned before (a per-channel cache kept for the driver's
   lifetime), write the 4 cached dwords to `0x5630/0x5634/0x563c/0x5640 (+0x2000p)` and return.
2. `power[] = {48, 20, 4, 4}` (same for 2G and 5G). The value is written to
   `BB 0x4594[30:22]` by the PMAC helper. **Units unverified**; plausibly 0.25 dBm (48 → 12 dBm).
3. Backup the TSSI BB state (`rtw8852bx_bb_backup_tssi`, cmn.c:1570-1583):
   `0x458c[31:28]`, `0x49c4[3:0]`, `0x12ac`, `0x12b0`, `0x32ac`, `0x32b0`, `0x4594[30:22]`.
   Also backup BB `{0x5820, 0x7820, 0x4978, 0x58e4, 0x78e4, 0x49c0, 0x0d18, 0x0d80}`.
4. `0x5820[15:12] = 8`, `0x7820[15:12] = 8`, `0x58e4[13:11] = 2`, `0x78e4[13:11] = 2`.
5. For j = 0, 1 (`_tssi_get_cw_report`, rfk.c:3484-3557):
   1. `BB (p ? 0x7820 : 0x5820) bit31` 0→1 (re-arm).
   2. Start the PMAC TX of 100 packets (period 5000, tx_time 20) at `power[j]`.
      * j = 0: TX path = p, RX path = p.
      * j = 1: `RF_PATH_ABCD`, which leaves the previous TX path unchanged.
   3. Poll `BB (p ? 0x3c18 : 0x1c18) bit16` (report ready) up to 100 times with 30 µs
      delay (≈3 ms).
   4. If never ready: stop the PMAC TX and **abort** (goto restore, no alignment stored).
   5. Otherwise `cw[j] = 0x1c18/0x3c18[8:0]`, then stop the PMAC TX.
6. Compute (sext9 = sign-extend from bit 8):
   * `d1 = sext9(0x5630(+0x2000p)[29:20])`; `o1 = cw0 − (power0 − power1)·2 − cw1 + d1`; `diff = o1 − d1`.
   * `o2 = sext9(0x5630[19:10]) + diff`; `o3 = sext9(0x5630[9:0]) + diff`.
7. Write `v = (o1 << 20 & 0x3ff00000) | (o2 << 10 & 0xffc00) | (o3 & 0x3ff)` into
   `0x5630[29:0]` and `0x563c[29:0]` (path B: 0x7630 / 0x763c).
8. Store `alignment_value[p][band]` and the per-channel cache from
   `0x5630, 0x5634, 0x563c, 0x5640 (+0x2000p)`.
9. Restore:
   1. The 8 BB regs, then the TSSI BB state. `restore_tssi` also sets `0x9a4 bit16 = 1` and
      `0x458c`/`0x45b4` for the saved path.
   2. `tx_mode_switch(0)`:
      * `0x980 bit0 = 0`, `bit16 = 0`; `0x988[11:0] = 0`; `0x994[7:4] = 0`.
      * `0x9a4 bit10 = 0`, `0x9a4[4:2] = 0`, `0x9a4 bit16 = 0`.

**PMAC TX helper sequence** (cmn.c:1432-1567), used above:
* **Enable:**
  1. PLCP table `rtw8852bx_pmac_ht20_mcs7_tbl`: 120 BB writes to 0x4530–0x45b8
     (cmn.c:14-136); port it as data.
  2. TX path: `0x9a4[4:2] = 7`, `0x458c[31:28] = 1/2/3` (A/B/AB), `0x45b4[20:17] = 0/0/4`.
  3. RX path config (`bb_ctrl_rx_path`, cmn.c:1654-1708). It touches `0x49c4`, `0x49c0`,
     `0xd18`, `0xd80`, the RX gain offset, BTG BT-RX and `0x58dc/0x78dc[31:30]` 1→3.
  4. Power: `0x9a4 bit16 = 1`; `0x4594[30:22] = pwr`.
  5. `0x980 bit0 = 1`, `bit16 = 1`; `0x988[11:0] = 0x3f`; `0x704 bit1 = 0`;
     `0xc3c bit9 = 1`; `0x2344 bit31 = 1`; `0x704 bit1 = 1`.
  6. `0x9c4 bit4 = 1` (packet TX); `0x9c4[31:8] = period`; `0x9c8 = count`.
  7. `0x9c0 bit0` 1→0 (go).
* **Disable:** `0x9c4 bit4 = 0`; `0xc3c bit9 = 0`; in 2G also `0x2344 bit31 = 0`.
* The TX counter can be read at `0x1a40[15:0]` (debug only).

Side effect to note: `bb_ctrl_rx_path` leaves BTG BT-RX in the state computed for the last
call. For the j = 1 call (`RF_ABCD`) that state is "off", and rtw89 does not re-apply
`cfg_txrx_path` afterwards (open question in §8).

#### 3.8.6 Efuse DE write `_tssi_set_efuse_to_de` (rfk.c:3296-3349)

For each path p:
1. `cck_g` = CCK group of ch (1–2 → 0, 3–5 → 1, 6–8 → 2, 9–11 → 3, 12–13 → 4, 14 → 5).
2. `trim` = `tssi_trim[p][tg]`, where tg = 0 for ch 1–8, 1 for 9–14, 2 for 36–48,
   3 for 52–64, 4 for 100–112, 5 for 116–128, 6 for 132–144, 7 for 149–177.
3. Write `BB 0x5858` and `0x5860` (+0x2000p) `[21:12] = tssi_cck[p][cck_g] + trim`.
4. OFDM group g (rfk.c:3132-3198):
   * 2G: 1–2 → 0, 3–5 → 1, 6–8 → 2, 9–11 → 3, 12–14 → 4.
   * 5G: groups 5..18 for 36–40, 44–48, 52–56, 60–64, 100–104, 108–112, 116–120, 124–128,
     132–136, 140–144, 149–153, 157–161, 165–169, 173–177.
   * "Between" channels (41–43, 49–51, 57–59, 105–107, 113–115, …, 170–172) use the
     truncated average of groups g and g+1.
5. Value = `tssi_mcs[p][g] + trim`. Write to `[21:12]` of
   `0x5838, 0x5840, 0x5848, 0x5850, 0x5828, 0x5830` (+0x2000p), i.e. 20M/40M/80M/80+80/5M/10M.

### 3.9 LCK / synthesizer-lock check and RF channel/BW programming

This is part of `set_channel_rf` (rfk.c:3943-4166). It is included here because it is the
only "LCK" in 8852B. It is probably also covered by the channel track.

1. `_ctrl_ch(ch)`: in order A(dav = RF 0x18), B(0x18), A(D-die 0x10018), B(0x10018).
   For each:
   1. `v = RF 0x18`.
   2. Clear `[17:16]` BAND1, bit15 POW_LCK, bit14 TRX_AH, bit13 BCN, `[9:8]` BAND0, `[7:0]` CH.
   3. Set `CH = ch`. For 5G (ch > 14), also `BAND1 = 1` and `BAND0 = 1`.
   4. Clear bits 15/14/13/12, then set bit12 (BW2).
   5. **Path A, dav:** `_set_ch(v)`:
      1. `save = RF[A] 0xb1`.
      2. `RF[A] 0xb1[8:6] = 1`.
      3. `RF[A] 0x18 = v`.
      4. Poll `RF[A] 0xb7 bit8 == 0` (LCK busy), 1 µs step, 1000 µs timeout.
      5. `RF[A] 0xb1 = save`.
      6. If there was no timeout: `_lck_check`.
   6. **Others:** plain write.
   7. Then, for every path/dav: `RF 0xcf bit0` 0→1.
2. `_lck_check` (rfk.c:4014-4060). Path A only. "Locked" is `RF 0xc5 bit15 == 1`:
   1. If unlocked: MMD reset: `RF 0xd5 bit8 = 1`, `bit6 = 0`, `bit6 = 1`, `bit8 = 0`.
   2. `udelay(10)`.
   3. If still unlocked: `RF 0xd3 bit8 = 1`; `_set_s0_arfc18(RF 0x18)` (re-write with busy poll);
      `RF 0xd3 bit8 = 0`.
   4. If still unlocked: synth off/on:
      1. Rewrite `RF 0xa0` and `RF 0xaf` with their own values.
      2. `RF 0xdd bit4 = 1`; `RF 0xa0[3:2] = 0`, then `3`; `RF 0xdd bit4 = 0`.
      3. `RF 0xd3 bit8 = 1`; `_set_s0_arfc18(RF 0x18)`; `RF 0xd3 bit8 = 0`.
3. `_ctrl_bw(bw)`: for A/0x18, B/0x18, A/0x10018, B/0x10018:
   1. `RF 0x18[11:10]` = 3 (20M), 2 (40M), 1 (80M).
   2. Clear bits 15/14/13/12, set bit12.
   3. Skip if the read returns 0xffffffff.
4. `_rxbb_bw(bw)`, per path:
   1. `RF 0xee bit2 = 1`.
   2. `RF 0x33[4:0] = 0x12`.
   3. `RF 0x3f[5:0]` = 0x1b (20M), 0x13 (40M), 0x0b (80M), 0x03 (other).
   4. `RF 0xee bit2 = 0`.

---------------------------------------------------------------------------------------------

## 4. RF register quick reference (as used by 8852B RFK)

"A" = A-die/SWSI, "D" = D-die direct (bit16). Field names are rtw89's (reg.h ~8480-8990).

| RF addr | name | fields used |
|---|---|---|
| 0x00 A | RR_MOD | [19:16] mode: 0 down, 1 standby, 2 TX, 3 RX, 4 TXIQK, 5 DPK, 6/7 RXK, 0xc RXK setup, 0xd LBK; [13:4] RX gain; [9:5] RXBB; [19:4] IQK field (0x403e); full 0x337e1 (DACK) / 0x50220 (DPK) |
| 0x01 A | RR_MODOPT | 0 during DACK |
| 0x05 A | RR_RSV1 | bit0: 1 = BB controls RF, 0 = direct |
| 0x0a A | RR_LOKVB | LOK vbuffer result [19:14] I, [9:4] Q |
| 0x11 A | RR_TXIG | [1:0] power range, [6:4] track range, [16:12] BB gain |
| 0x18 A / 0x10018 D | RR_CFGCH(_V1) | [7:0] ch, [9:8] band0, [11:10] BW (3/2/1 = 20/40/80), bit12 BW2, bit13 BCN, bit14 TRX_AH, bit15 POW_LCK, [17:16] band1 |
| 0x1a A | RR_BTC | [14:12] TXBB BW, [11:10] RXBB BW |
| 0x1b A | RR_RCKC | RCK trigger (0x00240), [14:10] result |
| 0x1c A | RR_RCKS | bit3 RCK done |
| 0x1e A | RR_RXKPLL | bit19 POW; full 0x80013/0x80014 |
| 0x1f A | RR_RSV4 | copy of 0x18 for loopback |
| 0x20 A | RR_RXK | bit8 SEL2G, bit7 SEL5G, bit5 PLLEN |
| 0x33 A | RR_LUTWA | LUT address ([7:0], [4:0]) |
| 0x3f A | RR_LUTWD0 | LUT data ([5:0] RXBB BW; LOK ibias) |
| 0x42 A | RR_TM | bit19 thermal trigger, [6:1] thermal value |
| 0x43 A | RR_TM2 | [19:16] thermal trim |
| 0x51 / 0x52 / 0x55 A | RR_TXG1/TXG2/TXGA | ATT bits; [4:0] LOK ext |
| 0x58 A | RR_TXMO | LOK IDAC [19:15] I, [14:10] Q |
| 0x5d A | RR_TXA | [19:14] TSSI track index (debug) |
| 0x60 A | RR_BIASA | [2:0] (IQK 5G), [15:12] PA bias 2G, [19:16] PA bias 5G |
| 0x7c A | RR_TXVBUF | bit5 DAC enable |
| 0x7f A | RR_TXPOW | bit1 2G TSSI, bit8 5G TSSI |
| 0x83 A | RR_RXBB | [7:0] FATT, [9:8] C1G, [16:10] C2G |
| 0x85 / 0x90 A | RR_XGLNA2 / RR_XALNA2 | switch fields; 0x90 full 0x002c0 in DPK 5G |
| 0x8b A | RR_RXA_LNA | full 0x920fc (DPK 5G) |
| 0x8c A | RR_RXA2 | [6:0] HATT, [8:7] CC2, [15:9] SWATT |
| 0x8f A | RR_RXBB2 | [11:10] TIA IDA enable; bit13 (DADC check) |
| 0x92 A | RR_DCK | bit0 RX-DCK trigger level, bit1 fine |
| 0x93 A | RR_DCK1 | [3:0] clear |
| 0x97 / 0x98 / 0x9e A | RR_IQGEN / RR_TXIQK / RR_TIA | DPK settings |
| 0xa0 / 0xaf / 0xb1 / 0xb7 / 0xc5 / 0xcf / 0xd3 / 0xd5 / 0xdd A | synth/LCK | see §3.9 |
| 0xde / 0xdf A | RR_RCKD / RR_LUTDBG | bit2 BW / bit12 TIA (DPK) |
| 0xee / 0xef A | RR_LUTWE2 / RR_LUTWE | LUT write enables (bit2 RTXBW / bit2 LOK; full 0x2 for LOK table) |
| 0x10001 D | RR_TXAGC | full TX AGC index (DPK) |
| 0x10005 D | RR_BBDC | bit0 D-die BB control select |
| 0x10055 D | RR_TXGA_V1 | bit7 TSSI track enable |

---------------------------------------------------------------------------------------------

## 5. Coex / FW interplay and the minimal safe handshake

### 5.1 What rtw89 does around RFK

`rtw89_btc_ntfy_wl_rfk(phy_map, type, state)` (coex.c:8366-8470).
* `phy_map = path[3:0] | BIT(phy)<<4 | band<<6` (coex.h:39-41, 303-316).
* Types: IQK 0, LCK 1, DPK 2, TXGAPK 3, DACK 4, RXDCK 5, TSSI 6, CHLK 7.

**START:**
1. Read the BT→WL scoreboard `MAC 0x00AC` (`rtw89_mac_get_sb`).
2. **Reject** if BT reports `RFK_RUN` (bit5) or `RFK_REQ` (bit6) (coex.c:7470-7486,
   bits coex.c:424-438).
3. The caller retries every 40 µs for up to **100 ms**. On timeout it warns "RFK notify
   timeout", sets `is_bt_iqk_timeout`, and from then on never waits again.
4. When allowed:
   1. Set WL scoreboard bit `BTC_WSCB_WLRFK = BIT(11)`.
   2. `_run_coex` → `_action_wl_rfk` (coex.c:5136-5157):
      1. Ctrl path to WL: `MAC 0x73 |= BIT(2)` (mac.c:6631-6638).
      2. GNT for **both** PHY bands: **GNT_WL = SW high, GNT_BT = SW low** (coex.c:4589-4593).
      3. BT PLT none.
      4. Coex FW policy `OFF_WL` (non-shared antenna) or `OFF_WL2` (shared antenna).
         Both are TDMA off with slot table 0xaaaaaaaa, sent by H2C (coex track).
   3. Arm a 300 ms watchdog (`coex_rfk_chk_work`). It force-clears the state if STOP never comes.

**ONESHOT_START/STOP:** only allowed (and only change state) inside START…STOP.

**STOP:** clear the scoreboard bit11, cancel the watchdog, `_run_coex` (normal policy
restores GNT).

**Which RFKs are wrapped:**
* DACK: START/STOP.
* RX DCK: START/STOP.
* IQK: START/STOP, plus ONESHOT per path.
* DPK: START/STOP.
* TSSI: ONESHOT only, so effectively unprotected.
* RCK: none.

**Scoreboard write format** (`rtw89_mac_cfg_sb`, mac.c:6602-6621, reg `R_AX_SCOREBOARD = 0x00AC`,
8852b.c:1105):
1. `prev = read32(0xAC)`.
2. `fw = (prev[30:24] & ~1) | (POWERON ? 1 : 0)`. The constants 0x81/0x80 are truncated
   to 7 bits.
3. Write `0xAC = BIT(31) | fw << 24 | drv_bits[23:0]`.
4. Then `fsleep(1000)`.

Driver bits: ACTIVE 0, ON 1, SCAN 2, …, WLRFK 11, BTLOG 14 (coex.c:697-712).

**GNT register** (8852B uses v0 `rtw89_mac_cfg_gnt`, mac.c:6489-6526): the value goes into
LTE-indirect register `0x38`:

| field | S0 bits | S1 bits |
|---|---|---|
| BT val | 15, 11 | 31, 27 |
| BT sw_en | 14, 10 | 30, 26 |
| WL val | 13, 9 | 29, 25 |
| WL sw_en | 12, 8 | 28, 24 |

WRFK state (WL = SW-hi, BT = SW-lo on both) = **0x77007700**.

LTE write (mac.c:86-100):
1. Poll `MAC 0xDAF3 bit5 == 1` (50 µs step, 50 ms timeout).
2. `MAC 0xDAF4 = val`.
3. `MAC 0xDAF0 = 0xC00F0000 | 0x38`.

**BT-A2DP courtesy:** `preserve_bt_time(30)` sleeps 30 ms between IQK/TSSI/DPK only if BT
A2DP is active.

### 5.2 TX pause

IQK, RX DCK, DPK and TSSI alignment stop all MAC scheduler TX first (`0xC348` enables → 0
via FW H2C-reg, §0.3) and wait for the RF to leave TX mode. DACK and RCK do not, because
they run before any traffic.

### 5.3 FW H2C involvement for 8852B

* **None of the FW-offloaded RFK H2Cs** (`rtw89_fw_h2c_rf_{pre_ntfy,iqk,dpk,tssi,dack,rxdck,…}`)
  are used on 8852B. They are BE-generation only (callers are rtw8922a/d and phy.c BE paths).
  `rtw89_fw_h2c_rf_ps_info` returns early for non-BE (fw.c:7260-7275).
  `WITH_RFK_PRE_NOTIFY` is an 8922A FW feature.
* The **only RFK-related H2C** is `rtw89_fw_h2c_rf_ntfy_mcc` at the end of rfk_channel
  (fw.c:7147-7205):
  * Header: cat `H2C_CAT_OUTSRC = 0x2`, class `H2C_CL_OUTSRC_RF_FW_NOTIFY = 0xa`,
    func `H2C_FUNC_OUTSRC_RF_GET_MCCCH = 0x2` (fw.h:4805-4821).
  * Payload (8852B FW does not have `RFK_NTFY_MCC_V0`; that flag is 8852C-only, fw.c:895).
    5 × le32 `{ch_0_0 = ch[0], ch_0_1 = ch[0], ch_1_0 = ch[1], ch_1_1 = ch[1], current_channel = ch[table_idx]}`.
  * For a single-channel STA this is `{ch, ch, 0, 0, ch}`.
  * FW uses it to know which RFK bank belongs to which channel (MCC). Cheap to send.
    Whether FW misbehaves without it is unknown.
* The sch-TX pause goes through FW via H2C-reg (§0.3).
* There is no C2H RFK report on 8852B.

### 5.4 Minimal safe handshake for a driver without full coex

This is a recommendation; the rtw89 behaviour it is derived from is documented above. BT is
active on the reference system (USB 0489:e123) and shares the 2.4 GHz path (BTG on S1), so at least
the following is advisable around IQK, DPK, RX DCK and TSSI alignment:

1. Read `MAC 0xAC`. If bit5 or bit6 (BT RFK run/req) is set, wait: poll every ~40 µs–1 ms
   for up to 100 ms, then proceed anyway.
2. Set WL scoreboard `WLRFK` (bit11) using the §5.1 write format, keeping your other WL bits.
3. Save the LTE `0x38` value (LTE read: poll `MAC 0xDAF3 bit5`, write
   `MAC 0xDAF0 = 0x800F0000|0x38`, read `MAC 0xDAF8` (R_AX_LTE_RDATA); mac.c:102-116,
   reg.h:3760-3762). Write `0x77007700`, and set `MAC 0x73 bit2 = 1`.
4. Stop MAC TX (H2C-reg SCH_TX_EN, or `0xC348` before FW is up). Wait for `RF 0x00[19:16] != 2`
   on both paths.
5. Run the calibration.
6. Resume TX. Restore LTE 0x38 and clear scoreboard bit11.

Things you can skip in a minimal driver:
* The FW coex policy H2Cs (`OFF_WL`/TDMA). Only needed if you run a TDMA coex policy at all.
* The 300 ms watchdog, the A2DP 30 ms delays, and `conn_rfk`.

At init (DACK/RCK/RX DCK) rtw89 also notifies. If coex is not initialised at all, the GNT
defaults are whatever power-on/`btc_init_cfg` leaves — see the coex track.

---------------------------------------------------------------------------------------------

## 6. Thermal meter

### 6.1 Raw read (`rtw8852bx_get_thermal`, cmn.c:1771-1786)

* **Path not in TSSI mode:**
  1. `RF[p] 0x42 bit19 = 1`, then `0`, then `1` (trigger).
  2. `fsleep(200 µs)`.
  3. Value = `RF[p] 0x42[6:1]` (6-bit raw code). The DPK uses the identical sequence
     (rfk.c:1876-1890).
* **Path in TSSI mode:** value = `BB 0x1c10 (+0x2000p)[29:24]`. This reads the TSSI engine's
  own thermal sample and does not disturb the RF.

rtw89 never converts the code to °C.
* DPK track scales the delta by ×1.5 (2G) / ×2.5 (5G) into pwsf units.
* The TSSI table indexes directly by the code delta.
* Thermal protection (only with a DMI quirk) compares the max code against 0x32/0x35
  (8852b.c:995, core.c:7083-7087) and sends H2C tx-duty. Out of scope.

### 6.2 Averaging (phy.c:5746-5764, core.h:5470)

`DECLARE_EWMA(thermal, 4, 4)`: 4 fractional bits, weight 1/4 (avg = ¾·avg + ¼·new). The
first sample seeds the average; zero readings are ignored. It is updated once at
`phy_stat_init` and every 2 s in track work (not while scanning). `_dpk_track` uses the average.

### 6.3 Trims written at init (after rfk_init; `power_trim`, cmn.c:335-407, 445-449)

* **Thermal trim:**
  * Physical efuse 0x5DF (A), 0x5DC (B). If both are 0xff, skip.
  * Otherwise `RF[p] 0x43[19:16] = ((raw & 1) << 3) | ((raw & 0x1f) >> 1)`.
* **PA bias trim:**
  * Physical efuse 0x5DE (A), 0x5DB (B). If both are 0xff, skip.
  * Otherwise `RF[p] 0x60[15:12] = raw[3:0]` (2G) and `RF[p] 0x60[19:16] = raw[7:4]` (5G).
  * Not thermal, but in the same routine.
* **TSSI thermal base** = logical efuse 0x2d0 / 0x2d1 (§3.8).

---------------------------------------------------------------------------------------------

## 7. Minimal viable RFK: ranking and implementation order

Facts that anchor this ranking (from the source):
1. **The chip's "uncalibrated" defaults are benign.**
   * The NCTL table sets IQC to identity (`0x40000000`) and DPD off (`0x81bc` bit24 = 0).
   * IQK/DPK failures fall back to exactly these values (§3.6.2, §3.7.7).
2. **rtw89 itself scans and associates with only init RFK plus band-change TSSI.**
   * During scan (including FW-offload scan) nobody runs IQK, DPK or RX DCK per channel.
     Only RCK, DACK and RX DCK at the power-on channel, the synth lock check at each
     `__rtw89_set_channel`, and `tssi_scan` at band change are in effect (§1.2-1.4).
   * Management-rate TX/RX across both bands therefore works with that subset.
3. **Per-channel IQK/TSSI-alignment/DPK only happen at station add** (§1.3). They target
   link quality at high MCS.

Ranking (engineering judgement for "works at reasonable throughput" on 5 GHz/80 MHz and
2.4 GHz/20 MHz):

| rank | item | necessity | why |
|---|---|---|---|
| 0 | NCTL pre-init and NCTL table (from FW element) | **mandatory** (for IQK/DPK), cheap | the microcode and KIP defaults; also sets identity IQC |
| 1 | AFE init table + MAC 0x8040 | **mandatory** | AFE register config nobody else writes |
| 1 | DACK (DRCK, ADDCK, MSBK/DADCK) | **mandatory** | TX DAC DC/linearity and ADC DC; without it expect LO/DC spurs and bad EVM on all rates |
| 1 | RCK (both paths) | **mandatory**, trivial | RF baseband filter RC corner. A wrong corner breaks the 80 MHz passband |
| 1 | RX DCK (init + per channel) | **mandatory**, trivial | RX DC offset. The ADC can saturate at high gain |
| 1 | Synth lock check (§3.9) | **mandatory**, trivial | rtw89 does it on every channel set; without it an unlocked LO = no link |
| 2 | IQK | **strongly recommended** for 80 MHz / ≥64-QAM | TX/RX image rejection and LO leakage. Without it low MCS works; 256-QAM/1024-QAM EVM targets will likely not be met. Needs NCTL |
| 3 | TSSI "scan-mode" (tables + default alignment + efuse DE + thermal table, no PMAC TX) | recommended | closed-loop power with factory (efuse) DE. rtw89 runs exactly this on band change. It keeps TX power near target instead of open-loop TXAGC |
| 4 | TSSI PMAC alignment (`alimentk`) | optional / refinement | per-channel alignment offsets. Costs over-the-air TX; ~2 × (≤3 ms) per path |
| 5 | DPK + DPD back-off init | optional (EVM at high power) | PA linearization for top MCS at full power. Disabling DPD is the rtw89 failure fallback anyway |
| 6 | DPK track, thermal EWMA | optional | only meaningful once DPK is on; compensates DPD gain over temperature |
| 6 | Thermal/PA-bias trims | recommended, trivial | factory trims; a few RF writes |
| — | TSSI tracking | nothing to do in SW | hardware, from the thermal table |
| — | LCK tracking | does not exist on 8852B | |

**Recommended implementation order:**
1. RF access (SWSI + direct window), BB/RF/NCTL table loaders (other tracks), and the NCTL
   pre-init and poll.
2. rfk_init equivalent. Order as rtw89: DPD back-off read, RCK A/B, DACK (AFE init → DRCK →
   ADDCK → reload → DACK S0/S1), RX DCK. Then power trims.
3. Channel set including the §3.9 lock check. RX DCK after every channel you intend to stay on.
4. TX-pause and RX-mode-wait helpers, plus the minimal coex handshake (§5.4).
5. IQK (LOK → WB TXK → WB RXK → restore). Validate: 0x8124 selects = 5/5, and the 0x9fe0
   fail bits are clear.
6. TSSI scan-mode (tables, thermal table, efuse DE), then full TSSI tables, then PMAC alignment.
7. DPK (AFE table, AGC state machine, IDL, fill), then DPK track driven by the thermal EWMA.
8. The `rf_ntfy_mcc` H2C after per-channel RFK (cheap; parity with rtw89).

Verification hooks for each stage: the debug readbacks rtw89 prints.
* DACK: `addck_d`, `msbk_d`, the `dack_done`/timeout flags.
* IQK: 0x9fe0 fail bits; `0x8008 bit26` per shot.
* DPK: `corr_val ≥ 170`, `|DC| ≤ 200`, final txagc within 0x2e–0x3f.
* TSSI: `0x1c18/0x3c18` report ready.

---------------------------------------------------------------------------------------------

## 8. Open questions / uncertainties

1. **NCTL table source.** rtw89 prefers the FW-file "NCTL" element over the static table.
   The two have not been diffed. Use the FW element. It is also unknown whether a mismatched NCTL
   version breaks IQK/DPK silently: there is no version check.
2. **IQC word bit1 semantics** (0x40000002 on failure) are inferred, not documented.
3. **PMAC `power[]` units** in TSSI alignment (48/20) and TSSI CW units: assumed 0.25 dB and
   0.125 dB, which fits the `(p0 − p1)·2` factor. Not verified.
4. **BTG BT-RX side effect** after TSSI alignment (the j = 1 call with `RF_ABCD` switches
   BTG BT-RX off for 2G). This looks accidental in rtw89. Whether later code
   re-applies it was not checked (`cfg_txrx_path` is called only at init/antenna change).
5. **The TSSI band-change path enables TSSI without `init_txpwr`/`dck`/`dac_gain`/`slope`
   tables** until the first rfk_channel. Presumably the BB table defaults are adequate; a
   clean driver could run the full table set (without PMAC) at band change instead.
6. **DACK S0 vs S1 completion logic** (AND vs OR) and the `_addck` S0 write to `0xc1d4[5:4]`
   look like vendor quirks. They are transcribed verbatim; whether they matter has not been tested.
7. **RF 0x20 bit8 (SEL2G) is left at 1** after a 2G IQK, because only SEL5G is cleared and
   0x20 is not backed up. It may be harmless.
8. **Whether FW needs `rf_ntfy_mcc` (GET_MCCCH)** for correct behaviour outside MCC (e.g.
   FW-offload scan or LPS) is unknown.
9. **Coex without the full coex engine.** It is unclear what GNT/TDMA state the FW/BT side
   assumes if the driver never sends the coex init/policy H2Cs; this interacts with §5.4.
   The coex track should confirm the idle GNT value to restore after RFK.
10. Runtime cost of each RFK is not measured. rtw89 only instruments TSSI alignment.

---------------------------------------------------------------------------------------------

## Appendix A — remaining TSSI tables (compact)

All entries are BB WM `(addr, mask, value)`. Path B = path A with `+0x2000` on
0x56xx/0x58xx/0x5axx/0x12xx; the shared regs (0x566c, 0x03xx, 0x00xx, 0x07xx/0x27xx,
0x06xx/0x26xx) are identical.

**tssi_sys** (tbl.c:239-257):
```
0x12a8[3:0]=5; 0x32a8[3:0]=5; 0x12bc[19:4]=0x5555; 0x32bc[19:4]=0x5555; 0x0300[31:24]=0x16;
0x0304[7:0]=0x19; 0x0314[31:16]=0x2041; 0x0318=0x2041; 0x0318=0x20012041; 0x0020[14:13]=3;
0x0024[14:13]=3; 0x0704[31:16]=0x601e; 0x2704[31:16]=0x601e; 0x0700[31:28]=4; 0x2700[31:28]=4;
0x0650[29:26]=0; 0x2650[29:26]=0
```

**tssi_sys_{a,b}_{2g,5g}**:
* A: `0x120c[7:0]=0x33/0x44`, `0x12c0[27:20]=0x33/0x44`, `0x58f8 b30=1/0`, `0x0304[15:8]=0x1e/0x1d`.
* B: `0x32c0[27:20]`, `0x320c[7:0]` (same values), `0x78f8 b30`, `0x0304[15:8]` (2G/5G).

**tssi_init_txpwr_a** (39 entries, tbl.c:297-337):
```
0x566c b12=0; 0x5800=0x003f807f; 0x580c[6:0]=0x40; 0x580c[27:8]=0x00040; 0x5810=0x59010000;
0x5814[24:0]=0x002d000; 0x5814[31:27]=0; 0x5818=0x002c1800; 0x581c[29:0]=0x1dc80280;
0x5820=0x00002080; 0x580c b28=1; 0x580c b30=1; 0x5834[29:0]=0x000115f2; 0x5838[30:0]=0x121;
0x5854[29:0]=0x000115f2; 0x5858[30:0]=0x121; 0x5860 b31=0; 0x5864[26:0]=0x00801ff;
0x5898=0; 0x589c=0; 0x58a4[7:0]=0x16; 0x58b0=0; 0x58b4[30:0]=0x0a002000; 0x58b8[30:0]=0x7628;
0x58bc[26:0]=0x7a7807f; 0x58c0[31:17]=0x3f; 0x58c4=0x0003ffff; 0x58c8[23:0]=0; 0x58c8[31:28]=0;
0x58cc=0; 0x58d0[26:0]=0x2008101; 0x58d4[7:0]=0; 0x58d4[17:9]=0xff; 0x58d4[26:18]=0x100;
0x58d8=0x8008016c; 0x58dc[16:0]=0x0807f; 0x58dc[31:20]=0x800; 0x58f0[17:0]=0x1ff; 0x58f4[19:0]=0
```

**he_tb**: `0x58a0 = 0xfe`, `0x58e4[6:0] = 0x1f`.

**dck**: `0x580c[27:16] = 0`, `0x5814[21:12] = 0xef`, `0x5814[28:27] = 0`.

**dac_gain**: `0x58b0[11:0] = 0`, `0x58b0 b11 = 1`, `0x5a00..0x5ac0` (step 4, 49 dwords) = 0.
The A table has an extra leading `0x58b0 b10 = 1`.

**slope_{2g,5g}** (A; differences are marked as 2G / 5G):
```
0x5608[26:0]=0x0801008 / 0x0201008; 0x560c[26:0]=0x0201020; 0x5610[26:0]=0x0201008;
0x5614[26:0]=0x0804008 / 0x0201008; 0x5618[26:0]=0x0201008; 0x561c[8:0]=0x008;
0x561c[31:16]=0x0808; 0x5620=0x08081e28 / 0x08081e08; 0x5624=0x08080808;
0x5628=0x08081e28 / 0x08080808; 0x562c[15:0]=0x0808; 0x581c b20=1
```

**slope_defs**: `0x5814 b11 = 1`, `0x581c b29 = 1`, `0x5814 b29 = 1`.

**align_*_all**. The common head is `0x5604 b31 = 1`, `0x5600[29:0] = 0x3f2d2721`,
`0x5604[21:0] = 0x010101`. Then the per-band values below, with `0x5638[19:0] = 0` and
`0x5644[19:0] = 0`.

| table | 0x5630 | 0x5634 | 0x563c | 0x5640 |
|---|---|---|---|---|
| A 2G | 0x01ef27af | 0x75 | 0x017f13ae | 0x6e |
| A 5G1 | 0x016037e7 | 0x6f | 0 | 0 |
| A 5G2 | 0x01f053f1 | 0x70 | 0 | 0 |
| A 5G3 | 0x01c047ee | 0x70 | 0 | 0 |
| B 2G (0x7630…) | 0x01ff2bb5 | 0x78 | 0x018f2bb0 | 0x72 |
| B 5G1 | 0x009003da | 0x69 | 0 | 0 |
| B 5G2 | 0x013027e6 | 0x69 | 0 | 0 |
| B 5G3 | 0x009003da | 0x69 | 0 | 0 |

The fields are masked `[29:0]` (0x5630/0x5634/0x563c/0x5640). B 5G3 equals B 5G1 in the
source, verbatim.

## Appendix B — power-tracking swing tables used by TSSI thermal table (table.c:14601-14651)

30 entries each (`DELTA_SWINGIDX_SIZE = 30`). The `_n` rows are negated when used (§3.8.2).

```
2ga_p : 0 1 1 1 1 1 1 1 2 2 2 2 2 2 3 3 3 3 3 3 3 4 4 4 4 4 4 5 5 5
2ga_n : 0 (all 30 zero)
2gb_p : 0 1 1 1 1 1 2 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 5 5 5 5 5 5 6 6
2gb_n : 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -2 -2
5ga_p[0]: 0 1 1 1 1 2 2 2 2 2 3 3 3 3 4 4 4 4 5 5 5 5 6 6 6 6 7 7 7 7
5ga_p[1]: 0 1 1 1 2 2 2 2 3 3 3 4 4 4 4 5 5 5 5 6 6 6 7 7 7 7 8 8 8 9
5ga_p[2]: 0 1 1 1 2 2 2 3 3 3 3 4 4 4 5 5 5 6 6 6 6 7 7 7 8 8 8 9 9 9
5ga_n[0]: 0 0 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 3 3 4 4 4
5ga_n[1]: 0 0 1 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 2 3 3 3 3 3 3
5ga_n[2]: (same as 5ga_n[1])
5gb_p[0]: 0 1 1 1 1 2 2 2 3 3 3 3 4 4 4 5 5 5 5 6 6 6 6 7 7 7 8 8 8 8
5gb_p[1]: 0 1 1 1 1 2 2 2 2 3 3 3 3 4 4 4 4 5 5 5 5 6 6 6 6 7 7 7 7 8
5gb_p[2]: 0 1 1 1 2 2 2 2 3 3 3 3 4 4 4 5 5 5 5 6 6 6 7 7 7 7 8 8 8 9
5gb_n[0]: 0 1 1 1 1 2 2 2 2 3 3 3 3 4 4 4 4 5 5 5 5 6 6 6 6 7 7 7 7 8
5gb_n[1]: 0 1 1 1 1 2 2 2 3 3 3 3 4 4 4 5 5 5 5 6 6 6 6 7 7 7 8 8 8 8
5gb_n[2]: 0 1 1 2 2 2 3 3 4 4 4 5 5 6 6 6 7 7 8 8 8 9 9 10 10 10 11 11 12 12
```

The `2g_cck_*` tables exist in table.c but are not used by 8852B RFK.
