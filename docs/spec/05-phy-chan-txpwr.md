# 05 — Baseband / RF register programming, channel switch, TX power, periodic PHY DM

Target: RTL8852B, PCIe, CV=1 (CHIP_CBV), RFE type 1, 2T2R, station mode, single channel
(no DBCC, no MCC). Source: `reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/`
(all `file:line` below are relative to that directory). RF calibration algorithms
(DACK/RCK/RX-DCK/IQK/TSSI/DPK) are track 06; element container format is track 03;
PHY-status (PPDU status) layout is track 07.

Conventions used in this document
- "BB reg X" = baseband register at BB offset X. MMIO address = `X + 0x10000` (see §1.1).
- "MAC reg X" = plain MMIO register X (no offset).
- "RF[p] reg X" = 20-bit RF register X of RF path p (A=0, B=1), accessed as in §1.2.
- `[h:l]` = bit field, `bN` = bit N. "RMW" = read-modify-write of the whole 32-bit word.
- Every BB/MAC masked write in rtw89 is a 32-bit RMW: `new = (old & ~mask) | ((val << ffs(mask)) & mask)`
  (core.h:7093). Even full-mask writes (`MASKDWORD`) read first. Negative values are
  truncated to the field width (two's complement).
- DBCC is never enabled for this chip/use case, so every `*_idx(..., phy_idx)` helper adds
  no offset (phy.c:2111, phy.c:2080). `rtw89_mac_reg_by_idx(reg, MAC_0)` = reg.

## 0. Resolved chip hooks for RTL8852B

`rtw8852b_chip_ops` (rtw8852b.c:872) / `rtw8852b_chip_info` (rtw8852b.c:957), resolved:

| hook | concrete function for 8852B |
|---|---|
| enable_bb_rf / disable_bb_rf | `__rtw8852bx_mac_enable_bb_rf` / `__rtw8852bx_mac_disable_bb_rf` (rtw8852b_common.c:2001/2036) |
| bb_preinit / bb_postinit | NULL / NULL |
| bb_reset | `rtw8852b_bb_reset` (rtw8852b.c:614) |
| bb_sethw | `__rtw8852bx_bb_sethw` (rtw8852b_common.c:1099) |
| read_rf / write_rf | `rtw89_phy_read_rf_v1` / `rtw89_phy_write_rf_v1` (phy.c:984/1190) |
| set_channel | `rtw8852b_set_channel` (rtw8852b.c:628) = mac + bb + rf parts |
| set_channel_help | `rtw8852b_set_channel_help` (rtw8852b.c:675) |
| set_txpwr / set_txpwr_ctrl / init_txpwr_unit | `__rtw8852bx_set_txpwr` / `__rtw8852bx_set_txpwr_ctrl` / `__rtw8852bx_init_txpwr_unit` (rtw8852b_common.c:1369/1381/1410) |
| power_trim | `__rtw8852bx_power_trim` (rtw8852b_common.c:445) |
| get_thermal | `__rtw8852bx_get_thermal` (rtw8852b_common.c:1771) |
| cfg_txrx_path | `__rtw8852bx_bb_cfg_txrx_path` (rtw8852b_common.c:1743) |
| ctrl_btg_bt_rx / ctrl_nbtg_bt_tx | `__rtw8852bx_ctrl_btg_bt_rx` (…:1610) / `__rtw8852bx_ctrl_nbtg_bt_tx` (…:1603) |
| query_ppdu / convert_rpl_to_rssi | `__rtw8852bx_query_ppdu` (…:1970) / `__rtw8852bx_convert_rpl_to_rssi` (…:1988) |
| rfk_init / rfk_channel / rfk_band_changed / rfk_scan / rfk_track | rtw8852b.c:697/711/731/738/746 (track 06) |
| phy_def (gen) | `rtw89_phy_gen_ax` (phy.c:8969): cr_base 0x10000, config_bb_gain = `rtw89_phy_config_bb_gain_ax`, preinit_rf_nctl = `rtw89_phy_preinit_rf_nctl_ax`, set_txpwr_{byrate,offset,limit,limit_ru} = `rtw89_phy_set_txpwr_*_ax` |
| chip constants | rf_base_addr {0xe000, 0xf000}; txpwr_factor bb=3, rf=2, mac=1; dig_table=NULL; cfo_src_fd=true; cfo_hw_comp=true; dcfo_comp={0x448C,[11:0]}; dcfo_comp_sft=10; support_fw_cmd_ofld=false; bbmcu_nr=0; ul_tb_waveform_ctrl=true |

Derived HAL flags (core.c:7072): `support_cckpd` = true for 8852B CV>CAV (our CV=1),
`support_igi` = false (only 8852A ≤CBV). `ant_diversity`=false (only 1-path chips,
mac.c:3257-3265). `tx_path_diversity`=true only if FW phycap reports tx_nss=1 with 2 antennas
(not the 2T2R case). `thermal_prot_th`=0 (no quirk) (core.c:7081).

## 1. Register access primitives

### 1.1 BB registers
- Raw IO ops are used (support_fw_cmd_ofld=false → `rtw89_raw_io`, fw.c:11610; pack/unpack = no-op).
- `phy_read32/phy_write32(addr)` = MMIO at `addr + 0x10000` (`cr_base`, phy.h `rtw89_raw_phy_write32`,
  phy.c:8970). BB space 0x0000–0xFFFF → MMIO 0x10000–0x1FFFF. No busy polling for BB accesses.
- BB MCU window (0x30000) is not used on 8852B (bbmcu_nr=0).

### 1.2 RF registers (20-bit, `RFREG_MASK = 0xFFFFF`)
RF addresses carry an "AD select" flag in bit 16 (`RTW89_RF_ADDR_ADSEL_MASK = BIT(16)`, phy.h:12).
`rtw89_phy_{read,write}_rf_v1` (phy.c:984, 1190) dispatch on it:

| RF address | mechanism | used for |
|---|---|---|
| bit16 = 1 (e.g. 0x10018, 0x10033) | **direct**: BB-mapped window | "D-die" RF control regs (rtw89 names these *_V1, e.g. `RR_CFGCH_V1 = 0x10018`, reg.h:8531) |
| bit16 = 0 (0x00–0xFF) | **SWSI** (software serial interface) through BB regs 0x370/0x374/0x378/0x174C | "A-die" RF regs (`rtw89_phy_*_rf_a`) |

Valid paths: 0,1 (else error, returns 0xFFFFFFFF = `INV_RF_DATA`).

**Direct access** (phy.c:928, 1128):
- `direct = rf_base[path] + ((addr & 0xFF) << 2)`, rf_base = {0xE000, 0xF000} → MMIO
  `0x1E000 + 4*reg` (path A) / `0x1F000 + 4*reg` (path B).
- Read: `(BB32(direct) & mask & 0xFFFFF) >> ffs(mask)`.
- Write: BB 32-bit RMW with `mask & 0xFFFFF`, then **udelay(1)**.

**SWSI read** (`rtw89_phy_read_rf_a`, phy.c:950):
1. Poll BB 0x174C (`R_SWSI_V1`) until b24 (W_BUSY) == 0 **and** b25 (R_BUSY) == 0; 1 µs step, 30 µs timeout → on timeout return 0xFFFFFFFF.
2. BB 0x0378 [10:0] (`R_SWSI_READ_ADDR_V1`) = `(path << 8) | (addr & 0xFF)` (RMW).
3. udelay(2).
4. Poll BB 0x174C b26 (R_DATA_DONE) == 1; 1 µs step, 30 µs timeout → on timeout return 0xFFFFFFFF.
5. Result = `(BB 0x174C & mask) >> ffs(mask)` (data is in [19:0]).

**SWSI write** (`rtw89_phy_write_rf_a`, phy.c:1153):
1. Busy poll as step 1 above (fail → abort write).
2. If mask != 0xFFFFF: BB 0x0374 [19:0] (`R_SWSI_BIT_MASK_V1`) = mask; `data = (data << ffs(mask)) & 0xFFFFF`; bit_mask_en = 1. Else bit_mask_en = 0.
3. BB 0x0370 (`R_SWSI_DATA_V1`) full word (via RMW with MASKDWORD) =
   `b31 bit_mask_en | [30:28] path | [27:20] addr[7:0] | [19:0] data`.
4. No completion wait (the next SWSI access waits on busy).

Bit definitions: reg.h:8806-8813, 9084. Accessor used by upper layers: `rtw89_read_rf` →
`chip->ops->read_rf`, `rtw89_write_rf` → io->write_rf → `chip->ops->write_rf` (core.h:7135-7157).

### 1.3 XTAL SI (MAC 0x0270, used for RF power switches and crystal cap)
Write (mac.c:7175): MAC 0x0270 = `b31 1(POLL) | [25:24] 0(write) | [23:16] bitmask | [15:8] data | [7:0] offset`;
poll 0x0270 b31 == 0 (50 µs step, 50 ms timeout). Read (mac.c:7205): same with mode=1, mask=0, data=0;
after poll, data = [15:8]. (Owned by track 02; listed because §2.1 and §7.5 use it.)

### 1.4 MAC TX-power registers
`rtw89_mac_txpwr_write32[_mask](phy_idx, reg)` = plain MMIO write/RMW of `reg` after a range
check (0xD200 ≤ reg ≤ CMAC1 end; mac.c:6257, mac.h:1545-1586).

## 2. Where PHY programming sits in bring-up (`rtw89_core_start`, core.c:6565)

1. `rtw89_mac_preinit` → power on (track 02).
2. `bb_preinit` — NULL for 8852B.
3. `rtw89_phy_init_bb_afe` — only acts if FW has an AFE_PWR_SEQ element (ID 27); the 8852B FW has none → no-op (phy.c:1913).
4. `rtw89_mac_init` (mac.c:4359): FW download (track 03) → **`enable_bb_rf`** (§2.1) → sys_init → trx_init → … (FW is running from here on).
5. `rtw89_btc_ntfy_poweron`.
6. **`rtw89_chip_reset_bb_rf`** (mac.h:1371): `disable_bb_rf` then `enable_bb_rf` again.
7. **`rtw89_phy_init_bb_reg`** (phy.c:1885): BB table (§3.3) → `init_txpwr_unit` (§6.9) → BB-gain table parse (§3.4, RAM only) → `bb_reset` (§3.7).
8. `bb_postinit` — NULL.
9. **`rtw89_phy_init_rf_reg(noio=false)`** (phy.c:2005): radio A table then radio B table (§3.5); after each path the collected D-die writes are sent to FW via H2C.
10. `rtw89_btc_ntfy_init` (coex; writes RF LUT/PTA — coex track).
11. **`rtw89_phy_dm_init`** (phy.c:8163), in this order:
    stat_init (thermal read, §6.10) → `bb_sethw` (§4.1) → diag init (RAM) → env-monitor init (§7.3)
    → NHM init (skipped: support_noise=false) → PHY-status IE bitmap init (§4.3) → DIG init (§7.4)
    → CFO init (§7.5) → bb_wrap/ch_info (NULL) → EDCCA init (§7.6) → UL-TB info init (§7.7)
    → antdiv init/set (skipped) → rfe_gpio / rfk_hw_init (NULL) → **NCTL table** (§3.6)
    → `rfk_init` (DPK init, RCK, DACK, RX-DCK: track 06) → `set_txpwr_ctrl` (§6.8)
    → `power_trim` (§4.4) → `cfg_txrx_path` (§4.5).
12. MAC side: edcca mode / phy_rpt hooks are NULL for AX; ppdu status on (track 07), RTS threshold.
13. HCI start; `track_work` queued with period 2 s (core.h:51).
14. Later, `rtw89_leave_ips` → `rtw89_set_channel` (ps.c:271) programs the first channel (§5). The
    RAM default channel before that is 2.4 GHz ch1/20 MHz (core.c:395), which is what
    `cfg_txrx_path` at step 11 uses.

**Ordering answer:** BB, BB-gain, RF A/B and NCTL tables are all applied **after** FW download
(FW is required for the RF H2C upload). Order: BB → txpwr unit → BB-gain → bb_reset → RF-A → RF-B
→ (coex init) → DM init … → NCTL → RF calibrations.

### 2.1 enable/disable BB+RF (rtw8852b_common.c:2001, 2036)
enable:
1. MAC 0x0002 (8-bit, `R_AX_SYS_FUNC_EN`) |= b0 (FEN_BBRSTB) | b1 (FEN_BB_GLB_RSTN).
2. MAC 0x0200 [18:17] (`R_AX_SPS_DIG_ON_CTRL0` REG_ZCDC_H) = 1.
3. MAC 0x02F0 (`R_AX_WLRF_CTRL`) b17 (AFC_AFEDIG): set, clear, set.
4. XTAL SI write offset 0x80 (`XTAL_SI_WL_RFC_S0`) = 0xC7, mask 0xFF.
5. XTAL SI write offset 0x81 (`XTAL_SI_WL_RFC_S1`) = 0xC7, mask 0xFF.
6. MAC 0x8040 (8-bit, `R_AX_PHYREG_SET`) = 0x0E (`PHYREG_SET_XYN_CYCLE`).

disable:
1. MAC 0x02F0 b17 clear. 2. MAC 0x0002 clear b0|b1.
3. XTAL SI read 0x80, clear [2:0], write back (mask 0xFF). 4. Same for 0x81.

## 3. PHY tables: formats and application

### 3.1 Sources
`init_bb_reg`/`init_rf_reg`/`init_rf_nctl` use the FW-element table if present, else the built-in
table in rtw8852b_table.c (phy.c:1892, 2019, 2072). The 8852B `-2` firmware
(`rtw8852b_fw-2.bin`, FW 0.29.29.18) contains these PHY elements (decoded from the file; header
= 32 bytes, `size` excludes header, entries = size/8 pairs `{le32 addr, le32 data}`, fw.h:4498):

| elem ID | name | entries | aid | notes |
|---|---|---|---|---|
| 2 | BB_REG | 1013 | 0 | no conditionals, no delay entries |
| 3 | BB_GAIN | 66 | 0 | no conditionals |
| 4 | RADIO_A (idx 0) | 8295 | 0 | 12 headlines + if/elif blocks |
| 5 | RADIO_B (idx 1) | 8392 | 0 | 12 headlines + if/elif blocks |
| 8 | RF_NCTL | 1320 | 0 | no conditionals |

These five FW tables are **byte-identical** to the built-in `rtw89_8852b_phy_{bb_regs,
bb_reg_gain, radioa_regs, radiob_regs, nctl_regs}` arrays (checked entry by entry). An element
is ignored if `aid != 0 && aid != hal.aid`, and the first element of a kind wins (fw.c:1118-1122).

### 3.2 Conditional-entry encoding (all table types)
Each entry is `{u32 addr, u32 data}`. `cond = addr[31:28]` (phy.h:14-29):

| addr[31:28] | meaning | other fields |
|---|---|---|
| 0xF | headline (only at table start) | target = addr[27:0] = `rfe[23:16] | pkg[15:8] | cv[7:0]` |
| 0x8 | IF | target = addr[27:0] |
| 0x9 | ELIF | target = addr[27:0] |
| 0xA | ELSE | – |
| 0xB | END | – |
| 0x4 | CHECK (follows every IF/ELIF; evaluates the pending target) | – |
| anything else | a normal register write | – |

Headline selection (`rtw89_phy_sel_headline`, phy.c:1733), with rfe = efuse RFE type (1),
cv = `hal.cv` (1) (acv is used only for 8922D):
1. `headline_size` = number of leading entries with addr[31:28]==0xF. If 0 → no conditions; start at entry 0.
2. Case 1: find headline whose addr[27:0] == `(rfe<<16)|cv` → cfg_target.
3. Case 2: … == `(rfe<<16)|0xFF` (CV don't-care).
4. Case 3: among headlines with rfe field == rfe, take the one with the largest cv field (≥ comparison, last wins on ties).
5. Case 4: among headlines with rfe field == 0xFF, largest cv.
6. None → error "invalid PHY package", table not loaded.
(Cases 1/2 compare the full 28-bit target with pkg = 0, so a headline with a non-zero pkg byte can only be picked by cases 3/4, which ignore pkg.)

Body evaluation (`rtw89_phy_init_reg`, phy.c:1815), starting after the headlines, state
`matched=true, found=false, target=0`:
- IF/ELIF: `target = addr[27:0]`.
- CHECK: if `found` → `matched=false`; else if `target == cfg_target` → `matched=true, found=true`; else `matched=false`.
- ELSE: `matched=false`; if `!found` → **abort loading the whole table** (warning "failed to load CR").
  Consequently an ELSE body is **never applied** by rtw89; every if/elif chain must enumerate the selected target.
- END: `matched=true, found=false`.
- other: apply `config(entry)` iff `matched`.

For our chip: RADIO_A/B headlines are
`0xF0010000, 0xF0020000, 0xF0010001, 0xF0020001, 0xF0030001, 0xF0040001, 0xF0050001,
0xF0060001, 0xF0070001, 0xF0080001, 0xF0290001, 0xF02B0001` → case 1 matches entry 2
(`0xF0010001`), cfg_target = **0x0010001**. Every block in both radio tables has an
`ELIF 0x90010001` arm (188 blocks in A, 190 in B), so the ELSE abort never triggers.
Evaluated result (simulated from the FW tables): radio A = 947 RF writes (294 of them D-die, addr ≥ 0x100);
radio B = 932 RF writes (294 D-die). No delay entries survive.

### 3.3 BB register table (`rtw89_phy_config_bb_reg`, phy.c:1347)
Per applied entry:
- addr 0xFE → mdelay(50); 0xFD → mdelay(5); 0xFC → mdelay(1); 0xFB → udelay(50); 0xFA → udelay(5); 0xF9 → udelay(1).
- data == 0xBABECAFE (`BYPASS_CR_DATA`) → skip.
- else: **full 32-bit write** BB[addr] = data (no mask, no RMW) (phy1 offset only for DBCC).

8852B BB table: 1013 plain writes, BB 0x0000–0xC1F8, first `{0x0704, 0x601E0100}`, last
`{0x00F8, 0x20220408}` (a date stamp). Notable defaults it leaves (useful for sanity checks):
0x49C4 = 0x00000103 (RX path AB), 0x49C0 = 0x800CD62D, 0x4884 (EDCCA lvl) = 0x38384242,
0x49B0 = 0xF8F8F418, 0x4A00 = 0xF8F8FA00, 0x12AC = 0x32AC = 0x12333121, 0x0C70 = 0x00000660,
final 0x0704 = 0x601C05FF (0x0704 is written by table entries #0, #704, #706, #880, #881, #920, #931, #940, #941, #942, #978 — apply strictly in order).

### 3.4 BB gain table (`rtw89_phy_config_bb_gain_ax`, phy.c:1577)
Not written to hardware at load time; parsed into RAM arrays used by set_channel (§5.4.3-5.4.5).
`addr` is reinterpreted (little-endian union, phy.c:1378):

| addr bits | field |
|---|---|
| [7:0] | `type`; for cfg_type 1 this byte is `rxsc_start[3:0]` + `bw[7:4]` |
| [15:8] | path (skip if ≥ 2) |
| [23:16] | gain_band: 0=2G, 1=5G-L(ch36-64), 2=5G-M(ch100-144), 3=5G-H(ch149-177) (skip if ≥ 8) |
| [31:24] | cfg_type |

Entries with addr 0xF9..0xFE warn and are skipped. `data` bytes are consumed LSB first:
- cfg_type 0 (gain error): type 0 → `lna_gain[band][path][0..3]` = data bytes 0..3; type 1 → `lna_gain[..][4..6]` = bytes 0..2; type 2 → `tia_gain[..][0..1]` = bytes 0..1.
- cfg_type 1 (RPL offset): bw 0 (20M) → `rpl_ofst_20[band][path] = (s8)data`; bw 1 (40M): rxsc_start 0 → `rpl_ofst_40[..][0]` = byte0; rxsc_start 1 → `[1],[2]` = bytes 0,1; bw 2 (80M): rxsc 0 → `rpl_ofst_80[..][0]`; rxsc 1 → `[1..4]` = 4 bytes; rxsc 9 → `[9],[10]` = 2 bytes. (160M variants unused.)
- cfg_type 2 (bypass) → `lna_gain_bypass`, cfg_type 3 → `lna_op1db` / `tia_lna_op1db`: parsed but unused by 8852B; the 8852B table has none.
- cfg_type 4: only rfe ≥ 50, else warn.

Decoded 8852B BB-gain table (hex bytes = s8 raw values; RPL offsets signed decimal):

| band | path | LNA[0..6] | TIA[0..1] | rpl20 | rpl40[0..2] | rpl80[0..4], [9],[10] |
|---|---|---|---|---|---|---|
| 0 (2G) | A | b7 dd fb 18 36 54 6f | 31 4f | -12 | -8 -8 -8 | 0 … 0 |
| 0 (2G) | B | b7 e0 fe 1b 38 52 6c | 31 50 | -8 | 0 0 0 | 0 … 0 |
| 1 (5G-L) | A | 9e c3 e6 07 26 45 65 | 50 67 | -12 | -8 -8 -8 | -8 -24 -24 8 8, -8 -8 |
| 1 | B | 9f c6 e9 09 27 46 67 | 50 67 | -12 | -8 -8 -8 | same |
| 2 (5G-M) | A | 9f c4 e8 06 26 45 65 | 50 67 | -12 | -8 -8 -8 | same |
| 2 | B | a0 c6 e9 07 28 47 67 | 50 68 | -12 | -8 -8 -8 | same |
| 3 (5G-H) | A | 9d c3 e5 04 25 43 63 | 50 67 | -12 | -8 -8 -8 | same |
| 3 | B | 9f c6 e9 06 27 45 65 | 50 67 | -12 | -8 -8 -8 | same |

### 3.5 RF radio tables (`rtw89_phy_config_rf_reg_v1`, phy.c:1713)
Both FW-element and built-in radio tables use `rtw89_phy_config_rf_reg_v1` for 8852B
(fw.c:1143-1146, rtw8852b_table.c:22871-22883 `.config`). Per applied entry:
1. addr == 0xFE → mdelay(50), done. (**Only 0xFE** is special here; 0xF9–0xFD would be written as RF regs. None occur.)
2. `write_rf(path, addr, 0xFFFFF, data)` → SWSI for addr < 0x100, direct for addr with b16.
3. If addr ≥ 0x100 (i.e. D-die `0x100xx` entries): append `le32((addr << 20) | data)` to the per-path H2C buffer
   (u32 truncation ⇒ word = `(addr & 0xFFF) << 20 | data[19:0]`, i.e. the FW receives the 8-bit reg number in [27:20]) (phy.c:1624).

After finishing a path (phy.c:2032, 1643; fw.c:7114):
- Buffer is split in pages of ≤ 500 dwords, max 3 pages (1500 dwords; overflow → -EINVAL, H2C skipped).
- Each page: H2C with cat = 2 (`H2C_CAT_OUTSRC`), class = 8 (`H2C_CL_OUTSRC_RF_REG_A`) for path A /
  9 (`..._RF_REG_B`) for path B, func = page index, rack = dack = 0, payload = the dwords.
- For our tables each path produces 294 dwords → one H2C (func 0) per path.

Purpose (inference): FW needs the D-die RF register image for FW-driven channel switching
(scan offload is enabled for this FW: `SCAN_OFFLOAD` ≥ 0.29.29.0, fw.c:874) and power-save restore.

Radio A/B write mix (effective, our chip): mostly LUT programming (RF 0x33/0x3E/0x3F, 0xEF/0xEE LUT
write enables) plus D-die 0x10033/0x1003F (140 each), 0x100EE, 0x10000/2/5/18. The full effective
sequence is reproducible with the §3.2 algorithm from either source.

### 3.6 NCTL table (`rtw89_phy_init_rf_nctl`, phy.c:2065) — during dm_init, before RF calibration
Pre-init (`rtw89_phy_preinit_rf_nctl_ax`, phy.c:2041):
1. BB 0x0C60 (`R_IOQ_IQK_DPK`) |= 0x3.
2. BB 0x0C6C (`R_GNT_BT_WGT_EN`) |= 0x1.
3. BB 0x58AC (`R_P0_PATH_RST`) |= b27; BB 0x78AC (`R_P1_PATH_RST`) |= b27.
4. BB 0x0C60 |= 0x2 (8852B/8852BT only).
5. BB 0x8000 (`R_NCTL_CFG`) = 0x8.
6. Poll: { BB 0x8080 = 0x4; udelay(1); read BB 0x8080 } until == 0x4; 10 µs step, 1000 µs timeout (error log only).
Then apply the NCTL table with the **BB** config function (§3.3): 1320 full-word writes to BB 0x8000–0xA7BC
(first `{0x8000,0x8}`, `{0x8008,0}`, `{0x8004,0xF0862966}` …). No `nctl_post_table` for 8852B.

### 3.7 Post-table steps
- `init_txpwr_unit` right after the BB table (§6.9).
- `rtw8852b_bb_reset` (rtw8852b.c:614) after the BB-gain parse:
  1. BB 0x58DC b30 = 1 (`R_P0_TXPW_RSTB` MANON); BB 0x5818 b30 = 1 (`R_P0_TSSI_TRK` EN); BB 0x78DC b30 = 1; BB 0x7818 b30 = 1.
  2. `bb_reset_all` (rtw8852b_common.c:1077): BB 0x1200 [30:28] = 7; BB 0x3200 [30:28] = 7; udelay(1);
     BB 0x0704 b1 = 1; BB 0x0704 b1 = 0; BB 0x1200 [30:28] = 0; BB 0x3200 [30:28] = 0; BB 0x0704 b1 = 1.
  3. Clear b30 of BB 0x58DC, 0x5818, 0x78DC, 0x7818.

## 4. BB init extras (dm_init and helpers)

### 4.1 bb_sethw (rtw8852b_common.c:1099)
1. BB 0x0D7C b1 = 0; BB 0x2D7C b1 = 0 (`EN_SOUND_WO_NDP`).
2. MAC 0xD36C..0xD568 step 4 (`R_AX_PWR_MACID_LMT_TABLE0..127`) = 0 (128 writes).
3. Save `offset_base` = BB 0x49B0 [7:0] (s8, S(8,4)) and `rssi_base` = BB 0x4A00 [7:0] — read **after**
   the BB table; used by §5.4.4. With the stock table: offset_base = 0x18, rssi_base = 0x00.

### 4.2 Stat init (phy.c:5829)
Initial thermal read on both paths (§6.10). No register writes other than the RF thermal trigger.

### 4.3 PHY-status IE bitmap init (`__rtw89_physts_parsing_init`, phy.c:7085)
1. BB 0x0738 (`R_PLCP_HISTOGRAM`) b3 = 1, b2 = 1 (disable status trigger by fail / by break).
2. For each PPDU-type page p (enum rtw89_phy_status_bitmap, core values 0..15, skip 9 and 16):
   register = BB `0x073C + 4*idx`, idx = p for p<9, p-1 for 10..15. RMW full word:
   - p=6 (HE_MU), 7 (VHT_MU): |= b13 (IE13).
   - p=10 (TRIG_BASE, reg 0x0760): |= b13 | b1.
   - p ≥ 11 (CCK 0x0764, LEGACY_OFDM 0x0768, HT 0x076C, VHT 0x0770, HE 0x0774): &= ~[7:4] (IE04-07);
     CCK page |= b1; HT/VHT/HE pages |= b20 (IE20).
   (monitor mode adds IE09/IE10 — out of scope.) Layout of the resulting status: track 07.

### 4.4 Power trim (rtw8852b_common.c:335-405), from phycap (physical efuse, base 0x580)
- Thermal trim: raw_A = phycap[0x5DF], raw_B = phycap[0x5DC]; if any ≠ 0xFF:
  RF[p] 0x43 [19:16] (`RR_TM2_OFF`) = `((raw & 1) << 3) | ((raw & 0x1F) >> 1)`.
- PA-bias trim: raw_A = phycap[0x5DE], raw_B = phycap[0x5DB]; if any ≠ 0xFF:
  RF[p] 0x60 [15:12] (`RR_BIASA_TXG`) = raw[3:0]; RF[p] 0x60 [19:16] (`RR_BIASA_TXA`) = raw[7:4].
(Both are SWSI A-die writes.)

### 4.5 TX/RX path config (`__rtw8852bx_bb_cfg_txrx_path`, rtw8852b_common.c:1743)
rx_path = `hal.antenna_rx` or RF_AB(3) if 0 (default); rx_nss = hal.rx_nss (2) unless rx_path ≠ AB.
For our default (AB, 2 NSS), channel = the current channel (2G ch1 at init):
1. `bb_ctrl_rx_path(AB)` (…:1655):
   - BB 0x49C4 [3:0] = 3; BB 0x49C0 [17:14] = 3; BB 0x49C0 [21:18] = 3.
   - BB 0x0D18 [9:8] = 1 (HT MCS limit); BB 0x0D18 [22:21] = 1 (VHT); BB 0x0D80 [13:6] = 4 (HE user max); BB 0x0D80 [16:14] = 1; BB 0x0D80 [25:23] = 1.
     (For single path A: 0x49C4[3:0]=1, 0x49C0 1RCCA fields=1, limits 0; path B: value 2.)
   - `set_gain_offset(subband)` (§5.4.4).
   - If band 2G and rx_path ∈ {B, AB}: `ctrl_btg_bt_rx(true)` else `(false)` (§4.6).
   - TX-power reset pulse: if rx_path == A: BB 0x58DC [31:30] = 1 then = 3; else BB 0x78DC [31:30] = 1 then = 3.
2. `ctrl_rf_mode_rx_path(AB)` (…:1710): BB 0x12AC [31:4] = 0x1233312; BB 0x12B0 [11:0] = 0x333;
   BB 0x32AC [31:4] = 0x1233312; BB 0x32B0 [11:0] = 0x333. (Single A: path B gets 0x1111111/0x111; single B: mirrored.)
3. rx_nss 2: BB 0x0D18 [9:8] = 1; [22:21] = 1; BB 0x0D80 [16:14] = 1; [25:23] = 1 (rx_nss 1 → all 0).
4. BB 0x09A4 [4:2] (`R_MAC_SEL` MOD) = 0 (MAC, not PMAC, drives TX).
TX path/NSS for normal traffic is not a BB write: it is per-MACID in the CMAC control table / TX
descriptor (tracks 04/07). `bb_cfg_tx_path` (BB 0x458C [31:28], 0x45B4 [20:17]) is only used with PMAC for TSSI (track 06).

### 4.6 BT-shared-antenna ("BTG") helpers (rtw8852b_common.c:1603-1653)
RFE 1 → coex antenna model: 2 antennas, shared, BT on BTG (rtw8852b.c:751; `rfe%2 ? 2 : 3`).
`ctrl_btg_bt_rx(en)`:

| reg | field | en=1 | en=0 |
|---|---|---|---|
| BB 0x4738 | b19 (P0 BT_SHARE) | 1 | 0 |
| BB 0x4738 | b22 (P0 BTG_PATH) | 0 | 0 |
| BB 0x476C | [31:24] (P1 G LNA6 OP1dB) | 0x20 | 0x1A |
| BB 0x4778 | [7:0] (P1 G TIA0 LNA6 OP1dB) | 0x30 | 0x2A |
| BB 0x4AA4 | b19 (P1 BT_SHARE) | 1 | 0 |
| BB 0x4AA4 | b22 (P1 BTG_PATH) | 1 | 0 |
| BB 0x0980 | [20:17] (PMAC_GNT P1) | 0x0 | 0xC |
| BB 0x49C4 | b14 (BT_SHARE) | 1 | 0 |
| BB 0x49C0 | [25:22] (ANT_RX_BT_SEG0) | 2 | 0 |
| BB 0x4420 | b31 (BT dyn DC est) | 1 | 1 |
| BB 0x0C6C | b21 (GNT_BT weight en) | 1 | 0 |

Called by `cfg_txrx_path` (above) and by coex (`_set_btg_ctrl`, coex.c:5159-5241: enable
when BT enabled & WL link not 5G-only). set_channel_bb clears the BT-share bits on every 5 GHz
switch (§5.4.8) but nothing in set_channel re-enables them for 2.4 GHz — coex does, after
`rtw89_btc_ntfy_switch_band`.

`ctrl_nbtg_bt_tx(en)` (coex "pre-AGC", rtw8852b_common.c:118-173) writes a 14-entry table:

| BB reg | mask | en=1 | en=0 |
|---|---|---|---|
| 0x46D0 | [1:0] | 0x3 | 0x0 |
| 0x4790 | [1:0] | 0x3 | 0x0 |
| 0x4AD4 | [31:0] | 0xF | 0x60 |
| 0x4AE0 | [31:0] | 0xF | 0x60 |
| 0x4688 | [31:24] | 0x80 | 0x1A |
| 0x476C | [31:24] | 0x80 | 0x1A |
| 0x4694 | [7:0] | 0x80 | 0x2A |
| 0x4694 | [15:8] | 0x80 | 0x2A |
| 0x4778 | [7:0] | 0x80 | 0x2A |
| 0x4778 | [15:8] | 0x80 | 0x2A |
| 0x4AE4 | [23:0] | 0x780D1E | 0x79E99E |
| 0x4AEC | [23:0] | 0x780D1E | 0x79E99E |
| 0x469C | [31:26] | 0x34 | 0x26 |
| 0x49F0 | [31:26] | 0x34 | 0x26 |

(Invoked by coex: `_set_wl_preagc_ctrl` (coex.c:5244-5343) and `btc_set_wl_rx_gain` (level 1 = FDD free-run → enable; level 0/2 → disable, rtw8852b_common.c:1931).)

### 4.7 Other `rtw8852bx_bb_*` helpers (PMAC/TSSI, used only by track 06)
`bb_set_plcp_tx` (HT20 MCS7 PMAC table, BB 0x4530–0x45B8, rtw8852b_common.c:14-116),
`bb_set_pmac_pkt_tx` (BB 0x0980, 0x0988, 0x0704, 0x0C3C, 0x2344, 0x09C0/0x09C4/0x09C8),
`bb_set_power` (BB 0x09A4 b16, BB 0x4594 [30:22]), `bb_tx_mode_switch`, `bb_backup/restore_tssi`.
Not needed for a normal-mode driver except as used inside TSSI calibration.

## 5. set_channel

Call chain for one channel (`__rtw89_set_channel`, core.c:531):
1. `set_channel_help(enter)` (§5.2)
2. `set_channel` = MAC part (§5.3) → BB part (§5.4) → RF part (§5.5)
3. `set_txpwr` (§6) — executed while TX is paused
4. `set_channel_help(exit)` (§5.6)
5. if first activation or band changed: `rtw89_btc_ntfy_switch_band` and `rfk_band_changed`
   (= `rtw8852b_tssi_scan`, track 06). Per-association calibration (`rfk_channel`: RX-DCK, IQK,
   TSSI, DPK) is triggered from chan.c/core.c on vif-chanctx assignment/association (track 06).

### 5.1 Derived channel parameters (core.c:401, chan.c:19-153)
From cfg80211 chandef: `primary` = control channel number; `center` = center channel
(20M: primary; 40M: primary ∓ 2; 80M: primary ∓ (2 + 4·k)); band 2G/5G.
- `pri_ch_idx` (enum rtw89_sc_offset): 20M → 0; 40M → 1 (UPPER) if f_pri > f_center else 2 (LOWER);
  80M → k = (|f_pri − f_center| − 10)/20; UPPER+2k (1,3) if above center, LOWER+2k (2,4) if below.
  So for 80M: 1 = upper-inner, 2 = lower-inner, 3 = upmost, 4 = lowest.
- `subband`: 2G→0, 5G center 36-64→1, 100-144→3, 149-177→4. gain_band (BB-gain index): 0,1,2,3 respectively.
  OFDM efuse gain-offset band: 2G→1 (2G_OFDM), 5G→2/3/4 (LOW/MID/HIGH); CCK uses index 0.
- `txsc(dbw)` (phy.c:805): if cbw == dbw or cbw == 20 → 0. cbw 40: 1 if pri > center else 2.
  cbw 80, dbw 20: pri > center ? (pri−center)>>1 : ((center−pri)>>1)+1 (→1..4); dbw 40: pri > center ? 9 : 10.

### 5.2 set_channel_help — enter (rtw8852b.c:675)
1. Stop scheduler TX (`rtw89_mac_stop_sch_tx`, mac.c:3403): save `tx_en` = MAC 0xC348 (16-bit, `R_AX_CTN_TXEN`);
   set all 16 enable bits to 0. With FW running this is an H2C-register message (mac.c:3338):
   func 5 (`SCH_TX_EN`), W0[31:16] = tx_en (0), W1[15:0] = mask 0xFFFF, W1[16] = band 0; expects C2H-reg func 4 (`TX_PAUSE_RPT`).
   Without FW: direct 16-bit RMW of 0xC348.
2. PPDU status off: MAC 0xCE40 (`R_AX_PPDU_STAT`) clear b0.
3. TSSI tracking hold, both paths: BB 0x58DC b30 = 1; BB 0x5818 b30 = 1; BB 0x78DC b30 = 1; BB 0x7818 b30 = 1.
4. ADC off: BB 0x20FC [31:24] (`R_ADC_FIFO` RST) = 0xF.
5. udelay 40.
6. `bb_reset_en(false)`: BB 0x2344 b31 (`R_RXCCA` DIS) = 1; BB 0x0C3C b9 (`R_PD_CTRL` PD_HIT_DIS) = 1;
   BB 0x1200 [30:28] = 7; BB 0x3200 [30:28] = 7; udelay(1); BB 0x0704 b1 = 0.

### 5.3 MAC part (`__rtw8852bx_set_channel_mac`, rtw8852b_common.c:451)
| bw | MAC 0xC010 [1:0] (8-bit RMW, `R_AX_WMAC_RFMOD`) | MAC 0xC088 (`R_AX_TX_SUB_CARRIER_VALUE`), full write |
|---|---|---|
| 20 | 0 | 0 |
| 40 | 1 | txsc20 |
| 80 | 2 | txsc20 \| (txsc40 << 4) |

MAC 0xC628 (8-bit, `R_AX_TXRATE_CHK`): channel > 14 → clear b4 (BAND_MODE), set b0 (CHECK_CCK_EN) | b1 (RTS_LIMIT_IN_OFDM6);
channel ≤ 14 → set b4, clear b0|b1. (Note: uses center channel.)

### 5.4 BB part (`__rtw8852bx_set_channel_bb`, rtw8852b_common.c:1167), in order

**5.4.1 CCK SCO thresholds** (2.4 GHz only, indexed by *primary* channel; …:509):
BB 0x23B0 [18:0] (`R_RXSCOBC`) = barker[pri−1]; BB 0x23B4 [18:0] (`R_RXSCOCCK`) = cck[pri−1].

| pri ch | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| barker | 1CFEA | 1D0E1 | 1D1D7 | 1D2CD | 1D3C3 | 1D4B9 | 1D5B0 | 1D6A6 | 1D79C | 1D892 | 1D988 | 1DA7F | 1DB75 | 1DDC4 |
| cck | 27DE3 | 27F35 | 28088 | 281DA | 2832D | 2847F | 285D2 | 28724 | 28877 | 289C9 | 28B1C | 28C6E | 28DC1 | 290ED |

**5.4.2 `ctrl_ch`** (…:745):
1. BB 0x4738 b17 (P0 BAND_SEL) = 1 if center ≤ 14 else 0; BB 0x4AA4 b17 (P1) = same.
2. BB 0x49C0 [6:0] (`R_FC0_BW_V1` FC0_BW_INV, "SCO compensate") = sco(center):
   center 1→109; 2-6→108; 7-10→107; 11-14→106; 36,38→51; 40-58→50; 60-64→49; 100,102→48;
   104-126→47; 128-151→46; 153-177→45; anything else → 0.
3. CCK TX FIR (BB 0x2300..0x231C step 4, mask [23:0]) — written for every band:

| reg | 0x2300 | 0x2304 | 0x2308 | 0x230C | 0x2310 | 0x2314 | 0x2318 | 0x231C |
|---|---|---|---|---|---|---|---|---|
| center == 14 | 3B13FF | 1C42DE | FDB0AD | F60F6E | FD8F92 | 02D011 | 01C02C | FFF00A |
| otherwise | 3D23FF | 29B354 | 0FC1C8 | FDB053 | F86F9A | FAEF92 | FE5FCC | FFDFF5 |

   (On 2.4 GHz these are overwritten by the TX-shape DFIR in set_txpwr, §6.7.)
4. Gain error (§5.4.3) path A, then path B; gain offset (§5.4.4); RX-subchannel RPL comp (§5.4.5).

**5.4.3 `set_gain_error(subband, path)`** (…:577): writes `lna_gain[gb][path][i]` and `tia_gain[gb][path][i]`
(gb = gain band) with 32-bit RMW byte masks. Register per path (A, B), G = 2.4 GHz set, A = 5 GHz set:

| item | 2G reg (A / B) | 5G reg (A / B) | mask |
|---|---|---|---|
| LNA0 | 0x4678 / 0x475C | 0x45DC / 0x4740 | [23:16] |
| LNA1 | 0x4678 / 0x475C | 0x45DC / 0x4740 | [31:24] |
| LNA2 | 0x467C / 0x4760 | 0x4660 / 0x4744 | [7:0] |
| LNA3 | 0x467C / 0x4760 | 0x4660 / 0x4744 | [15:8] |
| LNA4 | 0x467C / 0x4760 | 0x4660 / 0x4744 | [23:16] |
| LNA5 | 0x467C / 0x4760 | 0x4660 / 0x4744 | [31:24] |
| LNA6 | 0x4680 / 0x4764 | 0x4664 / 0x4748 | [7:0] |
| TIA0 | 0x4680 / 0x4764 | 0x4664 / 0x4748 | [23:16] |
| TIA1 | 0x4680 / 0x4764 | 0x4664 / 0x4748 | [31:24] |

Example 2G path A: 0x4678[23:16]=0xB7, [31:24]=0xDD; 0x467C = bytes FB,18,36,54 (LSB first); 0x4680[7:0]=0x6F, [23:16]=0x31, [31:24]=0x4F.

**5.4.4 `set_gain_offset(subband)`** (…:632) — efuse-dependent (values per device):
- If `comp_valid` (phycap gain comp: path A {0x5BB 2G, 0x5BA 5G-L, 0x5B9 5G-M, 0x5B8 5G-H}, path B {0x590, 0x58F, 0x58E, 0x58D}; value = signed low nibble; valid if byte ≠ 0xFF):
  BB 0x4ACC [7:0] (P0 AGC_RSVD) = clamp_s8(compA[subband] << 2); BB 0x4AD8 [7:0] (P1) = clamp_s8(compB << 2).
- If `offset_valid` (logical efuse rx gain bytes: 0x2D6 = 2G CCK, 0x2D4 = 2G OFDM, 0x2D8 = 5G low, 0x2DA = 5G mid, 0x2DC = 5G high; high nibble = path A, low nibble = path B, signed 4-bit; valid if ≠ 0xFF), with ob = OFDM band index of §5.1 and base = offset_base (§4.1):
  - `offA = −offset[A][ob]`, `offB = −offset[B][ob]`.
  - BB 0x4694 [23:16] = clamp_s8(−((offA << 2) + (base >> 2))); BB 0x4778 [23:16] = clamp_s8(−((offB << 2) + (base >> 2))).
  - `ofdm = −offset[A][ob]`, `cck = −offset[A][0]` (path B values if `antenna_rx == RF_B`).
  - BB 0x49B0 [7:0] = clamp_s8((ofdm << 4) + offset_base); BB 0x4A00 [7:0] = clamp_s8((ofdm << 4) + rssi_base).
  - 2.4 GHz only: BB 0x23AC [6:0] = clamp((cck << 3) + (offset_base >> 1), −64, 63).
- Neither valid → no writes (the 0x49B0/0x4A00 bias bytes keep BB-table values).

**5.4.5 `set_rxsc_rpl_comp(subband)`** (…:706), `avg(x) = (x[A] + x[B]) >> 1` (arithmetic):
- BB 0x49B0 and BB 0x4A00, mask [31:8]: [15:8] = avg(rpl20), [23:16] = avg(rpl40[0]), [31:24] = avg(rpl40[1]).
- BB 0x49B4 and 0x4A04 (full write): [7:0] = avg(rpl40[2]), [15:8] = avg(rpl80[0]), [23:16] = avg(rpl80[1]), [31:24] = avg(rpl80[10]).
- BB 0x49B8 and 0x4A08 (full write): [7:0] = avg(rpl80[2]), [15:8] = avg(rpl80[3]), [23:16] = avg(rpl80[4]), [31:24] = avg(rpl80[9]).
With the stock gain table: 2G → RPL1[31:8] = FC FC F6 (i.e. [15:8]=0xF6, [23:16]=0xFC, [31:24]=0xFC), RPL2 = 0x000000FC, RPL3 = 0.
5G (all three bands) → RPL1[31:8]: [15:8]=0xF4, [23:16]=0xF8, [31:24]=0xF8; RPL2 = 0xF8E8F8F8; RPL3 = 0xF80808E8.

**5.4.6 `ctrl_bw(pri_ch_idx, bw)`** (…:900):
1. Save `rx_path_0` = BB 0x49C4 [3:0].
2. Per bandwidth:

| bw | BB 0x49C0 [31:30] (FC0 BW_SET) | BB 0x49C4 [13:12] (SBW) | BB 0x49C4 [11:8] (PRICH) | BB 0x12AC & 0x32AC [23:12] | other |
|---|---|---|---|---|---|
| 20 | 0 | 0 | 0 | 0x333 | – |
| 40 | 1 | 0 | pri_ch_idx (1/2) | 0x333 | BB 0x237C b0 (CCK RXSC) = 1 if pri_ch_idx==1 (UPPER) else 0 |
| 80 | 2 | 0 | pri_ch_idx (1..4) | 0x333 | – |
| (5/10 MHz) | 0 | 1 / 2 | 0 | 0x333 | not used |

3. ADC config (`rtw8852b_bw_setting`, …:804) for path A and B: BB 0xC0EC / 0xC1EC [14:13] = 0 (20/40/80; 1 for 5M, 2 for 10M);
   BB 0xC0E4 / 0xC1E4 [5:4] = 2 (20/40/80; 0 for 5M, 1 for 10M).
4. If rx_path_0 == 1: BB 0x32AC [23:12] = 0x111; if == 2: BB 0x12AC [23:12] = 0x111.

**5.4.7 CCK enable** (…:1011): ch ≤ 14: BB 0x0700 b5 (`ENABLE_CCK`) = 1, BB 0x2344 b31 = 0; else b5 = 0, b31 = 1.

**5.4.8 5 GHz only (8852B): release BT-share** (…:1181): BB 0x4738 b19 = 0; BB 0x4738 b22 = 0; BB 0x4AA4 b19 = 0; BB 0x4AA4 b22 = 0;
BB 0x49C4 b14 = 0; BB 0x49C0 [25:22] = 0; BB 0x4420 b31 = 0; BB 0x0C6C b21 = 0.
(The 8852BT CSI-tone/spur code path is not taken for 8852B.)

**5.4.9 Channel index** (phy.c:8510): BB 0x0734 [23:16] (`R_MAC_PIN_SEL` CH_IDX_SEG0) = encode(primary, band):
2G → `0x00 | ch` (ch 1..14 → 0x01..0x0E). 5G → base table {36 (idx 2), 100 (3), 132 (4), 149 (5)}: pick the
largest base ≤ ch, value = `(idx << 4) | ((ch − base) >> 1)` (ch36→0x20, 64→0x2E, 100→0x30, 128→0x3E,
132→0x40, 144→0x46, 149→0x50, 165→0x58, 177→0x5E). The PHY status reports this index back (IE01 ch_idx, used by `query_ppdu` to fill rx freq).

**5.4.10 5 MHz-edge mask** (…:1022):
- enable if (bw 40) or (bw 80 and pri_ch_idx ∈ {3,4}); "low" if (40 and pri_ch_idx==2) or (80 and pri_ch_idx==4).
- disabled: BB 0x46F8 b12 = 0; BB 0x47B8 b12 = 0; BB 0x4440 b31 = 0.
- low: for R in {0x46F8, 0x47B8}: R[5:0] = 4, R b12 = 1, R b8 = 0, R b6 = 1; then BB 0x4440 b31 = 1.
- high: R[5:0] = 4, b12 = 1, b8 = 1, b6 = 0; BB 0x4440 b31 = 1.

**5.4.11** Monitor mode only: BB 0x47D4 b8 = 0 (pop disable). **5.4.12** `bb_reset_all` (§3.7 step 2).

### 5.5 RF part (`rtw8852b_set_channel_rf` → `rtw8852b_ctrl_bw_ch`, rtw8852b_rfk.c:4151)
RF reg 0x18 ("CFGCH", 20 bits) layout (reg.h:8530-8545):

| bits | field | value |
|---|---|---|
| [7:0] | CH | center channel number |
| [9:8] | BAND0 | 0 = 2.4 GHz, 1 = 5 GHz |
| [11:10] | BW | 3 = 20 MHz, 2 = 40 MHz, 1 = 80 MHz |
| [12] | BW2 | always set to 1 |
| [13] BCN, [14] TRX_AH, [15] POW_LCK | | always cleared |
| [17:16] | BAND1 | 0 = 2.4 GHz, 1 = 5 GHz |
| [19:18] | – | preserved from read |

Each RF18 exists twice per path: A-die `0x18` (SWSI) and D-die `0x10018` (direct, MMIO 0x1E060/0x1F060).

**Step 1 — channel** (`_ctrl_ch`, …:4107), order: (A, A-die), (B, A-die), (A, D-die), (B, D-die). For each:
1. v = read RF[p] reg (0x18 or 0x10018), full 20 bits.
2. Clear [17:13], [9:0]; set CH = center; if center > 14: BAND1 = 1, BAND0 = 1; set b12. (BW bits kept.)
3. Path A, A-die only: `_set_ch(v)` (below). Otherwise plain full write.
4. RF[p] 0xCF b0 (`RR_LCKST_BIN`) = 0, then = 1. (Also for every one of the 4 writes.)

`_set_ch(v)` / `_set_s0_arfc18` (…:3994, 4062):
1. bak = RF[A] 0xB1 (full). RF[A] 0xB1 [8:6] (`RR_LDO_SEL`) = 1.
2. RF[A] 0x18 = v (full).
3. Poll RF[A] 0xB7 b8 (`RR_LPF_BUSY`) == 0; 1 µs step, 1000 µs timeout (atomic poll).
4. RF[A] 0xB1 = bak.
5. If no timeout → `_lck_check` (…:4014):
   a. If RF[A] 0xC5 b15 (`RR_SYNFB_LK`, synth lock) == 0: RF[A] 0xD5 b8 = 1; b6 = 0; b6 = 1; b8 = 0 (MMD reset).
   b. udelay(10).
   c. If still 0: RF[A] 0xD3 b8 = 1; re-run steps 1-4 with v = RF[A] 0x18; RF[A] 0xD3 b8 = 0.
   d. If still 0: rewrite RF[A] 0xA0 with its own value; rewrite RF[A] 0xAF with its own value;
      RF[A] 0xDD b4 = 1; RF[A] 0xA0 [3:2] = 0; then = 3; RF[A] 0xDD b4 = 0;
      RF[A] 0xD3 b8 = 1; re-run steps 1-4 with v = RF[A] 0x18; RF[A] 0xD3 b8 = 0.

**Step 2 — bandwidth** (`_ctrl_bw`, …:3985), same four (path, die) combinations: read RF18; if 0xFFFFFFFF skip;
set [11:10] = 3/2/1 for 20/40/80; clear [15:13] and set b12; full write (no LCK here).

**Step 3 — RX baseband filter** (`_rxbb_bw`, …:4115), for path A and B (non-DBCC kpath = AB):
RF[p] 0xEE b2 (`RR_LUTWE2_RTXBW`) = 1; RF[p] 0x33 [4:0] = 0x12; RF[p] 0x3F [5:0] = 0x1B (20) / 0x13 (40) / 0x0B (80) (else 0x03);
RF[p] 0xEE b2 = 0.

Resulting RF18 (bits [19:18] assumed 0): 2G ch6/20 → 0x01C06; 2G center 3/40 → 0x01803; 5G ch36/20 → 0x11D24;
5G center 38/40 → 0x11926; 5G center 42/80 → 0x1152A.

### 5.6 set_channel_help — exit
1. PPDU status on: MAC 0xCE40 = 0x2B (b0 RPT_EN | b1 MAC_INFO | b3 PLCP_HDR | b5 CRC32) (full write); MAC 0x9C18 [1:0] = 1 (to host) (mac.c:6289).
2. ADC on: BB 0x20FC [31:24] = 0.
3. TSSI tracking resume: BB 0x58DC b30 = 0; 0x5818 b30 = 0; 0x78DC b30 = 0; 0x7818 b30 = 0.
4. `bb_reset_en(true)` (rtw8852b.c:590): BB 0x1200 [30:28] = 0; BB 0x3200 [30:28] = 0; BB 0x0704 b1 = 1;
   **2.4 GHz only:** BB 0x2344 b31 = 0; **always:** BB 0x0C3C b9 = 0. (On 5 GHz CCK CCA therefore stays disabled as set in §5.4.7.)
5. Resume scheduler TX with saved tx_en, mask 0xFFFF (same H2C-reg message).

### 5.7 Worked examples (all BB/MAC values; plus common steps above)

| item | 2G ch6/20 | 2G ch1+ (pri 1, center 3)/40 | 5G ch36/20 | 5G pri36 center38/40 | 5G pri36 center42/80 | 5G pri48 center42/80 |
|---|---|---|---|---|---|---|
| pri_ch_idx | 0 | 2 | 0 | 2 | 4 | 3 |
| MAC 0xC010[1:0] | 0 | 1 | 0 | 1 | 2 | 2 |
| MAC 0xC088 | 0 | 0x2 | 0 | 0x2 | 0xA4 | 0x93 |
| MAC 0xC628 b4/b1/b0 | 1/0/0 | 1/0/0 | 0/1/1 | 0/1/1 | 0/1/1 | 0/1/1 |
| BB 0x23B0 / 0x23B4 | 1D4B9 / 2847F | 1CFEA / 27DE3 | – | – | – | – |
| BB 0x4738 b17, 0x4AA4 b17 | 1 | 1 | 0 | 0 | 0 | 0 |
| BB 0x49C0 [6:0] | 108 | 108 | 51 | 51 | 50 | 50 |
| BB 0x49C0 [31:30] | 0 | 1 | 0 | 1 | 2 | 2 |
| BB 0x49C4 [11:8] | 0 | 2 | 0 | 2 | 4 | 3 |
| BB 0x237C b0 | – | 0 | – | 0 | – | – |
| BB 0x0700 b5 / 0x2344 b31 | 1/0 | 1/0 | 0/1 | 0/1 | 0/1 | 0/1 |
| BB 0x0734 [23:16] | 0x06 | 0x01 | 0x20 | 0x20 | 0x20 | 0x26 |
| 5M mask | off | low | off | low | low | high |
| RF18 | 0x01C06 | 0x01803 | 0x11D24 | 0x11926 | 0x1152A | 0x1152A |
| RF 0x3F[5:0] | 0x1B | 0x13 | 0x1B | 0x13 | 0x0B | 0x0B |

## 6. TX power

### 6.1 Units (rtw8852b.c:1004-1006, phy.h `rtw89_phy_txpwr_*`)
- BB unit: 1/8 dB (`txpwr_factor_bb = 3`); RF unit: 1/4 dB (factor 2); MAC unit: 1/2 dB (factor 1).
- By-rate and limit **tables** (FW elements / built-in) hold s8 values in **RF units (0.25 dB, i.e. dBm×4)**.
- **Registers** 0xD2C0–0xD368 hold s8 values in **MAC units (0.5 dB, dBm×2)**; conversion = arithmetic `>> 1`.
- SAR/TPE caps: `RTW89_SAR_TXPWR_MAC_MAX = 63` (31.5 dBm) when no SAR source (sar.c:287); TPE only for 6 GHz, else S8_MAX→clamped to 63.

### 6.2 Data sources & selection
- FW TXPWR elements (fw.c:1158): each has `{rsvd0, rsvd1, rfe_type, ent_sz, le32 num_ents, entries}`. An element is taken if its
  rfe_type == efuse RFE (last match wins), or if rfe_type == 0 and nothing better was taken. This FW carries sets for RFE 0, 11, 12
  only ⇒ **RFE 1 uses the rfe_type-0 set**:
  BYRATE (ID 9, ent_sz 9, 24 ents), LMT_2GHZ (10, ent 7, 2184), LMT_5GHZ (11, ent 7, 2639), LMT_RU_2GHZ (13, ent 5, 1092),
  LMT_RU_5GHZ (14, ent 5, 2184), TX_SHAPE_LMT (16, ent 4, 36), TX_SHAPE_LMT_RU (17, ent 3, 24; parsed but not used by 8852B code). No DA (dynamic-antenna-gain) tables ⇒ `has_da = false`.
- Entry layouts (fw.h:5739-5822; shorter-than-struct entries are zero-extended, longer ones accepted only if the extra bytes are 0):
  - BYRATE: `u8 band, nss, rs, shf, len; le32 data; u8 bw; u8 ofdma` → `byr[band][bw].<rs>[ofdma][nss][shf+i] = data byte i` for i < len.
  - LMT_2G/5G: `u8 bw, nt, rs, bf, regd, ch_idx; s8 v` → `lmt[bw][nt][rs][bf][regd][ch_idx] = v`.
  - LMT_RU_2G/5G: `u8 ru, nt, regd, ch_idx; s8 v`.
  - TX_SHAPE_LMT: `u8 band, tx_shape_rs, regd, v`; TX_SHAPE_LMT_RU: `u8 band, regd, v`.
- Enums: rs 0 CCK, 1 OFDM, 2 MCS (HT/VHT/HE), 3 HE-DCM, 4 OFFSET; bw 0=20, 1=40, 2=80, 3=160; nt/ntx 0=1TX, 1=2TX; bf 0 non-BF, 1 BF;
  ru 0 RU26, 1 RU52, 2 RU106; band 0 2G, 1 5G. ch_idx: 2G ch−1; 5G 36-64 → (ch−36)/2, 100-144 → (ch−100)/2+15, 149-177 → (ch−149)/2+38 (phy.c:2535).
- Regulatory index (regd.c:43, core.h:7779): country "US" → 2.4 GHz FCC (2), 5 GHz FCC (2) (FW REGD element ID 20 agrees: US = 2/2/NA(6G), fmap 0x1). Lookup rule:
  `v = lmt[..][FCC][ch_idx]; if v == 0 → v = lmt[..][WW(0)][ch_idx]` (phy.c:2604-2618; RU: phy.c:2888). Unknown country/"00" → WW.
- Antenna gain (DAG) and SAR are ACPI-driven → both 0 / no-op here (phy.c:2351, sar.c:287).

### 6.3 By-rate → MAC 0xD2C0..0xD2E8 (`rtw89_phy_set_txpwr_byrate_ax`, phy.c:3069)
Always read from `byr[band][bw=0]` with ofdma = 0; CCK always from the 2G table (phy.c:2500). Packing: 4 consecutive rate
values per dword, first value in [7:0]:

| MAC reg | content (bytes LSB→MSB) |
|---|---|
| 0xD2C0 | CCK 1, 2, 5.5, 11 |
| 0xD2C4 | OFDM 6, 9, 12, 18 |
| 0xD2C8 | OFDM 24, 36, 48, 54 |
| 0xD2CC / 0xD2D0 / 0xD2D4 | 1SS MCS0-3 / 4-7 / 8-11 |
| 0xD2D8 | 1SS HE-DCM MCS0,1,3,4 |
| 0xD2DC / 0xD2E0 / 0xD2E4 | 2SS MCS0-3 / 4-7 / 8-11 |
| 0xD2E8 | 2SS HE-DCM |

FW by-rate (RFE 0) values are identical for 2G/5G: CCK/OFDM6-36/MCS0-4 = 0x50 (20 dBm), OFDM48 0x4C, OFDM54 0x48,
MCS5-11 = 0x4C,0x48,0x44,0x40,0x3C,0x38,0x34, DCM 0x50, offsets 0. Register result (both bands):
0xD2C0 = 0x28282828, 0xD2C4 = 0x28282828, 0xD2C8 = 0x24262828, 0xD2CC = 0x28282828, 0xD2D0 = 0x22242628,
0xD2D4 = 0x1A1C1E20, 0xD2D8 = 0x28282828, 0xD2DC = 0x28282828, 0xD2E0 = 0x22242628, 0xD2E4 = 0x1A1C1E20, 0xD2E8 = 0x28282828.

### 6.4 Rate-section offsets → MAC 0xD204 [19:0] (phy.c:3126)
`byr[band][0].offset[i] >> 1` as 4-bit fields: [3:0] HE, [7:4] VHT, [11:8] HT, [15:12] OFDM, [19:16] CCK. FW values 0 ⇒ 0.

### 6.5 Limits → MAC 0xD2EC..0xD338 (phy.c:3154, 2570-2850)
Two 40-byte pages: ntx=0 (1TX) at 0xD2EC..0xD310, ntx=1 (2TX) at 0xD314..0xD338. Page layout
(`struct rtw89_txpwr_limit_ax`, phy.h:520), each item = [nonBF, BF] bytes:

| bytes | item |
|---|---|
| 0-1 | cck_20m | 
| 2-3 | cck_40m |
| 4-5 | ofdm |
| 6-21 | mcs_20m[0..7] |
| 22-29 | mcs_40m[0..3] |
| 30-33 | mcs_80m[0..1] |
| 34-35 | mcs_160m |
| 36-37 | mcs_40m_0p5 |
| 38-39 | mcs_40m_2p5 |

Dword k of a page = bytes 4k..4k+3, byte 4k in [7:0]. Every byte = `min(lmt(bw,ntx,rs,bf,ch) >> 1, 63, 63)` (rf→mac, SAR, TPE caps);
unfilled bytes = 0. Fill rules (c = center, p = primary):
- 20M: cck_20m ← (20M, CCK, c); cck_40m ← (40M, CCK, c); ofdm ← (20M, OFDM, c); mcs_20m[0] ← (20M, MCS, c).
- 40M: cck_20m ← (20M, CCK, c−2); cck_40m ← (40M, CCK, c); ofdm ← (20M, OFDM, p); mcs_20m[0] ← (20M,MCS,c−2); mcs_20m[1] ← (20M,MCS,c+2); mcs_40m[0] ← (40M,MCS,c).
- 80M: ofdm ← (20M, OFDM, p); mcs_20m[0..3] ← (20M, MCS, c−6, c−2, c+2, c+6); mcs_40m[0..1] ← (40M, MCS, c−4, c+4); mcs_80m[0] ← (80M, MCS, c);
  mcs_40m_0p5 ← min((40M,MCS,c−4), (40M,MCS,c+4)).
Examples computed from the FW tables with this algorithm (not captured from hardware):
2G ch6/20: 0xD2EC=0x00210027, 0xD2F0=0x00270027, 0xD2F4..0xD310=0; 0xD314=0x001F0025, 0xD318=0x27270027, rest 0.
5G center 42/80 (pri 36): 0xD2F0=0x00270027, 0xD2F4=0x00270027, 0xD2F8=0x00000027, 0xD300=0x00210000, 0xD304=0x00000027,
0xD308=0x00200000, 0xD310=0x00000021, others in page 0 = 0; page 1: 0xD318=0x22230022, 0xD31C=0x22232223, 0xD320=0x00002223,
0xD328=0x1F1F0000, 0xD32C=0x00002226, 0xD330=0x1C1C0000, 0xD338=0x00001F1F, rest 0.
(A table value 127 means "no limit" → 63.)

### 6.6 RU limits → MAC 0xD33C..0xD368 (phy.c:3189, 2855-3060)
Two 24-byte pages (1TX at 0xD33C..0xD350, 2TX at 0xD354..0xD368): bytes 0-7 ru26[0..7], 8-15 ru52[0..7], 16-23 ru106[0..7].
Sections: 20M → [0] at c; 40M → [0],[1] at c−2, c+2; 80M → [0..3] at c−6, c−2, c+2, c+6. Value = min(lmt_ru >> 1, 63, 63), same FCC→WW fallback.
Example 2G ch6/20: 0xD33C=0x2A, 0xD344=0x2A, 0xD34C=0x2A, 0xD354=0x25, 0xD35C=0x25, 0xD364=0x28, others 0.

### 6.7 TX shape (`rtw8852bx_set_tx_shape`, rtw8852b_common.c:1320)
Values from TX_SHAPE_LMT[band][rs][regd] (FW RFE0, FCC: 2G CCK = 1, 2G OFDM = 3, 5G OFDM = 3).
- 2.4 GHz only, CCK DFIR: 8 full-dword writes BB 0x2300 + 4i:
  - center 14 → "sharp_14": 023B13FF 001C42DE 00FDB0AD 00F60F6E 00FD8F92 0602D011 0001C02C 00FFF00A
  - shape 0 → "flat": 023D23FF 0029B354 000FC1C8 00FDB053 00F86F9A 06FAEF92 00FE5FCC 00FFDFF5
  - otherwise (FCC: 1) → "sharp": 023D83FF 002C636A 0013F204 00008090 00F87FB0 06F99F83 00FDBFBA 00003FF5
- All bands: BB 0x4494 [25:24] (`B_TXSHAPE_TRIANGULAR_CFG`) = OFDM shape (3 for FCC). (UL-TB tracking may change it, §7.7.)

### 6.8 Reference power / per-path difference (`rtw8852bx_set_txpwr_ref`, …:1235)
Called with pwr_ofst = 0 at init (`set_txpwr_ctrl`) and from set_txpwr with pwr_ofst = antenna-gain diff + SAR diff (both 0 here):
1. MAC 0xD200 [27:10] (`B_AX_PWR_REF`) = 0.
2. dec_A = pwr_ofst > 0 ? 0 : |pwr_ofst|; dec_B = pwr_ofst > 0 ? pwr_ofst : 0 (1/8 dB).
3. For each path, `val(ref=0, dec)` (…:1205): `p = (ref << 1) + (0x27 << 3) − dec`; bb_cw = p[2:0]; rf_cw = clamp(p[8:3], 15, 63);
   pwr_cw = (rf_cw << 3) | bb_cw; tssi_cw = 0x12C + (ref << 1) − 128 − dec;
   value = `tssi_cw[8:0] << 18 | pwr_cw[8:0] << 9 | ref[8:0]`.
4. Write mask [26:0] of BB 0x5804 (A, OFDM), BB 0x7804 (B, OFDM), BB 0x5808 (A, CCK), BB 0x7808 (B, CCK).
With dec = 0: **value = 0x02B27000** for all four (rf_cw = 0x27, bb_cw = 0, tssi_cw = 0xAC).

### 6.9 TX-power unit init and misc (…:1410, 1388)
`init_txpwr_unit` (during init_bb_reg): MAC 0xD248 = 0x07763333; MAC 0xD220 = 0x01EBF000; MAC 0xD240 = 0x0002F8FF; then
UL-TB offset 0: MAC 0xD288 |= b31; MAC 0xD28C [4:0] = 0; MAC 0xD290 [4:0] = max(0−3, −16) = −3 → 0x1D.
Later `cfo_init` sets MAC 0xD248 [2:0] = 6 (final 0xD248 = 0x07763336, §7.5).
Coex (`btc_set_wl_txpwr_ctrl`, rtw8852b.c:827) may program MAC 0xD200 [8:0]+b9 (force by-rate value/enable) and
MAC 0xD220 [11:3]+b1 (TX AGC while GNT_BT); both "off" (0xFFFF sentinel) by default.
Order inside `__rtw8852bx_set_txpwr`: by-rate → offset → tx shape → limit → limit_ru → ref (diff).

### 6.10 Thermal readout and power tracking
- `get_thermal(path)` (…:1771): if TSSI mode active on that path (set by TSSI calibration, track 06): BB `0x1C10 + (path << 13)` [29:24]
  (0x1C10 / 0x3C10). Otherwise: RF[p] 0x42 b19 (`RR_TM_TRI`) = 1, 0, 1; sleep 200 µs; thermal = RF[p] 0x42 [6:1].
- Power tracking on 8852B is done by the **TSSI hardware loop** with a thermal-offset table programmed by TSSI calibration
  (`_tssi_set_tmeter_tbl`, rtw8852b_rfk.c:2773, using the **built-in** `rtw89_8852b_trk_cfg` delta tables) plus DPK tracking
  (`rtw8852b_dpk_track`, called from track_work). The FW `PWR_TRK` element (ID 18, bitmap 0xFFF0) is only consumed by
  `rtw89_phy_rfk_tssi_fill_fwcmd_tmeter_tbl` (phy.c:4783) which only FW-TSSI chips (8922A/D) use — **not used for 8852B**.
  There is no driver-side periodic "TX power track" register write for 8852B other than DPK track. All of this is track 06.

## 7. Periodic PHY work (`rtw89_track_work`, core.c:5424, every 2 s)
Skipped entirely if not RUNNING or while scanning. Order: traffic stats → (LPS leave) → BF monitor → beacon track →
**phy_stat_track → env_monitor_track → DIG → rfk_track (DPK) → RA update (FW) → CFO track → tx-path-div → antdiv → UL-TB ctrl → EDCCA**
→ SAR → chanctx track → rfkill poll.

### 7.1 Stat track (phy.c:6181)
Thermal of both paths into an EWMA (RF 0x42 accesses as §6.10), thermal protection (disabled: th = 0), per-station RSSI
min/max for DIG/EDCCA (`ch_info.rssi_min`, units: raw = 2×(dBm+110)). BB counter statistics only if debugfs enables them.
**Required?** Only as input to DIG/EDCCA. Minimal: keep `rssi_min` from RX PHY status of the AP.

### 7.2 Antenna diversity / TX path diversity
Not applicable (2-path chip; flags false). Skip.

### 7.3 Environment monitor (IFS-CLM), phy.c:6212, 6406, 6947
Init:
1. BB 0x0C00 (`R_CCX`) b0 = 1 (EN), b1 = 1 (TRIG_OPT), b2 = 1 (MEASUREMENT_TRIG), [6:4] = 0 (EDCCA opt BW20_0).
2. Threshold registers with unit 32 µs (T1..T4 low/high = 0/2, 3/8, 9/32, 33/128): BB 0x0C2C [14:0]=0, [31:16]=2; BB 0x0C30 [14:0]=3, [31:16]=8;
   BB 0x0C34 [14:0]=9, [31:16]=32; BB 0x0C38 [14:0]=33, [31:16]=128.
3. BB 0x0C28 b12 (IFS collect EN) = 1; BB 0x0C2C/0x0C30/0x0C34/0x0C38 b15 (Tn EN) = 1.
Each track:
1. If BB 0x1AEC b16 (done) == 1: read counters — 0x1ACC [15:0] TX, [31:16] EDCCA-excl-CCA; 0x1AD0 [15:0] CCK-CCA-excl-FA, [31:16] OFDM;
   0x1AD4 [15:0] CCK FA, [31:16] OFDM FA; 0x1AD8 histograms; 0x1ADC/0x1AE0 averages; 0x1AE4/0x1AE8 CCA; 0x1AEC [15:0] total.
   FA permil = (cnt×1000 + period/2)/period (capped at 999), period = programmed period.
2. If BB 0x1AC8 b16: EDCCA-CLM ratio = 0x1AC8 [15:0] (percent formula as above).
3. Program period for 1900 ms (once): unit 32 µs → BB 0x0C28 [15:14] = 3, [31:16] = 59375 (0xE7EF).
4. Trigger: BB 0x0C28 b13 = 0; BB 0x0C00 b2 = 0; BB 0x0C28 b13 = 1; BB 0x0C00 b2 = 1.
**Required?** No (only feeds DIG noise level and diagnostics).

### 7.4 DIG (phy.c:7144-7890) — on 8852B only the packet-detection thresholds
`support_igi = false` ⇒ no LNA/TIA/RXB (IGI) forcing; `dig_table = NULL` never read. Registers used (rtw8852b.c:249-275):
OFDM PD: BB 0x4860 [10:6] (lower bound), b30 (spatial-reuse/enable); CCK PD: BB 0x4B74 b30 (enable), BB 0x4B64 [31:24] (lower bound, s8 dBm);
"SDAGC follow PAGC": BB 0x46E8 b5, 0x46EC b5, 0x47A8 b5, 0x47AC b5.
Init / reset (`rtw89_phy_dig_reset`, phy.c:7635): 0x4860 [10:6] = 0, b30 = 0; 0x4B74 b30 = 0; 0x4B64 [31:24] = (max(0 − under, −18) − 110) = −128 → 0x80
(igi_rssi is still 0 at init; under = 16 + 7 + bw_cmp); the four b5 bits = 0.
Track (`__rtw89_phy_dig`, phy.c:7831), per 2 s:
1. igi_rssi = linked ? rssi_min >> 1 : 22 (units dBm+110). On link state change: igi_fa_rssi = igi_rssi; FA thresholds: 2G linked {22,44,66,88}, 5G linked {4,8,12,16}, not linked {196,352,440,528} (permil).
2. Noise level from IFS-CLM FA permil (CCK+OFDM): lvl 0..4 by those thresholds; fa_rssi_ofst = (lvl==0 && prev<2) ? 0 : prev + 2·lvl, capped 25.
3. igi_min = max(igi_rssi−10, 0); dyn_max = min(igi_min+25, 90); dyn_min = max(igi_min, 12); igi_fa_rssi = clamp(igi_fa_rssi + fa_rssi_ofst, dyn_min, dyn_max) (or = dyn_max if max < min).
4. under = 16 + 7 + {20M:0, 40M:3, 80M:6}; final = min(igi_fa_rssi, igi_rssi); ofdm_th = clamp(final, 8+under, 70+under);
   **BB 0x4860 [10:6] = (ofdm_th − under − 8) >> 1; b30 = 1.** (DIG suspended: [10:6]=0, b30=0.)
5. CCK: cck_th = max(final − under, −18); **BB 0x4B74 b30 = 1; BB 0x4B64 [31:24] = cck_th − 110.**
6. SDAGC-follow bits (b5 ×4) = 1 iff igi_fa_rssi > igi_rssi, else 0.
DIG is suspended (PD lower bound 0, b30 0) during station connect and resumed after association (core.c:5693/5887), and during ROC/chanctx switches.
Note: the track also runs when not associated (igi_rssi = 22, dyn_pd_th_en = true): OFDM bound ends at 0 with b30 = 1, and the CCK
bound becomes max(min(igi_fa_rssi, 22) − 23, −18) − 110 ∈ [−121, −111] dBm with b30 = 1 (20 MHz).
**Required?** Not for basic operation (bound 0 = most sensitive; more false alarms in dense RF). Minimal version: every 2 s compute
steps 4-5 with final = (associated ? AP RSSI (dBm+110) : 22), skipping the FA-based offset, and write the OFDM/CCK PD fields; leaving the
reset values (§ init) permanently is also functional.

### 7.5 CFO / crystal tracking (phy.c:4907-5405)
Init (`rtw89_phy_cfo_init`, phy.c:5033) — **mandatory part**:
1. xcap = efuse xtal_k (logical efuse 0x2B9) & 0x7F. XTAL SI write offset 0x05 (SC_XO) = xcap, mask 0xFF; XTAL SI write 0x04 (SC_XI) = xcap, mask 0xFF; read both back.
2. BB 0x4494 b29 (`DCFO_OPT_EN`) = 1; BB 0x4490 [27:24] (`DCFO_WEIGHT`) = 8.
3. cfo_hw_comp: MAC 0xD248 [2:0] = 6.
Per-packet input: IE01 FD-CFO field, S(12,2) kHz, summed per MACID (`rtw89_phy_cfo_parse`, core.c:2118).
Track (`rtw89_phy_cfo_track` → `rtw89_phy_cfo_dm`, phy.c:5330/5244):
- Not associated → move xcap one step toward the efuse default per call (`cfo_reset`).
- No new packets → nothing. One associated STA: avg = Σcfo/Σcnt (¼ kHz); dcfo_avg = (Σcfo << 10)/Σcnt.
- If xcap hit bounds default±64 (clamped to [1,127]) → lock/reset for 15 periods.
- Hysteresis: start adjusting when |avg| > 8 (2 kHz), stop when ≤ 8. Step: |avg| > 120 or > 80 → ±3; > 40 or > 12 → ±1 (sign of avg); write new xcap as in init step 1.
- If xcap changed, dcfo_avg −= sign·(8 << 10). Then DCFO: `v = dcfo_avg/625 + sign(dcfo_avg)·BB 0x4264 [7:0]`; BB 0x448C [11:0] (`R_DCFO_COMP_S0`) = v.
- TX throughput ≥ 100 Mbps → "enhance" mode: same algorithm on a 250 ms timer until throughput ≤ 50.
**Required?** Initial xcap: yes (frequency accuracy). Tracking: recommended but optional; minimal = step-wise xcap nudging every 2 s from average CFO of beacons/data.

### 7.6 EDCCA (phy.c:8129, 8730, 8569)
Init: BB 0x0C70 [25:20] (`TX_COLLISION_T2R_ST`) = 0x29. Registers: BB 0x4884 [7:0] (EDCCA), [15:8] (EDCCA-P), [31:24] (PPDU level).
Track: th = linked ? max(rssi_min/2 − 110 + 128 − 3, 66) : 249 (field unit = dBm + 128, so 249 ≈ disabled); written to all three fields only when changed (first run writes 249).
Scan: save the three fields and write 249 to each; restore after scan (core.c:6982/7012).
BB table default is 0x38384242 (66/66/56). **Required?** Not for US station operation; minimal = leave BB-table defaults (or write 249 idle / RSSI-based when linked).

### 7.7 UL-TB control (phy.c:5498-5600), 8852B has ul_tb_waveform_ctrl
Init reads BB 0x4498 b30 (default IF band-edge). On association (`rtw89_phy_ul_tb_assoc`, phy.c:5405): def_tri_idx = BB 0x4494 [25:24]; dyn_tb_bedge_en = (5 GHz and bw ≥ 40) for our CV (CV > CBV disables it). Track (exactly one associated STA): if the received trigger-frame count of the tracking period (`stats.rx_tf_periodic`) > 100:
BB 0x4498 b30 = 0 and BB 0x4494 [25:24] = 0; if < 70: restore defaults (0x4498 b30 = saved, 0x4494 [25:24] = default triangular idx).
Power-diff variant not used (ul_tb_pwr_diff = false). **Required?** No (HE UL-OFDMA only).

### 7.8 RF-K track, RA
`rfk_track` = `rtw8852b_dpk_track` (track 06). `rtw89_phy_ra_update` = FW rate-adaptation H2C (not PHY registers).

## 8. RX PHY status / RSSI helpers (pointers; layout is track 07)
- Parsing entry: `rtw89_core_rx_parse_phy_sts` (core.c:2273), header decode `rtw89_core_update_phy_ppdu` (core.c:2231),
  IE00 CCK RPL (core.c:2124), IE01 OFDM: ch_idx, RSSI-avg-FD, SNR/EVM, CFO → `rtw89_phy_cfo_parse` (core.c:2079-2121).
- IE bitmaps that decide which IEs appear: §4.3. IE lengths table: phy.c:8973 (`physt_ie_len`).
- 8852B `convert_rpl_to_rssi` (rtw8852b_common.c:1988): `delta = rpl_avg − rssi_avg (u8)`; rssi[A,B] += delta; rssi_avg = rpl_avg.
- 8852B `query_ppdu` (…:1970): signal = (max(rssi[A], rssi[B]) >> 1) − 110 dBm; chain_signal[p] likewise;
  if PPDU valid: rx freq/band from IE01 ch_idx decoded per §5.4.9.
- Per-STA averages feed `rssi_min` (DIG/EDCCA) via `rtw89_core_rx_process_phy_ppdu_iter` (core.c:1999).

## 9. Minimal station driver checklist (this track)
Mandatory:
1. enable_bb_rf (§2.1) after FW download; optionally reset (disable+enable) before tables.
2. BB table (full writes), init_txpwr_unit, parse BB-gain table, bb_reset.
3. RF A/B tables with the §3.2 condition evaluator (cfg_target 0x0010001), SWSI/direct accessors, H2C upload of D-die entries (class 8/9).
4. bb_sethw (incl. MACID limit clear, read offset/rssi base), PHY-status IE bitmaps (needed for RSSI in RX status), CFO init (crystal cap), EDCCA init value, NCTL pre-init + table (needed before calibrations), RF calibrations (track 06), set_txpwr_ctrl (ref power), power trim, cfg_txrx_path.
5. Per channel: §5.2 → §5.3 → §5.4 → §5.5 → §6 (by-rate, offsets, tx shape, limits, RU limits, ref) → §5.6.
Optional (quality/regulatory): DIG PD thresholds, CFO tracking, EDCCA tracking, IFS-CLM, UL-TB control, coex BTG/pre-AGC switching (required only when BT is active on the shared antenna).

## 10. Open questions / uncertainties
- Exact hardware meaning of several fields is only known by rtw89 naming (e.g. 0xD200 [27:10] PWR_REF, BB 0x5804 fields, 0x4860 PD units ≈ 2 dB steps above −102 dBm by construction of the formula).
- Whether FW needs the H2C RF-reg upload when the driver does its own (non-offloaded) scanning/power-save was not verified; rtw89 always sends it.
- `bb_reset_en(true)` clears BB 0x0C3C b9 for both bands but BB 0x2344 b31 only for 2G; 5G relies on §5.4.7 — consistent, but note the asymmetry.
- Limit/RU example register values in §6.5/6.6 were computed offline from the FW element data with the documented algorithm, not read back from hardware.
- Per-device efuse values (xtal_k, rx gain offsets, gain comp, thermal/PA trim) were not read from the reference system; formulas only.
- "A-die" (SWSI, addr < 0x100) vs "D-die" (direct, addr | 0x10000) is naming inferred from rtw89 identifiers (`*_rf_a`, `dav` flag, `*_V1` regs); functionally only the access mechanism matters.
- Logical-efuse offsets quoted here (xtal_k 0x2B9, rx-gain bytes 0x2D4-0x2DC) are derived from `struct rtw8852bx_efuse` (rtw8852b_common.h) layout; cross-check with the efuse track.
