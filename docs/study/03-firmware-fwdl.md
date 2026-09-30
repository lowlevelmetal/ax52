# 03 — Firmware file format and firmware download (FWDL) — RTL8852BE (PCIe, CV_B, RFE 1)

Scope: `rtw8852b_fw-2.bin` container format (MFW + FW images + "elements"), the FW image
header, security sections, and the exact FWDL register/DMA procedure used by rtw89 for
RTL8852B over PCIe. All paths below are relative to
`reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/` unless stated.
All multi-byte fields are **little-endian**. Bit ranges are `[hi:lo]` inside a 32-bit LE dword.

Validation artefacts (read-only analysis of a decompressed copy of the real file):
- `docs/study/fwdump/rtw8852b_fw-2.bin` — `zstd -dc /usr/lib/firmware/rtw89/rtw8852b_fw-2.bin.zst`
- `docs/study/fwdump/parse_fw.py` — independent parser written from this spec
- `docs/study/fwdump/parse_fw_output.txt` — its output (`--cv 1 --rfe 1 --table-c rtw8852b_table.c`)

---------------------------------------------------------------------------------------------

## 0. Quick facts for RTL8852B (resolved indirections)

| Item | Value for 8852B | Source |
|---|---|---|
| chip_gen | AX (`RTW89_CHIP_AX`) | rtw8852b.c:959 |
| mac_def | `rtw89_mac_gen_ax` (disable_cpu=`rtw89_mac_disable_cpu_ax`, fwdl_enable_wcpu=`rtw89_mac_enable_cpu_ax`, fwdl_get_status=`rtw89_fw_get_rdy_ax`, fwdl_check_path_ready=`rtw89_fwdl_check_path_ready_ax`, fwdl_preconfig=NULL, fwdl_secure_idmem_share_mode=`rtw89_fwdl_secure_idmem_share_mode_ax`) | mac.c:7491-7501 |
| fw_basename / fw_format_max | `rtw89/rtw8852b_fw` / 2 → tries `rtw8852b_fw-2.bin`, then `-1.bin`, then `.bin` | rtw8852b.c:16-17, 963-967; fw.c:989-1000; fw.h:4582-4589 |
| fw_b_aid | 0 (no "NORMAL_B" images) | rtw8852b.c:966 |
| try_ce_fw | **true** → looks for type 5 (NORMAL_CE) first | rtw8852b.c:968; fw.c:1049-1053 |
| bbmcu_nr | 0 → no BB-MCU firmware, `include_bb=false` | rtw8852b.c:969; mac.c:4362 |
| needed_fw_elms | **0** → every element is optional (compiled-in tables used if absent) | rtw8852b.c:970; fw.c:1599-1603 |
| fw_blacklist | `rtw89_fw_blacklist_default` (ver 0, all-zero) — only used for secure boot | rtw8852b.c:971; fw.c:43-51 |
| h2c_desc_size | `sizeof(struct rtw89_txwd_body)` = **24 bytes** | rtw8852b.c:1080; core.h:1092-1099 |
| fill_txdesc_fwcmd | `rtw89_core_fill_txdesc` (AX WD body, not v1) | rtw8852b.c:914; core.c:1600 |
| FWDL part size (chunk) | **2020 bytes** fixed for AX (header field ignored/overwritten) | fw.h:287; fw.c:164-167 |
| FW header version in file | v0 (`hdr_ver`=0) with dynamic header | parse_fw_output.txt |
| hal.cv | `R_AX_SYS_CFG1 (0x00F0)[15:12]`; this machine: 1 (CHIP_CBV) | core.c:7039-7047; reg.h:195-196 |
| hal.aid | stays 0 on AX (only BE reads it) | core.c:7057-7069 |
| Selected FW | MFW entry #1: type 5 NORMAL_CE, cv 1, offset 0xCF50, size 0x4FC48, FW 0.29.29.18 | §1.3 |

---------------------------------------------------------------------------------------------

## 1. File-level format of `rtw8852b_fw-2.bin`

Decompressed size 1 554 594 (0x17B8A2) bytes. Layout:

```
0x000000  MFW header (16 B) + 5 × mfw_info (16 B each)          -> ends 0x60
0x000060  entry 0: LOGFMT (FW log format dictionary)  0xCEF0 B
0x00CF50  entry 1: NORMAL_CE cv1  (FW image, the one we load)
0x05CB98  entry 2: WOWLAN   cv1
0x09EA38  entry 3: WOWLAN   cv2
0x0E0888  entry 4: NORMAL_CE cv2
0x130480  element section (16-byte aligned) ... to EOF 0x17B8A2
```

### 1.1 MFW (multi-firmware) header — `struct rtw89_mfw_hdr` (fw.h:4307-4319)

| Offset | Size | Field | Meaning | Value in file |
|---|---|---|---|---|
| 0x00 | 1 | sig | 0xFF = MFW container (`RTW89_MFW_SIG`, fw.h:4295). Anything else ⇒ "legacy" single image, accepted only for NORMAL (fw.c:614-622) | 0xFF |
| 0x01 | 1 | fw_nr | number of `mfw_info` entries (must be ≥1) | 5 |
| 0x02 | 2 | rsvd0 | | 0 |
| 0x04 | 1 | ver.major | | 0 |
| 0x05 | 1 | ver.minor | | 0x1D (29) |
| 0x06 | 1 | ver.sub | | 0x1D (29) |
| 0x07 | 1 | ver.idx | | 0x12 (18) |
| 0x08 | 8 | rsvd1 | | 0 |
| 0x10 | 16×fw_nr | info[] | see below | |

The MFW version is used only for *early* feature detection before the image is parsed
(ver_code = `major<<24 | minor<<16 | sub<<8 | idx` = 0x001D1D12; core.h:4973-4982, fw.c:1006-1013).

`struct rtw89_mfw_info` (fw.h:4297-4305), 16 bytes each:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | cv | chip-cut the image targets (0=A-cut/CAV, 1=B-cut/CBV, 2=C-cut/CCV; core.h:199-201) |
| 1 | 1 | type | `enum rtw89_fw_type` (core.h:4888-4897): 1 NORMAL, 3 WOWLAN, 5 NORMAL_CE, 14 NORMAL_B, 15 WOWLAN_B, 64 BBMCU0, 65 BBMCU1, 255 LOGFMT |
| 2 | 1 | mp | 1 = manufacturing-test image (never selected) |
| 3 | 1 | rsvd | |
| 4 | 4 | shift | byte offset of the image from start of file |
| 8 | 4 | size | image size in bytes |
| 12 | 4 | rsvd2 | |

Entries in the real file (parse_fw_output.txt):

| idx | cv | type | mp | shift | size | end |
|---|---|---|---|---|---|---|
| 0 | 0 | 255 LOGFMT | 0 | 0x0000060 | 0x000CEF0 | 0x000CF50 |
| 1 | 1 | 5 NORMAL_CE | 0 | 0x000CF50 | 0x004FC48 | 0x005CB98 |
| 2 | 1 | 3 WOWLAN | 0 | 0x005CB98 | 0x0041EA0 | 0x009EA38 |
| 3 | 2 | 3 WOWLAN | 0 | 0x009EA38 | 0x0041E50 | 0x00E0888 |
| 4 | 2 | 5 NORMAL_CE | 0 | 0x00E0888 | 0x004FBF8 | 0x0130480 |

Note: there is **no type-1 (NORMAL) image** in this file; the "normal" firmware for 8852B is the
type-5 "CE" image, found only because `try_ce_fw = true`.

Validation rules applied by rtw89: `fw_nr != 0`, `&info[fw_nr]` inside file (fw.c:578-601);
chosen image `shift+size ≤ file size` (fw.c:657-661).

### 1.2 Image selection algorithm (`rtw89_mfw_recognize`, fw.c:603-666; `rtw89_fw_recognize`, fw.c:1038-1078)

1. Normal image: because `try_ce_fw`, first search type **5** (NORMAL_CE) with "nowarn"; only if
   none found, search type 1 (NORMAL) (fw.c:1049-1057). (`fw_b_aid==0` so types 14/15 are never used.)
2. For a given type: iterate *all* entries (order in file is not sorted), keep candidates with
   `entry.cv <= hal.cv && entry.mp == 0`, choose the one with the **largest cv** (fw.c:631-643).
3. LOGFMT (type 255): first entry with that type, cv ignored (fw.c:635-638).
4. WoWLAN (type 3) is recognised the same way but is optional (fw.c:1063-1064) and only
   downloaded on suspend (out of scope).
5. Then `rtw89_fw_update_ver` reads the image header (§2) and prints
   "Firmware version 0.29.29.18 (9e3d777f), cmd version 0, type 5" (fw.c:727-768).

Result for this chip (cv=1): **NORMAL = entry 1 (cv1, off 0xCF50, 0x4FC48 B)**, WOWLAN = entry 2,
LOGFMT = entry 0. A CV_C chip (cv=2) would get entries 4/3.

### 1.3 LOGFMT entry (`struct rtw89_fw_logsuit_hdr`, fw.h:4321-4325; fw.c:2134-2164) — optional

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | rsvd (file: 0x10) |
| 4 | 4 | count (file: 1092) |
| 8 | 4×count | ids[] — format IDs |
| … | rest | `count` NUL-separated printf format strings (leading NULs skipped) |

Used only to pretty-print C2H firmware log messages. Not needed by a minimal driver.

### 1.4 Element section

Location: `offset = ALIGN(info[fw_nr-1].shift + info[fw_nr-1].size, 16)` — rtw89 assumes the
**last** mfw_info entry is the furthest one in the file (fw.c:668-691, 1557-1560). Here
0x130480.

Iteration (fw.c:1562-1592):
```
while (offset + 32 < file_size):
    hdr = file[offset]
    if offset + hdr.size >= file_size: warn, stop           (note: header size not included)
    dispatch by hdr.id (unknown ids skipped)
    offset = ALIGN(offset + 32 + hdr.size, 16)
```
In the real file the last element ends exactly at EOF (0x17B8A2).

#### `struct rtw89_fw_element_hdr` (fw.h:4498-4560) — 32-byte header

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0x00 | 4 | id | `enum rtw89_fw_element_id` (fw.h:4329-4362) |
| 0x04 | 4 | size | payload bytes **after** this 32-byte header |
| 0x08 | 4 | ver[4] | opaque version bytes, printed as `%4ph` (e.g. "00 28 00 00") |
| 0x0C | 2 | aid | 0 = any; else must equal `hal.aid` (always 0 on AX, so only aid==0 elements are accepted) |
| 0x0E | 2 | rsvd0 | |
| 0x10 | 4 | rsvd1 | |
| 0x14 | 4 | rsvd2 | |
| 0x18 | 8 | u (head) | per-type 8-byte "union head" (see §1.5) |
| 0x20 | size | payload | per-type contents |

Element IDs (fw.h:4329-4360): 0 BBMCU0, 1 BBMCU1, 2 BB_REG, 3 BB_GAIN, 4 RADIO_A, 5 RADIO_B,
6 RADIO_C, 7 RADIO_D, 8 RF_NCTL, 9 TXPWR_BYRATE, 10 TXPWR_LMT_2GHZ, 11 TXPWR_LMT_5GHZ,
12 TXPWR_LMT_6GHZ, 13 TXPWR_LMT_RU_2GHZ, 14 TXPWR_LMT_RU_5GHZ, 15 TXPWR_LMT_RU_6GHZ,
16 TX_SHAPE_LMT, 17 TX_SHAPE_LMT_RU, 18 TXPWR_TRK, 19 RFKLOG_FMT, 20 REGD,
21-26 TXPWR_DA_LMT_{2,5,6}GHZ / _RU_{2,5,6}GHZ, 27 AFE_PWR_SEQ, 28 DIAG_MAC, 29 TX_COMP.
Handlers table: fw.c:1446-1540.

#### Elements present in the real file (parse_fw_output.txt)

| Offset | id | Name | size | ver bytes | union head / details |
|---|---|---|---|---|---|
| 0x130480 | 2 | BB_REG | 8104 | 00 28 00 00 | reg2, 1013 (addr,data) pairs, no headlines |
| 0x132450 | 3 | BB_GAIN | 528 | 00 28 00 00 | reg2, 66 pairs |
| 0x132680 | 4 | RADIO_A | 66360 | 00 32 00 00 | reg2 idx=0, 8295 pairs, 12 headlines, 4888 condition lines |
| 0x1429E0 | 5 | RADIO_B | 67136 | 00 32 00 00 | reg2 idx=1, 8392 pairs, 12 headlines, 4940 condition lines |
| 0x153040 | 8 | RF_NCTL | 10560 | 00 0a 00 00 | reg2, 1320 pairs |
| 0x1559A0 / 0x155AA0 / 0x155BA0 | 9 | TXPWR_BYRATE ×3 | 216 each | 00 43 00 00 | rfe_type 0 / 11 / 12; ent_sz 9; 24 entries |
| 0x155CA0 / 0x16A1C0 / 0x1725F0 | 10 | TXPWR_LMT_2GHZ ×3 | 15288 | 00 43 00 00 | rfe 0/11/12; ent_sz 7; 2184 entries |
| 0x159880 / 0x16DDA0 / 0x1761D0 | 11 | TXPWR_LMT_5GHZ ×3 | 18473 | 00 43 00 00 | rfe 0/11/12; ent_sz 7; 2639 entries |
| 0x15E0D0 / 0x162120 / 0x166170 | 13 | TXPWR_LMT_RU_2GHZ ×3 | 5460 | 00 43 00 00 | rfe 0/11/12; ent_sz 5; 1092 entries |
| 0x15F650 / 0x1636A0 / 0x1676F0 | 14 | TXPWR_LMT_RU_5GHZ ×3 | 10920 | 00 43 00 00 | rfe 0/11/12; ent_sz 5; 2184 entries |
| 0x17AA20 / 0x17AC20 / 0x17ACD0 | 16 | TX_SHAPE_LMT ×3 | 144 | 00 43 00 00 | rfe 0/11/12; ent_sz 4; 36 entries |
| 0x17AAD0 / 0x17AB40 / 0x17ABB0 | 17 | TX_SHAPE_LMT_RU ×3 | 72 | 00 43 00 00 | rfe 0/11/12; ent_sz 3; 24 entries |
| 0x17AD80 | 18 | TXPWR_TRK | 600 | 00 32 00 00 | bitmap 0x0000FFF0, 20 rows × 30 |
| 0x17B000 | 20 | REGD | 2178 | 00 49 00 33 | ent_sz 9; 242 countries |

All element `aid` fields are 0. No 6 GHz, DA-limit, AFE, BBMCU, RFKLOG, DIAG_MAC or TX_COMP
elements are present (8852B has no 6 GHz).

### 1.5 Element sub-formats and how rtw89 converts them

#### 1.5.1 "reg2" PHY tables: BB_REG (2), BB_GAIN (3), RADIO_A..D (4-7), RF_NCTL (8)
Handler `rtw89_build_phy_tbl_from_elm` (fw.c:1081-1156).

Union head (8 B): byte0 = `idx` (RF path index for RADIO_x: 0=A, 1=B; ignored otherwise), bytes1-7 rsvd.
Payload: `size/8` pairs of `{le32 addr, le32 data}` → converted 1:1 into `struct rtw89_reg2_def
{u32 addr; u32 data}` (core.h:4204-4207). First element of each id with matching aid wins;
duplicates ignored. For RADIO tables `rf_path` = handler arg (A/B) and `config` =
`rtw89_phy_config_rf_reg_v1` (all chips except 8852A) (fw.c:1140-1145).

**Conditional table syntax** (interpreted when the table is applied, `rtw89_phy_sel_headline` /
`rtw89_phy_init_reg`, phy.c:1733-1880; macros phy.h:13-29). `addr[31:28]` is an opcode:

| addr[31:28] | Meaning | Other fields |
|---|---|---|
| 0xF | **Headline** — only at start of table; one per supported (RFE, CV) package | target=addr[27:0]: RFE=addr[23:16], PKG=addr[15:8], CV=addr[7:0]; data = ordinal (unused) |
| 0x8 | IF — begin conditional block | target = addr[27:0] |
| 0x9 | ELIF | target = addr[27:0] |
| 0xA | ELSE | (if no branch matched so far → rtw89 aborts the table with a warning) |
| 0xB | END of conditional block | |
| 0x4 | CHECK — evaluates the preceding IF/ELIF target against the selected headline target | |
| other | ordinary write `(addr, data)` — applied only when "matched" | |

Headline selection for (rfe = efuse RFE type, cv = hal.cv), first hit wins:
1. exact target `(rfe<<16)|cv`; 2. `(rfe<<16)|0xFF`; 3. same rfe, largest cv; 4. rfe==0xFF, largest cv.
No headlines ⇒ every ordinary line is applied.

RADIO_A/B headlines in the file (RFE,CV): (1,0) (2,0) (1,1) (2,1) (3,1) (4,1) (5,1) (6,1) (7,1) (8,1)
(0x29,1) (0x2B,1). For this machine (RFE 1, CV 1) target 0x0010001 is selected; resulting applied
entries: RADIO_A 947, RADIO_B 932 (parse_fw.py emulation).

Application semantics (belong to the PHY/RF track; summarised):
- BB_REG / RF_NCTL lines → `rtw89_phy_config_bb_reg` (phy.c:1347-1376): addr 0xFE/0xFD/0xFC =
  delay 50/5/1 ms, 0xFB/0xFA/0xF9 = delay 50/5/1 µs, data 0xBABECAFE = skip; else MMIO write32 at
  `addr + 0x10000` (AX `cr_base`, phy.c:8970; phy.h:725-731).
- RADIO lines → `rtw89_phy_config_rf_reg_v1` (phy.c:1713-1731): addr 0xFE = 50 ms delay; otherwise
  RF write (20-bit mask) of `data` to RF reg `addr` on the path (addr bit16 = ADSEL flag,
  phy.h:11); lines with addr ≥ 0x100 are also packed as `(addr<<20)|data` into RF_REG H2Cs
  (class 8 path A / 9 path B, up to 3 pages × 500 words) (phy.c:1624-1669, fw.c:7114-7145).
- BB_GAIN lines → `rtw89_phy_config_bb_gain_ax` (phy.c:1577-1621): the *addr* is a packed
  descriptor: byte0 = type (or rxsc_start[3:0]/bw[7:4]), byte1 = RF path, byte2 = gain band,
  byte3 = cfg_type (0 gain error, 1 RPL offset, 2 LNA bypass gain, 3 op1dB, 4 eFEM only).
- NCTL is applied after a preinit poll (phy.c:2041-2077).

Validation: all five PHY element tables are **byte-for-byte identical** to the compiled-in
tables in rtw8852b_table.c (BB 1013, BB_GAIN 66, RADIO_A 8295, RADIO_B 8392, NCTL 1320 pairs;
parse_fw_output.txt). Precedence: element table if present, else `chip->*_table`
(phy.c:1892, 1905, 2021-2022, 2073).

#### 1.5.2 TX-power family: ids 9-17 (and 21-26 DA variants)
Handler `rtw89_fw_recognize_txpwr_from_elm` (fw.c:1158-1204).

Union head `struct __rtw89_fw_txpwr_element` (fw.h:4403-4410):

| Byte | Field |
|---|---|
| 0 | rsvd0 |
| 1 | rsvd1 |
| 2 | rfe_type — 0 = default/fallback |
| 3 | ent_sz — size of one entry in *this file* |
| 4-7 | num_ents (le32) |

Payload = `num_ents × ent_sz` bytes (size field must equal that; all do in the file).

Selection per element id (fw.c:1180-1201): an element whose `rfe_type == efuse.rfe_type` always
replaces the current choice (last exact match wins); an rfe_type 0 element is taken only if
nothing is chosen yet or the current choice is also rfe 0. For RFE 1 there is no exact match,
so the **rfe_type 0** instance of every id is used (parse_fw_output.txt).

Entries are copied with forward/backward compatibility (core.h:4327-4342; fw.c:10990-11007): if
`ent_sz < native size`, the missing trailing fields read as 0; if `ent_sz > native size`, the
extra bytes must be all-zero or the entry is dropped. Each entry is range-checked and silently
skipped if out of range. Conversion happens in `rtw89_load_rfe_data_from_fw` (fw.c:11397-11490),
called from `rtw89_core_setup_rfe_parms` (core.c:7090-7116); it overrides the corresponding
pointers of `rtw89_8852b_dflt_parms`.

Entry layouts (fw.h:5739-5826); enums in core.h:334-339, 706-790, 945-958, 995-1005:

| Element | Native size (file ent_sz) | Bytes | Stored into |
|---|---|---|---|
| TXPWR_BYRATE (9) | 11 (file 9: `bw`,`ofdma` absent ⇒ 0) | 0 band (0=2G,1=5G) · 1 nss (0=1SS..) · 2 rs (0 CCK,1 OFDM,2 MCS,3 HEDCM,4 OFFSET) · 3 shf (first rate index) · 4 len (1-4 values) · 5-8 data (le32: `len` s8 values, LSB first) · 9 bw · 10 ofdma | `rtwdev->byr[band][bw]` rate slots `shf..shf+len-1` (fw.c:11053-11078) |
| TXPWR_LMT_2GHZ (10) / 5GHZ (11) | 7 (7) | bw (0=20,1=40[,2=80,3=160]) · nt (0=1TX,1=2TX) · rs (0 CCK,1 OFDM,2 MCS) · bf (0/1) · regd (enum rtw89_regulation_type 0-15) · ch_idx (2G 0-13; 5G 0-52) · v (s8) | `v[bw][nt][rs][bf][regd][ch_idx]` (fw.c:11108-11119, 11148-11159) |
| TXPWR_LMT_RU_2GHZ (13) / 5GHZ (14) | 5 (5) | ru (0 RU26,1 RU52,2 RU106,…7) · nt · regd · ch_idx · v (s8) | `v[ru][nt][regd][ch_idx]` (fw.c:11226-11272) |
| TX_SHAPE_LMT (16) | 4 (4) | band · tx_shape_rs (0 CCK,1 OFDM) · regd · v (u8) | `v[band][rs][regd]` (fw.c:11332-11343) |
| TX_SHAPE_LMT_RU (17) | 3 (3) | band · regd · v (u8) | `v[band][regd]` (fw.c:11363-11376) |

Units / semantics of the values belong to the TX-power track.

#### 1.5.3 TXPWR_TRK (18) — thermal power-tracking deltas
Handler `rtw89_build_txpwr_trk_tbl_from_elm` (fw.c:1206-1275). Union head: `le32 bitmap`
(bit n = `enum rtw89_fw_txpwr_trk_type` n, fw.h:4421-4448), `le32 rsvd`. Payload: for every set
bit in ascending order, rows of `DELTA_SWINGIDX_SIZE` = 30 s8 (core.h:60): 6 GHz types (0-3) 4 rows,
5 GHz types (4-7: 5GB_N,5GB_P,5GA_N,5GA_P) 3 rows (one per 5G sub-band), 2 GHz types (8-15:
2GB_N,2GB_P,2GA_N,2GA_P,2G_CCK_B_N,2G_CCK_B_P,2G_CCK_A_N,2G_CCK_A_P) 1 row. The bitmap must
contain all types of the supported bands (8852B: 0xFFF0). File: bitmap 0xFFF0 → 4×3+8×1 = 20 rows
= 600 B ✓.
**8852B does not use this element**: rtw8852b_rfk.c uses the compiled-in `rtw89_8852b_trk_cfg`
(rtw8852b_rfk.c:2800-2823); the element pointer is consumed only by 8852C/8852BT/BE code
(rtw8852c_rfk.c:2995, rtw8852bt_rfk.c:2877, phy.c:4788).

#### 1.5.4 REGD (20) — country → regulation map
Handler `rtw89_recognize_regd_from_elm` (fw.c:1350-1395). Union head `__rtw89_fw_regd_element`
(fw.h:4412-4419): bytes 0-2 rsvd, byte 3 ent_sz, bytes 4-7 num_ents (≤255 else element ignored).
Entry `struct rtw89_fw_regd_entry` (fw.h:5729-5736), 9 bytes: `alpha2[0]`, `alpha2[1]`,
`rule_2ghz`, `rule_5ghz`, `rule_6ghz` (enum rtw89_regulation_type; ≥16 → NA=4), `le32 fmap`
(bit0 TAS, bit1 DAG; core.h:5842-5847). Shorter entries get defaults (rules NA, fmap 0)
(fw.c:1301-1340). Later REGD elements replace earlier ones. Used instead of the compiled-in
`rtw89_regd_map` (regd.c:708-721). File: 242 countries, e.g. `AR 9/9/4`, `BR 2/2/4`, `CL 10/10/10`.

#### 1.5.5 Other ids (not in this file; for completeness)
BBMCU0/1 (0/1): head byte0 = cv, payload = a BB-MCU FW image (fw.c:784-806) — BE chips only.
RFKLOG_FMT (19): head `nr, rsvd[3], rfk_id, rsvd[3]`, payload `le16 offset[]` (fw.c:1277-1299).
AFE_PWR_SEQ (27): payload of 24-byte `{action,cat,class,addr,mask,val}` records (fw.h:4533-4541).
DIAG_MAC (28), TX_COMP (29): BE-only.

### 1.6 What a new 8852B driver needs from the element section
- Mandatory: nothing (needed_fw_elms = 0). Recommended: take BB/BB_GAIN/RADIO/NCTL from the
  file (identical to rtw89's built-in tables, avoids shipping 700 KB of tables), TXPWR (rfe-0
  variants for RFE 1) and REGD from the file.
- Skippable: TXPWR_TRK (unused by 8852B code path), LOGFMT.
- Elements are parsed after efuse is read (needs `rfe_type`): rtw89 order is FWDL → efuse/phycap →
  `rtw89_fw_recognize_elements` (core.c:7299-7313).

---------------------------------------------------------------------------------------------

## 2. FW image header (inside the selected MFW entry)

### 2.1 v0 header — `struct rtw89_fw_hdr` (fw.h:605-634), 32 bytes + sections

`hdr_ver` = w3[31:24]; 0 → v0 parser (fw.c:140-236), 1 → v1 (fw.c:443-537). The 8852B file is v0.

| DW | Bits | Field | Used by rtw89 | Value (entry 1) |
|---|---|---|---|---|
| w0 | 31:0 | (not parsed by rtw89) | no — consumed by on-chip ROM | 0x88520102 (entry 4 / cv2: 0x88520103). Observation only: [31:16]=0x8852, [7:0] = cv+1 |
| w1 | 7:0 | major | version | 0 |
| w1 | 15:8 | minor | | 29 |
| w1 | 23:16 | sub | | 29 |
| w1 | 31:24 | idx (sub index) | | 18 → **0.29.29.18** |
| w2 | 31:0 | commit id | log only | 0x9E3D777F |
| w3 | 23:16 | LEN = total header length incl. dynamic header (only used if DYN_HDR) | yes | 0x90 = 144 |
| w3 | 31:24 | HDR_VER | yes | 0 |
| w3 | 15:0 | (undocumented) | no | 0x1020 |
| w4 | 7:0 / 15:8 / 23:16 / 31:24 | build month / day / hour / minute | log | 4 / 9 / 20 / 13 |
| w5 | 31:0 | build year | log | 2026 |
| w6 | 15:8 | SEC_NUM (≤ 10, FWDL_SECTION_MAX_NUM) | yes | 3 |
| w6 | 23:16 | (undocumented) | no | 0x50 (WoWLAN images: 0x80) |
| w7 | 15:0 | PART_SIZE (chunk size) — overwritten by driver with 2020 on AX before sending | yes (written) | 0x07E4 = 2020 |
| w7 | 16 | DYN_HDR — a dynamic header follows the section table | yes | 1 |
| w7 | 21:18 | IDMEM_SHARE_MODE (only written to HW when secure boot) | yes | 0 |
| w7 | 31:24 | CMD_VERSION | log | 0 |

Section table: `SEC_NUM × 16 B` right after w7 (offset 0x20). `base_hdr_len = 32 + 16·SEC_NUM`
= 80. If DYN_HDR: `hdr_len = w3.LEN` (144) and `dynamic_hdr_len = hdr_len − base_hdr_len` (64);
the dynamic header starts at `base_hdr_len` with `{le32 hdr_len, le32 section_count}` and rtw89
only checks that its `hdr_len` equals 64 (fw.c:167-178; file: 64 ✓, section_count 1). **The
dynamic header is never sent to the chip.** Section payloads start at `image + hdr_len`, in
table order, contiguous. Parser requires `hdr_len + Σ(section len + mssc_len) == image size`
(fw.c:230-233; true for all 4 images in the file).

### 2.2 v0 section header — `struct rtw89_fw_hdr_section` (fw.h:590-603), 16 bytes

| DW | Bits | Field | Meaning |
|---|---|---|---|
| s0 | 31:0 | DL_ADDR | chip load address (rtw89 stores `& 0x1FFFFFFF` but **never uses it**; the ROM reads it from the header) |
| s1 | 23:0 | SEC_SIZE | payload bytes |
| s1 | 27:24 | SECTIONTYPE | 9 = security section (`FWDL_SECURITY_SECTION_TYPE`, fw.h:575); others not interpreted by rtw89 |
| s1 | 28 | CHECKSUM | if 1, an extra 8-byte checksum follows the payload (len += 8) |
| s1 | 29 | REDL | "re-download" flag — parsed, unused by rtw89 |
| s1 | 31:30 | (part of METADATA [31:24]) | undocumented; 1 on code sections in file |
| s2 | 31:0 | MSSC | security sections only: number of 512-byte signatures appended after the payload; low byte 0xFF = "formatted" MSS key pool |
| s3 | 31:0 | rsvd | |

Sections of the selected image (entry 1):

| # | type | s0 raw (masked) | size | chk | redl | file offset | FWDL packets (2020 B) |
|---|---|---|---|---|---|---|---|
| 0 | 2 | 0xB8970000 (0x18970000) | 0x4BF40 = 311104 | 0 | 0 | 0xCFE0 | 155 (last 24 B) |
| 1 | 1 | 0xB8E12400 (0x18E12400) | 0x3478 = 13432 | 0 | 0 | 0x58F20 | 7 (last 1312 B) |
| 2 | **9 (security)** | 0xB8E11928 (0x18E11928) | 0x800 = 2048, MSSC=0 | 0 | 0 | 0x5C398 | 2 (last 28 B) |

Total: 1 header packet + 164 data packets, 326 584 payload bytes.

### 2.3 v1 header (for completeness; not used by 8852B) — fw.h:636-687
48-byte header (w0..w11) + 16-byte sections. Differences: w3[23:16] cmd version (rtw89 actually reads
it from w7, fw.c:723), w5[15:0] year, **w5[31:16] HDR_SIZE** (total header length when DYN_HDR),
w6[24] DSP_CHKSUM (adds 8 B per signature), section s2[7:0] MSSC, s2[27:24] BBMCU_IDX, DL_ADDR
not masked. Before sending, sections marked `ignore` are removed from the table and SEC_NUM patched
(fw.c:1672-1699).

### 2.4 Version-derived feature flags (fw.c:853-961; enum core.h:4906-4953)

Evaluated with the NORMAL image's ver_code 0x001D1D12 (fw.c:963-973) and also early with the MFW
header version (identical here). RTL8852B rows (fw.c:871-883):

| Cond | Version | Feature | 0.29.29.18 |
|---|---|---|---|
| ≥ | 0.29.26.0 | NO_LPS_PG | **set** |
| ≥ | 0.29.26.0 | TX_WAKE | **set** |
| ≥ | 0.29.29.0 | CRASH_TRIGGER_TYPE_0 | **set** |
| ≥ | 0.29.29.0 | SCAN_OFFLOAD | **set** |
| ≥ | 0.29.29.7 | BEACON_FILTER | **set** |
| ≥ | 0.29.29.15 | BEACON_LOSS_COUNT_V1 | **set** |
| < | 0.29.30.0 | NO_WOW_CPU_IO_RX | **set** |
| ≥ | 0.29.127.0 | LPS_DACK_BY_C2H_REG | – |
| ≥ | 0.29.127.0 | SER_L1_BY_EVENT | – |
| ≥ | 0.29.128.0 | CRASH_TRIGGER_TYPE_1 | – |
| ≥ | 0.29.128.0 | SCAN_OFFLOAD_EXTRA_OP | – |
| ≥ | 0.29.128.0 | BEACON_TRACKING | – |
| ≥ | 0.29.130.0 | SIM_SER_L0L1_BY_HALT_H2C | – |

(SCAN_OFFLOAD + BEACON_FILTER also enable real channel contexts in mac80211, core.c:7556-7558.)

### 2.5 Security sections, secure boot and MSS

Efuse source (`rtw89_efuse_read_fw_secure_ax`, efuse.c:484-521), read once at first power-on
before probe completes (mac.c:1557-1561): **physical** efuse bytes b1 = 0x5EC, b2 = 0x5ED
(EFUSE_EXTERNALPN_ADDR_AX).
- b1 = b2 = 0xFF → `secure_boot = false` (nothing else happens).
- MSS v0 index (efuse.c:458-482): externalPN = 0xFF − b1; customer = 0xF − b2[3:0];
  serialNum = 0x7 − b2[6:4]; if (externalPN,customer,serialNum) ∈ {(0,0,0)→idx 0, (0,1,1)→idx 1}
  then `mss_idx` = idx, `can_mss_v0`.
- MSS v1 info (efuse.c:427-456): dev_type = b1[3:0] ∈ {0xC→0, 0xA→1, 0x9→2, 0x6→3, 0xF→DEF};
  cust_idx = 0x1F − (b1[7:4] | b2[6]<<4); key_num = 0xF − b2[3:0]. Special case 8852B with
  b1=0xFF, b2=0x6E → dev_type 0xA (idx 1), cust 0, key 0.
- `secure_boot = can_mss_v0 || can_mss_v1`.

Security section processing (type 9; fw.c:404-441):
- Non-formatted (MSSC low byte ≠ 0xFF): `mssc_len = MSSC × 512` (+8 each if DSP checksum); if
  secure boot, requires `mss_idx < MSSC`, key = `payload_end + mss_idx×512`, len 512.
- Formatted (MSSC low byte = 0xFF, v1 only in practice): a key pool header
  `rtw89_fw_mss_pool_hdr` ("MSSKPOOL" signature, remap bitmap, key table) follows the payload
  (fw.h:689-730; fw.c:238-362); key index computed from (dev_type, cust_idx, key_num).
- Secure boot + 8852B + v0: the security section's transmitted length is overridden to **960**
  bytes and the header's SEC_SIZE is patched to 960; the selected 512-byte key is copied over the
  **last 512 bytes of that single 960-byte packet** (fw.c:208-209, 1660-1666, 1797-1827). Also
  `R_AX_WCPU_FW_CTRL[27:24] = idmem_share_mode, bit23 = 1` before download (mac.c:7417-7428).

**Conclusion for this file / this machine:** every image carries one type-9 section of 2048 B with
MSSC = 0 (no signatures appended). With `secure_boot = true`, rtw89's v0 parser would fail
(`mss_idx (≥0) >= mssc (0)` → -EFAULT, fw.c:423-428), so any 8852B that successfully loads this
file (as this machine does, FW 0.29.29.18) has `secure_boot = false`. In that case the security
section is just downloaded as an ordinary 2048-byte section (2 packets), no key selection, no
idmem-share write, no blacklist check. A new driver can treat 8852B as non-secure; if it wants to be
defensive, read efuse 0x5EC/0x5ED and refuse (or warn) if they are not 0xFF/0xFF.

---------------------------------------------------------------------------------------------

## 3. FWDL procedure (exact sequence for 8852B PCIe)

### 3.0 When FWDL happens
- **Probe**: `rtw89_chip_info_setup` → power on → `rtw89_mac_partial_init(include_bb=false)`
  (FWDL) → efuse / phycap → pre-deinit → elements → power off (core.c:7180-7206, 7279-7333).
- **Every start** (mac80211 start / leave IPS): `rtw89_core_start` → `rtw89_mac_preinit` (power
  on) → `rtw89_mac_init` → `rtw89_mac_partial_init` (FWDL again) → rest of MAC init
  (core.c:6565-6588; mac.c:4341-4395).
- Whole download is retried up to **5 times** (`rtw89_fw_download`, fw.c:2014-2027).

Prerequisite: MAC powered on (power-on track). `rtw89_h2c_tx` silently drops packets when the
POWERON flag is not set (core.c:1343-1349).

### 3.1 Pre-FWDL MAC/HCI setup (`rtw89_mac_partial_init`, mac.c:4307-4339)

| # | Action | Register writes | Source |
|---|---|---|---|
| 1 | HCI TX/RX DMA enable | `R_AX_HCI_FUNC_EN 0x8380 |= BIT0 (TXDMA) | BIT1 (RXDMA)` | mac.h:1613-1623 |
| 2 | (bb_preinit skipped: include_bb=false; 8852B `bb_preinit`=NULL anyway) | | mac.c:4313-4320; rtw8852b.c:875 |
| 3 | DMAC func enable | `R_AX_DMAC_FUNC_EN 0x8400 = 0x60440000` (MAC_FUNC_EN b30, DMAC_FUNC_EN b29, PKT_BUF_EN b22, DISPATCHER_EN b18) | mac.c:4208-4220 |
| 4 | DMAC clock pre-enable | `R_AX_DMAC_CLK_EN 0x8404 = 0x00040000` (DISPATCHER_CLK_EN b18) | mac.c:4222-4232 |
| 5 | DLE init, mode DLFW (ext mode SCC) | see table 3.1a | mac.c:2274-2344 |
| 6 | HFC init (reset, en=0, h2c_en=1) | see table 3.1b | mac.c:1194-1216 |
| 7 | PCIe `mac_pre_init` | PHY/LTR/AUTOK etc. (PCIe track), then: `0x1010 |= BIT19 (STOP_WPDMA)`; stop all DMA (`0x1010 |= BIT20 STOP_PCIEIO`, `0x1000 &= ~(BIT13 RXHCI_EN|BIT11 TXHCI_EN)`); poll idle; clear ring indexes; program rings (CH12: see §5.1); reset BD RAM; `0x1010 |= 0x00070F00` (stop ACH0-3, CH8, CH9, CH12) then `0x1010 &= ~BIT18` (**un-stop CH12 only**); start DMA (`0x1010 &= ~BIT20`, `0x1000 |= BIT13|BIT11`) | pci.c:3070-3144, 248-275, 2091-2118 |
| 8 | `fwdl_preconfig` | NULL for AX (nothing) | mac.c:7493 |
| 9 | FW download | §3.2-3.9 | fw.c:1965-2012 |

Note STOP_WPDMA (0x1010 bit19) stays set during FWDL: CH12 has no WD pages. It is released only
in `mac_post_init` after MAC init (pci.c:3238-3265).

**3.1a DLE, DLFW quota (8852B PCIe: wde_size9, ple_size8, wde_qt4, ple_qt13; rtw8852b.c:103-106,
mac.c:1742, 1773, 1797, 1839).** Precondition check: `0x8400` has b30|b29 (mac.c:62-84).

| Step | Register | Value |
|---|---|---|
| a | `0x8400 &= ~(BIT26 DLE_WDE_EN | BIT23 DLE_PLE_EN)` | |
| b | `0x8404 |= BIT26 | BIT23` (WDE/PLE clocks) | |
| c | `R_AX_WDE_PKTBUF_CFG 0x8C08`: [1:0]=0 (64 B pages), [13:8]=0 (start bound), [28:16]=0 (free/link pages) | RMW |
| d | `R_AX_PLE_PKTBUF_CFG 0x9008`: [1:0]=1 (128 B pages), [13:8]=8 (bound = (0+1024)·64/8192), [28:16]=64 | RMW |
| e | WDE quota regs, value = min[11:0] | max[27:16]: `0x8C40`=0 (hif), `0x8C44`=0x00000030 (wcpu: min 48 taken from SCC wde_qt7, max 0), `0x8C4C`=0 (pkt_in), `0x8C50`=0 (cpu_io) | write32 |
| f | PLE quota: `0x9040`=0, `0x9044`=0, `0x9048`=0x00100010 (c2h 16/16), `0x904C`=0x00300030 (h2c 48/48), `0x9050`…`0x9068`=0 | write32 |
| g | `0x8400 |= BIT26 | BIT23` | |
| h | poll `R_AX_WDE_INI_STATUS 0x8D00 & 0x3 == 0x3`, 1 µs step, 2 ms timeout; same for `R_AX_PLE_INI_STATUS 0x9100` | |

Sizes: WDE 64·1024 + PLE 128·1024 = 196 608 = fifo_size (checked, mac.c:2305-2309).

**3.1b HFC for FWDL (only the H2C channel is flow-controlled).** (rtw8852b.c:44-49, mac.c:1717,
1127-1192; regs reg.h:1084-1095). Quirk: `dle_info.qta_mode` ends up SCC (second
`get_dle_mem_cfg` call, mac.c:1906-1932, 2290-2298) but the H2C values are identical.

| Step | Register | Value |
|---|---|---|
| a | `R_AX_HCI_FC_CTRL 0x8A00 &= ~(BIT0 HCI_FC_EN | BIT3 HCI_FC_CH12_EN)` | |
| b | `R_AX_CH_PAGE_CTRL 0x8A04 = 0x00280000` (CH12 pre-cost [24:16] = 40 pages) | write32 |
| c | `0x8A00[11:10] = 0` (CH12 full condition) | RMW |
| d | `0x8A00`: BIT0=0, **BIT3=1** (H2C flow control on) | RMW |

### 3.2 Step 1 — stop the WCPU (`rtw89_mac_disable_cpu_ax`, mac.c:4146-4159)

1. (driver) clear "FW ready" state.
2. `R_AX_PLATFORM_ENABLE 0x0088 &= ~BIT1` (WCPU_EN).
3. `R_AX_WCPU_FW_CTRL 0x01E0 &= ~(BIT0 WCPU_FWDL_EN | BIT1 H2C_PATH_RDY | BIT2 FWDL_PATH_RDY)`.
4. `R_AX_SYS_CLK_CTRL 0x0008 &= ~BIT14` (CPU_CLK_EN).
5. FW watchdog disable for 885xB (`rtw89_disable_fw_watchdog`, mac.c:4127-4144):
   `0x0088 &= ~BIT2` (APB_WRAP_EN) then `0x0088 |= BIT2`.
6. Platform reset: `0x0088 &= ~BIT0` (PLATFORM_EN) then `0x0088 |= BIT0`.

### 3.3 Step 2 — boot WCPU into download mode (`rtw89_mac_enable_cpu_ax(boot_reason=0, dlfw=true)`, mac.c:4161-4206; call fw.c:1976)

1. If `0x0088 & BIT1` is still set → abort (-EFAULT).
2. write32: `R_AX_UDM1 0x01F4 = 0`, `R_AX_UDM2 0x01F8 = 0`, `R_AX_HALT_H2C_CTRL 0x0160 = 0`,
   `R_AX_HALT_C2H_CTRL 0x0164 = 0`, `R_AX_HALT_H2C 0x0168 = 0`, `R_AX_HALT_C2H 0x016C = 0`.
3. `0x0008 |= BIT14` (CPU_CLK_EN).
4. RMW `0x01E0`: clear bits 0,1,2; `[7:5] = 0` (FWDL_STS = INITIAL); **set BIT0 (WCPU_FWDL_EN)**.
5. 885xB only: `R_AX_SEC_CTRL 0x0C00[17:16] = 2` (SEC_IDMEM_SIZE_CONFIG).
6. write16-mask `R_AX_BOOT_REASON 0x01E6[2:0] = 0` (boot reason 0 = normal; rtw89 never uses another value).
7. `0x0088 |= BIT1` (WCPU_EN) — ROM bootloader starts.
(The `dlfw=false` branch, which would poll for FW_INIT_RDY directly, is unused.)

### 3.4 Step 3 — parse header; secure-mode idmem (fw.c:1928-1945)
Parse §2. `rtw89_fwdl_secure_idmem_share_mode`: only if secure boot (not our case):
`0x01E0[27:24] = w7.IDMEM_SHARE_MODE; 0x01E0 |= BIT23`.

### 3.5 Step 4 — wait "H2C path ready" (`rtw89_fwdl_check_path_ready_ax(h2c_or_fwdl=true)`, mac.c:7399-7415)
Poll **read8** `0x01E0` until `BIT1` (H2C_PATH_RDY) set; 1 µs between reads, budget
`FWDL_WAIT_CNT` = 400 000 µs (fw.h:5281) (≈0.4 s of delays; wall time longer by MMIO latency).
Timeout → "[ERR]H2C path ready", attempt fails.

### 3.6 Step 5 — send the FW header (`__rtw89_fw_download_hdr`, fw.c:1701-1755)
- Payload = first `hdr_len − dynamic_hdr_len` = **80 bytes** of the image (32 B header + 3×16 B
  section table), with `w7[15:0]` forced to 2020 (fw.c:1648-1670; already 2020 in file). The 64-byte
  dynamic header is not sent.
- Prefixed by an 8-byte H2C header built by `rtw89_h2c_pkt_set_hdr_fwdl` (fw.c:1629-1646;
  fields fw.h:4601-4611):

| DW | Bits | Field | Value |
|---|---|---|---|
| hdr0 | 1:0 | CAT | 1 (`H2C_CAT_MAC`) |
| hdr0 | 7:2 | CLASS | 3 (`H2C_CL_MAC_FWDL`) |
| hdr0 | 15:8 | FUNC | 0 (`H2C_FUNC_MAC_FWHDR_DL`) |
| hdr0 | 19:16 | DEL_TYPE | 0 (`FWCMD_TYPE_H2C`) |
| hdr0 | 31:24 | H2C_SEQ | current `h2c_seq` (not incremented; 0 on first boot) |
| hdr1 | 13:0 | TOTAL_LEN | payload + 8 = **88** |
| hdr1 | 14 / 15 | REC_ACK / DONE_ACK | 0 / 0 |

  → hdr0 = `0x0000000D | seq<<24`, hdr1 = `0x00000058`.
- Transmitted through `rtw89_h2c_tx(fwdl=false)` → CH12 with WD `FW_DL=0` (§5).

### 3.7 Step 6 — wait "FWDL path ready", clear halt regs (fw.c:1757-1780)
1. Poll read8 `0x01E0` until `BIT2` (FWDL_PATH_RDY); 1 µs step, 400 000 µs budget.
   Timeout → "[ERR]FWDL path ready".
2. `R_AX_HALT_H2C_CTRL 0x0160 = 0`; `R_AX_HALT_C2H_CTRL 0x0164 = 0`.

### 3.8 Step 7 — stream the sections (`rtw89_fw_download_main`, fw.c:1782-1888)
For each section in table order (none are `ignore` here):
- Split the section payload (length incl. optional 8-byte checksum) into chunks of
  `min(remaining, 2020)` bytes.
- Each chunk is sent raw (**no H2C header**) via `rtw89_h2c_tx(fwdl=true)` → CH12 with WD
  `FW_DL=1`, one DMA buffer per chunk, doorbell after each (§5).
- No per-chunk handshake; the only throttle is CH12 TX-BD availability (rtw89 reclaims completed
  BDs by reading CH12 hw index before each packet; ring = 256 BDs, so all 165 packets fit anyway).
- AX chips do **not** poll status after each section (fw.c:1875-1876).

### 3.9 Step 8 — wait for firmware ready (fw.c:1985-2006, 106-137)
1. Driver resets its H2C bookkeeping: `h2c_seq = 0`, `rec_seq = 0`, H2C/C2H counters 0,
   RPWM/CPWM sequence numbers = 3 (next used value wraps to 0).
2. `mdelay(5)`.
3. Poll read8 `0x01E0`, field `[7:5]` (FWDL_STS), every 1 µs, budget 400 000 µs, until
   **== 7 (WCPU_FW_INIT_RDY)**. Success ⇒ firmware (FreeRTOS) is running and H2C/C2H usable.

`R_AX_WCPU_FW_CTRL (0x01E0)` bit map (reg.h:214-220):

| Bits | Name | Written by | Meaning |
|---|---|---|---|
| 0 | WCPU_FWDL_EN | host | request download-mode boot |
| 1 | H2C_PATH_RDY | chip | ROM ready to accept H2C (FW header) |
| 2 | FWDL_PATH_RDY | chip | header accepted, ready for section data |
| 7:5 | WCPU_FWDL_STS | chip | 0 INITIAL, 1 FWDL_ONGOING, 2 CHECKSUM_FAIL, 3 SECURITY_FAIL, 4 CV_NOT_MATCH, 5 RSVD0, 6 WCPU_FWDL_RDY (image loaded), 7 WCPU_FW_INIT_RDY (fw.h:10-19) |
| 23 | IDMEM_SHARE_MODE_RECORD_VALID | host (secure only) | |
| 27:24 | IDMEM_SHARE_MODE_RECORD | host (secure only) | |

### 3.10 Errors, diagnostics, retries
- Timeout of step 8 decodes the last FWDL_STS: 2 → "fw checksum fail" (-EINVAL), 3 → "fw security
  fail" (-EINVAL), 4 → "fw cv not match" (-EINVAL; wrong cv image), other → "unexpected status"
  (-EBUSY) (fw.c:115-131).
- On any failure after step 2 (`rtw89_fw_dl_fail_dump`, fw.c:1890-1926): log `0x01E0` and
  `R_AX_BOOT_DBG 0x83F0`; then PC dump: `R_AX_DBG_CTRL 0x0058 = 0x00F200F2` (DBG_SEL0/1 = 0xF2),
  `R_AX_SYS_STATUS1 0x00F4[17:16] = 1`, read `R_AX_DBG_PORT_SEL 0x00C0` 15× with 10 µs spacing
  (WCPU program counter).
- Entire sequence (§3.2-3.9) retried up to 5× (fw.c:2014-2027). The pre-setup (§3.1) is not redone
  per retry.
- There is **no** FW-ready C2H and no FW-version query: readiness = FWDL_STS 7; version comes from
  the file header.

### 3.11 Minimal-driver notes
- Mandatory: §3.1 steps 1,3,4,5,6 and the CH12 part of 7 (ring program + un-stop CH12 + DMA
  start), §3.2-3.9 exactly. Interrupts are not needed (all polling).
- Optional: fail dumps, retries (recommended to keep ≥1 retry), secure-boot paths, LOGFMT.
- The FW header H2C must be the first packet on CH12 after WCPU enable; data chunks must follow
  in section order without any H2C header.

---------------------------------------------------------------------------------------------

## 4. Immediately after FWDL (formats → track 04 H2C/C2H)

Probe path (core.c:7180-7206): efuse map parse (efuse track) → phycap map → **H2C-register
exchange** `GET_FEATURE` (H2CREG func 3, part 0, content_len 0 on AX) answered by C2H-register
`PHY_CAP` (func 3) with rx/tx NSS, antenna counts (mac.c:3177-3283, `rtw89_fw_msg_reg`) — first
FW interaction after download; then `hci mac_pre_deinit`, elements, power off.

Start path (`rtw89_mac_init` rest, mac.c:4359-4395; `rtw89_core_start`, core.c:6565-6631):
1. `rtw89_chip_enable_bb_rf`, `sys_init`, `trx_init` (DMAC/CMAC init incl. DLE re-init in SCC
   mode and full HFC) — MAC-init track.
2. `rtw89_mac_feat_init` — no-op for 8852B (BACAM V0).
3. PCIe `mac_post_init`: LTR, `R_AX_TX_ADDRESS_INFO_MODE_SETTING` 8-byte addr-info, clear
   `R_AX_PKTIN_SETTING.WD_ADDR_INFO_LENGTH`, un-stop all TX DMA channels, clear STOP_WPDMA|STOP_PCIEIO
   (pci.c:3238-3265) — PCIe track.
4. Early H2C list (normally empty), **H2C OFLD_CFG** (cat 1, class 9 `H2C_CL_MAC_FW_OFLD`, func 0x14,
   DONE_ACK, fixed payload `09 00 00 00 5e 00 00 00`) (fw.c:5274-5303).
5. BB table, RF tables (+ RF_REG H2Cs, class 8/9, cat 2), BT-coex init H2Cs, DM init, RFK, then
   **H2C FW log config** (cat 1, class 0 `H2C_CL_FW_INFO`, func 0 `LOG_CFG`; level LOUD, path C2H,
   component mask 0 unless FW log enabled via debugfs) (fw.c:2799-2839).

---------------------------------------------------------------------------------------------

## 5. PCIe framing of FWDL packets (FWCMD channel, CH12)

### 5.1 CH12 ring (programmed in PCIe pre-init; pci.c:1776-1830, pci.h:553-655)

| Register | Addr | Value |
|---|---|---|
| R_AX_CH12_TXBD_NUM | 0x1038 (16-bit) | ring length = 256 (`RTW89_PCI_TXBD_NUM_MAX`, pci.h:1122) |
| R_AX_CH12_BDRAM_CTRL | 0x1228 | `start_idx 28 [7:0] | max 4 [15:8] | min 1 [23:16]` = 0x0001041C (single-band table, pci.c:1711-1721) |
| R_AX_CH12_TXBD_DESA_L / _H | 0x1160 / 0x1164 | ring DMA address low / high 32 bits |
| R_AX_CH12_TXBD_IDX | 0x1080 | host write index `[11:0]` (driver writes 16-bit), HW read index `[27:16]` (pci.h:553-559) |
| R_AX_PCIE_DMA_STOP1 | 0x1010 | BIT18 STOP_CH12 must be 0 |

No WD pages are allocated for CH12 (pci.c:3552-3554); the WD body is prepended inline in the
same buffer as the payload. DMA mask is 32-bit unless the upstream bridge is DAC-capable (36-bit)
(pci.c:3370-3388).

### 5.2 TX buffer descriptor (`struct rtw89_pci_tx_bd_32`, pci.h:1465-1471), 8 bytes

| Offset | Size | Field | Value for FWCMD |
|---|---|---|---|
| 0 | 2 | length | total buffer bytes = 24 (WD body) + payload |
| 2 | 2 | opt | BIT14 LS (last segment) = 1; [13:6] = DMA address bits 39:32 |
| 4 | 4 | dma | DMA address bits 31:0 |

One BD per packet (`rtw89_pci_fwcmd_submit`, pci.c:1551-1587). Doorbell: write16 host index to
0x1080 after each packet (`rtw89_h2c_tx` kicks every time; core.c:1361-1372, pci.c:1332-1345).
Completion/reclaim: read 0x1080, HW index `[27:16]` = number of consumed BDs (pci.c:61-98,
133-143). Max outstanding = 255 (one slot kept empty, pci.c:1222-1231).

### 5.3 WD body (`struct rtw89_txwd_body`, 24 bytes; filled by `rtw89_core_fill_txdesc`, core.c:1600-1620)

Descriptor info for H2C (core.c:905-913, 1241-1246, 1351-1356): `pkt_size = payload length`
(H2C header included when present), `ch_dma = 12`, `wd_page = 0`, `fw_dl` = 1 for section chunks /
0 for the FW-header H2C; everything else 0 (no WD info, qsel 0, macid 0).

| DW | Bits (txrx.h:69-97) | Field | FW header pkt | Section chunk |
|---|---|---|---|---|
| 0 | 31:24 | WP_OFFSET | 0 | 0 |
| 0 | 22 | WD_INFO_EN | 0 | 0 |
| 0 | 20 | FW_DL | 0 | **1** |
| 0 | 19:16 | CHANNEL_DMA | **12** | **12** |
| 0 | 15:11 | HDR_LLC_LEN | 0 | 0 |
| 0 | 7 | WD_PAGE | 0 | 0 |
| 0 | 3:2 / 1:0 | HW_SSN_SEL / MODE | 0 | 0 |
| 1 | – | (not written, zeroed) | 0 | 0 |
| 2 | 13:0 | TXPKT_SIZE | 88 | chunk len (≤ 2020) |
| 2 | 22:17 | QSEL | 0 | 0 |
| 2 | 30:24 / 23 | MACID / TID_IND | 0 | 0 |
| 3 | 11:0 / 12 / 13 | SW_SEQ / AGG_EN / BK | 0 | 0 |
| 4-5 | – | zero | 0 | 0 |

So dword0 = **0x000C0000** (header / normal H2C) or **0x001C0000** (FWDL data); dword2 = length.

### 5.4 Byte images

FW-header packet (DMA buffer, 112 bytes, BD length 112):
```
WD   : 00 00 0c 00  00 00 00 00  58 00 00 00  00 00 00 00  00 00 00 00  00 00 00 00
H2C  : 0d 00 00 00  58 00 00 00                      (seq 0)
FWHDR: 02 01 52 88  00 1d 1d 12  7f 77 3d 9e  20 10 90 00  04 09 14 0d  ea 07 00 00
       00 03 50 00  e4 07 01 00
       00 00 97 b8  40 bf 04 42  00 00 00 00  00 00 00 00      (section 0)
       00 24 e1 b8  78 34 00 41  00 00 00 00  00 00 00 00      (section 1)
       28 19 e1 b8  00 08 00 09  00 00 00 00  00 00 00 00      (section 2)
```
Section chunk: `WD = 00 00 1c 00 | 00 00 00 00 | e4 07 00 00 | 0 | 0 | 0` followed by 2020 bytes
of section data (BD length 2044); last chunk of section 0 carries 24 bytes (TXPKT_SIZE 0x18).

### 5.5 Size / alignment limits
- Chunk payload ≤ 2020 B (AX fixed; the file's PART_SIZE agrees). TXPKT_SIZE field is 14 bits.
  Per-packet DMA length ≤ 2044 B, which stays under the 2048 B TX burst configured for 8852B
  (`MAC_AX_TX_BURST_2048B`, rtw8852be.c:20) — plausible reason for 2020, not stated in source.
- Chunk lengths in this file are all multiples of 4 (2020, 24, 1312, 28); rtw89 does no padding.
- rtw89 imposes no explicit buffer alignment; in practice the WD starts at the skb's default
  data offset (NET_SKB_PAD, cache-line aligned). A new driver should use ≥8-byte (preferably
  64-byte) aligned coherent/streaming buffers.
- DMA address width ≤ 40 bits via opt[13:6] (actually limited to 32/36-bit by the DMA mask).
- H2C total (non-FWDL) must be ≤ `RTW89_H2C_MAX_SIZE` 2048 (fw.h:329).

---------------------------------------------------------------------------------------------

## 6. parse_fw.py output summary

```
MFW: sig=0xFF fw_nr=5 ver=0.29.29.18 ; selection cv=1 → NORMAL_CE entry1, WOWLAN entry2, LOGFMT entry0
entry1 NORMAL_CE cv1: hdr_ver=0 ver 0.29.29.18 commit 9e3d777f build 2026-04-09 20:13
  sec_num=3 dyn_hdr=1 base=80 hdr_len=144 dyn=64(ok) part_size=2020 idmem=0
  sec0 type2 311104B dl 0xb8970000 ; sec1 type1 13432B dl 0xb8e12400 ; sec2 type9 2048B mssc 0
  → 1 header H2C (80+8 B) + 164 data packets ; sections end exactly at entry end
elements @0x130480..EOF 0x17B8A2:
  2 BB_REG 1013 regs | 3 BB_GAIN 66 | 4 RADIO_A 8295 (12 headlines, sel 0x0010001, 947 applied)
  5 RADIO_B 8392 (932 applied) | 8 NCTL 1320 | 9/10/11/13/14/16/17 TXPWR ×3 (rfe 0,11,12)
  18 TXPWR_TRK bitmap 0xFFF0 600B | 20 REGD 242 countries
  TXPWR chosen for rfe 1: all rfe_type-0 instances
  PHY element tables identical to rtw8852b_table.c: BB, BB_GAIN, RADIO_A, RADIO_B, NCTL = True
```
Full output: `docs/study/fwdump/parse_fw_output.txt`.

---------------------------------------------------------------------------------------------

## 7. Open questions / uncertainties

1. FW header w0, w3[15:0], w6[23:16] and section s1[31:30] are not interpreted by rtw89; the
   meanings suggested above (chip id / cut in w0) are inferred from the file only.
2. `DL_ADDR`, section types 1/2 and REDL are consumed only by the on-chip ROM; rtw89 never
   uses them. A new driver must still send the header verbatim.
3. Whether the ROM really requires CH12's `FW_DL=0` for the header and `FW_DL=1` for data (vs.
   just rtw89 convention) is not verifiable from source; follow rtw89 exactly.
4. Secure-boot 8852B parts: with this firmware file rtw89 would fail to parse (MSSC=0); it is
   unclear whether such parts exist with this file in the field. Efuse 0x5EC/0x5ED of this unit
   was not read (live system untouched); success of the current rtw89 load implies non-secure.
5. The TX-power element values vs. the compiled-in 8852B TX-power tables were not compared
   (only PHY tables were); units/semantics are for the TX-power track.
6. The 400 000-iteration polls are CPU-time based (1 µs udelay each); real wall-clock budget is
   larger. A new driver can use a jiffies/ktime timeout of ~0.5–1 s.
