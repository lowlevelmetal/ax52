# 02 — Power on/off, MAC hardware init, efuse, minimal BT-coex (RTL8852BE)

Scope: RTL8852B, PCIe, CV_B (hal.cv = CHIP_CBV = 1), RFE type 1, station mode only.
All paths below are resolved for that combination. Source root for citations:
`reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/` (file:line).

Numeric values were computed by compiling `reg.h` macros (not hand-copied), so compound
masks such as `B_AX_PLE_IMR_SET` are exact.

---------------------------------------------------------------------------------------------
## 0. Conventions

Register access primitives used by rtw89 (and assumed in the step lists below):

| Notation in this doc | Meaning |
|---|---|
| `W32(a)=v` / `W16` / `W8` | plain MMIO write of that width at BAR offset `a` |
| `SET32(a, m)` | read32, OR `m`, write32 (`rtw89_write32_set`) |
| `CLR32(a, m)` | read32, AND ~`m`, write32 |
| `SET8/CLR8/SET16/CLR16` | same but 8/16-bit access at byte address `a` |
| `MASK32(a, m, v)` | read32, replace field `m` with `v` (shifted to lowest set bit of `m`), write32. If `m`=0xFFFFFFFF rtw89 writes directly |
| `MASK16/MASK8` | same with 16/8-bit access; mask is relative to that access |
| `POLL32(a, cond, interval, timeout)` | `read_poll_timeout`: read, test, sleep `interval` µs, give up after `timeout` µs |
| `XSI_W(off, data, mask)` | XTAL-SI indirect write, see §2 |
| `XSI_R(off)` | XTAL-SI indirect read, see §2 |
| `RF_W(path, addr, mask, v)` | RF register write through the BB/RF serial interface (owned by the PHY/RF track; 8852B uses `rtw89_phy_write_rf_v1`, phy.c:1190) |

Band-1 (CMAC1) registers = band-0 address + 0x2000 (`RTW89_MAC_AX_BAND_REG_OFFSET`, mac.h:590;
`rtw89_mac_reg_by_idx`, mac.h:1183). Port registers = port-0 address + port*0x40
(`rtw89_mac_reg_by_port`, mac.h:1207). **8852B runs single band (dbcc_en=false, qta_mode=SCC,
core.c:6926-6929), so everything is band 0 / CMAC0.**

### 0.1 Indirection resolution for 8852B (rtw8852b.c:869-955, 957-1122; mac.c:7430-7525)

| Generic hook | 8852B concrete |
|---|---|
| `chip->ops->pwr_on_func / pwr_off_func` | `rtw8852b_pwr_on_func` (rtw8852b.c:385) / `rtw8852b_pwr_off_func` (rtw8852b.c:514). `pwr_on_seq/pwr_off_seq` = NULL (table interpreter `rtw89_mac_pwr_seq` NOT used) |
| `chip->ops->enable_bb_rf / disable_bb_rf` | `__rtw8852bx_mac_enable_bb_rf` (rtw8852b_common.c:2001) / `__rtw8852bx_mac_disable_bb_rf` (rtw8852b_common.c:2036). The generic `rtw89_mac_enable_bb_rf` (mac.c:4281) is NOT used |
| `chip->ops->read_efuse / read_phycap` | `__rtw8852bx_read_efuse` (rtw8852b_common.c:241) / `__rtw8852bx_read_phycap` (:434) |
| `chip->ops->power_trim` | `__rtw8852bx_power_trim` (thermal + PA-bias trim, :445) |
| `chip->ops->cfg_ctrl_path / mac_cfg_gnt` | `rtw89_mac_cfg_ctrl_path` (mac.c:6631) / `rtw89_mac_cfg_gnt` (mac.c:6489) — the non-`_v1` variants |
| `chip->ops->btc_init_cfg / btc_set_rfe / btc_set_wl_pri` | `__rtw8852bx_btc_init_cfg` (rtw8852b_common.c:1797) / `rtw8852b_btc_set_rfe` (rtw8852b.c:751) / `__rtw8852bx_btc_set_wl_pri` (rtw8852b_common.c:1845) |
| `chip->ops->bb_preinit / bb_postinit / fem_setup / data_setup` | NULL (no-ops) |
| `mac_def` | `rtw89_mac_gen_ax`: `sys_init=sys_init_ax`, `trx_init=trx_init_ax`, `disable_cpu=rtw89_mac_disable_cpu_ax`, `fwdl_enable_wcpu=rtw89_mac_enable_cpu_ax`, `hci_func_en=rtw89_mac_hci_func_en_ax`, `dmac_func_pre_en=rtw89_mac_dmac_func_pre_en_ax`, `dle_*_ax`, `hfc_*_ax`, `parse_efuse_map=rtw89_parse_efuse_map_ax`, `parse_phycap_map=rtw89_parse_phycap_map_ax`, `efuse_read_fw_secure=rtw89_efuse_read_fw_secure_ax`, `write/read_xtal_si=rtw89_mac_*_xtal_si_ax`. NULL: `mac_func_en`, `clr_aon_intr`, `fwdl_preconfig`, `efuse_read_ecv`, `cfg_phy_rpt`, `set_edcca_mode` |
| `qta_def.dle_mem / hfc_param_ini` (hci.dle_type = PCIE = index 0, pci.c:4786) | `rtw8852b_dle_mem_pcie` (rtw8852b.c:94) / `rtw8852b_hfc_param_ini_pcie` (rtw8852b.c:44) |
| BTC version table (FW 0.29.29.18 ≥ 0.29.29.0) | entry coex.c:216: `fwlrole=2, fcxctrl=1, fcxinit=0, fcxtdma=3, fcxslots=1, drvinfo_type=0` → old (non-v7) structs |
| `rtw89_is_rtl885xb()` (core.h:8045) | TRUE for 8852B — gates several 885xB-specific branches below |

---------------------------------------------------------------------------------------------
## 1. Global call order

### 1.1 Probe (`rtw89_pci_probe`, pci.c:4765 → `rtw89_chip_info_setup`, core.c:7279)
1. `rtw89_core_init` sets `hal.rx_fltr = DEFAULT_AX_RX_FLTR (0x030044BE)`, `dbcc_en=false`,
   `mac.qta_mode = RTW89_QTA_SCC` (core.c:6926-6929).
2. PCI claim / BAR map (PCIe track).
3. `rtw89_read_chip_ver` (core.c:7030): `hal.cv = R32(0x00F0)[15:12]` (R_AX_SYS_CFG1,
   B_AX_CHIP_VER_MASK) → 1 = CBV. `hal.acv = XSI_R(0x41)[3:0]` (XTAL_SI_CV). Done **before** power on.
4. `rtw89_mac_pwr_on` (§3) — includes, on first power-on only, the FW-secure efuse read (§11.6).
5. wait for firmware file, `rtw89_fw_recognize` (FW track).
6. `rtw89_chip_efuse_info_setup` (core.c:7180):
   1. `rtw89_mac_partial_init(include_bb=false)` (§6) — i.e. DLE/HFC in DLFW mode + PCIe pre-init
      + **firmware download**. FW must be running for step 6.4.
   2. `rtw89_parse_efuse_map_ax` (§11.1–11.4).
   3. `rtw89_parse_phycap_map_ax` (§11.5).
   4. `rtw89_mac_setup_phycap` — H2C-register "GET_FEATURE part 0" to FW (§11.7).
   5. `rtw89_core_setup_phycap` (SW only).
   6. `rtw89_hci_mac_pre_deinit` (PCIe power-wake off; PCIe track).
7. FW elements, rfe params (SW), log "chip info CID..RFE".
8. `rtw89_mac_pwr_off` (§4) — **chip is powered off again at end of probe** (core.c:7329-7330).
9. rfkill GPIO init (`rtw89_core_rfkill_init`, core.c:7220): `MASK16(0x02D4, 0x00F0, 0xF)`
   (GPIO9 pinmux = 0xF), `MASK16(0x0062, 0x0202, 0)` (GPIO9 input). Poll: blocked if
   `R8(0x0060) & BIT1 == 0` (core.c:7230). Optional.

### 1.2 Interface up (`rtw89_core_start`, core.c:6565)
1. `rtw89_mac_preinit` (mac.c:4341) = `rtw89_mac_pwr_on` (§3). `mac_func_en` NULL.
2. `rtw89_chip_bb_preinit` → NULL; `rtw89_phy_init_bb_afe` → no-op (8852B FW file has no AFE element).
3. `rtw89_mac_init` (mac.c:4359):
   1. `rtw89_mac_partial_init(false)` (§6): HCI DMA enable, DMAC pre-init (DLE **DLFW** + HFC h2c-only),
      PCIe `mac_pre_init`, FW download (FW track; MAC side in §5).
   2. `rtw89_chip_enable_bb_rf` (§7).
   3. `sys_init_ax` (§8).
   4. `trx_init_ax` (§9): DMAC init (DLE **SCC**, HFC full, STA-sched, MPDU-proc, sec-eng), CMAC0 init,
      IMRs, error IMR, host RPR.
   5. `rtw89_mac_feat_init` → no-op (bacam_ver V0, mac.c:4082).
   6. PCIe `mac_post_init` (LTR, 8-byte addr-info, enable all TX DMA channels, release PCI IO; PCIe track).
   7. FW early H2Cs + `h2c_set_ofld_cfg` (FW track).
   On any failure `rtw89_mac_pwr_off`.
4. `rtw89_btc_ntfy_poweron` — counter only (coex.c:7692).
5. `rtw89_chip_reset_bb_rf` = disable_bb_rf then enable_bb_rf (mac.h:1371; §7).
6. BB table load, RF table load (PHY track).
7. `rtw89_btc_ntfy_init(BTC_MODE_NORMAL)` (§13).
8. `rtw89_phy_dm_init` (PHY track; includes XTAL cap programming, §11.4 note).
9. `rtw89_mac_cfg_ppdu_status_bands(true)` (§10), `rtw89_mac_update_rts_threshold` (§10).
10. `rtw89_hci_start` (PCIe interrupts on).
11. queue `track_work` (2 s) and `track_ps_work` (100 ms) (§14).
12. `rtw89_btc_ntfy_radio_state(WL_ON)` (§13), FW log H2C, BA-CAM init H2C.

### 1.3 Interface down (`rtw89_core_stop`, core.c:6634)
coex radio_state(WL_OFF) → cancel works → `rtw89_btc_ntfy_poweroff` (antenna to BT, §13) →
flush → `hci_stop/deinit` → `rtw89_mac_pwr_off` (§4) → `hci_reset`.

---------------------------------------------------------------------------------------------
## 2. XTAL-SI indirect interface (analog "A-die"/XTAL register file)

Single command/status register: **R_AX_WLAN_XTAL_SI_CTRL = 0x0270** (reg.h:268).

| Bits | Field | Meaning |
|---|---|---|
| 31 | B_AX_WL_XTAL_SI_CMD_POLL | write 1 to start; HW clears when done |
| 30 | B_AX_BT_XTAL_SI_ERR_FLAG | (status, not used) |
| 29 / 28 | WL / BT XTAL_GNT | (arbitration status, not used) |
| 25:24 | MODE | 0 = write, 1 = read |
| 23:16 | BITMASK | bits of the target byte to modify (write) — HW does the RMW |
| 15:8 | DATA | write data / read result |
| 7:0 | ADDR | XTAL-SI byte offset |

**Write** (`rtw89_mac_write_xtal_si_ax`, mac.c:7175): `W32(0x0270) = BIT31 | (0<<24) | mask<<16 | data<<8 | off`;
then `POLL32(0x0270, BIT31==0, 50 µs, 50 000 µs)`. rtw89 warns (does not fail) if the read-back
ADDR ≠ off or DATA ≠ data. So `XSI_W(off, 0, BITn)` clears bit n and `XSI_W(off, BITn, BITn)` sets it.

**Read** (mac.c:7205): `W32(0x0270) = BIT31 | (1<<24) | off`; same poll; result = `R32(0x0270)[15:8]`.

XTAL-SI offsets used on 8852B (mac.h:1646-1698):

| Off | Name | Bits used |
|---|---|---|
| 0x04 / 0x05 | XTAL_SC_XI / XTAL_SC_XO | crystal cap (full byte) |
| 0x24 | XTAL_XMD_2 | [6:4] LDO_LPS |
| 0x26 | XTAL_XMD_4 | [3:0] LPS_CAP |
| 0x41 | CV | [3:0] analog cut version |
| 0x62 / 0x63 / 0x7A | DAV efuse LOW_ADDR / CTRL / READ_VAL | see §11.3 |
| 0x80 | WL_RFC_S0 | [2:0] RF00S_EN (BIT0 = RF00) |
| 0x81 | WL_RFC_S1 | [2:0] RF10S_EN (BIT0 = RF10) |
| 0x90 | ANAPAR_WL | 7 SRAM2RFC, 6 GND_SHDN_WL, 5 SHDN_WL, 4 RFC2RF, 3 OFF_EI, 2 OFF_WEI, 1 PON_EI, 0 PON_WEI |
| 0xA1 | SRAM_CTRL | 1 SRAM_DIS |

---------------------------------------------------------------------------------------------
## 3. Power ON

### 3.1 Wrapper `rtw89_mac_pwr_on` / `rtw89_mac_power_switch(on)` (mac.c:1586, 1521)
1. (USB boot-mode fix — skipped for PCIe.)
2. If FW was running (`FW_RDY`): leave PS (PS track). Not the case at cold start.
3. `rtw89_mac_reset_pwr_state_ax` (mac.c:4104): read **R_AX_IC_PWR_STATE 0x03F0 [9:8]**
   (WLMAC_PWR_STE: 0=OFF,1=ON,2=LPS). If ==1 → error -EBUSY ("MAC has already powered on").
4. `rtw8852b_pwr_on_func` (§3.2).
5. If probe not done yet: `efuse_read_ecv` (NULL for AX → skipped), `rtw89_efuse_read_fw_secure_ax` (§11.6).
6. Scoreboard notify: `W8(0x00AF) = 0x81` (R_AX_SCOREBOARD+3 = MAC_AX_NOTIFY_TP_MAJOR; bit31 = toggle)
   (mac.c:1506).
7. On any failure in 3–4, `rtw89_mac_pwr_on` does power_switch(off) then retries power_switch(on) once.

### 3.2 `rtw8852b_pwr_on_func` (rtw8852b.c:385-512) — ordered register sequence

| # | Access | Reg (name) | Value / bits | Note |
|---|---|---|---|---|
| 0 | — | (R_AX_SPS_ANA_ON_CTRL2 0x0228 = 0x4A82 only if RFE==5) | skipped for RFE 1 | rtw8852b.c:356 |
| 1 | CLR32 | 0x0004 SYS_PW_CTRL | BIT11 AFSM_WLSUS_EN, BIT12 AFSM_PCIE_SUS_EN | |
| 2 | SET32 | 0x0004 | BIT18 DIS_WLBT_PDNSUSEN_SOPC | |
| 3 | SET32 | 0x0090 WLLPS_CTRL | BIT1 DIS_WLBT_LPSEN_LOPC | |
| 4 | CLR32 | 0x0004 | BIT15 APDM_HPDN | |
| 5 | CLR32 | 0x0004 | BIT10 APFM_SWLPS | |
| 6 | POLL32 | 0x0004 | BIT17 RDY_SYSPWR == 1; 1000 µs / 20 000 µs | fail → -ETIMEDOUT |
| 7 | SET32 | 0x0020 AFE_LDO_CTRL | BIT23 AON_OFF_PC_EN | |
| 8 | POLL32 | 0x0020 | BIT23 == 1; 1000/20 000 µs | |
| 9 | MASK32 | 0x0400 SPS_DIG_OFF_CTRL0 | [1:0] C1_L1 = 1 | `rtw8852b_pwr_sps_dig_off`, RFE≠5 branch |
| 10 | MASK32 | 0x0400 | [5:4] C3_L1 = 3 | (RFE 5 would write C1=1,C2=1,C3=2,R1=1) |
| 11 | SET32 | 0x0004 | BIT16 EN_WLON | |
| 12 | SET32 | 0x0004 | BIT8 APFN_ONMAC | request MAC power-on |
| 13 | POLL32 | 0x0004 | BIT8 == 0 (HW self-clears); 1000/20 000 µs | |
| 14 | SET8, CLR8, SET8, CLR8, SET8 | 0x0088 PLATFORM_ENABLE | BIT0 PLATFORM_EN toggled, ends =1 | |
| 15 | CLR32 | 0x0070 SYS_SDIO_CTRL | BIT12 PCIE_CALIB_EN_V1 | PCIe only |
| 16 | SET32 | 0x0018 SYS_ADIE_PAD_PWR_CTRL | BIT6 SYM_PADPDN_WL_PTA_1P3 | |
| 17 | XSI_W | 0x90 | data 0x40 mask 0x40 (set GND_SHDN_WL) | |
| 18 | SET32 | 0x0018 | BIT5 SYM_PADPDN_WL_RFC_1P3 | |
| 19 | XSI_W | 0x90 | set BIT5 SHDN_WL (0x20/0x20) | |
| 20 | XSI_W | 0x90 | set BIT2 OFF_WEI (0x04/0x04) | |
| 21 | XSI_W | 0x90 | set BIT3 OFF_EI (0x08/0x08) | |
| 22 | XSI_W | 0x90 | clear BIT4 RFC2RF (0x00/0x10) | |
| 23 | XSI_W | 0x90 | set BIT0 PON_WEI (0x01/0x01) | |
| 24 | XSI_W | 0x90 | set BIT1 PON_EI (0x02/0x02) | |
| 25 | XSI_W | 0x90 | clear BIT7 SRAM2RFC (0x00/0x80) | |
| 26 | XSI_W | 0xA1 | clear BIT1 SRAM_DIS (0x00/0x02) | |
| 27 | XSI_W | 0x24 | clear [6:4] LDO_LPS (0x00/0x70) | |
| 28 | XSI_W | 0x26 | clear [3:0] LPS_CAP (0x00/0x0F) | |
| 29 | SET32 | 0x00CC PMC_DBG_CTRL2 | BIT2 SYSON_DIS_PMCR_AX_WRMSK (unlock) | |
| 30 | SET32 | 0x0000 SYS_ISO_CTRL | BIT8 ISO_EB2CORE | efuse isolation on |
| 31 | CLR32 | 0x0000 | BIT15 PWC_EV2EF_B15 | efuse power off |
| 32 | sleep | — | 1000 µs | |
| 33 | CLR32 | 0x0000 | BIT14 PWC_EV2EF_B14 | |
| 34 | CLR32 | 0x00CC | BIT2 (re-lock) | |
| 35 | cond. | — | skip 36–38 if `!efuse.valid` **or** `efuse.power_k_valid` | at probe efuse not parsed yet → skipped |
| 36 | MASK32 | 0x0200 SPS_DIG_ON_CTRL0 | [3:0] VOL_L1 = 0x9 | |
| 37 | MASK32 | 0x0200 | [25:22] VREFPFM_L = 0xA | |
| 38 | CBV && PCIe: SET32 0x00CC BIT2; MASK16 0x007A HCI_LDO_CTRL [3:0] VADJ = 0xA; CLR32 0x00CC BIT2 | | | applies to this chip (CV_B, PCIe) when efuse valid and phycap byte 0x5E9 ≠ 0xAA |
| 39 | SET32 | 0x8400 DMAC_FUNC_EN | 0x7FFF8000 (bits 30..15: MAC_FUNC, DMAC_FUNC, MPDU_PROC, WD_RLS, DLE_WDE, TXPKT_CTRL, STA_SCH, DLE_PLE, PKT_BUF, DMAC_TBL, PKT_IN, DLE_CPUIO, DISPATCHER, BBRPT, MAC_SEC, DMACREG_GCKEN) | |
| 40 | SET32 | 0xC000 CMAC_FUNC_EN | 0x7000803F (CMAC_EN b30, CMAC_TXEN b29, CMAC_RXEN b28, FORCE_CMACREG_GCKEN b15, PHYINTF b5, CMAC_DMA b4, PTCLTOP b3, SCHEDULER b2, TMAC b1, RMAC b0) | |
| 41 | MASK32 | 0x02D8 EECS_EESK_FUNC_SEL | [7:4] = 1 (EESK pin = BT log) | optional |

Mandatory: all of 1–40 (41 is a debug pin-mux). Note that later steps (§6, §8) **overwrite**
0x8400 with full 32-bit writes, so the exact bits of step 39/40 only matter until then.

---------------------------------------------------------------------------------------------
## 4. Power OFF (`rtw8852b_pwr_off_func`, rtw8852b.c:514-590)

Called through `rtw89_mac_power_switch(false)`; afterwards `W8(0x00AF) = 0x80`
(MAC_AX_NOTIFY_PWR_MAJOR) (mac.c:1575). If the device is flagged unplugged, no IO at all.

| # | Access | Reg | Value |
|---|---|---|---|
| 0 | (0x0228 = 0x4A82 only RFE 5) | | skipped |
| 1 | XSI_W 0x90 | set BIT4 RFC2RF (0x10/0x10) | |
| 2 | XSI_W 0x90 | clear BIT3 OFF_EI | |
| 3 | XSI_W 0x90 | clear BIT2 OFF_WEI | |
| 4 | XSI_W 0x80 | clear BIT0 RF00 | |
| 5 | XSI_W 0x81 | clear BIT0 RF10 | |
| 6 | XSI_W 0x90 | set BIT7 SRAM2RFC | |
| 7 | XSI_W 0x90 | clear BIT1 PON_EI | |
| 8 | XSI_W 0x90 | clear BIT0 PON_WEI | |
| 9 | SET32 0x0004 | BIT16 EN_WLON | |
| 10 | CLR32 0x02F0 WLRF_CTRL | BIT17 AFC_AFEDIG | |
| 11 | CLR8 0x0002 SYS_FUNC_EN | BIT1 FEN_BB_GLB_RSTN, BIT0 FEN_BBRSTB | |
| 12 | CLR32 0x0018 | BIT5 RFC_1P3 | |
| 13 | XSI_W 0x90 | clear BIT5 SHDN_WL | |
| 14 | CLR32 0x0018 | BIT6 PTA_1P3 | |
| 15 | XSI_W 0x90 | clear BIT6 GND_SHDN_WL | |
| 16 | SET32 0x0004 | BIT9 APFM_OFFMAC | request MAC off |
| 17 | POLL32 0x0004 | BIT9 == 0; 1000/20 000 µs | |
| 18 | W32 0x0090 | 0x0001A0B2 (SW_LPS_OPTION) | PCIe branch |
| 19 | SET32 0x0010 SYS_SWR_CTRL1 | BIT10 SYM_CTRL_SPS_PWMFREQ | |
| 20 | MASK32 0x0200 | [18:17] REG_ZCDC_H = 3 | |
| 21 | SET32 0x0004 | BIT10 APFM_SWLPS | PCIe branch |

---------------------------------------------------------------------------------------------
## 5. WCPU disable / enable (MAC part of firmware download)

FW download itself (header/section DMA over the FWCMD queue) belongs to the FW track; the MAC
registers it touches:

### 5.1 `rtw89_mac_disable_cpu_ax` (mac.c:4146)
1. `CLR32(0x0088, BIT1)` WCPU_EN.
2. `CLR32(0x01E0 WCPU_FW_CTRL, BIT0|BIT1|BIT2)` (WCPU_FWDL_EN, H2C_PATH_RDY, FWDL_PATH_RDY).
3. `CLR32(0x0008 SYS_CLK_CTRL, BIT14)` CPU_CLK_EN.
4. FW watchdog disable, 885xB variant (mac.c:4127): `CLR32(0x0088, BIT2)` APB_WRAP_EN then `SET32(0x0088, BIT2)`.
5. `CLR32(0x0088, BIT0)` then `SET32(0x0088, BIT0)` (PLATFORM_EN reset pulse).

### 5.2 `rtw89_mac_enable_cpu_ax(boot_reason=0, dlfw=true)` (mac.c:4161)
1. If `R32(0x0088) & BIT1` → error (CPU already on).
2. `W32(0x01F4 UDM1)=0; W32(0x01F8 UDM2)=0; W32(0x0160 HALT_H2C_CTRL)=0; W32(0x0164 HALT_C2H_CTRL)=0;
   W32(0x0168 HALT_H2C)=0; W32(0x016C HALT_C2H)=0`.
3. `SET32(0x0008, BIT14)` CPU_CLK_EN.
4. RMW 0x01E0: clear bits 0,1,2; field [7:5] FWDL_STS = 0 (INITIAL); set BIT0 WCPU_FWDL_EN.
5. 885xB: `MASK32(0x0C00 SEC_CTRL, [17:16], 2)` (SEC_IDMEM_SIZE_CONFIG).
6. `MASK16(0x01E6 BOOT_REASON, [2:0], 0)`.
7. `SET32(0x0088, BIT1)` WCPU_EN.

Then the FW track: poll `R8(0x01E0) & BIT1` (H2C_PATH_RDY), 1 µs step, 400 000 µs
(`rtw89_fwdl_check_path_ready_ax`, mac.c:7399); download; `mdelay(5)`; poll
`R8(0x01E0)[7:5] == 7` (WCPU_FW_INIT_RDY) 1 µs/400 000 µs (fw.c:106). Status codes in [7:5]:
0 initial, 1 ongoing, 2 checksum fail, 3 security fail, 4 CV mismatch, 6 FWDL ready, 7 FW init ready
(fw.h:10-19). Whole download retried up to 5× (fw.c:2014).

---------------------------------------------------------------------------------------------
## 6. `rtw89_mac_partial_init` — everything BEFORE firmware download (mac.c:4307)

1. `rtw89_mac_ctrl_hci_dma_trx(true)`: `SET32(0x8380 HCI_FUNC_EN, BIT0|BIT1)` (HCI TXDMA/RXDMA) (mac.h:1613).
2. (bb_preinit — NULL.)
3. `rtw89_mac_dmac_pre_init` (mac.c:4258):
   1. `hci_func_en_ax`: **W32(0x8400) = 0x60440000** (MAC_FUNC_EN b30 | DMAC_FUNC_EN b29 | PKT_BUF_EN b22 | DISPATCHER_EN b18) — full overwrite.
   2. `dmac_func_pre_en_ax`: **W32(0x8404 DMAC_CLK_EN) = 0x00040000** (DISPATCHER_CLK_EN only; the AXIDMA clock is added only for 8851B/8852BT).
   3. `rtw89_mac_dle_init(RTW89_QTA_DLFW, ext=SCC)` (§6.1).
   4. `rtw89_mac_hfc_init(reset=1, en=0, h2c_en=1)` (§6.2).
4. PCIe `mac_pre_init` (PCIe track: PHY tweaks, stop DMA, reset BD rings, enable only FWCMD channel).
5. `fwdl_preconfig` — NULL.
6. `rtw89_fw_download(NORMAL)` (§5 + FW track).

### 6.1 DLE init — common procedure (`rtw89_mac_dle_init`, mac.c:2274)
1. Require `R32(0x8400)` has BIT30|BIT29 (DMAC enabled, `rtw89_mac_check_mac_en_ax`, mac.c:62); else fail.
2. Select table row `rtw8852b_dle_mem_pcie[mode]`; record `ple_pg_size`, `c0_rx_qta = ple_min_qt.cma0_dma`, `qta_mode` (mac.c:1906).
3. Sanity: `wde.pg*(lnk+unlnk) + ple.pg*(lnk+unlnk)` must equal `fifo_size (196608)` minus
   `dle_scc_rsvd_size (98304)` for SCC (not subtracted for DLFW) (mac.c:2041-2067).
4. `CLR32(0x8400, BIT26|BIT23)` (DLE_WDE_EN, DLE_PLE_EN).
5. `SET32(0x8404, BIT26|BIT23)` (DLE_WDE_CLK_EN, DLE_PLE_CLK_EN).
6. `dle_mix_cfg_ax` (mac.c:2092):
   - RMW **0x8C08 WDE_PKTBUF_CFG**: [1:0] PAGE_SEL (0=64 B), [13:8] START_BOUND = 0, [28:16] FREE_PAGE_NUM = wde.lnk_pge_num.
   - RMW **0x9008 PLE_PKTBUF_CFG**: [1:0] PAGE_SEL (1=128 B), [13:8] START_BOUND = (wde.lnk+wde.unlnk)*64/8192, [28:16] FREE_PAGE_NUM = ple.lnk_pge_num.
7. Quotas (mac.c:2178-2212). Each quota register = `min | max<<16` (min [11:0], max [27:16]), full W32:
   - WDE: QTA0 0x8C40 = hif, QTA1 0x8C44 = wcpu, QTA3 0x8C4C = pkt_in, QTA4 0x8C50 = cpu_io (QTA2 = DCPU not written).
   - PLE: QTA0 0x9040 cma0_tx (B0 TX payload), QTA1 0x9044 cma1_tx, QTA2 0x9048 c2h, QTA3 0x904C h2c, QTA4 0x9050 wcpu, QTA5 0x9054 mpdu_proc, QTA6 0x9058 cma0_dma (B0 RX), QTA7 0x905C cma1_dma, QTA8 0x9060 bb_rpt, QTA9 0x9064 wd_rel, QTA10 0x9068 cpu_io. (QTA11 tx_rpt only on 8852C.)
   - In DLFW mode the WDE wcpu *min* is replaced by the SCC table's wcpu min (48).
8. `SET32(0x8400, BIT26|BIT23)`.
9. `POLL32(0x8D00 WDE_INI_STATUS, [1:0]==3, 1 µs, 2000 µs)` then `POLL32(0x9100 PLE_INI_STATUS, [1:0]==3, 1, 2000)`.

### 6.2 DLE tables for 8852B PCIe (rtw8852b.c:94-108, mac.c:1716-1903)

| Mode | WDE size (pg B, linked, unlinked) | PLE size | WDE min=max | PLE min | PLE max |
|---|---|---|---|---|---|
| **DLFW** (pre-FW) | wde_size9 = 64, 0, 1024 | ple_size8 = 128, 64, 960 | wde_qt4 = 0,0,0,0 | ple_qt13 | ple_qt13 |
| **SCC** (normal) | wde_size7 = 64, 510, 2 | ple_size6 = 128, 496, 16 | wde_qt7 = 446,48,0,16 | ple_qt18 | ple_qt58 |
| WOW (not used) | wde_size7 | ple_size6 | wde_qt7 | ple_qt18 | ple_qt_52b_wow |

WDE quota fields order: hif, wcpu, pkt_in, cpu_io. PLE fields: cma0_tx, cma1_tx, c2h, h2c, wcpu,
mpdu_proc, cma0_dma, cma1_dma, bb_rpt, wd_rel, cpu_io, tx_rpt:

| PLE table | cma0_tx | cma1_tx | c2h | h2c | wcpu | mpdu | cma0_dma | cma1_dma | bb_rpt | wd_rel | cpu_io | tx_rpt |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ple_qt13 (DLFW) | 0 | 0 | 16 | 48 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | – |
| ple_qt18 (SCC min) | 147 | 0 | 16 | 20 | 17 | 13 | 89 | 0 | 32 | 14 | 8 | 0 |
| ple_qt58 (SCC max) | 147 | 0 | 16 | 20 | 157 | 13 | 229 | 0 | 172 | 14 | 24 | 0 |

Size check: SCC 64*512 + 128*512 = 98304 = 196608−98304 ✓; DLFW 64*1024 + 128*1024 = 196608 ✓.

Resulting register values:

| Reg | DLFW | SCC |
|---|---|---|
| 0x8C08 WDE_PKTBUF_CFG (fields) | pg_sel 0, bound 0, free 0 | pg_sel 0, bound 0, free 510 (0x1FE) |
| 0x9008 PLE_PKTBUF_CFG | pg_sel 1, bound 8, free 64 | pg_sel 1, bound 4, free 496 (0x1F0) |
| 0x8C40 WDE QTA0 hif | 0x00000000 | 0x01BE01BE (446/446) |
| 0x8C44 WDE QTA1 wcpu | 0x00000030 (min 48 from SCC, max 0) | 0x00300030 |
| 0x8C4C WDE QTA3 pkt_in | 0 | 0 |
| 0x8C50 WDE QTA4 cpu_io | 0 | 0x00100010 |
| 0x9040 PLE QTA0 cma0_tx | 0 | 0x00930093 |
| 0x9044 PLE QTA1 | 0 | 0 |
| 0x9048 PLE QTA2 c2h | 0x00100010 | 0x00100010 |
| 0x904C PLE QTA3 h2c | 0x00300030 | 0x00140014 |
| 0x9050 PLE QTA4 wcpu | 0 | 0x009D0011 |
| 0x9054 PLE QTA5 mpdu | 0 | 0x000D000D |
| 0x9058 PLE QTA6 cma0_dma | 0 | 0x00E50059 |
| 0x905C PLE QTA7 | 0 | 0 |
| 0x9060 PLE QTA8 bb_rpt | 0 | 0x00AC0020 |
| 0x9064 PLE QTA9 wd_rel | 0 | 0x000E000E |
| 0x9068 PLE QTA10 cpu_io | 0 | 0x00180008 |

### 6.3 HFC (HCI flow control) — `rtw89_mac_hfc_init` (mac.c:1194)

Page-control registers (rtw8852b_page_regs, rtw8852b.c:139): HCI_FC_CTRL 0x8A00, CH_PAGE_CTRL 0x8A04,
ACH0_PAGE_CTRL 0x8A10 (+4·ch), ACH0_PAGE_INFO 0x8A50 (+4·ch), PUB_PAGE_INFO3 0x8A8C,
PUB_PAGE_CTRL1 0x8A90, PUB_PAGE_CTRL2 0x8A94, PUB_PAGE_INFO1 0x8A98, PUB_PAGE_INFO2 0x8A9C,
WP_PAGE_CTRL1 0x8AA0, WP_PAGE_CTRL2 0x8AA4, WP_PAGE_INFO1 0x8AA8.

Fields: 0x8A00: b0 HCI_FC_EN, [2:1] MODE (0=POH for PCIe), b3 CH12 (H2C) EN, [5:4] WD_FULL_COND,
[7:6] WP_CH07_FULL_COND, [9:8] WP_CH811_FULL_COND, [11:10] CH12_FULL_COND.
0x8A04: [8:0] PREC_PAGE_CH011, [24:16] PREC_PAGE_CH12. ACHn_PAGE_CTRL: [12:0] MIN, [28:16] MAX, b31 GRP.
0x8A90: [12:0] G0, [28:16] G1. 0x8A94: [12:0] PUB_ALL. 0x8AA0: [8:0] WP_CH07 prec, [24:16] WP_CH811 prec.
0x8AA4: [12:0] WP_THRD. INFO regs: ACHn_PAGE_INFO [27:16] AVAL, [12:0] USED; PUB_INFO1 [12:0] G0_USE,
[28:16] G1_USE; PUB_INFO3 [12:0] G0_AVAL, [28:16] G1_AVAL; PUB_INFO2 [12:0] PUB_AVAL; WP_INFO1 [28:16] WP_AVAL.

8852B PCIe parameters (rtw8852b.c:21-50, mac.c:1717):
- prec cfg (`hfc_preccfg_pcie`) = ch011_prec 2, h2c_prec 40, wp_ch07_prec 0, wp_ch811_prec 0, ch011_full_cond 1, h2c_full_cond 0, wp_ch07_full_cond 0, wp_ch811_full_cond 0; mode POH (0).
- pub cfg (SCC) = grp0 446, grp1 0, pub_max 446, wp_thrd 0.
- ch cfg (SCC) {min, max, grp}: ACH0 {5,341,0}, ACH1 {5,341,0}, ACH2 {4,342,0}, ACH3 {4,342,0}, ACH4–7 {0,0,0}, B0MGQ(ch8) {4,342,0}, B0HIQ(ch9) {4,342,0}, B1MGQ/B1HIQ {0,0,0}, FWCMDQ(ch12) {40,0,0}.
- `dma_ch_mask` = ACH4–7, B1MG, B1HI (rtw8852b.c:1106) → those channels are never configured.

**DLFW call (reset=1, en=0, h2c_en=1)** — param row = DLFW (ch/pub NULL, prec as above):
1. require DMAC enabled.
2. RMW 0x8A00: clear b0, b3.
3. `W32(0x8A04) = 0x00280000` (h2c prec 40; ch011 prec 0).
4. `MASK32(0x8A00, [11:10], 0)`.
5. RMW 0x8A00: clear b0, set b3 (H2C channel flow control only).

**SCC call (reset=1, en=1, h2c_en=1)** (from dmac_init §9.1):
1. require DMAC enabled; RMW 0x8A00 clear b0, b3.
2. Per channel 0,1,2,3,8,9 (check min≥prec(2) or 0, max≤446): `W32(0x8A10+4·ch)`:
   ch0 0x01550005, ch1 0x01550005, ch2 0x01560004, ch3 0x01560004, ch8 (0x8A30) 0x01560004, ch9 (0x8A34) 0x01560004.
3. check grp0+grp1 == pub_max; `W32(0x8A90) = 0x000001BE`; `W32(0x8AA4) = 0`.
4. `W32(0x8A04) = 0x00280002`; `W32(0x8A94) = 0x000001BE`; `W32(0x8AA0) = 0`;
   RMW 0x8A00: MODE=0, WD_FULL_COND=1, CH12_FULL_COND=0, WP_CH07=0, WP_CH811=0.
5. RMW 0x8A00: set b0 and b3; `udelay(10)`.
6. Read back ACHn_PAGE_INFO for ch 0–3,8,9 and PUB/WP info (bookkeeping). Consistency check
   `G0_USE + G1_USE + PUB_AVAL == 446`, else -EFAULT (mac.c:945, 1108).

---------------------------------------------------------------------------------------------
## 7. BB/RF power (`__rtw8852bx_mac_enable_bb_rf`, rtw8852b_common.c:2001)

Enable (called in mac_init after FW is up, and again in `rtw89_chip_reset_bb_rf` after a disable):
1. `SET8(0x0002 SYS_FUNC_EN, BIT0|BIT1)` (FEN_BBRSTB, FEN_BB_GLB_RSTN) — releases BB reset.
2. `MASK32(0x0200, [18:17], 1)` (REG_ZCDC_H).
3. `SET32(0x02F0 WLRF_CTRL, BIT17)`, `CLR32(0x02F0, BIT17)`, `SET32(0x02F0, BIT17)` (AFC_AFEDIG pulse, ends 1).
4. (8852BT-only LDO_VSEL write — skipped.)
5. `XSI_W(0x80, 0xC7, 0xFF)` (WL_RFC_S0); `XSI_W(0x81, 0xC7, 0xFF)` (WL_RFC_S1) — RF path A/B power.
6. `W8(0x8040 PHYREG_SET) = 0x0E` (PHYREG_SET_XYN_CYCLE).

Disable (`__rtw8852bx_mac_disable_bb_rf`, :2036):
1. `CLR32(0x02F0, BIT17)`.
2. `CLR8(0x0002, BIT0|BIT1)`.
3. `v = XSI_R(0x80); XSI_W(0x80, v & ~0x07, 0xFF)`; same for 0x81.

BB-only reset helpers (PHY registers, owned by PHY track): `rtw8852b_bb_reset` / `__rtw8852bx_bb_reset_all`
(rtw8852b_common.c:1077) toggle `R_S0/S1_HW_SI_DIS` and `R_RSTB_ASYNC`.
There is no separate "RF on/off" MAC helper; RF power is the XTAL-SI 0x80/0x81 write above.

---------------------------------------------------------------------------------------------
## 8. `sys_init_ax` (mac.c:1697)
1. `dmac_func_en_ax` (mac.c:1652): **W32(0x8400) = 0xFB7D0000** = MAC_FUNC b30, DMAC_FUNC b29, DMAC_CRPRT b31,
   MPDU_PROC b28, WD_RLS b27, TXPKT_CTRL b25, STA_SCH b24, PKT_BUF b22, DMAC_TBL b21, PKT_IN b20,
   DLE_CPUIO b19, DISPATCHER b18, MAC_SEC b16. (**DLE_WDE b26 / DLE_PLE b23 are cleared here**; they
   are re-enabled by the SCC DLE init. BBRPT b17 and DMACREG_GCKEN b15 end up cleared.)
   **W32(0x8404) = 0x0B1F0000** = MAC_SEC_CLK b16, BBRPT_CLK b17, DISPATCHER_CLK b18, DLE_CPUIO_CLK b19,
   PKT_IN_CLK b20, STA_SCH_CLK b24, TXPKT_CTRL_CLK b25, WD_RLS_CLK b27 (DLE clocks re-added by DLE init).
2. `cmac_func_en_ax(0, true)` (mac.c:1606): `SET32(0xC004 CK_EN, 0x4000003F)`; `SET32(0xC000 CMAC_FUNC_EN, 0xF000003F)`
   (adds CMAC_CRPRT b31). Band-1 branch (AFE_CTRL1 0x0024 C1 power, ISO_CTRL_EXTEND 0x0080) — **skip, band 1 only**.
3. `chip_func_en_ax`: `SET32(0x0200, 0x0000E000)` (OCP_L1 [15:13] = 7).

After §9 the steady values are: 0x8400 = 0xFFFD0000, 0x8404 = 0x0F9F0000.

---------------------------------------------------------------------------------------------
## 9. `trx_init_ax` (mac.c:4029)

Order: dmac_init → cmac_init(0) → (dbcc: no, `rtw89_mac_is_qta_dbcc(SCC)` false because
ple_qt18.cma1_dma = 0) → DMAC IMRs → CMAC0 IMRs → err IMR → host RPR.

### 9.1 DMAC init (`dmac_init_ax`, mac.c:2521)
1. **DLE init, SCC** (§6.1/6.2).
2. Preload init — skipped for 885xB (mac.c:2375).
3. **HFC init, SCC, en+h2c** (§6.3).
4. **STA scheduler** (`sta_sch_init_ax`, mac.c:2425):
   1. `SET8(0x9E10 SS_CTRL, BIT0)` SS_EN.
   2. `POLL32(0x9E10, BIT31 SS_INIT_DONE_1, 1 µs, 2000 µs)`.
   3. `SET32(0x9E10, BIT29)` SS_WARM_INIT_FLG; `CLR32(0x9E10, BIT28)` SS_NONEMPTY_SS2FINFO_EN.
   4. (`_patch_ss2f_path` skipped for 885xB.)
5. **MPDU processor** (`mpdu_proc_init_ax`, mac.c:2458):
   `W32(0x9C04 ACTION_FWD0) = 0x02A95A95`; `W32(0x9C14 TF_FWD) = 0x0000AA55`;
   `SET32(0x9C00 MPDU_PROC, BIT0|BIT1)` (APPEND_FCS, A_ICV_ERR); `W32(0x9C40 CUT_AMSDU_CTRL) = 0x010E05F0`.
   Note APPEND_FCS=1 → RX MPDUs are delivered with the 4-byte FCS appended.
6. **Security engine** (`sec_eng_init_ax`, mac.c:2479): RMW 0x9D00 SEC_ENG_CTRL: set BIT10|BIT9|BIT8
   (CLK_EN_CGCMP/WAPI/WEP_TKIP), BIT0 SEC_TX_ENC, BIT1 SEC_RX_DEC, BIT3 MC_DEC, BIT2 BC_DEC; clear BIT11
   TX_PARTIAL_MODE (i.e. `|= 0x70F, &= ~0x800`). `SET32(0x9D04 SEC_MPDU_PROC, BIT1|BIT0)` (APPEND_ICV, APPEND_MIC).
   (PMF helper `rtw89_mac_hw_mgnt_sec` would set 0x9D00 BIT4|BIT5; not called at init.)

"dmac_tbl" / "sta_tbl": not initialised globally; per-MACID tables are written at interface
creation (§12.5) and via FW H2C.

### 9.2 CMAC0 init (`cmac_init_ax`, mac.c:3093). Each sub-step first requires 0xC000 BIT30.

| Sub-init | Writes (band 0) | Cite |
|---|---|---|
| scheduler | `MASK32(0xC33C PREBKF_CFG_1, [6:0], 0x47)` (SIFS_MACTXEN_T1); 885xB: `SET32(0xC3FC SCH_EXT_CTRL, BIT1)` PORT_RST_TSF_ADV; `CLR32(0xC340 CCA_CFG_0, BIT5)` BTCCA_EN; `MASK32(0xC338 PREBKF_CFG_0, [4:0], 0x18)` (24 µs) | mac.c:2591 |
| addr CAM | RMW 0xCE34 ADDR_CAM_CTRL `|= 0x007F0000 (RANGE=0x7F) | BIT8 CLR | BIT0 EN`; `POLL16(0xCE34, BIT8==0, 1 µs, 2000 µs)` | mac.c:2564 |
| RX filter | `W32(0xCE28 MGNT_FLTR)=0x55555555`, `W32(0xCE24 CTRL_FLTR)=0x55555555`, `W32(0xCE2C DATA_FLTR)=0x55555555` (every subtype → host); `W32(0xCE20 RX_FLTR_OPT) = hal.rx_fltr = 0x030044BE`; `W16(0xCE04 PLCP_HDR_FLTR) = 0x007F` | mac.c:2692 |
| CCA ctrl | RMW 0xC390 CCA_CONTROL `|= 0x712100FF`, `&= ~0x8E1E0100`; then 8852B `_patch_dis_resp_chk`: `CLR32(0xCC00 RSP_CHK_SIG, 0x00380000)` (TXNAV/INTRA_NAV/BASIC_NAV), `CLR32(0xCC04 TRXPTCL_RESP_0, 0x1F800000)` (SEC_CCA_80/40/20, BTCCA, EDCCA, CCA) | mac.c:2758, 2724 |
| NAV | `SET32(0xCC80 WMAC_NAV_CTL, BIT26|BIT17|BIT16)` (NAV_UPPER_EN, PLCP_UP_NAV_EN, TF_UP_NAV_EN); `MASK32(0xCC80, [15:8], 0xC4)` (25 ms upper) | mac.c:2794 |
| spatial reuse | `CLR8(0xCE4A RX_SR_CTRL, BIT0)` SR_EN; `SET8(0xCE4B BSSID_SRC_CTRL, BIT0)` PLCP_SRC_EN | mac.c:2808 |
| TMAC | `CLR32(0xCC20 MAC_LOOPBACK, BIT0)`; `MASK32(0xCA00 TCR0, [22:16], 6)`; `MASK32(0xCA1C TXD_FIFO_CTRL, [15:12], 7)`; `MASK32(0xCA1C, [11:8], 7)` | mac.c:2830 |
| TRX protocol | RMW 0xCC04: [7:0] SPEC_SIFS_CCK = 0x0A, [15:8] SPEC_SIFS_OFDM = 0x11 (52B value); `SET32(0xCCB0 RXTRIG_TEST_USER_2, BIT20)` FCSCHK_EN; `MASK32(0xCC08 TRXPTCL_RRSR_CTL_0, BIT9, 0)` (resp ref rate sel); `MASK32(0xCC08, [11:10], 2)` (resp RSC) | mac.c:2856, rtw8852b.c:203 |
| RMAC | BA CAM reset: `MASK32(0xCE3C RESPBA_CAM_CTRL, [1:0], 2)`, poll [1:0]==0 (1 µs, 1000 µs, atomic); `SET8(0xCE3C, BIT2)` SSN_SEL; RMW16 0xCE02 DLK_PROTECT_CTL: [7:4]=15 (data TO), [15:8]=32 (CCA TO); `MASK8(0xCE00 RCR, [3:0], 1)` CH_EN; `MASK32(0xCE20, [21:16] RX_MPDU_MAX_LEN, 22)` (=min(89 pages,127)×128 B=11392 → /512 = 22); `CLR8(0xCE04, BIT4)` (VHT_SU_SIGB_CRC_CHK off → PLCP filter 0x6F) | mac.c:2919 |
| CMAC common | RMW 0xC088 TX_SUB_CARRIER_VALUE: [3:0]=[7:4]=[11:8]=0; `MASK32(0xC090 PTCL_RRSR1, [11:8], 3)` (RRSR OFDM+CCK) | mac.c:2983 |
| PTCL | PCIe: RMW 0xC624 SIFS_SETTING: [31:24]=4 (HW CTS2SELF len th 1K), [23:18]=1 (256 B), set BIT16 HW_CTS2SELF_EN; RMW 0xC6E8 PTCL_FSM_MON: [5:0]=0x3F (2 ms), clear BIT6. Band0: `SET8(0xC600 PTCL_COMMON_SETTING_0, BIT0|BIT1)` (CMAC_TX_MODE_0/1), `CLR8(0xC600, BIT2|BIT3|BIT4)` (TRIGGER_SS_EN_0/1/UL); `MASK8(0xC660 PTCLRPT_FULL_HDL, [5:4], 1)` (to WLCPU). 885xB: `MASK32(0xC618 AGG_LEN_VHT_0, [19:0], 0x3FF80)` | mac.c:3021 |
| CMAC DMA | 885xB: `CLR8(0xC804 RXDMA_CTRL_0, 0x3F)` (RU0–3/CSI/RXSTS "full mode" off) | mac.c:3075 |

Band-1-only (skippable): `PTCLRPT_FULL_HDL_C1`, `band1_enable_ax` (mac.c:3678), `dbcc_enable_ax`,
CMAC1 error IMR.

### 9.3 Error interrupt masks (`enable_imr_ax`, mac.c:3936; values from `rtw8852b_imr_info`, rtw8852b.c:157)
These feed FW's SER (system-error-recovery) reporting. Recommended but not functionally required for RX/TX.

DMAC (each line = CLR32 then SET32):

| Reg | clear | set |
|---|---|---|
| 0x9430 WDRLS_ERR_IMR | 0x00003337 | 0x00003327 |
| 0x9D1C SEC_DEBUG | — | 0x00000008 (IMR_ERROR) |
| 0x9BF4 MPDU_TX_ERR_IMR | 0x0000003E | 0 |
| 0x9CF4 MPDU_RX_ERR_IMR | 0x0000000B | 0 |
| 0x9EF0 STA_SCHEDULER_ERR_IMR | 0x00000007 | 0x00000007 |
| 0x9F1C TXPKTCTL_ERR_IMR_ISR | 0x0000030F | 0x00000101 |
| 0x9F2C TXPKTCTL_ERR_IMR_ISR_B1 | 0x0000030F | 0x00000303 |
| 0x8C38 WDE_ERR_IMR | 0x070FF0FF | 0x070FF0FF |
| 0x9038 PLE_ERR_IMR | 0x070FF0FF | 0x070FF0DF |
| 0x9A20 PKTIN_ERR_IMR | — | 0x00000001 |
| 0x8850 HOST_DISPATCHER_ERR_IMR | 0xFF0FFFFF | 0x0C000161 |
| 0x8854 CPU_DISPATCHER_ERR_IMR | 0xFF07FFFF | 0x04000062 |
| 0x8858 OTHER_DISPATCHER_ERR_IMR | 0x3F031F1F | 0 |
| 0x9840 CPUIO_ERR_IMR | 0x00001111 | 0x00001111 |
| 0x960C BBRPT_COM_ERR_IMR_ISR | — | 0x00000001 |
| 0x962C BBRPT_CHINFO_ERR_IMR_ISR | 0x000000FF | 0 |
| 0x963C BBRPT_DFS_ERR_IMR_ISR | — | 0x00000001 |
| 0x966C LA_ERRFLAG | — | 0x00000001 |

CMAC0:

| Reg | clear | set |
|---|---|---|
| 0xC3E8 SCHEDULE_ERR_IMR | 0x00000003 | 0x00000001 |
| 0xC6C0 PTCL_IMR0 | 0xFFFFFFFF | 0x10800001 |
| 0xC800 DLE_CTRL (CDMA IMR) | 0x0080C000 | 0x0000C000 |
| 0xCCFC PHYINFO_ERR_IMR | 0 | 0 (no-op) |
| 0xCEF4 RMAC_ERR_ISR | 0x000FF000 | 0x000E4000 |
| 0xCCEC TMAC_ERR_IMR_ISR | 0x00000780 | 0x00000780 |

Then `err_imr_ctrl_ax(true)` (mac.c:3974): `W32(0x8520 DMAC_ERR_IMR) = 0xFFFFFFFF`; `W32(0xC160 CMAC_ERR_IMR) = 0xFFFFFFFF`.

### 9.4 Host release-report path (`set_host_rpr_ax`, mac.c:4009), PCIe
`MASK32(0x9408 WDRLS_CFG, [1:0], 0)` (POH mode); `SET32(0x9410 RLSRPT0_CFG0, 0x0F000000)` (filter map);
`MASK32(0x9414 RLSRPT0_CFG1, [7:0], 30)` (agg num); `MASK32(0x9414, [23:16], 255)` (timeout).
**Mandatory** — this is how TX-completion reports reach the host RX ring (TX/RX track).

---------------------------------------------------------------------------------------------
## 10. Post-init MAC settings in `rtw89_core_start`
- PPDU status reports (`rtw89_mac_cfg_ppdu_status_ax`, mac.c:6289): `W32(0xCE40 PPDU_STAT) = 0x0000002B`
  (RPT_EN b0, APP_MAC_INFO b1, APP_PLCP_HDR b3, RPT_CRC32 b5); `MASK32(0x9C18 HW_RPT_FWD, [1:0], 1)` (to host).
  Needed for RSSI/PHY status on RX. Disabled (`CLR32(0xCE40, BIT0)`) around channel switch.
- RTS threshold (mac.c:6314), default `rts_threshold = -1`: `MASK16(0xC614 AGG_LEN_HT_0, 0xFF00, 2)` (88 µs>>5)
  and `MASK16(0xC614, 0x00FF, 255)` (4080>>4).
- Scheduler TX pause/resume (used by channel switch): register **0xC348 CTN_TXEN** [15:0] (all queues;
  b8 MGQ, b11 HGQ). When FW is running rtw89 does NOT write it directly but sends H2C-register
  func 5 SCH_TX_EN (w0[31:16]=enable, w1[15:0]=mask, w1 b16 band) and waits for C2H-reg TX_PAUSE_RPT (mac.c:3338-3495).
- Per-channel MAC writes (`__rtw8852bx_set_channel_mac`, rtw8852b_common.c:451): `MASK8(0xC010 WMAC_RFMOD, [1:0])` = 0/1/2 for 20/40/80 MHz;
  `W32(0xC088) = txsc20 | txsc40<<4` (0 for 20 MHz); 0xC628 TXRATE_CHK: 5 GHz → clr b4 BAND_MODE, set b0|b1; 2.4 GHz → set b4, clr b0|b1.

---------------------------------------------------------------------------------------------
## 11. Efuse

### 11.1 Physical dump, normal ("DDV") path (efuse.c:74-138)
Enable efuse power (`rtw89_enable_efuse_pwr_cut_ddv`, 8852B):
1. `SET8(0x00CC, BIT2)`; 2. `SET16(0x0000, BIT14)`; 3. sleep 1000 µs; 4. `SET16(0x0000, BIT15)`;
5. `CLR16(0x0000, BIT8)` (ISO_EB2CORE off). (OTP burst mode only for CAV — skipped.)

Per byte `addr`:
1. `W32(0x0030 EFUSE_CTRL) = addr << 16` (ADDR [26:16], RDY b29 = 0, MODE [31:30] = 0).
2. Poll `R32(0x0030) & BIT29` — 1 µs step, 1 000 000 µs timeout; fail → -EBUSY.
3. byte = `R32(0x0030) & 0xFF`.

Disable: `SET16(0x0000, BIT8)`; `CLR16(0x0000, BIT15)`; sleep 1000 µs; `CLR16(0x0000, BIT14)`; `CLR8(0x00CC, BIT2)`.
Whole dump retried up to 5 times (efuse.c:211). Note: on a poll failure rtw89 returns without the
disable step (the retry re-enables). Efuse-bank switch (`R_AX_EFUSE_CTRL_1`) is 8852A-only.

### 11.2 Autoload flag
`efuse.valid = R16(0x000A SYS_WL_EFUSE_CTRL) & BIT5` (AUTOLOAD_SUS) (efuse.c:295). If 0 rtw89 only warns.

### 11.3 DAV ("A-die") efuse via XTAL-SI (efuse.c:145-186), 96 bytes (`dav_phy_efuse_size`)
Per byte: `XSI_W(0x63, 0x40, 0xFF)`; `XSI_W(0x62, addr&0xFF, 0xFF)`; `XSI_W(0x63, addr>>8, 0x07)`;
`XSI_W(0x63, 0, 0xC0)` (mode sel = 0); poll `XSI_R(0x63) & BIT5` (RDY) 1 µs/10 000 µs; byte = `XSI_R(0x7A)`.
Decoded into a 16-byte logical area appended after the 2048-byte map. **Nothing in the 8852B
parser reads it** → optional for a minimal driver.

### 11.4 Physical → logical decode (`rtw89_dump_logical_efuse_map`, efuse.c:238)
8852B sizes: physical 1216 (0x4C0), logical 2048 (0x800), sec_ctrl 4 (rtw8852b.c:1044-1050).
Logical buffer pre-filled with 0xFF.
```
i = 4                                   # skip 4 security-control bytes
while i < 1216-4:
    h1 = P[i]; h2 = P[i+1]
    if h1 == 0xFF or h2 == 0xFF: stop
    blk  = ((h1 & 0x0F) << 4) | (h2 >> 4)      # 8-byte block index 0..255
    wen  = h2 & 0x0F                           # word-enable, ACTIVE LOW
    i += 2
    for w in 0..3:
        if wen & (1<<w): continue              # word absent
        L[blk*8 + w*2 + 0] = P[i]; L[blk*8 + w*2 + 1] = P[i+1]; i += 2
        (abort -EINVAL if i+1 > 1211 or log index+1 > 2048)
```
Then `__rtw8852bx_read_efuse` interprets the logical map (struct `rtw8852bx_efuse`,
rtw8852b_common.h:36; offsets computed with offsetof, bit-fields LSB-first):

| Logical offset | Field | Used by rtw89 |
|---|---|---|
| 0x210–0x215 | path A CCK TSSI DE, 6 ch-groups | TSSI (`tssi_cck[A][0..5]`) |
| 0x216–0x21A | path A 2G BW40 1S TSSI, 5 groups | `tssi_mcs[A][0..4]` |
| 0x21B–0x221 | rsvd | |
| 0x222–0x22F | path A 5G BW40 1S TSSI, 14 groups | `tssi_mcs[A][5..18]` |
| 0x230–0x239 | rsvd | |
| 0x23A–0x23F | path B CCK TSSI (6) | `tssi_cck[B]` |
| 0x240–0x244 | path B 2G BW40 (5) | `tssi_mcs[B][0..4]` |
| 0x245–0x24B | rsvd | |
| 0x24C–0x259 | path B 5G (14) | `tssi_mcs[B][5..18]` |
| 0x2B8 | channel_plan | no |
| **0x2B9** | **xtal_k** (crystal cap) | yes → `efuse.xtal_cap`; PHY DM init writes `xtal_k & 0x7F` to XSI 0x05 (SC_XO) and 0x04 (SC_XI), full mask (phy.c:5038, 4949) |
| 0x2BB | iqk_lck | no |
| 0x2C1 | b[1:0] reg_setting, b2 tx_diversity, b[4:3] rx_diversity, b5 ac_mode, b[7:6] module_type | no |
| 0x2C3 | b0 shared_ant, b[3:1] coex_type, b4 ant_iso, b5 radio_on_off | no (coex uses RFE type instead) |
| 0x2C4 / 0x2C5 | eeprom_version / customer_id | no |
| 0x2C6 / 0x2C7 | tx_bb_swing_2g / _5g | no |
| 0x2C8 / 0x2C9 | tx_cali_pwr_trk_mode / trx_path_selection | no |
| **0x2CA** | **rfe_type** | yes (=1 here) |
| **0x2CB–0x2CC** | **country_code[2]** (ASCII) | yes (regd) |
| **0x2D0 / 0x2D1** | **thermal A / B** (TSSI thermal base) | yes |
| **0x2D4** | rx_gain_2g_ofdm | yes |
| **0x2D6** | rx_gain_2g_cck | yes |
| **0x2D8 / 0x2DA / 0x2DC** | rx_gain_5g_low / mid / high | yes |
| 0x300–0x305 | path A CCK pwr idx (6) | no |
| 0x306–0x30A | path A BW40 1TX pwr idx (5) | no |
| 0x30B–0x30D | nibble diffs (ofdm/bw20 1tx; bw20/bw40 2tx; cck/ofdm 2tx) | no |
| **0x400–0x405** | **MAC address (PCIe "e" variant)** | yes → permanent MAC (core.c:7375) |
| 0x488–0x48D | MAC address (USB variant) | not for PCIe |

RX-gain byte decode (`_decode_efuse_gain`, rtw8852b_common.c:206): high nibble (signed 4-bit) → path A
offset, low nibble (signed 4-bit) → path B; `offset_valid` if any of the 5 bytes ≠ 0xFF.
TSSI bytes are copied verbatim (signed values interpreted by TSSI code — RF-calibration track).

### 11.5 PHYCAP map (`rtw89_parse_phycap_map_ax`, efuse.c:350; `__rtw8852bx_read_phycap`, rtw8852b_common.c:434)
Physical efuse range 0x580–0x5FF (128 bytes) read via §11.1 (NOT via the logical map). Offsets
below are physical addresses:

| Phys addr | Meaning | Handling |
|---|---|---|
| 0x5E9 | power-K check | `power_k_valid = (byte == 0xAA)` → gates pwr_on steps 36–38 |
| 0x5D6,0x5D5,…,0x5CF | TSSI trim path A, groups 0..7 (decreasing addresses) | if all 16 A+B bytes are 0xFF → all trims = 0 |
| 0x5AB,0x5AA,…,0x5A4 | TSSI trim path B, groups 0..7 | |
| 0x5DF / 0x5DC | thermal trim A / B | if ≠0xFF: `RF_W(path, 0x43 RR_TM2, [19:16], ((v&1)<<3) | ((v&0x1F)>>1))` |
| 0x5DE / 0x5DB | PA-bias trim A / B | if ≠0xFF: `RF_W(path, 0x60 RR_BIASA, [15:12], v & 0xF)` (2G) and `[19:16] = v >> 4` (5G) |
| A: 0x5BB,0x5BA,–,0x5B9,0x5B8 ; B: 0x590,0x58F,–,0x58E,0x58D | RX gain comp per subband (2G, 5G-band1, [band2 unused], 5G-band3, 5G-band4) | signed low nibble; `comp_valid` if any ≠0xFF |

Thermal/PA-bias trims are applied by `chip->ops->power_trim` after RF init (RF track).

### 11.6 FW-secure bytes (`rtw89_efuse_read_fw_secure_ax`, efuse.c:484)
Physical 0x5EC–0x5ED via §11.1, during the first power-on in probe. `FF FF` → no secure boot.
8852B special case `b1=0xFF, b2=0x6E` → MSS dev type "NONLIN_INBOX_NON_COB", cust 0, key 0 (efuse.c:431).
Otherwise decoded per efuse.c:427-481. `secure_boot` selects the signed FW section and suppresses the
direct DMAC/CMAC table writes of §12.5 (FW track).

### 11.7 PHY capability from FW (`rtw89_mac_setup_phycap`, mac.c:3322)
Uses the H2C/C2H **register** mailbox (fw.c:8188-8295):
- H2C: poll `R8(0x8160 H2CREG_CTRL) == 0` (1 ms step, 5 ms); write 4 dwords to 0x8140/44/48/4C
  (w0: [6:0] func, [11:8] len in dwords, [23:16] part number); increment counter in
  `R8(0x01F5)[3:0]` (UDM1 byte1, HALMAC_H2C_DEQ_CNT); `W8(0x8160) = 1`.
- C2H: poll `R8(0x8164 C2HREG_CTRL) != 0` (1 µs, 1 s); read 0x8150/54/58/5C; `W8(0x8164)=0`;
  counter `R8(0x01F5)[7:4]`.
- Request: func 3 (GET_FEATURE), len 1 dword, part 0. Expected reply func 3 (PHY_CAP):
  w0[23:16] RX_NSS, w1[7:0] TX_NSS, w1[15:8] protocol, w3[15:8] ANT_TX_NUM, w3[23:16] ANT_RX_NUM.
  hal.tx_nss/rx_nss = min(reported, 2) (0 → 2). ANT num 1 → use RF_B only. Part 1 not requested on AX chips.
  Optional for a minimal driver if you hard-code 2T2R.

---------------------------------------------------------------------------------------------
## 12. RX filter, MAC address/BSSID, port configuration, TSF

### 12.1 R_AX_RX_FLTR_OPT 0xCE20 (reg.h:3325)
| Bits | Name | Default (0x030044BE) |
|---|---|---|
| 31:24 | UID_FILTER | 3 |
| 23:22 | UNSPT_FILTER | 0 |
| 21:16 | RX_MPDU_MAX_LEN (units of 512 B) | set to 22 by RMAC init (not part of `rx_fltr`) |
| 14 | A_FTM_REQ | 1 |
| 13 | A_ERR_PKT | 0 |
| 12 | A_UNSUP_PKT | 0 |
| 11 | A_CRC32_ERR | 0 |
| 10 | A_PWR_MGNT | 1 |
| 9:8 | A_BCN_CHK_RULE | 0 |
| 7 | A_BCN_CHK_EN | 1 |
| 6 | A_MC_LIST_CAM_MATCH | 0 |
| 5 | A_BC_CAM_MATCH | 1 |
| 4 | A_UC_CAM_MATCH | 1 |
| 3 | A_MC | 1 |
| 2 | A_BC | 1 |
| 1 | A_A1_MATCH | 1 |
| 0 | SNIFFER_MODE | 0 |

Semantics inferred from mac80211 `configure_filter` (mac80211.c:308-384): these bits are *match
requirements* — clearing A_A1_MATCH = accept other-BSS unicast (FIF_OTHER_BSS); clearing A_MC =
all multicast (FIF_ALLMULTI); setting A_CRC32_ERR = pass FCS-error frames (FIF_FCSFAIL); clearing
BCN_CHK_EN+A_BC+A_A1_MATCH = all beacons/probe-resp (FIF_BCN_PRBRESP_PROMISC, also forced during HW
scan, fw.c:9362-9366 → 0x03004438); clearing BC_CAM_MATCH+UC_CAM_MATCH = FIF_PROBE_REQ; SNIFFER_MODE
for monitor. Updates are RMW preserving [21:16] (`rtw89_mac_set_rx_fltr`, mac.c:2678).
Frame-type filters 0xCE24/28/2C: 2 bits per subtype, 01 = host, 10 = WLCPU, 00 = drop.
"RCR" = R_AX_RCR 0xCE00 [3:0] CH_EN = 1.

### 12.2 MAC address and BSSID — no plain registers
rtw89/AX has no station-address or BSSID MMIO register. Own address (SMA), peer/BSSID address (TMA)
and BSSID live in the **address CAM** and **BSSID CAM**, written by FW from H2C
cat 1 (MAC) / class 0x06 (ADDR_CAM_UPDATE) / func 0x00, payload `rtw89_h2c_addr_cam_v0` (15 dwords,
cam.c:814 / cam.h:15-116). Address CAM enabled by §9.2 (RANGE 0x7F). Entry: `len = ADDR_CAM_ENT_SIZE 0x40`,
BSSID entry len 0x08; chip has 128 addr-CAM and 10 BSSID-CAM entries.

| Dword | Fields |
|---|---|
| w0 | 0 |
| w1 | [7:0] addr_cam_idx, [15:8] offset (0), [23:16] len (0x40) |
| w2 | b0 valid, [2:1] net_type (0 no-link, 1 adhoc, 2 infra, 3 AP), [4:3] bcn_hit_cond (0), [6:5] hit_rule (0), b7 BB sel (phy 0), [13:8] addr_mask (0), [15:14] mask_sel (0), [23:16] SMA hash (XOR of 6 bytes), [31:24] TMA hash |
| w3 | [5:0] bssid_cam_idx |
| w4 | SMA[0..3] (byte0 in [7:0]) |
| w5 | [15:0] SMA[4..5], [31:16] TMA[0..1] |
| w6 | TMA[2..5] |
| w7 | 0 |
| w8 | [7:0] MACID, [10:8] port (interrupt), [13:11] TSF-sync port, b14 TF_TRS (=HE support when associated), b15 LSIG_TXOP, [26:24] tgt_ind, [29:27] frm_tgt_ind |
| w9 | [11:0] AID (infra), b12–14 WoL, b15 WAPI, [17:16] sec_ent_mode (2 = normal), [31:18] 7× 2-bit keyid |
| w10–w11 | [7:0] sec-entry valid bitmap, then 7 sec-CAM indices (8 bit each) |
| w12 | [7:0] bssid idx, [15:8] offset, [23:16] len (8) |
| w13 | b0 valid, b1 BB sel, [7:2] BSSID mask (0x3F = all 6 bytes), [13:8] BSS color, [23:16] BSSID[0], [31:24] BSSID[1] |
| w14 | BSSID[2..5] |

SMA = vif address (from efuse 0x400 unless overridden); TMA = BSSID (no STA) or peer address.
H2C framing/transport: FW track.

### 12.3 Port registers (port 0, `rtw89_port_base_ax`, mac.c:4458)
0xC400 PORT_CFG_P0 bits: b0 RXBCN_RPT_EN, b1 TXBCN_RPT_EN, b2 PORT_FUNC_EN, b3 TSF_UDT_EN, b4 RX_BSSID_FIT_EN,
b5 TSFTR_RST, [11:10] NET_TYPE, b12 BCNTX_EN, b13 TBTT_PROHIB_EN, b16 BRK_SETUP.
Other port-0 regs: 0xC404 TBTT_PROHIB ([7:0] setup, [27:16] hold), 0xC408 BCN_AREA ([27:16] mask area),
0xC40C BCNERLYINT ([11:0]), 0xC40E TBTTERLYINT (16-bit, [11:0]), 0xC412 TBTT_AGG (16-bit, [15:8]),
0xC414 BCN_SPACE ([15:0] TU), 0xC426 DTIM_CTRL (16-bit, [15:8] DTIM num), 0xC434 BCN_CNT_TMR,
0xC438/0xC43C TSFTR low/high, 0xC590 P0MB_HGQ_WINDOW_CFG_0 (8-bit), 0xC63C MBSSID_DROP_0,
0xC568 MBSSID_CTRL, 0xC6A0 PTCL_BSS_COLOR_0 ([5:0] port 0), 0xCA08 MD_TSFT_STMP_CTL, 0xCE84 BCN_PSR_RPT_P0 ([10:0]),
0xC2A0 PORT0_TSF_SYNC.

### 12.4 `rtw89_mac_port_update` for a station (mac.c:5103) — at add_interface (net_type=NO_LINK) and at
assoc (net_type=INFRA, `__rtw89_ops_bss_link_assoc`, mac80211.c:679)
1. func_sw (mac.c:4556): only if PORT_FUNC_EN already set: msleep(beacon_int+1); CLR32(0xC400, b2|b16);
   SET32(0xC400, b5 TSFTR_RST); W32(0xC434)=0. (8852A/AP extras skipped.)
2. CLR32(0xC400, b1); CLR32(0xC400, b0).
3. MASK32(0xC400, [11:10], net_type).
4. net_type≠NO_LINK ? SET32 : CLR32 (0xC400, b13|b16).
5. INFRA/ADHOC ? SET32 : CLR32 (0xC400, b4 RX_BSSID_FIT_EN).
6. INFRA/ADHOC ? SET32 : CLR32 (0xC400, b3 TSF_UDT_EN) — **this is the station TSF sync**: HW updates the
   port TSF from beacons matching the BSSID CAM. Cleared during HW scan and restored after (fw.c:9360, 9401).
7. AP/ADHOC ? SET32 : CLR32 (0xC400, b12 BCNTX_EN) → cleared for STA.
8. MASK32(0xC414, [15:0], beacon_int or 100).
9. W8(0xC590) = 0 (16 only for AP).
10. SET8(0xCA08, b1|b0) (UPD_HGQMD, UPD_TIMIE); MASK16(0xC426, 0xFF00, dtim_period).
11. RMW 0xC63C: clear bit (16+port) and, for port 0, bit 0.
12. MASK32(0xC404, [7:0], 2); MASK32(0xC404, [27:16], 200); MASK32(0xC408, [27:16], 0).
13. MASK16(0xC40E, [11:0], 5); MASK16(0xC412, [15:8], 1).
14. MASK32(0xC6A0, [5:0], HE BSS color).
15. port 0 & non-AP: CLR32(0xC568, 0x00FFFFFE).
16. SET32(0xC400, b2 PORT_FUNC_EN).
17. TSF re-sync of AP ports — no AP → nothing. sleep 20 µs.
18. MASK32(0xC40C, [11:0], 160); MASK32(0xCE84 + 4·port, [10:0], bssid_index (0 unless non-transmitted BSSID)).

Beacon-related fields (steps 8–13, 18) are consumed by FW beacon tracking/LPS; for a minimal STA
without power-save they are harmless defaults. Skippable AP-only helpers: `rtw89_mac_bcn_drop`,
`rtw89_mac_port_tsf_sync(_rand)`, `tsf_resync_all`, HIQ window 16, BCNTX_EN.
Read TSF: `(R32(0xC43C) << 32) | R32(0xC438)` (mac.c:5138).

### 12.5 `rtw89_mac_vif_init` (mac.c:5044), per interface
1. port_update (above).
2. DMAC table clear for macid (skipped if secure_boot) (mac.c:4402): for i=0..3:
   `W32(0x0C04 FILTER_MODEL_ADDR) = 0x18800000 + macid*16 + i*4; W32(0x40000 INDIR_ACCESS_ENTRY) = 0`.
3. CMAC control table default (mac.c:4417): `W32(0x0C04) = 0x18840000 + macid*32`; then
   W32 0x40000=0x00000004, 0x40004=0x400A0004, 0x40008=0, 0x4000C=0, 0x40010=0, 0x40014=0x0E43000B,
   0x40018=0, 0x4001C=0x000B8109.
4. H2C: MACID un-pause, role maintain, join info, addr/BSSID CAM (§12.2), default CMAC table (FW track).

### 12.6 Other assoc-time MAC write
`rtw89_mac_set_he_obss_narrow_bw_ru` (mac.c:5176): only for HE STA on a radar channel —
SET/CLR32(0xCCB0, BIT21 RXTRIG_RU26_DIS). Optional.

---------------------------------------------------------------------------------------------
## 13. BT coexistence — what rtw89 does, and the minimum for Wi-Fi to own the antennas

### 13.1 Antenna topology for RFE 1 (`rtw8852b_btc_set_rfe`, rtw8852b.c:751)
`ant.num = (rfe_type % 2) ? 2 : 3` → RFE 1 → **2 antennas, BTC_ANT_SHARED, bt_pos = BTC_BT_BTG**: BT shares
the 2.4 GHz front end of WL path S1 (RF path B) through the BTG port. isolation 10, no diversity,
switch_type internal. (Odd RFE = shared, even RFE = dedicated 3rd antenna.)

### 13.2 Control points (hardware)
- **Control-path owner** (`rtw89_mac_cfg_ctrl_path`, mac.c:6631): byte 0x0073 (R_AX_SYS_SDIO_CTRL+3) bit2
  = 0x0070 BIT26 (LTE_MUX_CTRL_PATH): 1 = WL controls the RF switch/PTA, 0 = BT.
- **GNT override** (`rtw89_mac_cfg_gnt`, mac.c:6489): one 32-bit value written to the *LTE/coex indirect*
  register **R_AX_LTE_SW_CFG_1 = 0x38** (in LTE space). Bits for S0 (band[0]) / S1 (band[1]):

  | | S0 | S1 |
  |---|---|---|
  | GNT_BT RFC SW value / SW ctrl | b15 / b14 | b31 / b30 |
  | GNT_BT BB SW value / SW ctrl | b11 / b10 | b27 / b26 |
  | GNT_WL RFC SW value / SW ctrl | b13 / b12 | b29 / b28 |
  | GNT_WL BB SW value / SW ctrl | b9 / b8 | b25 / b24 |

  `gnt_x_sw_en` sets both CTRL bits (RFC+BB), `gnt_x` sets both VAL bits. HW mode = ctrl 0, val 0.
- **LTE indirect access** (mac.c:86-116): wait `R8(0xDAF3) & BIT5` (ready; 50 µs step, 50 ms timeout);
  write: `W32(0xDAF4 LTE_WDATA) = val; W32(0xDAF0 LTE_CTRL) = 0xC00F0000 | offset`;
  read: `W32(0xDAF0) = 0x800F0000 | offset; val = R32(0xDAF8 LTE_RDATA)`.
- **PTA/PLT "polluted" config** (`rtw89_mac_cfg_plt_ax`, mac.c:6577): `W16(0xC67C BT_PLT)` = PLT_EN b8 |
  TX: b0 GNT_WL, b1 GNT_BT_TX, b2 GNT_BT_RX, b3 LTE_RX | RX: b4 GNT_WL, b5 BT_TX, b6 BT_RX, b7 LTE_RX.
  BTC_PLT_BT → 0x0166; BTC_PLT_NONE → 0x0100; BTC_PLT_GNT_WL → 0x0111.
- **Scoreboard** R_AX_SCOREBOARD 0x00AC (shared WL↔BT mailbox bits; `rtw89_mac_cfg_sb`, mac.c:6602):
  write = BIT31 toggle | [30:24] FW bits (read-back & ~BIT0, | 0x01 when powered) | [23:0] driver bits; then sleep 1 ms.
  WL driver bits: b0 ACTIVE, b1 ON, b2 SCAN, b7 WLBUSY, b8 EXTFEM, b9 TDMA, b13 BT_HILNA, b14 BTLOG.
  BT side (read): b0 ACT (mailbox avail), b1 ON (BT enabled), b2 WHQL, b3 BT on S1, b5 RFK_RUN, b8 LNAB0...

### 13.3 What rtw89 does at init (`rtw89_btc_ntfy_init(NORMAL)`, coex.c:7746)
1. SW reset of coex state, `igno_bt = true`, `wl_only = 0`.
2. `btc_set_rfe` (13.1, SW only).
3. `__rtw8852bx_btc_init_cfg` (rtw8852b_common.c:1797) — **register writes**:
   1. `rtw89_mac_coex_init(RTK mode, INNER)` (mac.c:6365):
      - `SET8(0x0040 GPIO_MUXCFG, BIT5)` ENBT.
      - `SET8(0xDA20 BTC_FUNC_EN, BIT1)` PTA_WL_TX_EN.
      - `SET8(0xDA35, 0x01)` (= 0xDA34 BT_COEX_CFG_2 BIT8, GNT_BT_POLARITY).
      - `SET8(0xDA40 CSR_MODE, BIT2|BIT3)` (STATIS_BT_EN, WL_ACT_MSK).
      - `SET8(0xDA42, 0x01)` (= 0xDA40 BIT16, BT_CNT_RST).
      - `CLR8(0xCC07, 0x02)` (= 0xCC04 BIT25, RSP_CHK_BTCCA).
      - RMW16 0xC340 CCA_CFG_0: set BIT5 BTCCA_EN, clear BIT9 BTCCA_BRK_TXOP_EN.
      - LTE 0x3C (SW_CFG_2): `v = LTE_R(0x3C) & 0x100; LTE_W(0x3C, v)` (keep only WL_RX_CTRL b8).
      - RTK mode: RMW8 0x0040: [7:6] BTMODE = 0; `SET8(0xDA4C TDMA_MODE, BIT0)` RTK_BT_ENABLE;
        RMW8 0xDA6C BT_COEX_CFG_5: [5:0] = 5 (BT report sample rate).
      - INNER direction: RMW8 0x0041: clear BIT2, set BIT1.
   2. WL TX-response and beacon high priority: `SET32(0xDA30 BTC_BT_COEX_MSK_TABLE, BIT3)`; `SET32(0xDA10 WL_PRI_MSK, BIT8)`.
   3. RF GNT debug off: `RF_W(A, 0x02 RR_WLSEL, 0xFFFFF, 0)`, `RF_W(B, 0x02, 0xFFFFF, 0)`.
   4. RF TRX mask LUT (shared antenna): for (path, group, val) in (A,0,0x5FF), (B,0,0x5FF), (A,2,0x5FF), (B,2,0x55F):
      `RF_W(p, 0xEF RR_LUTWE, 0xFFFFF, 0x20000); RF_W(p, 0x33 RR_LUTWA, 0xFFFFF, group);
      RF_W(p, 0x3F RR_LUTWD0, 0xFFFFF, val); RF_W(p, 0xEF, 0xFFFFF, 0)`. (Group 0 = BT SS, 2 = BT TX.)
   5. `W32(0xDA2C BTC_BREAK_TABLE) = 0xF0FFFFFF`.
   6. `SET32(0xDA40, BIT16|BIT2)` (BT counter reset + enable).
   7. `wl.init_ok = true`.
4. Scoreboard: driver bits |= ACTIVE|ON|BTLOG (0x4003); read BT scoreboard (BT ON bit → `bt.enable.now`).
5. Warn if control path currently BT (`rtw89_mac_get_ctrl_path`: 0x0073 bit2).
6. WL TX-power coex ctrl default (`rtw8852b_btc_set_wl_txpwr_ctrl` with 0xFFFFFFFF): RMW 0xD200 PWR_RATE_CTRL
   [8:0]=0 and clear b9 (FORCE_PWR_BY_RATE_EN); RMW 0xD220 PWR_COEXT_CTRL [11:3]=0, clear b1 (TXAGC_BT_EN).
7. H2C to FW coex engine: monitor-register list (16 regs incl. 0xDA24..0xDA4C, 0xCEF4, 0x8424, 0xD200,
   0xD220, BB 0x980/0x4738/0x4688/0x4694, rtw8852b.c:323), slot table, DRV_INFO init/ctrl (FW track / coex).
8. `_run_coex(NTFY_INIT)` → `_action_wl_init` → `_set_ant(WINIT)` (coex.c:4531):
   - BT enabled (scoreboard b1): GNT WL = SW_LO, BT = SW_HI → LTE 0x38 = **0xDD00DD00**.
   - BT disabled: GNT WL = SW_HI, BT = SW_LO → LTE 0x38 = **0x77007700**.
   - control path → WL (0x0073 |= BIT2); PLT band0 = BTC_PLT_BT (`W16(0xC67C) = 0x0166`).
   - policy BTC_CXP_OFF_BT (TDMA off; slot/coex tables sent to FW via H2C), then `_action_common`
     (more H2C + scoreboard write).
9. Later `radio_state(WL_ON)` (end of core_start) re-runs `btc_init_cfg` (step 3) and `_run_coex`:
   BT off → `_action_bt_off` = WONLY; BT on and idle/no link → W2G (GNT both HW, PLT BT) with TDMA
   policies — this is the dynamic coex algorithm (out of scope).
10. During RF calibration rtw89 sets WRFK: GNT WL SW_HI / BT SW_LO (0x77007700 for both paths or the
    calibrated phy), ctrl path WL, PLT NONE (coex.c:4589).
11. Power-off (`rtw89_btc_ntfy_poweroff`, coex.c:7700): scoreboard driver bits cleared (all 24),
    `_set_ant(WOFF)`: control path → BT (0x0073 &= ~BIT2), PLT NONE (0x0100).

### 13.4 Minimum for Wi-Fi to own the antennas without implementing coex
Equivalent to rtw89's "WL only" action (`_action_wl_only`, coex.c:4716 → `_set_ant(WONLY)`), done once
after `trx_init` and BB/RF table load:
1. PTA init (13.3 step 3.1) — mandatory so the PTA/GNT logic is in a known RTK-mode state.
2. `LTE_W(0x38, 0x77007700)` — GNT_WL forced 1, GNT_BT forced 0 on S0 and S1 (RFC and BB).
3. `SET8(0x0073, BIT2)` — WL owns the control path.
4. `W16(0xC67C, 0x0100)` — PLT enable, no pollution flags (BTC_PLT_NONE).
5. Recommended: 13.3 step 3.3 (RR_WLSEL = 0 on both paths) and step 3.2 (WL response/beacon priority).
6. Scoreboard: write 0x00AC per §13.2 with driver bits ACTIVE|ON|BTLOG (e.g. `0x81004003` when the FW
   bits read back as 0: BIT31 toggle | 0x01<<24 | 0x4003), then sleep 1 ms, so BT FW knows WL is on;
   on shutdown clear driver bits and hand the control path back to BT (step 11) — otherwise the BT
   radio (USB 0489:e123 on the reference system) loses the shared S1 2.4 GHz path.

Trade-off: with GNT_BT forced 0 the BT controller cannot use the shared antenna while WL is up
(BT audio/HID will break on 2.4 GHz). The least-intrusive static alternatives rtw89 itself uses are
W5G when WL is on 5 GHz (GNT_WL SW_HI, GNT_BT HW → LTE 0x38 = **0x33003300** [S0 0x3300 + S1 0x33000000],
ctrl path WL, PLT NONE 0x0100) and W2G = HW arbitration (LTE 0x38 = 0x00000000, PLT BT 0x0166), which
relies on the FW TDMA/coex tables to share airtime.

---------------------------------------------------------------------------------------------
## 14. Periodic / watchdog work
- `track_work` every 2 s (core.c:5424): traffic stats, `rtw89_leave_lps`, BF monitor, beacon tracking
  (FW feature, LPS only), PHY stat/env monitor, **DIG**, **RFK tracking (DPK)**, RA update, **CFO tracking
  (adjusts XTAL cap via XSI 0x04/0x05)**, EDCCA, SAR, rfkill poll (GPIO9). None of these is a MAC-register
  watchdog; DIG/CFO/DPK/thermal belong to the PHY track and matter for link quality, not for the MAC staying alive.
- `track_ps_work` every 100 ms — LPS only (disabled on the reference system).
- FW watchdog is internal to the WCPU (rtw89 only resets APB wrapper before FWDL on 885xB). SER (error
  recovery) is interrupt/C2H-driven (out of scope).
- **No periodic MAC register access is mandatory for stability.**

---------------------------------------------------------------------------------------------
## 15. Minimal-driver checklist (station, no PS, no coex engine)

Mandatory, in order:
1. (probe) chip version, power on (§3), DMAC pre-init DLFW + HFC h2c (§6), PCIe pre-init, FW download (§5),
   efuse dump + logical decode (§11.1, 11.2, 11.4) for MAC address, RFE, xtal cap, country, TSSI/gain;
   phycap physical 0x580–0x5FF (§11.5); power off.
2. (up) power on, partial init + FW download again, enable BB/RF (§7), sys_init (§8), trx_init (§9.1, 9.2, 9.4),
   PCIe post-init, BB/RF reset+tables (PHY track), PTA init + WL-only antenna (§13.4), XTAL cap, PPDU status (§10).
3. (vif) addr/BSSID CAM via H2C, port 0 config (§12.4), CMAC/DMAC per-MACID table defaults (§12.5).

Skippable/optional: EESK pinmux (§3 step 41), rfkill GPIO, DAV efuse (§11.3), FW-secure bytes only if the
FW file choice does not need it (FW track), error IMRs (§9.3), band-1/DBCC paths, preload (not used on 885xB),
AP beacon/TSF-sync helpers, coex H2C (monreg/slots/drv_info/TDMA), `set_he_obss_narrow_bw_ru`, phycap H2C (§11.7).

---------------------------------------------------------------------------------------------
## 16. Open questions / uncertainties
1. Exact semantics of individual RX_FLTR_OPT "A_*" bits are inferred from mac80211 flag handling, not from a datasheet.
2. Meaning of the CMAC control-table default dwords (0x400A0004, 0x0E43000B, 0x000B8109) is not documented in rtw89;
   they are replicated verbatim from `rtw89_mac_cmac_tbl_init`.
3. Whether the FW coex engine needs at least one DRV_INFO/TDMA-off H2C when the driver forces GNT by SW is
   unverified; rtw89 always sends them. Test with BT active.
4. Steps 36–38 of power-on depend on efuse autoload + phycap 0x5E9; on the reference unit the result is unknown until
   the efuse is dumped (if 0x5E9 == 0xAA they are skipped).
5. `rtw89_dump_logical_efuse_map` is run on the 96-byte DAV map with the 1216-byte bounds (potential overread
   if no 0xFF terminator) — irrelevant for 8852B parsing but do not copy that bound.
6. The W5G GNT value in §13.4 (0x33003300) is derived from `_set_gnt(SW_HI, HW)` bit mapping, not observed.
