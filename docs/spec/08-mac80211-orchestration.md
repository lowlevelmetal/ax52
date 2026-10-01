# 08 — Orchestration, mac80211 glue, capability advertisement (RTL8852BE)

Track 08 of the RTL8852BE specification. Scope: how rtw89 is assembled for 8852B-on-PCIe,
the order in which hardware/firmware operations happen (probe, `.start`, `.stop`,
interface/station lifecycle), what every `ieee80211_ops` callback does, what is
advertised to mac80211/cfg80211, regulatory hooks, deferred work and locking, and a
recommended minimal architecture for a new station-only driver.

Internals (register sequences, H2C layouts, PHY tables, RFK, descriptors) belong to
tracks 01–07; this document names the entry point and its position in the sequence
and gives numeric values wherever the value is part of the glue.

Conventions
- All source paths are relative to
  `reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/` unless prefixed
  `include/net/`.
- MAC registers are MMIO offsets in BAR memory. BB (PHY) registers are accessed at
  `0x10000 + addr` (`rtw89_phy_gen_ax.cr_base = 0x10000`, phy.c:8970).
- Per-port MAC registers are at `base + port * 0x40` (mac.h:1207); band-1 (MAC_1)
  copies are at `+0x2000` (mac.h:590). 8852B only uses MAC_0/PHY_0 and port 0 for a
  single station.
- "FW 0.29.29.18" is the firmware of the reference system (see README.md).

---------------------------------------------------------------------------------

## 0. Summary

1. **Probe does real hardware work.** `rtw89_chip_info_setup()` powers the chip on,
   **downloads firmware**, reads efuse, asks the firmware for its PHY capabilities
   over the H2C/C2H register mailbox, then powers the chip off again
   (core.c:7279-7334). `ieee80211_register_hw()` happens after that, while the chip is
   off.
2. **`.start` powers the chip on again and downloads firmware again.** The full MAC
   init, BB/RF table load, RF calibration init (DACK/RCK/RX-DCK), and coex init all
   happen in `rtw89_core_start()` (core.c:6565-6632). Interrupts are enabled last.
   **No channel is set in `.start`.** The channel is first programmed when mac80211
   assigns a channel context, or when it calls `.config(CHANGE_CHANNEL)`.
3. **With FW 0.29.29.18, rtw89 runs in real channel-context mode and uses
   firmware hardware scan.** `no_chanctx` is false because 8852B has
   `support_chanctx_num = 2` and the firmware has both SCAN_OFFLOAD (≥0.29.29.0) and
   BEACON_FILTER (≥0.29.29.7) (core.c:7557-7570, fw.c:874-875). So on the reference system
   `sw_scan_start`/`sw_scan_complete` are never called. The legacy `.config`
   channel path together with mac80211's `ieee80211_emulate_*chanctx` helpers is the
   path rtw89 uses for older firmware. It is fully supported and is the recommended
   path for a new driver.
4. **IPS: rtw89 powers the chip off whenever mac80211 reports IDLE.** The
   `disable_ps_mode` parameter does not change this. `rtw89_enter_ips()` calls
   `rtw89_core_stop()`, and `rtw89_leave_ips()` calls `rtw89_core_start()`, then
   `set_channel()`, then per-vif MAC init again (ps.c:233-278). A new driver should
   not do this: keep the chip powered from `.start` to `.stop`.
5. **Firmware does many things that look like MAC-register work:**
   - EDCA (`conf_tx`) is an H2C: class `FW_OFLD`, func `USR_EDCA`
     (mac80211.c:414-440, fw.c:5196).
   - Address/BSSID CAM entries and the role/join state are H2Cs.
   - Rate adaptation is firmware-side (HAS_RATE_CONTROL).
   - BA CAM is an H2C (optional, because hardware creates dynamic BA CAM entries).
6. **Coexistence with the on-chip Bluetooth is not covered by any other track.** The
   minimum needed to own the antenna is in §9.

---------------------------------------------------------------------------------

## 1. Static configuration for 8852BE

### 1.1 Module / PCI ID glue (rtw8852be.c)

| Item | Value | Source |
|---|---|---|
| PCI IDs | 10ec:b852 and 10ec:b85b → `rtw89_8852be_info` | rtw8852be.c:82-92 |
| driver_info | `.chip = &rtw8852b_chip_info`, `.variant = NULL`, `.quirks = NULL`, `.dev_id_quirks = 0`, `.bus.pci = &rtw8852b_pci_info` | rtw8852be.c:72-80 |
| pci_driver | `.probe = rtw89_pci_probe`, `.remove = rtw89_pci_remove`, `.shutdown = rtw89_pci_shutdown` (only sets FLAG_SHUTDOWN), `.driver.pm = &rtw89_pm_ops`, `.err_handler = &rtw89_pci_err_handler` | rtw8852be.c:95-104, pci.c:4877 |

`rtw8852b_pci_info` (rtw8852be.c:12-70) is fully owned by track 01. Values that
matter for orchestration:
- `gen_def = &rtw89_pci_gen_ax`, `isr_def = &rtw89_pci_isr_ax`.
- `tx_dma_ch_mask`: ACH4-7, CH10 and CH11 are **unused**. The used TX channels are
  ACH0-3, CH8 (B0MG), CH9 (B0HI) and CH12 (FW command).
- `rpwm_addr = R_AX_PCIE_HRPWM = 0x10C0`, `cpwm_addr = R_AX_CPWM = 0x8170`.
- `mit_addr = R_AX_INT_MIT_RX = 0x10D4`.
- `init_cfg_reg = R_AX_PCIE_INIT_CFG1 = 0x1000`.
- `dma_stop1 = {0x1010, B_AX_TX_STOP1_MASK_V1}`, `dma_busy1 = {0x101C, …}`.
- `rxbd_rwptr_clr_reg = 0x1018`, `exp_ctrl_reg = 0x13F0`.

### 1.2 `rtw8852b_chip_info` — every field (rtw8852b.c:957-1116)

Registers are resolved to numbers. "Consumer" says where the field is used, when that
is non-obvious.

| Field | Value | Meaning / consumer |
|---|---|---|
| chip_id | RTL8852B | |
| chip_gen | RTW89_CHIP_AX | selects AX paths everywhere |
| ops | `rtw8852b_chip_ops` (§1.3) | |
| mac_def | `rtw89_mac_gen_ax` (mac.c:7430) | §1.4 |
| phy_def | `rtw89_phy_gen_ax` (phy.c:8969), `cr_base = 0x10000` | |
| fw_def.fw_basename | `"rtw89/rtw8852b_fw"` | files tried: `rtw8852b_fw-2.bin`, then `-1.bin`, then `rtw8852b_fw.bin` (fw.c:989-999, fw.h:4582) |
| fw_def.fw_format_max | 2 | |
| fw_def.fw_b_aid | 0 | no "B" firmware variant |
| try_ce_fw | true | prefer the RTW89_FW_NORMAL_CE image in the multi-FW file (fw.c:1052) |
| bbmcu_nr | 0 | no BB MCU firmware; `include_bb = false` in `mac_init` |
| needed_fw_elms | 0 | |
| fw_blacklist | `&rtw89_fw_blacklist_default` | |
| fifo_size | 196608 (192 KiB) | DLE sizing (mac.c:2061) |
| small_fifo_size | true | only lowers log level when TX resources are exhausted (pci.c:1304) |
| dle_scc_rsvd_size | 98304 | SCC DLE expected size = 196608 − 98304 = 98304 (mac.c:2064) |
| max_amsdu_limit | 5000 | upper bound for dynamic `max_rc_amsdu_len` at high rates (phy.c:50) |
| max_vht_mpdu_cap | IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 (=2) | |
| max_eht_mpdu_cap | 0 | |
| max_tx_agg_num | 128 | → `hw->max_tx_aggregation_subframes` |
| max_rx_agg_num | 64 | → `hw->max_rx_aggregation_subframes` |
| dis_2g_40m_ul_ofdma | true | HTC OM template to disable UL-OFDMA in 2.4G/40M (core.c:929) |
| rsvd_ple_ofst | 0x2F800 | SER dump only |
| qta_def.hfc_param_ini | [PCIE] = `rtw8852b_hfc_param_ini_pcie` | table below |
| qta_def.dle_mem | [PCIE] = `rtw8852b_dle_mem_pcie` | table below |
| wde_qempty_acq_grpnum | 4 | TX-queue-empty checks (mac.c:1993) |
| wde_qempty_mgq_grpsel | 4 | (mac.c:2012) |
| rf_base_addr | {0xE000, 0xF000} | direct RF register window for path A/B (phy.c:932) |
| thermal_th | {0x32, 0x35} | used only with the THERMAL_PROT quirks (not set) |
| pwr_on_seq / pwr_off_seq | NULL | the function versions are used instead (`pwr_on_func`/`pwr_off_func`) |
| bb_table / bb_gain_table | `rtw89_8852b_phy_bb_table` / `_bb_gain_table` | built-in fallback; FW-file elements take priority (phy.c:1891) |
| rf_table | {radioa, radiob} | built-in fallback |
| nctl_table | `rtw89_8852b_phy_nctl_table` | |
| nctl_post_table | NULL | |
| dflt_parms | `rtw89_8852b_dflt_parms` | TX-power tables; FW elements override (core.c:7090-7115) |
| rfe_parms_conf | NULL | all RFE types use dflt_parms |
| chanctx_listener | callback[RFK] = `rtw8852b_rfk_chanctx_cb` | pause/resume RFK tracking around MCC |
| txpwr_factor_bb / rf / mac | 3 / 2 / 1 | fractional bits: BB 1/8 dB, RF 1/4 dB, MAC 1/2 dB |
| dig_table | NULL | |
| dig_regs | `rtw8852b_dig_regs` (rtw8852b.c:217) | track 05 |
| tssi_dbw_table | NULL | |
| support_macid_num | RTW89_MAX_MAC_ID_NUM = 128 | |
| support_link_num | 0 | no MLO |
| support_chanctx_num | 2 | enables chanctx mode (§4.2) |
| support_rnr | false | → WIPHY_FLAG_SPLIT_SCAN_6GHZ |
| support_bands | 2 GHz \| 5 GHz | no 6 GHz |
| support_bandwidths | 20 \| 40 \| 80 | no 160 |
| support_unii4 | true | 5 GHz channels 169/173/177 are present (policy-gated, §6) |
| support_ant_gain | true | ant-gain TX-power offset (ACPI/regd gated) |
| support_tas | false | |
| support_sar_by_ant | true | |
| support_noise | false | `.get_survey` is removed from ops (core.c:7572) |
| support_fw_cmd_ofld | false | normal MMIO IO ops |
| ul_tb_waveform_ctrl | true | HE UL-TB only |
| ul_tb_pwr_diff | false | |
| rx_freq_from_ie | true | during scan, RX freq is taken from the DS-param IE (core.c:3785) |
| hw_sec_hdr | false | driver/mac80211 builds IV (`IEEE80211_KEY_FLAG_GENERATE_IV`) |
| hw_mgmt_tx_encrypt | false | `IEEE80211_KEY_FLAG_SW_MGMT_TX` for CCMP/GCMP |
| hw_tkip_crypto | false | TKIP is software |
| hw_mlo_bmc_crypto | false | |
| rf_path_num | 2 | |
| tx_nss / rx_nss | 2 / 2 | capped by FW phycap (mac.c:3247) |
| acam_num | 128 | address CAM entries |
| bcam_num | 10 | BSSID CAM entries |
| scam_num | 128 | security CAM entries |
| bacam_num | 2 | static BA CAM entries |
| bacam_dynamic_num | 4 | used only for BACAM_V0_EXT (not 8852B) |
| bacam_ver | RTW89_BACAM_V0 | `init_ba_cam` H2C is a no-op (fw.h:5585) |
| addrcam_ver | 0 | addr CAM H2C v0 layout |
| ppdu_max_usr | 4 | MU PPDU status parsing |
| sec_ctrl_efuse_size | 4 | bytes skipped at start of the physical efuse |
| physical_efuse_size | 1216 | |
| logical_efuse_size | 2048 | |
| limit_efuse_size | 1280 | **not consumed** anywhere in rtw89 core |
| dav_phy_efuse_size / dav_log_efuse_size | 96 / 16 | the "DAV" efuse bank, read via XTAL-SI (efuse.c:145) |
| efuse_blocks | NULL | |
| phycap_addr / phycap_size | 0x580 / 128 | physical-efuse window for the phycap map (efuse.c:350) |
| para_ver | 0 | coex |
| wlcx_desired | 0x05050000 | coex version gate |
| scbd | 0x1 | coex scoreboard used |
| mailbox | 0x1 | coex |
| afh_guard_ch | 6 | coex |
| wl_rssi_thres / bt_rssi_thres | {70,60,50,40} / {50,40,30,20} | coex |
| rssi_tol | 2 | coex |
| mon_reg / rf_para_ulink / rf_para_dlink | 8852B tables (rtw8852b.c:310-351) | coex |
| *_v9 fields | NULL/0 | |
| ps_mode_supported | RFOFF \| CLK_GATED \| PWR_GATED | but FW has NO_LPS_PG, so PWR_GATED is never chosen (core.c:4416) |
| low_power_hci_modes | 0 | |
| h2c_cctl_func_id | H2C_FUNC_MAC_CCTLINFO_UD = 0x2 | cmac table H2C function |
| hci_func_en_addr | R_AX_HCI_FUNC_EN = 0x8380 | |
| h2c_desc_size / txwd_body_size / txwd_info_size | 24 / 24 / 24 bytes | `struct rtw89_txwd_body` / `_info` (core.h:1092,1123) |
| h2c_ctrl_reg | R_AX_H2CREG_CTRL = 0x8160 | register mailbox (track 04) |
| h2c_regs | {0x8140, 0x8144, 0x8148, 0x814C} | |
| h2c_counter_reg | {0x01F5 (R_AX_UDM1+1), mask 0x0F} | |
| c2h_ctrl_reg | R_AX_C2HREG_CTRL = 0x8164 | |
| c2h_regs | {0x8150, 0x8154, 0x8158, 0x815C} | |
| c2h_counter_reg | {0x01F5, mask 0xF0} | |
| page_regs | HCI_FC_CTRL 0x8A00, CH_PAGE_CTRL 0x8A04, ACH0_PAGE_CTRL 0x8A10, ACH0_PAGE_INFO 0x8A50, PUB_PAGE_INFO3 0x8A8C, PUB_PAGE_CTRL1 0x8A90, PUB_PAGE_CTRL2 0x8A94, PUB_PAGE_INFO1 0x8A98, PUB_PAGE_INFO2 0x8A9C, WP_PAGE_CTRL1 0x8AA0, WP_PAGE_CTRL2 0x8AA4, WP_PAGE_INFO1 0x8AA8 | HFC (track 02) |
| wow_reason_reg | {0x815F, 0x815F} | WoWLAN only |
| cfo_src_fd / cfo_hw_comp | true / true | CFO tracking |
| dcfo_comp / dcfo_comp_sft | {R_DCFO_COMP_S0 = 0x448C (BB), mask 0xFFF} / 10 | |
| nhm_report / nhm_th | NULL | |
| imr_info | `rtw8852b_imr_info` (below) | error IMR enables in trx_init (track 02) |
| imr_dmac_table / imr_cmac_table | NULL | |
| rrsr_cfgs | ref_rate {0xCC08, mask 0x200, 0}; rsc {0xCC08, mask 0xC00, 2} | response rate config |
| bss_clr_vld | {R_BSS_CLR_MAP_V1 = 0x43B0 (BB), BIT(28)} | BSS color (§4.4) |
| bss_clr_map_reg | 0x43B0 (BB) | |
| rfkill_init | pinmux {0x02D4, mask 0x00F0, data 0xF}; mode {0x0062, mask 0x0202, data 0} (16-bit RMW) | GPIO9 as input |
| rfkill_get | {R_AX_GPIO_EXT_CTRL = 0x0060, BIT(1)} (8-bit read) | blocked when the bit reads 0 (core.c:7230) |
| btc_sb | {R_AX_SCOREBOARD = 0x00AC} | coex scoreboard |
| dma_ch_mask | BIT(4..7) \| BIT(10) \| BIT(11) = 0x0CF0 | disabled DMA channels (ACH4-7, B1MG, B1HI) |
| edcca_regs | level reg 0x4884 (BB): mask0 0xFF, mask_p 0xFF00, ppdu mask 0xFF000000 | §4.7 |
| pmac_regs | counters (debug) | |
| wowlan_stub | MAGIC_PKT\|DISCONNECT, patterns | skip |
| xtal_info | NULL | |
| default_quirks | 0 | |

**DLE memory, PCIe** (rtw8852b.c:94-122, mac.c:1716-1900)

| Mode | WDE size | PLE size | WDE quota (min = max) | PLE quota min / max |
|---|---|---|---|---|
| SCC (normal) | wde_size7 = {page 64 B, 510 linked, 2 unlinked} | ple_size6 = {page 128 B, 496 linked, 16 unlinked} | wde_qt7 = {hif 446, wcpu 48, pkt_in 0, cpu_io 16} | ple_qt18 = {cma0_tx 147, cma1_tx 0, c2h 16, h2c 20, wcpu 17, mpdu_proc 13, cma0_dma 89, cma1_dma 0, bb_rpt 32, wd_rel 14, cpu_io 8, tx_rpt 0} / ple_qt58 = {147, 0, 16, 20, 157, 13, 229, 0, 172, 14, 24, 0} |
| DLFW (FW download) | wde_size9 = {64 B, 0, 1024} | ple_size8 = {128 B, 64, 960} | wde_qt4 = all 0 | ple_qt13 = {0, 0, 16, 48, 0…0} (min = max) |
| WOW | as SCC | as SCC | wde_qt7 | ple_qt18 / ple_qt_52b_wow |

Check: in SCC mode, 512×64 + 512×128 = 98304, which is the expected size. In DLFW
mode the whole 196608 bytes are used.

**HFC, PCIe** (rtw8852b.c:21-49): `{min, max, group}` per channel:

| Channel | min | max | group |
|---|---|---|---|
| ACH0 | 5 | 341 | 0 |
| ACH1 | 5 | 341 | 0 |
| ACH2 | 4 | 342 | 0 |
| ACH3 | 4 | 342 | 0 |
| ACH4-7 | 0 | 0 | — |
| B0MGQ | 4 | 342 | 0 |
| B0HIQ | 4 | 342 | 0 |
| B1MGQ, B1HIQ | 0 | 0 | — |
| FWCMDQ | 40 | 0 | — |

- Public pages: grp0 446, grp1 0, pub_max 446, wp_thrd 0.
- Pre-cost: `hfc_preccfg_pcie = {ch011_prec 2, h2c_prec 40, wp_ch07 0, wp_ch811 0,
  full_cond 1/0/0/0}`, mode RTW89_HCIFC_POH.
- DLFW mode: no channel config, same pre-cost, POH.

**imr_info** (rtw8852b.c:159-199), all resolved (track 02 uses these):

| Field | Value |
|---|---|
| wdrls_imr_set | 0x3327 |
| wsec_imr | reg 0x9D1C, set 0x8 |
| sta_sch_imr_set | 0x7 |
| txpktctl B0 | reg 0x9F1C, clr 0x30F, set 0x101 |
| txpktctl B1 | reg 0x9F2C, clr 0x30F, set 0x303 |
| wde | clr 0x070FF0FF, set 0x070FF0FF |
| ple | clr 0x070FF0FF, set 0x070FF0DF |
| host_disp | clr 0xFF0FFFFF, set 0x0C000161 |
| cpu_disp | clr 0xFF07FFFF, set 0x04000062 |
| other_disp | clr 0x3F031F1F, set 0 |
| bbrpt regs | com 0x960C, chinfo 0x962C, dfs 0x963C, set 0 |
| ptcl | clr 0xFFFFFFFF, set 0x10800001 |
| cdma0 | reg 0xC800 (R_AX_DLE_CTRL), clr 0x0080C000, set 0x0000C000 |
| phy_intf | reg 0xCCFC, 0/0 |
| rmac | reg 0xCEF4, clr 0xFF000, set 0xE4000 |
| tmac | reg 0xCCEC, clr 0x780, set 0x780 |

### 1.3 `rtw8852b_chip_ops` (rtw8852b.c:872-944) — resolved

| Op | Implementation | Owning track |
|---|---|---|
| enable_bb_rf / disable_bb_rf | rtw8852bx_mac_enable_bb_rf / _disable_bb_rf | 02/05 |
| bb_preinit / bb_postinit | NULL | |
| bb_reset | rtw8852b_bb_reset (rtw8852b.c:614) | 05 |
| bb_sethw | rtw8852bx_bb_sethw | 05 |
| read_rf / write_rf | rtw89_phy_read_rf_v1 / write_rf_v1 | 05 |
| set_channel | rtw8852b_set_channel = mac → bb → rf (rtw8852b.c:628) | 05 |
| set_channel_help | rtw8852b_set_channel_help (rtw8852b.c:675): enter = stop sch TX, PPDU status off, TSSI off, ADC FIFO reset, BB reset; exit = reverse | 05 |
| read_efuse / read_phycap | rtw8852bx_read_efuse / _read_phycap (rtw8852b_common.c:241/434) | 02 |
| fem_setup, data_setup, rfe_gpio, rfk_hw_init, rfk_init_late | NULL | |
| rfk_init | rtw8852b_rfk_init: DPK init, RCK, DACK, RX-DCK (rtw8852b.c:697) | 06 |
| rfk_channel | rtw8852b_rfk_channel: RX-DCK, IQK, TSSI, DPK + coex notifications (rtw8852b.c:711) | 06 |
| rfk_band_changed | TSSI scan (rtw8852b.c:731) | 06 |
| rfk_scan | rtw8852b_wifi_scan_notify → TSSI default TXAGC on/off (rtw8852b_rfk.c:3933) | 06 |
| rfk_track | DPK track | 06 |
| power_trim, set_txpwr, set_txpwr_ctrl, init_txpwr_unit, get_thermal | rtw8852bx_* | 05 |
| chan_to_rf18_val | NULL | |
| ctrl_btg_bt_rx, ctrl_nbtg_bt_tx | rtw8852bx_* | coex |
| query_ppdu, convert_rpl_to_rssi | rtw8852bx_* | 07 |
| phy_rpt_to_rssi, digital_pwr_comp, calc_rx_gain_normal | NULL | |
| cfg_txrx_path | rtw8852bx_bb_cfg_txrx_path | 05 |
| set_txpwr_ul_tb_offset | rtw8852bx_* | HE UL only |
| pwr_on_func / pwr_off_func | rtw8852b_pwr_on_func / _off_func (rtw8852b.c:385/514) | 02 |
| query_rxdesc | rtw89_core_query_rxdesc (AX v0) | 07 |
| fill_txdesc / fill_txdesc_fwcmd | rtw89_core_fill_txdesc (both) | 07 |
| get_ch_dma | {rtw89_core_get_ch_dma, same, NULL} | 07 |
| cfg_ctrl_path | rtw89_mac_cfg_ctrl_path | coex |
| mac_cfg_gnt | rtw89_mac_cfg_gnt | coex (§9) |
| stop_sch_tx / resume_sch_tx | rtw89_mac_stop_sch_tx / resume | |
| h2c_dctl_sec_cam, h2c_ampdu_cmac_tbl, h2c_punctured_cmac_tbl, h2c_default_dmac_tbl | NULL (wrappers return 0) | |
| h2c_default_cmac_tbl / h2c_assoc_cmac_tbl / h2c_txtime_cmac_tbl | rtw89_fw_h2c_* (v0) | 04 |
| h2c_update_beacon, h2c_wow_cam_update | AP / WoW only | |
| h2c_ba_cam | rtw89_fw_h2c_ba_cam | 04 |
| btc_* | rtw8852b(x)_btc_*; `btc_set_policy = rtw89_btc_set_policy_v1` | coex |

### 1.4 AX MAC gen hooks used by the glue (`rtw89_mac_gen_ax`, mac.c:7430-7514)

| Hook | Value |
|---|---|
| rx_fltr | R_AX_RX_FLTR_OPT = 0xCE20 |
| port_base | `rtw89_port_base_ax` (port_cfg 0xC400 …) |
| agg_len_ht | 0xC614 (RTS threshold) |
| ps_status | 0xCA0C |
| muedca_ctrl | {0xC370, BIT0\|BIT4} |
| bfee_ctrl | {0xCD80, HT\|VHT\|HE NDPA enable} |
| narrow_bw_ru_dis | {0xCCB0, BIT21} |
| agg_limit / ra_agg_limit | 0xC610 |
| txcnt_limit | 0xC62C |
| sys_init | sys_init_ax |
| trx_init | trx_init_ax |
| mac_func_en | NULL |
| hci_func_en | rtw89_mac_hci_func_en_ax |
| dmac_func_pre_en | rtw89_mac_dmac_func_pre_en_ax |
| cfg_ppdu_status | rtw89_mac_cfg_ppdu_status_ax |
| cfg_phy_rpt, set_edcca_mode | NULL (so `rtw89_mac_set_edcca_mode_bands` and `cfg_phy_rpt_bands` are no-ops) |
| fwdl_preconfig | NULL |
| fwdl_enable_wcpu | rtw89_mac_enable_cpu_ax |
| parse_efuse_map / parse_phycap_map | rtw89_parse_*_ax |
| cnv_efuse_state | no-op |
| scan_offload | rtw89_fw_h2c_scan_offload_ax (hw_scan only) |

### 1.5 Firmware feature flags for 8852B (fw.c:871-883) — FW 0.29.29.18

| Enabled | Not enabled (need newer FW) |
|---|---|
| NO_LPS_PG (≥29.26), TX_WAKE (≥29.26), CRASH_TRIGGER_TYPE_0 (≥29.29.0), SCAN_OFFLOAD (≥29.29.0), BEACON_FILTER (≥29.29.7), BEACON_LOSS_COUNT_V1 (≥29.29.15), NO_WOW_CPU_IO_RX (<29.30) | LPS_DACK_BY_C2H_REG, SER_L1_BY_EVENT (29.127); CRASH_TRIGGER_TYPE_1, SCAN_OFFLOAD_EXTRA_OP, BEACON_TRACKING (29.128); SIM_SER_L0L1_BY_HALT_H2C (29.130) |

The feature set is computed twice:
- **early**, in `rtw89_alloc_ieee80211_hw()` from the header of the requested file
  (fw.c:976-1015), to decide chanctx mode before `ieee80211_alloc_hw`;
- **fully**, in `rtw89_fw_recognize()` (fw.c:1038).

---------------------------------------------------------------------------------

## 2. Probe sequence (end to end)

Entry point: `rtw89_pci_probe()` (pci.c:4765-4839). Each step is marked **[SW]**
(no device access), **[HW]** (MMIO/config access) or **[FW]** (requires running
firmware).

1. **`rtw89_alloc_ieee80211_hw(dev, sizeof(struct rtw89_pci), info)`**
   (core.c:7533-7620)
   1. [SW] `rtw89_early_fw_feature_recognize()`: `request_firmware("rtw89/rtw8852b_fw-2.bin")`
      (falls back to `-1`, then no suffix). Reads the header version and fills
      `early_fw.feature_map` (fw.c:976). The `struct firmware` is kept in
      `rtwdev->fw.req.firmware` and is not requested again.
   2. [SW] `kmemdup(&rtw89_ops)`. Decide `no_chanctx` = `support_chanctx_num == 0 ||
      !SCAN_OFFLOAD || !BEACON_FILTER`. For 8852B with FW 0.29.29.18 it is **false**.
      If it were true: `add/remove/change/switch_vif_chanctx =
      ieee80211_emulate_*`, `assign/unassign_vif_chanctx = NULL`,
      `remain_on_channel/cancel = NULL` (core.c:7557-7570).
   3. [SW] `support_noise == false` → `ops->get_survey = NULL` (core.c:7572).
   4. [SW] `ieee80211_alloc_hw(sizeof(rtw89_dev) + sizeof(rtw89_pci), ops)`.
      `support_mlo = false` (support_link_num = 0).
      `wiphy->iface_combinations = rtw89_iface_combs`, `n = 2` (chanctx, 2 contexts)
      (core.c:7584-7593).
2. [SW] Set `hci.ops = &rtw89_pci_ops`, `hci.type = PCIE`,
   `hci.dle_type = PCIE`, `rpwm_addr = 0x10C0`, `cpwm_addr = 0x8170`. Run the
   DMI/SSID quirk checks (none for 8852BE). `SET_IEEE80211_DEV`.
3. **`rtw89_core_init()`** (core.c:6868-6956) — [SW] only:
   - I/O ops selected (MMIO; `support_fw_cmd_ofld = false`).
   - List heads and spinlocks (`ba_lock`, `rpwm_lock`, `tx_rpt.skb_lock`).
   - All work items (§7). The `rtw89_tx_wq` workqueue (WQ_UNBOUND | WQ_HIGHPRI).
   - `hal.rx_fltr = DEFAULT_AX_RX_FLTR` (0x030044BE, §4.3). `dbcc_en = false`.
     `mac.qta_mode = RTW89_QTA_SCC`.
   - `schedule_work(load_firmware_work)`: this only completes the completion,
     because the firmware was early-requested (fw.c:2046-2051).
   - `rtw89_ser_init`, `rtw89_entity_init` (default chandef = 2412 MHz
     `NL80211_CHAN_NO_HT` stored in chanctx 0, not marked active; chan.c:296-325,
     core.c:395), `rtw89_sar_init`, `rtw89_phy_ant_gain_init`.
4. **`rtw89_pci_claim_device()`** (pci.c:3270): `pci_enable_device`,
   `pci_set_master`, `pci_set_drvdata(hw)`.
5. **`rtw89_pci_setup_resource()`** (pci.c:3849): BAR mapping and DMA mask; allocate
   all TX/RX BD/WD rings; H2C skb queue; `irq_lock`, `trx_lock`. [HW-alloc] (track 01).
6. **`rtw89_chip_info_setup()`** (core.c:7279-7334):
   1. [HW] `rtw89_read_chip_ver()` (core.c:7030):
      - `hal.cv = R_AX_SYS_CFG1 (0x00F0) bits[15:12]` (reference system: 1 = CBV).
      - `hal.acv = XTAL-SI reg 0x41 (XTAL_SI_CV) bits[3:0]`.
      - CID/AID are BE-only (both remain 0).
   2. [HW] `rtw89_mac_pwr_on()` (mac.c:1586): `power_switch(true)` → `reset_pwr_state`
      → `rtw8852b_pwr_on_func` (track 02). If it fails, power off and retry once. On
      the first power-on (PROBE_DONE not yet set) it also reads the efuse ECV and the
      FW-secure efuse bits (mac.c:1556-1559). Sets flags POWERON/DMAC_FUNC/CMAC0_FUNC.
      Writes the coex scoreboard (MAC_AX_NOTIFY_TP_MAJOR) and clears AON interrupts.
   3. [SW] `rtw89_wait_firmware_completion()`.
   4. [SW] `rtw89_fw_recognize()` (fw.c:1038): picks the NORMAL_CE image for this CV
      from the multi-firmware file (falls back to NORMAL), validates the minimum
      version, recognizes the optional WoWLAN and log-format images, fills the
      feature flags, and recognizes the coex version. Track 03.
   5. **`rtw89_chip_efuse_info_setup()`** (core.c:7180-7206):
      1. [HW][FW] **`rtw89_mac_partial_init(rtwdev, false)`** (mac.c:4307):
         HCI DMA TRX on, `dmac_pre_init` (hci_func_en, dmac_func_pre_en, DLE init in
         **DLFW quota**, HFC init), PCI `mac_pre_init`, then
         **`rtw89_fw_download(NORMAL)`**. This is a full firmware download at probe
         time. Interrupts are not requested yet, so it runs on polling. Track 03.
      2. [HW] `parse_efuse_map_ax` (efuse.c:281): autoload check
         (`R_AX_SYS_WL_EFUSE_CTRL 0x000A` bit5), dump the 1216-byte physical efuse
         via `R_AX_EFUSE_CTRL 0x0030` (no firmware needed), build the 2048-byte
         logical map, then `rtw8852bx_read_efuse`. Logical-map offsets
         (`struct rtw8852bx_efuse`, rtw8852b_common.h:36-94):

         | Logical offset | Field |
         |---|---|
         | 0x210 | path-A TSSI (32 bytes) |
         | 0x23A | path-B TSSI (32 bytes) |
         | 0x2B8 | channel_plan |
         | 0x2B9 | xtal_k |
         | 0x2CA | **rfe_type** |
         | 0x2CB-0x2CC | **country_code** |
         | 0x2D0 / 0x2D1 | thermal A / B |
         | 0x2D4-0x2DC | RX gain offsets |
         | **0x400** | **PCIe MAC address (6 bytes)** (USB: 0x488) |

      3. [HW] `parse_phycap_map_ax`: physical-efuse bytes 0x580..0x5FF →
         `rtw8852bx_read_phycap` (power-cal, TSSI trim, thermal trim, PA-bias trim,
         gain comp). No firmware needed.
      4. [FW] **`rtw89_mac_setup_phycap()`** (mac.c:3322): H2C-register
         `FUNC_GET_FEATURE` part 0 → C2H-register PHY_CAP:
         - `tx_nss = W1[7:0]`, `rx_nss = W0[23:16]`, `ant_tx/rx = W3`,
           `protocol = W1[15:8]`.
         - `hal.tx_nss/rx_nss = min(FW, chip = 2)`.
         - `ant_tx == 1` → `antenna_tx = RF_B`; `protocol < 11BE` → `no_eht`.
         - AX chips skip part 1.
         - **This is the only probe-time step that needs running firmware.**
      5. [SW] `rtw89_core_setup_phycap`: `support_cckpd = true` for 8852B CV > CAV;
         `support_igi = false`; `thermal_prot_th = 0`.
      6. [HW] `rtw89_hci_mac_pre_deinit` → PCI `power_wake(false)`.
   6. [SW] `rtw89_fw_recognize_elements()` (fw.c:1542): parses the BB, radio A/B,
      NCTL, TXPWR, PWR_TRK and REGD elements from the firmware file into
      `fw.elm_info` (tracks 03/05).
   7. `board_info_setup` (fem_setup NULL), `rtw89_chip_data_setup` (NULL):
      no-ops.
   8. [SW] `rtw89_core_setup_rfe_parms` (core.c:7090): `sel = dflt_parms`, then
      `rtw89_load_rfe_data_from_fw()` overlays the FW-element TX-power tables, then
      `rtw89_load_txpwr_table(byr_tbl)`.
   9. [SW] `ps_mode = rtw89_update_ps_mode()`: NONE if the `disable_ps_mode=Y`
      parameter is set (the reference configuration), otherwise CLK_GATED
      (because of NO_LPS_PG).
   10. Logs "chip info CID … CV … RFE …".
   11. [HW] **`rtw89_mac_pwr_off()`**. This runs always, on both success and error
       paths.
7. [HW] `rtw89_pci_basic_cfg(rtwdev, false)` (pci.c:4605): disable EQ, filter-out,
   CPL timeout, link config (ASPM), L1SS config. Track 01; the reference system keeps
   ASPM L1/L1SS/CLKREQ off.
8. [SW] `rtw89_core_napi_init()` (dummy netdev + NAPI).
9. [HW] `rtw89_pci_request_irq()` (pci.c:4056): 1 vector (MSI or INTx), threaded IRQ
   `rtw89_pci_interrupt_handler`/`_threadfn`, then
   `config_intr_mask(RTW89_PCI_INTR_MASK_RESET)`. Interrupts are **not** enabled
   here.
10. **`rtw89_core_register()`** → `rtw89_core_register_hw()` (core.c:7357-7498):
    fill in `ieee80211_hw`/wiphy (§5); `rtw89_core_set_supported_band`;
    `rtw89_regd_setup` (sets `reg_notifier`); `sar_capa`;
    **`ieee80211_register_hw()`**; `rtw89_regd_init_hint()`;
    `rtw89_rfkill_polling_init()`.
    - The rfkill init performs [HW] GPIO9 pinmux/mode writes on a powered-off chip
      (see the note below).
    - Then `rtw89_phy_dm_init_data` (a no-op when `support_noise = false`) and
      debugfs.
11. `set_bit(RTW89_FLAG_PROBE_DONE)`.

Note on step 10: the chip is powered off during and after registration.
`rtw89_core_rfkill_init` writes 0x02D4/0x0062 and reads 0x0060 at that point, so
the GPIO block must be in the always-on domain. This is inferred from the ordering,
not documented anywhere.

**Remove** (pci.c:4860-4875): free IRQ → NAPI deinit → `rtw89_core_unregister` (stop
rfkill polling; `ieee80211_unregister_hw` calls `.stop` if the device is running) →
free rings/unmap → `pci_disable_device` → `rtw89_core_deinit` (SER deinit, unload
firmware, free early H2Cs, destroy tx wq) → `rtw89_free_ieee80211_hw`.

### 2.1 Work done at probe time vs at `.start`

| Operation | Probe | `.start` (`rtw89_core_start`) |
|---|---|---|
| Read CV/ACV | yes | no (cached) |
| Power on | yes (then off) | yes |
| DLE/HFC in DLFW quota | yes | yes |
| Firmware download | **yes** | **yes** |
| Efuse and phycap-efuse read | yes | no (cached) |
| FW phycap (H2C-reg) | yes | no |
| FW element parsing / TX-power table load | yes | no |
| DLE/HFC in SCC quota, sys_init/trx_init | no | yes |
| BB/RF table load, DM init, RFK init | no | yes |
| Coex init | no | yes |
| Interrupts enabled | no | yes (last) |
| Channel programmed | no | **no** (first at chanctx assign or `.config`) |
| Per-vif port/CAM/role | no | no (`add_interface`) |

---------------------------------------------------------------------------------

## 3. `.start` and `.stop`

### 3.1 `rtw89_ops_start` → `rtw89_core_start()` (mac80211.c:57, core.c:6565-6632)

Called with the wiphy mutex held. Ordered steps:

1. **`rtw89_mac_preinit()`** (mac.c:4341): `rtw89_mac_pwr_on()` (as in probe step
   6.2); `mac_func_en` is NULL for AX.
2. `rtw89_chip_bb_preinit()`: NULL for 8852B (`bbmcu_nr == 0`, so it would run
   here, but the op is NULL).
3. `rtw89_phy_init_bb_afe()` (phy.c:1913): applies the FW-file AFE element if one
   exists. The 8852B `-2` file has no AFE element (README.md), so this is a no-op.
4. **`rtw89_mac_init()`** (mac.c:4359-4396). On any failure it calls
   `rtw89_mac_pwr_off`.
   1. `rtw89_mac_partial_init(include_bb = false)`: HCI DMA on; `dmac_pre_init`
      (write 0x8400 R_AX_DMAC_FUNC_EN = 0x60440000: MAC_FUNC \| DMAC_FUNC \|
      DISPATCHER \| PKT_BUF; write 0x8404 R_AX_DMAC_CLK_EN = 0x00040000; DLE in
      DLFW quota; HFC; mac.c:4208-4280); PCI
      `mac_pre_init_ax`; **firmware download** (normal image).
   2. `rtw89_chip_enable_bb_rf()` → `rtw8852bx_mac_enable_bb_rf`.
   3. `sys_init_ax` (mac.c:1697): `dmac_func_en_ax`, `cmac_func_en_ax(0)`,
      `chip_func_en_ax`.
   4. `trx_init_ax` (mac.c:4029): DMAC init (DLE/HFC in SCC quota, …), CMAC init
      (including `rx_fltr_init_ax`, which writes `hal.rx_fltr` to 0xCE20 and the
      PLCP header filter to 0xCE04), error IMRs, host RPR, …. Track 02.
   5. `rtw89_mac_feat_init`: no-op (only for BACAM_V1).
   6. PCI `mac_post_init_ax` (pci.c:3238): LTR, 8-byte address-info mode, enable all
      TX DMA channels, clear `B_AX_STOP_WPDMA|B_AX_STOP_PCIEIO` in 0x1010.
   7. `rtw89_fw_send_all_early_h2c()` (debugfs-queued H2Cs; normally empty).
   8. **H2C `OFLD_CFG`**: cat MAC(1), class FW_OFLD(0x9), func 0x14, 8 bytes
      `09 00 00 00 5E 00 00 00` (fw.c:5274-5303).
5. `rtw89_btc_ntfy_poweron()`: counter only.
6. `rtw89_chip_reset_bb_rf()` (mac.h:1371): AX only: `disable_bb_rf` then
   `enable_bb_rf`.
7. **`rtw89_phy_init_bb_reg()`** (phy.c:1885): BB table (FW element preferred) →
   `init_txpwr_unit` → BB gain table → `rtw89_phy_bb_reset`.
8. `rtw89_chip_bb_postinit()`: NULL.
9. **`rtw89_phy_init_rf_reg(rtwdev, false)`**: radio A/B tables (FW elements
   preferred). Track 05.
10. **`rtw89_btc_ntfy_init(BTC_MODE_NORMAL)`** (coex.c:7746): `btc_set_rfe`,
    `btc_init_cfg` (PTA/GNT registers, §9), scoreboard, coex H2Cs, `_run_coex`.
11. **`rtw89_phy_dm_init()`** (phy.c:8163), in this order:
    - stat init, `bb_sethw`, diag, env monitor, NHM, PHY-status parsing init,
      DIG init, CFO init, BB wrap, EDCCA init, ch-info, UL-TB, antdiv, rfe_gpio
      (NULL), antdiv set;
    - `rfk_hw_init` (NULL);
    - **`rtw89_phy_init_rf_nctl`**;
    - **`rtw89_chip_rfk_init` = `rtw8852b_rfk_init`**: DPK init, RCK, DACK
      (chanctx 0), RX-DCK;
    - `set_txpwr_ctrl`, `power_trim`, `cfg_txrx_path`.
    - Note: RFK init runs while the RF is still on the table-default channel.
12. `rtw89_mac_set_edcca_mode_bands(true)`: no-op on AX.
13. **`rtw89_mac_cfg_ppdu_status_bands(true)`** (mac.c:6289):
    - write 0xCE40 (R_AX_PPDU_STAT) = `RPT_EN BIT0 | APP_MAC_INFO BIT1 |
      APP_PLCP_HDR BIT3 | RPT_CRC32 BIT5` = 0x2B;
    - 0x9C18 (R_AX_HW_RPT_FWD) bits[1:0] = 1 (PPDU status to host).
14. `rtw89_mac_cfg_phy_rpt_bands(true)`: no-op on AX.
15. **`rtw89_mac_update_rts_threshold()`** (mac.c:6314): see §4.8.
16. **`rtw89_hci_start()`** → PCI `ops_start` (pci.c:1922): `napi_enable` and
    **enable interrupts** (`rtwpci->running = true`).
17. Queue `track_work` (2 s) and `track_ps_work` (100 ms) as wiphy delayed work.
18. `set_bit(RTW89_FLAG_RUNNING)`. C2H events are ignored until this is set
    (fw.c:8107).
19. `rfk_init_late`: NULL.
20. `rtw89_btc_ntfy_radio_state(BTC_RFCTRL_WL_ON)`.
21. H2C `LOG_CFG` (`rtw89_fw_h2c_fw_log`, cat MAC, class FW_INFO(0), func 0).
22. `rtw89_fw_h2c_init_ba_cam`: no-op (BACAM_V0).
23. `rtw89_tas_fw_timer_enable`: no-op (AX). `ps_hang_cnt = 0`.

Everything before step 16 runs with interrupts masked. Every H2C sent before step 16
(including OFLD_CFG and the coex H2Cs) is fire-and-forget on the H2C DMA ring
(CH12); no C2H is processed until the device is RUNNING.

### 3.2 `rtw89_ops_stop` → `rtw89_core_stop()` (core.c:6634-6676)

1. Return early if not RUNNING. This prevents a double stop between IPS and `.stop`.
2. `tas_fw_timer_enable(false)` (no-op). `btc_ntfy_radio_state(WL_OFF)`.
3. Clear RUNNING.
4. Cancel works:
   - wiphy works: `c2h_work`, `cancel_6ghz_probe_work`, the btc
     eapol/arp/dhcp/icmp works;
   - `txq_reinvoke_work` (sync);
   - wiphy delayed works: `tx_wait_work`, `track_work`, `track_ps_work`,
     `chanctx_work`, `coex_act1_work`, `coex_bt_devinfo_work`, `coex_rfk_chk_work`,
     `cfo_track_work`, `mcc_prepare_done_work`, `antdiv_work`;
   - `forbid_ba_work` (sync).
5. `rtw89_btc_ntfy_poweroff()`: clear the WL scoreboard, run coex, disable FW
   reports.
6. `rtw89_hci_flush_queues(all ACs, drop = true)`; `rtw89_mac_flush_txq(all,
   drop = true)`.
7. `rtw89_hci_stop()` → PCI: disable interrupts, `synchronize_irq`, NAPI stop
   (pci.c:1930).
8. `rtw89_hci_deinit()` → PCI `ops_deinit` (pci.c:3053): power_wake off, LTR off,
   stop all DMA, clear all ring indexes.
9. **`rtw89_mac_pwr_off()`** → `rtw8852b_pwr_off_func`. Clears
   POWERON/DMAC/CMAC/FW_RDY. The chanctx entity state becomes inactive.
10. `rtw89_hci_reset()` → PCI `ops_reset` (pci.c:1878): reset TRX rings, release
    pending TX skbs and queued FW commands.

The firmware is lost at every `.stop`. Every `.start` downloads it again.

### 3.3 IPS: power-off while idle (active on the reference system)

These three hooks together mean the chip is powered off while no interface is
associated or scanning:

- `rtw89_ops_config()`: when `CONF_CHANGE_IDLE` goes to idle and the device is not
  scanning → `rtw89_enter_ips()`. When it goes to not-idle → `rtw89_leave_ips()`
  (mac80211.c:75-105).
- `rtw89_enter_ips` (ps.c:233): per vif `rtw89_mac_vif_deinit`, then
  `rtw89_core_stop`.
- `rtw89_leave_ips` (ps.c:257): `rtw89_core_start`, `rtw89_set_channel`, then per
  vif `rtw89_mac_vif_init` (§4.1).

Two more triggers:
- `add_interface`, `hw_scan`, `set_key` (WEP) and `remove_interface` also leave or
  re-enter IPS based on `hw->conf.flags & IEEE80211_CONF_IDLE` (ps.h:32-50).
- After an HW scan completes while idle, `ips_work` re-enters IPS (core.c:7027).

Recommendation: do not implement IPS. Ignore `CONF_CHANGE_IDLE`.

---------------------------------------------------------------------------------

## 4. `ieee80211_ops` implemented by rtw89 (mac80211.c:2014-2067)

Every op except `tx`, `wake_tx_queue`, `sta_statistics`, `rfkill_poll` and
`get_survey` asserts that the wiphy lock is held. Nearly every op first calls
`rtw89_leave_ps_mode()`, which is a no-op when `ps_mode == NONE`, and some call
`rtw89_leave_lps()`.

Minimal-driver verdicts: **R** = required, **P** = required from a later phase,
**S** = stub or trivial, **O** = omit (NULL).

| op | rtw89 behaviour (8852B) | Minimal |
|---|---|---|
| tx (19) | ROC queueing if off-channel; `rtw89_core_tx_write` + `tx_kick_off` (track 07) | R |
| wake_tx_queue (48) | `ieee80211_schedule_txq` + `queue_work(txq_wq, txq_work)` → per-AC scheduler with BA auto-start (§7) | R (use `ieee80211_handle_wake_tx_queue` first) |
| start / stop (57/66) | §3 | R |
| config (75) | cancel `ips_work`; leave deep PS; IDLE → IPS; **CHANGE_CHANNEL → `rtw89_config_entity_chandef(CHANCTX_0, &hw->conf.chandef)` + `rtw89_set_channel()`**; CHANGE_MONITOR → `rtw89_physts_parsing_init` | R (channel only) |
| add_interface (169) | §4.1 | R |
| change_interface (282) | remove + add | O (mac80211 falls back to remove + add) |
| remove_interface (243) | §4.1 | R |
| configure_filter (308) | §4.3 | R (mandatory in mac80211) |
| vif_cfg_changed (699) / link_info_changed (750) | §4.4 | R (as a single `bss_info_changed`) |
| start_ap / stop_ap / set_tim / channel_switch_beacon | AP only | O |
| conf_tx (912) | §4.6, EDCA via H2C + MU-EDCA registers | P (phase 5; FW defaults work) |
| sta_state (975) | §4.5 | R |
| set_key (987) | sec CAM via H2C (cam.c:465); `-EOPNOTSUPP` for TKIP and unknown ciphers → SW fallback | O in phase 4 (NULL = all-SW crypto), P in phase 5 |
| ampdu_action (1026) | §4.9 | P |
| get_survey | NULL for 8852B (core.c:7572) | O |
| set_rts_threshold (1078) | if POWERON, `rtw89_mac_update_rts_threshold` | S |
| sta_statistics (1092) | `sinfo->txrate` from FW RA report C2H | O (P later) |
| flush (1122) | leave LPS; `hci_flush_queues(queues, drop)`; drop → H2C pkt-drop per vif (`!NO_PACKET_DROP`), else `mac_flush_txq` | R (wait for TX rings to drain) |
| set_bitrate_mask | RA mask override | O |
| set_antenna / get_antenna (1191/1219) | stores `hal.antenna_tx/rx` (applied at next `cfg_txrx_path`, i.e. next start) | O |
| sw_scan_start / sw_scan_complete (1231/1252) | §4.7 | R (with SW scan) |
| reconfig_complete (1270) | RESTART → `rtw89_ser_recfg_done` | O |
| hw_scan / cancel_hw_scan (1279/…) | FW scan offload; returns 1 (→ SW scan) if no SCAN_OFFLOAD | O |
| add/remove/change_chanctx, assign/unassign_vif_chanctx, switch_vif_chanctx | §4.2 | use `ieee80211_emulate_*` |
| remain_on_channel / cancel | driver-side ROC (set to NULL in no_chanctx mode) | O |
| set_sar_specs, set_tid_config, can_activate_links, change_vif_links, change_sta_links | SAR / TID / MLO | O |
| link_sta_rc_update (1348) | `rtw89_phy_ra_update_sta_link` (RA H2C) | P (with RA) |
| suspend / resume / set_wakeup / set_rekey_data | WoWLAN | O |
| rfkill_poll (2000) | poll GPIO9 only when not RUNNING | O (or S) |
| mgd_prepare_tx / mgd_complete_tx | **not implemented by rtw89** | O |

Signatures that changed in recent kernels (7.2.7 `include/net/mac80211.h`, struct
`ieee80211_ops`):

```text
stop(hw, bool suspend)
config(hw, int radio_idx, u32 changed)
set_rts_threshold(hw, int radio_idx, u32)
set_antenna(hw, int radio_idx, u32, u32)
get_antenna(hw, int radio_idx, u32 *, u32 *)
conf_tx(hw, vif, unsigned int link_id, u16 ac, const struct ieee80211_tx_queue_params *)
link_sta_rc_update(hw, vif, struct ieee80211_link_sta *, u32)
```

`ieee80211_handle_wake_tx_queue()` exists (mac80211.h:7779).

### 4.1 add_interface / remove_interface — hardware and firmware state

**`rtw89_ops_add_interface`** (mac80211.c:169-241):

1. Leave IPS if idle.
2. If FW has BEACON_FILTER: set `vif->driver_flags |= IEEE80211_VIF_BEACON_FILTER |
   IEEE80211_VIF_SUPPORTS_CQM_RSSI` (true on this FW).
3. **mac_id** = first zero bit in `mac_id_map[128]` (core.c:6678). The first vif
   gets 0.
4. **port** = first zero bit in `hw_port[5]`. The first vif gets 0.
5. `rtw89_init_vif`: link instance 0 gets `mac_id`, `mac_idx = MAC_0`,
   `phy_idx = PHY_0`, `port`.
6. Initialise the vif's txq, add the vif to the list, copy the MAC address, init ROC
   and traffic stats.
7. `rtw89_vif_set_link(RTW89_VIF_IDLE_LINK_ID = 0)`.
8. `__rtw89_ops_add_iface_link` (mac80211.c:107):
   - `rtw89_vif_type_mapping(assoc = false)`: STATION → `wifi_role = STATION`,
     `net_type = NO_LINK (0)`, `self_role = CLIENT`,
     `addr_cam.sec_ent_mode = NORMAL`.
   - Init the per-link works. `chanctx_idx = 0`, `chanctx_assigned = false`.
     Copy `bss_conf->addr`.
   - **`rtw89_mac_add_vif` = `rtw89_mac_vif_init`** (mac.c:5044-5084), in order:
     1. **`rtw89_mac_port_update`**: port register programming (table below).
     2. `rtw89_mac_dmac_tbl_init(mac_id)`, only if the firmware is not secure-boot:
        four times, write 0x0C04 (R_AX_FILTER_MODEL_ADDR) =
        `0x18800000 + (mac_id << 4) + i*4`, then write 0x40000 = 0 (mac.c:4402).
     3. `rtw89_mac_cmac_tbl_init(mac_id)`, same condition: write 0x0C04 =
        `0x18840000 + mac_id*0x20`, then write the eight dwords at
        0x40000…0x4001C = {0x00000004, 0x400A0004, 0, 0, 0, 0x0E43000B, 0,
        0x000B8109} (mac.c:4417).
     4. H2C **MACID_PAUSE** (unpause): cat 1, class FW_OFLD 0x9, func 0x8
        (mac.c:4436).
     5. H2C **FWROLE_MAINTAIN**(CREATE = 0): class MEDIA_RPT 0x8, func 0x4.
     6. H2C **JOININFO**(dis_conn = true): class 0x8, func 0x0.
     7. `rtw89_cam_init`: allocate a BSSID-CAM index (of 10) and an addr-CAM index
        (of 128). Both are 0 for the first vif (cam.c:741).
     8. H2C **ADDR_CAM_UPD**(CREATE): class 0x6, func 0x0. Carries SMA = own MAC,
        BSSID CAM, mac_id, port, net_type, …. Track 04.
     9. H2C **CCTLINFO_UD** (default cmac table): class FR_EXCHG 0x5, func 0x2.
     10. `default_dmac_tbl`: NULL.
   - `rtw89_btc_ntfy_role_info(ROLE_START)`.
9. `pure_monitor_mode_vif` is set if the vif type is MONITOR.
10. `rtw89_recalc_lps`.

**Port update for port 0, mac_idx 0** (`rtw89_mac_port_update`, mac.c:5103-5132).
It runs at add_interface (net_type NO_LINK), at association (INFRA; §4.5) and at
IPS leave. Steps:

| # | Step | Register / action |
|---|---|---|
| 1 | func_sw | If 0xC400 bit2 (PORT_FUNC_EN) is set: `msleep(beacon_int + 1)`; clear 0xC400 BIT2\|BIT16; set BIT5 (TSFTR_RST); write 0xC434 = 0 |
| 2 | tx_rpt off | clear 0xC400 BIT1 |
| 3 | rx_rpt off | clear 0xC400 BIT0 |
| 4 | net_type | 0xC400[11:10] = net_type (NO_LINK 0 / ADHOC 1 / INFRA 2 / AP 3) |
| 5 | bcn_prct | net_type ≠ NO_LINK: set 0xC400 BIT13\|BIT16; else clear |
| 6 | rx_sw | INFRA/ADHOC: set 0xC400 BIT4 (RX_BSSID_FIT_EN); else clear |
| 7 | rx_sync | INFRA/ADHOC: set 0xC400 BIT3 (TSF_UDT_EN); else clear |
| 8 | tx_sw | AP/ADHOC: set 0xC400 BIT12 (BCNTX_EN); else clear |
| 9 | bcn_intv | 0xC414[15:0] = `beacon_int` (100 if 0) |
| 10 | hiq_win | write8 0xC590 = 0 (16 for AP) |
| 11 | hiq_dtim | set 0xCA08 BIT1\|BIT0; 16-bit 0xC426 bits[15:8] = `dtim_period` |
| 12 | hiq_drop | 0xC63C: clear bit (16+port); port 0 also clears BIT0 |
| 13 | bcn_setup | 0xC404[7:0] = 2 |
| 14 | bcn_hold | 0xC404[27:16] = 200 |
| 15 | bcn_mask_area | 0xC408[27:16] = 0 |
| 16 | tbtt_early | 16-bit 0xC40E [11:0] = 5 |
| 17 | tbtt_agg | 16-bit 0xC412 [15:8] = 1 |
| 18 | bss_color | 0xC6A0[5:0] = HE BSS color |
| 19 | mbssid | port 0, non-AP: clear 0xC568 bits[23:1] (mask 0xFFFFFE) |
| 20 | func_en | set 0xC400 BIT2 |
| 21 | tsf_resync_all | no-op without AP vifs |
| 22 | delay | `fsleep(20)` |
| 23 | bcn_early | 0xC40C[11:0] = 160 |
| 24 | bcn_psr_rpt | 0xCE84[10:0] = BSSID index (non-transmitted BSS) or 0 |

**`rtw89_ops_remove_interface`** (mac80211.c:243-280):
1. Cancel `roc_work`.
2. `__rtw89_ops_remove_iface_link`: cancel link works, `btc ROLE_STOP`,
   **`rtw89_mac_vif_deinit`** (mac.c:5086):
   - H2C FWROLE_MAINTAIN(REMOVE = 1);
   - `rtw89_cam_deinit` (free the addr/BSSID CAM indexes);
   - H2C ADDR_CAM_UPD(REMOVE).
   The port registers are not cleared.
3. Unset the link, remove the vif from the list, release the port and `mac_id`.
4. `recalc_lps`, then enter IPS if idle.

### 4.2 Channel path: chanctx vs `.config`

rtw89 has one internal channel model: `hal.chanctx[idx].chandef` (a
cfg80211_chan_def), plus `entity_map` and `rtw89_set_channel()`, which recomputes
entity mode and programs PHY_0 from chanctx 0 (core.c:563-579).

| Mode | How a channel reaches `rtw89_set_channel` |
|---|---|
| **chanctx** (reference system) | `add_chanctx` → `rtw89_config_entity_chandef(idx = first free, &ctx->def)` (chan.c:3348). `assign_vif_chanctx` → set `chanctx_idx`/`assigned`, abort HW scan, move the first active context to index 0, then **`rtw89_set_channel()`** (chan.c:3391-3430). `change_chanctx(WIDTH)` → config + `set_channel` (chan.c:3375). `unassign` → possible MCC stop + `set_channel`. |
| **no_chanctx** (older FW) | ops are `ieee80211_emulate_{add,remove,change,switch_vif}_chanctx`; `assign/unassign = NULL`. mac80211 then delivers channel changes as **`.config(IEEE80211_CONF_CHANGE_CHANNEL)` with `hw->conf.chandef`**. `rtw89_ops_config` copies that into chanctx 0 and calls `rtw89_set_channel()` (mac80211.c:90-94). SW scan and SW ROC use the same path. |

`rtw89_set_channel` → `__rtw89_set_channel(chan0, MAC_0, PHY_0)` (core.c:531-561):

1. Convert the chandef to `rtw89_chan`: center channel, primary channel, band,
   bandwidth, subband, `pri_ch_idx`, `pri_sb_idx` (core.c:401-472, chan.c:131-155).
2. `set_channel_help(enter)`: stop the scheduler TX, PPDU status off, TSSI off, ADC
   FIFO reset, BB reset.
3. `chip->ops->set_channel` = MAC (`rtw8852bx_set_channel_mac`) → BB → RF. Track 05.
4. `chip->ops->set_txpwr` (by-rate + limit tables, using the current regd; §6).
5. `set_channel_help(exit)`.
6. If this is the first activation or the band changed: `btc_ntfy_switch_band` and
   `rfk_band_changed` (TSSI scan).
7. Mark the entity active. For a pure-monitor vif, run `rfk_channel`.

RF calibration for the operating channel (`rtw89_chip_rfk_channel`) is **not** part
of `set_channel`. In station mode it runs in `sta_state NOTEXIST→NONE` (§4.5).

**Minimal driver:** use exactly rtw89's no_chanctx configuration:
- `.add_chanctx = ieee80211_emulate_add_chanctx`, and likewise for remove, change and
  switch_vif;
- no assign/unassign ops;
- implement `.config` for `IEEE80211_CONF_CHANGE_CHANNEL` using `hw->conf.chandef`.

This is proven in 7.2.7, because rtw89 selects it at runtime (core.c:7561-7570).
Keep a single "current chandef" in the driver, and use it for
`rx_status->freq/band` (core.c:4340).

### 4.3 configure_filter → RX filter (mac80211.c:308-388)

Supported flags: `FIF_ALLMULTI | FIF_OTHER_BSS | FIF_FCSFAIL |
FIF_BCN_PRBRESP_PROMISC | FIF_PROBE_REQ`. The driver keeps `hal.rx_fltr`, which
starts at DEFAULT_AX_RX_FLTR, and writes it to **R_AX_RX_FLTR_OPT 0xCE20** with RMW,
preserving bits[21:16] (`B_AX_RX_MPDU_MAX_LEN_MASK`) (mac.c:2678-2689).

R_AX_RX_FLTR_OPT bits (reg.h:3325-3345). Semantics: "A_*" bits are acceptance
checks; clearing one makes that class promiscuous.

| Bit | Name | Default | Flag action |
|---|---|---|---|
| 31:24 | UID_FILTER | 3 | |
| 23:22 | UNSPT_FILTER | 0 | |
| 21:16 | RX_MPDU_MAX_LEN | preserved | |
| 14 | A_FTM_REQ | 1 | |
| 13 | A_ERR_PKT | 0 | |
| 12 | A_UNSUP_PKT | 0 | |
| 11 | A_CRC32_ERR | 0 | FIF_FCSFAIL → set (accept bad FCS) |
| 10 | A_PWR_MGNT | 1 | |
| 9:8 | A_BCN_CHK_RULE | 0 | |
| 7 | A_BCN_CHK_EN | 1 | BCN_PRBRESP_PROMISC → clear |
| 6 | A_MC_LIST_CAM_MATCH | 0 | |
| 5 | A_BC_CAM_MATCH | 1 | FIF_PROBE_REQ → clear |
| 4 | A_UC_CAM_MATCH | 1 | FIF_PROBE_REQ → clear |
| 3 | A_MC | 1 | FIF_ALLMULTI → clear |
| 2 | A_BC | 1 | BCN_PRBRESP_PROMISC → clear |
| 1 | A_A1_MATCH | 1 | FIF_OTHER_BSS or BCN_PRBRESP_PROMISC → clear |
| 0 | SNIFFER_MODE | 0 | set iff `hw->conf.flags & IEEE80211_CONF_MONITOR` |

DEFAULT_AX_RX_FLTR = **0x030044BE**.

While `rtwdev->scanning` is set, rtw89 additionally clears BCN_CHK_EN, A_BC and
A1_MATCH, because mac80211 does not call configure_filter for HW scan. The same
value is re-applied by `rx_fltr_init_ax` at every start (mac.c:2692-2721).

The per-type filters are set at init to "forward to host" for MGNT/CTRL/DATA: 0xCE28,
0xCE24 and 0xCE2C (`RX_FLTR_FRAME_TO_HOST`, mac.c:2640-2676). The PLCP header
filter 0xCE04 = CCK_CRC | CCK_SIG | LSIG_PARITY | SIGA_CRC | VHT_SU_SIGB |
VHT_MU_SIGB | HE_SIGB = 0x7F.

### 4.4 bss_info / vif_cfg / link_info changes

rtw89 uses the split `vif_cfg_changed` + `link_info_changed` API (mac80211.c:699-798).
Bits handled:

| Change bit | rtw89 action | Needed for minimal station |
|---|---|---|
| ASSOC (vif) | assoc=1: `rtw89_station_mode_sta_assoc` (runs the deferred sta assoc, §4.5), then `__rtw89_ops_bss_link_assoc`: `rtw89_phy_set_bss_color`, UL-TB power offset, **`rtw89_mac_port_update`** (now INFRA), `set_he_obss_narrow_bw_ru`, `set_he_tb` (BE only), queue chanctx work. assoc=0: abort HW scan. | yes |
| PS (vif) | `rtw89_recalc_lps` | no |
| ARP_FILTER | store IPv4 (WoW) | no |
| MLD_VALID_LINKS | MLO | no |
| BSSID (link) | copy BSSID; `rtw89_cam_bssid_changed`; H2C ADDR_CAM_UPD(INFO_CHANGE = 3); reset `sync_bcn_tsf` | yes (arrives before auth) |
| BEACON | AP | no |
| ERP_SLOT | re-send all four EDCA H2Cs (AIFS depends on slot time) | yes, with conf_tx |
| HE_BSS_COLOR | `rtw89_phy_set_bss_color` | optional |
| MU_GROUPS | VHT MU GID table | no |
| P2P_PS | P2P | no |
| CQM | H2C BCN filter config | no (without beacon-filter offload) |
| TPE | 6 GHz | no |

Not handled at all: ERP_CTS_PROT, ERP_PREAMBLE (no CCK rate in the table carries a
short-preamble flag), HT, BASIC_RATES, BEACON_INT (taken by `port_update` at assoc),
TXPOWER, BANDWIDTH (handled through chanctx/config).

`rtw89_phy_set_bss_color` (phy.c:8234-8268), applied only if HE and associated:
- BB 0x43B0 BIT28 = 1;
- 0x43B0 bits[27:22] = color;
- 0x43B0 bits[21:11] = AID.

### 4.5 sta_state transitions and association timeline

`__rtw89_ops_sta_state` (mac80211.c:939-973). For the station interface's AP entry
(`vif->type == STATION && !sta->tdls`):

| Transition | Action |
|---|---|
| NOTEXIST → NONE | **`__rtw89_ops_sta_add`** (mac80211.c:488). `mlo_mode = MLSR`. The **AP sta reuses the vif's mac_id** (no new mac_id, no new addr CAM). `rtw89_init_sta`, txq init, `sta_set_link`. Then **`rtw89_core_sta_link_add`** (core.c:5656): init EWMAs; `reg_6ghz_recalc` (no-op off 6 GHz); `btc CONN_START`; **`rtw89_chip_rfk_channel` = `rtw8852b_rfk_channel` (RX-DCK, IQK, TSSI, DPK on the current channel)**; `rtw89_phy_dig_suspend`. Finally `init_trx_protect` (BE-only; no-op). |
| NONE → AUTH | nothing |
| AUTH → ASSOC | returns 0; the work is deferred to `vif_cfg_changed(ASSOC)` so that vif info is available |
| vif_cfg_changed(ASSOC, assoc=1) | `__rtw89_ops_sta_assoc(station_mode = true)`: `rtw89_vif_type_mapping(assoc = true)` → `net_type = INFRA`, `trigger = he_support`. Then **`rtw89_core_sta_link_assoc`** (core.c:5799), in order: (1) H2C **assoc CMAC table** (CCTLINFO_UD); (2) H2C **JOININFO**(dis_conn = false); (3) H2C **ADDR_CAM_UPD**(CON_DISCONN = 4) — AID, mac_id, net_type; (4) **`rtw89_phy_ra_assoc`** → H2C RA MACIDCFG (cat OUTSRC 2, class RA 1, func 0); (5) `rtw89_mac_bf_assoc` (BFee registers if the AP is a beamformer) + BF monitor; (6) ER-SU capability; `btc CONN_END`; HTC template; UL-TB assoc; beacon-track assoc; (7) H2C **GENERAL_PKT** (class FW_INFO 0, func 1: PS-poll/null/QoS-null templates); (8) H2C **CFG_BCNFLTR** (enable); (9) `dig_resume`; `assoc_link_set`. `total_sta_assoc++`. Then the BSS assoc part (§4.4) runs, including the port update to INFRA. |
| ASSOC → AUTH | **`__rtw89_ops_sta_disassoc`**: `assoc_link_clr`; station: H2C beacon filter off; beacon-track reset. `rtwsta->disassoc = true`; `total_sta_assoc--`. |
| AUTH → NONE | **`__rtw89_ops_sta_disconnect`**: free pending BA / forbid-BA / ROC TX; then `rtw89_core_sta_link_disconnect` (core.c:5739): BF monitor off + `bf_disassoc` (clear BFee enable); station: `vif_type_mapping(false)` → NO_LINK, release the general-pkt list; H2C assoc CMAC table; H2C JOININFO(dis_conn = true); H2C ADDR_CAM_UPD(CON_DISCONN). **The port registers stay INFRA until the next `port_update`.** |
| NONE → NOTEXIST | **`__rtw89_ops_sta_remove`**: `rtw89_core_sta_link_remove` (6 GHz recalc, `btc DIS_CONN`); unset the link. The station keeps the vif's mac_id. |

Typical mac80211 order for a connect (emulated chanctx):

1. `.config(CHANGE_CHANNEL, operating chandef)` when the link starts using the
   channel.
2. `link_info_changed(BSSID)`.
3. `sta_state NOTEXIST→NONE` → **RFK runs here, on the operating channel**.
4. NONE→AUTH.
5. Auth/assoc management frames go out.
6. AUTH→ASSOC.
7. `vif_cfg_changed(ASSOC)` → assoc H2Cs, RA, port INFRA.
8. `conf_tx` for each AC.
9. `set_key` (after the 4-way handshake).
10. `ampdu_action`.

### 4.6 conf_tx → EDCA (mac80211.c:385-485, fw.c:5196-5230)

EDCA is **not** written to MAC registers by the driver. rtw89 sends one H2C per AC:
cat MAC(1), class FW_OFLD(0x9), func USR_EDCA(0xF), length 12 (3 dwords).

| Dword | Bits | Field |
|---|---|---|
| 0 | [1:0] | SEL = 0 |
| 0 | [3] | BAND = mac_idx (0) |
| 0 | [4] | WMM = 0 |
| 0 | [6:5] | AC = firmware index: BE 0, BK 1, VI 2, VO 3 (`ac_to_fw_idx`) |
| 1 | [26:16] | TXOP (in 32 µs units, as `params->txop`) |
| 1 | [15:12] | ECWmax = ilog2(cw_max + 1) |
| 1 | [11:8] | ECWmin = ilog2(cw_min + 1) |
| 1 | [7:0] | AIFS in µs = aifsn × slot + SIFS |
| 2 | — | 0 |

slot = 9 if `bss_conf->use_short_slot`, else 20; SIFS = 10 in 2.4 GHz, 16 in 5 GHz.
AIFS is recomputed on ERP_SLOT changes.

MU-EDCA (only if `params->mu_edca`, i.e. an HE AP sends an MU-EDCA IE):
- write32 to 0xC350 (BE), 0xC354 (BK), 0xC358 (VI) or 0xC35C (VO):
  - bits[31:16] = `mu_edca_timer << 8`;
  - bits[15:8] = `ecw_min_max` byte from the IE;
  - bits[7:0] = AIFS µs (0 if aifsn = 0);
- then write16_set 0xC370 |= BIT0 | BIT4 (mac.c:7152).

### 4.7 Scanning: HW scan by default; what SW scan needs

- On the reference system rtw89 implements `hw_scan` and the firmware has SCAN_OFFLOAD, so
  mac80211 uses **firmware scan offload** (`rtw89_hw_scan_start` +
  `rtw89_hw_scan_offload`; mac80211.c:1279-1320).
- `sw_scan_*` would only be used if `hw_scan` returned 1 (no SCAN_OFFLOAD).
- A driver that does not implement `hw_scan` gets mac80211 software scan. mac80211
  then calls:
  - `sw_scan_start`;
  - `configure_filter` with `FIF_BCN_PRBRESP_PROMISC`;
  - `flush`;
  - `.config(CHANGE_CHANNEL)` for each channel (20 MHz NO_HT chandef);
  - `.config` back to the operating channel;
  - `sw_scan_complete`.
- mac80211 also sets default `max_scan_ssids`/`max_scan_ie_len` for SW-scan drivers.
  This is recalled from `net/mac80211/main.c`, which is not in the reference tree.

What rtw89 does around a SW scan (`rtw89_core_scan_start/complete`,
core.c:6969-7028):

| Step | Start | Complete |
|---|---|---|
| 1 | `rtwdev->scanning = true` (RX uses the DS-param IE channel for beacons/probe responses, core.c:3785) | restore `mac_addr = bss_conf->addr`; H2C ADDR_CAM_UPD(INFO_CHANGE) |
| 2 | copy the scan MAC address (random-MAC support) into the link | `rfk_scan(false)` |
| 3 | `btc_ntfy_scan_start` | `btc_ntfy_scan_finish` |
| 4 | **`rfk_scan(start)`** → TSSI default TX AGC on (rtw8852b_rfk.c:3933) | EDCCA restore |
| 5 | recalc PCI interrupt mitigation | TAS (no-op) |
| 6 | **EDCCA to max**: BB 0x4884 fields [7:0], [15:8], [31:24] = 249, backing up the old values (phy.c:8569) | `scanning = false` |
| 7 | TAS (no-op) | set `bypass_dig` |
| 8 | H2C ADDR_CAM_UPD(INFO_CHANGE) with the scan MAC | — |

Minimal SW-scan hooks:
- start: set `scanning`; optionally raise EDCCA to max and switch TSSI to default TX
  AGC.
- complete: undo those.
- Skip random-MAC scanning (do not set `NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR`) so no
  CAM update is needed.
- Each channel hop is an ordinary `set_channel` (no RFK).

### 4.8 Small ops

- **RTS threshold** (mac.c:6314-6342), register **0xC614** (R_AX_AGG_LEN_HT_0), 16-bit
  RMW:

  | Case | bits[15:8] (time, 32 µs units) | bits[7:0] (length, 16-byte units) |
  |---|---|---|
  | `wiphy->rts_threshold == (u32)-1` | 88 >> 5 = 2 | 4080 >> 4 = 255 |
  | otherwise | 255 | min(rts_threshold >> 4, 255) |

  Called at start and from `set_rts_threshold` when POWERON.
- **flush**: see the table in §4. The TX-queue-empty checks use
  `wde_qempty_*` = 4 (track 07).
- **set_key** (cam.c:465-535):
  - Supported in hardware: WEP40/104, CCMP, CCMP-256, GCMP, GCMP-256, AES-CMAC
    (BIP).
  - TKIP and anything else → `-EOPNOTSUPP` (mac80211 then uses SW crypto for that
    key).
  - Always sets `GENERATE_IV` (`hw_sec_hdr = false`), `SW_MGMT_TX` for
    CCMP/GCMP, and `GENERATE_MMIC` for TKIP.
  - DISABLE_KEY first flushes `txq_work` and the HCI/MAC TX queues.
  - The RX path marks a frame `RX_FLAG_DECRYPTED` only when the RX descriptor says
    `hw_dec && !sw_dec && !icv_err` (core.c:4362-4364).
- **get/set_antenna**: `available_antennas_tx/rx = 0x3`; values stored only.
- **sta_statistics**: `sinfo->txrate` comes from the RA report.
- **link_sta_rc_update**: `rtw89_phy_ra_update_sta_link(changed)` → RA H2C.
- **reconfig_complete**: SER only.

### 4.9 ampdu_action and BA (mac80211.c:1026-1076, core.c:4564-4700)

| Action | rtw89 behaviour |
|---|---|
| TX_START | return `IEEE80211_AMPDU_TX_START_IMMEDIATE` |
| TX_OPERATIONAL | set `RTW89_TXQ_F_AMPDU`; store `buf_size` / `amsdu`; `ampdu_cmac_tbl` (NULL on 8852B); recalc the RA aggregation limit |
| TX_STOP_* | clear the flags; `ieee80211_stop_tx_ba_cb_irqsafe` |
| RX_START / RX_STOP | H2C **BA_CAM** (cat 1, class 0xC, func 0): mac_id, entry index (static entries 0..1), valid, TID, bitmap size (0 for ≤64, 4 for >64), `init_req = 1` |

If both static entries are in use (only TID 0 or 5 may take one over), the H2C is
skipped and the hardware creates a dynamic BA CAM entry automatically (fw.c:2558-2571,
core.c:5522-5566). RX reordering is done by mac80211 (no REORDERING_BUFFER flag).

**TX BA sessions are started by the driver.** In the txq path,
`rtw89_core_txq_check_agg` queues `ba_work`, which calls
`ieee80211_start_tx_ba_session(sta, tid, 0)` for any QoS TID with a station, except:
- EAPOL frames, which stop BA and forbid it for 4 s (`forbid_ba_work`);
- TIDs that already failed with `-EINVAL` (`BLOCK_BA`).

---------------------------------------------------------------------------------

## 5. ieee80211_hw and wiphy advertisement (core.c:7357-7498)

### 5.1 Hardware fields

| Field | Value |
|---|---|
| vif_data_size / sta_data_size | `struct_size(rtw89_vif/sta, links_inst, 1)` |
| txq_data_size | `sizeof(struct rtw89_txq)` |
| chanctx_data_size | `sizeof(struct rtw89_chanctx_cfg)` |
| perm_addr | efuse logical 0x400 |
| extra_tx_headroom | IEEE80211_HT_CTL_LEN = 4 (PCIe: the TX WD lives in a separate DMA buffer, so no descriptor headroom) |
| queues | IEEE80211_NUM_ACS = 4 |
| max_rx_aggregation_subframes | 64 |
| max_tx_aggregation_subframes | 128 |
| uapsd_max_sp_len | IEEE80211_WMM_IE_STA_QOSINFO_SP_ALL (0) |
| radiotap_mcs_details | \|= HAVE_FEC \| HAVE_STBC |
| radiotap_vht_details | \|= KNOWN_STBC \| KNOWN_BEAMFORMED |

`ieee80211_hw_set` flags (core.c:7388-7411):

| Flag | Condition | Minimal driver |
|---|---|---|
| SIGNAL_DBM | always | keep |
| HAS_RATE_CONTROL | always (FW RA) | keep from day 1; use a fixed descriptor rate until RA is implemented |
| MFP_CAPABLE | always | keep (works with SW crypto) |
| REPORTS_TX_ACK_STATUS | always | keep once RPP TX status works (track 07) |
| AMPDU_AGGREGATION | always | phase 5 |
| RX_INCLUDES_FCS | always | keep (the RX buffer includes the FCS) |
| TX_AMSDU | always | omit initially |
| SUPPORT_FAST_XMIT | always | optional |
| SUPPORTS_AMSDU_IN_AMPDU | always | optional |
| SUPPORTS_PS, SUPPORTS_DYNAMIC_PS | always | **omit** (no PS) |
| SINGLE_SCAN_ON_ALL_BANDS | always | HW-scan only; omit |
| SUPPORTS_MULTI_BSSID | always | optional |
| WANT_MONITOR_VIF | always | optional |
| CHANCTX_STA_CSA | always | optional |
| SUPPORTS_VHT_EXT_NSS_BW | only with 160 MHz | not set on 8852B |
| CONNECTION_MONITOR | FW BEACON_FILTER — **set on this FW** | omit (mac80211 monitors the connection) |
| AP_LINK_PS | FW NOTIFY_AP_INFO | not set |

### 5.2 wiphy fields

| Field | Value | Minimal driver |
|---|---|---|
| interface_modes | STATION \| AP \| P2P_CLIENT \| P2P_GO (mac80211 adds MONITOR itself) | STATION |
| iface_combinations (core.c:192-205) | [0] limits {STA ×1}, {P2P_CLIENT\|P2P_GO\|AP ×1}; max_interfaces 2; channels 1. [1] {STA ×1}, {P2P_CLIENT\|P2P_GO ×1}; max 2; channels 2. n = 2 on this FW (1 without chanctx). | omit |
| available_antennas_tx/rx | 0x3 (`BIT(rf_path_num) - 1`) | 0 (or 0x3 with get/set ops) |
| flags | SUPPORTS_TDLS \| TDLS_EXTERNAL_SETUP \| AP_UAPSD \| HAS_CHANNEL_SWITCH \| SUPPORTS_EXT_KEK_KCK \| SPLIT_SCAN_6GHZ | none. rtw89 does not clear `WIPHY_FLAG_PS_ON_BY_DEFAULT`, so with SUPPORTS_PS, cfg80211 turns PS on. |
| features | NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR | none |
| max_scan_ssids / max_scan_ie_len | 8 / 512 (FW scan offload limits) | leave to mac80211 (SW scan) |
| wowlan, max_sched_scan_ssids | stub / 8 (CONFIG_PM) | none |
| tid_config_support | AMPDU_CTRL, AMSDU_CTRL (vif and peer) | none |
| max_remain_on_channel_duration | 1000 ms | none |
| ext features | CAN_REPLACE_PTK0, SCAN_RANDOM_SN, SET_SCAN_DWELL | CAN_REPLACE_PTK0 once HW keys work |
| iftype_ext_capab | MLO only (not set) | — |
| sar_capa | `rtw89_sar_capa` | none |
| cipher_suites | not set → mac80211 default list | same |
| reg_notifier | `rtw89_regd_notifier` (§6) | keep |
| vif `driver_flags` (add_interface) | BEACON_FILTER \| SUPPORTS_CQM_RSSI (FW BEACON_FILTER) | none |

### 5.3 Bands, channels, bitrates (core.c:39-89, 158-171, 292-320)

**2.4 GHz** (14 channels): 2412…2472 in 5 MHz steps (channels 1-13) plus 2484
(channel 14). `hw_value` = channel number, flags 0.

**5 GHz** (28 channels), in this order:
- 5180-5320 (36-64);
- 5500-5720 (100-144);
- 5745-5825 (149-165; 165 carries `IEEE80211_CHAN_NO_HT40MINUS`);
- 5845/5865/5885 (169/173/177 = UNII-4, indexes 25-27).

**6 GHz**: not registered (8852B has no 6 GHz support).

**Bitrates** (`rtw89_bitrates`):

| Rate (100 kbps units) | 10 | 20 | 55 | 110 | 60 | 90 | 120 | 180 | 240 | 360 | 480 | 540 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hw_value | 0x00 | 0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 | 0x07 | 0x08 | 0x09 | 0x0A | 0x0B |

No `IEEE80211_RATE_SHORT_PREAMBLE` flags. 5 GHz uses entries 4..11 only.

The sband structures are `devm_kmemdup`-ed per device, so policy code can write
channel flags (core.c:6303-6325).

### 5.4 HT/VHT/HE capabilities for 8852B 2T2R (hal.tx_nss = rx_nss = 2)

Values were computed from the upstream `ieee80211-{ht,vht,he}.h` constants. Build them
directly.

**HT** (both bands; core.c:5976-6003):

| Field | Value | Decoding |
|---|---|---|
| ht_supported | true | |
| cap | **0x19E3** | LDPC 0x0001, SUP_WIDTH_20_40 0x0002, SGI_20 0x0020, SGI_40 0x0040, TX_STBC 0x0080, RX_STBC = 1 stream (0x0100), MAX_AMSDU (7935) 0x0800, DSSSCCK40 0x1000. SM_PS field 0 (mac80211 rewrites it). |
| ampdu_factor | 3 (64 KiB) | |
| ampdu_density | 0 (no restriction) | |
| mcs.rx_mask | {0xFF, 0xFF, 0, 0, 0x01 (MCS32), 0…} | |
| mcs.rx_highest | 300 | |
| mcs.tx_params | 0x01 (TX_DEFINED) | |

**VHT** (5 GHz only; core.c:6005-6056):

| Field | Value | Decoding |
|---|---|---|
| vht_supported | true | |
| cap | **0x03D071B2** | MAX_MPDU_11454 (2), RXLDPC 0x10, SHORT_GI_80 0x20, TXSTBC 0x80, RXSTBC_1 0x100, SU_BFEE 0x1000, BFEE_STS = 3 (<<13 → 0x6000), MU_BFEE 0x100000, HTC_VHT 0x400000, MAX_A_MPDU_LEN_EXP = 7 (0x3800000) |
| rx_mcs_map / tx_mcs_map | 0xFFFA | NSS1-2 MCS0-9, others not supported |
| rx_highest / tx_highest | 867 / 867 | no EXT_NSS_BW bit (no 160 MHz) |

**HE** — `iftype_data` has 2 entries per band, `types_mask` BIT(STATION) and
BIT(AP) (core.c:6263-6300). STATION entry (core.c:6058-6166):

| Field | 2.4 GHz | 5 GHz |
|---|---|---|
| mac_cap_info[0..5] | 01 08 0A 10 60 80 | same |
| phy_cap_info[0..10] | 02 70 1E 1F 0D C0 27 0E 91 BD 00 | 04 70 1E 1F 0D C0 27 0E 91 BD 00 |
| he_mcs_nss_supp rx_mcs_80 / tx_mcs_80 | 0xFFFA | 0xFFFA |
| he_mcs_nss_supp 160 / 80p80 | 0 | 0 |
| PPE thresholds | none | none |
| he_6ghz_capa | n/a | n/a |

Decoding of the STATION values:
- MAC0 HTC_HE. MAC1 TF_MAC_PAD_DUR_16US. MAC2 ALL_ACK \| BSR. MAC3
  MAX_AMPDU_LEN_EXP_EXT_2. MAC4 OPS \| AMSDU_IN_AMPDU. MAC5 HT_VHT_TRIG_FRAME_RX.
- PHY0: 40 MHz in 2.4 GHz (0x02) / 40+80 MHz in 5 GHz (0x04).
- PHY1: DEVICE_CLASS_A \| LDPC \| HE_LTF_AND_GI_0_8US.
- PHY2: NDP_4x_LTF_3_2US \| STBC_TX<80 \| STBC_RX<80 \| DOPPLER_TX.
- PHY3: DCM_MAX_CONST_RX 16-QAM \| DCM_MAX_CONST_TX 16-QAM \| DCM_MAX_TX_NSS_2.
- PHY4: SU_BEAMFORMEE \| BFEE_MAX_STS<80 = 4.
- PHY5: NG16_SU \| NG16_MU (cleared only for 8852B CAV).
- PHY6: CODEBOOK_42_SU \| CODEBOOK_75_MU \| TRIG_SU_BF_FB \| PARTIAL_BW_EXT_RANGE.
- PHY7: POWER_BOOST_FACTOR \| SU_MU_PPDU_4XLTF_08US_GI \| MAX_NC_1.
- PHY8: ER_SU_PPDU_4XLTF_08US_GI \| ER_SU_1XLTF_08US_GI \| DCM_MAX_RU_996.
- PHY9: LONGER_THAN_16_SIGB \| TX_1024QAM_<242 \| RX_1024QAM_<242 \|
  RX_FULL_BW_SU_MU_COMP_SIGB \| RX_FULL_BW_SU_MU_NON_COMP_SIGB \| nominal packet
  padding 16 µs (2 << 6).

AP entry (reference only):
- mac: `01 00 0A 12 60 00`;
- phy[3] = 0x58;
- phy[9] = 0xB9.

EHT: none (AX chip).

**Minimal-driver caution.** The advertised SU/MU-beamformee bits (VHT
0x1000/0x100000, HE PHY4 bit0) make the AP sound us. rtw89 answers because of the
beamformee setup in `rtw89_mac_bf_assoc_ax` (mac.c:6880; registers around 0xCD80).
Until that is implemented, clear the BFee bits: VHT cap becomes 0x03C001B2 after also
removing STS; HE PHY4 = 0. Also consider dropping the HE ER-SU, DCM and trigger bits
until HE TB is validated.

---------------------------------------------------------------------------------

## 6. Regulatory (regd.c)

rtw89 does not use a custom regdomain: no `wiphy_apply_custom_regulatory` and no
`REGULATORY_CUSTOM_REG`. All channels are registered enabled, and cfg80211's
regdomain gates them.

1. **`rtw89_regd_setup()`**, before `ieee80211_register_hw` (regd.c:709-736):
   - Country→regulation map: the firmware-file **REGD element** if present
     (`elm_info.regd`), otherwise the built-in `rtw89_regd_map[]`
     (regd.c:24-305). Entries have the form
     `COUNTRY_REGD(alpha2, 2G, 5G, 6G, func_bitmap)`, e.g.
     - `"US", FCC, FCC, FCC, 0x1` (TAS);
     - `"DE", ETSI, ETSI, ETSI, 0x2` (DAG).
   - Unknown or "00" → worldwide `rtw89_ww_regd` = WW for every band.
   - ACPI DSM policy: `txpwr_uk_follow_etsi` and the UNII-4 enable; 6 GHz (n/a).
   - **UNII-4**: every country is blocked by default. If the ACPI DSM cannot be
     evaluated, the value defaults to "US allowed" (regd.c:359-409).
   - Sets `wiphy->reg_notifier = rtw89_regd_notifier`.
2. **`rtw89_regd_init_hint()`**, after `register_hw` (regd.c:738-768). The efuse
   country code (logical 0x2CB-0x2CC) is looked up in the map:
   - If found (programmed chip): `regulatory.regd = entry`, `programmed = true`,
     set `REGULATORY_COUNTRY_IE_IGNORE | REGULATORY_STRICT_REG`, and call
     **`regulatory_hint(wiphy, alpha2)`**.
   - Otherwise: worldwide roaming chip; follow the stack.
3. **`rtw89_regd_notifier()`** (regd.c:899-927), which takes the wiphy lock itself:
   1. Unless `programmed`: `regd = map lookup(request->alpha2)`. If the initiator is
      USER and the result is not WW, set COUNTRY_IE_IGNORE; otherwise clear it.
   2. Apply the policies:
      - UNII-4: disable channel indexes 25-27 if blocked (regd.c:770-797);
      - 6 GHz: n/a;
      - TAS: n/a;
      - antenna-gain `block_country` = !DAG bit.
   3. **`rtw89_core_set_chip_txpwr()`** → `chip->ops->set_txpwr(chan0, PHY_0)`,
      only if the entity is active (core.c:475-487).
4. **Hook into TX power**: `rtw89_regd_get(rtwdev, band)` =
   `regulatory.regd->txpwr_regd[band]`, with UK→ETSI when `txpwr_uk_follow_etsi`
   (core.h:7779-7789). phy.c uses it as the regulation index into the TX-power limit
   and RU-limit tables (phy.c:2356, 2586, 2871). The tables themselves come from the
   FW TXPWR elements via `rfe_parms` (track 05). Regulation enum (core.h:771-788):

   | Name | Value | Name | Value | Name | Value | Name | Value |
   |---|---|---|---|---|---|---|---|
   | WW | 0 | NA | 4 | NCC | 8 | CN | 12 |
   | ETSI | 1 | IC | 5 | MEXICO | 9 | QATAR | 13 |
   | FCC | 2 | KCC | 6 | CHILE | 10 | UK | 14 |
   | MKK | 3 | ACMA | 7 | UKRAINE | 11 | THAILAND | 15 |

5. `regulatory.regd` is NULL until the first notifier call. cfg80211 calls the
   notifier during `wiphy_register`, so it is set before `.start`. A new driver
   should initialise it to WW explicitly.

**Minimal driver:**
- Keep `reg_notifier`: store alpha2 → regulation index (FW REGD element or a small
  built-in table), then re-apply TX power if the device is running.
- Optionally `regulatory_hint` the efuse country.
- Disable or omit channels 169/173/177.

---------------------------------------------------------------------------------

## 7. Deferred work, timers, locking

### 7.1 Work items (created in core.c:6868-6947)

| Item | Type / queue | Trigger → content | Cancelled in stop | Minimal driver |
|---|---|---|---|---|
| **track_work** | wiphy delayed, **2 s** (`round_jiffies_relative(2*HZ)`, core.h:51) | re-arms itself while RUNNING. Content (core.c:5424-5470): traffic stats + H2C TP offload per vif. If not scanning: leave LPS; on traffic change recalc interrupt mitigation and `btc_ntfy_wl_sta`; BF monitor; beacon track; **phy_stat_track** (RSSI); env monitor (CCX/NHM); **DIG**; **rfk_track** (DPK thermal tracking; skipped in MCC); **RA update**; **CFO track**; TX path div; antdiv; UL-TB ctrl; **EDCCA track**; SAR; chanctx (MCC); rfkill poll; MLO; LPS entry decision | yes | keep one 2 s watchdog: stats → DIG → CFO → DPK track (phase 6) |
| track_ps_work | wiphy delayed, 100 ms | LPS traffic tracking only | yes | omit |
| txq_work | `rtw89_tx_wq` (unbound, high priority) | wake_tx_queue → per-AC `ieee80211_next_txq` loop, TX-resource check, aggregation wait, BA check, push, kick | via reinvoke cancel | replace with `ieee80211_handle_wake_tx_queue` initially |
| txq_reinvoke_work | delayed (1 jiffy) on tx_wq | re-run txq_work | yes (sync) | — |
| ba_work | `ieee80211_queue_work` | `ieee80211_start_tx_ba_session` for queued TIDs | no | phase 5 (can be a wiphy_work) |
| forbid_ba_work | mac80211 delayed wq, 4 s | re-allow BA after EAPOL | yes | optional |
| **c2h_work** | wiphy work | non-atomic C2H events queued from RX (fw.c:8082-8152) | yes | keep |
| ips_work | wiphy work | enter IPS after an HW scan | no | omit |
| chanctx_work | wiphy delayed | MCC | yes | omit |
| cfo_track_work | wiphy delayed | CFO | yes | phase 6 |
| coex_act1 / bt_devinfo / rfk_chk | wiphy delayed | coex | yes | omit (§9) |
| btc eapol/arp/dhcp/icmp notify | wiphy work | coex hints | yes | omit |
| tx_wait_work | wiphy delayed | reap waited TX (nullfunc) | yes | omit |
| antdiv_work, mcc_prepare_done_work, cancel_6ghz_probe_work | — | n/a | yes | omit |
| load_firmware_work | system wq | completes the FW request | — | omit (request firmware synchronously) |
| ser_hdl_work, ser_alarm_work | ser.c:796 | SER state machine | — | omit |
| per-vif update_beacon_work / csa_beacon_work / roc_work / mcc_gc_detect | wiphy | AP/MCC/ROC | on remove | omit |

C2H dispatch (fw.c:8082-8095, mac.c:6159-6201) is split by context:

- **Handled in RX (NAPI) context**, because they complete waiters:
  - MAC class INFO: REC_ACK, DONE_ACK;
  - OFLD: PKT_OFLD_RSP (SCANOFLD_RSP is peeked and then queued);
  - MCC, MISC, MLO, MRC and WOW classes;
  - AP PWR_INT_NOTIFY;
  - some PHY (OUTSRC) C2H.
- **Everything else** is queued to `c2h_work` and handled under the wiphy lock, and
  only if RUNNING.

### 7.2 Locking model in rtw89

- **wiphy mutex** (mac80211's `wiphy->mtx`) is the main lock. All control ops assert
  it. All periodic and state work is `wiphy_work`/`wiphy_delayed_work`, so it is
  serialized with the ops without a private mutex. The regd notifier takes
  `wiphy_lock` itself.
- **Spinlocks**:
  - `ba_lock`: BA lists;
  - `rpwm_lock`: PS RPWM;
  - `tx_rpt.skb_lock`: TX report skbs;
  - PCI `trx_lock`: TX/RX rings;
  - PCI `irq_lock`: interrupt mask/ISR, also taken in the hard IRQ;
  - `c2h_queue.lock`.
- **TX path**: `.tx` and `.wake_tx_queue` run in atomic or softirq context. The TX
  data path runs in `txq_work` on its own workqueue, outside the wiphy lock, and
  protects the rings with `trx_lock`.
- **RX path**: threaded IRQ → NAPI poll (softirq) → `ieee80211_rx_napi`. C2H frames
  arrive through the RX ring.
- Flags such as RUNNING, POWERON and FW_RDY are atomic bitops used as guards.

### 7.3 Recommended model for a single-vif station driver

1. **One control lock**: rely on the wiphy mutex held by mac80211 for every control
   op. Use `wiphy_work`/`wiphy_delayed_work` for the 2 s watchdog and for C2H event
   handling. Do not add a driver mutex.
2. **Two spinlocks**:
   - `tx_lock` (`_bh`) covers the data TX rings and the H2C ring (CH12). H2C may be
     sent from wiphy-locked process context, so `spin_lock_bh` is enough.
   - `irq_lock` (`irqsave`) covers the HIMR/HISR registers shared by the hard IRQ,
     the threaded handler, and enable/disable.
3. **RX in NAPI.** C2H acknowledgements (REC_ACK/DONE_ACK, and any C2H that a
   blocking waiter depends on) must be completed from NAPI. Never block under the
   wiphy mutex waiting for a C2H that is only delivered through `c2h_work`: that work
   needs the same mutex and would deadlock.
4. **Register-mailbox H2C/C2H** (used for phycap at probe) is polled; it is not
   interrupt-driven.
5. Start with **mac80211's `ieee80211_handle_wake_tx_queue`**, which calls your
   `.tx`. Move to a custom `txq_work` (for TX-resource back-pressure) when ring-full
   drops become an issue.
6. No IPS and no LPS: the chip is powered exactly between `.start` and `.stop`.

---------------------------------------------------------------------------------

## 8. Power save and SER: what can be skipped

**Power-save layers in rtw89:**

| Layer | Trigger | With PS disabled | Minimal driver |
|---|---|---|---|
| **IPS** (inactive PS) | `CONF_IDLE` → full `core_stop`/`core_start` (ps.c:233-278) | **not** affected by `disable_ps_mode`; active on the reference system | skip |
| **LPS** (802.11 legacy PS via FW) | `rtw89_recalc_lps` sets `lps_enabled` when the only vif is STATION with `vif->cfg.ps`; `track_ps_work`/`track_work` then call `rtw89_enter_lps` → H2C LPS_PARM (class MAC_PS 0x2, func 0x0) + RF PS info + LPS ML info (ps.c:130-199, 362-393) | still used unless userspace turns power_save off, because rtw89 sets SUPPORTS_PS and leaves PS_ON_BY_DEFAULT | skip (do not set SUPPORTS_PS) |
| **Deep PS** (clock/power gating via RPWM/CPWM, `__rtw89_enter_ps_mode`) | runs only if `ps_mode ≠ NONE` | forced NONE by `disable_ps_mode=Y` | skip |

`rtw89_leave_ps_mode()` and the leave-LPS calls sprinkled through the ops can all be
dropped.

**SER** (system error recovery, ser.c). `rtw89_ser_notify()` (ser.c:817) is called
from:
- the PCI threaded IRQ on `B_AX_HALT_C2H_INT_EN` in the halt-C2H ISR group (reason
  read from R_AX_HALT_C2H 0x016C after polling R_AX_HALT_C2H_CTRL 0x0164 ≠ 0;
  mac.c:814-831);
- the PCI threaded IRQ on `B_AX_WDT_TIMEOUT_INT_EN` (firmware watchdog) (pci.c:966-971);
- firmware failing to ack an RPWM power-state change (mac.c:1463);
- LPS entry/leave check failures (ps.c:48, 69);
- PCI AER `slot_reset` (pci.c:4663).

L1 events trigger a DMAC reset handshake with the firmware. L2 events trigger a full
restart via `ieee80211_restart_hw` → `reconfig_complete`.

Minimal driver: on halt-C2H or WDT, log R_AX_HALT_C2H and call
`ieee80211_restart_hw()` (mac80211 then re-runs start, add_interface, sta_state, …).
Do not implement the L0/L1 state machine.

---------------------------------------------------------------------------------

## 9. Bluetooth coexistence — not covered by any other track

The BT radio shares the chip. The target has RFE 1, so `ant.num = 2`:
`BTC_ANT_SHARED`, `bt_pos = BTG` (rtw8852b.c:751-800). WL and BT arbitrate
antenna/RF access through the PTA using GNT signals.

rtw89 initialises coex in `.start` step 10:
- `btc_set_rfe`;
- `btc_init_cfg` = `__rtw8852bx_btc_init_cfg` (rtw8852b_common.c:1797-1843);
- scoreboard writes;
- coex H2Cs (`_fw_set_drv_info`, slots, monitor registers);
- `_run_coex`, which picks a policy and sets GNT through `rtw89_mac_cfg_gnt`.

**Register-level part of `btc_init_cfg` for 8852B (RTK PTA mode, inner direction).**
Registers are MAC unless noted.

1. `rtw89_mac_coex_init` (mac.c:6365-6457):
   1. 0x0040 |= BIT5 (ENBT).
   2. 0xDA20 |= BIT1 (PTA_WL_TX_EN).
   3. byte 0xDA35 |= 0x01 (GNT_BT_POLARITY).
   4. byte 0xDA40 |= 0x0C (STATIS_BT_EN \| WL_ACT_MSK).
   5. byte 0xDA42 |= 0x01 (BT_CNT_RST).
   6. byte 0xCC07 &= ~0x02 (RSP_CHK_BTCCA).
   7. 16-bit 0xC340: set BIT5 (BTCCA_EN), clear BIT9 (BTCCA_BRK_TXOP_EN).
   8. LTE-indirect 0x3C (`R_AX_LTE_SW_CFG_2`) &= BIT8 (keeps only WL_RX_CTRL).
   9. 0x0040 bits[7:6] = 0 (BT mode 0/3).
   10. byte 0xDA4C |= BIT0 (RTK_BT_ENABLE).
   11. 0xDA6C bits[5:0] = 5 (sample rate).
   12. byte 0x0041: clear BIT2, set BIT1 (inner direction).
2. 0xDA30 |= BIT3 (TX response = high priority); 0xDA10 |= BIT8 (BCNQ = high
   priority).
3. RF register 0x02 (RR_WLSEL) = 0 on paths A and B (GNT debug off).
4. TRX mask table entries for the shared-antenna case: `rtw8852bx_set_trx_mask` for
   (path A, SS group) = 0x5FF, (B, SS) = 0x5FF, (A, TX group) = 0x5FF,
   (B, TX group) = 0x55F.
5. 0xDA2C = 0xF0FFFFFF (PTA break table).
6. 0xDA40 |= BIT16 \| BIT2.
7. Set `init_ok`.

**LTE-indirect access** (mac.c:86-116):
- wait until 0xDAF3 BIT5 is set;
- write: write 0xDAF4 = value, then 0xDAF0 = 0xC00F0000 | offset;
- read: write 0xDAF0 = 0x800F0000 | offset, then read 0xDAF8.

**WL-only antenna ownership** (what `_set_ant(BTC_ANT_WONLY)` does, coex.c:4540):
- `rtw89_mac_cfg_gnt` with gnt_wl = SW-HI and gnt_bt = SW-LO on both S0 and S1 →
  LTE-indirect **0x38 (`R_AX_LTE_SW_CFG_1`) = 0x77007700**. Per path:
  - GNT_WL SW value and SW control, RFC and BB: S0 0x2000\|0x0200\|0x1000\|0x0100,
    S1 ×0x10000.
  - GNT_BT SW control, value 0: S0 0x4000\|0x0400.
- `rtw89_mac_cfg_ctrl_path(WL)`: byte 0x0073 |= BIT2 (0x70 bit26,
  B_AX_LTE_MUX_CTRL_PATH).

**Minimal driver recommendation:**
- Phase 3+: do the `coex_init` register block, then force WL-only GNT as above.
  WL owns the antenna and BT audio/HID degrades.
- Later: implement the firmware coex protocol (driver-info H2Cs, slot tables,
  `_run_coex` policies).
- Also write the scoreboard 0x00AC WL-active/ON bits so BT firmware knows WL is up.
  The exact bits are in coex.c `_write_scbd`; not extracted here.

---------------------------------------------------------------------------------

## 10. Recommended minimal architecture and bring-up plan

### 10.1 Structure

- `drv_pci.c`: probe/remove, BAR/DMA/rings/IRQ/NAPI (track 01).
- `drv_mac.c`: power on/off, DLE/HFC, sys/trx init, port, RX filter, efuse
  (track 02).
- `drv_fw.c`: firmware file parsing, download, H2C/C2H (DMA and register mailbox)
  (tracks 03/04).
- `drv_phy.c`: BB/RF tables, set_channel, TX power (track 05); `drv_rfk.c` (track 06).
- `drv_txrx.c`: descriptors, TX status (RPP), RX parse (track 07).
- `drv_main.c`: the mac80211 glue in this document.

**ops set:**
- `tx`, `wake_tx_queue` = `ieee80211_handle_wake_tx_queue`;
- `start`, `stop`, `config`, `add_interface`, `remove_interface`,
  `configure_filter`;
- `bss_info_changed` (legacy single-link);
- `sta_state`, `sw_scan_start`, `sw_scan_complete`, `flush`;
- `add/remove/change/switch_vif_chanctx` = `ieee80211_emulate_*`;
- from phase 5: `conf_tx`, `set_key`, `ampdu_action`, `link_sta_rc_update`,
  `set_rts_threshold`.

**hw flags:**
- SIGNAL_DBM, HAS_RATE_CONTROL, MFP_CAPABLE, RX_INCLUDES_FCS,
  REPORTS_TX_ACK_STATUS (once RPP works);
- AMPDU_AGGREGATION (phase 5).
- No PS flags, no CONNECTION_MONITOR, no hw_scan, no chanctx ops, no IPS.

**State:**
- one vif: mac_id 0, port 0, addr-CAM 0, BSSID-CAM 0;
- one AP sta sharing mac_id 0;
- current chandef;
- efuse (MAC address, rfe_type, country, xtal, TSSI/gain);
- hal (cv, acv, nss = 2);
- FW image + element tables;
- regd index.

**Omitted features:** AP/P2P/TDLS/mesh, MCC/MLO, hw_scan, ROC, WoWLAN/PM, SER state
machine, SAR/TAS/ACPI, beacon-filter offload, LPS/IPS/deep PS, BT coex policies,
beamformee (clear the cap bits), HE UL-TB/OFDMA extras, debugfs.

### 10.2 Phases

**Phase 1 — probe, power-on, efuse MAC, registration.**
1. PCI enable, BAR, DMA mask; allocate rings (track 01). Enforce ASPM L1/L1SS/CLKREQ
   off.
2. Read CV from 0x00F0[15:12] and ACV from XTAL-SI 0x41.
3. `pwr_on` (track 02).
4. Read the physical efuse via 0x0030, build the logical map, take
   MAC = logical[0x400..0x405], rfe = [0x2CA], country = [0x2CB..0x2CC]. The phycap
   window at physical 0x580 is needed later.
5. `pwr_off`.
6. Fill hw/wiphy (§5: 2 bands, HT/VHT/HE caps with nss = 2 hard-coded),
   `reg_notifier`, `ieee80211_register_hw`. Implement `.start`/`.stop` as power
   on/off only.
7. Verify: `iw phy` matches rtw89's output; the MAC address matches.

Differences from rtw89:
- The FW phycap query is skipped (nss 2/2 is known for this SKU).
- Efuse is read without the probe-time FW download. rtw89 happens to read it after
  FWDL, but the efuse controller only needs power-on plus the power-cut sequence in
  efuse.c:74-111. This must be verified.

**Phase 2 — firmware download and H2C/C2H.**
- In `.start`: power on → HCI DMA on → DMAC pre-init with DLFW quota → PCI
  `mac_pre_init` → FW download → wait for FW ready (tracks 02/03).
- Implement the H2C DMA ring (CH12) and the H2C/C2H register mailbox
  (0x8140-0x8164).
- Verify with the register-mailbox phycap query
  (`FUNC_GET_FEATURE` → PHY_CAP; expect tx/rx NSS = 2) and with the FW version.

**Phase 3 — MAC/PHY init, RX (monitor), SW scan.**
- Complete `.start` in rtw89 order (§3.1):
  - `enable_bb_rf`, `sys_init`, `trx_init` (SCC quota), PCI `post_init`, OFLD_CFG
    H2C;
  - `reset_bb_rf`, BB table, RF tables, NCTL, `rfk_init` (DACK/RCK/RX-DCK; track 06);
  - coex register init + WL-only GNT (§9);
  - PPDU status 0xCE40 = 0x2B; RTS 0xC614;
  - enable interrupts + NAPI.
- `.config(CHANGE_CHANNEL)` → `set_channel` (track 05) + TX power (track 05; needed
  before any TX).
- `configure_filter` → 0xCE20.
- RX path: RX descriptor parse, PHY status → signal, `rx_status` freq from the
  driver's chandef (track 07).
- Verify: a monitor interface sees beacons on a fixed channel. `iw scan` via
  mac80211 SW scan (sw_scan hooks + channel hopping) lists APs on 2.4 and 5 GHz.

**Phase 4 — management TX and association (open network or SW crypto).**
- `add_interface`: port update (§4.1 table), DMAC/CMAC table init, H2C MACID_PAUSE,
  ROLE_MAINTAIN(CREATE), JOININFO, ADDR_CAM(CREATE), default CCTL.
- `bss_info_changed(BSSID)` → ADDR_CAM(INFO_CHANGE).
- TX:
  - management frames on channel B0MG (8) with a fixed low rate;
  - data frames on ACH0-3;
  - RPP-based TX status (track 07).
- `sta_state`: NOTEXIST→NONE runs `rfk_channel` (IQK/TSSI/DPK; track 06; can be
  deferred to phase 6 at some TX-quality cost).
- `bss_info_changed(ASSOC)`: assoc CCTL, JOININFO(connected), ADDR_CAM(CON_DISCONN,
  with AID), `port_update(INFRA)`. Leave `set_key` NULL (SW CCMP).
- Verify: association and DHCP on WPA2-PSK with SW crypto. Confirm that protected RX
  frames are delivered undecrypted (`sw_dec`); see open questions.

**Phase 5 — data performance.**
- RA: H2C RA MACIDCFG at assoc and `link_sta_rc_update`.
- HW keys: sec CAM H2C, `GENERATE_IV`, `SW_MGMT_TX`.
- EDCA H2C (`conf_tx`).
- AMPDU: `ampdu_action`, driver-initiated TX BA, BA CAM H2C.
- AMSDU/fast-xmit flags; `flush`; RTS.
- Verify: iperf at HT/VHT/HE rates with aggregation.

**Phase 6 — RF quality and dynamic mechanisms.**
- Full RFK on connect (RX-DCK, IQK, TSSI, DPK).
- TX power by regulation (§6) with limit tables.
- Watchdog (2 s): RSSI stats → DIG, CFO track, DPK track, EDCCA.
- BSS color; beamformee (then re-advertise BFee caps); proper BT coex; optionally PS.

---------------------------------------------------------------------------------

## 11. Open questions and uncertainties

1. **Emulated-chanctx contract.** `net/mac80211` is not in the reference tree. The
   claim that mac80211 accepts {emulate add/remove/change/switch + assign/unassign
   NULL} and then drives channels through `.config(CHANGE_CHANNEL)` is inferred from
   rtw89's no_chanctx branch (core.c:7561-7570) and from `rtw89_ops_config`. Verify
   in `net/mac80211/main.c` (the chanctx-ops sanity check) and `chan.c`/`util.c`
   (`ieee80211_emulate_*`) for 7.2.7.
2. **Efuse before firmware.** rtw89 always reads efuse after a probe-time FW
   download. That the efuse block (0x0030) works right after `pwr_on` without FWDL
   or DMAC pre-init is plausible (efuse.c needs only the power-cut writes) but not
   proven.
3. **Protected-frame RX without keys.** It is unclear whether the MAC passes
   protected frames to the host (`sw_dec = 1`) when no sec-CAM entry exists, or drops
   them. This decides whether the all-SW-crypto phase 4 works. The addr-CAM
   `sec_ent_mode = NORMAL` and the CAM H2C layout (track 04) are relevant.
4. **Minimum TX power setup before phase-4 TX.** It is not known what TX power
   results if `set_txpwr` (by-rate/limit registers) has not run. rtw89 always calls it
   inside `set_channel`, so a new driver should too. Track 05 must supply the minimal
   subset.
5. **Coex scoreboard and GNT.** The exact `_write_scbd` bits (0x00AC) and whether BT
   firmware reclaims the antenna when WL uses SW GNT without coex H2Cs were not
   extracted (coex.c is ~8k lines). This needs a follow-up track if BT must keep
   working.
6. **HW scan vs SW scan RX frequency.** The target FW uses HW scan in rtw89, so the
   SW-scan path (`sw_scan_*` + RX freq-from-IE) has not been exercised on this
   hardware with this firmware.
7. **Channel before `add_interface`.** Whether mac80211 issues `.config(~0)`
   (including CHANGE_CHANNEL) right after the first `add_interface` in emulate mode is
   assumed, not verified. The driver should tolerate RX before any channel is set:
   RF is on the table default channel after `.start`.
8. **Secure boot.** `rtw89_mac_dmac/cmac_tbl_init` are skipped if
   `fw.sec.secure_boot` is set (efuse-dependent; track 03). Whether the reference unit is
   secure-boot is unknown.
