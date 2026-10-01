# 01 — PCIe bus / DMA layer of RTL8852BE (resolved for 8852B, gen AX)

Source root (all paths relative to it):
`reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/`

Everything below is resolved for **RTL8852B + PCIe** (`rtw8852be.c` → `rtw8852b_pci_info`,
`rtw89_pci_gen_ax`, `rtw89_pci_isr_ax`, `rtw89_pci_ch_dma_addr_set`, `rtw89_bd_ram_table_single`).
Where rtw89 has chip branches, the 8852B branch is the one listed; branches taken only by
8852A/8852C/8851B/BE chips are noted as "skipped for 8852B". `rtw89_is_rtl885xb()` is TRUE for
8852B (core.h:8045-8053), which matters for many branches.

Conventions: `set(reg, bits)` = read-modify-write OR; `clr(reg, bits)` = RMW AND-NOT;
`mask(reg, M, v)` = RMW replace field M with v (see §2.2). All descriptor fields are little-endian.

---------------------------------------------------------------------------------------------------

## 0. Resolved configuration and machine facts

### 0.1 `rtw8852b_pci_info` (rtw8852be.c:12-70), resolved

| field | value | numeric / meaning |
|---|---|---|
| gen_def | rtw89_pci_gen_ax | pci.c:4696-4717 |
| isr_def | rtw89_pci_isr_ax | pci.c:4685-4693 |
| txbd_trunc_mode / rxbd_trunc_mode | MAC_AX_BD_TRUNC | 1 ("truncated" 8-byte BD format, §4) |
| rxbd_mode | MAC_AX_RXBD_PKT | 0 (packet mode: rxbd_info header in each RX buffer) |
| tag_mode | MAC_AX_TAG_MULTI | 1 |
| tx_burst | MAC_AX_TX_BURST_2048B | 7 |
| rx_burst | MAC_AX_RX_BURST_128B | 3 |
| wd_dma_idle_intvl / wd_dma_act_intvl | MAC_AX_WD_DMA_INTVL_256NS | 1 |
| multi_tag_num | MAC_AX_TAG_NUM_8 | 7 (encoded n-1) |
| lbc_en / lbc_tmr | ENABLE / MAC_AX_LBC_TMR_2MS | 1 / 8 |
| autok_en | DISABLE | 0 — field is never read on AX; auto_refclk_cal is always called with `false` |
| io_rcy_en / io_rcy_tmr | DISABLE / 6MS(72000) | only used by 8852C → unused |
| rx_ring_eq_is_full | false | RX ring "host idx" starts at 0 (§7) |
| check_rx_tag | false | RX tag not validated |
| no_rxbd_fs | false | FS bit in rxbd_info is trusted |
| group_bd_addr | false | per-channel BD base registers (not BE-style grouped) |
| rpp_fmt_size | sizeof(rtw89_pci_rpp_fmt) = 4 | one release report = 1 dword |
| init_cfg_reg | R_AX_PCIE_INIT_CFG1 | 0x1000 |
| txhci_en_bit / rxhci_en_bit | B_AX_TXHCI_EN / B_AX_RXHCI_EN | 0x1000 BIT(11) / BIT(13) |
| rxbd_mode_bit | B_AX_RXBD_MODE | 0x1000 BIT(18) |
| exp_ctrl_reg / max_tag_num_mask | R_AX_PCIE_EXP_CTRL / B_AX_MAX_TAG_NUM | 0x13F0, GENMASK(18,16) |
| rxbd_rwptr_clr_reg | R_AX_RXBD_RWPTR_CLR | 0x1018 |
| txbd_rwptr_clr2_reg | 0 | (CH10/11 clear reg not used) |
| dma_io_stop | {R_AX_PCIE_DMA_STOP1, B_AX_STOP_PCIEIO} | 0x1010 BIT(20) |
| dma_stop1 | {0x1010, B_AX_TX_STOP1_MASK_V1} | mask 0x00070F00 (ACH0-3 bits 8-11, CH8 bit16, CH9 bit17, CH12 bit18) |
| dma_stop2 | {0} | none |
| dma_busy1 | {R_AX_PCIE_DMA_BUSY1, DMA_BUSY1_CHECK_V1} | 0x101C mask 0x00070F00 |
| dma_busy2_reg | 0 | none |
| dma_busy3_reg | R_AX_PCIE_DMA_BUSY1 | 0x101C (RX busy bits 0,1) |
| rpwm_addr / cpwm_addr | R_AX_PCIE_HRPWM / R_AX_CPWM | 0x10C0 / 0x8170 (PS handshake only; PS is off for this user) |
| mit_addr | R_AX_INT_MIT_RX | 0x10D4 |
| wp_sel_addr | 0 | `rtw89_pci_init_wp_16sel` does nothing |
| tx_dma_ch_mask | BIT(ACH4..ACH7) \| BIT(CH10) \| BIT(CH11) | 0x0CF0 — **set bits = channels NOT used** (§3.1) |
| bd_idx_addr_low_power | NULL | no low-power HCI mode (chip `low_power_hci_modes = 0`, rtw8852b.c:1077) |
| dma_addr_set | rtw89_pci_ch_dma_addr_set | pci.c:1093-1114 (non-`_V1` addresses) |
| bd_ram_table | rtw89_bd_ram_table_single | pci.c:1711-1719 |
| ltr_set | rtw89_pci_ltr_set | pci.c:3153-3186 |
| fill_txaddr_info | rtw89_pci_fill_txaddr_info | pci.c:1438-1455 (8-byte addr info) |
| parse_rpp | rtw89_pci_parse_rpp | pci.c:572-582 (1-dword RPP) |
| config_intr_mask / enable_intr / disable_intr / recognize_intrs | rtw89_pci_config_intr_mask / _enable_intr / _disable_intr / _recognize_intrs | the non-suffixed (v0) versions, pci.c:3891, 853, 861, 774 |
| ssid_quirks | NULL | no subsystem-ID quirks |

Driver-info (rtw8852be.c:72-80): `quirks = NULL` (no DMI quirks, so `RTW89_QUIRK_PCI_BER` is never
set — that quirk exists only in rtw8852ce.c:87,96), `dev_id_quirks = 0`. PCI IDs 10ec:b852 and
10ec:b85b (rtw8852be.c:82-92).

Relevant chip_info (rtw8852b.c): `h2c_desc_size = txwd_body_size = sizeof(rtw89_txwd_body) = 24`,
`txwd_info_size = sizeof(rtw89_txwd_info) = 24` (rtw8852b.c:1080-1082, core.h:1092-1130),
`hci_func_en_addr = R_AX_HCI_FUNC_EN (0x8380)` (rtw8852b.c:1079), `small_fifo_size = true`
(rtw8852b.c:973), qsel→channel mapping `rtw89_core_get_ch_dma` (rtw8852b.c:915, core.c:782-817),
RX descriptor parser `rtw89_core_query_rxdesc` (core.c:4070-4123), TX descriptor builder
`rtw89_core_fill_txdesc` for both data and FWCMD (core.c:1600-1621).

### 0.2 Reference system (read-only sysfs, 0000:06:00.0)
- Upstream bridge 0000:00:02.2 = **AMD 1022:14ba** → not Intel and not ASMedia 0x2806 →
  rtw89 does **not** enable 36-bit DAC; the device runs with the default **32-bit DMA mask** (§1.3).
  All BD "DMA_HI" fields are therefore 0 on the reference system.
- Link: 2.5 GT/s (Gen1) x1 → `auto_refclk_cal` uses PCIE_PHY_GEN1 (MDIO pages 0/1).
- BAR0 = I/O ports 0xd000-0xd0ff (unused by rtw89); **BAR2 = 64-bit non-prefetchable MMIO,
  1 MiB** (0xdcb00000-0xdcbfffff) — this is the register window.
- MSI is in use (msi_irqs present, irq 95).

---------------------------------------------------------------------------------------------------

## 1. Probe (`rtw89_pci_probe`, pci.c:4765-4857)

### 1.1 Order
1. `rtw89_alloc_ieee80211_hw(dev, sizeof(struct rtw89_pci), info)`; set `hci.ops = rtw89_pci_ops`
   (pci.c:4720-4763), `hci.type = PCIE`, `dle_type = PCIE`, `rpwm_addr = 0x10C0`, `cpwm_addr = 0x8170`
   (pci.c:4774-4788).
2. `rtw89_check_quirks` (DMI, none) and `rtw89_check_pci_ssid_quirks` (NULL → none) (pci.c:4790-4791).
3. `rtw89_core_init` (software only).
4. `rtw89_pci_claim_device` (pci.c:3270-3288): `pci_enable_device`, `pci_set_master`,
   `pci_set_drvdata(pdev, hw)`.
5. `rtw89_pci_setup_resource` (pci.c:3849-3878):
   a. `rtw89_pci_setup_mapping` — request regions, DMA mask, iomap BAR2 (§1.2, §1.3).
   b. `rtw89_pci_alloc_trx_rings` — TX BD pool, per-channel WD page pools, RX BD pool and 2×256 RX
      buffers (§5).
   c. init `h2c_queue` and `h2c_release_queue`; init `irq_lock`, `trx_lock`.
6. `rtw89_chip_info_setup` (core.c:7279-7333) — **touches hardware without interrupts**: read chip
   version, `rtw89_mac_pwr_on`, wait FW file, `rtw89_chip_efuse_info_setup` (core.c:7180-7205) =
   `rtw89_mac_partial_init(false)` (which runs the full PCI `mac_pre_init` of §9.1 and a complete
   firmware download over CH12, §6.5) → parse efuse → `rtw89_hci_mac_pre_deinit` (§9.3) →
   … → `rtw89_mac_pwr_off`. So after probe the MAC is powered **off**; everything is redone at
   interface up (`rtw89_core_start`).
   Consequence for a new driver: H2C/FW download works purely by polling BD indices; no IRQ needed.
7. `rtw89_pci_basic_cfg(rtwdev, false)` (pci.c:4605-4615) — PCI config-space tweaks (§1.5).
8. `rtw89_core_napi_init` (core.c:4544-4553): dummy netdev + `netif_napi_add(…, rtw89_pci_napi_poll)`
   (default weight = NAPI_POLL_WEIGHT = 64).
9. `rtw89_pci_request_irq` (§1.4); then `rtw89_chip_config_intr_mask(RESET)` (computes masks only;
   IMRs are written only at `hci.start`).
10. `rtw89_core_register` (ieee80211_register_hw); set RTW89_FLAG_PROBE_DONE.

Error unwinding (pci.c:4843-4856): free irq → napi deinit → clear resource → pci_disable_device →
core deinit → free hw.

### 1.2 PCI enable / BAR
- `pci_enable_device` + `pci_set_master` (pci.c:3276-3282).
- `pci_request_regions(pdev, "rtw89_pci")` requests all BARs (pci.c:3364).
- **BAR 2** only: `pci_iomap(pdev, 2, pci_resource_len(pdev, 2))` (pci.c:3361, 3390-3391). All
  registers in this document are offsets into BAR2.

### 1.3 DMA mask (pci.c:3296-3404)
- 8852B is a "manual DAC" chip (`rtw89_pci_chip_is_manual_dac`, pci.c:3296-3309).
- `rtw89_pci_is_dac_compatible_bridge` (pci.c:3311-3332): true only if the upstream bridge vendor is
  Intel, or ASMedia with device 0x2806. Otherwise → skip all mask setting ("try_dac_done") → kernel
  default 32-bit mask.
- If compatible: `dma_set_mask_and_coherent(36 bits)`; on success `rtw89_pci_cfg_dac(force=true)`
  (pci.c:3334-3354): read config byte **0x719**, OR **BIT(5)** (`RTW89_PCIE_BIT_EN_64BITS`), write it
  back — using only the PCI config API (not DBI). On success `enable_dac = true`. If the config
  write fails, fall back to `dma_set_mask_and_coherent(32)`. (If the 36-bit mask call itself fails,
  nothing else is done — default 32-bit stays.)
- On resume `rtw89_pci_cfg_dac(false)` re-applies BIT(5) if `enable_dac` (pci.c:4607-4608).
- **Reference system: AMD bridge → 32-bit, 0x719 BIT(5) untouched.** DMA high bits in BDs = 0.
- BD/addr-info formats carry 8 upper address bits (bits 32-39), so the hardware can address 40 bits
  once DAC is enabled; rtw89 never uses more than 36.

### 1.4 IRQ (pci.c:4056-4093)
- `pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_INTX | PCI_IRQ_MSI)` → one vector, MSI preferred,
  legacy INTx fallback. No MSI-X.
- `devm_request_threaded_irq(dev, pdev->irq, rtw89_pci_interrupt_handler (hard),
  rtw89_pci_interrupt_threadfn (thread), IRQF_SHARED, "rtw89_pci", rtwdev)`.
- Then `rtw89_chip_config_intr_mask(RESET)` → clears low_power/under_recovery and computes mask
  values (§8.2).
- Free: `devm_free_irq` + `pci_free_irq_vectors` (pci.c:4088-4093).

### 1.5 PCI config-space tweaks — `rtw89_pci_basic_cfg(resume=false)` (pci.c:4605-4615)
Executed in this order:
1. `rtw89_pci_disable_eq` → `rtw89_pci_disable_eq_ax` returns immediately unless 8852C
   (pci.c:2585-2586) → **nothing for 8852B**.
2. `rtw89_pci_filter_out` → 8852C only (pci.c:4115-4116) → **nothing**.
3. `rtw89_pci_cpl_timeout_cfg` (pci.c:4380-4397): `pcie_capability_set_word(PCI_EXP_DEVCTL2,
   PCI_EXP_DEVCTL2_COMP_TMOUT_DIS)` — i.e. **set bit 4 of PCIe Device Control 2 (Completion
   Timeout Disable)**. (Only 8922D/CID7090 clears it.) Recommended to replicate.
4. `rtw89_pci_link_cfg` (pci.c:4282-4316): read PCIe Link Control (`PCI_EXP_LNKCTL`):
   - if `PCI_EXP_LNKCTL_CLKREQ_EN` (bit 8) → `rtw89_pci_clkreq_set(true)`; skipped entirely when
     module param `disable_clkreq=Y` (pci.c:4167-4168).
     `rtw89_pci_clkreq_set_ax(true)` (pci.c:4173-4205): write config byte **0x725** =
     `PCIE_CLKDLY_HW_30US` (0x01); then (885xb branch) **set BIT(4)** (`RTW89_PCIE_BIT_CLK`) in config
     byte **0x719**.
   - if `PCI_EXP_LNKCTL_ASPM_L1` (bit 1) → `rtw89_pci_aspm_set(true)`; skipped when
     `disable_aspm_l1=Y` (pci.c:4212-4213).
     `rtw89_pci_aspm_set_ax(true)` (pci.c:4218-4255): read config byte **0x70F**, set bits[5:3]
     (`RTW89_L1DLY_MASK`) = 4 (`PCIE_L1DLY_16US`) and bits[2:0] (`RTW89_L0DLY_MASK`) = 3
     (`PCIE_L0SDLY_4US`), write back; then **set BIT(3)** (`RTW89_PCIE_BIT_L1`) in config byte 0x719.
   - Rationale in comment pci.c:4289-4304: the Realtek-internal ASPM/CLKREQ engine is off by
     default; driver turns it on only if the host enabled the standard link-control bits.
5. `rtw89_pci_l1ss_cfg` (pci.c:4361-4378): skipped when `disable_aspm_l1ss=Y`. Else find ext cap
   L1SS; if `PCI_L1SS_CTL1 & PCI_L1SS_CTL1_L1SS_MASK` (any of the 4 L1.x enables) →
   `rtw89_pci_l1ss_set_ax(true)` (pci.c:4329-4359): **set BIT(5)** (`RTW89_PCIE_BIT_L1SUB`) in config
   byte **0x718**.

rtw89 never calls clkreq/aspm/l1ss *set(false)* in this path — it only ever enables. Standard PCIe
LNKCTL/L1SS registers are left to the PCI core.

**User configuration**: ASPM L1, L1SS and CLKREQ disabled (module params Y). Therefore the only
config-space write rtw89 performs at probe on this system is DEVCTL2.COMP_TMOUT_DIS. A new driver
should: not set 0x719 BIT(3)/BIT(4), 0x718 BIT(5), 0x70F, 0x725. (Optionally also
`pci_disable_link_state(L1 | L1SS | CLKPM)` for belt-and-braces — rtw89 does not do this.)

### 1.6 Realtek vendor config-space registers used (pci.h:1133-1161)

| offset (cfg space) | name | bits used |
|---|---|---|
| 0x80 (dword) | RTW89_PCIE_L1_STS_V1 | bits[19:16] = current link speed (8852C paths only) |
| 0x82 (byte) | RTW89_PCIE_PHY_RATE | bits[1:0]: 1 = Gen1, 2 = Gen2 (read by auto_refclk_cal) |
| 0x70F (byte) | RTW89_PCIE_ASPM_CTRL | [5:3] L1 entry delay, [2:0] L0s entry delay |
| 0x718 (byte) | RTW89_PCIE_TIMER_CTRL | BIT(5) L1SUB enable |
| 0x719 (byte) | RTW89_PCIE_L1_CTRL | BIT(5) EN_64BITS (DAC), BIT(4) CLKREQ, BIT(3) ASPM L1 |
| 0x725 (byte) | RTW89_PCIE_CLK_CTRL | CLKREQ delay (enum rtw89_pcie_clkdly_hw; 0=0, 1=30us…) |
| 0xB48 (byte) | RTW89_PCIE_RST_MSTATE | BIT(0), written twice on resume only (pci.c:4593-4603) |

(0x80/0x82 assume the PCIe capability at 0x70 → Link Control/Status at 0x80/0x82.)

Access path (pci.c:2280-2314): `rtw89_pci_{read,write}_config_byte` first tries the kernel config
API; only if that fails (e.g. no extended config access for offsets ≥ 0x100) and chip is
8852A/885xB, it falls back to the **DBI** indirect path (§2.3).

### 1.7 Filter-out / EQ / BER / quirks
- `rtw89_pci_filter_out`, `rtw89_pci_disable_eq_ax`, `rtw89_pci_deglitch_setting`,
  `rtw89_pci_autoload_hang`, `l12_vmain`, `gen2_force_ib`, `l1_ent_lat`, `wd_exit_l1`,
  `set_io_rcy`: all 8852C (or 8852A) only → skipped.
- `rtw89_pci_ber` requires `RTW89_QUIRK_PCI_BER` (8852CE DMI only) → skipped.
- The PHY tweaks that **do** apply to 8852B (dphy delay, autok_x, refclk-cal disable) are part of
  `mac_pre_init` (§9.1 steps 7-9).

---------------------------------------------------------------------------------------------------

## 2. Register I/O model

### 2.1 MMIO accessors (pci.c:1996-2089)
- All I/O is plain MMIO on BAR2 with `readb/readw/readl/writeb/writew/writel` (so native 8/16/32-bit
  accesses are supported by the device; rtw89 uses 16-bit writes for ring index/length registers and
  8-bit writes for DBI/MDIO).
- For PCIe, `rtwdev->io` is `rtw89_raw_io` (fw.c:11823-11829) → writes go straight to the hci ops.
- **CMAC read workaround**: if `0xC000 <= addr <= 0xFFFF` (`ACCESS_CMAC`, mac.h:587-589;
  R_AX_CMAC_REG_START/END reg.h:2141/3769):
  - 8/16-bit reads are done as an aligned 32-bit read and shifted (pci.c:2018-2044);
  - if the 32-bit value reads `0xDEADBEEF` (RTW89_R32_DEAD), write `R_AX_CK_EN (0xC004) =
    0xFFFFFFFF` (B_AX_CMAC_ALLCKEN) and re-read, up to `MAC_REG_POOL_COUNT = 10` retries
    (pci.c:1998-2016). CMAC clocks may be gated; this re-enables them.
  - Writes have no special handling.
- `read32_pci_cfg` op = `pci_read_config_dword`, returns 0xEAEAEAEA on failure (pci.c:2077-2089).

### 2.2 Read-modify-write helpers (core.h:6999-7121)
- `rtw89_write{8,16,32}_set(addr, bits)`: `v = read(addr); write(addr, v | bits)`.
- `rtw89_write{8,16,32}_clr(addr, bits)`: `write(addr, v & ~bits)`.
- `rtw89_write32_mask(addr, mask, data)` (core.h:7093-7104): `shift = __ffs(mask)`;
  `write(addr, (read(addr) & ~mask) | ((data << shift) & mask))`. **`data` is the field value (not
  pre-shifted).** WARNs if `addr & 3`. `rtw89_write16_mask/8_mask` identical with mask truncated to
  16/8 bits. `rtw89_read32_mask` returns `(read & mask) >> shift`.
- None of these are atomic vs. the hardware; rtw89 relies on single-threaded init plus `irq_lock` /
  `trx_lock` (§8.4).

### 2.3 DBI — indirect access to the device's own PCI config space (pci.c:2233-2278, pci.h:65-72)
Registers: `R_AX_DBI_FLAG = 0x1090` (bits[11:2] B_AX_DBI_ADDR_MSK = dword address, bits[15:12]
B_AX_DBI_WREN_MSK = byte-enable, BIT(16) WFLAG, BIT(17) RFLAG), `R_AX_DBI_WDATA = 0x1094`,
`R_AX_DBI_RDATA = 0x1098`.
- **write8(addr, data)**: `lsb = addr & 3`; `write8(0x1094 + lsb, data)`;
  `write16(0x1090, (addr & 0xFFC) | (BIT(lsb) << 12))`; `write8(0x1092, 0x01)` (WFLAG);
  poll `read8(0x1092) == 0` every 10 µs, timeout 200 µs (atomic).
- **read8(addr)**: `write16(0x1090, addr & 0xFFC)`; `write8(0x1092, 0x02)` (RFLAG); poll
  `read8(0x1092) == 0` (10 µs / 200 µs); value = `read8(0x1098 + (addr & 3))`.

### 2.4 MDIO — PCIe PHY registers (pci.c:2120-2231, pci.h:10-63, 74-75, 177-181)
Registers: `R_AX_MDIO_CFG = 0x10A0` (bits[4:0] PHY reg address, BIT(8) WFLAG, BIT(9) RFLAG,
bits[13:12] PHY page), `R_AX_MDIO_WDATA = 0x10A4` (16-bit), `R_AX_MDIO_RDATA = 0x10A6` (16-bit).
PHY page selection: Gen1: reg < 0x20 → page 0 (MDIO_PG0_G1), else page 1; Gen2: reg < 0x20 → page
2, else page 3. (The `speed` argument chooses the Gen1 or Gen2 register bank, independent of the
current link speed.)
- **access(reg, speed, rw_bit)**: `write8(0x10A0, reg & 0x1F)`; `v = read16(0x10A0)`; replace
  bits[13:12] with page; `write16(0x10A0, v)`; `write16_set(0x10A0, rw_bit)`; poll
  `read16(0x10A0) & rw_bit == 0` every 10 µs, timeout 2000 µs.
- **read16**: access(RFLAG=BIT(9)) then `read16(0x10A6)`.
- **write16**: `write16(0x10A4, data)` **first**, then access(WFLAG=BIT(8)).
- mask/set/clr variants are read16 + modify + write16 (same shift semantics as §2.2).

### 2.5 RAC direct window
`R_RAC_DIRECT_OFFSET_G1 = 0x3800`, `_G2 = 0x3880`; PHY reg N is at `base + N*2` (16-bit). Used only
by 8852C paths and the BER quirk → **not used for 8852B**.

---------------------------------------------------------------------------------------------------

## 3. DMA channel set

### 3.1 Channels and the mask
`enum rtw89_tx_channel` (txrx.h:769-786): ACH0=0 … ACH7=7, CH8=8 (MGMT band0), CH9=9 (HI band0),
CH10=10 (MGMT band1), CH11=11 (HI band1), CH12=12 (FW CMD / H2C). `enum rtw89_rx_channel`
(txrx.h:789-795): RXQ=0, RPQ=1.

`tx_dma_ch_mask` is the set of **disabled / non-existent** TX channels: every loop that sets up,
resets, frees or flushes rings does `if (info->tx_dma_ch_mask & BIT(i)) continue;`
(pci.c:1424, 1795, 1888, 3449, 3660), and an RPP naming a masked channel is rejected (pci.c:607-612).
For 8852B mask = ACH4|ACH5|ACH6|ACH7|CH10|CH11 = 0x0CF0 → **active TX channels: ACH0, ACH1, ACH2,
ACH3, CH8, CH9, CH12 (7 rings)**. 8852B is single-band (no band1 queues).

qsel → channel (`rtw89_core_get_ch_dma`, core.c:782-817): BE_x→ACH0, BK_x→ACH1, VI_x→ACH2,
VO_x→ACH3 (x = 0..3), B0_MGMT(0x12)→CH8, B0_HI(0x11)→CH9, B1_MGMT→CH10, B1_HI→CH11 (the latter two
masked, never used). Data TID→qsel: TID 0,3→BE_0; 1,2→BK_0; 4,5→VI_0; 6,7→VO_0 (txrx.h:833-851).
FWCMD always uses CH12 (`RTW89_DMA_H2C = 12`, core.h:3973-3988).

### 3.2 Per-channel registers (dma_addr_set `rtw89_pci_ch_dma_addr_set`, pci.c:1060-1114;
addresses pci.h:539-669) and BD-RAM programming (pci.c:1711-1719, 1819-1825)

`num` = 16-bit ring length register (B_AX_DESC_NUM_MSK bits[11:0]); `idx` = index register;
`bdram` = BD-RAM control; `desa_l/h` = ring base low/high 32 bits.

| ch | used | num | idx | bdram | desa_l | desa_h | BDRAM start/max/min | BDRAM reg value |
|---|---|---|---|---|---|---|---|---|
| ACH0 | yes | 0x1024 | 0x1058 | 0x1200 | 0x1110 | 0x1114 | 0 / 5 / 2 | 0x00020500 |
| ACH1 | yes | 0x1026 | 0x105C | 0x1204 | 0x1118 | 0x111C | 5 / 5 / 2 | 0x00020505 |
| ACH2 | yes | 0x1028 | 0x1060 | 0x1208 | 0x1120 | 0x1124 | 10 / 5 / 2 | 0x0002050A |
| ACH3 | yes | 0x102A | 0x1064 | 0x120C | 0x1128 | 0x112C | 15 / 5 / 2 | 0x0002050F |
| ACH4 | no | 0x102C | 0x1068 | 0x1210 | 0x1130 | 0x1134 | — | — |
| ACH5 | no | 0x102E | 0x106C | 0x1214 | 0x1138 | 0x113C | — | — |
| ACH6 | no | 0x1030 | 0x1070 | 0x1218 | 0x1140 | 0x1144 | — | — |
| ACH7 | no | 0x1032 | 0x1074 | 0x121C | 0x1148 | 0x114C | — | — |
| CH8 | yes | 0x1034 | 0x1078 | 0x1220 | 0x1150 | 0x1154 | 20 / 4 / 1 | 0x00010414 |
| CH9 | yes | 0x1036 | 0x107C | 0x1224 | 0x1158 | 0x115C | 24 / 4 / 1 | 0x00010418 |
| CH10 | no | 0x1338 | 0x137C | 0x1320 | 0x1358 | 0x135C | — | — |
| CH11 | no | 0x133A | 0x1380 | 0x1324 | 0x1360 | 0x1364 | — | — |
| CH12 | yes | 0x1038 | 0x1080 | 0x1228 | 0x1160 | 0x1164 | 28 / 4 / 1 | 0x0001041C |
| RXQ | yes | 0x1020 | 0x1050 | — | 0x1100 | 0x1104 | — | — |
| RPQ | yes | 0x1022 | 0x1054 | — | 0x1108 | 0x110C | — | — |

BDRAM_CTRL layout (pci.h:669-671): bits[7:0] `BDRAM_SIDX` start index, bits[15:8] `BDRAM_MAX`,
bits[23:16] `BDRAM_MIN`. The single-band table partitions 32 on-chip BD cache entries (0..31):
4 ACs × 5 + 3 × 4. (Interpretation: per-channel share of the on-chip BD prefetch RAM; the hardware
meaning of max/min is not documented in source.) Note the `_V1` register sets (0x1210+…) belong to
8852C and must **not** be used — e.g. 0x1210 is ACH4_BDRAM_CTRL here but RXQ_RXBD_NUM_V1 on 8852C.

### 3.3 Index register format (pci.h:558-559)
Same layout for TX and RX index registers:
- bits[11:0] `TXBD_HOST_IDX_MASK` — host index (written by driver with a **16-bit write** of the
  plain index value to the register address, pci.c:1342, 433, 717).
- bits[27:16] `TXBD_HW_IDX_MASK` — hardware index (read with a 32-bit read, pci.c:94, 153).
Ring lengths are programmed with a 16-bit write of the BD count to `num` (pci.c:1817, 1857).

### 3.4 Global PCIe-HCI control registers (AX, as used for 8852B)

`R_AX_PCIE_INIT_CFG1 = 0x1000` (pci.h:673-688)
| bit | name | use |
|---|---|---|
| 23 | B_AX_PCIE_RXRST_KEEP_REG | set in pre-init (keep regs across RX reset) |
| 22 | B_AX_PCIE_TXRST_KEEP_REG | set in pre-init |
| 21 | B_AX_PCIE_PERST_KEEP_REG | suspend only |
| 20 | B_AX_PCIE_FLR_KEEP_REG | unused |
| 19 | B_AX_PCIE_TRAIN_KEEP_REG | suspend only |
| 18 | B_AX_RXBD_MODE | 0 = packet mode (cleared in pre-init) |
| 16:14 | B_AX_PCIE_MAX_RXDMA_MASK | RX burst = 3 (128 B) |
| 13 | B_AX_RXHCI_EN | RX HCI DMA enable |
| 12 | B_AX_LATENCY_CONTROL | 1 = multi-tag mode |
| 11 | B_AX_TXHCI_EN | TX HCI DMA enable |
| 10:8 | B_AX_PCIE_MAX_TXDMA_MASK | TX burst = 7 (2048 B) |
| 5 | B_AX_TX_TRUNC_MODE | **not written for 8852B** (only 8852A-CBV sets it) |
| 4 | B_AX_RX_TRUNC_MODE | **not written for 8852B** |
| 3 | B_AX_RST_BDRAM | self-clearing BD-RAM reset |
| 2 | B_AX_DIS_RXDMA_PRE | 8852A only |

`R_AX_PCIE_INIT_CFG2 = 0x1004`: bits[27:24] B_AX_WD_ITVL_IDLE, bits[19:16] B_AX_WD_ITVL_ACT,
bits[13:0] B_AX_PCIE_RX_APPLEN_MASK (separate-mode only) (pci.h:895-898).
`R_AX_PCIE_PS_CTRL = 0x1008`: BIT(5) B_AX_L1OFF_PWR_OFF_EN (pci.h:900-901).

`R_AX_PCIE_DMA_STOP1 = 0x1010` (pci.h:693-719): BIT(20) STOP_PCIEIO, BIT(19) STOP_WPDMA,
BIT(18) CH12, BIT(17) CH9, BIT(16) CH8, BIT(15..8) ACH7..ACH0, BIT(1) RPQ, BIT(0) RXQ.
`R_AX_TXBD_RWPTR_CLR1 = 0x1014`: BIT(10) CH12, BIT(9) CH9, BIT(8) CH8, BIT(7..0) ACH7..ACH0
(pci.h:726-738). `R_AX_RXBD_RWPTR_CLR = 0x1018`: BIT(1) RPQ, BIT(0) RXQ (pci.h:740-743).
`R_AX_PCIE_DMA_BUSY1 = 0x101C` (pci.h:750-774): BIT(22) PCIEIO_RX_BUSY, BIT(21) PCIEIO_TX_BUSY,
BIT(20) PCIEIO_BUSY, BIT(19) WPDMA_BUSY, BIT(18) CH12, BIT(17) CH9, BIT(16) CH8, BIT(15..8)
ACH7..0, BIT(1) RPQ, BIT(0) RXQ.
(`R_AX_PCIE_DMA_STOP2 0x1310`, `R_AX_TXBD_RWPTR_CLR2 0x1314`, `R_AX_PCIE_DMA_BUSY2 0x131C` are the
CH10/CH11 counterparts — not used for 8852B.)

`R_AX_PCIE_EXP_CTRL = 0x13F0`: bits[18:16] B_AX_MAX_TAG_NUM, BIT(4) B_AX_SIC_EN_FORCE_CLKREQ,
BIT(20) EN_CHKDSC_NO_RX_STUCK (8852A only); bits[1:0] written by `set_dbg` (§9.1 step 20).
`R_AX_PCIE_DBG_CTRL = 0x11C0`: BIT(1) B_AX_ASFF_FULL_NO_STK, BIT(0) B_AX_EN_STUCK_DBG (reg.h:416-421).
`R_AX_LBC_WATCHDOG = 0x11D8`: bits[7:4] B_AX_LBC_TIMER, BIT(1) B_AX_LBC_FLAG, BIT(0) B_AX_LBC_EN
(pci.h:938-941).
`R_AX_DBG_ERR_FLAG = 0x11C4` (diagnostic, read only by dump/recovery): BIT(29) RPQ_FULL, BIT(28)
RXQ_FULL, [27:25] CPL_STATUS, BIT(22) RX_STUCK, BIT(21) TX_STUCK, BIT(16) TXERR0, BIT(4)
RXP1_ERR0, BIT(1) TXBD_LEN0, BIT(0) TXBD_4KBOUD_LENERR (pci.h:923-932).

MAC-domain registers written by the PCI layer: `R_AX_TX_ADDRESS_INFO_MODE_SETTING = 0x8810`
BIT(0) B_AX_HOST_ADDR_INFO_8B_SEL (reg.h:702-703); `R_AX_PKTIN_SETTING = 0x9A00` BIT(1)
B_AX_WD_ADDR_INFO_LENGTH (reg.h:1791-1792); LTR block 0x8410-0x841C (§9.4).
System registers written by the PCI layer: `R_AX_SYS_PW_CTRL 0x0004` BIT(14) PSUS_OFF_CAPC_EN;
`R_AX_SYS_SDIO_CTRL 0x0070` BIT(15) PCIE_DIS_L2_CTRL_LDO_HCI, BIT(14) PCIE_DIS_WLSUS_AFT_PDN;
`R_AX_HCI_OPT_CTRL 0x0074` BIT(5) BIT_WAKE_CTRL; `R_AX_RSV_CTRL 0x001C` BIT(6) R_DIS_PRST, BIT(5)
WLOCK_1C_BIT6 (suspend/resume only) (reg.h:21-30, 47-49, 120-130).
Related MAC gate: `R_AX_HCI_FUNC_EN = 0x8380` BIT(1) HCI_RXDMA_EN, BIT(0) HCI_TXDMA_EN
(reg.h:545-547), set by `rtw89_mac_partial_init` before PCI pre-init (mac.c:4311, mac.h:1613-1624).

---------------------------------------------------------------------------------------------------

## 4. Descriptor and buffer formats (all little-endian)

"Truncated" (MAC_AX_BD_TRUNC) = 8-byte BDs with a 32-bit address plus 8 high address bits squeezed
into the option word, and 8-byte TX address-info entries (enabled by 0x8810 BIT(0)=1 and 0x9A00
BIT(1)=0, §9.1 step 26 / §9.2).

### 4.1 TX BD — `struct rtw89_pci_tx_bd_32` (8 bytes, pci.h:1465-1471)
| byte off | width | field | bits |
|---|---|---|---|
| 0 | le16 | length | byte count of the buffer the BD points to |
| 2 | le16 | opt | BIT(14) `RTW89_PCI_TXBD_OPT_LS` (last segment; always 1), bits[13:6] `DMA_HI` = address bits 39:32; other bits 0 |
| 4 | le32 | dma | address bits 31:0 |

Data channels: BD points to the **WD page** (length = WD+WP+addr-info bytes, §4.2), not to the
frame. CH12: BD points to the H2C skb (24-byte WD body + payload).

### 4.2 WD page layout for data/mgmt channels (pci.c:1494-1549)
WD page = 128 bytes (`RTW89_PCI_TXWD_PAGE_SIZE`), zeroed whenever it returns to the free list
(pci.h:1719-1728).

| offset | size | content |
|---|---|---|
| 0 | 24 | TX WD body (`rtw89_txwd_body`, 6 dwords; built by `rtw89_core_fill_txdesc`, core.c:1600-1621 — DW0, DW2, DW3 written; DW1/4/5 stay 0) |
| 24 | 24 | TX WD info (`rtw89_txwd_info`), present iff `en_wd_info` (always true for data and mgmt frames, core.c:888, 1112) |
| 48 (or 24) | 8 | WP info (§4.3) |
| 56 (or 32) | 8 | address info #0 (§4.4) |

So for all mgmt/data frames `txwd->len` = 24+24+8+8 = **64 bytes** = TX BD length.
Bus-relevant WD body fields (txrx.h:69-97; full WD semantics are the TX-descriptor track's job):
DW0: [31:24] WP_OFFSET (8-byte units), BIT(22) WD_INFO_EN, BIT(20) FW_DL, [19:16] CHANNEL_DMA
(= txch), [15:11] HDR_LLC_LEN, BIT(7) WD_PAGE (1 for data/mgmt), [3:2] HW_SSN_SEL, [1:0]
HW_SSN_MODE. DW2: [30:24] MACID, BIT(23) TID_INDICATE, [22:17] QSEL, [13:0] TXPKT_SIZE (= skb->len).
DW3: BIT(13) BK, BIT(12) AGG_EN, [11:0] SW_SEQ. On AX-v0 the WD does **not** carry
ADDR_INFO_NUM (that is DW1 in the v1 format only); the count comes from the addr-info entry itself.

### 4.3 WP info — `struct rtw89_pci_tx_wp_info` (8 bytes, pci.h:1473-1480)
| off | field | value |
|---|---|---|
| 0 | le16 seq0 | `txwd->seq` (WD page index 0..511) \| BIT(15) `RTW89_PCI_TXWP_VALID` |
| 2 | le16 seq1 | 0 |
| 4 | le16 seq2 | 0 |
| 6 | le16 seq3 | 0 |
`seq0` is echoed back in the release report (RPP SEQ) and is how the driver finds the WD page/skb.

### 4.4 TX address info — `struct rtw89_pci_tx_addr_info_32` (8 bytes, pci.h:1482-1491; filled pci.c:1438-1455)
| off | width | field | bits / value |
|---|---|---|---|
| 0 | le16 | length | payload length = skb->len |
| 2 | le16 | option | BIT(15) `ADDR_MSDU_LS` = 1; BIT(14) `ADDR_LS` = 0 (not set by rtw89); bits[13:6] `ADDR_HIGH` = addr bits 39:32; bits[5:0] `ADDR_NUM` = 1 |
| 4 | le32 | dma | payload address bits 31:0 (skb->data mapped `DMA_TO_DEVICE`) |
Always exactly one entry (`add_info_nr = 1`); the whole 802.11 frame must be one contiguous mapping.

### 4.5 RX BD — `struct rtw89_pci_rx_bd_32` (8 bytes, pci.h:1528-1533; filled pci.c:3508-3533)
| off | width | field | value |
|---|---|---|---|
| 0 | le16 | buf_size | RTW89_PCI_RX_BUF_SIZE = 11498 (0x2CEA) |
| 2 | le16 | opt | bits[13:6] `RXBD_OPT_DMA_HI` = addr bits 39:32, rest 0 |
| 4 | le32 | dma | buffer address bits 31:0 |
Written once at ring allocation; never rewritten (buffers are recycled in place, §7).

### 4.6 RX buffer header — `struct rtw89_pci_rxbd_info` (first dword of every RX buffer; pci.h:1535-1542; parsed pci.c:183-197)
| bits | name | meaning |
|---|---|---|
| 31:29 | — | unused |
| 28:16 | RTW89_PCI_RXBD_TAG | 13-bit tag (1..0x1FFF); not validated for 8852B (`check_rx_tag = false`) |
| 15 | RTW89_PCI_RXBD_FS | first segment of a packet |
| 14 | RTW89_PCI_RXBD_LS | last segment |
| 13:0 | RTW89_PCI_RXBD_WRITE_SIZE | bytes written into this buffer, **including** this 4-byte header |

### 4.7 RX buffer layout (packet mode, RXQ and RPQ)
First segment (FS=1):
`[rxbd_info 4B][RX desc 16B short or 32B long][drv_info: drv_info_size*8 B][(shift*2 B, see note)][payload]`
- RX desc DW0 (txrx.h:345-352): [13:0] RPKT_LEN (payload length), [15:14] SHIFT, [21:16]
  WL_HD_IV_LEN, BIT(22) BB_SEL, BIT(23) MAC_INFO_VLD, [27:24] RPKT_TYPE, [30:28] DRV_INFO_SIZE
  (8-byte units), BIT(31) LONG_RXD. RPKT_TYPE values: 0 WIFI, 1 PPDU_STAT, 6 TX_REPORT, 7
  TX_REL_HOST, 9 TX_REL_CPU, 10 C2H, … (core.h:222-238).
- rtw89 computes `desc_info->offset = 4 + shift*2 + drv_info_size*8` and payload start =
  `offset + rxd_len` (core.c:4103-4109, pci.c:376). **Note**: the AX parser never reads the SHIFT
  field (desc_info->shift stays 0), so effectively payload = 4 + rxd_len + drv_info_size*8.
- Continuation segments (FS=0): `[rxbd_info 4B][payload continuation]` (pci.c:378).
- Payload bytes in this buffer = `WRITE_SIZE - start_offset`.

### 4.8 Release report (RPP) — `struct rtw89_pci_rpp_fmt` (1 dword, pci.h:1505-1513; parsed pci.c:572-582)
| bits | name | meaning |
|---|---|---|
| 31 | RTW89_PCI_RPP_POLLUTED | defined, ignored by rtw89 |
| 30:16 | RTW89_PCI_RPP_SEQ | WD page index (= WP info seq0 without the VALID bit); rtw89 rejects ≥ 512 |
| 15:13 | RTW89_PCI_RPP_TX_STATUS | 0 DONE (acked), 1 RETRY_LIMIT, 2 LIFE_TIME, 3 MACID_DROP (core.h:3645-3648) |
| 12:8 | RTW89_PCI_RPP_QSEL | qsel of the frame; txch = `rtw89_core_get_ch_dma(qsel)` |
| 7:0 | RTW89_PCI_RPP_MACID | MAC ID (not used by rtw89) |
An RPQ buffer: `[rxbd_info][RX desc (+drv_info)][RPP 0][RPP 1]…`; entries are parsed while
`offset + 4 <= WRITE_SIZE` (pci.c:680-684). rtw89 does not check the RX desc packet type for RPQ
(presumably 7 = TX_REL_HOST).

### 4.9 FWCMD / H2C packet on CH12 (pci.c:1551-1588, core.c:905-913, fw.c:77-94)
H2C skbs are allocated with `h2c_desc_size` (24) bytes of headroom (+8 for the H2C header when
present). At submit: `skb_push(24)`, zero it, fill with `rtw89_core_fill_txdesc` using
`ch_dma = 12`, `wd_page = 0`, `en_wd_info = 0`, `fw_dl = 1` only for firmware-download chunks,
`pkt_size` = length **before** the push. Resulting WD body: DW0 = `0x000C0000` (normal H2C) or
`0x001C0000` (FWDL, adds BIT(20)); DW2 = TXPKT_SIZE (payload length); DW1/3/4/5 = 0.
Whole skb (24 + payload bytes) is mapped `DMA_TO_DEVICE`; TX BD = {length = skb->len,
opt = LS | DMA_HI, dma = lo}. No WP info, no address info, no WD page, **no release report**.
FW download chunk payload = `FWDL_SECTION_PER_PKT_LEN = 2020` bytes (fw.h:287, fw.c:1814-1831).

---------------------------------------------------------------------------------------------------

## 5. Ring sizes, buffer sizes, memory layout

| item | value | source |
|---|---|---|
| TX BDs per ring | 256 (`RTW89_PCI_TXBD_NUM_MAX`; must be multiple of 16) | pci.h:1122, pci.c:3643-3646 |
| RX BDs per ring | 256 (`RTW89_PCI_RXBD_NUM_MAX`) | pci.h:1123, pci.c:3776 |
| BD size | 8 bytes (TX and RX) | pci.h:1465-1533 |
| TX BD pool | one `dma_alloc_coherent` of 7 rings × 256 × 8 = 14336 bytes; rings laid out consecutively in channel order ACH0 @+0, ACH1 @+0x800, ACH2 @+0x1000, ACH3 @+0x1800, CH8 @+0x2000, CH9 @+0x2800, CH12 @+0x3000 | pci.c:3626-3686 |
| RX BD pool | one coherent block 2 × 256 × 8 = 4096 bytes; RXQ @+0, RPQ @+0x800 | pci.c:3760-3815 |
| WD pages | per data channel (not CH12): 512 pages × 128 B = 64 KiB coherent; page i at `base + i*128`, `seq = i`; 6 channels → 384 KiB | pci.h:1124-1125, pci.c:3535-3584 |
| RX buffer | 11454 + 40 + 4 = **11498 bytes** (max VHT MPDU + long RX desc v2 + rxbd_info); `dev_alloc_skb(11498)`, zeroed, `dma_map_single(DMA_FROM_DEVICE)` for 11498 bytes, for all 256 BDs of **both** RXQ and RPQ | pci.h:1127-1128, pci.c:3688-3758 |
| addr info per WD | 1 (max defined 4, `RTW89_PCI_ADDRINFO_MAX`, unused) | pci.h:1126 |
| multi-tag keep-alive for H2C | 8 (`RTW89_PCI_MULTITAG`) | pci.h:1131, pci.c:119 |

Ring base alignment: rtw89 never checks; rings are ≥ 2 KiB aligned by construction (coherent
allocations are page-aligned and each ring is 2048 bytes). Keep at least that.

WD page state machine (pci.h:1701-1728, pci.c:501-570):
- free list (`wd_ring->free_pages`, count `curr_num`, initially 512) → `dequeue_txwd` at submit
  (len reset to 0) → busy list (`tx_ring->busy_pages`, FIFO in BD order) with the skb queued on
  `txwd->queue`.
- Leaves busy list when the TX BD is consumed by HW (`reclaim_txbd`), its skb is released when the
  RPP arrives; the page returns to the free list (memset 0) when **both** have happened, whichever
  comes last.

---------------------------------------------------------------------------------------------------

## 6. TX path at bus level

### 6.1 Data/mgmt frame submit (`rtw89_pci_tx_write`, pci.c:1637-1678; `txbd_submit` 1590-1635; `txwd_submit` 1494-1549)
Under `trx_lock`:
1. Sanity: CH12 only with tx_type FWCMD and vice versa (pci.c:1647-1653).
2. `avail = rtw89_pci_get_avail_txbd_num(ring)`; if 0 → -ENOSPC.
3. `txbd = &ring[wp]`.
4. Take a free WD page (none → -ENOSPC).
5. Map `skb->data, skb->len` `DMA_TO_DEVICE`.
6. Write WP info at `vaddr + wd_len` (wd_len = 24 or 48), address info right after (§4.2-4.4),
   `txwd->len = wd_len + 8 + 8`; then fill WD body/info at `vaddr + 0`.
7. Queue skb on `txwd->queue`; append txwd to `busy_pages`.
8. TX BD = {length = txwd->len, opt = BIT(14) | (upper32(txwd->paddr) << 6), dma = lower32(paddr)}.
9. `wp = (wp + 1) % 256` (pci.c:1347-1358). **No register write yet.**

### 6.2 Kick (`rtw89_pci_ops_tx_kick_off`, pci.c:1332-1371)
Separately, the core calls `tx_kick_off(txch)` after writing one or more frames (core.c:1300-1307;
from mac80211 tx and txq work): under `trx_lock`, `write16(idx_reg, wp)` — e.g. ACH0 → write16(0x1058,
wp). If `hci.paused` the kick is deferred via `kick_map` and replayed on unpause (pci.c:1365-1386);
not relevant for 8852B (never paused outside low-power HCI switching, which 8852B lacks).

### 6.3 Completion — two independent signals
(a) **BD consumption** (HW has DMA-read the BD and WD): `rtw89_pci_reclaim_txbd` (pci.c:501-520):
read32(idx_reg); `cnt = (hw_idx - rp) mod 256` (pci.c:61-85); `rp = hw_idx`; pop `cnt` WD pages
from the head of `busy_pages`; if a page's skb queue is already empty (RPP came first) → return it
to the free list.
(b) **Release report** on RPQ (§7.3): for each RPP: txch from QSEL, page = `wd_ring.pages[SEQ]`
(`rtw89_pci_release_rpp`, pci.c:596-625) → `rtw89_pci_release_txwd_skb` (pci.c:538-570): if the
page is still on the busy list, first run `reclaim_txbd`; if still busy afterwards warn "txwd is not
idle" (normal only in low-power mode). Then for every skb on the page: `dma_unmap_single`, report
status (`rtw89_pci_tx_status`, pci.c:460-499: if a waiter exists complete it; else
`ieee80211_tx_info_clear_status`, NOACK→`IEEE80211_TX_STAT_NOACK_TRANSMITTED`, status 0 →
`IEEE80211_TX_STAT_ACK`, then `ieee80211_tx_status_ni`). If the page is off the busy list → free it.
rtw89 assumes **one RPP per WD page for every frame**; without RPPs skbs leak and WD pages run out.

### 6.4 Flow control / resource accounting (pci.c:1222-1330)
- `avail_bd = (rp > wp) ? rp - wp - 1 : 256 - (wp - rp) - 1` (one slot always kept empty).
- Channel capacity reported to core = `min(avail_bd, wd_ring.curr_num)` via
  `check_and_reclaim_tx_resource(txch)`:
  - if paused → no-I/O variant (just the min);
  - CH12 → reclaim FWCMD (read 0x1080, §6.5), return avail_bd;
  - else under `trx_lock`: if `wd_cnt == 0 || bd_cnt == 0`: poll RPQ inline (read 0x1054, process
    all pending release reports, write host idx); if no RPQ work and `wd_cnt == 0` → return 0; if
    `bd_cnt` still 0 → `reclaim_txbd` (read ring idx). Return `min(bd_cnt, wd_cnt)`.
- The core queries this before pulling frames from mac80211 txqs (core.c:4736-4744) and before each
  H2C (core.c:1361-1365).

### 6.5 FWCMD (CH12) specifics
- Submit (pci.c:1551-1588): §4.9; skb appended to `h2c_queue`; `wp++`. Kick right after
  (`rtw89_h2c_tx`, core.c:1340-1375 → write16(0x1080, wp)). H2C is refused if MAC not powered.
- Reclaim (pci.c:100-144): read32(0x1080) → `cnt = (hw_idx - rp) mod 256`; move `cnt` skbs from
  `h2c_queue` to `h2c_release_queue`; then unmap+free all but the **8 most recent** entries of the
  release queue (kept alive, presumably because multi-tag read-ahead may still touch them). On reset
  / teardown `release_all` frees everything.
- Before FW download completes, all other TX channels are stopped (only CH12 released, §9.1 steps
  29-30), and STOP_WPDMA stays set until `mac_post_init` (WD-page DMA not needed for CH12).
- No interrupt is used for CH12 completion.

### 6.6 Flush (pci.c:1388-1436)
`flush_queues` → for each active channel except CH12: up to 60 × {read32(idx); if hw_idx == wp
return; udelay(1)}; timeout only logs.

---------------------------------------------------------------------------------------------------

## 7. RX path

### 7.1 Model
- Copy-out model: ring buffers stay mapped forever; each packet is copied into a freshly allocated
  skb of exactly `pkt_size` (`rtw89_alloc_skb_for_rx`, core.h:7950-7965), then the ring buffer is
  synced back to the device and reused. There is **no refill/re-map**; BD contents never change.
- `rx_ring_eq_is_full = false`: `wp` (host index) starts at 0 and is not written at init (the index
  was cleared by RWPTR_CLR). HW may fill BDs while `hw_idx + 1 != host_idx`; "RX descriptor
  unavailable" (RDU) = `(hw_idx + 1) % 256 == host_idx` (pci.c:747-772). Pending count
  = `(hw_idx - wp) mod 256` (pci.c:61-85).

### 7.2 RXQ processing (`rtw89_pci_poll_rxq_dma` pci.c:436-458, `rxbd_deliver` 413-434, `rxbd_deliver_skbs` 321-411)
1. `idx = read32(0x1050)`; `cnt = (HW_IDX - wp) mod 256`; `rp = HW_IDX`; if 0 → done.
2. `cnt = min(cnt, remaining budget)`.
3. While `cnt && napi_budget_countdown > 0`, for buffer `buf[wp]`:
   a. `dma_sync_single_for_cpu(11498)`, parse rxbd_info (FS/LS/TAG/LEN).
   b. If FS: must have no partial packet in progress; parse RX desc at `data + 4`
      (`rtw89_core_query_rxdesc`), allocate new skb(pkt_size), copy from
      `4 + drv_info*8 + rxd_len` up to LEN. Else (continuation): need a partial skb; copy from 4 to
      LEN.
   c. Copy guard: if `LEN - offset` exceeds the new skb tailroom and FS&&LS → copy only `pkt_size`
      bytes; otherwise drop.
   d. `dma_sync_single_for_device`, `wp = (wp + 1) % 256`.
   e. If LS: `rtw89_core_rx(desc_info, skb)` — dispatches by RPKT_TYPE: WIFI frames to mac80211
      (via PPDU-status pairing), others (C2H, PPDU status, …) to `rtw89_core_rx_process_report`
      (core.c:4490-4522).
   f. Any error → drop partial packet, still advance `wp`.
4. `write16(0x1050, wp)` — hands the consumed BDs back to HW (always, even on partial processing).
Multi-BD packets (FS…LS spanning several buffers) are supported but should not occur for 8852B
since one 11498-byte buffer holds a maximal MPDU.

### 7.3 RPQ processing (`rtw89_pci_poll_rpq_dma` pci.c:720-745, `release_tx` 697-718, `release_tx_skbs` 644-695)
Under `trx_lock` (also called from the TX resource path):
1. `cnt = (HW_IDX(read32(0x1054)) - wp) mod 256`.
2. For each buffer: sync for CPU, parse rxbd_info; require FS && LS (else error: all remaining
   `cnt` BDs are skipped by advancing `wp`); parse RX desc; walk RPP dwords from
   `4 + drv_info*8 + rxd_len` while `off + 4 <= LEN`, release each (§6.3b); sync for device;
   `wp++`.
3. `write16(0x1054, wp)`.
All pending RPQ entries are always processed regardless of NAPI budget; NAPI accounting counts
`min(cnt, budget)`.

### 7.4 NAPI poll (`rtw89_pci_napi_poll`, pci.c:4513-4539), budget = 64
1. `napi_budget_countdown = budget`.
2. `write32(0x10B4, 0x00100004)` — W1C RPQ status bits (RPQDMA_INT BIT(2) | RPQBD_FULL_INT BIT(20))
   (`isr_clear_rpq`, pci.c:4690).
3. RPQ poll; `countdown -= min(cnt, budget)`; if that equals budget → return budget (stay
   scheduled; RXQ is not polled this round).
4. `write32(0x10B4, 0x00080003)` — W1C RXQ status (RXDMA_INT BIT(0) | RXP1DMA_INT BIT(1) |
   RDU_INT BIT(19)) (`isr_clear_rxq`, pci.c:4691-4692).
5. RXQ poll. `napi_budget_countdown` is decremented once per frame handed to mac80211
   (core.c:3994-3996) — non-WIFI packets (C2H, PPDU status) do not consume budget. RXQ returns 0 if
   budget remains, or the remaining budget if it ran out (so total work == budget ⇔ exhausted).
6. If `work_done < budget` and `napi_complete_done()` → under `irq_lock`, if `running`, re-enable
   interrupts (write the three IMRs, §8.2).
Status bits are cleared *before* each ring is scanned, so an event arriving during the scan
re-asserts and will interrupt again after IMRs are restored.

---------------------------------------------------------------------------------------------------

## 8. Interrupts (AX)

### 8.1 Registers
| reg | addr | role |
|---|---|---|
| R_AX_HIMR0 | 0x01A0 | system/firmware IMR: BIT(21) HALT_C2H, BIT(22) WDT_TIMEOUT |
| R_AX_HISR0 | 0x01A4 | status for the above (W1C) |
| R_AX_PCIE_HIMR00 | 0x10B0 | PCIe DMA IMR (bits below) |
| R_AX_PCIE_HISR00 | 0x10B4 | PCIe DMA status (W1C) |
| R_AX_PCIE_HIMR10 | 0x13B0 | BIT(28) HC10ISR_IND, BIT(12) CH11, BIT(11) CH10 |
| R_AX_PCIE_HISR10 | 0x13B4 | status (W1C) |
(pci.h:162-266). HIMR00/HISR00 bits: 27 HC00ISR_IND, 26 HD1ISR_IND, 25 HD0ISR_IND, 24 HS0ISR_IND
(indicator that HISR0 has a pending event), 21 RETRAIN, 20 RPQBD_FULL, 19 RDU, 18 RXDMA_STUCK,
17 TXDMA_STUCK, 16 PCIE_HOTRST, 15 PCIE_FLR, 14 PCIE_PERST, 13 TXDMA_CH12, 12 CH9, 11 CH8,
10..3 ACH7..ACH0, 2 RPQDMA, 1 RXP1DMA, 0 RXDMA.
W1C semantics are inferred from rtw89 writing back the bits it read (pci.c:782-784, 4524, 4529).

### 8.2 Masks (`rtw89_pci_config_intr_mask`, pci.c:3891-3917; 8852B uses HS0ISR_IND BIT(24), not the 8851B workaround BIT(23))
| state | halt_c2h_intrs → HIMR0 | intrs[0] → HIMR00 | intrs[1] → HIMR10 |
|---|---|---|---|
| normal | 0x00200000 (HALT_C2H) | **0x011E0007** = RXDMA(0) \| RXP1DMA(1) \| RPQDMA(2) \| TXDMA_STUCK(17) \| RXDMA_STUCK(18) \| RDU(19) \| RPQBD_FULL(20) \| HS0ISR_IND(24) | **0x10000000** (HC10ISR_IND) |
| recovery (SER) | 0x00200000 | 0x01000000 (HS0ISR_IND only) | 0 |
- No TX-done (ACHx/CHx) interrupts are enabled: TX completion is purely RPQ + index polling.
- WDT_TIMEOUT BIT(22) is **not** in the AX v0 mask, so `isr_wdt_timeout` never fires on 8852B.
- `enable_intr` (pci.c:853-859): write32(0x01A0, halt_c2h), write32(0x10B0, intrs[0]),
  write32(0x13B0, intrs[1]). `disable_intr` (pci.c:861-867): write 0 to all three.
- IMRs are first written at `hci.start` (`rtw89_pci_ops_start` → `napi_enable`, `running = true`,
  enable, pci.c:1900-1928).

### 8.3 Flow (hard IRQ + thread + NAPI)
1. **Hard handler** (pci.c:1000-1022), under `irq_lock`: if `!running` → IRQ_HANDLED (stale
   event); else `disable_intr` (all IMRs = 0) → IRQ_WAKE_THREAD. It never reads status.
2. **Thread** (pci.c:951-998): under `irq_lock` `recognize_intrs` (pci.c:774-786):
   `halt = read32(0x01A4) & halt_c2h_intrs`; `isrs0 = read32(0x10B4) & intrs[0]`;
   `isrs1 = read32(0x13B4) & intrs[1]`; write each value back to its register (W1C, unconditionally,
   even if 0). Then:
   - `isrs0 & RDU (BIT19)` → debug log of RX indices only;
   - `halt & BIT21` → `rtw89_ser_notify(rtw89_mac_get_err_status())` (firmware halted; SER, out of
     scope — a minimal driver should at least log and read the error status);
   - `halt & BIT22` → SER WDT (unreachable here); `isr_sps_ocp = 0`;
   - if `under_recovery` or `low_power` → handle inline and re-enable IMRs (not normal path);
   - else if `running` → `napi_schedule()` inside `local_bh_disable/enable`. **IMRs stay 0** until
     NAPI completes (§7.4 step 6).
   `isrs1` (HC10ISR_IND) has no handler; it only causes a NAPI run. Semantics unknown.
3. **NAPI** (§7.4) re-enables the IMRs.

### 8.4 Locking
`irq_lock` (spin, irqsave) protects IMR/ISR accesses and `running`; `trx_lock` (spin_bh) protects TX
rings, WD pages, H2C queues and RPQ processing (RPQ can be processed from TX context). RXQ is
touched only by NAPI (plus flush/reset while stopped).

### 8.5 Interrupt mitigation (`rtw89_pci_recalc_int_mit`, pci.c:4257-4280)
`R_AX_INT_MIT_RX = 0x10D4`: BIT(19) RXMIT_RXP2_SEL, BIT(18) RXMIT_RXP1_SEL, [17:16] timer unit
(0 = 64 µs, 1 = 128, 2 = 256, 3 = 512), [15:8] counter match, [7:0] timer match (pci.h:903-912).
- Written only on traffic-level change (track work, core.c:5446-5448) and at scan start
  (core.c:6981); never at init (reset default used until then).
- If scanning or both TX/RX traffic level < RTW89_TFC_HIGH → write **0** (no mitigation).
- Else → `0x000C8020` = RXP2_SEL | RXP1_SEL | counter 128 (0x80) | unit 64 µs | timer 32
  (32 × 64 µs = 2.048 ms).
Optional for a minimal driver (write 0 or leave).

---------------------------------------------------------------------------------------------------

## 9. PCI ops invoked from the MAC layer

Call sites: `rtw89_mac_partial_init` (mac.c:4307-4339) = `set(0x8380, BIT0|BIT1)` (HCI TX/RX DMA
enable in MAC) → [BB preinit if bbmcu; not 8852B] → `rtw89_mac_dmac_pre_init` (DMAC func/clock
enable, DLE+HFC for FW-download quota) → **`hci.mac_pre_init`** → fwdl_preconfig (NULL on AX) →
firmware download over CH12. `rtw89_mac_init` (mac.c:4359-4400) = partial_init → enable BB/RF →
sys_init → trx_init → feat_init → **`hci.mac_post_init`** → early H2Cs.
`rtw89_core_start` (core.c:6565-6630) = pwr_on → … → mac_init → … → **`hci.start`**.
`rtw89_core_stop` (core.c:6634-6676) → flush → **`hci.stop`** → **`hci.deinit`** → mac_pwr_off →
**`hci.reset`**. Probe: `hci.mac_pre_deinit` after efuse (core.c:7203).

### 9.1 `mac_pre_init` = `rtw89_pci_ops_mac_pre_init_ax` (pci.c:3070-3144), 8852B, in order
| # | function | action for 8852B |
|---|---|---|
| 1 | rtw89_pci_ber | skipped (quirk not set) |
| 2 | rtw89_pci_rxdma_prefth | skipped (8852A only) |
| 3 | rtw89_pci_l1off_pwroff (2669) | `clr(0x1008, BIT(5))` L1OFF_PWR_OFF_EN |
| 4 | rtw89_pci_deglitch_setting | skipped (8852A/C) |
| 5 | rtw89_pci_l2_rxen_lat | skipped (8852A) |
| 6 | rtw89_pci_aphy_pwrcut (2699) | `clr(0x0004, BIT(14))` PSUS_OFF_CAPC_EN |
| 7 | rtw89_pci_hci_ldo (2709) | `set(0x0070, BIT(15))` PCIE_DIS_L2_CTRL_LDO_HCI; `clr(0x0070, BIT(14))` PCIE_DIS_WLSUS_AFT_PDN |
| 8 | rtw89_pci_dphy_delay (2724) | MDIO Gen1 reg 0x1B (RAC_REG_REV2, page 0): bits[15:12] BAC_CMU_EN_DLY := 1 (PCIE_DPHY_DLY_25US). Return value ignored. |
| 9 | rtw89_pci_autok_x (2388) | MDIO Gen1 reg 0x1D (RAC_REG_FLD_0, page 0): bits[3:2] BAC_AUTOK_N := 3 (PCIE_AUTOK_4). Failure aborts. |
| 10 | rtw89_pci_auto_refclk_cal(false) (2400) | (a) read cfg byte 0x82; bits[1:0]=1→Gen1, 2→Gen2, else **fail -EOPNOTSUPP**; (b) read cfg 0x719; if BIT(3) set, write it cleared (remember); (c) MDIO read reg 0x30 (RAC_CTRL_PPR_V1, page 1/3 for Gen1/Gen2); if BIT(13) B_AX_CALIB_EN set → write it cleared; (d) autook_en=false → skip calibration; (e) if 0x719 was modified, restore original. Net: **ref-clock auto-calibration disabled**. |
| 11 | rtw89_pci_power_wake_ax(true) (2733) | `set(0x0074, BIT(5))` BIT_WAKE_CTRL |
| 12-16 | autoload_hang, l12_vmain, gen2_force_ib, l1_ent_lat, wd_exit_l1 | skipped (8852C) |
| 17 | rtw89_pci_set_sic (2786) | `clr(0x13F0, BIT(4))` SIC_EN_FORCE_CLKREQ |
| 18 | rtw89_pci_set_lbc (2795) | `v = read32(0x11D8)`; v[7:4] := 8 (2 ms); v \|= BIT(1)\|BIT(0); `write32(0x11D8, v)`; then `set(0x11D8, v)` (redundant) |
| 19 | rtw89_pci_set_io_rcy | skipped (8852C) |
| 20 | rtw89_pci_set_dbg (2841) | `set(0x11C0, BIT(1)\|BIT(0))`; `mask(0x13F0, 0x3, 1)` → 0x13F0 bits[1:0] := 01 (macro names reused from 0x11C0; reproduce literally) |
| 21 | rtw89_pci_set_keep_reg (2858) | `set(0x1000, BIT(22)\|BIT(23))` TX/RXRST_KEEP_REG |
| 22 | (3113) | `set(0x1010, BIT(19))` STOP_WPDMA |
| 23 | ctrl_dma_all(false) (2114) | `set(0x1010, BIT(20))` STOP_PCIEIO; `clr(0x1000, BIT(13)\|BIT(11))` RX/TXHCI_EN |
| 24 | poll_dma_all_idle (2934) | poll `read32(0x101C) & 0x00070F00 == 0` (10 µs step, 100 µs timeout); then poll `read32(0x101C) & 0x3 == 0` (10/100 µs). Failure aborts. |
| 25 | clr_idx_all_ax (2867) | `set(0x1014, 0x0000070F)` (ACH0-3, CH8, CH9, CH12); `set(0x1018, 0x3)` (RXQ, RPQ). (ACH4-7 / CH10-11 clears skipped for 8852B.) |
| 26 | rtw89_pci_mode_op (2953) | TX/RX trunc bits (0x1000 BIT5/4): **no write** for 8852B; `clr(0x1000, BIT(18))` (packet RXBD mode); `mask(0x1000, [10:8], 7)` TX burst 2048 B; `mask(0x1000, [16:14], 3)` RX burst 128 B; `write32(0x1000, read32(0x1000) \| BIT(12))` multi-tag; `mask(0x13F0, [18:16], 7)` 8 tags; `mask(0x1004, [27:24], 1)` and `mask(0x1004, [19:16], 1)` WD DMA interval 256 ns idle/active; `set(0x8810, BIT(0))` 8-byte addr info; `clr(0x9A00, BIT(1))` |
| 27 | rtw89_pci_ops_reset (1878) → reset_trx_rings (1776) | for ACH0,1,2,3,CH8,CH9,CH12: wp=rp=0; `write16(num, 256)`; `write32(bdram, value)` (§3.2); `write32(desa_l, lo)`; `write32(desa_l+4, hi)`. For RXQ, RPQ: wp=0, rp=0, reset partial-packet state; `write16(num, 256)`; `write32(desa_l, lo)`; `write32(desa_h, hi)` (no idx write). Then free any pending TX skbs (status MACID_DROP) / H2C skbs. |
| 28 | rtw89_pci_rst_bdram_ax (47) | `set(0x1000, BIT(3))`; poll `read32(0x1000) & BIT(3) == 0`, 1 µs step, 100 µs timeout (atomic). Failure aborts ("reset bdram busy"). |
| 29 | ctrl_txdma_ch_ax(false) (249) | `set(0x1010, 0x00070F00)` stop all 7 TX channels |
| 30 | ctrl_txdma_fw_ch_ax(true) (266) | `clr(0x1010, BIT(18))` release CH12 only |
| 31 | ctrl_dma_all(true) | `clr(0x1010, BIT(20))` STOP_PCIEIO off; `set(0x1000, BIT(13)\|BIT(11))` RX/TX HCI on |
After this: CH12 + RXQ + RPQ DMA live, STOP_WPDMA (BIT19) still set, other TX channels stopped →
firmware download over CH12.

Resulting register state after pre-init (only bits touched): 0x1000: [23]=1,[22]=1,[18]=0,
[16:14]=3,[13]=1,[12]=1,[11]=1,[10:8]=7; 0x1004: [27:24]=1,[19:16]=1; 0x1008: [5]=0;
0x1010: [20]=0,[19]=1, TX bits 0x00070F00 = 0x00030F00 (all but CH12 stopped); 0x13F0: [18:16]=7,
[4]=0, [1:0]=01; 0x11C0: [1:0]=11; 0x11D8: [7:4]=8,[1]=1,[0]=1; 0x0004[14]=0; 0x0070[15]=1,[14]=0;
0x0074[5]=1; 0x8810[0]=1; 0x9A00[1]=0.

### 9.2 `mac_post_init` = `rtw89_pci_ops_mac_post_init_ax` (pci.c:3238-3268)
1. `rtw89_pci_ltr_set(true)` (§9.4); error aborts mac_init.
2. (8852A LTR SW trigger — skipped.)
3. 885xB: `set(0x8810, BIT(0))`; `clr(0x9A00, BIT(1))` (again).
4. `ctrl_txdma_ch_ax(true)`: `clr(0x1010, 0x00070F00)` — all TX channels running.
5. `clr(0x1010, BIT(19) | BIT(20))` — release STOP_WPDMA and STOP_PCIEIO.

### 9.3 `mac_pre_deinit` = `rtw89_pci_ops_mac_pre_deinit_ax` (pci.c:3146-3151)
`clr(0x0074, BIT(5))` (power_wake off). Called only in the probe-time efuse path.

### 9.4 LTR — `rtw89_pci_ltr_set` (pci.c:3153-3186, regs reg.h:585-621)
- `en == false` → return 0 (no-op; that is what `deinit` calls).
- If PCIe DEVCTL2.LTR_EN (bit 10) is clear → return 0 (nothing programmed).
- Read 0x8410, 0x8414, 0x8418, 0x841C; if any reads 0xFFFFFFFF or 0xEAEAEAEA → **-EINVAL** (fails
  mac_init).
- `set(0x8410, BIT(0) LTR_HW_EN | BIT(1) LTR_EN | BIT(6) LTR_WD_NOEMP_CHK)`;
  `mask(0x8410, [13:12], 2)` (space 500 µs); `mask(0x8410, [10:8], 7)` (idle timer 3.2 ms);
  `mask(0x8414, [11:0], 0x28)` (RX0 threshold); `mask(0x8414, [27:16], 0x28)` (RX1 threshold);
  `write32(0x8418, 0x90039003)` (idle latency: req=1, scale 4 = 1,048,576 ns, value 3 ≈ 3.1 ms, for
  snoop and no-snoop halves); `write32(0x841C, 0x880B880B)` (active: scale 2 = 1024 ns, value 11
  ≈ 11.3 µs).
Optional for a minimal driver (if skipped, device sends no LTR messages beyond defaults).

### 9.5 `hci.start` / `hci.stop` (pci.c:1922-1938)
start: `napi_enable`; `irq_lock`: running = true, enable IMRs. stop: `irq_lock`: running = false,
IMRs = 0; `synchronize_irq`; `napi_synchronize` + `napi_disable`.

### 9.6 `hci.deinit` = `rtw89_pci_ops_deinit` (pci.c:3053-3068)
1. `clr(0x0074, BIT(5))` (power_wake(false) → `rtw89_pci_power_wake_ax`).
2. (8852A LTR idle trigger — skipped.)
3. `ltr_set(false)` → no-op.
4. `ctrl_dma_all(false)`: `set(0x1010, BIT(20))`; `clr(0x1000, BIT(13)|BIT(11))`.
5. `clr_idx_all`: `set(0x1014, 0x70F)`; `set(0x1018, 0x3)`.

### 9.7 `hci.reset` = `rtw89_pci_ops_reset` (pci.c:1878-1898)
Re-program ring registers exactly as §9.1 step 27 (works even after MAC power-off — the 0x1000-0x13FF
PCIe block stays accessible), then under `trx_lock` release all pending H2C skbs and all TX skbs
still owned by WD pages (reported as MACID_DROP), returning pages to the free list.

### 9.8 Other gen_ax ops (for completeness)
`lv1rst_stop_dma/start_dma` (pci.c:4416-4463) are SER level-1 recovery (out of scope);
`poll_txdma_ch_idle`, `ctrl_trxhci`, `clr_idx_all`, `rst_bdram` hci ops are used only by WoWLAN
(wow.c:1356-1434, out of scope). `aspm/clkreq/l1ss_set` §1.5.

---------------------------------------------------------------------------------------------------

## 10. Deinit / remove / shutdown and clean hand-off

### 10.1 What rtw89 does
Interface down / IPS (`rtw89_core_stop`, core.c:6634-6676): … `hci_flush_queues(drop)` (§6.6) →
`rtw89_mac_flush_txq` → `hci.stop` (§9.5) → `hci.deinit` (§9.6) → `rtw89_mac_pwr_off` (power track)
→ `hci.reset` (§9.7). After probe the chip is also left powered off (§1.1 step 6).

`rtw89_pci_remove` (pci.c:4860-4874), in order:
1. `rtw89_pci_free_irq` (devm_free_irq + pci_free_irq_vectors);
2. `rtw89_core_napi_deinit`;
3. `rtw89_core_unregister` → mac80211 unregister → if the interface was up, `rtw89_core_stop` as
   above (its `hci.stop` then masks IMRs after the IRQ is already gone);
4. `rtw89_pci_clear_resource` (pci.c:3880-3889): unmap+free all RX skbs, free WD page pools, free
   TX and RX BD pools, `pci_iounmap` + `pci_release_regions`, free leftover H2C skbs;
5. `pci_disable_device` (also clears bus-master);
6. core deinit, free hw.

`rtw89_pci_shutdown` (pci.c:4877-4888): only sets RTW89_FLAG_SHUTDOWN (used to suppress rfkill
polling, mac80211.c:2007-2009). **No hardware action.**
Suspend/resume (pci.c:4571-4643) are out of scope for this track (they toggle 0x001C BIT(6) with the
BIT(5) write-lock, 0x0070 BIT(15), 0x1000 BIT(21)/BIT(19), and resume re-runs basic_cfg).

### 10.2 What the stock driver needs to re-probe cleanly
The stock probe/start path is self-healing: `mac_pwr_on` resets power state, and `mac_pre_init`
sets STOP_WPDMA/STOP_PCIEIO, disables HCI DMA, polls busy-idle, clears all indices, rewrites all
ring registers and resets BD-RAM. It does **not** undo: vendor config bits (0x719 BIT(5) DAC,
BIT(4) CLKREQ, BIT(3) L1; 0x718 BIT(5); 0x70F; 0x725), DEVCTL2 COMP_TMOUT_DIS, PHY regs 0x1B/0x1D/
0x30 (it rewrites the same values), mitigation 0x10D4.

Recommended teardown for a new driver (superset of rtw89, fixing its IRQ-order wart):
1. Stop submitting TX; optionally wait for hw_idx == host_idx on each active ring (≤ 60 µs).
2. Under the IRQ lock: running = false; write 0 to 0x01A0, 0x10B0, 0x13B0; `synchronize_irq`;
   disable NAPI.
3. `clr(0x0074, BIT(5))`; `set(0x1010, BIT(20))`; `clr(0x1000, BIT(13)|BIT(11))`;
   `set(0x1014, 0x70F)`; `set(0x1018, 0x3)`. (Optionally also `set(0x1010, 0x00070F00 | BIT(19))`
   and poll 0x101C busy bits to be sure no DMA is in flight — rtw89 relies on power-off instead.)
4. MAC power-off sequence (power track) — halts firmware and the DMAC.
5. Only now unmap/free RX buffers, WD pools and BD rings (DMA must be stopped first).
6. Free IRQ vectors, `pci_iounmap`, `pci_release_regions`, `pci_disable_device`.
7. If the new driver changed any vendor config bit that rtw89 would not set on the reference system
   (e.g. 0x719 BIT(5) — rtw89 leaves it clear behind an AMD bridge), restore the original value.
8. `.shutdown`: at minimum perform steps 2-4 (rtw89 does nothing; a device left DMA-active across
   kexec is a hazard).

---------------------------------------------------------------------------------------------------

## 11. Minimal station-mode driver: mandatory vs optional

Mandatory (hardware will not work without it):
- BAR2 mapping, bus mastering, one MSI/INTx vector, 32-bit DMA (DAC optional).
- Rings: at least CH12 (H2C + FW download), RXQ (RX + C2H), RPQ (TX release reports), ACH0 and CH8
  (data/mgmt). rtw89 programs all 7 active TX rings; programming all 7 with valid memory is
  recommended even if only some are used (post_init un-stops all of them).
- §9.1 steps 21-31 (DMA stop/idle poll, index clear, mode_op, ring programming, BD-RAM values,
  rst_bdram, CH12-only enable) and §9.2 steps 3-5.
- §9.1 steps 3, 6, 7, 11 (power/LDO/wake bits) — cheap, replicate.
- Interrupt masks §8.2 and the RPQ/RXQ W1C + NAPI-style re-enable protocol.
- WD page + WP seq + addr-info construction and RPP-driven skb release.

Recommended (replicate stock behaviour; low risk):
- PHY tweaks §9.1 steps 8-10 (dphy delay, autok_x, disable refclk calibration); LBC watchdog
  (step 18); debug/stuck settings (step 20); SIC force-clkreq off (step 17).
- DEVCTL2 completion-timeout disable (§1.5 step 3).

Optional / skip for this user:
- ASPM/CLKREQ/L1SS vendor bits (user keeps them off → do nothing).
- LTR programming (§9.4; only if DEVCTL2.LTR_EN), interrupt mitigation (§8.5), 36-bit DAC,
  low-power HCI mode, WoWLAN/SER hooks, suspend/resume bits.
- ACH1-3 / CH9 usage (can map all data to ACH0; CH9 is HIQ for AP DTIM traffic).

---------------------------------------------------------------------------------------------------

## 12. Open questions / uncertainties
1. **TX/RX TRUNC mode bits** (0x1000 BIT5/BIT4) are never written for 8852B although
   `txbd_trunc_mode = TRUNC`; rtw89 relies on the reset default producing 8-byte BDs. Semantics of
   the bits (and whether default = truncated for 8852B) are not visible in source. Replicate (don't
   touch).
2. `set_dbg` writes 0x13F0 bits[1:0] = 01 using macro names defined for 0x11C0 — may be a copy
   bug in rtw89, but it is what the working driver does; replicate literally.
3. Meaning of HIMR10 BIT(28) HC10ISR_IND, of RXP1DMA_INT vs RXDMA_INT, and of BD-RAM max/min
   fields are not documented; values above are what rtw89 programs.
4. `R_AX_DBG_ERR_FLAG` has a "TXBD 4K boundary length error" flag; rtw89 does not align TX
   payloads/H2C buffers to avoid 4 KiB crossings, so crossings are presumably legal (WD pages never
   cross). Unverified.
5. W1C semantics of HISR registers and of the LBC_FLAG bit are inferred from usage.
6. Whether every TX frame (incl. mgmt, no-ack, broadcast) always yields an RPP is assumed by rtw89's
   design, not stated.
7. AX RX-desc SHIFT field is ignored by rtw89 (treated as 0); presumably HW never sets it for this
   configuration.
8. LTR: whether DEVCTL2.LTR_EN is set on the reference system was not checked (would require config-space
   read); if set, 0x8410-0x841C must read sane values or `mac_init` fails.
