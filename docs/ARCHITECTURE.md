# ax52 architecture

This is a map of the driver for people who want to change it. It describes how the
code is organised and how control and data flow through it. The register-level
behaviour lives in [`docs/spec/`](spec/), and this document points there instead of
repeating it.

ax52 drives one RTL8852BE PCIe function with one station interface. The work is
split between host and firmware like this:

| Firmware (WCPU) | Host (ax52) |
|---|---|
| Rate adaptation, retries, EDCA, address/BSSID/security CAM, BA responder entries, the TX scheduler | Power sequencing, firmware download, the whole MAC/DMA setup, BB/RF tables, channel switching, TX power, **all RF calibration**, BT antenna arbitration |

The firmware is commanded with H2C messages and answers with C2H events
(see [Firmware interface](#firmware-interface)).

## Source map

| File | Responsibility | Entry points worth knowing |
|---|---|---|
| `ax52.h` | Device struct, ring/firmware/efuse types, MMIO helpers, cross-file API | `struct ax52_dev`, `rd32/wr32/mask32`, `ax52_poll32` |
| `reg.h` | Named registers for system, PCIe host interface and DMAC | |
| `main.c` | PCI probe/remove/shutdown, chip start/stop | `ax52_pci_probe`, `ax52_chip_start`, `ax52_chip_stop` |
| `pwr.c` | MMIO helpers, XTAL-SI bus, power on/off, WCPU control | `ax52_power_on`, `ax52_wcpu_enable_dl` |
| `pci.c` | Buffer-descriptor rings, DMA bring-up, FWCMD channel, TX/RX rings, release reports, IRQ/NAPI | `ax52_pci_pre_init`, `ax52_fwcmd_tx`, `ax52_pci_tx`, `ax52_napi_poll` |
| `fw.c` | Firmware container parsing, image download, element (table) lookup | `ax52_fw_load`, `ax52_fw_download`, `ax52_fw_txpwr_elem` |
| `efuse.c` | Efuse dump and logical-map decode | `ax52_efuse_read` |
| `mac.c` | Packet buffer (DLE), host flow control, DMAC/CMAC init, port config, BT coexistence registers | `ax52_mac_pre_fwdl`, `ax52_mac_init`, `ax52_mac_port_update`, `ax52_coex_*` |
| `h2c.c`, `h2c.h` | H2C/C2H protocol and every firmware command the driver sends | `ax52_h2c_send`, `ax52_h2creg_msg`, `ax52_c2h_handle` |
| `phy.c`, `phy.h` | BB/RF register access, PHY tables, DM init, channel switching, DIG | `ax52_rf_read/write`, `ax52_phy_init`, `ax52_phy_set_channel` |
| `phy_txpwr.c` | TX power: by-rate, regulatory limits, RU limits, TX shaping | `ax52_txpwr_load`, `ax52_txpwr_apply` |
| `rfk.c`, `rfk.h`, `rfk_table.c` | RF calibration and its register sequences | `ax52_rfk_init`, `ax52_rfk_sta_connect`, `ax52_rfk_track` |
| `tx.c` | WiFi descriptor (WD) build, queue selection, TXQ scheduler, TX status | `build_wd`, `txq_work_fn`, `ax52_tx_status` |
| `rx.c` | RX descriptor parse, rate/status translation, PPDU-status pairing | `ax52_rx_packet`, `rx_ppdu_status` |
| `mac80211.c` | `ieee80211_ops`, capabilities, regulatory notifier, periodic work | `ax52_ops`, `ax52_register_hw`, `track_work_fn` |

Private state of the larger subsystems sits behind opaque pointers in
`struct ax52_dev` (`phy_priv`, `rfk_priv`, `h2c_priv`) and is allocated at probe.

## Lifecycle

### Probe (`main.c: ax52_pci_probe`)

1. Allocate `ieee80211_hw` with `struct ax52_dev` as private data, and the TX workqueue.
2. Enable the PCI device and map BAR2. DMA is 32-bit only (no DAC). Completion
   timeouts are disabled in DEVCTL2, as rtw89 does.
3. Read the chip cut from `REG_SYS_CFG1`.
4. Allocate all rings (`ax52_pci_alloc_rings`). They stay allocated until remove.
5. Request `rtw89/rtw8852b_fw-2.bin`, pick the image for this cut and index its
   elements (`ax52_fw_load`). The driver ships no PHY or TX-power tables of its own;
   they come from this file.
6. `ax52_probe_bringup`: power on, run the pre-download MAC init, download the
   firmware, read the efuse (MAC address, RFE type, calibration data), then
   **power off**. The chip stays off until mac80211 calls `.start`.
7. Allocate the PHY/RFK/H2C private state, then the IRQ vector and NAPI, and
   register with mac80211.

### Interface up (`main.c: ax52_chip_start`, from `.start`)

The firmware is lost at every power-off, so every start repeats the download:

1. `power_up_fw`: `ax52_power_on` → `ax52_mac_pre_fwdl` (HCI DMA, minimal DMAC,
   firmware-download packet-buffer layout, PCIe pre-init with only the FWCMD ring
   running) → `ax52_fw_download`.
2. `ax52_mac_init`: BB/RF power, DMAC (normal packet-buffer layout, flow control,
   scheduler, MPDU processor, security engine), CMAC, error IMRs, release reports.
   It ends with `ax52_pci_post_init`, which starts all TX DMA channels.
3. `ax52_h2c_post_mac_init`: the firmware's offload-config and log-config commands.
4. BB/RF reset, then `ax52_phy_init`: BB table, BB-gain table, radio tables (with
   their mirror to the firmware), DM init, crystal trim, TX-power tables and
   reference, efuse trims, RX path setup.
5. `ax52_coex_init`: PTA into a known state; WL takes the antennas.
6. `ax52_rfk_init`: NCTL microcode, DPD back-off, RCK, DACK, RX DCK.
7. PPDU status, RX filter, RTS threshold, then interrupts/NAPI on (`ax52_pci_start`)
   and the 2 s track work.

No channel is programmed here; the first `.config(CHANGE_CHANNEL)` does that.

### Station lifecycle (`mac80211.c`)

| mac80211 event | What ax52 does |
|---|---|
| `add_interface` | Port 0 registers, per-MACID tables, then H2C: MACID unpause, role create, join (no link), address CAM, default CMAC table |
| `config(CHANGE_CHANNEL)` | `ax52_phy_set_channel` (see [PHY and RF](#phy-and-rf)), then `ax52_coex_wl_only` for the new band |
| `bss_info_changed(BSSID)` | Address CAM update with the BSSID |
| `sta_state` NOTEXIST→NONE | Full per-channel RF calibration (`ax52_rfk_sta_connect`) before authentication, then the RF "MCC channel" notify |
| `bss_info_changed(ASSOC)` | H2C: assoc CMAC table, join (connected), address CAM with AID, rate adaptation; port 0 to infrastructure mode |
| `conf_tx`, `ERP_SLOT` | EDCA via H2C (the firmware owns EDCA) |
| `set_key` | CCMP/GCMP into the security CAM; everything else stays in mac80211 software crypto |
| `ampdu_action` | TX: immediate start, aggregation limit register; RX: static BA CAM entry when one is free |
| `sta_state` AUTH→NONE | Reset aggregation state, H2C assoc CMAC table, join (disconnect), address CAM |
| `remove_interface` | Role remove, address CAM invalidated |

Scanning is mac80211 software scan: channel hops arrive as `.config` calls, and
`sw_scan_start/complete` only adjust TSSI (`ax52_rfk_scan`). Channel contexts are
mac80211's emulated ones (`ieee80211_emulate_*_chanctx`).

### Interface down, remove, shutdown

`ax52_chip_stop` stops the track and recovery work, interrupts and NAPI, cancels
the TX workers, hands the antennas back to BT, stops DMA, powers off and finally
completes every frame still in the rings as dropped (`ax52_pci_reset`).
`ax52_pci_remove` unregisters from mac80211 (which stops the radio if needed)
and then frees in reverse probe order. `ax52_pci_shutdown` runs the same
`ax52_chip_stop` under the wiphy mutex, so no DMA, interrupt source or worker
survives a reboot or kexec.

### Firmware failure and recovery (`main.c`)

The firmware is declared failed (`ax52_fw_failed`, callable from any context)
when it raises the halt interrupt, stops consuming H2C commands, or stops
answering the register mailbox. From then on every H2C fails at once instead of
waiting for ring space. `recovery_work` then does what rtw89's L2 recovery does:
it stops the radio, forgets the interface, and calls `ieee80211_restart_hw`.
mac80211 then starts the device again and replays the interface, channel, station,
keys and EDCA. Three restarts are allowed per minute. After that the radio stays
off until the interface is restarted, and the ops that touch hardware check
`running` in the meantime.

## Execution contexts and locking

| Context | Runs |
|---|---|
| **wiphy mutex** (process) | Almost every `ieee80211_ops` callback, `track_work`, `regd_work` and `recovery_work` (all `wiphy_work`), so all PHY/RFK code |
| **atomic mac80211 callbacks** | `.tx`, `.wake_tx_queue`, `.sta_statistics`, `.link_sta_rc_update` (RCU read side) |
| **`txq_wq`** (unbound, high priority) | `txq_work` (TXQ scheduler) and `ba_work` (starts TX BA sessions) |
| **NAPI** (softirq) | RX ring, release-report ring, C2H handling, PPDU-status pairing |
| **timer** | `ppdu_timer`: schedules NAPI again while MPDUs wait for their PPDU status |
| **hard IRQ** | `ax52_irq`: checks the device raised it, masks interrupts, acknowledges status, schedules NAPI |

| Lock | Type | Protects |
|---|---|---|
| `irq_lock` | spinlock, irqsave | Interrupt mask registers; writes of `running` (also only changed under the wiphy mutex) |
| `tx_lock` | spinlock, BH | Data/management rings, WD pages, release-report processing |
| `h2c_lock` | spinlock, BH | FWCMD ring and `h2c_seq` |
| `ax52_h2c.lock` | mutex | Register mailbox, nested scheduler TX pause, BA CAM bookkeeping |
| `ax52_h2c.ra_lock` | spinlock, BH | Last RA parameters and the RA report (NAPI writer vs. `sta_statistics`) |
| `regd_lock` | spinlock | Country handed from the regulatory notifier (no wiphy mutex) to `regd_work` |

Two rules follow from this:

- **H2C senders never sleep.** Some run in atomic mac80211 callbacks, so
  `ax52_fwcmd_tx` busy-waits for FWCMD ring space under `h2c_lock`. The wait is
  bounded at 100 ms, and a timeout declares the firmware failed, so it happens
  once and not for every later command. Nothing waits for a C2H acknowledgement.
- **PHY, RFK and register-mailbox code may sleep** and must only be called with the
  wiphy mutex held.

## Data path

### Rings (`pci.c`)

| Ring | Hardware channel | Used for |
|---|---|---|
| `TXQ_ACH0..3` | ACH0-3 | Data frames, one ring per access category (BE, BK, VI, VO) |
| `TXQ_MGMT` | CH8 | Management and (QoS-)null frames |
| `TXQ_HIQ` | CH9 | Programmed, unused (AP DTIM traffic in rtw89) |
| `TXQ_FWCMD` | CH12 | H2C commands and firmware download |
| `RXQ_DATA` | RXQ | Received frames, C2H events, PPDU status |
| `RXQ_RPQ` | RPQ | TX release reports |

All descriptors are 8-byte "truncated" buffer descriptors, 256 per ring.

### TX

1. mac80211 TXQs wake `txq_work` (`tx.c`). For each access category it asks the ring
   for free space (`ax52_pci_tx_avail`), dequeues that many frames, and kicks each
   ring once. Frames mac80211 does not queue arrive through `.tx` and are kicked
   immediately.
2. `build_wd` fills the 48-byte WiFi descriptor: queue select, MACID, sequence,
   aggregation, retry-lowest rate, LDPC/STBC, hardware-crypto fields. Management
   frames use a fixed low rate and hardware sequence numbers; data rates come from
   firmware rate adaptation. Rate bitmaps from mac80211 (basic rates, the AP's
   supported rates) are looked up in the **BSS band's** rate table
   (`ax52_bss_band`), which differs from the hardware's band during a scan.
3. `ax52_pci_tx` writes the descriptor into one of 512 128-byte **WD pages** of that
   ring, followed by the page-sequence tag and one address entry pointing at the
   separately mapped frame. The buffer descriptor points at the WD page.
4. Completion has two halves. The buffer descriptor is consumed (hardware index
   moves), and a **release report** for the page's sequence number arrives on the RPQ
   ring. The page is free only after both. The report's status becomes the mac80211
   TX status (ACK or not). The hardware index is trusted only within the BDs in
   flight. If a report never comes, `ax52_pci_tx_reap` (every 2 s) completes the
   frame as dropped once it is 2 s old and the packet engine holds no frames.
5. Back-pressure: when a ring is full, `txq_work` sets `tx_starved` and re-polls
   after one jiffy; processed release reports restart it at once.
6. TX block-ack sessions are started by the driver (`ba_work`), once per TID. After
   a session ends or fails to start, the TID may try again after 2 s.
7. `.flush` waits for the rings to be fetched and the packet engine to drain, then
   hands the pending release reports to mac80211. `drop` is treated as a flush,
   which mac80211 allows.

### RX

1. The RX rings use permanently mapped coherent buffers. `rxq_poll` copies each
   packet (including multi-buffer ones) into a fresh skb with the RX descriptor kept
   in front, so the buffers never need remapping.
2. `ax52_rx_packet` dispatches on the packet type: C2H to `h2c.c`, PPDU status to
   the pairing logic, frames to mac80211.
3. Signal strength is not in the frame descriptor. Management and data frames wait
   in `ppdu_q` until the PPDU status with the same 3-bit PPDU counter arrives. That
   report supplies per-chain RSSI and the real channel. A frame of a new PPDU flushes
   the queue, and frames that have waited 10 ms go up without signal, with
   `ppdu_timer` polling again when no interrupt follows.
4. With hardware decryption inside an RX BA session, `rx_pn_valid` drops frames that
   would trip mac80211's replay check (the same workaround as rtw89).

## Firmware interface

`h2c.c` implements two transports:

- **Packet H2C** on the FWCMD ring: `[24-byte WD][8-byte header][payload]`. Every
  fourth command requests a receive-ack (the firmware expects this). The 8 most
  recently consumed ring slots are never reused, because the DMA engine may still be
  reading ahead.
- **Register mailbox** (`0x8140`/`0x8150`): used to pause and resume the
  scheduler (`ax52_mac_sch_tx_en`), which the firmware owns once running. Pauses
  nest. If the firmware does not answer, the register is written directly so TX
  cannot stay stuck, and the firmware is declared failed.

The firmware file is parsed defensively: section and element sizes are checked in
64-bit arithmetic against the file, and BB table entries outside the 64 KiB BB
window are ignored.

Handled C2H events: receive/done acks (done-ack errors are logged), firmware log
(`fw_log=1`), and the rate-adaptation report behind `sta_statistics`. Others are
logged once at debug level.

## PHY and RF

- **Access.** BB registers are at MMIO `0x10000 + addr`. RF registers are 20 bits
  wide: "A-die" ones (`< 0x100`) go through the serial interface, "D-die" ones
  (bit 16 set) are direct. See `ax52_rf_read/write`.
- **Tables.** BB, radio and NCTL tables come from the firmware file and contain
  conditional blocks per (RFE type, chip cut). `tbl_apply` selects our variant
  ([05 §3.2](spec/05-phy-chan-txpwr.md)).
- **Channel switch** (`ax52_phy_set_channel`): pause scheduler TX, PPDU status and TX
  power control; program MAC, BB and RF; check synthesizer lock (`ax52_rfk_lck_check`);
  apply TX power; resume. On a band change, TSSI is re-armed from stored alignment
  (`ax52_rfk_channel`).
- **TX power** (`phy_txpwr.c`): by-rate targets, then per-regulation limits and RU
  limits for the country from the regulatory notifier. The lookup uses the
  firmware's REGD element, with worldwide as fallback.
- **RF calibration** (`rfk.c`):

  | When | What |
  |---|---|
  | Interface up | NCTL microcode, DPD back-off, RCK, DACK, RX DCK |
  | Band change | TSSI re-armed from stored or default alignment (no TX) |
  | Before authentication (`sta_state` NOTEXIST→NONE) | RX DCK, IQK, TSSI with PMAC test transmissions (cached per channel), DPK |
  | Scan start / end | TSSI default TX AGC, stored alignment restored |
  | Every 2 s (not while scanning) | Thermal EWMA, DPK power-scaling tracking |

  RX DCK, IQK, DPK and TSSI alignment run inside `rfk_begin/end`: the BT
  handshake (`ax52_coex_rfk_begin/end`, WL forced onto the antennas) plus a
  scheduler TX pause. DACK takes only the BT handshake. A failed IQK or DPK leaves
  the engine's neutral defaults (identity IQ correction, DPD off).
- **Coexistence.** No coex algorithm runs. On 2.4 GHz WL owns the shared antenna
  (BT loses it). On 5 GHz BT keeps hardware arbitration of its own path. At
  interface down the antenna is handed back to BT.

## Periodic work

`track_work` (every 2 s, wiphy work): DIG packet-detection thresholds from the AP's
beacon RSSI (`phy_dig`), RF tracking (`ax52_rfk_track`), reaping of TX frames whose
release report never came, and a rate-adaptation refresh so the firmware's
RSSI-based rate floor stays current. RF tracking and the rate-adaptation refresh
are skipped while scanning.

## Deliberately not implemented

Most of these are optional in the vendor flow; the specification says what each
would take.

- Power saving (IPS/LPS/deep PS), suspend/resume, WoWLAN.
- Firmware scan offload and real channel contexts ([08 §4.7](spec/08-mac80211-orchestration.md)).
- AP, P2P and monitor interfaces.
- Beamformee (its capability bits are cleared), TX A-MSDU, hardware BIP.
- Firmware error recovery below a full restart (SER levels 0/1); see
  [Firmware failure and recovery](#firmware-failure-and-recovery-mainc).
- The firmware packet-drop command for `.flush(drop)` ([04 §4.19](spec/04-h2c-c2h.md)).
- BT coexistence policies and coex H2Cs ([04 Appendix A](spec/04-h2c-c2h.md)).
- CFO/crystal tracking and EDCCA tracking ([05 §7](spec/05-phy-chan-txpwr.md)).
- PCIe LTR and interrupt mitigation ([01 §8.5, §9.4](spec/01-pci-dma.md)).

## Adding hardware behaviour

The project works spec-first. Document a new register sequence or firmware command
in the relevant `docs/spec/` track, resolved for RTL8852B with `file:line`
references to rtw89, and implement it from there. See
[CONTRIBUTING.md](../CONTRIBUTING.md).
