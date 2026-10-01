# RTL8852BE hardware specification

This directory specifies how to program the RTL8852BE: the power sequences,
firmware, DMA, firmware commands, PHY, RF calibration and mac80211 integration
that a station-mode driver needs. ax52 is implemented from it. Every non-trivial
statement cites the rtw89 sources of Linux v7.2.7 as `file:line`, so it can be
re-checked (see the checkout command in the [top-level README](../../README.md)).

## Documents

| Track | Document | Covers |
|---|---|---|
| 01 | [PCIe and DMA](01-pci-dma.md) | Probe, register access, DMA channels, descriptors, rings, interrupts, teardown |
| 02 | [Power and MAC init](02-power-mac-init.md) | Power on/off, XTAL-SI, packet buffer and flow control, DMAC/CMAC init, efuse, port setup, BT coexistence registers |
| 03 | [Firmware and download](03-firmware-fwdl.md) | Firmware container, image headers, elements (tables), the download procedure |
| 04 | [H2C and C2H](04-h2c-c2h.md) | Firmware command transport, acknowledgements, every command a station uses, event formats |
| 05 | [PHY, channel and TX power](05-phy-chan-txpwr.md) | BB/RF access, PHY tables, channel switching, TX power, periodic PHY work |
| 06 | [RF calibration](06-rfk.md) | RCK, DACK, RX DCK, IQK, DPK, TSSI, synthesizer lock, thermal tracking |
| 07 | [TX/RX data path](07-txrx-data.md) | TX and RX descriptors, PPDU status, aggregation, security, TX status |
| 08 | [mac80211 orchestration](08-mac80211-orchestration.md) | Probe/start/stop order, `ieee80211_ops`, capabilities, regulatory, locking |

[`fwdump/parse_fw.py`](fwdump/parse_fw.py) is an independent parser of the firmware
file written from track 03, used to validate it; its output on the reference
firmware is in [`fwdump/parse_fw_output.txt`](fwdump/parse_fw_output.txt).

## Scope

- **Chip:** RTL8852B (802.11ax, 2T2R, 2.4 and 5 GHz), AX generation, PCIe
  (`10ec:b852`, `10ec:b85b`). rtw89
  abstracts over 8851B, 8852A/B/BT/C and 8922A through `chip_info`, ops and
  `gen_def` tables. Every such indirection is resolved here to the concrete function
  or value used for 8852B, and named.
- **Configuration:** chip cut B (`CV 1`, `CHIP_CBV`) and RFE type 1. Paths taken only
  for other cuts or RFE types are noted where they differ.
- **Firmware:** `rtw89/rtw8852b_fw-2.bin` from linux-firmware (installed
  zstd-compressed as `rtw8852b_fw-2.bin.zst`), version 0.29.29.18. Its elements carry
  the BB, radio A/B, NCTL, TX-power (three variants), power-tracking and REGD tables.
- **Out of scope** unless a document says otherwise: AP, P2P and mesh, WoWLAN,
  MCC/MLO, USB, the SER recovery state machine, debugfs, SAR/ACPI/TAS.

## Reference system

Statements about the **reference system** were observed on this configuration:

| Item | Value |
|---|---|
| Device | PCI `0000:06:00.0`, `10ec:b852`, subsystem `105b:e111` (Foxconn) |
| Chip | `CID 0, CV 1, AID 0, ACV 1, RFE 1` as reported by rtw89 |
| Bluetooth | On the same chip, exposed as USB `0489:e123` |
| PCIe | Upstream bridge AMD `1022:14ba`, link Gen1 x1 |
| Software | Linux 7.2.7; rtw89 loaded with `disable_aspm_l1=Y disable_aspm_l1ss=Y disable_clkreq=Y disable_ps_mode=Y` |

ASPM L1, L1SS, CLKREQ and power saving therefore stay off throughout.

## Conventions

- Registers are given as numeric addresses and masks, with the rtw89 name kept for
  cross-reference, e.g. `R_AX_SYS_ISO_CTRL = 0x0000`, `B_AX_PWC_EV2EF_S = BIT(14)`.
  BB registers are BB-relative (MMIO address = BB address + `0x10000`).
- Ordered sequences are numbered steps with the exact register, mask and value.
  Polls give the register, mask, expected value, interval and timeout.
- Data structures (descriptors, H2C/C2H headers, firmware headers) are dword/bit
  tables. All fields are little-endian.
- Each document says what a minimal station-mode driver must implement and what is
  optional.
- The specification describes behaviour and lists values; it does not reproduce
  rtw89 code. Register values and table contents are hardware facts.
- Each document ends with its open questions and unverified assumptions.
