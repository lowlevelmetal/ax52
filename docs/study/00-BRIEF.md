# Study brief (shared by all study tracks)

Goal: document the RTL8852BE hardware programming model precisely enough that a
brand-new, independently structured Linux driver can be written from these notes
(without copy-pasting rtw89 code). rtw89 is dual-licensed GPL-2.0 OR BSD-3-Clause.

## Target hardware (this machine)
- PCI 0000:06:00.0, ID 10ec:b852, subsystem 105b:e111 (Foxconn)
- Chip: RTL8852B, gen AX, HCI PCIe. dmesg: "chip info CID: 0, CV: 1, AID: 0, ACV: 1, RFE: 1"
  (CV 1 => CHIP_CBV / B-cut). 2T2R, 2.4/5 GHz, HE (802.11ax), BT on same chip (BT is USB 0489:e123).
- Firmware file: /usr/lib/firmware/rtw89/rtw8852b_fw-2.bin.zst (zstd-compressed; loaded FW 0.29.29.18)
  The -2 file contains "elements": BB, radio A/B, NCTL, TXPWR (x3), PWR_TRK, REGD tables.
- Platform quirk: user runs rtw89 with ASPM L1, L1SS, CLKREQ and PS mode disabled (keep those off).
- Target kernel 7.2.7 (Arch). Reference source: reference/linux-v7.2.7/drivers/net/wireless/realtek/rtw89/

## Rules for study notes
- Follow ONLY the path taken for RTL8852B + PCIe + CV_B (CHIP_CBV) + RFE 1. rtw89 abstracts across
  8851B/8852A/8852B/8852BT/8852C/8922A via chip_info/ops/gen_def tables: resolve every indirection to
  the concrete function/value used for 8852B, and say which one it is.
- Resolve macros to NUMERIC register addresses and bit masks (e.g. R_AX_SYS_ISO_CTRL = 0x0000,
  B_AX_PWC_EV2EF_S = BIT(14)). Write ordered sequences as numbered steps with exact reg/mask/value
  and any polling (reg, mask, expected, timeout).
- Document data structure layouts (descriptors, H2C/C2H headers, FW file headers) as dword/bit tables.
- Note what is optional/skippable for a minimal station-mode driver vs mandatory for the HW to work.
- Cite source locations as file:line for every non-trivial claim so it can be re-checked.
- Do not paste large verbatim code blocks; describe behavior, list values. Tables of register
  values are fine (they are hardware facts).
- Out of scope unless stated: AP/P2P/mesh, WoWLAN, MCC/MLO, USB, SER recovery, debugfs, SAR/ACPI/TAS.
