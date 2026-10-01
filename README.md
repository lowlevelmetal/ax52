# ax52

An independent Linux driver for the **Realtek RTL8852BE** (PCIe 802.11ax, 2T2R,
PCI ID `10ec:b852`), written from scratch as a learning project.

The upstream `rtw89` driver already supports this chip well; ax52 is not meant to
replace it. It exists to understand the hardware end to end: every register
sequence was first written down as a specification (`docs/spec/`), then
implemented in a new, much smaller code base (~10,500 lines, station mode only).

> **Status: experimental.** It works on the author's hardware, but it drives DMA
> and RF hardware directly — a bug can hang the machine. Keep a wired connection
> and expect to reboot.

## What works

Tested on an RTL8852BE (cut B, RFE type 1) with kernel 7.2.7 and firmware
0.29.29.18:

- Scan, WPA2-Personal association, DHCP, normal use via NetworkManager
  (WPA3/SAE should work — it uses the same CCMP path — but is untested)
- 2.4 GHz (20/40 MHz) and 5 GHz (20/40/80 MHz), HE/VHT/HT, 2 spatial streams
- Firmware rate adaptation (reaches HE-MCS 11, 1201 Mbps link rate), TX/RX A-MPDU
- Hardware CCMP/GCMP encryption (TKIP, WEP and management-frame protection in software)
- Full host-side RF calibration: RCK, DACK, RX DCK, IQK, TSSI, DPK + DPK tracking
- TX power with per-country regulatory limits (from the firmware file's tables)

**Measured** (5 GHz, 80 MHz, same access point radio, same hour):

| Test | ax52 (HW crypto) | ax52 (SW crypto) | rtw89 |
|---|---|---|---|
| Linode Fremont, 4 × 100 MB | ~324 Mbps | ~255 Mbps | ~196 Mbps |
| Cloudflare, 4–8 streams | 599–680 Mbps | — | — |

Internet test endpoints are noisy and rate-limit aggressively; treat ax52 and
rtw89 as the same performance class, not one as faster.

**Not implemented:** AP/P2P/monitor modes, power saving, suspend/resume, WoWLAN,
beamformee, TX A-MSDU, hardware BIP, full Bluetooth coexistence (while ax52 is on
2.4 GHz the on-chip Bluetooth loses the shared antenna; on 5 GHz it is unaffected).

## Building and trying it

Requirements: kernel headers for the running kernel, and
`rtw89/rtw8852b_fw-2.bin` from linux-firmware (ax52 uses the same firmware as rtw89).

```sh
make -C src                  # builds src/ax52.ko for the running kernel
sudo tools/drv.sh load       # unbind rtw89 from the card, load ax52
tools/check.sh               # link state + kernel log (no root needed)
sudo tools/drv.sh restore    # unload ax52, reset the card, rebind rtw89
```

`drv.sh` binds ax52 only to the one device (via `driver_override`) and never
installs anything, so a reboot always returns the card to rtw89. Module
parameter `swcrypto=1` keeps all keys in software; `fw_log=1` enables firmware
log events.

Benchmark helpers (throughput is measured over the Wi-Fi interface only):
`tools/abtest.sh`, `tools/cryptotest.sh`, `tools/speedtest.sh`
(`WIFI_CON=<NetworkManager connection>`), and `tools/crypto_ab.sh`
(`sudo WIFI_CON=… WIFI_BSSID=… tools/crypto_ab.sh` runs HW crypto vs SW crypto
vs rtw89 back to back, ending on rtw89).

## Layout

| Path | What |
|---|---|
| `src/main.c` | PCI probe/remove, interface start/stop orchestration |
| `src/pci.c` | Buffer-descriptor rings, DMA bring-up, interrupts/NAPI, H2C channel |
| `src/pwr.c` | Power sequencing, XTAL-SI bus, firmware CPU control |
| `src/fw.c` | Firmware container parsing, download, PHY/TX-power tables ("elements") |
| `src/efuse.c` | Efuse readout (MAC address, RFE type, calibration trims) |
| `src/mac.c` | Packet buffer, flow control, DMAC/CMAC init, port config, BT coexistence |
| `src/phy.c`, `src/phy_txpwr.c` | BB/RF access, PHY tables, channel switching, TX power |
| `src/rfk.c`, `src/rfk_table.c` | RF calibration |
| `src/h2c.c` | Firmware command protocol (H2C/C2H) |
| `src/tx.c`, `src/rx.c` | Frame descriptors, TX scheduling/status, RX status, signal strength |
| `src/mac80211.c` | mac80211 operations and capabilities |
| `docs/spec/` | The hardware specification ax52 was written from (8 subsystems) |
| `tools/` | Driver swapping, status and benchmark scripts |

## How it was made

`docs/spec/` documents the RTL8852BE programming model — power sequences,
firmware format and download, DMA descriptors, firmware commands, PHY tables and
channel programming, RF calibration and the mac80211 integration — resolved
specifically for this chip and cross-referenced (`file:line`) to rtw89 in Linux
v7.2.7. The driver was then written against that specification with its own
structure; register values, table data and firmware command layouts are hardware
facts shared with rtw89.

To follow the citations, check out the referenced sources:

```sh
git clone --depth 1 --filter=blob:none --sparse --branch v7.2.7 \
    https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git reference/linux-v7.2.7
git -C reference/linux-v7.2.7 sparse-checkout set --no-cone \
    /drivers/net/wireless/realtek/rtw89/ /include/net/mac80211.h
```

## Credits and license

The hardware knowledge in this project comes from Realtek's `rtw89` driver
(Copyright © Realtek Corporation, `GPL-2.0 OR BSD-3-Clause`). ax52 is not
affiliated with or endorsed by Realtek.

ax52 is licensed under the GNU General Public License v2.0 (`LICENSE`).
