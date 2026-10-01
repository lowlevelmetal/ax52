# ax52

[![CI](https://github.com/lowlevelmetal/ax52/actions/workflows/ci.yml/badge.svg)](https://github.com/lowlevelmetal/ax52/actions/workflows/ci.yml)

ax52 is an independent Linux driver for the **Realtek RTL8852BE** (PCIe 802.11ax,
2T2R). It is a separate implementation from the in-kernel `rtw89` driver: a
compact, station-only code base (~10,500 lines) built from a documented hardware
specification ([`docs/spec/`](docs/spec/)). Every register sequence in that
specification is resolved for this chip and traced to its source.

rtw89 also supports this chip. ax52 can take over a single device at runtime and
hand it back (`tools/drv.sh`), so switching between the two needs no reinstall.

> **Status: experimental.** ax52 has been tested on one hardware configuration
> (see [Hardware](#hardware)). It programs DMA and RF hardware directly, so a driver
> bug can hang the machine: keep a wired connection while testing, and expect to
> reboot.

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

## Hardware

ax52 binds the same PCI IDs as rtw89's 8852BE driver: `10ec:b852` and `10ec:b85b`.
Only one configuration has been tested: chip cut B with RFE type 1 (see the table
in [docs/TESTING.md](docs/TESTING.md)). On any other cut or RFE type the driver
logs an "untested hardware" warning. Reports from such cards, working or not, are
very welcome as a
[hardware test report](https://github.com/lowlevelmetal/ax52/issues/new?template=hardware_report.yml).

## Building and trying it

Requirements:

- Kernel headers for Linux 7.2, the version ax52 is developed and tested on. It uses
  recent mac80211 interfaces (for example the `radio_idx` argument of `.config`), so
  older kernels will not build it. CI also builds against the newest Arch kernel
  every week.
- `rtw89/rtw8852b_fw-2.bin` from linux-firmware. ax52 uses the same firmware as
  rtw89 and takes its PHY and TX-power tables from that file.

```sh
make                         # builds src/ax52.ko for the running kernel
sudo tools/drv.sh load       # unbind rtw89 from the card, load ax52
tools/check.sh               # link state + kernel log (no root needed)
sudo tools/drv.sh restore    # unload ax52, reset the card, rebind rtw89
```

`drv.sh` binds ax52 only to the one device (via `driver_override`) and never
installs anything, so a reboot always returns the card to rtw89. Use
`make KVER=<version>` to build for another installed kernel, and `make check` to run
the same static checks as CI.

Module parameters (`sudo tools/drv.sh load swcrypto=1`):

| Parameter | Default | Effect |
|---|---|---|
| `swcrypto` | 0 | Keep all keys in mac80211 software crypto. Writable at runtime; applies to keys installed afterwards. |
| `fw_log` | 0 | Enable firmware log events, printed with dynamic debug. Load time only. |

Benchmark helpers (throughput is measured over the Wi-Fi interface only):
`tools/abtest.sh`, `tools/cryptotest.sh`, `tools/speedtest.sh`
(`WIFI_CON=<NetworkManager connection>`), and `tools/crypto_ab.sh`
(`sudo WIFI_CON=… WIFI_BSSID=… tools/crypto_ab.sh` runs HW crypto vs SW crypto
vs rtw89 back to back, ending on rtw89). [docs/TESTING.md](docs/TESTING.md) has the
full test procedure.

## Reporting problems

Use the [bug report form](https://github.com/lowlevelmetal/ax52/issues/new?template=bug_report.yml).
Include the output of `tools/check.sh "30 min ago"` right after the problem. It
prints the driver version, link state and the relevant kernel log lines. For
security issues, see [SECURITY.md](SECURITY.md).

## Documentation

| Document | Contents |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | How the driver is organised: lifecycle, contexts and locking, data path, firmware interface |
| [docs/TESTING.md](docs/TESTING.md) | Static checks, the hardware test procedure, debug output, tested configurations |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Spec-first workflow, style, commits |
| [docs/spec/](docs/spec/) | The hardware specification ax52 implements, one document per subsystem |

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
| `docs/spec/` | Hardware specification (8 tracks) and a firmware-file parser |
| `tools/` | Driver swapping, status and benchmark scripts |
| `Makefile` | Top-level build and `make check` (the kbuild file is `src/Makefile`) |
| `.github/` | CI workflow, issue forms, pull request template |

## Design

ax52 is developed specification-first. [`docs/spec/`](docs/spec/) documents the
RTL8852BE programming model: power sequences, firmware format and download, DMA
descriptors, firmware commands, PHY tables and channel programming, RF calibration
and the mac80211 integration. It is resolved for this chip and cross-referenced
(`file:line`) to rtw89 in Linux v7.2.7. The driver implements that specification in
its own structure ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)). Register values,
table data and firmware command layouts are hardware facts shared with rtw89.

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
