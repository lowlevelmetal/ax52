# Testing ax52

ax52 programs DMA engines and RF hardware directly. A bug can hang the machine, so
**keep a wired connection and expect to reboot**. `tools/drv.sh` binds ax52 to the
one card through `driver_override` and installs nothing, so a reboot always gives
the card back to rtw89.

## Static checks (no hardware)

```sh
make check
```

This is what CI runs: a full rebuild with `W=1` and `-Werror`, sparse, checkpatch,
shellcheck on `tools/`, and a lint of the firmware parser. It needs the kernel
headers plus `sparse`, `shellcheck` and `ruff` (all packaged on Arch). Individual
targets: `make werror`, `make sparse`, `make checkpatch`, `make shellcheck`.

## Hardware smoke test

Run these steps for any change that touches hardware behaviour, and say in the pull
request which ones you ran. `tools/check.sh` prints the link state and the relevant
kernel log lines without root (`tools/check.sh "5 min ago"`).

1. **Load.** `make && sudo tools/drv.sh load`. The kernel log shows the
   `RTL8852B cut …, ax52 version …` line, the firmware line, the efuse line and
   `radio up` once the interface is up. There must be no errors and no `untested
   hardware` warning on the reference card.
2. **Scan.** `nmcli dev wifi list --rescan yes` lists networks on both 2.4 and
   5 GHz with plausible signal values.
3. **Associate on 5 GHz.** Connect to a WPA2 (and, if available, WPA3) network on
   an 80 MHz channel. `iw dev <if> link` shows an HE (or VHT) rate and DHCP succeeds.
4. **Associate on 2.4 GHz.** Same on a 2.4 GHz network.
5. **Interface cycling.** `nmcli dev disconnect <if>`, then
   `ip link set <if> down` / `up` a few times and reconnect. Each up repeats the
   firmware download and RF calibration, so this exercises start/stop.
6. **Software crypto.** `sudo tools/drv.sh load swcrypto=1` and repeat step 3.
   `swcrypto` can also be flipped at runtime; it applies to keys installed
   afterwards.
7. **Throughput** (when the change can affect it). Compare against rtw89 within the
   same hour; internet endpoints vary a lot:
   - `tools/speedtest.sh`: Cloudflare download/upload, 4 and 8 streams
     (`WIFI_CON=<connection>`, optional BSSID argument to pin an AP).
   - `tools/cryptotest.sh [iface] [reps]`: throughput plus CPU and softirq load.
   - `sudo WIFI_CON=… WIFI_BSSID=… tools/crypto_ab.sh`: ax52 hardware crypto,
     ax52 software crypto and rtw89 back to back, with an Ethernet reference.
8. **Restore.** `sudo tools/drv.sh restore` unloads ax52, resets the function and
   rebinds rtw89.

All scripts find the card with `lspci -d 10ec:b852`; set `AX52_PCI` (and `WIFI_IF`)
if you have more than one, or the `10ec:b85b` variant.

## Kernel log messages that indicate trouble

| Message | Meaning |
|---|---|
| `firmware halted (reason …, UDM0 …)` | The firmware crashed. It is followed by `firmware failure` and a radio restart |
| `FWCMD ring stuck` | The firmware stopped consuming commands; treated as a firmware failure |
| `TX scheduler … not acked by firmware` | The register mailbox got no answer; the driver wrote the register itself. A timeout is treated as a firmware failure |
| `restarting the radio` | Recovery after a firmware failure: mac80211 restarts the device and reconnects |
| `firmware failed 4 times within 60 s` | Recovery gave up; restart the interface (`ip link set <if> down` / `up`) |
| `DMA stuck, isr …` | The DMA engine reported a stuck TX/RX channel (logged only, as in rtw89) |
| `… TX frames got no release report, dropped` | Release reports were lost; the frames were completed as dropped |
| `H2C …/…/… (seq …) failed in firmware` | The firmware rejected a command (done-ack with an error code) |
| `synthesizer not locked on channel …` | The RF PLL did not lock even after the recovery steps |
| `IO_PAGE_FAULT`, `AER`, `BUG`, `Oops`, `WARNING` | Always a driver bug; please report with the full log |

## Debug output

Debug messages (RF calibration results, unhandled firmware events, firmware log
records) use `dev_dbg`. Enable them with dynamic debug:

```sh
echo 'module ax52 +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
```

The firmware's own log is only sent when the module is loaded with `fw_log=1`
(`sudo tools/drv.sh load fw_log=1`). Records that need the firmware's format strings
are printed raw.

## Tested configurations

| Card | PCI ID (subsystem) | Chip cut / RFE | Kernel | Firmware | Result |
|---|---|---|---|---|---|
| Foxconn RTL8852BE | 10ec:b852 (105b:e111) | B / 1 | 7.2.7 | 0.29.29.18 | Works: scan, WPA2, 2.4 GHz 20/40, 5 GHz 20/40/80, HE-MCS 11 |

The driver warns at probe on any chip cut or RFE type other than B / 1. If you run
it on other hardware, please file a
[hardware test report](https://github.com/lowlevelmetal/ax52/issues/new?template=hardware_report.yml),
whether it works or not.
