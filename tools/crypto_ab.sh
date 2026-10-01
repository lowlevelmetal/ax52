#!/bin/bash
# Run as root: sudo WIFI_CON=... WIFI_BSSID=... tools/crypto_ab.sh
# HW crypto vs SW crypto (ax52) vs rtw89, interleaved with an
# ethernet reference so internet variance can be factored out. Ends on rtw89.
cd "$(dirname "$0")/.." || exit 1
PCI=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
IF=${WIFI_IF:-$(ls /sys/bus/pci/devices/$PCI/net 2>/dev/null | head -1)}
CON=${WIFI_CON:?set WIFI_CON to the NetworkManager connection name}
AP=${WIFI_BSSID:?set WIFI_BSSID to the access point to pin (lowercase)}
wait_assoc() {
	for _ in $(seq 1 90); do
		ip -4 addr show $IF 2>/dev/null | grep -q inet && iw dev $IF link 2>/dev/null | grep -q Connected && break
		sleep 1
	done
	iw dev $IF link | grep -qi $AP || { nmcli --wait 30 con up id $CON ifname $IF ap ${AP^^} >/dev/null; sleep 3; }
	iw dev $IF link | grep -E 'Connected|freq' | tr -s '\t ' ' '
}
phase() {
	echo "=== $1"
	for _ in 1 2 3; do
		tools/cryptotest.sh eno1 1 | tail -1 | sed "s/^rep 1/  eth  /"
		tools/cryptotest.sh $IF 1 | tail -1 | sed "s/^rep 1/  wifi /"
	done
}
tools/drv.sh load >/dev/null && wait_assoc && phase "ax52, hardware crypto"
echo 1 > /sys/module/ax52/parameters/swcrypto
nmcli --wait 30 con up id $CON ifname $IF ap ${AP^^} >/dev/null; sleep 3
wait_assoc && phase "ax52, software crypto"
tools/drv.sh restore >/dev/null && wait_assoc && phase "rtw89 (stock)"
echo "=== key log"
journalctl -k --since "15 min ago" -o cat | grep -a "hardware key"
