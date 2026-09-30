#!/bin/bash
# Throughput over the Wi-Fi interface only (ethernet stays the default route).
# usage: WIFI_CON=<connection> tools/speedtest.sh [bssid]   -- optionally pin a BSSID first
PCI=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
IF=${WIFI_IF:-$(ls /sys/bus/pci/devices/$PCI/net 2>/dev/null | head -1)}
CON=${WIFI_CON:?set WIFI_CON to the NetworkManager connection name}
T=$(mktemp -d); trap 'rm -rf $T' EXIT

for i in $(seq 1 60); do		# wait for association + DHCP
	ip -4 addr show $IF 2>/dev/null | grep -q inet && iw dev $IF link | grep -q Connected && break
	sleep 1
done
if [ -n "$1" ] && ! iw dev $IF link | grep -qi "$1"; then
	nmcli --wait 30 con up id "$CON" ifname $IF ap "$1" >/dev/null
	sleep 3
fi
echo "driver: $(basename "$(readlink /sys/class/net/$IF/device/driver)")"
iw dev $IF link | grep -E 'Connected|freq|signal'

run() {	# $1 = label, $2 = streams, $3 = MB each, $4 = up|down
	local s e i
	s=$(date +%s.%N)
	for i in $(seq 1 $2); do
		if [ "$4" = up ]; then
			curl -s --interface $IF -o /dev/null -w '%{size_upload}\n' \
				--data-binary @$T/up.bin https://speed.cloudflare.com/__up > $T/r$i &
		else
			curl -s --interface $IF -o /dev/null -w '%{size_download}\n' \
				"https://speed.cloudflare.com/__down?bytes=$(($3 * 1000000))" > $T/r$i &
		fi
	done
	wait; e=$(date +%s.%N)
	cat $T/r* | awk -v s=$s -v e=$e -v l="$1" '{b+=$1} END{printf "%-22s %7.1f Mbps\n", l, b*8/1e6/(e-s)}'
	rm -f $T/r*
}
head -c 25000000 /dev/urandom > $T/up.bin
for rep in 1 2; do
	run "download 4x50MB #$rep" 4 50 down
	run "download 8x50MB #$rep" 8 50 down
	run "upload 4x25MB #$rep" 4 25 up
done
iw dev $IF link | grep -E 'bitrate'
