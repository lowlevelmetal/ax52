#!/bin/bash
# Driver A/B throughput over an interface using two non-throttling mirrors.
# usage: tools/abtest.sh [iface]
PCI=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
IF=${1:-$(ls /sys/bus/pci/devices/$PCI/net 2>/dev/null | head -1)}; T=$(mktemp -d); trap 'rm -rf $T' EXIT
echo "iface $IF, driver: $(basename "$(readlink /sys/class/net/$IF/device/driver)")"
for rep in 1 2 3; do
	# Vultr LAX: one stream for 10 s
	curl -s --interface $IF -o /dev/null -w "%{size_download}\n" --max-time 10 \
		https://lax-ca-us-ping.vultr.com/vultr.com.1000MB.bin > $T/v
	# Linode Fremont: 4 x 100 MB, time to completion
	s=$(date +%s.%N)
	for i in 1 2 3 4; do
		curl -s --interface $IF -o /dev/null -w "%{size_download}\n" --max-time 30 \
			https://speedtest.fremont.linode.com/100MB-fremont.bin > $T/l$i &
	done
	wait; e=$(date +%s.%N)
	printf "rep %d: vultr 1-stream %6.1f Mbps | linode 4-stream %6.1f Mbps\n" $rep \
		"$(awk '{print $1*8/1e6/10}' $T/v)" \
		"$(cat $T/l? | awk -v s=$s -v e=$e '{b+=$1} END{print b*8/1e6/(e-s)}')"
done
