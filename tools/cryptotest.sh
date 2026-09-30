#!/bin/bash
# Download throughput + CPU cost over one interface (Linode Fremont, 4 x 100 MB).
# usage: tools/cryptotest.sh [iface] [reps]
PCI=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
IF=${1:-$(ls /sys/bus/pci/devices/$PCI/net 2>/dev/null | head -1)}; REPS=${2:-3}; T=$(mktemp -d); trap 'rm -rf $T' EXIT
cpu() { awk '/^cpu /{print $2+$3+$4+$6+$7+$8, $5+$6, $7+$8}' /proc/stat; }	# busy idle softirq
echo "iface $IF, driver: $(basename "$(readlink /sys/class/net/$IF/device/driver)")"
for rep in $(seq 1 $REPS); do
	read b0 i0 s0 < <(cpu); t0=$(date +%s.%N)
	for i in 1 2 3 4; do
		curl -s --interface $IF -o /dev/null -w "%{size_download}\n" --max-time 60 \
			https://speedtest.fremont.linode.com/100MB-fremont.bin > $T/l$i &
	done
	wait; t1=$(date +%s.%N); read b1 i1 s1 < <(cpu)
	cat $T/l? | awk -v t0=$t0 -v t1=$t1 -v db=$((b1-b0)) -v di=$((i1-i0)) -v ds=$((s1-s0)) -v r=$rep \
		'{b+=$1} END{printf "rep %d: %6.1f Mbps   CPU busy %4.1f%% (softirq %4.1f%%)\n", r, b*8/1e6/(t1-t0), 100*db/(db+di), 100*ds/(db+di)}'
done
