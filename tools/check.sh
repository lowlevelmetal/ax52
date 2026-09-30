#!/bin/bash
# Snapshot of the ax52 test state (no root needed).
# usage: tools/check.sh [since]   e.g. tools/check.sh "5 min ago"
since="${1:-10 min ago}"
PCI=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
dev=$(ls /sys/bus/pci/devices/$PCI/net 2>/dev/null | head -1)
echo "== driver: $(basename "$(readlink /sys/bus/pci/devices/$PCI/driver 2>/dev/null)" 2>/dev/null)  netdev: ${dev:-none}"
if [ -n "$dev" ]; then
	ip -br link show "$dev"
	iw dev "$dev" info 2>/dev/null | grep -E 'channel|txpower|type'
	iw dev "$dev" link 2>/dev/null
	nmcli -f GENERAL.STATE,GENERAL.CONNECTION dev show "$dev" 2>/dev/null
fi
echo "== kernel log (ax52, warnings, oopses) since $since"
journalctl -k --since "$since" --no-pager -o short-monotonic 2>/dev/null |
	grep -aE 'ax52|BUG|WARNING|Oops|Call Trace|IO_PAGE_FAULT|AER' | tail -${LINES_MAX:-60}
