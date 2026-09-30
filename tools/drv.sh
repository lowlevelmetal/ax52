#!/bin/bash
# Swap the RTL8852BE between the stock rtw89 driver and the new ax52 driver.
# Must run as root:  sudo tools/drv.sh {load|unload|restore|status}
set -u
DEV=${AX52_PCI:-$(lspci -Dn -d 10ec:b852 | awk 'NR==1{print $1}')}
[ -n "$DEV" ] || { echo "no RTL8852BE (10ec:b852) found; set AX52_PCI"; exit 1; }
SYS=/sys/bus/pci/devices/$DEV
KO="$(cd "$(dirname "$0")/.." && pwd)/src/ax52.ko"
STOCK=rtw89_8852be

cur_driver() { if [ -L $SYS/driver ]; then basename "$(readlink $SYS/driver)"; else echo none; fi; }

case "${1:-status}" in
load)
	[ -f "$KO" ] || { echo "missing $KO (build first)"; exit 1; }
	[ "$(uname -r)" = "$(modinfo -F vermagic "$KO" | cut -d' ' -f1)" ] || { echo "vermagic mismatch: module built for another kernel"; exit 1; }
	d=$(cur_driver); [ "$d" != none ] && echo "$DEV" > /sys/bus/pci/drivers/$d/unbind
	echo ax52 > $SYS/driver_override
	lsmod | grep -q '^ax52 ' && rmmod ax52
	insmod "$KO" "${@:2}" || exit 1
	echo "bound to: $(cur_driver)" ;;
unload)
	lsmod | grep -q '^ax52 ' && rmmod ax52
	echo "bound to: $(cur_driver)" ;;
restore)
	lsmod | grep -q '^ax52 ' && rmmod ax52
	echo > $SYS/driver_override
	echo 1 > $SYS/reset 2>/dev/null && echo "function reset done"
	modprobe $STOCK
	[ "$(cur_driver)" = none ] && echo "$DEV" > /sys/bus/pci/drivers_probe
	echo "bound to: $(cur_driver)" ;;
status)
	echo "bound to: $(cur_driver)  override: $(cat $SYS/driver_override)"
	lsmod | grep -E '^(ax52|rtw89)' ;;
*) echo "usage: $0 {load [params]|unload|restore|status}"; exit 1 ;;
esac
