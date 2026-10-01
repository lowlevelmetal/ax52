# SPDX-License-Identifier: GPL-2.0
# Out-of-tree build for the ax52 driver.
KVER  ?= $(shell uname -r)
KDIR  ?= /usr/lib/modules/$(KVER)/build

ifneq ($(KERNELRELEASE),)
obj-m += ax52.o
ax52-y := main.o pwr.o pci.o mac.o fw.o efuse.o tx.o rx.o mac80211.o phy.o phy_txpwr.o rfk.o rfk_table.o h2c.o
ccflags-y += -Wall -Wno-unused-parameter
# Only main.o carries the version, so a new version rebuilds one file.
CFLAGS_main.o += -DAX52_VERSION=\"$(or $(AX52_VERSION),unknown)\"
else
AX52_VERSION ?= $(shell git -C $(CURDIR) describe --tags --always --dirty 2>/dev/null)
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) AX52_VERSION=$(AX52_VERSION) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
endif
