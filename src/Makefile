# SPDX-License-Identifier: GPL-2.0
# Out-of-tree build for the ax52 driver.
KVER  ?= $(shell uname -r)
KDIR  ?= /usr/lib/modules/$(KVER)/build

ifneq ($(KERNELRELEASE),)
obj-m += ax52.o
ax52-y := main.o pwr.o pci.o mac.o fw.o efuse.o tx.o rx.o mac80211.o phy.o phy_txpwr.o rfk.o rfk_table.o h2c.o
ccflags-y += -Wall -Wno-unused-parameter
else
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
endif
