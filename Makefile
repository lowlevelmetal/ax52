# SPDX-License-Identifier: GPL-2.0
#
# Convenience targets; the kbuild Makefile of the module is src/Makefile.
#
#   make               build src/ax52.ko for the running kernel
#   make KVER=<ver>    ... for another installed kernel (or KDIR=<build dir>)
#   make check         everything CI runs (needs sparse, shellcheck, ruff)

KVER       ?= $(shell uname -r)
KDIR       ?= /usr/lib/modules/$(KVER)/build
CHECK      ?= sparse
CHECKPATCH ?= $(KDIR)/scripts/checkpatch.pl

C_SOURCES  := $(filter-out src/%.mod.c,$(wildcard src/*.c src/*.h))

.PHONY: all clean check werror sparse checkpatch shellcheck pylint

all:
	$(MAKE) -C src KDIR=$(KDIR)

clean:
	$(MAKE) -C src KDIR=$(KDIR) clean

check: werror sparse checkpatch shellcheck pylint

# Full rebuild, so every file sees the extra warnings.
werror:
	$(MAKE) -C src KDIR=$(KDIR) clean
	$(MAKE) -C src KDIR=$(KDIR) W=1 KCFLAGS=-Werror

# C=2 checks every file, rebuilt or not.
sparse:
	$(MAKE) -C src KDIR=$(KDIR) C=2 CHECK="$(CHECK)"

checkpatch:
	perl $(CHECKPATCH) --no-tree --terse --file $(C_SOURCES)

shellcheck:
	shellcheck -S warning tools/*.sh

pylint:
	ruff check --isolated --select E9,F docs/spec/fwdump/parse_fw.py
