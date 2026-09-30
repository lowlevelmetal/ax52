// SPDX-License-Identifier: GPL-2.0
/*
 * Efuse (one-time-programmable calibration memory) readout.
 *
 * The physical efuse is a packed log of 8-byte block writes; it is decoded
 * into a 2 KiB "logical" map holding the MAC address, RFE type, crystal
 * trim and TX-power calibration. A separate physical area (0x580..0x5ff)
 * carries per-unit PHY trims.
 */
#include <linux/delay.h>

#include "ax52.h"

/* logical map offsets (PCIe variant) */
#define EF_TSSI_A		0x210
#define EF_TSSI_B		0x23A
#define EF_XTAL_K		0x2B9
#define EF_RFE_TYPE		0x2CA
#define EF_COUNTRY		0x2CB
#define EF_THERMAL_A		0x2D0
#define EF_THERMAL_B		0x2D1
#define EF_MAC_ADDR		0x400

/* physical phycap bytes */
#define PHYCAP_POWER_K_CHK	0x5E9

static void efuse_power(struct ax52_dev *rd, bool on)
{
	if (on) {
		set8(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);
		set16(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B14);
		fsleep(1000);
		set16(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B15);
		clr16(rd, REG_SYS_ISO_CTRL, SYS_ISO_EB2CORE);
	} else {
		set16(rd, REG_SYS_ISO_CTRL, SYS_ISO_EB2CORE);
		clr16(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B15);
		fsleep(1000);
		clr16(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B14);
		clr8(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);
	}
}

static int efuse_dump(struct ax52_dev *rd, u32 start, u8 *out, u32 len)
{
	u32 i, v;
	int ret;

	for (i = 0; i < len; i++) {
		wr32(rd, REG_EFUSE_CTRL, (start + i) << EFUSE_CTRL_ADDR_SHIFT);
		ret = read_poll_timeout_atomic(rd32, v, v & EFUSE_CTRL_RDY, 1,
					       1000000, false, rd, REG_EFUSE_CTRL);
		if (ret) {
			ax52_err(rd, "efuse read of 0x%x timed out\n", start + i);
			return ret;
		}
		out[i] = v & 0xff;
	}
	return 0;
}

static int efuse_decode(struct ax52_dev *rd, const u8 *phy, u8 *log)
{
	u32 i = 4;	/* four security-control bytes precede the log */

	memset(log, 0xff, EFUSE_LOG_SIZE);
	while (i + 1 < EFUSE_PHY_SIZE - 4) {
		u8 h1 = phy[i], h2 = phy[i + 1];
		u32 blk, w;

		if (h1 == 0xff || h2 == 0xff)
			break;
		blk = ((h1 & 0x0f) << 4) | (h2 >> 4);
		i += 2;
		for (w = 0; w < 4; w++) {
			u32 lo = blk * 8 + w * 2;

			if (h2 & BIT(w))		/* word-enable is active low */
				continue;
			if (i + 1 >= EFUSE_PHY_SIZE - 4 || lo + 1 >= EFUSE_LOG_SIZE) {
				ax52_err(rd, "corrupt efuse map at 0x%x\n", i);
				return -EINVAL;
			}
			log[lo] = phy[i];
			log[lo + 1] = phy[i + 1];
			i += 2;
		}
	}
	return 0;
}

int ax52_efuse_read(struct ax52_dev *rd)
{
	struct ax52_efuse *ef = &rd->efuse;
	u8 *phy;
	u8 sec[2];
	int ret, tries;

	phy = kmalloc(EFUSE_PHY_SIZE, GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	ef->autoload = rd16(rd, REG_SYS_WL_EFUSE_CTRL) & EFUSE_AUTOLOAD_SUS;

	for (tries = 0; tries < 5; tries++) {
		efuse_power(rd, true);
		ret = efuse_dump(rd, 0, phy, EFUSE_PHY_SIZE) ?:
		      efuse_dump(rd, EFUSE_PHYCAP_ADDR, ef->phycap, EFUSE_PHYCAP_SIZE) ?:
		      efuse_dump(rd, 0x5EC, sec, sizeof(sec));
		efuse_power(rd, false);
		if (!ret)
			break;
	}
	if (ret)
		goto out;

	ret = efuse_decode(rd, phy, ef->log);
	if (ret)
		goto out;

	if (sec[0] != 0xff || sec[1] != 0xff)
		ax52_warn(rd, "efuse secure-boot bytes %02x %02x: firmware signing unsupported\n",
			  sec[0], sec[1]);

	ether_addr_copy(ef->addr, &ef->log[EF_MAC_ADDR]);
	ef->rfe_type = ef->log[EF_RFE_TYPE];
	ef->xtal_cap = ef->log[EF_XTAL_K];
	ef->thermal[0] = ef->log[EF_THERMAL_A];
	ef->thermal[1] = ef->log[EF_THERMAL_B];
	ef->country[0] = ef->log[EF_COUNTRY];
	ef->country[1] = ef->log[EF_COUNTRY + 1];
	ef->country[2] = 0;
	ef->power_k_valid = ef->phycap[PHYCAP_POWER_K_CHK - EFUSE_PHYCAP_ADDR] == 0xAA;

	if (!is_valid_ether_addr(ef->addr)) {
		ax52_warn(rd, "efuse MAC address %pM invalid, using a random one\n",
			  ef->addr);
		eth_random_addr(ef->addr);
	}

	ax52_info(rd, "efuse: autoload %d, MAC %pM, RFE %u, xtal 0x%02x, thermal %u/%u, country \"%.2s\", power-K %s\n",
		  ef->autoload, ef->addr, ef->rfe_type, ef->xtal_cap,
		   ef->thermal[0], ef->thermal[1],
		   (ef->country[0] >= 'A' && ef->country[0] <= 'Z') ? ef->country : "--",
		   ef->power_k_valid ? "valid" : "none");
out:
	kfree(phy);
	return ret;
}
