// SPDX-License-Identifier: GPL-2.0
/*
 * Register-access helpers, the XTAL-SI indirect bus, chip power on/off and
 * WCPU (firmware CPU) control.
 */
#include <linux/delay.h>

#include "ax52.h"

/* CMAC registers read back 0xdeadbeef while their clocks are gated. */
u32 ax52_rd32_cmac(struct ax52_dev *rd, u32 addr)
{
	u32 v = 0xdeadbeef;
	int i;

	for (i = 0; i < 10 && v == 0xdeadbeef; i++) {
		writel(0xffffffff, rd->mmio + 0xC004);	/* CMAC all-clock enable */
		v = readl(rd->mmio + addr);
	}
	return v;
}

int ax52_poll32(struct ax52_dev *rd, u32 addr, u32 mask, u32 want,
		u32 interval_us, u32 timeout_us)
{
	u32 v;

	return read_poll_timeout(rd32, v, (v & mask) == want, interval_us,
				 timeout_us, false, rd, addr);
}

int ax52_poll8(struct ax52_dev *rd, u32 addr, u8 mask, u8 want,
	       u32 interval_us, u32 timeout_us)
{
	u8 v;

	return read_poll_timeout(rd8, v, (v & mask) == want, interval_us,
				 timeout_us, false, rd, addr);
}

/* ------------------------------------------------------------ XTAL-SI */

int ax52_xsi_write(struct ax52_dev *rd, u8 off, u8 val, u8 mask)
{
	int ret;

	wr32(rd, REG_WLAN_XTAL_SI_CTRL, XSI_CMD_POLL |
	     FIELD_PREP(XSI_MODE_MASK, 0) |
	     FIELD_PREP(XSI_BITMASK_MASK, mask) |
	     FIELD_PREP(XSI_DATA_MASK, val) |
	     FIELD_PREP(XSI_ADDR_MASK, off));
	ret = ax52_poll32(rd, REG_WLAN_XTAL_SI_CTRL, XSI_CMD_POLL, 0, 50, 50000);
	if (ret)
		ax52_err(rd, "xtal-si write 0x%02x timed out\n", off);
	return ret;
}

int ax52_xsi_read(struct ax52_dev *rd, u8 off, u8 *val)
{
	int ret;

	wr32(rd, REG_WLAN_XTAL_SI_CTRL, XSI_CMD_POLL |
	     FIELD_PREP(XSI_MODE_MASK, 1) | FIELD_PREP(XSI_ADDR_MASK, off));
	ret = ax52_poll32(rd, REG_WLAN_XTAL_SI_CTRL, XSI_CMD_POLL, 0, 50, 50000);
	if (ret) {
		ax52_err(rd, "xtal-si read 0x%02x timed out\n", off);
		return ret;
	}
	*val = rd32_mask(rd, REG_WLAN_XTAL_SI_CTRL, XSI_DATA_MASK);
	return 0;
}

static inline int xsi_set(struct ax52_dev *rd, u8 off, u8 bits)
{
	return ax52_xsi_write(rd, off, bits, bits);
}

static inline int xsi_clr(struct ax52_dev *rd, u8 off, u8 bits)
{
	return ax52_xsi_write(rd, off, 0, bits);
}

/* ------------------------------------------------------------- power */

static int __power_on(struct ax52_dev *rd)
{
	struct ax52_efuse *ef = &rd->efuse;
	int ret;

	clr32(rd, REG_SYS_PW_CTRL, PW_AFSM_WLSUS_EN | PW_AFSM_PCIE_SUS_EN);
	set32(rd, REG_SYS_PW_CTRL, PW_DIS_WLBT_PDNSUSEN_SOPC);
	set32(rd, REG_WLLPS_CTRL, WLLPS_DIS_WLBT_LPSEN_LOPC);
	clr32(rd, REG_SYS_PW_CTRL, PW_APDM_HPDN);
	clr32(rd, REG_SYS_PW_CTRL, PW_APFM_SWLPS);

	ret = ax52_poll32(rd, REG_SYS_PW_CTRL, PW_RDY_SYSPWR, PW_RDY_SYSPWR,
			  1000, 20000);
	if (ret)
		return ret;

	set32(rd, REG_AFE_LDO_CTRL, AFE_AON_OFF_PC_EN);
	ret = ax52_poll32(rd, REG_AFE_LDO_CTRL, AFE_AON_OFF_PC_EN,
			  AFE_AON_OFF_PC_EN, 1000, 20000);
	if (ret)
		return ret;

	mask32(rd, REG_SPS_DIG_OFF_CTRL0, SPS_OFF_C1_L1_MASK, 1);
	mask32(rd, REG_SPS_DIG_OFF_CTRL0, SPS_OFF_C3_L1_MASK, 3);

	set32(rd, REG_SYS_PW_CTRL, PW_EN_WLON);
	set32(rd, REG_SYS_PW_CTRL, PW_APFN_ONMAC);
	ret = ax52_poll32(rd, REG_SYS_PW_CTRL, PW_APFN_ONMAC, 0, 1000, 20000);
	if (ret)
		return ret;

	set8(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
	clr8(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
	set8(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
	clr8(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
	set8(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);

	clr32(rd, REG_SYS_SDIO_CTRL, SDIO_PCIE_CALIB_EN_V1);

	set32(rd, REG_SYS_ADIE_PAD_PWR_CTRL, PAD_SYM_PADPDN_WL_PTA_1P3);
	ret = xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_GND_SHDN_WL);
	if (ret)
		return ret;
	set32(rd, REG_SYS_ADIE_PAD_PWR_CTRL, PAD_SYM_PADPDN_WL_RFC_1P3);

	ret = xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_SHDN_WL) ?:
	      xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_OFF_WEI) ?:
	      xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_OFF_EI) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_RFC2RF) ?:
	      xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_PON_WEI) ?:
	      xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_PON_EI) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_SRAM2RFC) ?:
	      xsi_clr(rd, XSI_SRAM_CTRL, XSI_SRAM_DIS) ?:
	      xsi_clr(rd, XSI_XTAL_XMD_2, 0x70) ?:
	      xsi_clr(rd, XSI_XTAL_XMD_4, 0x0F);
	if (ret)
		return ret;

	/* isolate and power down the efuse block again */
	set32(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);
	set32(rd, REG_SYS_ISO_CTRL, SYS_ISO_EB2CORE);
	clr32(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B15);
	fsleep(1000);
	clr32(rd, REG_SYS_ISO_CTRL, SYS_PWC_EV2EF_B14);
	clr32(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);

	/* Voltage trims for B-cut parts without factory power calibration. */
	if (ef->autoload && !ef->power_k_valid) {
		mask32(rd, REG_SPS_DIG_ON_CTRL0, SPS_VOL_L1_MASK, 0x9);
		mask32(rd, REG_SPS_DIG_ON_CTRL0, SPS_VREFPFM_L_MASK, 0xA);
		if (rd->cv == 1) {
			set32(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);
			mask16(rd, REG_HCI_LDO_CTRL, 0x000F, 0xA);
			clr32(rd, REG_PMC_DBG_CTRL2, PMC_SYSON_DIS_PMCR_WRMSK);
		}
	}

	set32(rd, REG_DMAC_FUNC_EN, 0x7FFF8000);
	set32(rd, 0xC000 /* CMAC_FUNC_EN */, 0x7000803F);
	return 0;
}

static int __power_off(struct ax52_dev *rd)
{
	int ret;

	ret = xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_RFC2RF) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_OFF_EI) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_OFF_WEI) ?:
	      xsi_clr(rd, XSI_WL_RFC_S0, BIT(0)) ?:
	      xsi_clr(rd, XSI_WL_RFC_S1, BIT(0)) ?:
	      xsi_set(rd, XSI_ANAPAR_WL, ANAPAR_SRAM2RFC) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_PON_EI) ?:
	      xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_PON_WEI);
	if (ret)
		return ret;

	set32(rd, REG_SYS_PW_CTRL, PW_EN_WLON);
	clr32(rd, REG_WLRF_CTRL, WLRF_AFC_AFEDIG);
	clr8(rd, REG_SYS_FUNC_EN, FEN_BB_GLB_RSTN | FEN_BBRSTB);
	clr32(rd, REG_SYS_ADIE_PAD_PWR_CTRL, PAD_SYM_PADPDN_WL_RFC_1P3);
	ret = xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_SHDN_WL);
	if (ret)
		return ret;
	clr32(rd, REG_SYS_ADIE_PAD_PWR_CTRL, PAD_SYM_PADPDN_WL_PTA_1P3);
	ret = xsi_clr(rd, XSI_ANAPAR_WL, ANAPAR_GND_SHDN_WL);
	if (ret)
		return ret;

	set32(rd, REG_SYS_PW_CTRL, PW_APFM_OFFMAC);
	ret = ax52_poll32(rd, REG_SYS_PW_CTRL, PW_APFM_OFFMAC, 0, 1000, 20000);
	if (ret)
		return ret;

	wr32(rd, REG_WLLPS_CTRL, WLLPS_SW_LPS_OPTION_PCIE);
	set32(rd, REG_SYS_SWR_CTRL1, SWR_SYM_CTRL_SPS_PWMFREQ);
	mask32(rd, REG_SPS_DIG_ON_CTRL0, SPS_REG_ZCDC_H_MASK, 3);
	set32(rd, REG_SYS_PW_CTRL, PW_APFM_SWLPS);
	return 0;
}

int ax52_power_on(struct ax52_dev *rd)
{
	int ret, tries;

	if (rd32_mask(rd, REG_IC_PWR_STATE, IC_WLMAC_PWR_STE_MASK) == 1) {
		ax52_warn(rd, "MAC was left powered on; cycling power\n");
		__power_off(rd);
	}

	for (tries = 0; tries < 2; tries++) {
		ret = __power_on(rd);
		if (!ret)
			break;
		ax52_warn(rd, "power-on attempt %d failed (%d)\n", tries + 1, ret);
		__power_off(rd);
	}
	if (ret)
		return ret;

	wr8(rd, REG_SCOREBOARD_B3, 0x81);	/* notify BT side: WL power major */
	rd->mac_on = true;
	return 0;
}

void ax52_power_off(struct ax52_dev *rd)
{
	int ret;

	rd->fw_ready = false;
	ret = __power_off(rd);
	if (ret)
		ax52_warn(rd, "power-off sequence failed (%d)\n", ret);
	wr8(rd, REG_SCOREBOARD_B3, 0x80);
	rd->mac_on = false;
}

/* ------------------------------------------------------------- WCPU */

void ax52_wcpu_disable(struct ax52_dev *rd)
{
	rd->fw_ready = false;
	clr32(rd, REG_PLATFORM_ENABLE, PLAT_WCPU_EN);
	clr32(rd, REG_WCPU_FW_CTRL, FWCTRL_WCPU_FWDL_EN | FWCTRL_H2C_PATH_RDY |
				    FWCTRL_FWDL_PATH_RDY);
	clr32(rd, REG_SYS_CLK_CTRL, CLK_CPU_CLK_EN);

	/* reset the APB wrapper: disables the firmware watchdog */
	clr32(rd, REG_PLATFORM_ENABLE, PLAT_APB_WRAP_EN);
	set32(rd, REG_PLATFORM_ENABLE, PLAT_APB_WRAP_EN);

	clr32(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
	set32(rd, REG_PLATFORM_ENABLE, PLAT_PLATFORM_EN);
}

/* Start the WCPU boot ROM in firmware-download mode. */
int ax52_wcpu_enable_dl(struct ax52_dev *rd)
{
	u32 v;

	if (rd32(rd, REG_PLATFORM_ENABLE) & PLAT_WCPU_EN) {
		ax52_err(rd, "WCPU already running\n");
		return -EBUSY;
	}

	wr32(rd, REG_UDM1, 0);
	wr32(rd, REG_UDM2, 0);
	wr32(rd, REG_HALT_H2C_CTRL, 0);
	wr32(rd, REG_HALT_C2H_CTRL, 0);
	wr32(rd, REG_HALT_H2C, 0);
	wr32(rd, REG_HALT_C2H, 0);

	set32(rd, REG_SYS_CLK_CTRL, CLK_CPU_CLK_EN);

	v = rd32(rd, REG_WCPU_FW_CTRL);
	v &= ~(FWCTRL_WCPU_FWDL_EN | FWCTRL_H2C_PATH_RDY | FWCTRL_FWDL_PATH_RDY |
	       FWCTRL_FWDL_STS_MASK);
	v |= FWCTRL_WCPU_FWDL_EN;
	wr32(rd, REG_WCPU_FW_CTRL, v);

	mask32(rd, REG_SEC_CTRL, SEC_IDMEM_SIZE_CONFIG_MASK, 2);
	mask16(rd, REG_BOOT_REASON, BOOT_REASON_MASK, 0);

	set32(rd, REG_PLATFORM_ENABLE, PLAT_WCPU_EN);
	return 0;
}
