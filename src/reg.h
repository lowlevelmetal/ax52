/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RTL8852BE register map (subset used by this driver).
 * Offsets are into PCI BAR2. Values come from the hardware specification in
 * docs/spec/, which cites the rtw89 sources they were checked against.
 */
#ifndef AX52_REG_H
#define AX52_REG_H

#include <linux/bits.h>

/* ---------------------------------------------------------------- system */
#define REG_SYS_ISO_CTRL		0x0000
#define   SYS_ISO_EB2CORE		BIT(8)
#define   SYS_PWC_EV2EF_B14		BIT(14)
#define   SYS_PWC_EV2EF_B15		BIT(15)
#define REG_SYS_FUNC_EN			0x0002	/* 8-bit */
#define   FEN_BBRSTB			BIT(0)
#define   FEN_BB_GLB_RSTN		BIT(1)
#define REG_SYS_PW_CTRL			0x0004
#define   PW_APFN_ONMAC			BIT(8)
#define   PW_APFM_OFFMAC		BIT(9)
#define   PW_APFM_SWLPS			BIT(10)
#define   PW_AFSM_WLSUS_EN		BIT(11)
#define   PW_AFSM_PCIE_SUS_EN		BIT(12)
#define   PW_PSUS_OFF_CAPC_EN		BIT(14)
#define   PW_APDM_HPDN			BIT(15)
#define   PW_EN_WLON			BIT(16)
#define   PW_RDY_SYSPWR			BIT(17)
#define   PW_DIS_WLBT_PDNSUSEN_SOPC	BIT(18)
#define REG_SYS_CLK_CTRL		0x0008
#define   CLK_CPU_CLK_EN		BIT(14)
#define REG_SYS_WL_EFUSE_CTRL		0x000A	/* 16-bit */
#define   EFUSE_AUTOLOAD_SUS		BIT(5)
#define REG_SYS_SWR_CTRL1		0x0010
#define   SWR_SYM_CTRL_SPS_PWMFREQ	BIT(10)
#define REG_SYS_ADIE_PAD_PWR_CTRL	0x0018
#define   PAD_SYM_PADPDN_WL_RFC_1P3	BIT(5)
#define   PAD_SYM_PADPDN_WL_PTA_1P3	BIT(6)
#define REG_AFE_LDO_CTRL		0x0020
#define   AFE_AON_OFF_PC_EN		BIT(23)
#define REG_EFUSE_CTRL			0x0030
#define   EFUSE_CTRL_RDY		BIT(29)
#define   EFUSE_CTRL_ADDR_SHIFT		16
#define REG_GPIO_MUXCFG			0x0040
#define REG_PLATFORM_ENABLE		0x0088
#define   PLAT_PLATFORM_EN		BIT(0)
#define   PLAT_WCPU_EN			BIT(1)
#define   PLAT_APB_WRAP_EN		BIT(2)
#define REG_SYS_SDIO_CTRL		0x0070
#define   SDIO_PCIE_CALIB_EN_V1		BIT(12)
#define   SDIO_PCIE_DIS_WLSUS_AFT_PDN	BIT(14)
#define   SDIO_PCIE_DIS_L2_CTRL_LDO_HCI	BIT(15)
#define REG_HCI_OPT_CTRL		0x0074
#define   HCI_BIT_WAKE_CTRL		BIT(5)
#define REG_HCI_LDO_CTRL		0x007A	/* 16-bit */
#define REG_WLLPS_CTRL			0x0090
#define   WLLPS_DIS_WLBT_LPSEN_LOPC	BIT(1)
#define   WLLPS_SW_LPS_OPTION_PCIE	0x0001A0B2
#define REG_SCOREBOARD			0x00AC
#define REG_SCOREBOARD_B3		0x00AF	/* 8-bit */
#define REG_PMC_DBG_CTRL2		0x00CC
#define   PMC_SYSON_DIS_PMCR_WRMSK	BIT(2)
#define REG_SYS_CFG1			0x00F0
#define   SYS_CFG1_CHIP_VER_MASK	GENMASK(15, 12)
#define REG_HIMR0			0x01A0
#define   HIMR0_HALT_C2H		BIT(21)
#define   HIMR0_WDT_TIMEOUT		BIT(22)
#define REG_HISR0			0x01A4
#define REG_HALT_H2C_CTRL		0x0160
#define REG_HALT_C2H_CTRL		0x0164
#define REG_HALT_H2C			0x0168
#define REG_HALT_C2H			0x016C
#define REG_WCPU_FW_CTRL		0x01E0
#define   FWCTRL_WCPU_FWDL_EN		BIT(0)
#define   FWCTRL_H2C_PATH_RDY		BIT(1)
#define   FWCTRL_FWDL_PATH_RDY		BIT(2)
#define   FWCTRL_FWDL_STS_MASK		GENMASK(7, 5)
#define     FWDL_STS_CHECKSUM_FAIL	2
#define     FWDL_STS_SECURITY_FAIL	3
#define     FWDL_STS_CV_NOT_MATCH	4
#define     FWDL_STS_WCPU_FW_INIT_RDY	7
#define REG_BOOT_REASON			0x01E6	/* 16-bit */
#define   BOOT_REASON_MASK		GENMASK(2, 0)
#define REG_UDM0			0x01F0
#define REG_UDM1			0x01F4
#define REG_UDM2			0x01F8
#define REG_SPS_DIG_ON_CTRL0		0x0200
#define   SPS_VOL_L1_MASK		GENMASK(3, 0)
#define   SPS_OCP_L1_MASK		GENMASK(15, 13)
#define   SPS_REG_ZCDC_H_MASK		GENMASK(18, 17)
#define   SPS_VREFPFM_L_MASK		GENMASK(25, 22)
#define REG_WLAN_XTAL_SI_CTRL		0x0270
#define   XSI_CMD_POLL			BIT(31)
#define   XSI_MODE_MASK			GENMASK(25, 24)
#define   XSI_BITMASK_MASK		GENMASK(23, 16)
#define   XSI_DATA_MASK			GENMASK(15, 8)
#define   XSI_ADDR_MASK			GENMASK(7, 0)
#define REG_WLRF_CTRL			0x02F0
#define   WLRF_AFC_AFEDIG		BIT(17)
#define REG_IC_PWR_STATE		0x03F0
#define   IC_WLMAC_PWR_STE_MASK		GENMASK(9, 8)
#define REG_SPS_DIG_OFF_CTRL0		0x0400
#define   SPS_OFF_C1_L1_MASK		GENMASK(1, 0)
#define   SPS_OFF_C3_L1_MASK		GENMASK(5, 4)
#define REG_SEC_CTRL			0x0C00
#define   SEC_IDMEM_SIZE_CONFIG_MASK	GENMASK(17, 16)
#define REG_FILTER_MODEL_ADDR		0x0C04
#define REG_INDIR_ACCESS_ENTRY		0x40000

/* XTAL-SI (indirect analog register file) offsets */
#define XSI_XTAL_SC_XI			0x04
#define XSI_XTAL_SC_XO			0x05
#define XSI_XTAL_XMD_2			0x24
#define XSI_XTAL_XMD_4			0x26
#define XSI_CV				0x41
#define XSI_WL_RFC_S0			0x80
#define XSI_WL_RFC_S1			0x81
#define XSI_ANAPAR_WL			0x90
#define   ANAPAR_PON_WEI		BIT(0)
#define   ANAPAR_PON_EI			BIT(1)
#define   ANAPAR_OFF_WEI		BIT(2)
#define   ANAPAR_OFF_EI			BIT(3)
#define   ANAPAR_RFC2RF			BIT(4)
#define   ANAPAR_SHDN_WL		BIT(5)
#define   ANAPAR_GND_SHDN_WL		BIT(6)
#define   ANAPAR_SRAM2RFC		BIT(7)
#define XSI_SRAM_CTRL			0xA1
#define   XSI_SRAM_DIS			BIT(1)

/* ------------------------------------------------ PCIe host interface */
#define REG_PCIE_INIT_CFG1		0x1000
#define   CFG1_RST_BDRAM		BIT(3)
#define   CFG1_MAX_TXDMA_MASK		GENMASK(10, 8)
#define   CFG1_TXHCI_EN			BIT(11)
#define   CFG1_LATENCY_CONTROL		BIT(12)
#define   CFG1_RXHCI_EN			BIT(13)
#define   CFG1_MAX_RXDMA_MASK		GENMASK(16, 14)
#define   CFG1_RXBD_MODE		BIT(18)
#define   CFG1_TXRST_KEEP_REG		BIT(22)
#define   CFG1_RXRST_KEEP_REG		BIT(23)
#define REG_PCIE_INIT_CFG2		0x1004
#define   CFG2_WD_ITVL_ACT_MASK		GENMASK(19, 16)
#define   CFG2_WD_ITVL_IDLE_MASK	GENMASK(27, 24)
#define REG_PCIE_PS_CTRL		0x1008
#define   PS_L1OFF_PWR_OFF_EN		BIT(5)
#define REG_PCIE_DMA_STOP1		0x1010
#define   DMA_STOP_TXCH_MASK		0x00070F00	/* ACH0-3, CH8, CH9, CH12 */
#define   DMA_STOP_CH12			BIT(18)
#define   DMA_STOP_WPDMA		BIT(19)
#define   DMA_STOP_PCIEIO		BIT(20)
#define REG_TXBD_RWPTR_CLR1		0x1014
#define   TXBD_CLR_ALL			0x0000070F
#define REG_RXBD_RWPTR_CLR		0x1018
#define   RXBD_CLR_ALL			0x00000003
#define REG_PCIE_DMA_BUSY1		0x101C
#define   DMA_BUSY_TXCH_MASK		0x00070F00
#define   DMA_BUSY_RXCH_MASK		0x00000003
#define REG_RXQ_RXBD_NUM		0x1020
#define REG_RPQ_RXBD_NUM		0x1022
#define REG_RXQ_RXBD_IDX		0x1050
#define REG_RPQ_RXBD_IDX		0x1054
#define   BD_HOST_IDX_MASK		GENMASK(11, 0)
#define   BD_HW_IDX_MASK		GENMASK(27, 16)
#define REG_RXQ_RXBD_DESA_L		0x1100
#define REG_RPQ_RXBD_DESA_L		0x1108
#define REG_DBI_FLAG			0x1090
#define REG_MDIO_CFG			0x10A0
#define   MDIO_ADDR_MASK		GENMASK(4, 0)
#define   MDIO_WFLAG			BIT(8)
#define   MDIO_RFLAG			BIT(9)
#define   MDIO_PAGE_MASK		GENMASK(13, 12)
#define REG_MDIO_WDATA			0x10A4	/* 16-bit */
#define REG_MDIO_RDATA			0x10A6	/* 16-bit */
#define REG_PCIE_HIMR00			0x10B0
#define REG_PCIE_HISR00			0x10B4
#define   HI00_RXDMA			BIT(0)
#define   HI00_RXP1DMA			BIT(1)
#define   HI00_RPQDMA			BIT(2)
#define   HI00_TXDMA_STUCK		BIT(17)
#define   HI00_RXDMA_STUCK		BIT(18)
#define   HI00_RDU			BIT(19)
#define   HI00_RPQBD_FULL		BIT(20)
#define   HI00_HS0ISR_IND		BIT(24)
#define REG_INT_MIT_RX			0x10D4
#define REG_PCIE_DBG_CTRL		0x11C0
#define REG_LBC_WATCHDOG		0x11D8
#define   LBC_TIMER_MASK		GENMASK(7, 4)
#define   LBC_FLAG			BIT(1)
#define   LBC_EN			BIT(0)
#define REG_PCIE_HIMR10			0x13B0
#define REG_PCIE_HISR10			0x13B4
#define   HI10_HC10ISR_IND		BIT(28)
#define REG_PCIE_EXP_CTRL		0x13F0
#define   EXP_MAX_TAG_NUM_MASK		GENMASK(18, 16)
#define   EXP_SIC_EN_FORCE_CLKREQ	BIT(4)

/* PCIe PHY (MDIO) registers, Gen1 bank */
#define MDIO_RAC_REG_REV2		0x1B
#define   BAC_CMU_EN_DLY_MASK		GENMASK(15, 12)
#define MDIO_RAC_REG_FLD_0		0x1D
#define   BAC_AUTOK_N_MASK		GENMASK(3, 2)
#define MDIO_RAC_CTRL_PPR_V1		0x30
#define   BAC_CALIB_EN			BIT(13)
#define MDIO_PAGE_G1_LOW		0
#define MDIO_PAGE_G1_HIGH		1
#define MDIO_PAGE_G2_LOW		2
#define MDIO_PAGE_G2_HIGH		3

/* Realtek vendor PCI config space bytes */
#define PCICFG_PHY_RATE			0x82
#define PCICFG_L1_CTRL			0x719
#define   PCICFG_L1_CTRL_ASPM_L1	BIT(3)

/* ------------------------------------------------------------- MAC/DMAC */
#define REG_H2CREG_DATA0		0x8140
#define REG_C2HREG_DATA0		0x8150
#define REG_H2CREG_CTRL			0x8160
#define REG_C2HREG_CTRL			0x8164
#define REG_HCI_FUNC_EN			0x8380
#define   HCI_TXDMA_EN			BIT(0)
#define   HCI_RXDMA_EN			BIT(1)
#define REG_BOOT_DBG			0x83F0
#define REG_DMAC_FUNC_EN		0x8400
#define   DMAC_FUNC_MAC_FUNC_EN		BIT(30)
#define   DMAC_FUNC_DMAC_FUNC_EN	BIT(29)
#define   DMAC_FUNC_DLE_WDE_EN		BIT(26)
#define   DMAC_FUNC_DLE_PLE_EN		BIT(23)
#define REG_DMAC_CLK_EN			0x8404
#define   DMAC_CLK_DLE_WDE		BIT(26)
#define   DMAC_CLK_DLE_PLE		BIT(23)
#define REG_LTR_CTRL_0			0x8410
#define REG_TX_ADDR_INFO_MODE		0x8810
#define   HOST_ADDR_INFO_8B_SEL		BIT(0)
#define REG_HCI_FC_CTRL			0x8A00
#define   HCI_FC_EN			BIT(0)
#define   HCI_FC_MODE_MASK		GENMASK(2, 1)
#define   HCI_FC_CH12_EN		BIT(3)
#define   HCI_FC_WD_FULL_COND_MASK	GENMASK(5, 4)
#define   HCI_FC_WP_CH07_FULL_COND_MASK	GENMASK(7, 6)
#define   HCI_FC_WP_CH811_FULL_COND_MASK GENMASK(9, 8)
#define   HCI_FC_CH12_FULL_COND_MASK	GENMASK(11, 10)
#define REG_CH_PAGE_CTRL		0x8A04
#define REG_ACH0_PAGE_CTRL		0x8A10	/* + 4 * ch */
#define REG_ACH0_PAGE_INFO		0x8A50	/* + 4 * ch */
#define REG_PUB_PAGE_INFO3		0x8A8C
#define REG_PUB_PAGE_CTRL1		0x8A90
#define REG_PUB_PAGE_CTRL2		0x8A94
#define REG_PUB_PAGE_INFO1		0x8A98
#define REG_PUB_PAGE_INFO2		0x8A9C
#define REG_WP_PAGE_CTRL1		0x8AA0
#define REG_WP_PAGE_CTRL2		0x8AA4
#define REG_WDE_PKTBUF_CFG		0x8C08
#define REG_PLE_PKTBUF_CFG		0x9008
#define   PKTBUF_PAGE_SEL_MASK		GENMASK(1, 0)
#define   PKTBUF_START_BOUND_MASK	GENMASK(13, 8)
#define   PKTBUF_FREE_PAGE_MASK		GENMASK(28, 16)
#define REG_WDE_QTA0			0x8C40	/* hif, wcpu, (dcpu), pkt_in, cpu_io */
#define REG_WDE_INI_STATUS		0x8D00
#define REG_PLE_QTA0			0x9040	/* 11 quota regs, 4 apart */
#define REG_PLE_INI_STATUS		0x9100
#define REG_PKTIN_SETTING		0x9A00
#define   PKTIN_WD_ADDR_INFO_LENGTH	BIT(1)

#endif
