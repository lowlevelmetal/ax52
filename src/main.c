// SPDX-License-Identifier: GPL-2.0
/*
 * ax52 - an independent Linux driver for the Realtek RTL8852BE.
 *
 * Module entry, PCI probe/remove and chip bring-up orchestration.
 */
#include <linux/module.h>

#include "ax52.h"

#ifndef AX52_VERSION
#define AX52_VERSION "unknown"
#endif

/* Power on and boot the firmware (every interface-up and at probe). */
static int power_up_fw(struct ax52_dev *rd)
{
	int ret;

	ret = ax52_power_on(rd);
	if (ret) {
		ax52_err(rd, "power-on failed: %d\n", ret);
		return ret;
	}
	ret = ax52_mac_pre_fwdl(rd);
	if (ret) {
		ax52_err(rd, "pre-download MAC init failed: %d\n", ret);
		goto err;
	}
	ret = ax52_fw_download(rd);
	if (ret)
		goto err;
	return 0;

err:
	ax52_pci_deinit(rd);
	ax52_power_off(rd);
	return ret;
}

/*
 * Probe-time bring-up: boot the firmware and read the efuse, then power
 * the chip back down until mac80211 starts the interface.
 */
static int ax52_probe_bringup(struct ax52_dev *rd)
{
	int ret;

	ret = power_up_fw(rd);
	if (ret)
		return ret;
	ret = ax52_efuse_read(rd);
	if (ret)
		ax52_err(rd, "efuse read failed: %d\n", ret);

	ax52_pci_deinit(rd);
	ax52_power_off(rd);
	return ret;
}

int ax52_chip_start(struct ax52_dev *rd)
{
	int ret;

	ret = power_up_fw(rd);
	if (ret)
		return ret;

	ret = ax52_mac_init(rd);
	if (ret) {
		ax52_err(rd, "MAC init failed: %d\n", ret);
		goto err;
	}
	ret = ax52_h2c_post_mac_init(rd);
	if (ret)
		goto err;

	ax52_mac_reset_bb_rf(rd);
	ret = ax52_phy_init(rd);
	if (ret) {
		ax52_err(rd, "PHY init failed: %d\n", ret);
		goto err;
	}
	ax52_coex_init(rd);
	ret = ax52_rfk_init(rd);
	if (ret)
		ax52_warn(rd, "RF calibration at start failed: %d\n", ret);

	ax52_mac_ppdu_status(rd, true);
	ax52_mac_set_rx_filter(rd, rd->rx_fltr);
	ax52_mac_set_rts_threshold(rd, rd->hw->wiphy->rts_threshold);

	ax52_pci_start(rd);
	ax52_track_start(rd);
	ax52_info(rd, "radio up\n");
	return 0;

err:
	ax52_pci_deinit(rd);
	ax52_power_off(rd);
	ax52_pci_reset(rd);
	return ret;
}

void ax52_chip_stop(struct ax52_dev *rd)
{
	ax52_track_stop(rd);
	ax52_pci_stop(rd);
	cancel_delayed_work_sync(&rd->txq_work);
	cancel_work_sync(&rd->ba_work);

	ax52_coex_release(rd);
	ax52_pci_deinit(rd);
	ax52_power_off(rd);
	ax52_pci_reset(rd);
	memset(&rd->chandef, 0, sizeof(rd->chandef));
	ax52_info(rd, "radio down\n");
}

static void ax52_free_subsys(struct ax52_dev *rd)
{
	ax52_h2c_free(rd);
	ax52_rfk_free(rd);
	ax52_phy_free(rd);
}

static int ax52_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ax52_dev *rd;
	int ret;

	rd = ax52_alloc_hw(&pdev->dev);
	if (!rd)
		return -ENOMEM;
	rd->pdev = pdev;
	rd->dev = &pdev->dev;
	spin_lock_init(&rd->h2c_lock);
	spin_lock_init(&rd->tx_lock);
	spin_lock_init(&rd->irq_lock);
	__skb_queue_head_init(&rd->ppdu_q);
	ax52_tx_init(rd);
	pci_set_drvdata(pdev, rd);

	rd->txq_wq = alloc_workqueue(DRV_NAME "_tx", WQ_UNBOUND | WQ_HIGHPRI, 0);
	if (!rd->txq_wq) {
		ret = -ENOMEM;
		goto err_hw;
	}

	ret = pci_enable_device(pdev);
	if (ret)
		goto err_wq;
	pci_set_master(pdev);

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret)
		goto err_disable;

	/* The upstream AMD bridge is not DAC-capable for this chip: 32-bit DMA. */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_regions;

	rd->mmio = pci_iomap(pdev, 2, 0);
	if (!rd->mmio) {
		ret = -ENOMEM;
		goto err_regions;
	}

	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL2,
				 PCI_EXP_DEVCTL2_COMP_TMOUT_DIS);

	rd->cv = rd32_mask(rd, REG_SYS_CFG1, SYS_CFG1_CHIP_VER_MASK);
	ax52_info(rd, "RTL8852B cut %c (cv %u), ax52 version %s\n",
		  'A' + rd->cv, rd->cv, AX52_VERSION);

	ret = ax52_pci_alloc_rings(rd);
	if (ret)
		goto err_unmap;
	ret = ax52_fw_load(rd);
	if (ret)
		goto err_rings;
	ret = ax52_probe_bringup(rd);
	if (ret)
		goto err_fw;
	if (rd->cv != 1 || rd->efuse.rfe_type != 1)
		ax52_warn(rd, "untested hardware (cut %c, RFE %u); only cut B / RFE 1 has been tested, please report results\n",
			  'A' + rd->cv, rd->efuse.rfe_type);

	ret = ax52_phy_alloc(rd) ?: ax52_rfk_alloc(rd) ?: ax52_h2c_alloc(rd);
	if (ret)
		goto err_subsys;
	ret = ax52_pci_irq_init(rd);
	if (ret)
		goto err_subsys;
	ret = ax52_register_hw(rd);
	if (ret)
		goto err_irq;
	return 0;

err_irq:
	ax52_pci_irq_deinit(rd);
err_subsys:
	ax52_free_subsys(rd);
err_fw:
	ax52_fw_release(rd);
err_rings:
	ax52_pci_free_rings(rd);
err_unmap:
	pci_iounmap(pdev, rd->mmio);
err_regions:
	pci_release_regions(pdev);
err_disable:
	pci_disable_device(pdev);
err_wq:
	destroy_workqueue(rd->txq_wq);
err_hw:
	ax52_free_hw(rd);
	return ret;
}

static void ax52_pci_remove(struct pci_dev *pdev)
{
	struct ax52_dev *rd = pci_get_drvdata(pdev);

	ax52_unregister_hw(rd);	/* stops the radio if it was up */
	ax52_pci_irq_deinit(rd);
	ax52_free_subsys(rd);
	ax52_fw_release(rd);
	ax52_pci_free_rings(rd);
	pci_iounmap(pdev, rd->mmio);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	destroy_workqueue(rd->txq_wq);
	ax52_free_hw(rd);
}

/* Leave no DMA running across reboot/kexec. */
static void ax52_pci_shutdown(struct pci_dev *pdev)
{
	struct ax52_dev *rd = pci_get_drvdata(pdev);

	if (!rd->mac_on)
		return;
	wr32(rd, REG_HIMR0, 0);
	wr32(rd, REG_PCIE_HIMR00, 0);
	wr32(rd, REG_PCIE_HIMR10, 0);
	ax52_pci_deinit(rd);
	ax52_power_off(rd);
}

static const struct pci_device_id ax52_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0xb852) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0xb85b) },
	{ }
};
MODULE_DEVICE_TABLE(pci, ax52_pci_ids);

static struct pci_driver ax52_pci_driver = {
	.name = DRV_NAME,
	.id_table = ax52_pci_ids,
	.probe = ax52_pci_probe,
	.remove = ax52_pci_remove,
	.shutdown = ax52_pci_shutdown,
};
module_pci_driver(ax52_pci_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Independent driver for the Realtek RTL8852BE 802.11ax PCIe NIC");
MODULE_FIRMWARE(AX52_FW_NAME);
MODULE_VERSION(AX52_VERSION);
