/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ax52 - an independent Linux driver for the Realtek RTL8852BE
 * (PCIe 802.11ax, 2T2R). Station mode only.
 */
#ifndef AX52_H
#define AX52_H

#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>

#include "reg.h"

#define DRV_NAME		"ax52"
#define AX52_FW_NAME		"rtw89/rtw8852b_fw-2.bin"

/* ------------------------------------------------------------- DMA rings */

/* Our TX ring index (dense) -> hardware DMA channel number. */
enum ax52_txq {
	TXQ_ACH0,	/* BE */
	TXQ_ACH1,	/* BK */
	TXQ_ACH2,	/* VI */
	TXQ_ACH3,	/* VO */
	TXQ_MGMT,	/* hw CH8: band-0 management */
	TXQ_HIQ,	/* hw CH9: band-0 high queue */
	TXQ_FWCMD,	/* hw CH12: H2C and firmware download */
	TXQ_NUM,
};

enum ax52_rxq {
	RXQ_DATA,	/* frames, C2H events, PPDU status */
	RXQ_RPQ,	/* TX release reports */
	RXQ_NUM,
};

#define TXBD_NUM		256
#define RXBD_NUM		256
#define RXBUF_SIZE		11498	/* max MPDU + long RX desc + rxbd_info */
#define WD_PAGE_SIZE		128
#define WD_PAGE_NUM		512
#define FWCMD_SLOT_SIZE		PAGE_SIZE
#define FWCMD_KEEP		8	/* recently consumed slots never reused */

/* 8-byte buffer descriptor, shared layout for TX and RX ("truncated" mode). */
struct ax52_bd {
	__le16 len;
	__le16 opt;
	__le32 dma;
} __packed;

#define TXBD_OPT_LS		BIT(14)
#define BD_OPT_DMA_HI		GENMASK(13, 6)

/* Bookkeeping for one 128-byte WD page of a data/mgmt ring. */
struct ax52_wd_page {
	struct sk_buff *skb;	/* frame awaiting its release report */
	dma_addr_t skb_dma;
	bool bd_busy;		/* its BD not yet consumed by the DMA engine */
};

struct ax52_txring {
	u8 hwch;
	u16 reg_num, reg_idx, reg_bdram, reg_desa;
	u32 bdram_val;

	struct ax52_bd *bd;
	dma_addr_t bd_dma;
	u16 wp;			/* next BD the host fills */
	u16 rp;			/* last hardware read index we observed */

	/* data/mgmt rings: WD pages (freed when both BD and RPP are done) */
	struct ax52_wd_page *page;	/* [WD_PAGE_NUM] */
	u16 *bd2page;			/* [TXBD_NUM] */
	u16 *free_page;			/* [WD_PAGE_NUM], stack */
	u16 nfree;

	/* FWCMD only: one coherent buffer per BD slot */
	void **slot;			/* [TXBD_NUM] */
	dma_addr_t *slot_dma;		/* [TXBD_NUM] */
};

struct ax52_rxring {
	u16 reg_num, reg_idx, reg_desa;
	struct ax52_bd *bd;
	dma_addr_t bd_dma;
	void *buf[RXBD_NUM];
	dma_addr_t buf_dma[RXBD_NUM];
	u16 wp;			/* next buffer the host will read */
	struct sk_buff *partial;	/* multi-buffer packet being assembled */
	u32 partial_len;
};

/* --------------------------------------------------------------- firmware */

#define FW_MAX_SECTIONS		10
#define FWDL_CHUNK		2020

struct ax52_fw_section {
	const u8 *data;
	u32 len;		/* bytes to download (incl. optional checksum) */
	u8 type;
};

/* A "reg2" table from the firmware file: pairs of {addr, data}. */
struct ax52_reg2_tbl {
	const __le32 *pairs;
	u32 n;			/* number of pairs */
};

/*
 * One firmware-file element. For the TX-power family (ids 9..17) the 8-byte
 * head is: [2] rfe_type, [3] entry size, [4..7] le32 entry count.
 */
struct ax52_fw_elem {
	u32 id;
	const u8 *head;		/* 8 bytes */
	const u8 *data;		/* payload */
	u32 size;		/* payload bytes */
};

#define FW_MAX_ELEMS		48

enum ax52_fw_elem_id {
	FW_ELEM_BB_REG = 2,
	FW_ELEM_BB_GAIN = 3,
	FW_ELEM_RADIO_A = 4,
	FW_ELEM_RADIO_B = 5,
	FW_ELEM_RF_NCTL = 8,
	FW_ELEM_TXPWR_BYRATE = 9,
	FW_ELEM_TXPWR_LMT_2GHZ = 10,
	FW_ELEM_TXPWR_LMT_5GHZ = 11,
	FW_ELEM_TXPWR_LMT_RU_2GHZ = 13,
	FW_ELEM_TXPWR_LMT_RU_5GHZ = 14,
	FW_ELEM_TX_SHAPE_LMT = 16,
	FW_ELEM_TX_SHAPE_LMT_RU = 17,
	FW_ELEM_TXPWR_TRK = 18,
	FW_ELEM_REGD = 20,
};

struct ax52_fw {
	const struct firmware *blob;
	u8 ver_major, ver_minor, ver_sub, ver_idx;
	u32 commit;
	const u8 *hdr;		/* image header, sent as the first FWDL packet */
	u32 hdr_send_len;	/* header bytes to send (dynamic header excluded) */
	struct ax52_fw_section sec[FW_MAX_SECTIONS];
	u8 nsec;

	/* PHY tables carried by the firmware file ("elements") */
	struct ax52_reg2_tbl bb, bb_gain, radio[2], nctl;
	struct ax52_fw_elem elem[FW_MAX_ELEMS];
	u8 nelem;
};

/* ----------------------------------------------------------------- efuse */

#define EFUSE_PHY_SIZE		1216
#define EFUSE_LOG_SIZE		2048
#define EFUSE_PHYCAP_ADDR	0x580
#define EFUSE_PHYCAP_SIZE	128

struct ax52_efuse {
	bool autoload;
	u8 log[EFUSE_LOG_SIZE];
	u8 phycap[EFUSE_PHYCAP_SIZE];	/* physical 0x580..0x5ff */
	u8 addr[ETH_ALEN];
	u8 rfe_type;
	u8 xtal_cap;
	u8 thermal[2];
	char country[3];
	bool power_k_valid;
};

/* ------------------------------------------------------ channel / radio */

enum ax52_band { AX52_BAND_2G = 0, AX52_BAND_5G = 1 };
enum ax52_bw { AX52_BW_20 = 0, AX52_BW_40 = 1, AX52_BW_80 = 2 };
enum ax52_rf_path { RF_PATH_A = 0, RF_PATH_B = 1, RF_PATH_NUM = 2 };

/*
 * Sub-band of the operating channel. Vendor numbering (5 GHz "band 2" is
 * unused), so per-sub-band calibration tables can be indexed directly.
 */
enum ax52_subband {
	AX52_SB_2G = 0,
	AX52_SB_5G_1 = 1,	/* ch 36-64 */
	AX52_SB_5G_3 = 3,	/* ch 100-144 */
	AX52_SB_5G_4 = 4,	/* ch 149-177 */
	AX52_SB_NUM,
};

/* The operating channel, derived from a cfg80211_chan_def by phy.c. */
struct ax52_chan {
	u8 band;		/* enum ax52_band */
	u8 bw;			/* enum ax52_bw */
	u8 ch;			/* channel number of the centre of the whole BW */
	u8 pri_ch;		/* primary 20 MHz channel number */
	u16 freq;		/* centre frequency of the whole BW, MHz */
	u16 pri_freq;		/* primary 20 MHz centre frequency, MHz */
	u8 subband;		/* enum ax52_subband */
};

/* ------------------------------------------------------ interfaces */

/* Station interface state. Only one vif, on port 0, mac_id 0. */
struct ax52_vif {
	u8 mac_id;
	u8 port;
	u8 addr_cam_idx;
	u8 bssid_cam_idx;
	u8 net_type;		/* 0 = no link, 2 = infrastructure */
	bool assoc;
	u8 addr[ETH_ALEN];
	u8 bssid[ETH_ALEN];
	u16 aid;
	u8 bss_color;

	/* key slots of our address-CAM entry (0-1 pairwise, 2-4 group, 5-6 BIP) */
	u8 sec_ent_map;
	u8 sec_ent[7];			/* security-CAM index per slot */
	u8 sec_ent_keyid[7];
};

/* Hardware cipher types in the security CAM and the TX descriptor. */
enum ax52_sec_type {
	AX52_SEC_NONE = 0,
	AX52_SEC_CCMP128 = 6,
	AX52_SEC_CCMP256 = 7,
	AX52_SEC_GCMP128 = 8,
	AX52_SEC_GCMP256 = 9,
};

#define AX52_SEC_CAM_NUM	128

/* RX PN/SN tracking of one TID with an RX BA session (hardware-decrypt quirk). */
struct ax52_tid_rx {
	bool started;
	u16 last_sn;
	s64 last_pn;
};

/* The AP as seen by our station. In station mode it shares the vif mac_id. */
struct ax52_sta {
	u8 mac_id;
};

/* Last firmware rate-adaptation report, for sta_statistics. */
struct ax52_ra_report {
	bool valid;
	struct rate_info txrate;
};

/* ---------------------------------------------------------------- device */

struct ax52_dev {
	struct pci_dev *pdev;
	struct device *dev;
	void __iomem *mmio;
	struct ieee80211_hw *hw;

	u8 cv;			/* chip cut: 1 = B-cut */
	bool mac_on;
	bool fw_ready;

	struct ax52_chan chan;	/* current operating channel */
	char regd_alpha2[3];	/* current regulatory country ("00" = world) */
	struct ax52_ra_report ra;

	void *phy_priv;		/* owned by phy.c */
	void *rfk_priv;		/* owned by rfk.c */
	void *h2c_priv;		/* owned by h2c.c */

	struct ax52_txring tx[TXQ_NUM];
	struct ax52_rxring rx[RXQ_NUM];

	/* WD pages for the six data/mgmt rings */
	void *wd[TXQ_FWCMD];
	dma_addr_t wd_dma[TXQ_FWCMD];

	spinlock_t tx_lock;	/* data rings, WD pages (BH) */
	spinlock_t irq_lock;	/* interrupt masks, running */
	bool running;		/* interrupts wanted */
	struct net_device *napi_dev;
	struct napi_struct napi;
	struct sk_buff_head ppdu_q;	/* MPDUs waiting for their PPDU status */
	u8 ppdu_cnt;

	/* TX scheduling */
	struct workqueue_struct *txq_wq;
	struct delayed_work txq_work;
	struct work_struct ba_work;
	bool tx_starved;
	unsigned long ba_tried;		/* TIDs we tried to start TX BA on */
	unsigned long ba_pending;	/* TIDs queued for ba_work */
	u8 agg_num[IEEE80211_NUM_TIDS];	/* negotiated TX BA buffer sizes */

	/* periodic work, mac80211 state */
	struct wiphy_delayed_work track_work;
	struct wiphy_work regd_work;
	struct ieee80211_vif *vif;	/* the single station interface */
	struct ax52_vif rvif;
	u32 rx_fltr;
	bool scanning;
	struct cfg80211_chan_def chandef;

	struct ieee80211_tx_queue_params edca[IEEE80211_NUM_ACS];
	u8 edca_valid;

	/* hardware crypto */
	DECLARE_BITMAP(sec_cam_map, AX52_SEC_CAM_NUM);
	u8 sec_cam_type[AX52_SEC_CAM_NUM];	/* enum ax52_sec_type per index */
	struct ax52_tid_rx tid_rx[IEEE80211_NUM_TIDS];

	u32 coex_sb;		/* our WL scoreboard bits */
	u32 coex_gnt_saved;

	spinlock_t h2c_lock;
	u8 h2c_seq;

	struct ax52_fw fw;
	struct ax52_efuse efuse;
};

/* ------------------------------------------------------------ MMIO access */

static inline bool ax52_is_cmac(u32 addr)
{
	return addr >= 0xC000 && addr <= 0xFFFF;
}

u32 ax52_rd32_cmac(struct ax52_dev *rd, u32 addr);

static inline u32 rd32(struct ax52_dev *rd, u32 addr)
{
	u32 v = readl(rd->mmio + addr);

	if (unlikely(v == 0xdeadbeef && ax52_is_cmac(addr)))
		v = ax52_rd32_cmac(rd, addr);
	return v;
}

static inline u16 rd16(struct ax52_dev *rd, u32 addr)
{
	if (ax52_is_cmac(addr))
		return rd32(rd, addr & ~3) >> ((addr & 2) * 8);
	return readw(rd->mmio + addr);
}

static inline u8 rd8(struct ax52_dev *rd, u32 addr)
{
	if (ax52_is_cmac(addr))
		return rd32(rd, addr & ~3) >> ((addr & 3) * 8);
	return readb(rd->mmio + addr);
}

static inline void wr32(struct ax52_dev *rd, u32 addr, u32 v) { writel(v, rd->mmio + addr); }
static inline void wr16(struct ax52_dev *rd, u32 addr, u16 v) { writew(v, rd->mmio + addr); }
static inline void wr8(struct ax52_dev *rd, u32 addr, u8 v)   { writeb(v, rd->mmio + addr); }

static inline void set32(struct ax52_dev *rd, u32 a, u32 b) { wr32(rd, a, rd32(rd, a) | b); }
static inline void clr32(struct ax52_dev *rd, u32 a, u32 b) { wr32(rd, a, rd32(rd, a) & ~b); }
static inline void set16(struct ax52_dev *rd, u32 a, u16 b) { wr16(rd, a, rd16(rd, a) | b); }
static inline void clr16(struct ax52_dev *rd, u32 a, u16 b) { wr16(rd, a, rd16(rd, a) & ~b); }
static inline void set8(struct ax52_dev *rd, u32 a, u8 b)   { wr8(rd, a, rd8(rd, a) | b); }
static inline void clr8(struct ax52_dev *rd, u32 a, u8 b)   { wr8(rd, a, rd8(rd, a) & ~b); }

/* Replace the field selected by @mask with @val (val is NOT pre-shifted). */
static inline void mask32(struct ax52_dev *rd, u32 a, u32 mask, u32 val)
{
	u32 v = rd32(rd, a);

	v = (v & ~mask) | ((val << __ffs(mask)) & mask);
	wr32(rd, a, v);
}

static inline void mask16(struct ax52_dev *rd, u32 a, u16 mask, u16 val)
{
	u16 v = rd16(rd, a);

	v = (v & ~mask) | ((val << __ffs(mask)) & mask);
	wr16(rd, a, v);
}

static inline void mask8(struct ax52_dev *rd, u32 a, u8 mask, u8 val)
{
	u8 v = rd8(rd, a);

	v = (v & ~mask) | ((val << __ffs(mask)) & mask);
	wr8(rd, a, v);
}

static inline u32 rd32_mask(struct ax52_dev *rd, u32 a, u32 mask)
{
	return (rd32(rd, a) & mask) >> __ffs(mask);
}

/* Poll until (reg & mask) == want. Sleeps; interval/timeout in microseconds. */
int ax52_poll32(struct ax52_dev *rd, u32 addr, u32 mask, u32 want,
		u32 interval_us, u32 timeout_us);
int ax52_poll8(struct ax52_dev *rd, u32 addr, u8 mask, u8 want,
	       u32 interval_us, u32 timeout_us);

#define ax52_err(rd, fmt, ...)  dev_err((rd)->dev, fmt, ##__VA_ARGS__)
#define ax52_warn(rd, fmt, ...) dev_warn((rd)->dev, fmt, ##__VA_ARGS__)
#define ax52_info(rd, fmt, ...) dev_info((rd)->dev, fmt, ##__VA_ARGS__)
#define ax52_dbg(rd, fmt, ...)  dev_dbg((rd)->dev, fmt, ##__VA_ARGS__)

/* ------------------------------------------------------------- functions */

/* pwr.c */
int ax52_xsi_write(struct ax52_dev *rd, u8 off, u8 val, u8 mask);
int ax52_xsi_read(struct ax52_dev *rd, u8 off, u8 *val);
int ax52_power_on(struct ax52_dev *rd);
void ax52_power_off(struct ax52_dev *rd);
void ax52_wcpu_disable(struct ax52_dev *rd);
int ax52_wcpu_enable_dl(struct ax52_dev *rd);

/* pci.c */
int ax52_pci_alloc_rings(struct ax52_dev *rd);
void ax52_pci_free_rings(struct ax52_dev *rd);
void ax52_pci_program_rings(struct ax52_dev *rd);
int ax52_pci_pre_init(struct ax52_dev *rd);
void ax52_pci_post_init(struct ax52_dev *rd);
void ax52_pci_deinit(struct ax52_dev *rd);
int ax52_fwcmd_tx(struct ax52_dev *rd, const void *hdr, u32 hdr_len,
		  const void *data, u32 len, bool fwdl);
int ax52_pci_irq_init(struct ax52_dev *rd);
void ax52_pci_irq_deinit(struct ax52_dev *rd);
void ax52_pci_start(struct ax52_dev *rd);		/* interrupts + NAPI on */
void ax52_pci_stop(struct ax52_dev *rd);
void ax52_pci_reset(struct ax52_dev *rd);		/* drop pending TX */
int ax52_pci_tx_avail(struct ax52_dev *rd, enum ax52_txq q);
/* @wd: 48 bytes of WD body + info; the frame must be one linear buffer */
int ax52_pci_tx(struct ax52_dev *rd, enum ax52_txq q, const __le32 *wd,
		struct sk_buff *skb);
void ax52_pci_tx_kick(struct ax52_dev *rd, enum ax52_txq q);
void ax52_pci_flush(struct ax52_dev *rd);

/* tx.c */
void ax52_tx_init(struct ax52_dev *rd);
void ax52_op_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
		struct sk_buff *skb);
void ax52_op_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq);
void ax52_tx_status(struct ax52_dev *rd, struct sk_buff *skb, u8 status);
void ax52_tx_resources_freed(struct ax52_dev *rd);

/* rx.c */
void ax52_rx_packet(struct ax52_dev *rd, struct sk_buff *skb);
u32 ax52_rx_payload_offset(const u8 *desc, u32 *pkt_len, u8 *type);

/* mac80211.c */
struct ax52_dev *ax52_alloc_hw(struct device *dev);
void ax52_free_hw(struct ax52_dev *rd);
int ax52_register_hw(struct ax52_dev *rd);
void ax52_unregister_hw(struct ax52_dev *rd);
void ax52_track_start(struct ax52_dev *rd);
void ax52_track_stop(struct ax52_dev *rd);

/* main.c */
int ax52_chip_start(struct ax52_dev *rd);
void ax52_chip_stop(struct ax52_dev *rd);

/* mac.c */
#define AX52_RX_FLTR_DEFAULT	0x030044BE
int ax52_mac_pre_fwdl(struct ax52_dev *rd);
int ax52_mac_init(struct ax52_dev *rd);		/* after FW download */
void ax52_mac_reset_bb_rf(struct ax52_dev *rd);
void ax52_mac_ppdu_status(struct ax52_dev *rd, bool enable);
void ax52_mac_set_rx_filter(struct ax52_dev *rd, u32 fltr);
void ax52_mac_set_rts_threshold(struct ax52_dev *rd, u32 thr);
void ax52_mac_set_agg_limit(struct ax52_dev *rd, u8 lmt);
int ax52_mac_wait_txq_empty(struct ax52_dev *rd);
void ax52_mac_port_update(struct ax52_dev *rd, struct ax52_vif *rv,
			  struct ieee80211_vif *vif);
void ax52_mac_macid_tbl_init(struct ax52_dev *rd, u8 mac_id);
void ax52_coex_init(struct ax52_dev *rd);
int ax52_lte_write(struct ax52_dev *rd, u32 off, u32 val);	/* coex space */
int ax52_lte_read(struct ax52_dev *rd, u32 off, u32 *val);
void ax52_coex_wl_only(struct ax52_dev *rd);		/* WL owns antennas */
void ax52_coex_release(struct ax52_dev *rd);		/* hand back to BT */
/* Bracket host-side RF calibration: BT handshake + force GNT to WL. */
void ax52_coex_rfk_begin(struct ax52_dev *rd);
void ax52_coex_rfk_end(struct ax52_dev *rd);

/* fw.c */
int ax52_fw_load(struct ax52_dev *rd);
void ax52_fw_release(struct ax52_dev *rd);
int ax52_fw_download(struct ax52_dev *rd);
/* TX-power family element for this RFE type (exact match, else rfe 0). */
const struct ax52_fw_elem *ax52_fw_txpwr_elem(struct ax52_dev *rd, u32 id);
const struct ax52_fw_elem *ax52_fw_elem(struct ax52_dev *rd, u32 id);

/* efuse.c */
int ax52_efuse_read(struct ax52_dev *rd);

/*
 * phy.c — baseband/RF access, tables, channel, TX power.
 * BB addresses are BB-relative (MMIO = addr + 0x10000). RF data is 20 bits.
 * All functions may sleep and are called with the wiphy mutex held.
 */
u32 ax52_bb_read(struct ax52_dev *rd, u32 addr);
void ax52_bb_write(struct ax52_dev *rd, u32 addr, u32 val);
u32 ax52_bb_read_mask(struct ax52_dev *rd, u32 addr, u32 mask);
void ax52_bb_write_mask(struct ax52_dev *rd, u32 addr, u32 mask, u32 val);
u32 ax52_rf_read(struct ax52_dev *rd, u8 path, u32 addr, u32 mask);
void ax52_rf_write(struct ax52_dev *rd, u8 path, u32 addr, u32 mask, u32 val);
int ax52_phy_alloc(struct ax52_dev *rd);
void ax52_phy_free(struct ax52_dev *rd);
int ax52_phy_init(struct ax52_dev *rd);		/* BB/RF tables + DM init */
int ax52_phy_set_channel(struct ax52_dev *rd,
			 const struct cfg80211_chan_def *chandef);
void ax52_phy_set_txpwr(struct ax52_dev *rd);	/* re-apply (regd change) */
void ax52_phy_track(struct ax52_dev *rd);		/* every 2 s */
u8 ax52_phy_thermal(struct ax52_dev *rd, u8 path);
void ax52_phy_bb_reset(struct ax52_dev *rd);

/* rfk.c — host-side RF calibration. Called with the wiphy mutex held. */
int ax52_rfk_alloc(struct ax52_dev *rd);
void ax52_rfk_free(struct ax52_dev *rd);
int ax52_rfk_init(struct ax52_dev *rd);		/* NCTL + power-on cals */
void ax52_rfk_channel(struct ax52_dev *rd);		/* after each set_channel */
void ax52_rfk_lck_check(struct ax52_dev *rd);	/* inside the RF channel write */
void ax52_rfk_sta_connect(struct ax52_dev *rd);	/* full per-channel cals */
void ax52_rfk_scan(struct ax52_dev *rd, bool start);
void ax52_rfk_track(struct ax52_dev *rd);		/* every 2 s */

/*
 * h2c.c — firmware command protocol.
 * Senders never sleep (some run in atomic mac80211 callbacks): they busy-wait,
 * bounded, for FWCMD ring space. C2H handling runs in NAPI (softirq) context.
 */
int ax52_h2c_alloc(struct ax52_dev *rd);
void ax52_h2c_free(struct ax52_dev *rd);
int ax52_h2c_send(struct ax52_dev *rd, u8 cat, u8 cls, u8 func,
		  bool rack, bool dack, const void *payload, u32 len);
/* Register mailbox. Returns 0 and fills @c2h (4 dwords) if requested. */
int ax52_h2creg_msg(struct ax52_dev *rd, const u32 h2c[4], u32 c2h[4]);
int ax52_mac_sch_tx_en(struct ax52_dev *rd, bool enable);	/* band 0 */
void ax52_c2h_handle(struct ax52_dev *rd, const u8 *c2h, u32 len);
int ax52_h2c_post_mac_init(struct ax52_dev *rd);	/* offload cfg, log cfg */
int ax52_h2c_rf_reg(struct ax52_dev *rd, u8 path, const u32 *words, u32 n);
int ax52_h2c_role_maintain(struct ax52_dev *rd, struct ax52_vif *rv,
			   bool remove);
int ax52_h2c_join_info(struct ax52_dev *rd, struct ax52_vif *rv,
		       bool disconnect);
int ax52_h2c_addr_cam(struct ax52_dev *rd, struct ax52_vif *rv,
		      struct ieee80211_vif *vif, struct ieee80211_sta *sta);
int ax52_h2c_default_cmac_tbl(struct ax52_dev *rd, u8 mac_id);
int ax52_h2c_assoc_cmac_tbl(struct ax52_dev *rd, struct ax52_vif *rv,
			    struct ieee80211_vif *vif, struct ieee80211_sta *sta);
int ax52_h2c_macid_pause(struct ax52_dev *rd, u8 mac_id, bool pause);
int ax52_h2c_edca(struct ax52_dev *rd, struct ax52_vif *rv, u8 ac,
		  const struct ieee80211_tx_queue_params *p);
int ax52_h2c_ra(struct ax52_dev *rd, struct ax52_vif *rv,
		struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		 bool update);
int ax52_h2c_ba_cam(struct ax52_dev *rd, struct ax52_vif *rv, u8 tid,
		    u16 ssn, u16 buf_size, bool valid);
/* Copy of the last FW rate report (taken under its lock); false if none. */
bool ax52_h2c_get_txrate(struct ax52_dev *rd, struct rate_info *txrate);
int ax52_h2c_rf_ntfy_mcc(struct ax52_dev *rd);	/* after sta RFK */
/* Write a key into security-CAM entry @idx (and @idx + 1 for 256-bit keys). */
int ax52_h2c_sec_cam(struct ax52_dev *rd, u8 idx, u8 type, const u8 *key,
		     u8 keylen);

#endif
