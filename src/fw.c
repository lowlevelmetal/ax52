// SPDX-License-Identifier: GPL-2.0
/*
 * Firmware container parsing and firmware download.
 *
 * rtw8852b_fw-2.bin is a multi-firmware container: a header, a table of
 * images (per chip cut and purpose), the images themselves and finally a
 * list of "elements" carrying the BB/RF register tables and TX power data.
 */
#include <linux/delay.h>

#include "ax52.h"

/* container ("MFW") */
#define MFW_SIG			0xFF
#define MFW_TYPE_NORMAL		1
#define MFW_TYPE_NORMAL_CE	5

struct mfw_hdr {
	u8 sig;
	u8 fw_nr;
	u8 rsvd0[2];
	u8 ver[4];
	u8 rsvd1[8];
} __packed;

struct mfw_info {
	u8 cv;
	u8 type;
	u8 mp;
	u8 rsvd;
	__le32 shift;
	__le32 size;
	__le32 rsvd2;
} __packed;

/* image header (v0) */
#define FWHDR_W3_LEN		GENMASK(23, 16)
#define FWHDR_W3_HDR_VER	GENMASK(31, 24)
#define FWHDR_W6_SEC_NUM	GENMASK(15, 8)
#define FWHDR_W7_PART_SIZE	GENMASK(15, 0)
#define FWHDR_W7_DYN_HDR	BIT(16)
#define FWSEC_W1_SIZE		GENMASK(23, 0)
#define FWSEC_W1_TYPE		GENMASK(27, 24)
#define FWSEC_W1_CHECKSUM	BIT(28)
#define FWSEC_TYPE_SECURITY	9
#define FWSEC_SIGLEN		512

/* elements */
struct fw_elem_hdr {
	__le32 id;
	__le32 size;
	u8 ver[4];
	__le16 aid;
	__le16 rsvd0;
	__le32 rsvd1;
	__le32 rsvd2;
	u8 head[8];
} __packed;

enum {
	ELEM_BB_REG = 2,
	ELEM_BB_GAIN = 3,
	ELEM_RADIO_A = 4,
	ELEM_RADIO_B = 5,
	ELEM_RF_NCTL = 8,
};

static int fw_pick_image(struct ax52_dev *rd, const u8 *d, size_t sz,
			 const u8 **img, u32 *img_len, u32 *elem_off)
{
	const struct mfw_hdr *mh = (const void *)d;
	const struct mfw_info *mi = (const void *)(d + sizeof(*mh));
	const struct mfw_info *best = NULL;
	u64 last_end;
	int i, type;

	if (sz < sizeof(*mh) || mh->sig != MFW_SIG || !mh->fw_nr ||
	    sizeof(*mh) + mh->fw_nr * sizeof(*mi) > sz) {
		ax52_err(rd, "firmware is not a multi-firmware container\n");
		return -EINVAL;
	}

	/* 8852B ships its station firmware as the "CE" flavour */
	for (type = MFW_TYPE_NORMAL_CE; !best && type >= MFW_TYPE_NORMAL;
	     type -= MFW_TYPE_NORMAL_CE - MFW_TYPE_NORMAL) {
		for (i = 0; i < mh->fw_nr; i++) {
			if (mi[i].type != type || mi[i].mp || mi[i].cv > rd->cv)
				continue;
			if (!best || mi[i].cv > best->cv)
				best = &mi[i];
		}
	}
	if (!best) {
		ax52_err(rd, "no firmware image for chip cut %u\n", rd->cv);
		return -ENOENT;
	}
	if ((u64)le32_to_cpu(best->shift) + le32_to_cpu(best->size) > sz)
		return -EINVAL;

	*img = d + le32_to_cpu(best->shift);
	*img_len = le32_to_cpu(best->size);
	/* elements follow the last image; past the end means there are none */
	last_end = (u64)le32_to_cpu(mi[mh->fw_nr - 1].shift) +
		   le32_to_cpu(mi[mh->fw_nr - 1].size);
	*elem_off = min_t(u64, ALIGN(last_end, 16), sz);
	return 0;
}

static int fw_parse_image(struct ax52_dev *rd, const u8 *img, u32 len)
{
	struct ax52_fw *fw = &rd->fw;
	const __le32 *w = (const void *)img;
	u32 nsec, base_len, hdr_len, i;
	const u8 *bin, *end = img + len;

	if (len < 32)
		return -EINVAL;
	if (le32_get_bits(w[3], FWHDR_W3_HDR_VER) != 0) {
		ax52_err(rd, "unsupported firmware header version %u\n",
			 le32_get_bits(w[3], FWHDR_W3_HDR_VER));
		return -EINVAL;
	}

	fw->ver_major = le32_to_cpu(w[1]) & 0xff;
	fw->ver_minor = (le32_to_cpu(w[1]) >> 8) & 0xff;
	fw->ver_sub = (le32_to_cpu(w[1]) >> 16) & 0xff;
	fw->ver_idx = (le32_to_cpu(w[1]) >> 24) & 0xff;
	fw->commit = le32_to_cpu(w[2]);

	nsec = le32_get_bits(w[6], FWHDR_W6_SEC_NUM);
	if (!nsec || nsec > FW_MAX_SECTIONS)
		return -EINVAL;
	base_len = 32 + 16 * nsec;
	hdr_len = base_len;
	if (le32_get_bits(w[7], FWHDR_W7_DYN_HDR)) {
		hdr_len = le32_get_bits(w[3], FWHDR_W3_LEN);
		if (hdr_len < base_len + 8 || hdr_len > len ||
		    le32_to_cpu(*(const __le32 *)(img + base_len)) != hdr_len - base_len) {
			ax52_err(rd, "bad dynamic firmware header\n");
			return -EINVAL;
		}
	}
	if (base_len > len)
		return -EINVAL;

	fw->hdr = img;
	fw->hdr_send_len = base_len;	/* the dynamic header stays on the host */
	fw->nsec = nsec;

	bin = img + hdr_len;
	for (i = 0; i < nsec; i++) {
		const __le32 *s = (const void *)(img + 32 + 16 * i);
		struct ax52_fw_section *sec = &fw->sec[i];
		u64 skip = 0, span;

		sec->type = le32_get_bits(s[1], FWSEC_W1_TYPE);
		sec->len = le32_get_bits(s[1], FWSEC_W1_SIZE);
		if (le32_get_bits(s[1], FWSEC_W1_CHECKSUM))
			sec->len += 8;
		if (sec->type == FWSEC_TYPE_SECURITY) {
			u32 mssc = le32_to_cpu(s[2]);

			if ((mssc & 0xff) == 0xff) {
				ax52_err(rd, "formatted MSS key pools are not supported\n");
				return -EOPNOTSUPP;
			}
			/* signatures, only used for secure boot */
			skip = (u64)mssc * FWSEC_SIGLEN;
		}
		/* 64-bit, so a hostile size cannot wrap past the image end */
		span = (u64)sec->len + skip;
		if (span > (u64)(end - bin)) {
			ax52_err(rd, "firmware section %u exceeds the image\n", i);
			return -EINVAL;
		}
		sec->data = bin;
		bin += span;
	}
	if (bin != end) {
		ax52_err(rd, "firmware image size mismatch\n");
		return -EINVAL;
	}
	return 0;
}

static void fw_parse_elements(struct ax52_dev *rd, const u8 *d, size_t sz,
			      u32 off)
{
	struct ax52_fw *fw = &rd->fw;

	while (off + sizeof(struct fw_elem_hdr) < sz) {
		const struct fw_elem_hdr *h = (const void *)(d + off);
		u32 id = le32_to_cpu(h->id), size = le32_to_cpu(h->size);
		const void *payload = d + off + sizeof(*h);
		struct ax52_reg2_tbl *t = NULL;

		/* the loop condition guarantees sz > off + sizeof(*h) */
		if (size > sz - off - sizeof(*h)) {
			ax52_warn(rd, "truncated firmware element %u\n", id);
			break;
		}

		if (le16_to_cpu(h->aid) == 0) {
			switch (id) {
			case ELEM_BB_REG:
				t = &fw->bb;
				break;
			case ELEM_BB_GAIN:
				t = &fw->bb_gain;
				break;
			case ELEM_RADIO_A:
			case ELEM_RADIO_B:
				if (h->head[0] < 2)
					t = &fw->radio[h->head[0]];
				break;
			case ELEM_RF_NCTL:
				t = &fw->nctl;
				break;
			}
		}
		if (t && !t->pairs) {		/* first match wins */
			t->pairs = payload;
			t->n = size / 8;
		}

		if (le16_to_cpu(h->aid) == 0 && fw->nelem < FW_MAX_ELEMS) {
			struct ax52_fw_elem *e = &fw->elem[fw->nelem++];

			e->id = id;
			e->head = h->head;
			e->data = payload;
			e->size = size;
		}

		off = ALIGN(off + sizeof(*h) + size, 16);
	}
}

int ax52_fw_load(struct ax52_dev *rd)
{
	struct ax52_fw *fw = &rd->fw;
	const u8 *img;
	u32 img_len, elem_off;
	int ret;

	ret = request_firmware(&fw->blob, AX52_FW_NAME, rd->dev);
	if (ret) {
		ax52_err(rd, "cannot load %s: %d\n", AX52_FW_NAME, ret);
		return ret;
	}

	ret = fw_pick_image(rd, fw->blob->data, fw->blob->size, &img, &img_len,
			    &elem_off);
	if (!ret)
		ret = fw_parse_image(rd, img, img_len);
	if (ret) {
		ax52_fw_release(rd);
		return ret;
	}
	fw_parse_elements(rd, fw->blob->data, fw->blob->size, elem_off);

	ax52_info(rd, "firmware %u.%u.%u.%u (%08x), %u sections; tables: BB %u, gain %u, RF A %u, RF B %u, NCTL %u\n",
		  fw->ver_major, fw->ver_minor, fw->ver_sub, fw->ver_idx,
		   fw->commit, fw->nsec, fw->bb.n, fw->bb_gain.n,
		   fw->radio[0].n, fw->radio[1].n, fw->nctl.n);
	return 0;
}

const struct ax52_fw_elem *ax52_fw_elem(struct ax52_dev *rd, u32 id)
{
	int i;

	for (i = 0; i < rd->fw.nelem; i++)
		if (rd->fw.elem[i].id == id)
			return &rd->fw.elem[i];
	return NULL;
}

/*
 * TX-power family: an element whose rfe_type matches the board wins (the
 * last one if several); otherwise the first generic (rfe_type 0) one.
 */
const struct ax52_fw_elem *ax52_fw_txpwr_elem(struct ax52_dev *rd, u32 id)
{
	const struct ax52_fw_elem *pick = NULL;
	int i;

	for (i = 0; i < rd->fw.nelem; i++) {
		const struct ax52_fw_elem *e = &rd->fw.elem[i];
		u8 rfe = e->head[2];

		if (e->id != id)
			continue;
		if (rfe == rd->efuse.rfe_type)
			pick = e;
		else if (rfe == 0 && (!pick || pick->head[2] == 0))
			pick = e;
	}
	return pick;
}

void ax52_fw_release(struct ax52_dev *rd)
{
	release_firmware(rd->fw.blob);
	memset(&rd->fw, 0, sizeof(rd->fw));
}

/* ------------------------------------------------------------ download */

static int fwdl_wait_bit(struct ax52_dev *rd, u8 bit, const char *what)
{
	int ret = ax52_poll8(rd, REG_WCPU_FW_CTRL, bit, bit, 5, 500000);

	if (ret)
		ax52_err(rd, "timeout waiting for %s (fw_ctrl 0x%08x)\n", what,
			 rd32(rd, REG_WCPU_FW_CTRL));
	return ret;
}

static void fwdl_fail_dump(struct ax52_dev *rd)
{
	int i;

	ax52_err(rd, "fw_ctrl 0x%08x boot_dbg 0x%08x\n",
		 rd32(rd, REG_WCPU_FW_CTRL), rd32(rd, REG_BOOT_DBG));
	/* WCPU program counter via the debug port */
	wr32(rd, 0x0058, 0x00F200F2);
	mask32(rd, 0x00F4, GENMASK(17, 16), 1);
	for (i = 0; i < 4; i++) {
		ax52_err(rd, "wcpu pc 0x%08x\n", rd32(rd, 0x00C0));
		udelay(10);
	}
}

static int fwdl_once(struct ax52_dev *rd)
{
	struct ax52_fw *fw = &rd->fw;
	u8 hdrbuf[32 + 16 * FW_MAX_SECTIONS];
	__le32 h2c[2];
	u8 sts;
	int i, ret;

	ax52_wcpu_disable(rd);
	ret = ax52_wcpu_enable_dl(rd);
	if (ret)
		return ret;

	ret = fwdl_wait_bit(rd, FWCTRL_H2C_PATH_RDY, "H2C path");
	if (ret)
		goto fail;

	/* 1. the image header, wrapped in an H2C (cat MAC, class FWDL, func 0) */
	memcpy(hdrbuf, fw->hdr, fw->hdr_send_len);
	((__le32 *)hdrbuf)[7] &= ~cpu_to_le32(FWHDR_W7_PART_SIZE);
	((__le32 *)hdrbuf)[7] |= cpu_to_le32(FWDL_CHUNK);
	h2c[0] = cpu_to_le32(1 | (3 << 2) | ((u32)rd->h2c_seq << 24));
	h2c[1] = cpu_to_le32(fw->hdr_send_len + 8);

	spin_lock_bh(&rd->h2c_lock);
	ret = ax52_fwcmd_tx(rd, h2c, sizeof(h2c), hdrbuf, fw->hdr_send_len, false);
	spin_unlock_bh(&rd->h2c_lock);
	if (ret)
		goto fail;

	ret = fwdl_wait_bit(rd, FWCTRL_FWDL_PATH_RDY, "FWDL path");
	if (ret)
		goto fail;
	wr32(rd, REG_HALT_H2C_CTRL, 0);
	wr32(rd, REG_HALT_C2H_CTRL, 0);

	/* 2. section payloads, raw, in FWDL_CHUNK pieces */
	for (i = 0; i < fw->nsec; i++) {
		const u8 *p = fw->sec[i].data;
		u32 left = fw->sec[i].len;

		while (left) {
			u32 n = min_t(u32, left, FWDL_CHUNK);

			spin_lock_bh(&rd->h2c_lock);
			ret = ax52_fwcmd_tx(rd, NULL, 0, p, n, true);
			spin_unlock_bh(&rd->h2c_lock);
			if (ret)
				goto fail;
			p += n;
			left -= n;
		}
	}

	/* 3. wait for the firmware to boot */
	mdelay(5);
	ret = read_poll_timeout(rd8, sts,
				FIELD_GET(FWCTRL_FWDL_STS_MASK, sts) ==
					FWDL_STS_WCPU_FW_INIT_RDY,
				5, 500000, false, rd, REG_WCPU_FW_CTRL);
	if (ret) {
		switch (FIELD_GET(FWCTRL_FWDL_STS_MASK, sts)) {
		case FWDL_STS_CHECKSUM_FAIL:
			ax52_err(rd, "firmware checksum failure\n"); break;
		case FWDL_STS_SECURITY_FAIL:
			ax52_err(rd, "firmware security failure\n"); break;
		case FWDL_STS_CV_NOT_MATCH:
			ax52_err(rd, "firmware does not match chip cut\n"); break;
		default:
			ax52_err(rd, "firmware not ready, status %lu\n",
				 FIELD_GET(FWCTRL_FWDL_STS_MASK, sts));
		}
		goto fail;
	}

	rd->h2c_seq = 0;
	/* a failure seen by an earlier attempt belonged to that firmware */
	clear_bit(0, &rd->fw_failed);
	WRITE_ONCE(rd->fw_ready, true);
	return 0;

fail:
	fwdl_fail_dump(rd);
	return ret ?: -EIO;
}

int ax52_fw_download(struct ax52_dev *rd)
{
	int i, ret = -EIO;

	for (i = 0; i < 5; i++) {
		ret = fwdl_once(rd);
		if (!ret)
			return 0;
		ax52_warn(rd, "firmware download attempt %d failed (%d)\n",
			  i + 1, ret);
	}
	return ret;
}
