#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
"""
Independent parser for Realtek rtw89 "multi-firmware" (MFW) files, written from
the format description in docs/study/03-firmware-fwdl.md, used to validate that
description against /usr/lib/firmware/rtw89/rtw8852b_fw-2.bin(.zst).

Usage:
  zstd -dc /usr/lib/firmware/rtw89/rtw8852b_fw-2.bin.zst > rtw8852b_fw-2.bin
  python3 parse_fw.py rtw8852b_fw-2.bin [--cv 1] [--rfe 1] [--table-c path/to/rtw8852b_table.c]

Only reads the file; never touches hardware.
"""
import argparse
import os
import re
import struct
import sys

FW_TYPES = {1: "NORMAL", 3: "WOWLAN", 5: "NORMAL_CE", 14: "NORMAL_B",
            15: "WOWLAN_B", 64: "BBMCU0", 65: "BBMCU1", 255: "LOGFMT"}

ELM_NAMES = {
    0: "BBMCU0", 1: "BBMCU1", 2: "BB_REG", 3: "BB_GAIN", 4: "RADIO_A",
    5: "RADIO_B", 6: "RADIO_C", 7: "RADIO_D", 8: "RF_NCTL",
    9: "TXPWR_BYRATE", 10: "TXPWR_LMT_2GHZ", 11: "TXPWR_LMT_5GHZ",
    12: "TXPWR_LMT_6GHZ", 13: "TXPWR_LMT_RU_2GHZ", 14: "TXPWR_LMT_RU_5GHZ",
    15: "TXPWR_LMT_RU_6GHZ", 16: "TX_SHAPE_LMT", 17: "TX_SHAPE_LMT_RU",
    18: "TXPWR_TRK", 19: "RFKLOG_FMT", 20: "REGD",
    21: "TXPWR_DA_LMT_2GHZ", 22: "TXPWR_DA_LMT_5GHZ", 23: "TXPWR_DA_LMT_6GHZ",
    24: "TXPWR_DA_LMT_RU_2GHZ", 25: "TXPWR_DA_LMT_RU_5GHZ",
    26: "TXPWR_DA_LMT_RU_6GHZ", 27: "AFE_PWR_SEQ", 28: "DIAG_MAC", 29: "TX_COMP",
}
REG2_IDS = {2, 3, 4, 5, 6, 7, 8}
TXPWR_IDS = set(range(9, 18)) | set(range(21, 27))
# native (current rtw89) entry sizes; file entries may be shorter (zero-extended)
TXPWR_NATIVE_SZ = {9: 11, 10: 7, 11: 7, 12: 8, 13: 5, 14: 5, 15: 6, 16: 4, 17: 3,
                   21: 7, 22: 7, 23: 8, 24: 5, 25: 5, 26: 6}
TRK_TYPES = ["6GB_N", "6GB_P", "6GA_N", "6GA_P", "5GB_N", "5GB_P", "5GA_N",
             "5GA_P", "2GB_N", "2GB_P", "2GA_N", "2GA_P", "2G_CCK_B_N",
             "2G_CCK_B_P", "2G_CCK_A_N", "2G_CCK_A_P"]
DELTA_SWINGIDX_SIZE = 30
ELM_HDR_LEN = 32          # id,size,ver[4],aid,rsvd0,rsvd1,rsvd2 (24) + 8-byte union head
ELM_ALIGN = 16
FWDL_SECTION_PER_PKT_LEN = 2020   # AX chips: fixed part size
FWDL_SECURITY_SECTION_TYPE = 9
FWDL_SECTION_CHKSUM_LEN = 8
FWDL_SECURITY_SIGLEN = 512

# rtw89 fw_feat_tbl rows for RTL8852B (fw.c:865-877): (cond, ver, feature)
FEAT_8852B = [
    ("ge", (0, 29, 26, 0), "NO_LPS_PG"),
    ("ge", (0, 29, 26, 0), "TX_WAKE"),
    ("ge", (0, 29, 29, 0), "CRASH_TRIGGER_TYPE_0"),
    ("ge", (0, 29, 29, 0), "SCAN_OFFLOAD"),
    ("ge", (0, 29, 29, 7), "BEACON_FILTER"),
    ("ge", (0, 29, 29, 15), "BEACON_LOSS_COUNT_V1"),
    ("lt", (0, 29, 30, 0), "NO_WOW_CPU_IO_RX"),
    ("ge", (0, 29, 127, 0), "LPS_DACK_BY_C2H_REG"),
    ("ge", (0, 29, 127, 0), "SER_L1_BY_EVENT"),
    ("ge", (0, 29, 128, 0), "CRASH_TRIGGER_TYPE_1"),
    ("ge", (0, 29, 128, 0), "SCAN_OFFLOAD_EXTRA_OP"),
    ("ge", (0, 29, 128, 0), "BEACON_TRACKING"),
    ("ge", (0, 29, 130, 0), "SIM_SER_L0L1_BY_HALT_H2C"),
]


def vcode(v):
    return (v[0] << 24) | (v[1] << 16) | (v[2] << 8) | v[3]


def bits(v, hi, lo):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)


def u32(d, o):
    return struct.unpack_from("<I", d, o)[0]


# --------------------------------------------------------------------------
# MFW container
# --------------------------------------------------------------------------
def parse_mfw(d):
    if d[0] != 0xFF:
        return None
    fw_nr = d[1]
    ver = tuple(d[4:8])
    ents = []
    for i in range(fw_nr):
        o = 16 + 16 * i
        cv, typ, mp, _rsvd = d[o:o + 4]
        shift, size = struct.unpack_from("<II", d, o + 4)
        ents.append(dict(idx=i, cv=cv, type=typ, mp=mp, shift=shift, size=size))
    return dict(fw_nr=fw_nr, ver=ver, ents=ents)


def select_fw(mfw, fwtype, cv):
    """rtw89_mfw_recognize(): LOGFMT = first match; else best cv <= chip cv, mp==0."""
    best = None
    for e in mfw["ents"]:
        if e["type"] != fwtype:
            continue
        if fwtype == 255:
            return e
        if e["cv"] <= cv and not e["mp"]:
            if best is None or best["cv"] < e["cv"]:
                best = e
    return best


# --------------------------------------------------------------------------
# FW image header (v0 / v1)
# --------------------------------------------------------------------------
def parse_fw_image(d, off, size):
    w = struct.unpack_from("<8I", d, off)
    hdr_ver = bits(w[3], 31, 24)
    r = dict(off=off, size=size, hdr_ver=hdr_ver, w=w)
    r["ver"] = (bits(w[1], 7, 0), bits(w[1], 15, 8), bits(w[1], 23, 16), bits(w[1], 31, 24))
    r["commit"] = w[2]
    r["build"] = dict(mon=bits(w[4], 7, 0), day=bits(w[4], 15, 8),
                      hour=bits(w[4], 23, 16), min=bits(w[4], 31, 24))
    r["sec_num"] = bits(w[6], 15, 8)
    r["dyn_hdr"] = bits(w[7], 16, 16)
    r["idmem_share_mode"] = bits(w[7], 21, 18)
    if hdr_ver == 0:
        base = 32 + 16 * r["sec_num"]
        r["build"]["year"] = w[5]
        r["cmd_ver"] = bits(w[7], 31, 24)
        r["part_size_file"] = bits(w[7], 15, 0)
        hdr_len = bits(w[3], 23, 16) if r["dyn_hdr"] else base
        dsp_chk = 0
    elif hdr_ver == 1:
        w8 = struct.unpack_from("<12I", d, off)
        base = 48 + 16 * r["sec_num"]
        r["build"]["year"] = bits(w[5], 15, 0)
        r["cmd_ver"] = bits(w[7], 23, 16)   # rtw89 reads V1_W3 mask from w7 (sic)
        r["part_size_file"] = bits(w[7], 15, 0)
        hdr_len = bits(w[5], 31, 16) if r["dyn_hdr"] else base
        dsp_chk = bits(w[6], 24, 24)
    else:
        raise ValueError("unknown hdr_ver %d" % hdr_ver)
    r["base_hdr_len"] = base
    r["hdr_len"] = hdr_len
    r["dyn_hdr_len"] = hdr_len - base
    if r["dyn_hdr"]:
        dl, cnt = struct.unpack_from("<II", d, off + base)
        r["dyn_hdr_inner_len"] = dl
        r["dyn_hdr_sec_cnt"] = cnt
        r["dyn_hdr_ok"] = (dl == r["dyn_hdr_len"])
    secs = []
    bin_off = off + hdr_len
    sec_base = off + (32 if hdr_ver == 0 else 48)
    for i in range(r["sec_num"]):
        s0, s1, s2, s3 = struct.unpack_from("<4I", d, sec_base + 16 * i)
        typ = bits(s1, 27, 24)
        ln = bits(s1, 23, 0)
        chk = bits(s1, 28, 28)
        redl = bits(s1, 29, 29)
        if chk:
            ln += FWDL_SECTION_CHKSUM_LEN
        mssc = 0
        mssc_len = 0
        if typ == FWDL_SECURITY_SECTION_TYPE:
            mssc = s2 if hdr_ver == 0 else bits(s2, 7, 0)
            if (mssc & 0xFF) == 0xFF:
                mssc_len = None   # formatted MSS pool; not present in 8852B file
            else:
                mssc_len = mssc * FWDL_SECURITY_SIGLEN + (mssc * 8 if dsp_chk else 0)
        dladdr = s0 & 0x1FFFFFFF if hdr_ver == 0 else s0
        secs.append(dict(i=i, raw=(s0, s1, s2, s3), type=typ, len=ln, checksum=chk,
                         redl=redl, meta_hi=bits(s1, 31, 30), dladdr_raw=s0,
                         dladdr=dladdr, mssc=mssc, mssc_len=mssc_len,
                         file_off=bin_off))
        bin_off += ln + (mssc_len or 0)
    r["sections"] = secs
    r["end_ok"] = (bin_off == off + size)
    return r


# --------------------------------------------------------------------------
# Elements
# --------------------------------------------------------------------------
def headline_select(regs, rfe, cv):
    """rtw89_phy_sel_headline(): returns (headline_size, idx) or (n, None)."""
    n = 0
    for a, _ in regs:
        if (a >> 28) != 0xF:
            break
        n += 1
    if n == 0:
        return 0, 0
    tgt = lambda a: a & 0x0FFFFFFF
    cmp1 = (rfe << 16) | cv
    for i in range(n):
        if tgt(regs[i][0]) == cmp1:
            return n, i
    cmp2 = (rfe << 16) | 0xFF
    for i in range(n):
        if tgt(regs[i][0]) == cmp2:
            return n, i
    best, cvmax = None, 0
    for i in range(n):
        if bits(regs[i][0], 23, 16) == rfe and bits(regs[i][0], 7, 0) >= cvmax:
            cvmax, best = bits(regs[i][0], 7, 0), i
    if best is not None:
        return n, best
    for i in range(n):
        if bits(regs[i][0], 23, 16) == 0xFF and bits(regs[i][0], 7, 0) >= cvmax:
            cvmax, best = bits(regs[i][0], 7, 0), i
    return n, best


def resolve_phy_table(regs, rfe, cv):
    """rtw89_phy_init_reg(): which (addr,data) pairs would be applied."""
    hsize, hidx = headline_select(regs, rfe, cv)
    if hidx is None:
        return None, hsize, None
    cfg_target = regs[hidx][0] & 0x0FFFFFFF if hsize else None
    out = []
    target = 0
    matched, found = True, False
    for a, v in regs[hsize:]:
        c = a >> 28
        if c in (0x8, 0x9):
            target = a & 0x0FFFFFFF
        elif c == 0xA:
            matched = False
            if not found:
                return out, hsize, "ELSE without match"
        elif c == 0xB:
            matched, found = True, False
        elif c == 0x4:
            if found:
                matched = False
            elif target == cfg_target:
                matched, found = True, True
            else:
                matched, found = False, False
        else:
            if matched:
                out.append((a, v))
    return out, hsize, cfg_target


def parse_elements(d, start):
    off = (start + ELM_ALIGN - 1) & ~(ELM_ALIGN - 1)
    out = []
    while off + ELM_HDR_LEN < len(d):
        eid, size = struct.unpack_from("<II", d, off)
        if off + size >= len(d):
            out.append(dict(off=off, id=eid, size=size, error="size exceeds file"))
            break
        ver = d[off + 8:off + 12]
        aid = struct.unpack_from("<H", d, off + 12)[0]
        uhead = d[off + 24:off + 32]
        body = d[off + 32:off + 32 + size]
        out.append(dict(off=off, id=eid, size=size, ver=ver, aid=aid, uhead=uhead, body=body))
        off += ELM_HDR_LEN + size
        off = (off + ELM_ALIGN - 1) & ~(ELM_ALIGN - 1)
    return out, off


def load_c_table(path, name):
    """Extract {addr,data} pairs of a static rtw89_reg2_def array from a .c file."""
    txt = open(path).read()
    m = re.search(r"%s\[\]\s*=\s*\{(.*?)\n\};" % re.escape(name), txt, re.S)
    if not m:
        return None
    return [(int(a, 16), int(b, 16)) for a, b in
            re.findall(r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}", m.group(1))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--cv", type=int, default=1, help="hal.cv (R_AX_SYS_CFG1[15:12]); 1=CBV")
    ap.add_argument("--rfe", type=int, default=1, help="efuse rfe_type")
    ap.add_argument("--try-ce", type=int, default=1, help="chip->try_ce_fw (8852B: 1)")
    ap.add_argument("--table-c", default=None, help="rtw8852b_table.c to diff PHY tables")
    a = ap.parse_args()
    d = open(a.file, "rb").read()
    print("file: %s  size=%d (0x%x)" % (a.file, len(d), len(d)))

    mfw = parse_mfw(d)
    if not mfw:
        print("not an MFW file (legacy single image)")
        return
    v = mfw["ver"]
    print("\n== MFW header ==")
    print("sig=0xFF fw_nr=%d  mfw ver=%d.%d.%d.%d (ver_code 0x%08x)" %
          (mfw["fw_nr"], *v, vcode(v)))
    print(" idx  cv type             mp   shift      size     end")
    for e in mfw["ents"]:
        print(" %3d %3d %3d %-12s %2d  0x%07x 0x%07x 0x%07x" %
              (e["idx"], e["cv"], e["type"], FW_TYPES.get(e["type"], "?"), e["mp"],
               e["shift"], e["size"], e["shift"] + e["size"]))
    last = mfw["ents"][-1]
    mfw_end = last["shift"] + last["size"]
    print("rtw89_mfw_get_size() (last entry shift+size) = 0x%x" % mfw_end)

    print("\n== selection for cv=%d (try_ce_fw=%d) ==" % (a.cv, a.try_ce))
    normal = select_fw(mfw, 5, a.cv) if a.try_ce else None
    if not normal:
        normal = select_fw(mfw, 1, a.cv)
    wow = select_fw(mfw, 3, a.cv)
    logf = select_fw(mfw, 255, a.cv)
    for name, e in (("normal", normal), ("wowlan", wow), ("logfmt", logf)):
        print("  %-7s -> %s" % (name, "entry %d (type %s, cv %d, off 0x%x, size 0x%x)" %
              (e["idx"], FW_TYPES[e["type"]], e["cv"], e["shift"], e["size"]) if e else "none"))

    print("\n== FW image headers ==")
    for e in mfw["ents"]:
        if e["type"] == 255:
            cnt = u32(d, e["shift"] + 4)
            print("entry %d LOGFMT: rsvd=0x%x count=%d (ids[] then NUL-separated fmt strings)" %
                  (e["idx"], u32(d, e["shift"]), cnt))
            continue
        h = parse_fw_image(d, e["shift"], e["size"])
        b = h["build"]
        print("entry %d %s cv%d: hdr_ver=%d ver=%d.%d.%d.%d commit=%08x build=%04d-%02d-%02d %02d:%02d "
              "cmd_ver=%d" % (e["idx"], FW_TYPES[e["type"]], e["cv"], h["hdr_ver"], *h["ver"],
                             h["commit"], b["year"], b["mon"], b["day"], b["hour"], b["min"],
                             h["cmd_ver"]))
        print("   w0=0x%08x w3=0x%08x w6=0x%08x w7=0x%08x" % (h["w"][0], h["w"][3], h["w"][6], h["w"][7]))
        print("   sec_num=%d dyn_hdr=%d base_hdr_len=%d hdr_len=%d dyn_len=%d%s part_size(file)=%d "
              "idmem_share_mode=%d" % (h["sec_num"], h["dyn_hdr"], h["base_hdr_len"], h["hdr_len"],
                                       h["dyn_hdr_len"],
                                       (" (inner len=%d cnt=%d ok=%s)" % (h["dyn_hdr_inner_len"],
                                        h["dyn_hdr_sec_cnt"], h["dyn_hdr_ok"])) if h["dyn_hdr"] else "",
                                       h["part_size_file"], h["idmem_share_mode"]))
        for s in h["sections"]:
            nchunks = (s["len"] + FWDL_SECTION_PER_PKT_LEN - 1) // FWDL_SECTION_PER_PKT_LEN
            lastc = s["len"] - (nchunks - 1) * FWDL_SECTION_PER_PKT_LEN
            print("   sec%d type=%d len=0x%06x(%6d) chk=%d redl=%d w1[31:30]=%d dl_addr raw=0x%08x "
                  "masked=0x%08x mssc=%s mssc_len=%s file_off=0x%x -> %d chunk(s), last=%d" %
                  (s["i"], s["type"], s["len"], s["len"], s["checksum"], s["redl"], s["meta_hi"],
                   s["dladdr_raw"], s["dladdr"], s["mssc"], s["mssc_len"], s["file_off"],
                   nchunks, lastc))
        print("   sections end exactly at entry end: %s" % h["end_ok"])
        if normal and e is normal:
            tot = sum((s["len"] + FWDL_SECTION_PER_PKT_LEN - 1) // FWDL_SECTION_PER_PKT_LEN
                      for s in h["sections"])
            print("   ** selected NORMAL: FWDL = 1 header H2C (%d bytes FW hdr + 8 H2C hdr) + %d "
                  "section packets" % (h["hdr_len"] - h["dyn_hdr_len"], tot))
            vc = vcode(h["ver"])
            print("   ** fw features (RTL8852B rows of fw_feat_tbl) at ver_code 0x%08x:" % vc)
            for cond, fv, feat in FEAT_8852B:
                ok = vc >= vcode(fv) if cond == "ge" else vc < vcode(fv)
                print("        %-3s %d.%d.%d.%-3d %-26s %s" % (cond, *fv, feat, "SET" if ok else "-"))

    print("\n== Elements (start 0x%x, align %d) ==" % ((mfw_end + 15) & ~15, ELM_ALIGN))
    elms, endoff = parse_elements(d, mfw_end)
    print(" off       id name                  size   ver          aid  details")
    phy_tabs = {}
    txpwr_pick = {}
    for el in elms:
        if "error" in el:
            print(" 0x%06x ERROR %s" % (el["off"], el["error"]))
            continue
        eid = el["id"]
        name = ELM_NAMES.get(eid, "unknown")
        det = ""
        uh = el["uhead"]
        body = el["body"]
        if eid in REG2_IDS:
            n = el["size"] // 8
            regs = [struct.unpack_from("<II", body, 8 * i) for i in range(n)]
            res, hsize, tgt = resolve_phy_table(regs, a.rfe, a.cv)
            ncond = sum(1 for x, _ in regs[hsize:] if (x >> 28) in (4, 8, 9, 0xA, 0xB))
            det = "reg2 idx=%d n_regs=%d headlines=%d cond_lines=%d" % (uh[0], n, hsize, ncond)
            if hsize:
                det += " sel_target=0x%07x" % tgt
            det += " -> applied(rfe=%d,cv=%d)=%d" % (a.rfe, a.cv, len(res) if res is not None else -1)
            phy_tabs[eid] = regs
        elif eid in TXPWR_IDS or eid == 20:
            rfe = uh[2]
            ent_sz = uh[3]
            num = struct.unpack_from("<I", uh, 4)[0]
            det = "rfe_type=%d ent_sz=%d num_ents=%d (%s)" % (
                rfe, ent_sz, num, "size ok" if ent_sz * num == el["size"] else "SIZE MISMATCH")
            if eid in TXPWR_NATIVE_SZ:
                det += " native_ent=%d" % TXPWR_NATIVE_SZ[eid]
            if eid != 20:
                # rtw89_fw_recognize_txpwr_from_elm(): exact rfe wins (last one), else rfe 0
                cur = txpwr_pick.get(eid)
                if rfe == a.rfe or (rfe == 0 and (cur is None or cur[0] == 0)):
                    txpwr_pick[eid] = (rfe, el["off"])
        elif eid == 18:
            bm, _ = struct.unpack_from("<II", uh, 0)
            rows = 0
            names = []
            for t in range(16):
                if bm & (1 << t):
                    rows += 4 if t <= 3 else (3 if t <= 7 else 1)
                    names.append(TRK_TYPES[t])
            det = "bitmap=0x%04x rows=%d*%d=%d bytes (%s) types=%s" % (
                bm, rows, DELTA_SWINGIDX_SIZE, rows * DELTA_SWINGIDX_SIZE,
                "size ok" if rows * DELTA_SWINGIDX_SIZE == el["size"] else "MISMATCH",
                ",".join(names))
        print(" 0x%06x %3d %-20s %6d  %s  0x%x %s" % (el["off"], eid, name, el["size"],
              " ".join("%02x" % x for x in el["ver"]), el["aid"], det))
    print("elements end at 0x%x (file size 0x%x)" % (min(endoff, len(d)), len(d)))

    print("\n== TXPWR element chosen for rfe_type=%d ==" % a.rfe)
    for eid in sorted(txpwr_pick):
        print("  %-20s -> element with rfe_type %d at 0x%x" % (ELM_NAMES[eid], *txpwr_pick[eid]))

    regd = [el for el in elms if el.get("id") == 20]
    if regd:
        el = regd[-1]
        ent_sz = el["uhead"][3]
        num = struct.unpack_from("<I", el["uhead"], 4)[0]
        ents = []
        for i in range(num):
            e = el["body"][i * ent_sz:(i + 1) * ent_sz]
            fmap = struct.unpack_from("<I", e, 5)[0] if ent_sz >= 9 else 0
            ents.append("%s%s:%d/%d/%d/0x%x" % (chr(e[0]), chr(e[1]), e[2], e[3], e[4], fmap))
        print("\n== REGD sample (alpha2:rule2g/rule5g/rule6g/fmap), %d countries ==" % num)
        print("  " + " ".join(ents[:12]) + " ...")

    if a.table_c and os.path.exists(a.table_c):
        print("\n== element PHY tables vs compiled-in %s ==" % os.path.basename(a.table_c))
        m = {2: "rtw89_8852b_phy_bb_regs", 3: "rtw89_8852b_phy_bb_reg_gain",
             4: "rtw89_8852b_phy_radioa_regs", 5: "rtw89_8852b_phy_radiob_regs",
             8: "rtw89_8852b_phy_nctl_regs"}
        for eid, cname in m.items():
            ct = load_c_table(a.table_c, cname)
            et = phy_tabs.get(eid)
            if ct is None or et is None:
                print("  %-10s: n/a" % ELM_NAMES[eid])
                continue
            print("  %-10s: element %5d regs, built-in %5d regs, identical=%s" %
                  (ELM_NAMES[eid], len(et), len(ct), et == ct))


if __name__ == "__main__":
    main()
