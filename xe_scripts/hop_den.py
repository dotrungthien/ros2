#!/usr/bin/env python3
"""hop_den.py (10/11 Thien 17:59 muc 0, CHUA CHAY TREN XE): hop den cua gui_duong. Phan thuan (numpy), khong ROS.

Moi su kien (DOI_VAT, CHO_VAT, KHONG_TIEN, THOAT*, DIEM_*, KHE, LUOT_CHAN, CHO_NGUOI, DEN_NOI) va moi nhip_s khi dang di: MOT tep
~/hop_den/<chuyen>/<HHMMSS_mmm>_<su_kien>.npz: global + local costmap (mang + [w, h, res, ox, oy]), map<-odom, /plan, tu the,
diem quet khung ban do + nhan cum (tuong | ong | vat | khac), vong an toan, trang thai sep, dong log. Tran tran_mb MB / chuyen:
vuot => xoa tep CU NHAT cua chuyen do (xoay vong). Ve tren PC: python3 ve_hop_den.py ~/hop_den/<chuyen>.
"""
import os
import re
import time

import numpy as np

NHAN = ('khac', 'tuong', 'ong', 'vat')
_SU_KIEN = [(re.compile(p), t) for p, t in (
    (r'^SEP_LIDAR: DOI_VAT', 'DOI_VAT'), (r'^CHO_VAT:', 'CHO_VAT'), (r'KHONG_TIEN', 'KHONG_TIEN'),
    (r'^(SEP_LIDAR: )?THOAT', 'THOAT'), (r'^DIEM_(DAN|VUOT)', 'DIEM'), (r'^KHE:', 'KHE'), (r'^== Luot', 'LUOT_CHAN'),
    (r'^CHO_NGUOI', 'CHO_NGUOI'), (r'^DEN_NOI', 'DEN_NOI'), (r'^HET_DUONG', 'HET_DUONG'))]


def su_kien(dong):
    """Dong log gui_duong -> ten su kien | None."""
    s = dong.strip()
    return next((t for p, t in _SU_KIEN if p.search(s)), None)


def nhan_diem(n, ds):
    """danh_gia (ks, nhan, loai) -> mang n ma nhan (chi so trong NHAN)."""
    out = np.zeros(n, dtype=np.int8)
    for d in ds or ():
        ma = 2 if d.get('nhan') == 'ong' else 1 if d.get('loai') == 'TUONG' else 3
        for k in d.get('ks') or ():
            out[k] = max(out[k], ma)
    return out


def _luoi(cm):
    """(w, h, res, ox, oy, data) | None -> (mang int8 [h, w], [w, h, res, ox, oy])."""
    if cm is None:
        return np.zeros((0, 0), np.int8), np.zeros(5)
    w, h, res, ox, oy, data = cm
    return np.asarray(data, dtype=np.int8).reshape(h, w), np.array([w, h, res, ox, oy], dtype=float)


class HopDen:
    def __init__(self, goc, chuyen, tran_mb=20.0):
        self.thu_muc = os.path.join(goc, chuyen)
        self.tran = int(tran_mb * 1e6)
        self.n = 0

    def luu(self, ten, t, pose=None, cm_g=None, cm_l=None, map_odom=None, plan=(), pts_map=None, ds=None, vong=(), sep='', dong=''):
        """-> duong dan tep (hoac None khi loi ghi: hop den KHONG duoc lam hong chuyen)."""
        try:
            os.makedirs(self.thu_muc, exist_ok=True)
            g, gi = _luoi(cm_g)
            l, li = _luoi(cm_l)
            pm = [(p if p is not None else (np.nan, np.nan)) for p in (pts_map or [])]
            p = os.path.join(self.thu_muc, '%s_%03d_%s.npz' % (time.strftime('%H%M%S', time.localtime(t)), int(t * 1000) % 1000, ten))
            np.savez_compressed(p, su_kien=ten, t=t, pose=np.array(pose if pose is not None else (np.nan,) * 3, dtype=float),
                                cm_g=g, cm_g_info=gi, cm_l=l, cm_l_info=li,
                                map_odom=np.array(map_odom if map_odom is not None else (np.nan,) * 3, dtype=float),
                                plan=np.array(list(plan) or np.zeros((0, 2)), dtype=float).reshape(-1, 2),
                                pts=np.array(pm, dtype=float).reshape(-1, 2), nhan=nhan_diem(len(pm), ds),
                                vong=np.array(list(vong) or np.zeros((0, 3)), dtype=float).reshape(-1, 3), sep=str(sep or ''), dong=str(dong))
            self.n += 1
            self._xoay()
            return p
        except (OSError, ValueError):
            return None

    def _xoay(self):
        ts = sorted((os.path.getmtime(os.path.join(self.thu_muc, f)), os.path.join(self.thu_muc, f))
                    for f in os.listdir(self.thu_muc) if f.endswith('.npz'))
        tong = sum(os.path.getsize(f) for _, f in ts)
        while ts and tong > self.tran:
            _, f = ts.pop(0)
            tong -= os.path.getsize(f)
            os.remove(f)
