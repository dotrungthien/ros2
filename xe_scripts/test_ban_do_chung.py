#!/usr/bin/env python3
"""test_ban_do_chung.py (10/11 Thien duyet 17:59 "SEP VA NAV2 DUNG CHUNG MOT BAN DO"). GIA LAP: tuong map_xuong_v3, LiDAR gia,
Nav2 gia DUNG CHUNG costmap (tuong 251 + dem mem, LiDAR 254 + inflation, lop vong_vat tu gui_duong), NavFn gia vach lai 1 Hz.
KHONG chung minh tren xe. Chay: python3 xe_scripts/test_ban_do_chung.py"""
import math
import os
import sys
import tempfile
import time

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import test_gui_duong as X  # noqa: E402
import test_phan_loai_cum as P  # noqa: E402  (ghim ba dau hieu + thoat ra xa = yaml mac dinh)
import ban_do_chung as BC  # noqa: E402
import gac_vat_la as GV  # noqa: E402
import hop_den as HD  # noqa: E402
import the_gioi_gia as TG  # noqa: E402
import numpy as np  # noqa: E402

HV, G, BD = X.HV, X.G, X.BD
CFGC = dict(G.cau_hinh({}), so_luot=3)          # yaml mac dinh moi: giu_khe true, diem_dan / diem_vuot false
A, B = (0.0, 0.0), (1.60, 0.82)


def tam_o(c):
    res, ox, oy = BD['res'], BD['ox'], BD['oy']
    return lambda x, y: (int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res)))


class IOChung(X.NgatKhiChoNguoi):
    """Nav2 gia dung chung ban do: costmap_gia('dem') + LiDAR (tuong that + vat dang co) + lop vong_vat (gui_vong) + /chon_tuyen
    (khe kia 251, ton khong chan); NavFn gia, vach lai moi 1 s nhu RateController bt_mat."""

    def __init__(self, *a, lech=(0.0, 0.0), vong=True, **k):
        super().__init__(*a, **k)
        self.vong, self.vong_ds, self.lech, self.co_vong = [], [], lech, vong
        self.t_ke, self._cm, self._cm_k, self.duong_ds = 0.0, None, None, []

    def _dat(self, p):
        super()._dat(p)
        self.poses = getattr(self, 'poses', []) + [p]

    def than_vat(self):
        """Khoang nho nhat THAN xe (25 x 16) -> mep moi vat tron (ong) qua moi tu the da di."""
        out = 9.0
        for ds, _, _ in self.vat:
            xs = [a for a, _ in ds]
            cx, cy = sum(x for x, _ in xs) / len(xs), sum(y for _, y in xs) / len(xs)
            r = max(math.hypot(x - cx, y - cy) for x, y in xs)
            for x, y, th in getattr(self, 'poses', []):
                u = (cx - x) * math.cos(th) + (cy - y) * math.sin(th)
                v = -(cx - x) * math.sin(th) + (cy - y) * math.cos(th)
                out = min(out, math.hypot(u - min(max(u, -0.105), 0.145), v - min(max(v, -0.08), 0.08)) - r)
        return out

    def gui_vong(self, v):
        if self.co_vong:
            self.vong = list(v)
            self.vong_ds.append((round(self.t, 1), list(v)))

    def _vat(self):
        return [a for ds, t0, t1 in self.vat if t0 <= self.t < t1 for a, _ in ds]

    def costmap(self):
        k = (tuple(self.vong), tuple(i for i, (_, a, b) in enumerate(self.vat) if a <= self.t < b), self.chon_cuoi.split()[0])
        if k != self._cm_k:
            c = TG.costmap_gia(BD, 'dem', self.lech, vong=self.vong, vat=self._vat())
            cam = {'1': X.KHE_1, '2': X.KHE_2}.get(k[2])
            if cam is not None:
                for j in range(BD['h']):
                    for i in range(BD['w']):
                        if cam(BD['ox'] + (i + 0.5) * BD['res'], BD['oy'] + (j + 0.5) * BD['res']):
                            c[j, i] = max(c[j, i], 251)
            self._cm, self._cm_k, self._msg = c, k, TG.o_msg(c)
        return self._cm

    def o_costmap(self, x, y):
        self.costmap()
        i, j = tam_o(None)(x, y)
        return int(self._msg[j, i]) if 0 <= i < BD['w'] and 0 <= j < BD['h'] else None

    def _vach(self):
        n = self.nav
        d = TG.navfn_gia(self.costmap(), BD, self.p0[:2], n['dich'])
        n['duong'], n['i'], self.t_ke = d, 1, self.t + 1.0
        self.duong_ds.append((round(self.t, 1), d))

    def gui_nav(self, dich, yaw):
        super().gui_nav(dich, yaw)
        self._vach()
        return True

    def ngu(self, s):
        while s > 1e-9:
            b = min(s, 0.25)
            if self.nav is not None and not self.nav['xong'] and self.t >= self.t_ke - 1e-9:
                self._vach()
            super().ngu(b)
            s -= b


def chay(pose, vat=(), huong='di_B', cfg=CFGC, **k):
    io = IOChung(pose, vat=[(o, 0.0, 1e9) for o in vat], **k)
    try:
        ma = G.chay_mat(io, huong, 1, cfg)
    except KeyboardInterrupt:
        ma = 'CHO_NGUOI'
    return ma, io


def qua_khe_duoi(io):
    return any(0.61 <= x <= 1.16 and y < 0.33 for x, y in io.vet_xy)


def qua_khe_tren(io):
    return any(0.81 <= x <= 1.16 and y > 0.54 for x, y in io.vet_xy)


# ---------- phan thuan ----------
def test_01_vong_chi_quanh_vat_chac_khong_quanh_tuong():
    o = P.ong(0.68, 0.085, r=0.055)
    p = (0.25, 0.085, 0.0)
    q = TG.quet(X.TUONG + o, p)
    pm = GV.bien_doi(q, p)
    v, cum = BC.vong_quet(q, pm, X.BAN_GAC, GV.tham_so())
    assert len(v) == 1 and sum(c['nhan'] == 'tuong' for c in cum) >= 3, (v, [c['nhan'] for c in cum])
    x, y, r = v[0]
    assert math.hypot(x - 0.68, y - 0.085) <= 0.04 and 0.055 + 0.07 - 0.01 <= r <= 0.055 + 0.07 + 0.05, v


def test_02_giu_3s_roi_het_va_thay_toan_bo():
    g = BC.GiuVong(3.0)
    assert g.cap_nhat([(1.0, 0.0, 0.15)], 0.0) == [(1.0, 0.0, 0.15)]
    assert g.cap_nhat([(1.02, 0.0, 0.14)], 1.0) == [(1.02, 0.0, 0.14)]          # cung vat: thay, khong chong
    assert g.cap_nhat([], 3.9) == [(1.02, 0.0, 0.14)] and g.hien(4.1) == []
    assert BC.doc_vong(BC.chuoi_vong([(1.0, 2.0, 0.125)])) == [(1.0, 2.0, 0.125)] and BC.doc_vong('x;1 2') == []


def test_03_luoi_vong_cung_hinh_hoc_0_100():
    hh = dict(w=BD['w'], h=BD['h'], res=BD['res'], ox=BD['ox'], oy=BD['oy'])
    d = BC.luoi_vong(hh, [(0.68, 0.085, 0.125)])
    assert len(d) == BD['w'] * BD['h'] and set(d) == {0, 100} and 18 <= d.count(100) <= 40, d.count(100)
    assert BC.luoi_vong(hh, []) == [0] * (BD['w'] * BD['h'])


def test_04_khe_dong_voi_Nav2_khi_ong_giua_khe_duoi():
    o = P.ong(0.68, 0.085, r=0.055)
    pts = [a for a, _ in o]
    msg = lambda c: (lambda x, y: (lambda i, j: int(TG.o_msg(c)[j, i]) if 0 <= i < BD['w'] and 0 <= j < BD['h'] else None)(*tam_o(None)(x, y)))
    co = BC.do_khe(msg(TG.costmap_gia(BD, 'dem', vat=pts)), BC.KHE['duoi'])[0]
    vong = BC.do_khe(msg(TG.costmap_gia(BD, 'dem', vat=pts, vong=[(0.68, 0.085, 0.125)])), BC.KHE['duoi'])[0]
    print('   khe duoi: chi LiDAR %.2f m (Nav2 tuong qua duoc), co vong %.2f m' % (co, vong))
    assert co > 0 and vong == 0
    assert BC.chon_khe(msg(TG.costmap_gia(BD, 'dem', vat=pts, vong=[(0.68, 0.085, 0.125)])))[0] == 'tren'


def test_05_D4_tuong_dem_khong_dong_khe_nao_khi_lech_008():
    """Muc 2: bang D(4) (gia lap). Ba cach tuong; ban do lech 0 / +-0,08 (y) / +-0,08 (x). 'dem' (chon): khong khe nao dong."""
    msg = lambda c: (lambda x, y: (lambda i, j: int(TG.o_msg(c)[j, i]) if 0 <= i < BD['w'] and 0 <= j < BD['h'] else None)(*tam_o(None)(x, y)))
    for kieu in ('giu251', 'lethal', 'dem'):
        for lech in ((0, 0), (0, 0.08), (0, -0.08), (0.08, 0), (-0.08, 0)):
            c = TG.costmap_gia(BD, kieu, lech)
            f = msg(c)
            r = [BC.do_khe(f, BC.KHE[k])[0] for k in ('duoi', 'tren')]
            d = TG.navfn_gia(c, BD, A, B)
            print('   %-6s lech %+.2f %+.2f: khe duoi %.2f, khe tren %.2f, A %s, B %s, A->B %s' % (
                kieu, lech[0], lech[1], r[0], r[1], f(*A), f(*B), 'co' if d else 'KHONG'))
            if kieu == 'dem':
                assert min(r) >= 0.25 and f(*A) < 99 and f(*B) < 99 and d is not None


def test_06_su_kien_hop_den():
    assert HD.su_kien('SEP_LIDAR: DOI_VAT truoc vat chac cach 0.09 m') == 'DOI_VAT'
    assert HD.su_kien('   khong den duoc (KHONG_TIEN_10s, cach dich 0.50 m).') == 'KHONG_TIEN'
    assert HD.su_kien('SEP_LIDAR: THOAT 1/4 ...') == 'THOAT' and HD.su_kien('== Luot 1 bi chan (xa dich 1/3). ==') == 'LUOT_CHAN'
    assert HD.su_kien('KHE: chon tren ...') == 'KHE' and HD.su_kien('DIEM_DAN: bo ...') == 'DIEM'
    assert HD.su_kien('DEN_NOI: di_B xong') == 'DEN_NOI' and HD.su_kien('MEM: sach') is None


def test_07_hop_den_luu_ve_xoay_vong_va_CPU():
    o = P.ong(0.68, 0.085, r=0.055)
    p = (0.25, 0.085, 0.0)
    q = TG.quet(X.TUONG + o, p)
    pm = GV.bien_doi(q, p)
    vong, ds = BC.vong_quet(q, pm, X.BAN_GAC, GV.tham_so())
    c = TG.o_msg(TG.costmap_gia(BD, 'dem', vat=[a for a, _ in o]))
    cm = (BD['w'], BD['h'], BD['res'], BD['ox'], BD['oy'], c.ravel().tolist())
    cl = (60, 60, 0.05, -1.25, -1.4, [0] * 3600)
    with tempfile.TemporaryDirectory() as tm:
        h = HD.HopDen(tm, 'thu', tran_mb=0.05)
        t0 = time.process_time()
        for k in range(20):
            f = h.luu('DI' if k else 'DOI_VAT', 1e9 + k, p, cm, cl, (0.0, 0.0, 0.0), [(0.25, 0.08), (1.0, 0.7)], pm, ds,
                      vong, '{"tin": true}', 'SEP_LIDAR: DOI_VAT')
        ms = (time.process_time() - t0) / 20 * 1000
        z = np.load(f)
        assert z['cm_g'].shape == (BD['h'], BD['w']) and z['cm_l'].shape == (60, 60) and len(z['vong']) == 1
        assert (z['nhan'] == 1).sum() > 50 and ((z['nhan'] == 2) | (z['nhan'] == 3)).sum() >= 3
        tong = sum(os.path.getsize(os.path.join(tm, 'thu', x)) for x in os.listdir(os.path.join(tm, 'thu')))
        assert tong <= 0.05e6 and len(os.listdir(os.path.join(tm, 'thu'))) < 20          # xoay vong: tep cu bi xoa
        import ve_hop_den as VH
        assert os.path.exists(VH.ve_tep(f)) and os.path.exists(VH.ve_chuyen(os.path.join(tm, 'thu')))
    print('   hop den: %.1f ms / tep (moi 2 s => %.2f %% mot nhan)' % (ms, ms / 2000 * 100))
    assert ms / 2000 * 100 < 3.0


# ---------- gia lap chuyen (Nav2 gia dung chung ban do) ----------
def test_10_ca_1748_ong_giua_khe_duoi_chon_khe_tren_ngay_DEN_NOI():
    """Ong O11 (r 0,055) giua khe duoi o hep nhat (0,68; 0,085): vong dong khe duoi voi Nav2 => KHE chon tren ngay, DEN_NOI, khong
    CHO_NGUOI, khong luot bi chan, khong lat qua lai."""
    ma, io = chay((0.25, 0.085, 0.0), [P.ong(0.68, 0.085, r=0.055)])
    khe = [c for c in io.cau if c.startswith('KHE:')]
    assert ma == 0 and khe and 'chon tren' in khe[0], (ma, khe, io.cau[-12:])
    assert not any(c.startswith('== Luot') or c.startswith('CHO_NGUOI') for c in io.cau), io.cau
    assert qua_khe_tren(io) and not qua_khe_duoi(io)
    assert io.than_vat() >= 0.06 - 0.005, io.than_vat()                       # ong: nhieu 5 mm cua da giac


def test_11_doi_chung_khong_vong_Nav2_vach_vao_khe_duoi():
    """Doi chung (tat vong_vat, giu_khe): cung ca 17:48, Nav2 gia vach qua khe duoi (19 cm moi ben) => sep chan (goc cu)."""
    cfg = dict(CFGC, giu_khe=False)
    ma, io = chay((0.25, 0.085, 0.0), [P.ong(0.68, 0.085, r=0.055)], cfg=cfg, vong=False)
    d0 = io.duong_ds[0][1]
    assert d0 is not None and any(0.61 <= x <= 0.75 and y < 0.33 for x, y in d0), d0
    print('   khong vong: ma %s, %d lan vach, dong: %s' % (ma, len(io.duong_ds), [c for c in io.cau if 'DOI_VAT' in c or 'Luot' in c][:3]))


def test_12_ca_1708_ong_ap_H0_khe_35cm_qua_khe_duoi():
    o = P.ong(0.68, 0.335 - 0.075, r=0.075)
    ma, io = chay((0.25, 0.0, 0.0), [o])
    khe = [c for c in io.cau if c.startswith('KHE:')]
    assert ma == 0 and khe and 'chon duoi' in khe[0], (ma, khe, io.cau[-10:])
    assert qua_khe_duoi(io) and not any(c.startswith('CHO_NGUOI') for c in io.cau)
    assert io.than_vat() >= 0.06 - 0.005, io.than_vat()                       # ong: nhieu 5 mm cua da giac


def test_13_khong_ong_di_giua_khe_khong_sat_K4():
    ma, io = chay((0.25, 0.085, 0.0), [])
    assert ma == 0 and qua_khe_duoi(io), io.cau[-6:]
    sat = min(math.hypot(x - min(max(x, 0.384), 0.734), y - (-0.165)) for x, y in io.vet_xy if 0.40 <= x <= 0.80 and y > -0.165)
    tran = min(0.335 - y for x, y in io.vet_xy if 0.61 <= x <= 1.16)
    print('   khong ong: tam xe cach mep K4 >= %.3f m, cach H0 >= %.3f m' % (sat, tran))
    assert sat >= 0.18 and tran >= 0.15


def test_14_ong_truoc_mui_giua_san_van_dung_006_doi_30s():
    o = P.ong(0.0 + 0.145 + 0.09 + 0.09, 0.45)
    ma, io = chay((0.0, 0.45, 0.0), [o])
    i = next((k for k, c in enumerate(io.cau) if c.startswith('SEP_LIDAR: DOI_VAT truoc vat chac')), None)
    assert i is not None and any(c.startswith('CHO_VAT: 5/30') for c in io.cau[i:]), io.cau[:12]
    assert io.than_vat() >= 0.06 - 0.005, (ma, io.than_vat(), io.cau[-6:])
    print('   ong truoc mui: ket qua %s (thoat sau 30 s: %s)' % (ma, [c for c in io.cau if c.startswith('SEP_LIDAR: THOAT')][:2]))


def test_15_ve_A_tu_B_ong_bi_H0_che_khe_dong_giua_chung_bo_diem_dan_khong_tinh_luot():
    """Tu B ong giua khe duoi bi H0 che: luot dau chon khe duoi (thong); lai gan thay ong => vong dong khe => bo diem dan (khong tinh
    luot), Nav2 tu vach qua khe tren => DEN_NOI, khong luot bi chan, khong CHO_NGUOI, than luon cach vat >= 0,06."""
    ma, io = chay((1.60, 0.82, math.pi), [P.ong(0.68, 0.085, r=0.055)], huong='ve_A')
    assert ma == 0 and any('HONG (khe duoi dong voi Nav2)' in c for c in io.cau), io.cau
    assert not any(c.startswith('== Luot') or c.startswith('CHO_NGUOI') for c in io.cau), io.cau
    assert not any(0.55 <= x <= 0.80 and y < 0.33 for x, y in io.vet_xy) and io.than_vat() >= 0.06 - 0.005, io.than_vat()


if __name__ == '__main__':
    import traceback
    ca = sorted((k, f) for k, f in globals().items() if k.startswith('test_') and callable(f))
    dat = 0
    for k, f in ca:
        try:
            f()
            dat += 1
            print('PASS ', k)
        except Exception:
            print('FAIL ', k)
            traceback.print_exc()
    print('TONG: %d/%d dat (gia lap)' % (dat, len(ca)))
    sys.exit(0 if dat == len(ca) else 1)
