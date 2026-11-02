#!/usr/bin/env python3
"""Thu gui_duong.py va phan ham moi cua tinh_tuyen.py (viec 109a) KHONG can ROS: python3 xe_scripts/test_gui_duong.py
IO gia: dong ho gia, ket qua FollowPath dat san theo tung lan gui. Costmap = du_lieu_0111/costmap_v3_020 (ban do xuong 01/11).
KHONG chung minh chay tren xe: chi kiem logic chuyen (thu lai, cho, DUNG, CSV, ma thoat) va duong day."""
import contextlib
import io as _io
import math
import os
import re

import gui_duong as G
import tinh_tuyen as T

DAY = os.path.dirname(os.path.abspath(__file__))
BAN = T.Ban(T.YAML)
CFG = G.cau_hinh({})


class IOGia:
    """kq: danh sach (ket_qua, giay chay, vi tri cuoi) cho tung lan gui. lenh(t) -> (chu, ly) | None; ban_phu(t) -> bool."""

    def __init__(self, kq, pose=(0.0, 0.0, 0.0), lenh=None, ban_phu=None, ban=BAN):
        self.t, self.p, self.kq_ds, self.b = 0.0, pose, list(kq), ban
        self.lenh = lenh or (lambda t: None)
        self.ban_phu = ban_phu or (lambda t: False)
        self.rows, self.cau, self.duong, self.gui_ds, self.da_huy = [], [], [], [], 0
        self.dang = None

    def now(self):
        return self.t

    def ngu(self, s):
        self.t += s

    def bao(self, c):
        self.cau.append(c)

    def ghi(self, *a):
        self.rows.append([str(x) for x in a])

    def cho_san_sang(self):
        return self.p, self.b

    def pose(self):
        return self.p

    def ban(self):
        return self.b

    def lenh_sep(self):
        return self.lenh(self.t)

    def sep_dang_giao_thoat(self):
        return False

    def phu_lai_ban(self):
        return self.ban_phu(self.t)

    def phat_duong(self, d):
        self.duong.append(list(d))

    def gui(self, d):
        self.gui_ds.append((self.t, list(d)))
        kq, giay, cuoi = self.kq_ds.pop(0)
        self.dang = (self.t + giay, kq, cuoi)
        return kq != 'TU_CHOI'

    def ket_qua(self):
        if self.dang and self.t >= self.dang[0]:
            _, kq, cuoi = self.dang
            self.dang, self.p = None, cuoi
            return kq
        return None

    def huy(self):
        self.da_huy += 1
        self.dang = None


Bp = (1.60, 0.82, 0.0)
CUA = lambda rows: [r[4] for r in rows]                 # cot ket_qua


def test_01_toi_noi_lan_dau():
    io = IOGia([('SUCCEEDED', 20, (1.55, 0.80, 0.0))])
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'] and all(r[0] == 'di_B_day' for r in io.rows)
    d = io.gui_ds[0][1]
    assert math.hypot(d[0][0], d[0][1]) < 0.01 and math.hypot(d[-1][0] - 1.60, d[-1][1] - 0.82) < 0.01 and d[-1][2] == 0.0
    assert all(not BAN.do(BAN.o(x, y)) for x, y, _ in d)
    assert len(io.duong) >= 2                                          # /duong_day: luc gui + cap nhat 2 lan/giay


def test_02_succeeded_lech_khong_tinh_la_toi():
    io = IOGia([('SUCCEEDED', 20, (1.30, 0.70, 0.0)), ('SUCCEEDED', 10, (1.58, 0.81, 0.0))])
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI']
    t1, d2 = io.gui_ds[1]
    assert t1 - (io.gui_ds[0][0] + 20) >= 30.0                         # cho >= 30 s sau lan hong
    assert math.hypot(d2[0][0] - 1.30, d2[0][1] - 0.70) < 0.01         # lan 2 tinh lai tu vi tri hien tai


def test_03_ba_lan_hong_thi_doi_tuyen():
    hong = [('ABORTED', 5, (0.40, 0.10, 0.0))] * 3
    io = IOGia(hong + [('SUCCEEDED', 30, (1.59, 0.80, 0.0))])
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'KHONG_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI']
    assert [r[1] for r in io.rows[1:5]] == ['1', '1', '1', '2'] and [r[3] for r in io.rows[1:4]] == ['1', '2', '3']
    d4 = io.gui_ds[3][1]
    assert any(y > 0.5 and 0.3 <= x <= 1.2 for x, y, _ in d4)          # tuyen 2 di khe tren


def test_04_ca_hai_hong_het_duong():
    io = IOGia([('ABORTED', 5, (0.40, 0.10, 0.0))] * 6)
    assert G.chay(io, 'di_B', 1, CFG) == 1
    assert len(io.gui_ds) == 6 and CUA(io.rows)[-1] == 'HET_DUONG' and io.rows[-1][1:3] == ['-', 'het']


def test_05_dung_giua_chung_huy_dich():
    io = IOGia([('SUCCEEDED', 60, Bp)], lenh=lambda t: ('DUNG', 'HET_3_LUOT_THOAT') if t >= 12 else ('DI', ''))
    assert G.chay(io, 'di_B', 1, CFG) == 4
    assert io.da_huy >= 1 and CUA(io.rows) == ['KIEM_DAT', 'DUNG_SEP']


def test_06_cho_truoc_khi_gui_toi_da_hai_lan():
    io = IOGia([('SUCCEEDED', 20, Bp)], lenh=lambda t: ('CHO', 'TF_CU'))
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert abs(io.gui_ds[0][0] - 2 * CFG['cho_sep']) < 0.5             # CHO 2 lan x 10 s roi gui (nhu chuyen.sh)


def test_07_phu_lai_dang_lai_thi_cho_them():
    io = IOGia([('ABORTED', 5, (0.60, 0.04, -1.5)), ('SUCCEEDED', 20, Bp)], ban_phu=lambda t: 5 <= t < 50)
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert io.gui_ds[1][0] >= 50                                       # 30 s chua du: phu lai con lai toi 50 s
    io = IOGia([('ABORTED', 5, (0.60, 0.04, -1.5)), ('SUCCEEDED', 20, Bp)], ban_phu=lambda t: t >= 5)
    assert G.chay(io, 'di_B', 1, CFG) == 0 and io.gui_ds[1][0] <= 5 + 3 * CFG['cho_hong'] + 1   # tran: 2 x cho_hong sau hong + 1 x cho_hong truoc khi gui


def test_08_sai_vi_tri():
    io = IOGia([], pose=None)
    assert G.chay(io, 'di_B', 1, CFG) == 3 and CUA(io.rows) == ['SAI_VI_TRI'] and not io.gui_ds
    k = next(k for k, v in BAN.v.items() if v == 0)
    io = IOGia([], pose=BAN.tam(k) + (0.0,))
    assert G.chay(io, 'di_B', 1, CFG) == 3 and not io.gui_ds


def test_09_ham_nho():
    assert G.doc_sep_lenh('100.00 DUNG HET_3_LUOT_THOAT', 101.0) == ('DUNG', 'HET_3_LUOT_THOAT')
    assert G.doc_sep_lenh('100.00 CHO', 100.5) == ('CHO', '')
    assert G.doc_sep_lenh('100.00 DUNG x', 104.5) is None and G.doc_sep_lenh('100.00 DUNG x', 98.0) is None
    assert G.doc_sep_lenh('', 1.0) is None and G.doc_sep_lenh('100 THOAT 1 2 3 4', 100.0) is None and G.doc_sep_lenh('x DI', 1.0) is None
    d = [(0.05 * k, 0.0, 0.0) for k in range(10)]
    assert G.cat_con_lai(d, (0.21, 0.02, 0.0))[0] == d[4]
    assert all(G.phu_lai_ban(t) for t in ('RA_LUI', 'RA_TIEN', 'XOAY_P', 'THOAT_CUNG'))
    assert not any(G.phu_lai_ban(t) for t in ('RA_XONG', 'XOAY_XONG', 'NGHI', 'CHO_3', '', None))


def test_10_ban_tu_luoi_giong_ban_pgm():
    data = [0] * (BAN.w * BAN.h)
    for (i, j), v in BAN.v.items():
        data[j * BAN.w + i] = {0: 100, 205: -1, 254: 0}[v]
    b2 = T.Ban.tu_luoi(BAN.w, BAN.h, BAN.res, BAN.ox, BAN.oy, data)
    assert b2.v == BAN.v and b2.d == BAN.d
    assert [T.pixel_tu_chi_phi(c) for c in (-1, 0, 29, 30, 97, 98, 99, 100)] == [205, 254, 254, 205, 205, 0, 0, 0]
    for tuyen in (1, 2):
        assert T.tinh_duong(b2, T.A, T.B, tuyen) == T.tinh_duong(BAN, T.A, T.B, tuyen)


def test_11_main_van_in_dung_bang_xs_cua_chuyen_sh():
    cu = T.ve
    T.ve = lambda *a: False                                           # khong ghi de anh trong kho
    try:
        buf = _io.StringIO()
        with contextlib.redirect_stdout(buf):
            T.main()
    finally:
        T.ve = cu
    dong = [d for d in buf.getvalue().splitlines() if d.strip().startswith('xs:')]
    sh = open(os.path.join(DAY, 'chuyen.sh')).read()
    assert len(dong) == 4 and all(d in sh for d in dong)


def test_12_duong_day_va_vung_cam():
    for huong, dau, cuoi in (('di_B', T.A, T.B), ('ve_A', T.B, T.A)):
        for tuyen in (1, 2):
            d = T.tinh_duong(BAN, dau, cuoi, tuyen, kieu='muot')                # duong muot 109a: 6 diem xs nam tren duong
            kc = [math.hypot(d[k][0] - d[k - 1][0], d[k][1] - d[k - 1][1]) for k in range(1, len(d))]
            assert max(kc) <= 0.0501 and not any(BAN.do(BAN.o(x, y)) for x, y, _ in d)
            m = re.search(r'xs:%s:%d\) WP=\((.*)\) ;;' % (huong, tuyen), open(os.path.join(DAY, 'chuyen.sh')).read()).group(1)
            xs = [tuple(map(float, q.split()[:2])) for q in re.findall(r'"([^"]+)"', m)]
            assert max(min(math.hypot(x - a, y - b) for a, b, _ in d) for x, y in xs) < 0.03   # 6 diem xs nam tren duong day
    assert len(T.tinh_duong(BAN, (0.60, 0.11), T.B, 1)) > 10                 # tu giua khe duoi (sau thoat)
    try:
        T.tinh_duong(BAN, (0.60, 0.76), T.B, 1)                           # dang o khe tren ma doi tuyen 1: khong ra duoc
        assert False
    except T.KhongCoDuong:
        pass
    assert len(T.tinh_duong(BAN, (0.60, 0.76), T.B, 2)) > 10


def test_13_luat_G_csv_ma_thoat_va_khong_goi_ros2_cli():
    sh = open(os.path.join(DAY, 'chuyen.sh')).read()
    assert '"%s" > "$CSV"' % G.COT_CSV in sh                         # cung tieu de CSV voi chuyen.sh
    sp = open(os.path.join(DAY, 'sep.py')).read()
    for chu in ('KIEM_DAT', 'DEN_NOI', 'HET_DUONG', 'SAI_VI_TRI', 'DUNG_SEP'):
        assert "'%s'" % chu in sp                                      # sep.py _csv nhan dung cac chu bat dau / ket thuc chuyen
    assert re.search(r'0 den noi; 1 het duong; 2 sai doi so; 3 [^;]*; 4 sep bao DUNG', sh)
    src = open(os.path.join(DAY, 'gui_duong.py')).read()
    assert not re.search(r'subprocess|os\.system|Popen|ACCESS', src) and src.count('class GuiDuongNode(Node)') == 1
    assert "'/duong_day'" in src and "'follow_path'" in src and "'/global_costmap/costmap'" in src


def test_14_dung_huy_dich_dang_chay_ca_hai_bang():
    sh = open(os.path.join(DAY, 'chuyen.sh')).read()
    gui = sh[sh.index('gui() {'):sh.index('\n}\n', sh.index('gui() {'))]
    assert 'ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"' in sh   # huy het dich cua bt_navigator
    assert 'while kill -0 "$P"' in gui and 'doc_sep' in gui and gui.index('huy_nav') < gui.index('exit 4')    # BANG=xs: hoi sep moi giay khi dang chay
    ts = sh[sh.index('theo_sep() {'):sh.index('CHO)', sh.index('theo_sep() {'))]
    assert 'huy_nav' in ts                                             # DUNG truoc khi gui cung huy dich con sot
    io = IOGia([('SUCCEEDED', 60, Bp)], lenh=lambda t: ('DUNG', 'x') if t >= 5 else None)
    assert G.chay(io, 'di_B', 1, CFG) == 4 and io.da_huy >= 1 and io.dang is None   # BANG=day: huy /follow_path


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _dinh(d):
    """Cac dinh re (x, y, goc re do) cua duong day doan thang."""
    return [(d[k][0], d[k][1], math.degrees(abs(_wrap(d[k][2] - d[k - 1][2])))) for k in range(1, len(d) - 1) if abs(_wrap(d[k][2] - d[k - 1][2])) > 1e-6]


def test_15_duong_doan_thang_tren_xuong():
    import lai_xe_lidar as L
    assert abs(T.R_XOAY_CAN - (L.R_XOAY + 0.05)) < 1e-3                # cung R_XOAY voi phu lai
    y = open(os.path.join(DAY, 'nav2_params_cho.yaml')).read()
    assert float(re.search(r'rotate_to_heading_min_angle:\s*([0-9.]+)', y).group(1)) == T.GOC_XOAY   # nguong dinh re = nguong xoay cua RPP
    for huong, dau, cuoi in (('di_B', T.A, T.B), ('ve_A', T.B, T.A)):
        for tuyen in (1, 2):
            cb = []
            d = T.tinh_duong(BAN, dau, cuoi, tuyen, canh_bao=cb)
            assert not any(BAN.do(BAN.o(x, y)) for x, y, _ in d)
            assert (not cb) if tuyen == 1 else [(round(x, 2), round(y, 2)) for x, y, _ in cb] == [(0.41, 0.76)]   # khe tren: 1 dinh thieu 1,2 cm
            assert all(L.R_XOAY < c <= T.R_XOAY_CAN for _, _, c in cb)      # dinh canh bao van xoay duoc (> R_XOAY), chi thieu 5 cm du phong
            kc = [math.hypot(d[k][0] - d[k - 1][0], d[k][1] - d[k - 1][1]) for k in range(1, len(d))]
            assert max(kc) <= 0.0501
            for x, y, g in _dinh(d):
                assert g < 20 or T.cach_do(BAN, x, y) > T.R_XOAY_CAN or (x, y) in [(a, b) for a, b, _ in cb], (huong, tuyen, x, y, g)
            m = re.search(r'xs:%s:%d\) WP=\((.*)\) ;;' % (huong, tuyen), open(os.path.join(DAY, 'chuyen.sh')).read()).group(1)
            xs = [tuple(map(float, q.split()[:2])) for q in re.findall(r'"([^"]+)"', m)]
            assert max(min(math.hypot(x - a, y - b) for a, b, _ in d) for x, y in xs) < 0.06   # Douglas-Peucker 5 cm
    re52 = [v for v in _dinh(T.tinh_duong(BAN, T.A, T.B, 1)) if abs(v[0] - 1.26) < 0.03 and abs(v[1] - 0.06) < 0.03]
    assert re52 and 45 <= re52[0][2] <= 60                             # dung khuc re 52 do thay tren xe 17:57


def _ban_phong(do_o):
    """Ban 40 x 30 o (5 cm), trong tru cac o (i, j) trong do_o (do)."""
    data = [0] * (40 * 30)
    for i, j in do_o:
        data[j * 40 + i] = 100
    return T.Ban.tu_luoi(40, 30, 0.05, 0.0, 0.0, data)


def test_16_doi_dinh_re_toi_cho_du_cho_xoay():
    b = _ban_phong([(i, j) for i in range(21, 24) for j in range(0, 2)])     # khoi do o x 1,05..1,20 y 0..0,10 sat goc re
    pl = [(0.20 + 0.05 * k, 0.20) for k in range(17)] + [(1.00, 0.25 + 0.05 * k) for k in range(16)]   # re 90 do tai (1,00; 0,20)
    assert T.cach_do(b, 1.00, 0.20) <= T.R_XOAY_CAN
    dinh, cb = T.doan_thang(b, pl)
    assert len(dinh) == 3 and not cb
    v = dinh[1]
    assert v != (1.00, 0.20) and T.cach_do(b, *v) > T.R_XOAY_CAN and T.doan_hop_le(b, dinh[0], v) and T.doan_hop_le(b, v, dinh[2])
    b2 = _ban_phong([(i, j) for i in range(12, 28) for j in range(0, 2)] + [(i, 8) for i in range(12, 19)] + [(23, j) for j in range(3, 20)])
    dinh2, cb2 = T.doan_thang(b2, pl)                                  # hanh lang hep quanh goc: khong cho nao du => canh bao, giu dinh
    assert cb2 and dinh2[1] == (1.00, 0.20)
    cb3 = []
    T.tinh_duong(BAN, T.A, T.B, 2, canh_bao=cb3)                       # 18:2x: doi (0,41; 0,76) ve (0,21; 0,56) se cat cheo khe tren 15 cm
    assert cb3 and all(abs(c - T.cach_do(BAN, x, y)) < 1e-9 for x, y, c in cb3)   # => khong doi, chi canh bao


def _mo_phong_xe(duong, goc_xoay, v=0.15, L=0.30, b=0.16, s_min=0.092, dt=0.05):
    """Xe do choi: pure pursuit, RPP ha toc khi ban kinh < 0,9 m (san 0,06 m/s), xoay tai cho khi lech carrot > goc_xoay;
    MOI BANH khong chay cham hon s_min (0,092 m/s, nhu xe that). Chi de so sanh hai cach, KHONG phai RPP."""
    x, y, th = duong[0][0], duong[0][1], duong[1][2]
    pts, ra, i0 = [(p[0], p[1]) for p in duong], [], 0
    kep = lambda s: 0.0 if abs(s) < 1e-9 else math.copysign(max(abs(s), s_min), s)
    for _ in range(1600):
        i0 = min(range(i0, min(i0 + 20, len(pts))), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
        cx, cy = pts[next((k for k in range(i0, len(pts)) if math.hypot(pts[k][0] - x, pts[k][1] - y) >= L), len(pts) - 1)]
        if math.hypot(pts[-1][0] - x, pts[-1][1] - y) < 0.05:
            break
        a = _wrap(math.atan2(cy - y, cx - x) - th)
        if abs(a) > goc_xoay:
            vc, wc = 0.0, math.copysign(1.0, a)
        else:
            k = 2 * math.sin(a) / L
            vc = v if abs(k) < 1 / 0.9 else max(0.06, v / (abs(k) * 0.9))
            wc = vc * k
        vl, vr = kep(vc - wc * b / 2), kep(vc + wc * b / 2)
        x, y, th = x + (vl + vr) / 2 * math.cos(th) * dt, y + (vl + vr) / 2 * math.sin(th) * dt, th + (vr - vl) / b * dt
        ra.append((x, y))
    return ra


def test_17_khuc_re_52_do_xe_toi_thieu_0092():
    doan, muot = T.tinh_duong(BAN, T.A, T.B, 1), T.tinh_duong(BAN, T.A, T.B, 1, kieu='muot')
    ref = [(p[0], p[1]) for p in doan]

    def lech(p):
        best = 1e9
        for k in range(len(ref) - 1):
            (ax, ay), (cx, cy) = ref[k], ref[k + 1]
            L2 = (cx - ax) ** 2 + (cy - ay) ** 2
            t = 0 if L2 == 0 else max(0, min(1, ((p[0] - ax) * (cx - ax) + (p[1] - ay) * (cy - ay)) / L2))
            best = min(best, math.hypot(p[0] - ax - t * (cx - ax), p[1] - ay - t * (cy - ay)))
        return best
    kq = {}
    for ten, d, g in (('cu', muot, 0.785), ('moi', doan, 0.35)):
        tr = _mo_phong_xe(d, g)
        gan = [p for p in tr if math.hypot(p[0] - 1.26, p[1] - 0.06) < 0.45]
        kq[ten] = (max(lech(p) for p in gan), min(T.cach_do(BAN, *p) for p in gan), math.hypot(tr[-1][0] - 1.6, tr[-1][1] - 0.82))
    assert kq['cu'][0] > 0.10 and kq['cu'][1] < 0.10                  # cach cu: cua rong, sat o do (giong 17:57)
    assert kq['moi'][0] < 0.08 and kq['moi'][1] > 0.12 and kq['moi'][2] < 0.06, kq   # cach moi: lech nho, con cho, van toi B


if __name__ == '__main__':
    import sys
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
    print('TONG: %d/%d dat' % (dat, len(ca)))
    sys.exit(0 if dat == len(ca) else 1)
