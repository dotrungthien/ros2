#!/usr/bin/env python3
"""Thu gui_duong.py va phan ham moi cua tinh_tuyen.py (viec 109a) KHONG can ROS: python3 xe_scripts/test_gui_duong.py
IO gia: dong ho gia, ket qua FollowPath dat san theo tung lan gui. Costmap = du_lieu_0111/costmap_v3_020 (ban do xuong 01/11).
KHONG chung minh chay tren xe: chi kiem logic chuyen (thu lai, cho, DUNG, CSV, ma thoat) va duong day."""
import contextlib
import copy
import io as _io
import math
import os
import re
import signal
import time

import gui_duong as G
import tinh_tuyen as T

DAY = os.path.dirname(os.path.abspath(__file__))
BAN = T.Ban(T.YAML)
CFG = dict(G.cau_hinh({}), so_vong=1, cho_lai=0.0)    # ca cu: moi khe 1 lan, het thi HET_DUONG (truoc 18:44 ②a / 18:47 (i)); ca moi dung CFG3
CFG3 = G.cau_hinh({})
CFG['gac'] = False          # 09/11 dung im: test cu = hanh vi 48995c2 (gac tat); test gac dung CFG_GAC
CFG_GAC = dict(CFG, gac=True)


class IOGia:
    """kq: danh sach (ket_qua, giay chay, vi tri cuoi) cho tung lan gui. lenh(t) -> (chu, ly) | None; ban_phu(t) -> bool."""

    def __init__(self, kq, pose=(0.0, 0.0, 0.0), lenh=None, ban_phu=None, ban=BAN, lop2=None):
        self.t, self.p, self.kq_ds, self.b = 0.0, pose, list(kq), ban
        self.lenh = lenh or (lambda t: None)
        self.ban_phu = ban_phu or (lambda t: False)
        # 04/11 lop 2: lop2(t, ma, t_sap_gui) -> (tra loi sep | None, so lan RA_XONG); mac dinh sep tra KHONG_CAN ngay
        self.lop2 = lop2 or (lambda t, ma, t0: (dict(chu='KHONG_CAN', ma_luot=ma, ly='vung_trong'), 0))
        self.sap_gui = []
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

    def so_lan_phu_lai_xong(self):
        return sum(1 for t in getattr(self, 'xong_luc', ()) if t <= self.t)

    def da_xu_ly_thoat(self):
        self.da_xl = getattr(self, 'da_xl', 0) + 1

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

    def gui_sap_gui(self, huong, r, lan):
        self.sap_gui.append((self.t, huong, r, lan))
        return len(self.sap_gui)

    def huy_luot(self):
        self.huy_luot_t = self.t

    def co_dich(self):                                   # 06/11 viec 121
        return self.dang is not None

    def tra_lop2(self):
        return self.lop2(self.t, len(self.sap_gui), self.sap_gui[-1][0])[0] if self.sap_gui else None

    def so_lan_ra_xong(self):
        return self.lop2(self.t, len(self.sap_gui), self.sap_gui[-1][0])[1] if self.sap_gui else 0


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
    assert len(T.tinh_duong(BAN, (0.60, 0.76), T.B, 1)) > 10            # 09/11 19:05: khe tren + tuyen 1: vung CAM[1] la TON, van ra duoc
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


def test_18_tinh_lai_sau_khi_phu_lai_xoay_xong():
    io = IOGia([('SUCCEEDED', 60, Bp), ('SUCCEEDED', 20, (1.58, 0.81, 0.0))])
    io.xong_luc = [10.0]                                              # phu lai XOAY xong luc 10 s (dich dau con dang chay)
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'TINH_LAI', 'SUCCEEDED', 'DEN_NOI'] and io.rows[2][3] == '1'   # khong mat mot lan
    assert io.da_huy >= 1 and io.gui_ds[1][0] < 12.0                  # huy duong cu, gui lai ngay (khong cho 30 s)
    io = IOGia([('SUCCEEDED', 60, Bp)] * 5)
    io.xong_luc = [5.0, 6.0, 7.0, 8.0]                                # tinh lai toi da LUOT lan moi tuyen
    G.chay(io, 'di_B', 1, CFG)
    assert CUA(io.rows).count('TINH_LAI') == CFG['luot']


def test_19_chuoi_chan_xoay_sep_thoat_ra_xoay_gui_lai():
    import lai_xe_lidar as L
    import mo_phong_he_thong as M
    import sep as S
    pose = [0.60, -0.005, -math.pi / 2]                               # mui cach K4 3 cm (nhu xe 17:57 sau khi kep)
    quet = lambda: L.scan_to_xy(M.make_scan(M.XUONG, tuple(pose)))
    b = L.NguoiLai(sep_ra_thoat=True)
    ten = []
    for k in range(40):                                               # RPP muon xoay tai cho ve doan moi nhung bi chan
        ten.append(b.step(quet(), 0.1 * (k + 1), True, None, (0.0, 1.0))[2])
    assert 'CHAN_XOAY' in ten and b.state == 'CHO_SEP' and b.ly_do == 'XOAY_BI_CHAN_QUA_LAU' and 'THOAT_CUNG' not in ten
    doi = '%s -> %s (%s)%s | %s'                                      # khuon cau doi trang thai cua nut lai_xe_lidar
    day = [(x, y) for x, y, _ in T.lam_day_doan([(0.60, -0.005), (1.26, 0.06), (1.56, 0.41)])]
    sep = S.Sep()
    v0 = dict(pts=quet(), scan_tuoi=0.05, pose=tuple(pose), tf_tuoi=0.1, mo=(0.0, 0.0, 0.0), grid=M.GRID, ket=False, gio='t', ra_thoat=True)
    t = 0.0
    for k in range(15):                                               # 1,5 s: co dich /follow_path, /duong_day; vi tri TIN
        t += 0.1
        tt, _ = sep.vong(t, dict(v0, goals_fp=[[(b'f', 2)]] if k == 0 else [], duong_day=[day]))
    tt, log = sep.vong(t + 0.1, dict(v0, rosout=[('lai_xe_lidar', doi % ('CHAN_XOAY', 'CHO_SEP', b.ly_do, '', 'x'))], duong_day=[day]))
    th = tt['thoat']
    yaw_doan = math.atan2(0.065, 0.66)
    assert th and th['luot'] == 1 and th['p'] == (1.26, 0.06) and abs(th['yaw'] - yaw_doan) < 0.01, th   # giao NGAY, khong doi 10 s
    assert sep.luot_thoat == 0 and any(',L1_CHO_SEP,' in d for d in log)   # chua dem luot: phu lai chua lam gi
    td = L.doc_sep_thoat('%.2f THOAT %s' % (100.0, S.ly_thoat(th)), 100.5)
    ok, cau = b.ra_bat_dau(quet(), 4.1, True)
    assert ok and cau.startswith('NHUONG CHO_SEP: RA BAT DAU LUI'), cau
    t, cham = 4.1, False
    for _ in range(300):
        t += 0.1
        v, w, n = b.step(quet(), t, True, None, None, pose[2] + 0.3, L.goc_xoay_ve(tuple(pose), td['p'], td['yaw']))
        ten.append(n)
        if v is not None:
            pose = [pose[0] + v * math.cos(pose[2]) * 0.1, pose[1] + v * math.sin(pose[2]) * 0.1, pose[2] + w * 0.1]
            cham = cham or M.cham(M.XUONG, tuple(pose))
        if n == 'XOAY_XONG':
            break
    assert 'RA_LUI' in ten and 'XOAY_P' in ten and ten[-1] == 'XOAY_XONG' and not cham, ten[-5:]
    assert b.xoay_xong.startswith('XOAY XONG DEN_HUONG') and abs(_wrap(pose[2] - yaw_doan)) < math.radians(26), (b.xoay_xong, math.degrees(pose[2]))
    for n in ('RA_LUI', 'RA_XONG', 'XOAY_P', 'XOAY_XONG'):            # sep dem luot khi RA/XOAY chay
        sep.vong(t, dict(v0, pose=tuple(pose), rosout=[('lai_xe_lidar', doi % ('-', n, '', '', 'x'))], duong_day=[day]))
    assert sep.luot_thoat == 1 and sep.thoat_xong_t is not None
    assert not G.phu_lai_ban('XOAY_XONG') and G.phu_lai_ban('XOAY_P')   # gui_duong: XOAY_XONG = xong => TINH_LAI (test_18)


class _XeCham:
    """Gia nut that tren Pi: costmap toi sau t_cm s; doi luoi ra Ban mat 2 s ma KHONG spin; tf 'moi' chi khi da spin trong 1 s."""

    def __init__(self, t_cm=3.0):
        self.t, self.t_cm, self.t_spin = 0.0, t_cm, 0.0

    def now(self):
        return self.t

    def ngu(self, s):
        self.t += s
        self.t_spin = self.t

    def co_cm(self):
        return self.t >= self.t_cm

    def lay_ban(self):
        self.t += 2.0                                                 # doi costmap 52 x 32 tren Pi: vai giay, khong spin
        return BAN

    def lay_pose(self):
        return (0.0, 0.0, 0.0) if self.t - self.t_spin <= 1.0 else None


def test_20_cho_tf_costmap_toi_15_s():
    x = _XeCham(3.0)
    pose, ban = G.cho_tf_costmap(x.co_cm, x.lay_ban, x.lay_pose, x.now, x.ngu)
    assert pose is not None and ban is BAN and x.t < 6.0               # costmap 3 s + doi 2 s + mot lan spin: van co tf moi
    x = _XeCham(99.0)
    pose, ban = G.cho_tf_costmap(x.co_cm, x.lay_ban, x.lay_pose, x.now, x.ngu)
    assert ban is None and 15.0 <= x.t < 15.2 and G.thieu_gi(pose, ban) == 'costmap'
    x = _XeCham(0.0)
    x.lay_pose = lambda: None
    pose, ban = G.cho_tf_costmap(x.co_cm, x.lay_ban, x.lay_pose, x.now, x.ngu)
    assert pose is None and ban is BAN and G.thieu_gi(pose, ban) == 'tf moi'


def test_21_costmap_toi_sau_3_s_di_ngay_khong_lan_2():
    class IOCham(IOGia):
        def cho_san_sang(self):
            x = self.x
            x.t = self.t
            p, b = G.cho_tf_costmap(x.co_cm, x.lay_ban, x.lay_pose, x.now, x.ngu)
            self.t = x.t
            return p, b
    io = IOCham([('SUCCEEDED', 20, Bp)])
    io.x = _XeCham(3.0)
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'] and len(io.gui_ds) == 1 and io.gui_ds[0][0] < 12.0
    assert not any('khong co' in c for c in io.cau)



class _NgatGiua(IOGia):
    """Bi ngat (KeyboardInterrupt nhu SIGINT/SIGTERM trong main) luc t >= 5 s khi dich dang chay."""

    def ngu(self, s):
        IOGia.ngu(self, s)
        if self.t >= 5 and not getattr(self, 'da_ngat', False):
            self.da_ngat = True
            raise KeyboardInterrupt

    def huy_khi_ngat(self):
        assert getattr(self, 'huy_luot_t', None) is not None      # 05/11 lop 2: HUY ghi truoc khi huy dich (cho <= 2 s)
        self.huy_ngat = getattr(self, 'huy_ngat', 0) + 1
        return 'ok' if self.dang else 'khong_co_dich'


def test_22_bi_ngat_huy_dich_ghi_NGAT_ma_130():
    io = _NgatGiua([('SUCCEEDED', 60, Bp)])
    bo = []
    assert G.chay_co_ngat(io, 'di_B', 1, CFG, lambda: bo.append(1)) == 130
    assert bo == [1] and io.huy_ngat == 1 and 'HUY_DICH_KHI_NGAT ok' in io.cau
    assert CUA(io.rows) == ['KIEM_DAT', 'NGAT'] and io.rows[-1] == ['di_B_day', '-', 'ngat', '0', 'NGAT', '0']
    assert "'NGAT'" in open(os.path.join(DAY, 'sep.py')).read()     # luat G: sep.py _csv dong chuyen khi NGAT
    assert '130 = bi ngat (NGAT)' in open(os.path.join(DAY, 'chuyen.sh')).read()
    io2 = IOGia([('SUCCEEDED', 20, Bp)])                              # khong bi ngat: nhu chay()
    assert G.chay_co_ngat(io2, 'di_B', 1, CFG) == 0 and CUA(io2.rows)[-1] == 'DEN_NOI'
    assert getattr(io2, 'huy_luot_t', None) is None                   # khong bi ngat: khong ghi HUY
    assert G.dong_huy(12.345) == '12.35 HUY'


# ---------- 06/11 viec 121: SIGINT roi vao rclpy take_message => pybind doi KeyboardInterrupt thanh RuntimeError ----------
class _LoiGiua(_NgatGiua):
    """Luc t >= 5 s (dich dang chay): tin_hieu=True: gui SIGINT THAT cho chinh tien trinh (G.ngat dang ky), bat
    KeyboardInterrupt roi nem RuntimeError nhu pybind; tin_hieu=False: chi nem RuntimeError (loi khac, khong ngat)."""

    def __init__(self, *a, tin_hieu=True, t_loi=5, **k):
        _NgatGiua.__init__(self, *a, **k)
        self.tin_hieu, self.t_loi = tin_hieu, t_loi

    def ngu(self, s):
        IOGia.ngu(self, s)
        if self.t >= self.t_loi and not getattr(self, 'da_ngat', False):
            self.da_ngat = True
            if self.tin_hieu:
                try:
                    os.kill(os.getpid(), signal.SIGINT)
                    time.sleep(1.0)                          # tin hieu toi trong luc ngu; khong toi thi test hong
                except KeyboardInterrupt:
                    pass
            raise RuntimeError('Unable to convert call argument to Python object (compile in debug mode for details)')

    def huy_khi_ngat(self):
        if self.dang:
            assert getattr(self, 'huy_luot_t', None) is not None      # con dich: HUY ghi truoc khi huy dich
        self.huy_ngat = getattr(self, 'huy_ngat', 0) + 1
        kq = 'ok' if self.dang else 'khong_co_dich'
        self.dang = None
        return kq


def _voi_ngat(f):
    cu = signal.signal(signal.SIGINT, G.ngat)
    G.DA_NGAT[0] = False
    try:
        return f()
    finally:
        signal.signal(signal.SIGINT, cu)
        G.DA_NGAT[0] = False


def test_31_ngat_thanh_runtimeerror_khi_co_dich_van_huy_NGAT_130():
    try:
        G.ngat(signal.SIGINT, None)
        assert False, 'ngat phai raise KeyboardInterrupt'
    except KeyboardInterrupt:
        assert G.DA_NGAT[0] is True                                   # co dat TRUOC khi raise
    G.DA_NGAT[0] = False
    io = _LoiGiua([('SUCCEEDED', 60, Bp)])
    bo = []
    ma = _voi_ngat(lambda: G.chay_co_ngat(io, 'di_B', 1, CFG, lambda: bo.append(1)))
    assert ma == 130, ma
    assert bo == [1] and io.huy_ngat == 1 and 'HUY_DICH_KHI_NGAT ok' in io.cau
    assert io.huy_luot_t is not None
    assert CUA(io.rows) == ['KIEM_DAT', 'NGAT'] and io.rows[-1] == ['di_B_day', '-', 'ngat', '0', 'NGAT', '0']


def test_32_loi_khac_khi_co_dich_van_huy_dich_ma_5():
    io = _LoiGiua([('SUCCEEDED', 60, Bp)], tin_hieu=False)
    bo = []
    ma = _voi_ngat(lambda: G.chay_co_ngat(io, 'di_B', 1, CFG, lambda: bo.append(1)))
    assert ma == G.MA_LOI == 5, ma
    assert bo == [1] and io.huy_ngat == 1 and io.huy_luot_t is not None
    assert any(c.startswith('LOI_GUI_DUONG RuntimeError: Unable to convert') and c.endswith('; HUY_DICH ok') for c in io.cau), io.cau
    assert io.rows[-1] == ['di_B_day', '-', 'loi', '0', 'LOI_GUI_DUONG', '0']
    assert not any(r[4] == 'NGAT' for r in io.rows)


def test_33_loi_khi_chua_co_dich_khong_ghi_HUY_ma_5_va_D1():
    io = _LoiGiua([('SUCCEEDED', 60, Bp)], tin_hieu=False)

    def hong():                                                       # loi o buoc KIEM (truoc khi gui dich)
        raise RuntimeError('loi gia truoc khi gui')
    io.cho_san_sang = hong
    ma = _voi_ngat(lambda: G.chay_co_ngat(io, 'di_B', 1, CFG))
    assert ma == 5 and getattr(io, 'huy_luot_t', None) is None and io.gui_ds == []
    assert 'LOI_GUI_DUONG RuntimeError' in io.cau[-1] and io.cau[-1].endswith('; HUY_DICH khong_co_dich')
    # luat G: ma 5 khong trung ma co san cua chuyen.sh; cau noi doi ma la 'LOI', dong log khop DONG_CHUYEN
    sh = open(os.path.join(DAY, 'chuyen.sh')).read()
    assert 'exec python3 "$HOME/gui_duong.py"' in sh and 'exit 5' not in sh
    cn = open(os.path.join(DAY, 'cau_noi_xe.py')).read()
    assert 'MA_CHUYEN.get(rc, "LOI")' in cn and '|LOI|' in cn


# ---------- G3 (dong 9): ong O18 chan giua duong. Phan Nav2 (RPP dung vi va cham phia truoc, FollowPath ABORTED) dat san
# trong IO gia; o day chi kiem gui_duong: cho, tinh lai tren costmap luc do, doi tuyen, HET_DUONG ----------
ONG1 = (0.70, 0.88, 0.00, 0.18)            # (x0, x1, y0, y1) ong O18 giua hanh lang duoi (= mo_phong_he_thong.NGUOI)
ONG2 = (0.70, 0.88, 0.66, 0.84)            # ong thu hai giua hanh lang tren
XE_DUNG = (0.45, 0.09, 0.0)                # RPP dung truoc ong (mui x = 0,595, cach ong 10,5 cm)


def cach_ong(x, y, r):
    return math.hypot(max(r[0] - x, 0.0, x - r[1]), max(r[2] - y, 0.0, y - r[3]))


def ban_co_ong(ong, le=0.08, ban=BAN):
    """Costmap them ong: o cach mep ong <= le (nua be ngang xe = vung inscribed cua Nav2, chi phi >= 98) thanh do."""
    b = copy.copy(ban)
    b.v = dict(ban.v)
    for k in b.v:
        if any(cach_ong(*b.tam(k), r) <= le for r in ong):
            b.v[k] = 0
    b._khoang_cach()
    return b


class _IOOng(IOGia):
    """Costmap theo thoi gian: co_ong(t) True thi costmap co ong (Nav2 da thay ong tren /global_costmap)."""

    def __init__(self, kq, ban_ong, co_ong, **kw):
        IOGia.__init__(self, kq, **kw)
        self.ban_ong, self.co_ong = ban_ong, co_ong

    def cho_san_sang(self):
        return self.p, self.ban()

    def ban(self):
        return self.ban_ong if self.co_ong(self.t) else self.b


def khong_qua_ong(d, ban, ong):
    """Moi diem duong ngoai o do cua costmap luc gui va tam xe cach mep ong >= nua be ngang xe (0,08 m)."""
    return all(not ban.do(ban.o(x, y)) and all(cach_ong(x, y, r) >= 0.08 for r in ong) for x, y, _ in d)


def test_23_G3_ong_chan_cho_vat_di_thi_di_tiep_den_noi():
    """Chuoi (1): ong xuat hien luc t = 3 s; RPP dung (va cham phia truoc), FollowPath ABORTED luc t = 8 s o XE_DUNG;
    nguoi lay ong di luc t = 30 s. gui_duong dung yen >= CHO_HONG, tinh lai tu cho xe tren costmap luc do => tuyen 1 => DEN_NOI."""
    b1 = ban_co_ong([ONG1])
    io = _IOOng([('ABORTED', 8, XE_DUNG), ('SUCCEEDED', 20, Bp)], b1, lambda t: 3 <= t < 30)
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI'] and [r[1] for r in io.rows[1:3]] == ['1', '1']
    (t1, _), (t2, d2) = io.gui_ds
    assert t2 - (t1 + 8) >= CFG['cho_hong'] and t2 >= 30               # khong gui gi trong luc ong con do
    assert math.hypot(d2[0][0] - XE_DUNG[0], d2[0][1] - XE_DUNG[1]) < 0.01
    assert all(y < 0.5 for x, y, _ in d2 if 0.3 <= x <= 1.2)          # van hanh lang duoi (tuyen 1)
    assert io.da_huy == 0


def test_24_G3_ong_khong_di_het_luot_het_duong_khong_dam_khong_lui():
    """Chuoi (2): ong o ca hai hanh lang, khong ai lay. Moi lan RPP dung, ABORTED. Tuyen 1: 3 luot, cach nhau >= CHO_HONG,
    moi duong tinh tren costmap co ong va khong qua ong; tuyen 2: khong co duong => HET_DUONG (ma 1), xe dung yen.
    Chi mot ong (hanh lang duoi): het 3 luot tuyen 1 thi doi sang tuyen 2 (hanh lang tren) => DEN_NOI."""
    b2 = ban_co_ong([ONG1, ONG2])
    io = _IOOng([('ABORTED', 8, XE_DUNG)] + [('ABORTED', 1, XE_DUNG)] * 5, b2, lambda t: t >= 3)
    assert G.chay(io, 'di_B', 1, CFG) == 1
    # 09/11 19:05: CAM[2] la TON (khong loai): tuyen 2 tinh duoc duong vong qua vung ton, khong qua ong => 3 luot moi tuyen roi HET_DUONG
    assert CUA(io.rows) == ['KIEM_DAT'] + ['KHONG_DAT'] * 6 + ['HET_DUONG'] and [r[1] for r in io.rows[1:7]] == ['1', '1', '1', '2', '2', '2']
    assert len(io.gui_ds) == 6
    t = [g[0] for g in io.gui_ds]
    assert t[1] - (t[0] + 8) >= CFG['cho_hong'] and t[2] - (t[1] + 1) >= CFG['cho_hong']
    assert all(khong_qua_ong(d, b2, [ONG1, ONG2]) for _, d in io.gui_ds[1:])
    assert re.search(r'^\s*allow_reversing:\s*false', open(os.path.join(DAY, 'nav2_params_cho.yaml')).read(), re.M)
    assert not re.search(r'<(BackUp|Spin)\b', open(os.path.join(DAY, 'bt_cho.xml')).read())   # khong lui, khong quay
    b1 = ban_co_ong([ONG1])
    io = _IOOng([('ABORTED', 8, XE_DUNG)] + [('ABORTED', 1, XE_DUNG)] * 2 + [('SUCCEEDED', 30, Bp)], b1, lambda t: t >= 3)
    assert G.chay(io, 'di_B', 1, CFG) == 0
    assert [r[1] for r in io.rows[1:5]] == ['1', '1', '1', '2'] and CUA(io.rows)[-1] == 'DEN_NOI'
    assert all(khong_qua_ong(d, b1, [ONG1]) for _, d in io.gui_ds[1:])
    assert any(y > 0.5 and 0.3 <= x <= 1.2 for x, y, _ in io.gui_ds[3][1])   # tuyen 2 di khe tren (nhu test_03)


# ---------- 04/11 lop 1: OccupancyGrid >= 72 cam khi tim duong; luat dinh re va SAI_VI_TRI do toi o CO VAT (>= 98) ----------
def luoi_nav2(w, h, vat, res=0.05):
    """OccupancyGrid (hang 0 o duoi) tu tap o vat theo InflationLayer + bang chuyen cua Nav2 Humble 1.1.20
    (inflation 0,20, scaling 3,0, inscribed 0,08 nhu nav2_params_cho.yaml)."""
    data = []
    for j in range(h):
        for i in range(w):
            dc = min((math.hypot(i - a, j - b) for a, b in vat), default=1e9)
            if dc == 0:
                c = 254
            elif dc * res <= 0.08:
                c = 253
            elif dc * res <= 0.20 + 1e-9:
                c = int(252 * math.exp(-3.0 * (dc * res - 0.08)))
            else:
                c = 0
            data.append({254: 100, 253: 99}.get(c, 0 if c == 0 else 1 + (97 * (c - 1)) // 251))
    return data


def test_25_lop1_nguong_72_va_luat_dinh_re_do_toi_o_vat():
    w, h = 40, 24                                                     # 2,0 x 1,2 m, tuong o hang 0 va 23
    vat = [(i, 0) for i in range(w)] + [(i, h - 1) for i in range(w)]
    data = luoi_nav2(w, h, vat)
    b = T.Ban.tu_luoi(w, h, 0.05, 0.0, 0.0, data)
    o = lambda j: data[j * w + 20]
    assert (o(1), o(2), o(3), o(4)) == (99, 92, 79, 68)               # tam cach vat 0,05 / 0,10 / 0,15 / 0,20 m
    assert b.vat((20, 1)) and not b.vat((20, 2)) and b.do((20, 2)) and not b.vat((20, 3)) and b.do((20, 3)) and not b.do((20, 4))
    assert T.NGUONG_CAM == 72 and T.NGUONG_VAT == 98
    assert abs(T.cach_do(b, *b.tam((20, 6))) - 0.25) < 1e-9          # do toi o CO VAT >= 98 (hang 1), khong toi dai cam (hang 3)
    d = T.tinh_duong(b, b.tam((2, 3)), b.tam((37, 8)), 1)             # xe dung trong dai cam (0,15 m tu tuong): van ra duoc
    assert all(not b.vat(b.o(x, y)) for x, y, _ in d)
    xa = [(x, y) for x, y, _ in d if math.hypot(x - b.tam((2, 3))[0], y - b.tam((2, 3))[1]) > T.R_THA_CAM]
    assert xa and all(not b.do(b.o(x, y)) for x, y in xa)             # ngoai R_THA_CAM: hanh lang rong => tranh dai ton (K_TON)
    pgm = copy.copy(b)                                                # cung luoi, khong co dai cam (nhu ban PGM)
    pgm.cam = set()
    pgm._khoang_cach()
    khoi = vat + [(i, j) for i in range(8, 21) for j in range(7, 10)]  # khe hang 1..6 duoi khoi: tam giua cach vat 0,15 m
    b2 = T.Ban.tu_luoi(w, h, 0.05, 0.0, 0.0, luoi_nav2(w, h, khoi))
    p2 = copy.copy(b2)
    p2.cam = set()
    p2._khoang_cach()
    dau, cuoi = (0.15, 0.45), (1.80, 0.45)
    assert T.tinh_duong(p2, dau, cuoi, 1)                              # khong dai ton: lach qua khe
    d2 = T.tinh_duong(b2, dau, cuoi, 1)                                # 09/11 18:47: o 72-97 la TON, khong chan => van lach qua khe
    assert d2 and all(not b2.vat(b2.o(x, y)) for x, y, _ in d2)        # (19:05: co the vong qua vung CAM[1] ton)
    kin = khoi + [(i, j) for i in range(8, 21) for j in range(10, 23)]  # lap phia tren: chi con khe duoi (dai 72-97) => di qua dai ton
    b4 = T.Ban.tu_luoi(w, h, 0.05, 0.0, 0.0, luoi_nav2(w, h, kin))
    d4 = T.tinh_duong(b4, dau, cuoi, 1)
    assert all(not b4.vat(b4.o(x, y)) for x, y, _ in d4) and any(b4.ton(b4.o(x, y)) for x, y, _ in d4)
    vat2 = khoi + [(i, j) for i in range(8, 21) for j in range(1, 7)]   # khe bi vat (>= 98) lap kin => vat chan that, phai vong
    b3 = T.Ban.tu_luoi(w, h, 0.05, 0.0, 0.0, luoi_nav2(w, h, vat2))
    assert all(not b3.vat(b3.o(x, y)) for x, y, _ in T.tinh_duong(b3, dau, cuoi, 1))   # 19:05: CAM[1] la ton => tuyen 1 cung vong, khong qua vat
    d3 = T.tinh_duong(b3, dau, cuoi, 2)                                # tuyen 2 vong phia tren
    assert all(y > 0.45 for x, y, _ in d3 if 0.45 <= x <= 1.0)         # di vong phia tren khoi


def test_26_lop1_xe_trong_dai_cam_khong_phai_SAI_VI_TRI():
    w, h = 40, 24
    vat = [(i, 0) for i in range(w)] + [(i, h - 1) for i in range(w)]
    b = T.Ban.tu_luoi(w, h, 0.05, 0.0, 0.0, luoi_nav2(w, h, vat))
    xe = (b.tam((2, 3))[0], b.tam((2, 3))[1], 0.0)                     # o 79: cam khi tim duong, khong co vat
    io = IOGia([('SUCCEEDED', 20, (1.60, 0.82, 0.0))], pose=xe, ban=b)
    assert G.chay(io, 'di_B', 1, CFG) == 0 and CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI']
    io = IOGia([], pose=(b.tam((2, 1))[0], b.tam((2, 1))[1], 0.0), ban=b)   # o 99 (co vat): nhu cu
    assert G.chay(io, 'di_B', 1, CFG) == 3 and CUA(io.rows) == ['SAI_VI_TRI']



# ---------- 04/11 lop 2: bat tay SAP_GUI / RA_LUI / RA_XONG truoc moi lan gui ----------
def test_27_lop2_ra_lui_cho_ra_xong_roi_moi_gui():
    def lop2(t, ma, t0):                                              # sep tra RA_LUI sau 0,3 s; phu lai RA_XONG luc t0 + 4 s
        tra = dict(chu='RA_LUI', ma_luot=ma, ma=900 + ma) if t - t0 >= 0.3 else None
        return (tra if ma == 1 else dict(chu='KHONG_CAN', ma_luot=ma, ly='vung_trong')), (1 if ma == 1 and t - t0 >= 4.0 else 0)
    io = IOGia([('SUCCEEDED', 20, Bp)], lop2=lop2)
    assert G.chay(io, 'di_B', 1, CFG) == 0 and CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI']
    assert io.sap_gui[0][1:] == ('di_B', 1, 1) and 4.0 <= io.gui_ds[0][0] < 4.2   # gui sau RA_XONG, khong truoc
    assert any('sep bao RA_LUI' in c for c in io.cau) and any(c.startswith('   lop 2: RA_XONG') for c in io.cau)
    io = IOGia([('SUCCEEDED', 20, Bp)])                                # sep tra KHONG_CAN ngay: khong ton them thoi gian
    assert G.chay(io, 'di_B', 1, CFG) == 0 and io.gui_ds[0][0] == 0.0 and not any('lop 2' in c or 'RA_HET_GIO' in c for c in io.cau)


def test_28_lop2_het_gio_thi_gui_nhu_cu():
    io = IOGia([('SUCCEEDED', 20, Bp)], lop2=lambda t, ma, t0: (None, 0))                       # sep khong tra loi
    assert G.chay(io, 'di_B', 1, CFG) == 0 and abs(io.gui_ds[0][0] - CFG['lop2_tra']) < 0.15
    assert any(c.startswith('   RA_HET_GIO: sep khong tra loi sau 2 s') for c in io.cau)
    io = IOGia([('SUCCEEDED', 20, Bp)], lop2=lambda t, ma, t0: (dict(chu='RA_LUI', ma_luot=ma, ma=7), 0))   # khong RA_XONG
    assert G.chay(io, 'di_B', 1, CFG) == 0 and abs(io.gui_ds[0][0] - CFG['lop2_cho']) < 0.15
    assert any(c.startswith('   RA_HET_GIO: khong thay RA_XONG sau 12 s') for c in io.cau) and CUA(io.rows)[-1] == 'DEN_NOI'
    io = IOGia([('SUCCEEDED', 20, Bp)], lop2=lambda t, ma, t0: (dict(chu='RA_LUI', ma_luot=ma, ma=7), 0),
               lenh=lambda t: ('DUNG', 'HET_3_LUOT_THOAT') if t >= 5 else None)
    assert G.chay(io, 'di_B', 1, CFG) == 4 and not io.gui_ds                                     # DUNG giua luc cho


def test_29_lop2_luat_G_sap_gui_ra_lui_khong_can():
    import sep as S
    import lai_xe_lidar as L
    d = G.dong_sap_gui(1234.5, 've_A', 2, 3, 12345)
    assert S.doc_sap_gui(d, 1235.0) == dict(huong='ve_A', tuyen=2, lan=3, ma=12345) and S.doc_sap_gui(d, 1240.0) is None
    for ra2 in (dict(chu='RA_LUI', ma_luot=12345, ma=777, ly='x'), dict(chu='KHONG_CAN', ma_luot=12345, ma=778, ly='vung_trong')):
        chu, ly = S.dong_ra2(ra2)
        dong = '%.2f %s %s' % (100.0, chu, ly)                       # dung khuon sep.ghi_lenh
        tra = L.doc_sep_ra2(dong, 100.5)
        assert tra['chu'] == ra2['chu'] and tra['ma_luot'] == 12345 and L.doc_sep_thoat(dong, 100.5) is None
    assert L.doc_sep_ra2('100.00 RA_LUI 12345 777', 100.5)['ma'] == 777
    assert 'RA_LUI' in G.PHU_LAI_BAN                                   # phu lai dang lui lop 2 = dang lai banh (cho_truoc_khi_gui)
    h = G.dong_huy(1236.0)                                             # 05/11: bi ngat => HUY: phu lai dung RA2, sep khong coi la SAP_GUI
    assert L.luot_huy(h) and L.luot_huy(None) and not L.luot_huy(d) and S.doc_sap_gui(h, 1236.0) is None


def test_30_lop2_ra_tien_cho_ra_xong_roi_moi_gui():
    """06/11 viec 120: sep tra RA_TIEN (vat sau than) => gui_duong cho RA_XONG nhu RA_LUI, khong coi la 'sep khong tra loi'."""
    import sep as S
    import lai_xe_lidar as L
    def lop2(t, ma, t0):                                              # sep tra RA_TIEN sau 0,2 s; RA_XONG luc t0 + 1,5 s
        return (dict(chu='RA_TIEN', ma_luot=ma, ma=800 + ma) if t - t0 >= 0.2 else None), (1 if t - t0 >= 1.5 else 0)
    io = IOGia([('SUCCEEDED', 20, Bp)], lop2=lop2)
    assert G.chay(io, 'di_B', 1, CFG) == 0 and 1.5 <= io.gui_ds[0][0] < 1.7
    assert any(c == '   lop 2: sep bao RA_TIEN: cho phu lai tien ra, toi da 12 s.' for c in io.cau)
    assert not any('RA_HET_GIO' in c for c in io.cau)
    chu, ly = S.dong_ra2(dict(chu='RA_TIEN', ma_luot=12345, ma=777, ly='vat_sau_than_0.06m'))   # luat G: sep ghi -> phu lai / gui_duong doc
    tra = L.doc_sep_ra2('%.2f %s %s' % (100.0, chu, ly), 100.5)
    assert (chu, ly) == ('RA_TIEN', '12345 777') and tra == dict(chu='RA_TIEN', ma_luot=12345, ma=777)
    assert 'RA_TIEN' in G.PHU_LAI_BAN and 'RA_TIEN' in S.DANG_LAM                # ten trang thai phu lai luc tien (cu, khong doi)

# ---------------- 08/11 bo 1e: kieu mat (chay_mat, DongHo, don_sau_chuyen) ----------------
class IOMat:
    """IO gia kieu mat. kich: danh sach kich ban cho tung lan gui navigate_to_pose: dict(
      ket=(ket_qua, t_ket) | None (khong tu ket thuc), fb=lambda t: (distance_remaining, recoveries) | None, pose=lambda t: (x, y, yaw))."""

    def __init__(self, kich, pose=(0.0, 0.0, 0.0), lenh=None, ban=BAN, kiem_pose=True):
        self.t, self.p0, self.kich, self.b = 0.0, pose, list(kich), ban
        self.lenh = lenh or (lambda t: None)
        self.kiem_pose = kiem_pose
        self.cau, self.rows, self.chon, self.kn, self.chup_ds, self.log_ds = [], [], [], [], [], []
        self.dang, self.t_gui, self.da_huy = None, None, 0
        self.dang_cu, self.vet, self.lui_dang = None, [], False

    def now(self):
        return self.t

    def ngu(self, s):
        self.vet.append((round(self.t, 2), self.dang is not None, self.lui_dang))   # 09/11 #2: co dich / dang lui luc cho
        p = self._pose_luc()
        self.vet_xy = getattr(self, 'vet_xy', []) + [(p[0], p[1])]                # 09/11 gia X: vet that (moi ngu)
        self.t += s

    def tuoi_plan(self):                                  # 09/11 #5: kich ban 'plan' = lambda dt: tuoi /plan (s) | None
        k = self.dang or self.dang_cu
        return None if k is None else k.get('plan', lambda dt: None)(self.t - self.t_gui)

    def su_kien_lui(self):                                # 09/11 #6: kich ban 'lui_bt' = [(dt, 'RA'|'XONG', chi_tiet)]
        if self.dang is None:
            return []
        con = self.dang.get('lui_bt', [])
        ra = [(l, c) for dt, l, c in con if self.t - self.t_gui >= dt]
        self.dang['lui_bt'] = [e for e in con if self.t - self.t_gui < e[0]]
        return ra

    def bao(self, c):
        self.cau.append(c)

    def ghi(self, *a):
        self.rows.append([str(x) for x in a])

    def _pose_luc(self):
        if self.dang is not None:
            return self.dang['pose'](self.t - self.t_gui)
        return self.p0

    def cho_san_sang(self):
        return (self._pose_luc() if self.kiem_pose else None), self.b

    def pose(self):
        return self._pose_luc()

    def lenh_sep(self):
        return self.lenh(self.t)

    def chon_tuyen(self, s):
        self.chon.append((round(self.t, 1), s))

    def gui_nav(self, dich, yaw):
        k = self.kich.pop(0)
        if k is None:
            return False
        self.dang, self.t_gui = dict(k), self.t
        self.dang.setdefault('pose', lambda dt: self.p0)
        return True

    def feedback(self):
        return None if self.dang is None else self.dang['fb'](self.t - self.t_gui)

    def ket_qua(self):
        if self.dang and self.dang.get('ket') and self.t - self.t_gui >= self.dang['ket'][1]:
            kq, p = self.dang['ket'][0], self.dang['pose'](self.t - self.t_gui)
            self.dang, self.p0 = None, p
            return kq
        return None

    def huy(self):
        if self.dang is not None:
            self.p0 = self.dang['pose'](self.t - self.t_gui)
        self.da_huy += 1
        self.dang, self.dang_cu = None, self.dang

    def co_dich(self):
        return self.dang is not None

    def huy_luot(self):
        pass

    def huy_khi_ngat(self):
        if self.lui_dang:                                 # 09/11 #7: bi ngat luc dang lui => huy ca /backup
            self.lui_dang, self.huy_lui = False, getattr(self, 'huy_lui', 0) + 1
            if self.dang is None:
                return 'ok'
        if self.dang is None:
            return 'khong_co_dich'
        self.huy()
        return 'ok'

    def ghi_kinh_nghiem(self, dong):                     # 09/11 nao xe viec 4: dict theo COT_KINH_NGHIEM (khong gio)
        self.kn_moi = getattr(self, 'kn_moi', []) + [dict(dong, t=round(self.t, 1))]
        if dong['su_co'] == 'DOI_KHE':                   # kn: chi dong bo khe (nhu truoc), (x, y, huong, ly_do)
            self.kn.append((round(dong['x'], 2), round(dong['y'], 2), dong['huong'], dong['ly_do']))
            self.thu_tu = getattr(self, 'thu_tu', []) + [('kn', self.t)]

    def ma_chuyen(self):
        return 'thu'

    def doc_kinh_nghiem(self):                           # 09/11 gia X: kn_doc (tep co san, dict nhu hanh_vi.doc_kinh_nghiem) + dong moi
        cu = list(getattr(self, 'kn_doc', []))
        moi = [dict({k: ('' if v is None else ('%.3f' % v if k in ('x', 'y') else str(v))) for k, v in d.items()}, so_dong=len(cu) + i + 1)
               for i, d in enumerate(getattr(self, 'kn_moi', []))]
        return cu + moi

    def vet_chuyen(self):
        return list(getattr(self, 'vet_xy', []))

    def gac_quet(self):                                   # 09/11 dung im: self.gac_fn(t) -> None (khong quet moi) | 'MU' | [diem khung xe]
        f = getattr(self, 'gac_fn', None)
        v = [] if f is None else f(self.t)                  # khong khai bao vat: moi lan quet deu sach
        if v is None:
            return 'CHUA', None
        if v == 'MU':
            return 'MU', 'gia'
        x, y, th = self._pose_luc()
        return 'OK', (self.t, v, [(x + a * math.cos(th) - b * math.sin(th), y + a * math.sin(th) + b * math.cos(th)) for a, b in v])

    def gac_ban(self):                                    # ban do trong 6 x 4 m (goc (-1,5; -1,5)), o 0,05
        import gac_vat_la as GV
        return GV.BanDo(120, 80, 0.05, -1.5, -1.5, [0] * 9600)

    def lui(self, m, v, dung_khi=None):                   # 08/11 buoc 2c: Nav2 BackUp gia (mat m / v giay)
        self.lui_ds = getattr(self, 'lui_ds', []) + [(round(self.t, 1), m, v)]
        if dung_khi is not None and getattr(self, 'kq_lui', 'SUCCEEDED') == 'SUCCEEDED':   # 09/11: gac hong luc lui
            self.lui_dang = True
            for k in range(int(round(m / v / 0.1))):
                self.t += 0.1
                if dung_khi():
                    self.lui_dang, self.huy_lui = False, getattr(self, 'huy_lui', 0) + 1
                    return dung_khi(), round(v * 0.1 * (k + 1), 3)
            self.lui_dang = False
            self.thu_tu = getattr(self, 'thu_tu', []) + [('lui', self.t - m / v)]
            if self.dang is None:                             # xe lui that (huong hien tai) m met
                x, y, th = self.p0
                self.p0 = (x - m * math.cos(th), y - m * math.sin(th), th)
            return 'SUCCEEDED', m
        self.thu_tu = getattr(self, 'thu_tu', []) + [('lui', self.t)]
        kq = getattr(self, 'kq_lui', 'SUCCEEDED')
        if kq == 'DUNG_GIUA':                             # sep bao DUNG trong luc lui (node: huy /backup, tra 'DUNG')
            self.t += 1.0
            return 'DUNG', None
        if kq == 'NGAT_GIUA':                             # 09/11 #7: SIGINT trong luc lui (node: KeyboardInterrupt tu ngu)
            self.lui_dang = True
            self.t += 1.0
            raise KeyboardInterrupt
        self.t += m / v
        return kq, getattr(self, 'quang_lui', m if kq == 'SUCCEEDED' else 0.05)

    def chup(self, huong, ly):
        self.chup_ds.append((huong, ly))
        return '/tmp/chup_gia.txt'

    def noi_log(self, huong, ma):
        self.log_ds.append((huong, ma, list(self.cau)))


TOI_B = lambda dt: (1.60, 0.82, 0.0)
DUNG_A = lambda dt: (0.0, 0.0, 0.0)


def test_34_mat_doc_goc_hai_kieu():
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'goc_dang_chay')
    assert G.doc_goc(p) == 'cu'                                    # khong co tep => kieu cu
    for nd, mong in (('mat\n', 'mat'), ('cu\n', 'cu'), ('mat_lai\n', 'cu'), ('', 'cu'), ('  mat  \n', 'mat')):
        with open(p, 'w') as f:
            f.write(nd)
        assert G.doc_goc(p) == mong, (nd, G.doc_goc(p))
    assert G.chay_co_ngat.__defaults__[-1] is None                 # mac dinh chay_fn = None => chay() kieu cu y het


def test_35_mat_toi_noi_lan_dau_va_chon_tuyen():
    io = IOMat([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0))])
    ma = G.chay_mat(io, 'di_B', 1, CFG)
    assert ma == 0, io.cau
    assert [s for _, s in io.chon] == ['0', '1 0.000 0.000'], io.chon        # "0" o DAU chuyen, roi tuyen 1 tai vi tri xe
    assert io.chon[1][0] - io.chon[0][0] >= CFG['cho_chon']                  # KIEM sau khi cho 2 s
    assert io.cau[0].startswith('-- tuyen 1, lan 1/1: navigate_to_pose tu (0.00; 0.00) toi (1.60; 0.82)'), io.cau
    assert CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'] and io.cau[-1].startswith('DEN_NOI: di_B xong theo tuyen 1')
    assert not any(c.startswith('CHO_VAT') for c in io.cau) and io.kn == [] and io.chup_ds == []


def test_36_mat_dong_ho_30s_bo_khe_roi_tuyen_kia():
    ket_khe = (0.70, 0.80, 0.0)
    lenh = lambda t: ('CHO', 'ABORT_NHANH 1s truoc') if t < 200 else None       # sep con bao CHO vi khe vua bi chan
    io = IOMat([dict(ket=None, fb=lambda dt: (1.20, 0), pose=lambda dt: ket_khe, plan=lambda dt: 0.5),
                dict(ket=('SUCCEEDED', 25.0), fb=lambda dt: (max(0.05, 1.5 - 0.06 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 25 else ket_khe)],
               lenh=lambda t: lenh(t) if t > 3 else None)
    ma = G.chay_mat(io, 'di_B', 1, CFG)
    assert ma == 0, io.cau
    cho = [c for c in io.cau if c.startswith('CHO_VAT:')]
    assert [c.split()[1] for c in cho] == ['5/30', '10/30', '15/30', '20/30', '25/30'], cho
    assert any(c.startswith('   khong den duoc (CHO_VAT_30s, cach dich ') for c in io.cau), io.cau
    assert io.kn == [(0.7, 0.8, 'di_B', 'KET_CO_DUONG')] and io.chup_ds == [('di_B', 'bo khe tuyen 1 (CHO_VAT_30s, KET_CO_DUONG)')]
    assert '   sep bao CHO (ABORT_NHANH 1s truoc): bo qua (khe vua bo).' in io.cau                 # gui lai: bo qua ABORT_NHANH
    assert [s.split()[0] for _, s in io.chon] == ['0', '1', '2'] and io.da_huy == 1
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI']
    assert '== Tuyen 1 hong. ==' in io.cau


def test_37_mat_dong_ho_tien_dat_lai_va_DI_TIEP():
    # giam nhe 0,05 < TIEN_MIN khong tinh; dung 16 s (moc 1,80 -> can <= 1,70 o dt 16: in CHO_VAT 5, 10, 15) roi tien lai: DI_TIEP
    def fb(dt):
        if dt < 3:
            return (1.80, 0)
        if dt < 15:
            return (1.75, 0)                                        # giam 0,05 < TIEN_MIN: van dung
        return (max(0.05, 1.75 - 0.05 * (dt - 15)), 0)
    io = IOMat([dict(ket=('SUCCEEDED', 60.0), fb=fb, pose=lambda dt: TOI_B(dt) if dt >= 60 else (0.0, 0.0, 0.0))])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0, io.cau
    cho = [c.split()[1] for c in io.cau if c.startswith('CHO_VAT:')]
    assert cho[:3] == ['5/30', '10/30', '15/30'] and '20/30' not in cho[:4], cho
    i_tiep = io.cau.index('   DI_TIEP: xe tien lai.')
    assert io.cau[i_tiep - 1].startswith('CHO_VAT: 15/30') and io.kn == [] and io.da_huy == 0


def test_38_mat_dong_ho_bo_so_0_nhay_dai_va_tf():
    T = 100.0
    dh = G.DongHo(0.0, 30.0, 0.10)
    assert dh.cap_nhat(1.0, None, None)[0] == 'CHUA'
    assert dh.cap_nhat(2.0, 1.0, 0.0) == ('DI', None, 0.0)           # feedback dau (0 = chua co duong): moc bat dau, bo so 0
    assert dh.cap_nhat(10.0, 1.0, 1.5)[0] == 'DI'                    # fb dau tien khac 0 lam moc
    assert dh.cap_nhat(12.0, 1.0, 2.5)[0] == 'DI'                    # tinh lai duong dai hon: khong lui moc
    assert dh.cap_nhat(14.0, 1.0, 1.45)[0] == 'DI'                   # giam 0,05 so voi moc 1,5: chua tien
    assert dh.cap_nhat(16.0, 0.89, 1.45)[0] == 'TIEN'                # tf giam 0,11: tien (mot trong hai la du)
    st = dh.cap_nhat(16.0 + 29.9, 0.89, 1.45)
    assert st[0] == 'DI' and abs(st[2] - 29.9) < 1e-9
    assert dh.cap_nhat(46.0, 0.89, 1.45) == ('BO_KHE', 'CHO_VAT_30s', 30.0)
    dh2 = G.DongHo(T, 30.0, 0.10)                                     # khong co feedback 30 s tu luc gui => bo khe
    assert dh2.cap_nhat(T + 29.0, 1.0, None)[0] == 'CHUA'
    assert dh2.cap_nhat(T + 30.0, 1.0, None) == ('BO_KHE', 'KHONG_FEEDBACK', 30.0)


def test_39_mat_khong_feedback_va_recoveries():
    io = IOMat([dict(ket=None, fb=lambda dt: None),
                dict(ket=('ABORTED', 40.0), fb=lambda dt: (1.4 - 0.2 * min(dt, 5), 1 if dt > 10 else 0) if dt < 12 else (0.4, 3))])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1, io.cau
    assert any(c.startswith('   khong den duoc (KHONG_FEEDBACK') for c in io.cau)
    assert io.kn[0][3] == 'VAT_CHAN'                                 # 09/11 #5: khong co /plan => planner khong ra duong
    rec = [c for c in io.cau if 'number_of_recoveries' in c]
    assert rec == ['   Nav2 hoi phuc: number_of_recoveries = 1.', '   Nav2 hoi phuc: number_of_recoveries = 3.'], rec
    assert io.cau[-1].startswith('HET_DUONG') and CUA(io.rows)[-1] == 'HET_DUONG'


def test_40_mat_don_sau_chuyen_moi_loi_ra_24_o():
    """6 loi ra (0, 1, 3, 4, 5, 130) + 09/11 #7 "ngat luc dang lui" (130) x 5 viec don (mask 0, chup, log noi, CSV,
    huy dich/lui dang chay) = 35 o."""
    ok_ket = dict(ket=('SUCCEEDED', 5.0), fb=lambda dt: (1.0 - 0.15 * dt, 0), pose=lambda dt: TOI_B(dt) if dt >= 5 else (0.0, 0.0, 0.0))
    hong = lambda: dict(ket=('ABORTED', 5.0), fb=lambda dt: (1.0, 0))

    class Ngat(IOMat):
        def __init__(self, loi, *a, **k):
            super().__init__(*a, **k)
            self.loi = loi

        def ngu(self, s):
            super().ngu(s)
            if self.dang is not None and self.t - self.t_gui > 3:
                raise self.loi
    ca = {0: IOMat([ok_ket]), 1: IOMat([hong(), hong()]), 3: IOMat([], kiem_pose=False),
          4: IOMat([dict(ket=None, fb=lambda dt: (1.0, 0))], lenh=lambda t: ('DUNG', 'x') if t > 8 else None),
          5: Ngat(RuntimeError('loi gia'), [dict(ket=None, fb=lambda dt: (1.0, 0))]),
          130: Ngat(KeyboardInterrupt(), [dict(ket=None, fb=lambda dt: (1.0, 0))]),
          '130_lui': IOMat([hong()])}
    ca['130_lui'].kq_lui = 'NGAT_GIUA'                               # tuyen 1 hong => /backup cua gui => SIGINT giua luc lui
    csv_cuoi = {0: 'DEN_NOI', 1: 'HET_DUONG', 3: 'SAI_VI_TRI', 4: 'DUNG_SEP', 5: 'LOI_GUI_DUONG', 130: 'NGAT', '130_lui': 'NGAT'}
    bang = []
    for nhan, io in ca.items():
        mong = 130 if nhan == '130_lui' else nhan
        ma = G.chay_co_ngat(io, 'di_B', 1, CFG, da_ngat=lambda: False, chay_fn=G.chay_mat)
        assert ma == mong, (mong, ma, io.cau)
        don = dict(G.don_sau_chuyen(io, ma, 'di_B'))
        mask0 = io.chon[-1][1] == '0' and don['mask0'] == 'ok'
        chup = (don['chup'] == 'ok' and io.chup_ds[-1] == ('di_B', 'ket thuc ma %d' % ma)) if ma else (don['chup'] == 'khong can')
        log = don['log'] == 'ok' and io.log_ds[-1][:2] == ('di_B', ma)
        csv = CUA(io.rows)[-1] == csv_cuoi[nhan]
        # viec 5: luc ket thuc khong con dich navigate_to_pose nao, khong con /backup cua gui nao dang chay
        con_chay = io.dang is not None or io.lui_dang
        huy = not con_chay and (io.da_huy >= 1 if nhan in (5, 130) else True) and (getattr(io, 'huy_lui', 0) == 1) == (nhan == '130_lui')
        bang.append((nhan, mask0, chup, log, csv, huy))
    assert all(all(o[1:]) for o in bang), bang
    assert len(bang) * 5 == 35
    io = ca['130_lui']
    i_lui = [k for k, c in enumerate(io.cau) if c.startswith('LUI_RA: gui 0.35')]
    assert io.cau[-1] == 'HUY_DICH_KHI_NGAT ok' and i_lui and not any(c.startswith('-- tuyen 2') for c in io.cau[i_lui[0]:]), io.cau


def test_41_mat_don_loi_mot_viec_khong_chan_viec_sau():
    io = IOMat([])

    def hong(s):
        raise RuntimeError('context da tat')
    io.chon_tuyen = hong
    don = dict(G.don_sau_chuyen(io, 130, 've_A'))
    assert don['mask0'].startswith('LOI RuntimeError') and don['chup'] == 'ok' and don['log'] == 'ok', don


def test_42_mat_kinh_nghiem_ghi_doc_khop_lop_huong_dan():
    import tempfile
    import lop_huong_dan as LH
    p = os.path.join(tempfile.mkdtemp(), 'kinh_nghiem.csv')
    with open(p, 'w') as f:                              # 09/11 nao xe viec 4: tep cu (truoc 09/11) giu nguyen, doc duoc
        f.write('gio,x,y,huong,tuyen,ly_do\n2026-11-08 19:00:00,0.69,0.80,di_B,1,KET_CO_DUONG\n')
    for k in range(3):
        G.ghi_kinh_nghiem(p, dict(gio='2026-11-08 20:0%d:00' % k, chuyen='c1', x=0.70 + 0.01 * k, y=0.80, huong='di_B', su_co='DOI_KHE',
                                  cach='cho=30', ket_qua='HONG', ly_do=G.KET_CO_DUONG))
    G.ghi_kinh_nghiem(p, dict(gio='2026-11-08 20:09:00', chuyen='c1', x=0.30, y=0.10, huong='di_B', su_co='DOI_KHE', cach='cho=30',
                              ket_qua='HONG', ly_do=G.VAT_CHAN))                   # 09/11 #5: lop_huong_dan khong dem
    G.ghi_kinh_nghiem(p, dict(gio='g', chuyen='c1', x=None, y=None, huong='di_B', su_co='BT_LUI', cach='lui=0.25', ket_qua='DUOC', ly_do='-'))
    with open(p) as f:
        dong = f.read().splitlines()
    assert G.COT_KINH_NGHIEM == 'gio,chuyen,x,y,huong,su_co,cach,ket_qua,ly_do' and dong[0] == 'gio,x,y,huong,tuyen,ly_do'
    assert dong[2] == G.COT_KINH_NGHIEM and len(dong) == 8 and dong.count(G.COT_KINH_NGHIEM) == 1          # dong dau moi chi 1 lan
    assert dong[3] == '2026-11-08 20:00:00,c1,0.700,0.800,di_B,DOI_KHE,cho=30,HONG,KET_CO_DUONG' and dong[7] == 'g,c1,,,di_B,BT_LUI,lui=0.25,DUOC,-'
    with open(p, 'a') as f:
        f.write('hong,khong,phai,so\n,,,\n')
    assert LH.doc_kinh_nghiem(p) == [(0.69, 0.80), (0.70, 0.80), (0.71, 0.80), (0.72, 0.80)]           # dong cu + dong moi


def test_43_mat_noi_dung_chup():
    t = G.noi_dung_chup('ket thuc ma 1', '2026-11-08 20:00:00', (0.5, 0.6, 0.0), (1.2, 2), [(0.0, 0.0), (0.5, 0.6)],
                        (3, 2, 0.05, -0.1, -0.1, [0, 49, 50, 100, 0, -1]))
    d = t.splitlines()
    assert d[1] == 'pose 0.500 0.600 0.0' and d[2] == 'feedback distance_remaining 1.200 number_of_recoveries 2'
    assert d[3] == 'plan 2 diem' and d[6].endswith('2 o >= 50 (x y gia_tri)') and d[7:] == ['0.025 -0.075 50', '-0.075 -0.025 100']
    assert 'feedback chua co' in G.noi_dung_chup('x', 'g', None, None, [], None)


def test_44_mat_dong_log_khop_cau_noi_va_trang():
    import trang_v10 as V
    import re as _re
    src = open(os.path.join(DAY, 'cau_noi_xe.py')).read()
    dong_chuyen = _re.compile(_re.search(r'DONG_CHUYEN = re\.compile\(r"(.+?)"\)', src).group(1))
    for c in ('DEN_NOI: di_B xong theo tuyen 2 (cach dich 0.10 m).', 'HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.',
              'DUNG_SEP: sep bao dung. Xe dung yen, khong gui them lenh.', 'DUNG: SAI_VI_TRI (x). KHONG gui lenh nao.'):
        assert dong_chuyen.match(c), c                                 # chu ket thuc kieu mat = kieu cu (cau_noi_xe.DONG_CHUYEN)
    assert V.dong_chuyen_cuoi(['KIEU gui_duong: mat (~/goc_dang_chay)']) is None   # dong dau: trang van "경로 계산 중"
    assert V.dong_chuyen_cuoi(['-- tuyen 2, lan 1/1: navigate_to_pose tu (0.01; 0.00) toi (1.60; 0.82)']) == 'DI'
    assert V.lan_thu(['-- tuyen 2, lan 1/1: navigate_to_pose tu (0.01; 0.00) toi (1.60; 0.82)']) == (2, 1, 1)
    assert V.dong_chuyen_cuoi(['-- tuyen 1, lan 1/1: x', 'CHO_VAT: 5/30 s (cach dich 1.20 m)']) == 'CHO'
    assert V.dong_chuyen_cuoi(['CHO_VAT: 10/30 s', '   DI_TIEP: xe tien lai.']) == 'DI'
    assert V.dong_chuyen_cuoi(['   khong den duoc (CHO_VAT_30s, cach dich 0.95 m).']) == 'CHO'


def test_45_mat_tuyen_2_phai_lui_xa_dich_truoc():
    """08/11 buoc 2c: bo khe tuyen 1 => /backup 0,35 m o 0,10 m/s TRUOC /chon_tuyen tuyen 2; tuyen dau khong lui."""
    ket_khe = (0.62, 0.03, 0.0)
    io = IOMat([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe),
                dict(ket=('SUCCEEDED', 30.0), fb=lambda dt: (max(0.05, 2.9 - 0.1 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 30 else ket_khe)])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0, io.cau
    assert io.lui_ds and len(io.lui_ds) == 1 and io.lui_ds[0][1:] == (0.35, 0.10), getattr(io, 'lui_ds', None)
    t_lui = io.lui_ds[0][0]
    chon2 = [t for t, c in io.chon if c.startswith('2 ')]
    assert chon2 and chon2[0] >= t_lui + 0.35 / 0.10 - 1e-6, (chon2, t_lui)              # lui XONG roi moi chon tuyen 2
    assert [k for k, _ in io.thu_tu] == ['kn', 'lui'] and [c for c in io.chup_ds] == [('di_B', 'bo khe tuyen 1 (CHO_VAT_30s, VAT_CHAN)')]
    i_lui = [k for k, c in enumerate(io.cau) if c == 'LUI_RA: gui 0.35 (truoc tuyen 2, /backup 0.10 m/s)']      # 09/11 #6
    assert io.cau[i_lui[0] + 1] == '   LUI_XONG: gui => SUCCEEDED (xe lui 0.35 m).', io.cau
    i_t2 = [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 2, lan 1/1')]
    i_bo = [k for k, c in enumerate(io.cau) if c.startswith('   khong den duoc (CHO_VAT_30s')]
    assert i_bo and i_lui and i_t2 and i_bo[0] < i_lui[0] < i_t2[0], io.cau
    assert CFG['lui_m'] == 0.35 and CFG['lui_v'] == 0.10
    io1 = IOMat([dict(ket=('SUCCEEDED', 5.0), fb=lambda dt: (1.0 - 0.15 * dt, 0), pose=lambda dt: TOI_B(dt) if dt >= 5 else (0.0, 0.0, 0.0))])
    assert G.chay_mat(io1, 'di_B', 1, CFG) == 0 and not getattr(io1, 'lui_ds', [])                # tuyen dau: khong lui


def test_46_mat_lui_hong_van_sang_tuyen_kia_dung_giua_lui_la_DUNG_SEP():
    hong = lambda: dict(ket=('ABORTED', 5.0), fb=lambda dt: (1.0, 0))
    io = IOMat([hong(), dict(ket=('SUCCEEDED', 5.0), fb=lambda dt: (1.0 - 0.15 * dt, 0), pose=lambda dt: TOI_B(dt) if dt >= 5 else (0.0, 0.0, 0.0))])
    io.kq_lui = 'ABORTED'                                                                       # lui khong duoc: van thu tuyen kia
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0, io.cau
    assert '   LUI_XONG: gui => ABORTED (xe lui 0.05 m).' in io.cau, io.cau
    io = IOMat([hong(), hong()])
    io.kq_lui = 'DUNG_GIUA'                                                                     # sep DUNG trong luc lui
    assert G.chay_mat(io, 'di_B', 1, CFG) == 4, io.cau
    assert len(io.lui_ds) == 1 and any(c.endswith('=> DUNG.') for c in io.cau), io.cau        # da vao lui, DUNG tu trong lui
    assert CUA(io.rows)[-1] == 'DUNG_SEP' and not any(c.startswith('-- tuyen 2') for c in io.cau)


GAN_B = lambda dt: (1.30, 0.82, 0.0)                    # cach B 0,30 m (< GAN_DICH_M 0,60)


def _gan_dich(kich2, lenh=None):
    io = IOMat([dict(ket=None, fb=lambda dt: (0.35, 0), pose=lambda dt: (0.05 * dt, 0.0, 0.0) if dt < 5 else GAN_B(dt), plan=lambda dt: 0.3),
                kich2], lenh=lenh)
    return io, G.chay_mat(io, 'di_B', 1, CFG)


def test_47_mat_gan_dich_huy_cho_60s_gui_lai_mot_lan():
    """09/11 buoc 3 #2: cach dich < 0,60 m (Thien chot 09/11), 10 s khong tien => huy, dung yen 60 s (CHO_VAT n/60), gui lai 1 lan cung tuyen."""
    assert (CFG['gan_dich_m'], CFG['gan_dich_s'], CFG['cho_dich']) == (0.60, 10.0, 60.0)
    io, ma = _gan_dich(dict(ket=('SUCCEEDED', 8.0), fb=lambda dt: (max(0.05, 0.3 - 0.04 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 8 else GAN_B(dt)))
    assert ma == 0, io.cau
    i_gan = [k for k, c in enumerate(io.cau) if c.startswith('   GAN_DICH: cach dich 0.30 m, khong tien 10 s => huy dich, dung yen 60 s')]
    i_l2 = [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 1, lan 2/2: navigate_to_pose tu (1.30; 0.82) toi (1.60; 0.82) (GAN_DICH gui lai)')]
    assert i_gan and i_l2, io.cau
    cho = [c.split()[1] for c in io.cau[i_gan[0]:i_l2[0]] if c.startswith('CHO_VAT:')]
    assert cho == ['%d/60' % k for k in range(5, 60, 5)], cho
    assert io.da_huy == 1 and not getattr(io, 'lui_ds', []) and io.kn == [] and io.chup_ds == []       # khong lui, khong bo khe
    assert [s.split()[0] for _, s in io.chon] == ['0', '1'], io.chon                                   # khong doi khe
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI'] and io.rows[2][3] == '2'
    # "BT khong duoc lui trong luc cho": suot 60 s khong co dich Nav2 nao (cay BT khong chay) va gui khong lui
    t_gan = [io.vet[k][0] for k in range(1, len(io.vet)) if io.vet[k - 1][1] and not io.vet[k][1]][0]     # luc huy dich lan 1
    trong_cho = [(co, lui) for t, co, lui in io.vet if t_gan <= t < t_gan + 60.0]
    assert len(trong_cho) >= 590 and not any(co or lui for co, lui in trong_cho), len(trong_cho)


def test_48_mat_gan_dich_lan_2_van_ket_het_duong_khong_doi_khe():
    io, ma = _gan_dich(dict(ket=None, fb=lambda dt: (0.35, 0), pose=GAN_B))
    assert ma == 1, io.cau
    assert io.cau[-1] == 'HET_DUONG: gan dich ma khong toi (GAN_DICH, tuyen 1), khong doi khe. Xe dung yen.'
    l2 = io.cau[[k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 1, lan 2/2')][0]:]
    assert [c.split()[1] for c in l2 if c.startswith('CHO_VAT:')] == ['5/10'] and '   khong den duoc (GAN_DICH_HET, cach dich 0.30 m).' in l2, l2
    assert io.da_huy == 2 and not getattr(io, 'lui_ds', []) and not any(c.startswith('-- tuyen 2') for c in io.cau)
    assert [s.split()[0] for _, s in io.chon] == ['0', '1'] and CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'KHONG_DAT', 'HET_DUONG']
    io, ma = _gan_dich(dict(ket=None, fb=lambda dt: (0.35, 0)), lenh=lambda t: ('DUNG', 'x') if t > 40 else None)
    assert ma == 4 and CUA(io.rows)[-1] == 'DUNG_SEP' and not any(c.startswith('-- tuyen 1, lan 2/2') for c in io.cau), io.cau
    # xa dich (> 0,60 m) van theo dong ho 30 s bo khe (test_36), khong GAN_DICH; 0,55 m (truoc la xa) nay la GAN_DICH
    io = IOMat([dict(ket=None, fb=lambda dt: (0.65, 0), pose=lambda dt: (0.95, 0.82, 0.0)), None])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1 and not any('GAN_DICH' in c for c in io.cau) and len(io.kn) == 1
    io = IOMat([dict(ket=None, fb=lambda dt: (0.55, 0), pose=lambda dt: (1.05, 0.82, 0.0)), None])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1 and any(c.startswith('   GAN_DICH: cach dich 0.55 m') for c in io.cau), io.cau


def test_52_mat_gan_dich_gui_lai_hoi_sep_nhu_doi_khe():
    """Thien chot 09/11: gui lai sau GAN_DICH dung chung hoi_sep_mat(gui_lai=True) voi gui lai sau doi khe."""
    ok2 = lambda: dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 0.3 - 0.06 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else GAN_B(dt))
    sau_cho = lambda chu, ly: (lambda t: (chu, ly) if t >= 70 else None)       # sep chi noi sau 60 s cho (GAN_DICH luc ~14 s)
    # CHO ly do vi tri: nghe (dung yen 10 s x 2) roi moi gui lai
    io, ma = _gan_dich(ok2(), lenh=sau_cho('CHO', 'NHAY_VI_TRI 0.30 m'))
    assert ma == 0, io.cau
    i1 = io.cau.index('   sep bao CHO (NHAY_VI_TRI 0.30 m): dung yen 10 s (lan 1/2).')
    i2 = io.cau.index('   sep bao CHO (NHAY_VI_TRI 0.30 m): dung yen 10 s (lan 2/2).')
    i_l2 = [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 1, lan 2/2')][0]
    i_cho = max(k for k, c in enumerate(io.cau) if c.startswith('CHO_VAT: 55/60'))
    assert i_cho < i1 < i2 < i_l2, io.cau
    # PLAN_* / ABORT_NHANH: bo qua, gui lai ngay
    for ly in ('PLAN_RONG 3s', 'ABORT_NHANH 1s truoc'):
        io, ma = _gan_dich(ok2(), lenh=sau_cho('CHO', ly))
        assert ma == 0 and '   sep bao CHO (%s): bo qua (khe vua bo).' % ly in io.cau, io.cau
        assert not any('dung yen 10 s' in c for c in io.cau)
    # DUNG (luon nghe): khong gui lai
    hop = []
    io = IOMat([dict(ket=None, fb=lambda dt: (0.35, 0), pose=lambda dt: (0.05 * dt, 0.0, 0.0) if dt < 5 else GAN_B(dt), plan=lambda dt: 0.3),
                ok2()], lenh=lambda t: ('DUNG', 'x') if hop[1:] and t >= hop[1] + 5.0 + 1e-6 else None)
    hop.append(io)
    _bao = io.bao
    io.bao = lambda c: (_bao(c), hop.append(io.t) if c.startswith('CHO_VAT: 55/60') else None)
    ma = G.chay_mat(io, 'di_B', 1, CFG)                                          # DUNG chi xuat hien SAU 60 s cho => do hoi_sep_mat bat
    assert ma == 4 and not any(c.startswith('-- tuyen 1, lan 2/2') for c in io.cau) and CUA(io.rows)[-1] == 'DUNG_SEP', io.cau
    assert io.t >= hop[1] + 5.0 and not any('sep bao CHO' in c for c in io.cau)           # het 60 s cho moi DUNG
    assert G.gan_dich.__code__.co_names.count('hoi_sep_mat') == 1                 # cung mot ham, khong ban sao


def test_49_mat_lui_ra_bt_va_gui_khop_bt_mat_va_trang():
    """09/11 #6: cay BT goi /backup => 'LUI_RA: bt 0.25' / '   LUI_XONG: bt => KQ'; 0,25 = backup_dist cua bt_mat.xml."""
    import re as _re
    import trang_v10 as V
    bt = open(os.path.join(DAY, 'bt_mat.xml')).read()
    assert float(_re.search(r'<BackUp backup_dist="([0-9.]+)"', bt).group(1)) == G.LUI_BT_M == 0.25
    io = IOMat([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 1 if dt > 3 else 0),
                     pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0), lui_bt=[(3.0, 'RA', None), (5.6, 'XONG', 'SUCCEEDED')])])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0, io.cau
    i = io.cau.index('LUI_RA: bt 0.25')
    assert '   LUI_XONG: bt => SUCCEEDED.' in io.cau[i:], io.cau
    assert V.dong_chuyen_cuoi(io.cau[:i + 1]) == 'THOAT' and V.dong_chuyen_cuoi(io.cau[:i + 2]) in ('DI', 'THOAT')
    assert V.dong_chuyen_cuoi(['LUI_RA: gui 0.35 (truoc tuyen 2, /backup 0.10 m/s)']) == 'THOAT'
    assert V.dong_chuyen_cuoi(['LUI_RA: bt 0.25', '   LUI_XONG: bt => ABORTED.']) == 'DI'
    # gia lap 09/11 (ong tai B): gui huy dich giua luc cay BT dang lui (BT huy BackUp) => van in LUI_XONG; su kien cu khong lan sang lan sau
    io = IOMat([dict(ket=None, fb=lambda dt: (0.35, 0), pose=GAN_B, lui_bt=[(9.5, 'RA', None)]),
                dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 0.3 - 0.06 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else GAN_B(dt))])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0, io.cau
    i = io.cau.index('LUI_RA: bt 0.25')
    assert io.cau[i + 1] == '   LUI_XONG: bt => HUY (dich GAN_DICH).' and io.cau[i + 2].startswith('   GAN_DICH:'), io.cau
    assert sum(c.startswith('   LUI_XONG') for c in io.cau) == 1
    assert V.dong_chuyen_cuoi(['   GAN_DICH: cach dich 0.30 m, khong tien 10 s => huy dich, dung yen 60 s roi gui lai 1 lan (khong doi khe).']) == 'CHO'


def test_50_mat_ly_do_kinh_nghiem():
    assert G.ly_do_kinh_nghiem(None, 30.0) == 'VAT_CHAN'
    assert G.ly_do_kinh_nghiem(0.0, 30.0) == G.ly_do_kinh_nghiem(30.0, 30.0) == 'KET_CO_DUONG'
    assert G.ly_do_kinh_nghiem(30.01, 30.0) == 'VAT_CHAN'
    io = IOMat([dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: (0.6, 0.0, 0.0), plan=lambda dt: dt - 5 if dt < 5 else 31.0), None])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1 and io.kn[0][3] == 'VAT_CHAN', io.kn          # /plan cuoi cu hon 30 s


def test_51_mat_vi_tri_xe_noi_dung():
    t = G.noi_dung_vi_tri(1794140000.5, (0.5, -0.25, math.pi / 2), [(0.0, 0.0), (0.5, 0.5)])
    assert t == '1794140000.50 0.500 -0.250 90.0\nplan 2 0.000 0.000 0.500 0.500\n', t
    assert G.noi_dung_vi_tri(1.0, None, []) == '1.00 - - -\nplan 0\n'
    d = G.noi_dung_vi_tri(1.0, (0, 0, 0), [(k * 0.01, 0.0) for k in range(351)]).splitlines()[1].split()
    assert d[1] == '100' and len(d) == 202 and d[-2:] == ['3.500', '0.000'] and d[2:4] == ['0.000', '0.000']


def _kn(io):
    return [(d['su_co'], d['cach'], d['ket_qua'], d['ly_do']) for d in getattr(io, 'kn_moi', [])]


def test_53_nao_xe_kinh_nghiem_moi_loai_su_co():
    """09/11 nao xe viec 4: DOI_KHE (ngay, HONG), BT_LUI / GUI_LUI / CHO_DICH (DUOC neu tien trong 15 s), DONG_BANG (lui < 5 cm)."""
    assert G.HV.so('kinh_nghiem_quan_sat_s') == 15.0 and G.HV.than_the('o_luoi_m') == 0.05
    # BT lui roi xe tien lai => BT_LUI DUOC
    io = IOMat([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 1 if dt > 3 else 0),
                     pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0), lui_bt=[(3.0, 'RA', None), (5.6, 'XONG', 'SUCCEEDED')])])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0 and _kn(io) == [('BT_LUI', 'lui=0.25', 'DUOC', '-')], _kn(io)
    assert io.kn_moi[0]['chuyen'] == 'thu' and io.kn_moi[0]['huong'] == 'di_B'
    # BT lui ma xe khong dich (vi tri nhu nhau) => DONG_BANG HONG (ly_do = su co goc)
    io = IOMat([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 1 if dt > 3 else 0),
                     pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.3, 0.0, 0.0), lui_bt=[(3.0, 'RA', None), (5.6, 'XONG', 'ABORTED')])])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0 and _kn(io) == [('DONG_BANG', 'lui=0.25', 'HONG', 'BT_LUI')], _kn(io)
    # BT lui roi xe van dung 15 s => BT_LUI HONG (chot boi nhip, truoc khi bo khe 30 s)
    io = IOMat([dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: (0.6, 0.0, 0.0) if dt < 3 else (0.4, 0.0, 0.0),
                     lui_bt=[(2.0, 'RA', None), (4.5, 'XONG', 'SUCCEEDED')]), None])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1
    k = _kn(io)
    assert k[0] == ('BT_LUI', 'lui=0.25', 'HONG', '-') and k[1] == ('DOI_KHE', 'cho=30', 'HONG', 'VAT_CHAN'), k
    assert io.kn_moi[0]['t'] < io.kn_moi[1]['t']
    # Gui lui truoc tuyen 2 roi tuyen 2 tien => GUI_LUI DUOC; DOI_KHE ghi truoc
    ket_khe = (0.62, 0.03, 0.0)
    io = IOMat([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe, plan=lambda dt: 1.0),
                dict(ket=('SUCCEEDED', 30.0), fb=lambda dt: (max(0.05, 2.9 - 0.1 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 30 else ket_khe)])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 0
    assert _kn(io) == [('DOI_KHE', 'cho=30', 'HONG', 'KET_CO_DUONG'), ('GUI_LUI', 'lui=0.35', 'DUOC', '-')], _kn(io)
    # Gui lui ma xe dich 0,02 m => DONG_BANG HONG (khong cho GUI_LUI)
    io = IOMat([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe), None])
    io.kq_lui, io.quang_lui = 'ABORTED', 0.02
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1
    assert _kn(io) == [('DOI_KHE', 'cho=30', 'HONG', 'VAT_CHAN'), ('DONG_BANG', 'lui=0.35', 'HONG', 'GUI_LUI')], _kn(io)
    # Gui lui xong nhung khong co tuyen nao chay tiep (tuyen 2 bi tu choi) => GUI_LUI HONG luc het chuyen
    io = IOMat([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe), None])
    assert G.chay_mat(io, 'di_B', 1, CFG) == 1 and _kn(io)[-1] == ('GUI_LUI', 'lui=0.35', 'HONG', '-'), _kn(io)
    # GAN_DICH: cho 60 s roi gui lai toi B => CHO_DICH DUOC; gui lai van ket => CHO_DICH HONG
    io, ma = _gan_dich(dict(ket=('SUCCEEDED', 8.0), fb=lambda dt: (max(0.05, 0.3 - 0.04 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 8 else GAN_B(dt)))
    assert ma == 0 and _kn(io) == [('CHO_DICH', 'cho=60', 'DUOC', '-')], _kn(io)
    io, ma = _gan_dich(dict(ket=None, fb=lambda dt: (0.35, 0), pose=GAN_B))
    assert ma == 1 and _kn(io) == [('CHO_DICH', 'cho=60', 'HONG', '-')], _kn(io)



# ---------- 09/11 dung im ban 2, A2 (gac vat la trong vong theo doi) ----------
TRUOC3 = [(0.40, -0.01), (0.40, 0.0), (0.40, 0.01)]                     # vat la cach mui 0,255 m (vung truoc)


def hong3(ho):
    """3 diem vat la ben TRAI cach canh than `ho` m (khung xe)."""
    return [(x, 0.08 + ho) for x in (-0.02, 0.0, 0.02)]


class IOGac(IOMat):
    """IOMat + Nav2 tra CANCELED sau khi huy (nhu that) de kiem K4."""

    def huy(self):
        super().huy()
        self.cho_canceled = True

    def gui_nav(self, dich, yaw):
        self.cho_canceled = False
        return super().gui_nav(dich, yaw)

    def ket_qua(self):
        if self.dang is None and getattr(self, 'cho_canceled', False):
            return 'CANCELED'
        return super().ket_qua()


def _trong(t, a, b, pts):
    return pts if a <= t < b else []


def test_54_gac_truoc_dung_im_tu_huy_khong_doi_tuyen_sach_2s_gui_lai():
    """Vat la truoc mat 6..14 s: dung im (tu huy, CANCELED khong thanh KHONG_DAT), cho bang dong ho 30 s (CHO_VAT n/30),
    sach 2 s => hoi sep, gui lai CUNG tuyen (khong /chon_tuyen moi), toi B. 1 su co = 1 dong CHO_VAT_LA."""
    di = lambda dt: (min(0.05 * dt, 0.2), 0.0, 0.0)
    io = IOGac([dict(ket=None, fb=lambda dt: (max(1.6, 1.8 - 0.05 * dt), 0), pose=di),
                dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 1.6 - 0.3 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else (0.2, 0.0, 0.0))])
    io.gac_fn = lambda t: _trong(t, 8.0, 17.0, TRUOC3)                  # gui dich luc ~6 s (cong 2 s o A)
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
    i_dung = [k for k, c in enumerate(io.cau) if c.startswith('GAC: dung im, vung=truoc, (0.5')]
    i_het = [k for k, c in enumerate(io.cau) if c.startswith('GAC: het chan sau ')]
    assert len(i_dung) == 1 and len(i_het) == 1 and i_dung[0] < i_het[0], io.cau
    cv = [c.split()[1] for c in io.cau[i_dung[0]:i_het[0]] if c.startswith('CHO_VAT:')]
    assert cv[:1] == ['5/30'] and all(x.endswith('/30') for x in cv), io.cau
    assert CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'] and [x.split()[0] for _, x in io.chon] == ['0', '1']
    assert not any('CANCELED' in c or c.startswith('-- tuyen 2') or c.startswith('GAC: cho cong') for c in io.cau)
    cho = [(co, lui) for t, co, lui in io.vet if 8.2 <= t < 18.9]
    assert cho and not any(co or lui for co, lui in cho)                 # suot luc cho: khong dich Nav2 (khong lenh xoay), khong lui
    assert _kn(io) == [('CHO_VAT_LA', 'lui_doi_khe', 'DUOC', '-')], _kn(io)


def test_55_mot_dong_ho_gui_lai_nhieu_lan_het_30s_lui_roi_doi_khe_mot_dong_csv():
    """K1, K11: gac bat / tat 2 lan, gui lai 1 lan (khong dat lai dong ho); 30 s khong tien => lui 0,35 roi tuyen 2. 1 dong csv."""
    dung = (0.6, 0.0, 0.0)
    io = IOGac([dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: dung, plan=lambda dt: 1.0),
                dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: dung, plan=lambda dt: 1.0),
                dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 2.4 - 0.12 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else dung)])
    io.gac_fn = lambda t: TRUOC3 if (5.0 <= t < 10.0 or 14.0 <= t < 38.0) else []
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
    assert sum(c.startswith('GAC: dung im') for c in io.cau) == 2 and sum(c.startswith('GAC: het chan') for c in io.cau) == 1
    i2 = [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 2')][0]
    cho = [int(c.split()[1].split('/')[0]) for c in io.cau[:i2] if c.startswith('CHO_VAT:')]
    assert cho == [5, 10, 15, 20, 25] and all(c.split()[1].endswith('/30') for c in io.cau[:i2] if c.startswith('CHO_VAT:')), io.cau
    assert '   khong den duoc (CHO_VAT_30s, cach dich 1.29 m).' in io.cau and len(io.lui_ds) == 1
    assert [x.split()[0] for _, x in io.chon] == ['0', '1', '2']
    assert _kn(io) == [('CHO_VAT_LA', 'lui_doi_khe', 'DUOC', 'KET_CO_DUONG')], _kn(io)   # khong DOI_KHE, khong GUI_LUI


def test_56_gac_gan_dich_theo_luat_GAN_DICH_60s():
    io = IOGac([dict(ket=None, fb=lambda dt: (0.35, 0), pose=lambda dt: (0.05 * dt, 0.0, 0.0) if dt < 2 else GAN_B(dt)),
                dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 0.3 - 0.05 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else GAN_B(dt))])
    io.gac_fn = lambda t: _trong(t, 9.0, 11.0, TRUOC3)
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
    i = [k for k, c in enumerate(io.cau) if c.startswith('GAC: dung im')][0]
    assert io.cau[i + 1].startswith('   GAN_DICH: cach dich 0.30 m'), io.cau                   # ngay, khong doi 10 s
    assert sum(c.startswith('CHO_VAT:') and c.split()[1].endswith('/60') for c in io.cau) == 11
    assert _kn(io) == [('CHO_DICH', 'cho=60', 'DUOC', '-')] and [x.split()[0] for _, x in io.chon] == ['0', '1']


def test_57_bt_lui_thang_hong_008_khong_huy_hong_005_huy():
    def chay(ho):
        io = IOGac([dict(ket=None, fb=lambda dt: (1.2, 1 if dt > 3 else 0), pose=lambda dt: (0.6 - 0.05 * max(0.0, min(dt, 5.5) - 3), 0.0, 0.0),
                         lui_bt=[(3.0, 'RA', None), (5.5, 'XONG', 'SUCCEEDED')]), None, None, None])
        io.gac_fn = lambda t: _trong(t, 9.5, 11.0, hong3(ho))  # gio tuyet doi; gui dich luc ~6 s => BT lui 9..11,5 s
        G.chay_mat(io, 'di_B', 1, CFG_GAC)
        return io
    io = chay(0.08)
    assert not any(c.startswith('GAC: huy lui') for c in io.cau) and '   LUI_XONG: bt => SUCCEEDED.' in io.cau, io.cau
    i_x = io.cau.index('   LUI_XONG: bt => SUCCEEDED.')
    assert not any(c.startswith('GAC: dung im') for c in io.cau[:i_x])                       # trong luc lui khong dung theo vung
    io = chay(0.05)
    i_h = [k for k, c in enumerate(io.cau) if c == 'GAC: huy lui, hong < 0.06']
    assert len(i_h) == 1 and i_h[0] < io.cau.index('LUI_RA: bt 0.25') + 3 and io.da_huy >= 1, io.cau
    cho = [c.split()[1] for c in io.cau[i_h[0]:] if c.startswith('CHO_VAT:')]
    assert cho and all(x.endswith('/30') for x in cho)                                        # cung dong ho, khong vong moi


def test_58_gui_lui_hong_005_huy_hong_008_khong():
    def chay(ho):
        ket_khe = (0.62, 0.03, 0.0)
        io = IOGac([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe),
                    dict(ket=('SUCCEEDED', 30.0), fb=lambda dt: (max(0.05, 2.9 - 0.1 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 30 else ket_khe)])
        io.gac_fn = lambda t: hong3(ho) if 37.0 <= t < 38.5 else []    # lui 36..39,5 s
        assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
        return io
    io = chay(0.05)
    assert 'GAC: huy lui, hong < 0.06' in io.cau and any(c.startswith('   LUI_XONG: gui => GAC_HONG') for c in io.cau) and io.huy_lui == 1
    i_c = [k for k, c in enumerate(io.cau) if c.startswith('GAC: cho cong, vung=trai')]          # vat roi di => qua cong, gui tuyen kia
    assert i_c and i_c[0] < [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 2')][0], io.cau
    io = chay(0.08)
    assert not any(c.startswith('GAC: huy lui') for c in io.cau) and '   LUI_XONG: gui => SUCCEEDED (xe lui 0.35 m).' in io.cau


def test_59_vat_khong_roi_ca_hai_cong_het_gio_ma_1_chi_1_lan_doi_khe():
    """Vat la truoc mat (khung xe) tu dau: cong tuyen 1 het 30 s => lui 0,35 => cong tuyen 2 het 30 s => HET_DUONG (ma 1), khong gui dich nao."""
    io = IOGac([dict(ket=None, fb=lambda dt: (1.2, 0))])
    io.gac_fn = lambda t: TRUOC3
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 1 and io.cau[-1].startswith('HET_DUONG'), io.cau
    assert [x.split()[0] for _, x in io.chon] == ['0'] and len(io.lui_ds) == 1 and len(io.kich) == 1     # khong gui dich nao
    assert sum(c.startswith('   khong den duoc (CHO_CONG_30s') for c in io.cau) == 2
    cv = [c.split()[1] for c in io.cau if c.startswith('CHO_VAT:')]
    assert cv == ['%d/30' % k for k in range(5, 30, 5)] * 2, cv
    assert [k[0] for k in _kn(io)] == ['CHO_VAT_LA', 'CHO_VAT_LA'] and [k[2] for k in _kn(io)] == ['HONG', 'HONG'], _kn(io)


def test_60_gac_mu_canh_bao_mot_lan_khong_dung():
    io = IOGac([dict(ket=None, fb=lambda dt: (max(1.0, 1.8 - 0.09 * dt), 0), pose=lambda dt: (min(0.05 * dt, 0.4), 0.0, 0.0)),
                dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 1.0 - 0.2 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else (0.4, 0.0, 0.0))])
    io.gac_fn = lambda t: 'MU' if t < 10 else (TRUOC3 if t < 12 else [])
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0
    assert io.cau.count('CANH BAO gac: mu (gia)') == 1 and sum(c.startswith('GAC: dung im') for c in io.cau) == 1


def test_61_ket_tu_phia_dong_bang_dung_im():
    """K9: truoc + hong chan, lui khong duoc (BackUp tu choi, xe dich 0) => DONG_BANG (ly_do CHO_VAT_LA), khong dich chuyen."""
    dung = (0.6, 0.0, 0.0)
    io = IOGac([dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: dung), dict(ket=None, fb=lambda dt: (1.2, 0), pose=lambda dt: dung)])
    io.gac_fn = lambda t: (TRUOC3 + hong3(0.07)) if t >= 3.0 else []
    io.kq_lui, io.quang_lui = 'ABORTED', 0.0
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 1
    assert _kn(io)[0] == ('CHO_VAT_LA', 'lui_doi_khe', 'HONG', 'KET_CO_DUONG') or _kn(io)[0][0] == 'DONG_BANG', _kn(io)
    assert ('DONG_BANG', 'lui_doi_khe', 'HONG', 'CHO_VAT_LA') in _kn(io) and not any(c.startswith('GAC: het chan') for c in io.cau)
    assert [x.split()[0] for _, x in io.chon] == ['0'] and io.cau[-1].startswith('HET_DUONG')        # dung im, khong gui dich nao


def test_62_gac_tat_y_het_48995c2():
    """gac_vat_la false: chay lai TOAN BO test kieu mat cu (34-53) + co vat la ma khong co dong GAC nao."""
    cu = CFG['gac']
    CFG['gac'] = False
    try:
        for ten in sorted(k for k in globals() if k.startswith('test_') and '34' <= k[5:7] <= '53'):
            globals()[ten]()
        io = IOGac([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0))])
        io.gac_fn = lambda t: TRUOC3
        assert G.chay_mat(io, 'di_B', 1, CFG) == 0 and not any('GAC' in c or 'gac' in c for c in io.cau)
    finally:
        CFG['gac'] = cu


def test_63_gac_doc_map_tinh_va_tf_luc_quet():
    """K10: gac dung /map_tinh (ban do goc), KHONG costmap co lop huong dan; K7: TF theo stamp lan quet; K13 QoS."""
    src = open(os.path.join(DAY, 'gui_duong.py')).read()
    assert "self.create_subscription(OccupancyGrid, '/map_tinh', self._on_map_tinh, giu)" in src
    assert "giu = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)" in src
    assert "self.tf_buf.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp))" in src
    i = src.index('def gac_ban(self):')
    assert 'return self.ban_tinh' in src[i:i + 80] and 'cm_ban' not in src[i:i + 80]


def test_64_ong_030_truoc_o_A_cho_lui_035_gui_tuyen_kia():
    """Ong cach mui 0,30 m o A (ban do (0,445; 0)): cong tuyen 1 cho 30 s (CHO_VAT n/30, khong gui) => lui 0,35 (ong ra 0,65 m,
    ngoai vung RA 0,40) => cong sach 2 s => hoi sep => gui tuyen 2 => toi B. 1 dong CHO_VAT_LA."""
    io = IOGac([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 2.4 - 0.12 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else (-0.35, 0.0, 0.0))])
    ong = [(0.445, -0.01), (0.445, 0.0), (0.445, 0.01)]
    io.gac_fn = lambda t: [(x - io._pose_luc()[0], y - io._pose_luc()[1]) for x, y in ong]
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
    assert io.cau[0].startswith('GAC: cho cong, vung=truoc, (0.45; 0.00)') and len(io.lui_ds) == 1, io.cau
    i_t2 = [k for k, c in enumerate(io.cau) if c.startswith('-- tuyen 2, lan 1/1: navigate_to_pose tu (-0.35; 0.00)')]
    i_lui = [k for k, c in enumerate(io.cau) if c.startswith('LUI_RA: gui 0.35')]
    assert i_lui and i_t2 and i_lui[0] < i_t2[0] and [x.split()[0] for _, x in io.chon] == ['0', '2'], io.cau
    assert [c.split()[1] for c in io.cau[:i_lui[0]] if c.startswith('CHO_VAT:')] == ['5/30', '10/30', '15/30', '20/30', '25/30']
    assert '   khong den duoc (CHO_CONG_30s, cach dich 1.80 m).' in io.cau and CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI']
    assert _kn(io) == [('CHO_VAT_LA', 'lui_doi_khe', 'DUOC', 'CHO_CONG')], _kn(io)


def test_65_huy_lui_vi_hong_vat_van_o_hong_khong_gui_tuyen_kia_30s_ma_1():
    ket_khe = (0.62, 0.03, 0.0)
    io = IOGac([dict(ket=None, fb=lambda dt: (1.75, 0), pose=lambda dt: ket_khe), dict(ket=('SUCCEEDED', 5.0), fb=lambda dt: (0.5, 0))])
    io.gac_fn = lambda t: hong3(0.05) if t >= 37.0 else []
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 1, io.cau
    assert 'GAC: huy lui, hong < 0.06' in io.cau and any(c.startswith('GAC: cho cong, vung=trai') for c in io.cau)
    assert not any(c.startswith('-- tuyen 2') for c in io.cau) and [x.split()[0] for _, x in io.chon] == ['0', '1'] and len(io.kich) == 1
    assert io.cau[-1].startswith('HET_DUONG') and '   khong den duoc (CHO_CONG_30s, cach dich 1.26 m).' in io.cau, io.cau


def test_66_gac_cong_chay_truoc_hoi_sep():
    io = IOGac([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0))],
               lenh=lambda t: ('CHO', 'NHAY_VI_TRI 0.2 m') if t < 40 else None)
    io.gac_fn = lambda t: TRUOC3 if t < 6.0 else []
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0, io.cau
    i_c = [k for k, c in enumerate(io.cau) if c.startswith('GAC: cho cong')]
    i_s = [k for k, c in enumerate(io.cau) if c.startswith('   sep bao CHO (NHAY_VI_TRI')]
    assert i_c and i_s and i_c[0] < i_s[0] and len(i_s) == 2, io.cau



# ---------- 09/11 dung im ban 2, A5: ca con thieu theo thiet ke ----------
def test_67_gac_phai_dung_hai_diem_khong_dung_mu_o_cong_cho_qua():
    for pts, mong in ((hong3(0.05), 'trai'), ([(x, -0.13) for x in (-0.02, 0.0, 0.02)], 'phai'), (TRUOC3[:2], 'khong')):
        io = IOGac([dict(ket=None, fb=lambda dt: (max(1.6, 1.8 - 0.05 * dt), 0), pose=lambda dt: (min(0.05 * dt, 0.2), 0.0, 0.0)),
                    dict(ket=('SUCCEEDED', 6.0), fb=lambda dt: (max(0.05, 1.6 - 0.3 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 6 else (0.2, 0.0, 0.0))])
        io.gac_fn = lambda t, pts=pts: _trong(t, 8.0, 10.0, pts)
        G.chay_mat(io, 'di_B', 1, CFG_GAC)
        dung = [c for c in io.cau if c.startswith('GAC: dung im')]
        if mong == 'khong':
            assert dung == [], io.cau                                                    # 2 diem: khong dung
        else:
            assert dung and dung[0].startswith('GAC: dung im, vung=%s' % mong), io.cau
    io = IOGac([dict(ket=('SUCCEEDED', 5.0), fb=lambda dt: (max(0.05, 1.0 - 0.2 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 5 else (0.0, 0.0, 0.0))])
    io.gac_fn = lambda t: 'MU'
    assert G.chay_mat(io, 'di_B', 1, CFG_GAC) == 0 and io.cau[0] == 'CANH BAO gac: mu (gia)' and io.cau.count('CANH BAO gac: mu (gia)') == 1


def test_68_yaml_gac_vat_la_false_tat_gac():
    import tempfile
    p = os.path.join(tempfile.mkdtemp(), 'hanh_vi_xe.yaml')
    with open(p, 'w') as f:
        f.write('thoi_quen:\n  gac_vat_la: {gia_tri: false}\n')
    cu = os.environ.get('HANH_VI_YAML')
    os.environ['HANH_VI_YAML'] = p
    try:
        with contextlib.redirect_stderr(_io.StringIO()):
            G.HV.nap_lai()
            assert G.cau_hinh({})['gac'] is False
    finally:
        os.environ.pop('HANH_VI_YAML')
        if cu is not None:
            os.environ['HANH_VI_YAML'] = cu
        with contextlib.redirect_stderr(_io.StringIO()):
            G.HV.nap_lai()
    assert G.cau_hinh({})['gac'] is True                                                 # mac dinh (yaml kho) = bat


def test_69_d1_dong_gac_khong_doi_trang_va_cau_noi():
    """K12: GAC: / CANH BAO gac: chi la log: cau_noi khong bat; trang giu trang thai cua dong co nghia truoc (CHO_VAT n/30)."""
    import trang_v10 as V
    src = open(os.path.join(DAY, 'cau_noi_xe.py')).read()
    dong_chuyen = re.compile(re.search(r'DONG_CHUYEN = re\.compile\(r"(.+?)"\)', src).group(1))
    for c in ('GAC: dung im, vung=truoc, (0.62; 0.03)', 'GAC: het chan sau 12 s', 'GAC: cho cong, vung=trai, (0.70; 0.13)',
              'GAC: huy lui, hong < 0.06', 'CANH BAO gac: mu (chua co /map_tinh)'):
        assert not dong_chuyen.match(c), c
        assert V.chuyen_cuoi_ct(['-- tuyen 1, lan 1/1: x', 'CHO_VAT: 10/30 s (cach dich 1.20 m)', c]) == ('CHO', 'CHO_VAT: 10/30 s (cach dich 1.20 m)')
        assert V.chuyen_cuoi_ct(['-- tuyen 1, lan 1/1: x', c])[0] == 'DI'
    assert V.chuyen_cuoi_ct(['GAC: cho cong, vung=truoc, (0.45; 0.00)', 'CHO_VAT: 5/30 s']) == ('CHO', 'CHO_VAT: 5/30 s')
    assert V.chuyen_cuoi_ct(['   khong den duoc (CHO_CONG_30s, cach dich 1.80 m).'])[0] == 'CHO'
    kx = open(os.path.join(DAY, 'kiem_xe.sh')).read()
    assert 'python3 ~/gac_vat_la.py --kiem-map 10 || KQ=1' in kx                          # K13
    sv = open(os.path.join(DAY, 'so_voi_pi.sh')).read()
    assert all(('\n%s|~/%s\n' % (t, t)) in sv for t in ('hanh_vi_xe.yaml', 'hanh_vi.py', 'kiem_hanh_vi.py', 'gac_vat_la.py'))

# ---------- 09/11 gia X (Thien chot 17:50-17:57): X = cho KET_CO_DUONG chua DA_GIAI; xep tuyen theo dai + v x giay X; DA_GIAI ----------
def _kn_x(x, y, n, ly_do='KET_CO_DUONG', cach='cho=30', chuyen='c0', su_co='DOI_KHE'):
    """Mot dong nhu hanh_vi.doc_kinh_nghiem tra (chuoi + so_dong)."""
    return dict(gio='g', chuyen=chuyen, x='%.3f' % x, y='%.3f' % y, huong='di_B', su_co=su_co, cach=cach, ket_qua='HONG', ly_do=ly_do, so_dong=n)


X1 = (0.66, 0.07)          # nam tren tuyen 1 (BAN: cach 0,013 m), cach tuyen 2 0,64 m
DI_QUA_X1 = lambda dt: TOI_B(dt) if dt >= 20 else (0.08 * dt, 0.07, 0.0)     # vet that di qua X1 (dt ~ 8 s)


def _mat_gia_x(kn_doc, pose=DI_QUA_X1):
    io = IOMat([dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 0), pose=pose)])
    io.kn_doc = kn_doc
    return io, G.chay_mat(io, 'di_B', 1, CFG)


def _voi_so(thay, f):
    """Chay f() voi HV.so tra so khac cho cac khoa trong `thay` (khong dung fixture pytest: tep cung chay bang python3)."""
    goc = G.HV.so
    G.HV.so = lambda k: thay[k] if k in thay else goc(k)
    try:
        return f()
    finally:
        G.HV.so = goc


def test_70_gia_x_diem_x_gop_giay_da_giai():
    d = [_kn_x(0.66, 0.07, 1), _kn_x(0.70, 0.08, 2, cach='cho=20'), _kn_x(1.50, 0.80, 3, cach=''), _kn_x(0.30, 0.10, 4, ly_do='VAT_CHAN'),
         _kn_x(1.0, 1.0, 5, su_co='BT_LUI', cach='lui=0.25', ly_do='-')]
    xs = G.diem_x(d, 0.20, 10.0)
    assert len(xs) == 2 and abs(xs[0][0] - 0.68) < 1e-9 and abs(xs[0][2] - 25.0) < 1e-9 and xs[1][2] == 10.0, xs   # TB cho=N; khong co => mac dinh
    assert G.diem_x(d + [_kn_x(0.67, 0.07, 6, ly_do='DA_GIAI', cach='-')], 0.20, 10.0) == [xs[1]]            # DA_GIAI sau => da giai
    assert G.diem_x([_kn_x(0.67, 0.07, 1, ly_do='DA_GIAI', cach='-')] + d[:2], 0.20, 10.0)[0][:2] == xs[0][:2]   # DA_GIAI truoc => van la X
    duong = [(0.0, 0.0), (1.0, 0.0)]
    assert G.kc_duong((0.5, 0.2), duong) == 0.2 and G.gia_tuyen(duong, [(0.5, 0.1, 30.0), (0.5, 0.5, 9.0)], 0.2, 0.11) == (1.0, 30.0, 1.0 + 3.3)


def test_71_gia_x_khong_co_X_y_het_cu():
    io, ma = _mat_gia_x([])
    assert ma == 0 and [s for _, s in io.chon] == ['0', '1 0.000 0.000'] and not any(c.startswith('GIA_X') for c in io.cau), io.cau
    assert io.rows[0][1] == '1' and CUA(io.rows) == ['KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'] and not getattr(io, 'kn_moi', [])


def test_72_gia_x_co_X_tren_tuyen_1_doi_thu_tu_va_DA_GIAI():
    io, ma = _mat_gia_x([_kn_x(X1[0], X1[1], 1)])
    g = [c for c in io.cau if c.startswith('GIA_X: t1')]
    assert ma == 0 and len(g) == 1 and g[0].endswith('thu tu (2,1)') and '+ 30s' in g[0].split(';')[0] and 't2 2.12m + 0s' in g[0], io.cau
    assert [s for _, s in io.chon] == ['0', '2 0.000 0.000'] and io.rows[0][1] == '2', (io.chon, io.rows)   # KIEM_DAT ghi tuyen dau moi
    dg = [d for d in io.kn_moi if d['ly_do'] == 'DA_GIAI']                                                    # vet di qua X, khong su co
    assert len(dg) == 1 and dg[0]['su_co'] == 'DA_GIAI' and abs(dg[0]['x'] - X1[0]) < 1e-9 and dg[0]['chuyen'] == 'thu', io.kn_moi
    assert any(c.startswith('GIA_X: DA_GIAI (0.66; 0.07)') for c in io.cau)


def test_73_gia_x_X_da_giai_bo_qua():
    io, ma = _mat_gia_x([_kn_x(X1[0], X1[1], 1), _kn_x(0.67, 0.07, 2, ly_do='DA_GIAI', cach='-', su_co='DA_GIAI')])
    assert ma == 0 and [s for _, s in io.chon] == ['0', '1 0.000 0.000'] and not any(c.startswith('GIA_X') for c in io.cau), io.cau


def test_74_gia_x_tat_y_het_cu():
    io, ma = _voi_so({'gia_x_bat': False}, lambda: _mat_gia_x([_kn_x(X1[0], X1[1], 1)]))
    assert ma == 0 and [s for _, s in io.chon] == ['0', '1 0.000 0.000'] and not any(c.startswith('GIA_X') for c in io.cau), io.cau
    assert not getattr(io, 'kn_moi', [])                                                                     # khong ghi DA_GIAI


def test_75_gia_x_khong_DA_GIAI_khi_vet_xa_hoac_co_su_co_gan_X():
    io, ma = _mat_gia_x([_kn_x(X1[0], X1[1], 1)], pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.03 * dt, 0.03 * dt * 0.8 / 0.6, 0.0))
    assert ma == 0 and not [d for d in getattr(io, 'kn_moi', []) if d['ly_do'] == 'DA_GIAI'], getattr(io, 'kn_moi', [])                    # vet khe tren, khong qua X
    su_co = _kn_x(0.70, 0.07, 2, ly_do='-', cach='lui=0.25', chuyen='thu', su_co='BT_LUI')                    # dong su co CUA chuyen nay gan X
    io, ma = _mat_gia_x([_kn_x(X1[0], X1[1], 1), su_co])
    assert ma == 0 and not [d for d in getattr(io, 'kn_moi', []) if d['ly_do'] == 'DA_GIAI'], getattr(io, 'kn_moi', [])


def test_76_gia_x_tinh_duong_loi_giu_thu_tu_log_1_dong():
    goc = G.TT.tinh_duong

    def hong(ban, dau, cuoi, tuyen, **k):
        if tuyen == 2:
            raise G.TT.KhongCoDuong('gia: khong toi duoc dich')
        return goc(ban, dau, cuoi, tuyen, **k)
    G.TT.tinh_duong = hong
    try:
        io, ma = _mat_gia_x([_kn_x(X1[0], X1[1], 1)])
    finally:
        G.TT.tinh_duong = goc
    g = [c for c in io.cau if c.startswith('GIA_X')]
    assert ma == 0 and g[0] == 'GIA_X: bo (tuyen 2 KhongCoDuong: gia: khong toi duoc dich); thu tu (1,2)', g
    assert [s for _, s in io.chon] == ['0', '1 0.000 0.000'], io.chon


# ---------- 09/11 18:44 ②a luan phien khe so_vong vong; ②b kinh nghiem DUOC gan nhat thu truoc (chi la huong dan) ----------
KET_KHE = (0.70, 0.80, 0.0)
KHE_KET = dict(ket=None, fb=lambda dt: (1.20, 0), pose=lambda dt: KET_KHE, plan=lambda dt: 0.5)      # 30 s khong tien => bo khe


def test_77_luan_phien_khe_3_vong_het_thi_CHO_NGUOI_roi_thu_lai():
    """18:44 ②a + 18:47 (i): 3 vong 1-2-1-2-1-2 hong => KHONG thoat: CHO_NGUOI (CSV, log n/60 s), dung yen 60 s, thu lai 1 vong."""
    toi = dict(ket=('SUCCEEDED', 25.0), fb=lambda dt: (max(0.05, 1.5 - 0.06 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 25 else KET_KHE)
    io = IOMat([dict(KHE_KET) for _ in range(7)] + [toi])
    ma = G.chay_mat(io, 'di_B', 1, CFG3)
    assert (CFG3['so_vong'], CFG3['cho_lai']) == (3, 60.0) and ma == 0, io.cau
    assert [s.split()[0] for _, s in io.chon] == ['0', '1', '2', '1', '2', '1', '2', '1', '2'], io.chon     # 3 vong + 1 vong thu lai
    assert CUA(io.rows) == ['KIEM_DAT'] + ['KHONG_DAT'] * 6 + ['CHO_NGUOI', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI'], CUA(io.rows)
    i = next(k for k, c in enumerate(io.cau) if c.startswith('CHO_NGUOI: da thu 6 lan khe'))
    cho = [c for c in io.cau[i:] if c.startswith('CHO_NGUOI: ') and '/60 s' in c]
    assert [c.split()[1] for c in cho] == ['5/60', '10/60', '15/60', '20/60', '25/60', '30/60', '35/60', '40/60', '45/60', '50/60', '55/60'], cho
    assert not any(c.startswith('HET_DUONG') for c in io.cau) and sum(c.startswith('LUI_RA: gui') for c in io.cau) == 7
    t_cho = next(t for t, s in io.chon[7:])                                                                  # lan thu lai sau >= 60 s dung yen
    t_het6 = [t for t, s in io.chon][6]
    assert t_cho - t_het6 >= 60.0


def test_77b_CHO_NGUOI_bam_dung_thi_thoat_NGAT_130():
    class NgatKhiCho(IOMat):
        def ngu(self, s):
            super().ngu(s)
            if any(c.startswith('CHO_NGUOI: 20/60 s') for c in self.cau):
                raise KeyboardInterrupt
    io = NgatKhiCho([dict(KHE_KET), dict(KHE_KET)])
    ma = G.chay_co_ngat(io, 'di_B', 1, dict(CFG3, so_vong=1), da_ngat=lambda: False, chay_fn=G.chay_mat)
    assert ma == 130 and CUA(io.rows)[-2:] == ['CHO_NGUOI', 'NGAT'] and io.dang is None, (ma, CUA(io.rows), io.cau[-3:])


def test_78_luan_phien_khe_toi_noi_o_vong_2():
    io = IOMat([dict(KHE_KET), dict(KHE_KET), dict(ket=('SUCCEEDED', 25.0), fb=lambda dt: (max(0.05, 1.5 - 0.06 * dt), 0),
                                                   pose=lambda dt: TOI_B(dt) if dt >= 25 else KET_KHE)])
    ma = G.chay_mat(io, 'di_B', 1, CFG3)
    assert ma == 0 and [s.split()[0] for _, s in io.chon] == ['0', '1', '2', '1'], (io.chon, io.cau)
    assert CUA(io.rows) == ['KIEM_DAT', 'KHONG_DAT', 'KHONG_DAT', 'SUCCEEDED', 'DEN_NOI']


def test_79_kinh_nghiem_BT_LUI_DUOC_gan_cho_ket_thi_gui_lai_cung_khe_truoc():
    io = IOMat([dict(KHE_KET) for _ in range(4)])
    io.kn_doc = [_kn_x(0.72, 0.80, 1, ly_do='-', cach='lui=0.25', su_co='BT_LUI') | dict(ket_qua='DUOC')]
    ma = G.chay_mat(io, 'di_B', 1, CFG)                                                                     # 1 vong (cu)
    assert ma == 1 and [s.split()[0] for _, s in io.chon] == ['0', '1', '1', '2', '2'], (io.chon, io.cau)   # moi khe chen 1 lan, khong lien tiep
    k = [c for c in io.cau if c.startswith('KINH_NGHIEM:')]
    assert k == ['KINH_NGHIEM: BT_LUI DUOC cach 0.02 m => lui roi gui lai tuyen %d truoc' % r if j % 2 == 0 else
                 'KINH_NGHIEM: BT_LUI DUOC cach 0.02 m => thu tu cu' for j, r in enumerate((1, 1, 2, 2))], k
    assert sum(c.startswith('LUI_RA: gui') for c in io.cau) == 3 and io.kich == []                           # moi lan gui lai deu lui truoc


def test_80_kinh_nghiem_khong_co_DUOC_gan_thu_tu_cu():
    io = IOMat([dict(KHE_KET), dict(KHE_KET)])
    io.kn_doc = [_kn_x(0.72, 0.80, 1, ly_do='-', cach='lui=0.25', su_co='BT_LUI'),                        # HONG: khong tinh
                 _kn_x(1.50, 0.10, 2, ly_do='-', cach='lui=0.25', su_co='BT_LUI') | dict(ket_qua='DUOC')]   # xa > r
    ma = G.chay_mat(io, 'di_B', 1, CFG)
    assert ma == 1 and [s.split()[0] for _, s in io.chon] == ['0', '1', '2'] and not any(c.startswith('KINH_NGHIEM') for c in io.cau)


def test_81_kinh_nghiem_GAN_DICH_GUI_LUI_DUOC_thi_lui_truoc_khi_cho():
    io = IOMat([dict(ket=None, fb=lambda dt: (0.35, 0), pose=lambda dt: (0.05 * dt, 0.0, 0.0) if dt < 5 else GAN_B(dt), plan=lambda dt: 0.3),
                dict(ket=('SUCCEEDED', 8.0), fb=lambda dt: (max(0.05, 0.3 - 0.04 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 8 else GAN_B(dt))])
    io.kn_doc = [_kn_x(1.32, 0.82, 1, ly_do='-', cach='lui=0.35', su_co='GUI_LUI') | dict(ket_qua='DUOC')]
    ma = G.chay_mat(io, 'di_B', 1, CFG)
    i = next(k for k, c in enumerate(io.cau) if c.startswith('   GAN_DICH:'))
    assert ma == 0 and io.cau[i + 1] == 'KINH_NGHIEM: GUI_LUI DUOC cach 0.02 m => lui 0.35 truoc khi cho', io.cau
    assert len(getattr(io, 'lui_ds', [])) == 1 and any(c.startswith('CHO_VAT:') for c in io.cau[i + 2:])     # lui roi van cho nhu cu


def test_82_SAI_VI_TRI_thanh_CHO_NGUOI_roi_kiem_lai():
    """09/11 19:05: chua co vi tri luc bat dau => KHONG thoat 3: CHO_NGUOI (khong gui lenh), het cho_lai s kiem lai, co vi tri thi di."""
    ok = dict(ket=('SUCCEEDED', 20.0), fb=lambda dt: (max(0.05, 1.8 - 0.09 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 20 else (0.05 * dt, 0.0, 0.0))

    class ViTriSau(IOMat):
        def cho_san_sang(self):
            return (self._pose_luc() if self.t >= 30 else None), self.b
    io = ViTriSau([ok])
    ma = G.chay_mat(io, 'di_B', 1, CFG3)
    assert ma == 0 and CUA(io.rows) == ['CHO_NGUOI', 'KIEM_DAT', 'SUCCEEDED', 'DEN_NOI'], (CUA(io.rows), io.cau)
    assert any(c.startswith('CHO_NGUOI: SAI_VI_TRI (khong co ') for c in io.cau) and not any(c.startswith('DUNG: SAI_VI_TRI') for c in io.cau)
    assert [s for _, s in io.chon][:2] == ['0', '1 0.000 0.000']                                           # chua gui gi truoc khi co vi tri


def test_83_GAN_DICH_lan_2_hong_thanh_CHO_NGUOI_roi_gui_lai():
    """09/11 19:05: GAN_DICH gui lai lan 2 van khong toi => KHONG HET_DUONG: CHO_NGUOI cho_lai s, gui lai (khong cho cho_dich nua)."""
    xa = dict(ket=None, fb=lambda dt: (0.35, 0), pose=lambda dt: (0.05 * dt, 0.0, 0.0) if dt < 5 else GAN_B(dt), plan=lambda dt: 0.3)
    ket_gan = dict(ket=None, fb=lambda dt: (0.30, 0), pose=GAN_B, plan=lambda dt: 0.3)
    toi = dict(ket=('SUCCEEDED', 8.0), fb=lambda dt: (max(0.05, 0.3 - 0.04 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 8 else GAN_B(dt))
    io = IOMat([xa, ket_gan, toi])
    ma = G.chay_mat(io, 'di_B', 1, dict(CFG3, so_vong=1))
    assert ma == 0 and 'HET_DUONG' not in CUA(io.rows) and CUA(io.rows)[-3:] == ['CHO_NGUOI', 'SUCCEEDED', 'DEN_NOI'], (CUA(io.rows), io.cau)
    i = next(k for k, c in enumerate(io.cau) if c.startswith('CHO_NGUOI: GAN_DICH tuyen 1 khong toi'))
    assert sum(c.startswith('CHO_VAT:') and '/60 s' in c for c in io.cau[i:]) == 0                          # sau CHO_NGUOI: gui lai ngay
    assert sum(c.startswith('   GAN_DICH: cach dich') for c in io.cau) == 1


def test_84_sep_DUNG_roi_DI_sau_1_luot_cho_thi_toi_dich():
    """09/11: sep DUNG KHONG la loi ra: dang chay sep bao DUNG => huy dich, CHO_NGUOI 60 s, hoi lai sep (het DUNG) => di tiep vong khe."""
    chay_mai = dict(ket=None, fb=lambda dt: (1.20, 0), pose=lambda dt: KET_KHE)
    toi = dict(ket=('SUCCEEDED', 25.0), fb=lambda dt: (max(0.05, 1.5 - 0.06 * dt), 0), pose=lambda dt: TOI_B(dt) if dt >= 25 else KET_KHE)
    io = IOMat([chay_mai, toi], lenh=lambda t: ('DUNG', 'HET_3_LUOT_THOAT') if 6 < t < 50 else None)
    ma = G.chay_mat(io, 'di_B', 1, CFG3)
    assert ma == 0 and 'DUNG_SEP' not in CUA(io.rows) and CUA(io.rows).count('CHO_NGUOI') == 1, (CUA(io.rows), io.cau)
    assert any(c.startswith('CHO_NGUOI: sep DUNG (HET_3_LUOT_THOAT)') for c in io.cau) and 'CHO_NGUOI: sep het DUNG, di tiep vong khe.' in io.cau
    assert io.da_huy >= 1 and CUA(io.rows)[-2:] == ['SUCCEEDED', 'DEN_NOI'] and [s.split()[0] for _, s in io.chon] == ['0', '1', '2']


def test_85_sep_DUNG_mai_thi_CHO_NGUOI_lap_khong_thoat():
    class DemCho(IOMat):
        def ngu(self, s):
            super().ngu(s)
            if CUA(self.rows).count('CHO_NGUOI') >= 3:
                raise KeyboardInterrupt                       # nguoi bam 정지 sau 3 lan cho
    io = DemCho([dict(ket=None, fb=lambda dt: (1.20, 0), pose=lambda dt: KET_KHE)], lenh=lambda t: ('DUNG', 'KHONG_TIN_QUA_15s') if t > 6 else None)
    ma = G.chay_co_ngat(io, 'di_B', 1, CFG3, da_ngat=lambda: False, chay_fn=G.chay_mat)
    assert ma == 130 and 'DUNG_SEP' not in CUA(io.rows) and CUA(io.rows).count('CHO_NGUOI') == 3 and CUA(io.rows)[-1] == 'NGAT', CUA(io.rows)
    assert io.t > 120 and io.kich == []                                    # da cho >= 2 lan 60 s, khong gui them dich nao


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
