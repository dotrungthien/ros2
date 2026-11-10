#!/usr/bin/env python3
"""test_phan_loai_cum.py (10/11 A+, Thien duyet 15:34): phan loai cum BA DAU HIEU + THOAT RA XA (yaml mac dinh) va DUONG LUI.
GIA LAP (canh phu): tuong = map_xuong_v3 (the_gioi_gia), LiDAR gia 420 tia, Nav2 gia (Dijkstra). KHONG chung minh tren xe.
Chay: python3 xe_scripts/test_phan_loai_cum.py"""
import json
import math
import os
import subprocess
import sys
import tempfile

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import test_gui_duong as X  # noqa: E402  (IOThe, the gioi gia; tep do tu ghim duong lui luc import)
import gac_vat_la as GV  # noqa: E402
import sep_lidar as SL  # noqa: E402
import the_gioi_gia as TG  # noqa: E402

HV, G = X.HV, X.G


def ghim(phan_loai='ba_dau_hieu', ra_xa=True):
    HV._nap()['tq'].update(phan_loai_cum=phan_loai, thoat_ra_xa=ra_xa)


ghim()
BAN = X.BAN_GAC
K4 = (0.734, -0.165)                     # goc K4 tren ban do v3 (canh tren y -0,165 x 0,384..0,734; canh phai x 0,734)
H0 = (0.534, 0.335)                      # goc H0 (khoi giua x 0,534..1,184, y 0,335..0,535)


def ong(cx, cy, r=0.09, n=24):
    """Ong tron (O18: r 0,09) = da giac n canh, khung ban do."""
    p = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    return [(p[k], p[(k + 1) % n]) for k in range(n)]


def xem(doan, pose):
    q = TG.quet(doan, pose)
    return GV.danh_gia(q, GV.bien_doi(q, pose), BAN, GV.tham_so())


def cum_gan(ds, xy, r=0.12):
    """Cac muc danh_gia co diem ban do gan nhat cach xy <= r."""
    return [d for d in ds if d['xy'] is not None and math.dist(d['xy'], xy) <= r]


def test_01_ong_luon_la_VAT():
    ca = [('giua san', ong(1.00, 0.10), (0.72, 0.10, 0.0), (1.00, 0.10)),
          ('sat tuong y -0,365', ong(1.30, -0.365 + 0.09 + 0.005), (0.95, -0.12, -0.3), (1.30, -0.365 + 0.095))]
    for g in (0.0, 0.01, 0.03):                                               # sat canh tren K4 cach 0 / 1 / 3 cm
        ca.append(('sat K4 %.2f' % g, ong(0.60, -0.165 + 0.09 + g), (0.95, -0.02, math.pi), (0.60, -0.165 + 0.09 + g)))
    ca.append(('sat H0', ong(0.42, 0.335 - 0.09 - 0.005), (0.20, 0.15, 0.4), (0.42, 0.335 - 0.095)))
    for ten, o, pose, tam in ca:
        q = TG.quet(X.TUONG + o, pose)
        m = GV.bien_doi(q, pose)
        ds = GV.danh_gia(q, m, BAN, GV.tham_so())
        dd = [d for d in ds if any(m[k] is not None and math.dist(m[k], tam) <= 0.095 for k in d['ks'])]     # cum chua diem cua ong (tam = tam ong, r 0,09)
        assert dd and all(d['loai'] == 'VAT' and d['nhan'] in ('ong', 'ngoai') for d in dd), (ten, [GV.dong_mem(d) for d in ds])
        print('   %-22s %s' % (ten, GV.dong_mem(dd[0])))          # nhan: ong (cung) hoac ngoai ban do (cum dinh tuong)


def test_02_goc_K4_tuong_thang_goc_H0_la_TUONG():
    for pose in ((0.76, -0.07, math.radians(172)), (0.76, -0.07, math.radians(-172)), (0.98, -0.05, math.radians(-160)),
                 (0.90, 0.02, math.radians(-150))):
        ds = cum_gan(xem(X.TUONG, pose), K4, 0.40)
        assert ds and all(d['loai'] == 'TUONG' and d['nhan'] == 'tuong' for d in ds), (pose, [GV.dong_mem(d) for d in ds])
        assert GV.do_vat(ds) is None or GV.do_vat(ds).get('cham'), pose                 # chi 'cham' (<= 0,02) moi la do
    ds = xem(X.TUONG, (1.30, -0.22, 0.0))                                       # tuong thang y -0,365 (duoi), hong ~0,065
    assert ds and all(d['loai'] == 'TUONG' for d in ds if d['vung'] == 'hong'), [GV.dong_mem(d) for d in ds]
    ds = cum_gan(xem(X.TUONG, (0.30, 0.15, math.radians(40))), H0, 0.30)
    assert ds and all(d['loai'] == 'TUONG' for d in ds), [GV.dong_mem(d) for d in ds]
    m = [GV.dong_mem(d) for d in ds]
    assert any('(tuong: tren ban do, goc ' in c or '(tuong: tren ban do, thang' in c for c in m), m


def test_03_hop_la_ngoai_ban_do_la_VAT_va_vat_chac():
    ds = xem(X.TUONG + TG.doan_hop(1.00, 0.10, 0.10), (0.72, 0.10, 0.0))
    d = ds[0]
    assert (d['loai'], d['nhan'], d['chac']) == ('VAT', 'ngoai', True) and 'ngoai ban do' in GV.dong_mem(d), GV.dong_mem(d)


def test_04_hong_tuong_khong_do_chi_cham_002():
    t = GV.tham_so()
    for y, mong in ((-0.165 + 0.08 + 0.04, False), (-0.165 + 0.08 + 0.015, True)):              # canh tren K4 cach hong 0,04 / 0,015
        ds = [d for d in xem(X.TUONG, (0.55, y, 0.0)) if d['vung'] == 'hong']
        assert ds and ds[0]['loai'] == 'TUONG', [GV.dong_mem(d) for d in ds]
        assert (GV.do_vat(ds) is not None) == mong, (y, [GV.dong_mem(d) for d in ds], t['khong_cham'])


def test_05_thoat_ra_xa_cho_dong_tac_khi_dang_trong_do_neu_khong_gan_hon():
    """Dang trong vung do (VAT hong 0,04): luat cu cam xoay (diem cach tam <= 0,186); thoat ra xa: xoay / lui duoc neu khe nho nhat
    khong giam; dong tac lam gan hon van bi cam."""
    t = SL.tham_so()
    vat = TG.doan_hop(0.10, 0.08 + 0.04 + 0.03, 0.06)                          # hop 6 cm hong trai phia mui, cach hong 0,04 (khung xe = ban do)
    pl = SL.diem_loc(TG.quet(vat, (0.0, 0.0, 0.0)), t['gac'])
    assert t['ra_xa'] is True and SL.cham(pl, ('lui', 0.10), t)['kha_thi']
    c = SL.cham(pl, ('xoay', math.radians(-30)), t)
    assert c['kha_thi'], c                                                      # xoay phai (mui ra xa hop): duoc
    c = SL.cham(pl, ('xoay', math.radians(60)), t)
    assert not c['kha_thi'] and c['ly'].startswith('cham vat'), c                       # xoay trai lam gan hon (dang < 0,06): cam
    ghim('cu', False)
    try:
        t0 = SL.tham_so()
        assert SL.cham(pl, ('xoay', math.radians(-30)), t0)['ly'].startswith('xoay: diem cach tam')    # 629a17b: cam xoay
    finally:
        ghim()


def test_05b_xoay_quet_ca_vong_goc_xe_quet_ong_bi_cam_xoay_ra_xa_duoc():
    """Thien (truoc commit): kiem xoay xet CA VONG QUET cua than 25 x 16 (mau 1 do <= 5 do), khong chi tu the cuoi. Ong O18 (VAT,
    cung) truoc-trai, cach hong 0,07 (ngoai do): goc truoc-trai quet vao ong khi xoay trai 30 / 90 => cam; xoay phai (ra xa) => duoc.
    Xoay 90 trai: tu the cuoi goc da qua khoi ong nhung GIUA vong quet thi cham => van cam."""
    t = SL.tham_so()
    o = ong(0.10, 0.08 + 0.07 + 0.09)
    q = TG.quet(o, (0.0, 0.0, 0.0))
    ds = GV.danh_gia(q, None, None, t['gac'])
    assert ds and all(d['nhan'] == 'ong' and d['loai'] == 'VAT' for d in ds), [GV.dong_mem(d) for d in ds]
    pl, fl = SL.diem_loc_nhan(q, t['gac'], ds)
    assert fl is not None and not any(fl)
    for g in (30, 90):
        c = SL.cham(pl, ('xoay', math.radians(g)), t, fl)
        assert not c['kha_thi'] and c['ly'].startswith('cham vat'), (g, c)
    # vong quet: cham dung o MAU dau tien cham (khong doi toi tu the cuoi): xoay 30 trai cham truoc khi het 30 do
    n_cham = next(i for i in range(1, 31) if SL.khe(pl, t, SL.tu_the_luc(('xoay', math.radians(30)), i / 30), fl)['nho_v'] < 0.06 - SL.SAI)
    assert n_cham < 30, n_cham
    c = SL.cham(pl, ('xoay', math.radians(-30)), t, fl)
    assert c['kha_thi'], c                                                      # xoay phai 30: mui ra xa ong => duoc
    c = SL.cham(pl, ('xoay', math.radians(-90)), t, fl)
    assert not c['kha_thi'] and c['ly'].startswith('cham vat'), c                # phai 90: goc SAU-trai quet vao ong => cam (dung)


def test_09_ca_1553_khong_ong_goc_K4_hong_006_la_tuong_khong_DOI_VAT():
    """Bang chung 15:53-15:54 (KHONG co ong, bo 629a17b + e9f0f94): ve_A 'GAC: dung im, hong vat cach 0.06 m (0.59; -0.20)' => DOI_VAT
    30 s => tien 0,20 / tien 0,10 / xoay +30 / diem => DEN_NOI 74 s. A+: goc K4 o hong 0,06 la TUONG (tren ban do, thang / goc) =>
    khong vung do, khong DOI_VAT; xe di thang ve A."""
    p = (0.68, -0.165 + 0.06 + 0.08, math.radians(176))
    ds = cum_gan(xem(X.TUONG, p), (0.59, -0.20), 0.15)
    assert ds and all(d['loai'] == 'TUONG' for d in ds) and GV.do_vat(ds) is None, [GV.dong_mem(d) for d in ds]
    cach = TG.CACH
    TG.CACH = 0.16
    try:
        io = X.NgatKhiChoNguoi(p, tuong=X.TUONG)
        ma = G.chay_mat(io, 've_A', 1, X.CFG3)
    finally:
        TG.CACH = cach
    assert ma == 0 and X.CUA(io.rows)[-1] == 'DEN_NOI', io.cau[-20:]
    assert not any('DOI_VAT' in c or c.startswith('GAC: dung im') or c.startswith('SEP_LIDAR: THOAT') for c in io.cau), io.cau
    print('   ca 15:53 (A+): DEN_NOI, khong DOI_VAT, khong THOAT; MEM %s' % [c for c in io.cau if c.startswith('MEM: hong')][:1])


# ---- phat lai 15:18-15:25 (ve_A, ong O18 gan K4, khe that K4-ong 39,5 cm: hong-K4 6 cm, xe 16, xe-ong 17,5) ----
Y_XE = -0.165 + 0.06 + 0.08            # tam xe: canh tren K4 + 6 cm + nua than
ONG_1518 = ong(0.60, Y_XE + 0.08 + 0.175 + 0.09)
P_1523 = (0.76, Y_XE, math.radians(174.5))  # tu the luc ket (log 15:23:37: (0,760; -0,068; 174,5 do), dich theo thuoc do that)


def _chay_1518(phan_loai, ra_xa):
    ghim(phan_loai, ra_xa)
    cach = TG.CACH
    TG.CACH = 0.16                     # Nav2 gia vach qua khe 39,5 cm (tam cach tuong >= nua than + 8 cm; that: inflation Nav2)
    try:
        io = X.NgatKhiChoNguoi(P_1523, tuong=X.TUONG + ONG_1518)
        try:
            ma = G.chay_mat(io, 've_A', 1, X.CFG3)
        except KeyboardInterrupt:
            ma = 'CHO_NGUOI'
        return ma, io
    finally:
        TG.CACH = cach
        ghim()


def test_06_phat_lai_1518_xe_thoat_qua_khe_khong_CHO_NGUOI():
    k = SL.khe(SL.diem_loc(TG.quet(X.TUONG + ONG_1518, P_1523), SL.tham_so()['gac']), SL.tham_so())
    assert abs(k['hong'] - 0.06) < 0.01, k                                     # dung so thuoc: hong-K4 6 cm
    ma, io = _chay_1518('ba_dau_hieu', True)
    assert ma == 0 and 'CHO_NGUOI' not in X.CUA(io.rows) and X.CUA(io.rows)[-1] == 'DEN_NOI', (ma, io.cau[-25:])
    assert not any('DOI_VAT' in c and 'vung do' in c for c in io.cau), [c for c in io.cau if 'DOI_VAT' in c]
    assert min(kc for _, kc in io.kc) > 0.0, min(kc for _, kc in io.kc)          # khong cham
    xs = [(x, y) for x, y in io.vet_xy if 0.40 < x < 0.75]
    assert xs and all(-0.165 < y < 0.32 - 0.09 for x, y in xs), xs[:5]       # di qua khe K4-ong (giua canh K4 va mat ong)
    print('   phat lai 15:18 (A+): DEN_NOI, %d dong tac thoat, khe nho nhat %.3f m' % (len(getattr(io, 'dt_ds', [])), min(kc for _, kc in io.kc)))


def test_07_phat_lai_1518_duong_lui_ghi_lai():
    """Duong lui (cu + false) tren cung the gioi: ghi lai ket qua (bao Thien), khong ep."""
    ma, io = _chay_1518('cu', False)
    dv = sum('DOI_VAT' in c for c in io.cau)
    print('   phat lai 15:18 (duong lui cu/false): ket qua %s, %d dong DOI_VAT, CSV %s' % (ma, dv, X.CUA(io.rows)[-3:]))


# ---- duong lui: cu + false cho CUNG quyet dinh voi 629a17b tren cung du lieu ----
_KICH = r'''
import json, math, random, sys
sys.path.insert(0, sys.argv[1])
import hanh_vi as HV
if 'phan_loai_cum' in HV.MAC_DINH:
    HV._nap()['tq'].update(phan_loai_cum='cu', thoat_ra_xa=False)
import gac_vat_la as GV, sep_lidar as SL, the_gioi_gia as TG
BD = TG.doc_ban_do(); W = TG.canh_tuong(BD)
ban = GV.BanDo(BD['w'], BD['h'], BD['res'], BD['ox'], BD['oy'], BD['data'])
t, gt = SL.tham_so(), GV.tham_so()
rd = random.Random(1011)
out = []
for n in range(40):
    pose = (rd.uniform(0.0, 1.6), rd.uniform(-0.25, 0.80), rd.uniform(-math.pi, math.pi))
    vat = [d for _ in range(rd.randint(0, 2)) for d in TG.doan_hop(pose[0] + rd.uniform(-0.4, 0.4), pose[1] + rd.uniform(-0.4, 0.4), rd.uniform(0.03, 0.2))]
    q = TG.quet(W + vat, pose)
    ds = GV.danh_gia(q, GV.bien_doi(q, pose), ban, gt)
    pl = SL.diem_loc(q, gt)
    dt, c, bo = SL.chon_dong_tac(pl, t, rd.uniform(-math.pi, math.pi))
    out.append(dict(ds=[[d['vung'], d['loai'], d['lidar'], d['bd'], d['lech'], d['n'], round(d['kc'], 9), d['do']] for d in ds],
                    mem=[GV.dong_mem(d) for d in ds], do=None if GV.do_vat(ds) is None else GV.dong_mem(GV.do_vat(ds)),
                    khe={k: round(v, 9) if v != math.inf else 'inf' for k, v in SL.khe(pl, t).items()},
                    cham=[[SL.ten_dong_tac(x), SL.cham(pl, x, t)['kha_thi'], SL.cham(pl, x, t)['ly']] for x in SL.DONG_TAC],
                    chon=None if dt is None else SL.ten_dong_tac(dt), bo=bo, thieu=round(SL.thieu(SL.khe(pl, t), t), 9),
                    huy=SL.ly_do_huy(SL.tom_tat(ds), SL.tom_tat(ds[1:]), t)))
print(json.dumps(out, sort_keys=True, default=lambda o: o.item() if hasattr(o, 'item') else str(o)))
'''


def test_08_duong_lui_cu_false_cung_quyet_dinh_629a17b():
    """40 canh ngau nhien (seed co dinh) tren ban do + hop: danh_gia, MEM, do_vat, khe, cham moi dong tac, chon_dong_tac, thieu,
    ly_do_huy: ma 629a17b (git archive) va ma moi voi phan_loai_cum cu + thoat_ra_xa false => JSON GIONG HET."""
    tam = tempfile.mkdtemp()
    goc = os.path.dirname(DAY)
    subprocess.run('git -C %s archive 629a17b xe_scripts | tar -x -C %s' % (goc, tam), shell=True, check=True)
    cu = os.path.join(tam, 'xe_scripts')
    env = dict(os.environ, HANH_VI_YAML=os.path.join(cu, 'hanh_vi_xe.yaml'))
    a = subprocess.run([sys.executable, '-c', _KICH, cu], capture_output=True, text=True, env=env, cwd=cu)
    env['HANH_VI_YAML'] = os.path.join(DAY, 'hanh_vi_xe.yaml')
    b = subprocess.run([sys.executable, '-c', _KICH, DAY], capture_output=True, text=True, env=env, cwd=DAY)
    assert a.returncode == 0 and b.returncode == 0, (a.stderr[-500:], b.stderr[-500:])
    ja, jb = json.loads(a.stdout), json.loads(b.stdout)
    assert len(ja) == 40 and ja == jb, next(((i, x, y) for i, (x, y) in enumerate(zip(ja, jb)) if x != y), None)
    assert sum(1 for x in ja if x['do']) >= 1 and sum(1 for x in ja if x['chon']) >= 1        # canh co ca vung do lan dong tac


# ==== 10/11 A++ (16:27-16:30: goc K4 hop xop khop nham cung "ong" le => CHO_VAT 30 s x 3 => CHO_NGUOI) ====
import random  # noqa: E402

A3 = ('ong_kiem_tam', 'ong_on_dinh', 'mui_tuong_khong_doi')


def ghim3(gt):
    HV._nap()['tq'].update({k: gt for k in A3})


def nhieu(q, rd, a=0.005):
    """Nhieu LiDAR +-a m tren tam do (theo tia)."""
    out = []
    for p in q:
        if p is None:
            out.append(None)
            continue
        r, g = math.hypot(p[0] - 0.025, p[1]), math.atan2(p[1], p[0] - 0.025)
        r += rd.uniform(-a, a)
        out.append((0.025 + r * math.cos(g), r * math.sin(g)))
    return out


def k4_tron(r=0.06, n=12, tuong=None):
    """Goc K4 (0,734; -0,165) bo tron r (hop xop): canh tren + canh phai rut ngan r, them cung 90 do."""
    W2 = []
    for a, b in (tuong or X.TUONG):
        A, B = tuple(round(v, 3) for v in a), tuple(round(v, 3) for v in b)
        if (A, B) == ((0.384, -0.165), (0.734, -0.165)):
            W2.append((a, (0.734 - r, -0.165)))
        elif (A, B) == ((0.734, -0.265), (0.734, -0.165)):
            W2.append((a, (0.734, -0.165 - r)))
        else:
            W2.append((a, b))
    cx, cy = 0.734 - r, -0.165 - r
    p = [(cx + r * math.cos(math.pi / 2 * k / n), cy + r * math.sin(math.pi / 2 * k / n)) for k in range(n + 1)]
    return W2 + [(p[k], p[k + 1]) for k in range(n)]


K4_TRON = k4_tron()
TU_THE_K4 = ((0.85, -0.06, math.radians(-163)), (0.76, -0.07, math.radians(172)), (0.76, -0.07, math.radians(-172)),
             (0.80, -0.03, math.radians(-150)))


def lui_toi_mui(doan, pose, mui=0.095):
    """Lui xe doc huong toi khi hanh lang truoc mui (moi diem) cach >= mui (dung so thuoc: mui-goc K4 0,09-0,10)."""
    x, y, th = pose
    for i in range(200):
        k = SL.khe(SL.diem_loc(TG.quet(doan, (x, y, th)), GV.tham_so()), SL.tham_so())
        tr = min(k['hl'], k['truoc'])                   # mui: hanh lang than hoac cheo truoc mui (goc K4 lech mot ben)
        if tr >= mui - 1e-9 and k['nho'] > 0.0:
            return (x, y, th), tr
        x, y = x - 0.005 * math.cos(th), y - 0.005 * math.sin(th)
    raise AssertionError('khong lui duoc toi mui %.2f' % mui)


def nhan_qua_38_quet(doan, pose, seed=1627):
    rd, th, dem = random.Random(seed), GV.TheoDoiOng(), {}
    q0, T = TG.quet(doan, pose), GV.tham_so()
    for i in range(38):
        q = nhieu(q0, rd)
        for d in GV.danh_gia(q, GV.bien_doi(q, pose), BAN, T, th, i * 0.1):
            if d['vung'] == 'truoc':
                dem[d['nhan']] = dem.get(d['nhan'], 0) + 1
    return dem


def test_10_goc_K4_hop_xop_38_quet_khong_bao_gio_ong():
    """A++ 1a+1b: goc K4 bo tron (hop xop), mui cach 0,09-0,10, nhieu +-5 mm, 38 lan quet: KHONG BAO GIO "ong". Duong lui (ba khoa
    false = 50e4146) tai hien loi: co lan "ong" (bao Thien)."""
    try:
        ghim3(False)
        cu = {}
        for p in TU_THE_K4:
            pm, _ = lui_toi_mui(K4_TRON, p)
            cu[p] = nhan_qua_38_quet(K4_TRON, pm).get('ong', 0)
        ghim3(True)
        for p in TU_THE_K4:
            pm, hl = lui_toi_mui(K4_TRON, p)
            dem = nhan_qua_38_quet(K4_TRON, pm)
            assert 0.09 - 1e-6 <= hl <= 0.10 + 1e-6 and dem.get('ong', 0) == 0, (p, hl, dem)
        print('   duong lui (50e4146) cung tu the: so lan "ong" / 38 = %s' % [cu[p] for p in TU_THE_K4])
        assert sum(cu.values()) > 0                                              # tai hien duoc loi 16:27 o duong lui
    finally:
        ghim3(True)


def test_11_ong_that_van_la_ong_voi_nhieu_5mm():
    """A++ 1c: ong giua san / sat tuong / sat K4 0-1-3 cm / sat H0, nhieu +-5 mm, 10 lan quet co theo doi: lan cuoi la "ong" (on dinh)."""
    ca = [('giua san', ong(1.00, 0.10), (0.72, 0.10, 0.0), (1.00, 0.10)),
          ('sat tuong', ong(1.30, -0.365 + 0.09 + 0.005), (0.95, -0.12, -0.3), (1.30, -0.365 + 0.095))]
    for g in (0.0, 0.01, 0.03):
        ca.append(('sat K4 %.2f' % g, ong(0.60, -0.165 + 0.09 + g), (0.95, -0.02, math.pi), (0.60, -0.165 + 0.09 + g)))
    ca.append(('sat H0', ong(0.42, 0.335 - 0.09 - 0.005), (0.20, 0.15, 0.4), (0.42, 0.335 - 0.095)))
    for ten, o, pose, tam in ca:
        rd, th, T = random.Random(11), GV.TheoDoiOng(), GV.tham_so()
        q0 = TG.quet(X.TUONG + o, pose)
        for i in range(10):
            q = nhieu(q0, rd)
            m = GV.bien_doi(q, pose)
            ds = GV.danh_gia(q, m, BAN, T, th, i * 0.1)
        dd = [d for d in ds if any(m[k] is not None and math.dist(m[k], tam) <= 0.095 for k in d['ks'])]   # diem tren mat ong (r 0,09 + nhieu)
        dinh = ten in ('sat tuong', 'sat K4 0.00')       # ong DINH lien tuong: cung cum voi tuong => co the 'ngoai ban do' (van VAT chac)
        assert dd and all(d['loai'] == 'VAT' and d['chac'] for d in dd) and (dinh or any(d['nhan'] == 'ong' for d in dd)), (
            ten, [GV.dong_mem(d) for d in dd])
        print('   %-12s %s' % (ten, GV.dong_mem(dd[0])))


class IONhieu(X.NgatKhiChoNguoi):
    """IOThe + nhieu LiDAR +-5 mm moi lan quet."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.rd = random.Random(1628)
        goc = self.gac_fn
        self.gac_fn = lambda t: nhieu(goc(t), self.rd)


def test_12_ve_A_tu_goc_K4_hop_xop_khong_CHO_VAT_toi_DEN_NOI():
    """A++ 2 + 3: tu (0,85; -0,06; -163 do) lui toi mui 0,095 truoc goc K4 bo tron, nhieu +-5 mm: khong "ong", khong CHO_VAT / DOI_VAT
    30 s, ve_A => DEN_NOI (khong CHO_NGUOI)."""
    p, hl = lui_toi_mui(K4_TRON, TU_THE_K4[0])
    cach = TG.CACH
    TG.CACH = 0.16
    try:
        io = IONhieu(p, tuong=K4_TRON)
        try:
            ma = G.chay_mat(io, 've_A', 1, X.CFG3)
        except KeyboardInterrupt:
            ma = 'CHO_NGUOI'
    finally:
        TG.CACH = cach
    assert ma == 0 and X.CUA(io.rows)[-1] == 'DEN_NOI', (ma, io.cau[-15:])
    assert not any('DOI_VAT' in c or c.startswith('CHO_VAT') or '(ong:' in c for c in io.cau), [c for c in io.cau if 'DOI_VAT' in c or '(ong' in c][:5]
    print('   ve_A tu goc K4 (mui %.3f): DEN_NOI, khong DOI_VAT / CHO_VAT / ong' % hl)


def test_13_ong_that_truoc_mui_009_giua_san_van_CHO_VAT_30s():
    """A++: ong that giua hanh lang duoi, truoc mui 0,09 (nhieu +-5 mm): van nhac doi (DOI_VAT ... vat chac, CHO_VAT n/30) nhu cu."""
    o = ong(0.0 + 0.145 + 0.09 + 0.09 + 0.40, 0.02)
    p = (0.40, 0.02, 0.0)
    io = IONhieu(p, vat=[(o, 0.0, 1e9)])
    cach = TG.CACH
    TG.CACH = 0.16
    try:
        try:
            G.chay_mat(io, 'di_B', 1, X.CFG3)
        except KeyboardInterrupt:
            pass
    finally:
        TG.CACH = cach
    i = next((k for k, c in enumerate(io.cau) if c.startswith('SEP_LIDAR: DOI_VAT truoc vat chac')), None)
    assert i is not None and any(c.startswith('CHO_VAT: 5/30') for c in io.cau[i:]), [c for c in io.cau if 'DOI_VAT' in c or 'MEM: truoc' in c][:6]
    assert any('(ong: cung r=0.09' in c and 'on dinh' in c for c in io.cau), [c for c in io.cau if c.startswith('MEM: truoc')][:4]


_KICH3 = r'''
import json, math, random, sys
sys.path.insert(0, sys.argv[1])
import hanh_vi as HV
if 'ong_on_dinh' in HV.MAC_DINH:
    HV._nap()['tq'].update(ong_kiem_tam=False, ong_on_dinh=False, mui_tuong_khong_doi=False)
import gac_vat_la as GV, sep_lidar as SL, the_gioi_gia as TG
sys.path.insert(0, sys.argv[2])
import test_phan_loai_cum_kich as KI
BD = TG.doc_ban_do()
ban = GV.BanDo(BD['w'], BD['h'], BD['res'], BD['ox'], BD['oy'], BD['data'])
t, gt = SL.tham_so(), GV.tham_so()
rd = random.Random(2026)
out = []
for n in range(40):
    W = KI.the_gioi(n)
    pose = (rd.uniform(0.0, 1.6), rd.uniform(-0.25, 0.80), rd.uniform(-math.pi, math.pi))
    q0 = TG.quet(W, pose)
    q = [None if p is None else (p[0] + rd.uniform(-0.003, 0.003), p[1] + rd.uniform(-0.003, 0.003)) for p in q0]
    ds = GV.danh_gia(q, GV.bien_doi(q, pose), ban, gt)
    pl, fl = SL.diem_loc_nhan(q, gt, ds)
    dt, c, bo = SL.chon_dong_tac(pl, t, rd.uniform(-math.pi, math.pi), None, fl)
    out.append(dict(mem=[GV.dong_mem(d) for d in ds], do=None if GV.do_vat(ds) is None else GV.dong_mem(GV.do_vat(ds)),
                    chac=[d.get('chac') for d in ds], chon=None if dt is None else SL.ten_dong_tac(dt), bo=bo,
                    khe={k: round(v, 9) if v != math.inf else 'inf' for k, v in SL.khe(pl, t, tuong=fl).items()}))
print(json.dumps(out, sort_keys=True, default=lambda o: o.item() if hasattr(o, 'item') else str(o)))
'''


def test_14_duong_lui_ba_khoa_false_cung_quyet_dinh_50e4146():
    """A++ duong lui: ong_kiem_tam / ong_on_dinh / mui_tuong_khong_doi = false => ma moi cho CUNG ket qua voi 50e4146 (ba dau hieu,
    thoat ra xa) tren 40 canh: ban do / + goc K4 bo tron / + ong, nhieu 3 mm (MEM, do_vat, vat chac, chon dong tac, khe)."""
    tam = tempfile.mkdtemp()
    goc = os.path.dirname(DAY)
    subprocess.run('git -C %s archive 50e4146 xe_scripts | tar -x -C %s' % (goc, tam), shell=True, check=True)
    kich = tempfile.mkdtemp()
    with open(os.path.join(kich, 'test_phan_loai_cum_kich.py'), 'w') as f:
        f.write('import math\nimport the_gioi_gia as TG\nW = TG.canh_tuong(TG.doc_ban_do())\n'
                'def ong(cx, cy, r=0.09, n=24):\n    p = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]\n'
                '    return [(p[k], p[(k + 1) %% n]) for k in range(n)]\n'
                'K4 = %r\n'
                'def the_gioi(n):\n    return [W, K4, W + ong(1.0, 0.1), K4 + ong(0.6, -0.065)][n %% 4]\n' % (K4_TRON,))
    cu = os.path.join(tam, 'xe_scripts')
    env = dict(os.environ, HANH_VI_YAML=os.path.join(cu, 'hanh_vi_xe.yaml'))
    a = subprocess.run([sys.executable, '-c', _KICH3, cu, kich], capture_output=True, text=True, env=env, cwd=cu)
    env['HANH_VI_YAML'] = os.path.join(DAY, 'hanh_vi_xe.yaml')
    b = subprocess.run([sys.executable, '-c', _KICH3, DAY, kich], capture_output=True, text=True, env=env, cwd=DAY)
    assert a.returncode == 0 and b.returncode == 0, (a.stderr[-600:], b.stderr[-600:])
    ja, jb = json.loads(a.stdout), json.loads(b.stdout)
    assert len(ja) == 40 and ja == jb, next(((i, x, y) for i, (x, y) in enumerate(zip(ja, jb)) if x != y), None)
    assert any('(ong:' in m for x in ja for m in x['mem'])


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
