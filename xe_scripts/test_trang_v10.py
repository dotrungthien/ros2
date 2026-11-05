#!/usr/bin/env python3
"""test_trang_v10.py (viec 111, 06/11): test trang /v10 (trang_v10.py + duong /v10 cua cau_noi_xe.py). Khong can ROS.
Cau noi that (cong ngau nhien), HOME gia, bat_xe.sh / chuyen.sh gia; tep that cua xe lay tu du_lieu_0511_toi (chi doc).
Chay: python3 -m unittest test_trang_v10   (trong xe_scripts; chay RIENG voi test_cau_noi: moi tep mot HOME gia)
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
DL = os.path.join(HERE, 'du_lieu_0511_toi')
TAM = tempfile.mkdtemp(prefix='test_trang_v10_')
os.environ['HOME'] = TAM
for k in [k for k in os.environ if k.startswith('CAU_NOI_')]:
    del os.environ[k]
os.environ['CAU_NOI_CHINH'] = 'gia_chinh_a gia_chinh_b'

BAT_XE = r'''#!/usr/bin/env bash
if [ "$1" = tat ]; then echo "DA_DON 0 tep"; exit 0; fi
echo "Con quay da lay so 0"
sleep "${GIA_BAT_S:-0.3}"
echo "SAN SANG. Con 400 giay de gui muc tieu (han chot 99:99:99)."
exit 0
'''
CHUYEN = r'''#!/usr/bin/env bash
trap 'echo "HUY_DICH_KHI_NGAT gia"; exit 130' INT TERM
echo "-- tuyen 1, lan 1/3: 30 diem tu (0.00; 0.00) toi (1.60; 0.82) [$1]"
[ -n "$GIA_IN" ] && echo "$GIA_IN"
sleep "${GIA_CHUYEN_S:-30}" &
wait
echo "${GIA_KET:-DEN_NOI: $1 xong theo tuyen 1 (cach dich 0.05 m).}"
exit "${GIA_MA:-0}"
'''
for ten, nd in (('bat_xe.sh', BAT_XE), ('chuyen.sh', CHUYEN)):
    with open(os.path.join(TAM, ten), 'w') as f:
        f.write(nd)
# M2: tep cua xe nhu tren Pi (slam yaml tro /home/project1/ -> doi sang du_lieu_0111, nhu gia_lap/bat_gia.sh)
with open(os.path.join(HERE, 'du_lieu_0111', 'slam_dinh_vi_xuong.yaml')) as f:
    SLAM_TXT = f.read().replace('/home/project1/', os.path.join(HERE, 'du_lieu_0111') + '/')
with open(os.path.join(TAM, 'slam_dinh_vi_xuong.yaml'), 'w') as f:
    f.write(SLAM_TXT)
os.environ['NAV_PARAMS'] = os.path.join(HERE, 'nav2_params_cho.yaml')
os.environ['CAU_NOI_THU_MUC_XE'] = HERE
os.environ['CAU_NOI_TWIST_MUX'] = os.path.join(HERE, '..', 'dev_ws', 'src', 'my_bot', 'config', 'twist_mux.yaml')
sys.path.insert(0, HERE)
import cau_noi_xe as C  # noqa: E402
import trang_v10 as V  # noqa: E402

LINK = {'on': False}
_quet_that = C.quet_tien_trinh
C.quet_tien_trinh = lambda: set(C.CHINH) if LINK['on'] else set()


def http(port, method, path, body=None, pin=None):
    data = None if body is None else json.dumps(body).encode()
    rq = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), data=data, method=method)
    if body is not None:
        rq.add_header('Content-Type', 'application/json')
    if pin is not None:
        rq.add_header('X-Pin', pin)
    try:
        with urllib.request.urlopen(rq, timeout=10) as r:
            return r.status, r.headers.get('Content-Type', ''), r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get('Content-Type', ''), e.read()


def js(port, method, path, body=None, pin=None):
    st, _, b = http(port, method, path, body, pin)
    return st, json.loads(b.decode())


def cho(dk, giay=10.0):
    t = time.monotonic() + giay
    while time.monotonic() < t:
        if dk():
            return True
        time.sleep(0.05)
    return False


def ros(t, msg):
    return '[INFO] [%.6f] [lai_xe_lidar]: %s' % (t, msg)


class Nen(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = C.ThreadingHTTPServer(('127.0.0.1', 0), C.H)
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def setUp(self):
        cho(lambda: C.trang_thai_nut()['viec'] == 'rong', 15)
        LINK['on'] = False
        for t in ('log_phu_lai.txt', 'sep_thoat', 'sep_nhat_ky.csv', '.cau_noi_pin'):
            try:
                os.remove(os.path.join(TAM, t))
            except OSError:
                pass
        with C.LOCK:
            C.CHUYEN.update(ma=None, ket_qua='', dong='', huong='')
            C.TAT.update(ma=None, dong='')
            C.JOB.update(ket_qua=None, chi_tiet='')
        os.environ.pop('GIA_IN', None)
        os.environ.pop('GIA_KET', None)
        os.environ.pop('GIA_MA', None)
        os.environ.pop('GIA_CHUYEN_S', None)

    def ma(self):
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertEqual(st, 200)
        return j['ma']


# ---------------- M1 ----------------
def link(l='on', chi=''):
    return {'link': l, 'chi_tiet': chi}


def nut(chuyen_chay=False, ket_qua='', bat=False, tat=False, tat_ma=None):
    return {'tat': {'chay': tat, 'ma': tat_ma, 'dong': ''}, 'bat': {'chay': bat},
            'chuyen': {'chay': chuyen_chay, 'ket_qua': ket_qua, 'dong': '', 'giay': 1.0}}


SEP0 = {'tu_the': None, 'lenh': None, 'pose': None, 'dung_va_bao': False}


class M1HamThuan(unittest.TestCase):
    def test_suy_trang_thai_moi_nhanh(self):
        f = V.suy_trang_thai
        now = 1000.0
        self.assertEqual(f(link('off'), nut(), SEP0, None, None, None, now)[0], 'TT_KHONG_KET_NOI')
        self.assertEqual(f(link('off'), nut(tat_ma=0), SEP0, None, None, None, now)[0], 'TT_DA_NGAT')
        self.assertEqual(f(link('fail', 'thieu: x'), nut(), SEP0, None, None, None, now), ('TT_KHONG_KET_NOI', 'thieu: x'))
        self.assertEqual(f(link('connecting'), nut(), SEP0, None, None, None, now)[0], 'TT_DANG_KET_NOI')
        self.assertEqual(f(link('off'), nut(bat=True), SEP0, None, None, None, now)[0], 'TT_DANG_KET_NOI')
        self.assertEqual(f(link('on'), nut(tat=True), SEP0, None, None, None, now)[0], 'TT_DANG_NGAT')
        self.assertEqual(f(link('on'), nut(), SEP0, None, None, None, now)[0], 'TT_SAN_SANG')
        self.assertEqual(f(link('on'), nut(ket_qua='NGAT'), SEP0, None, None, None, now)[0], 'TT_SAN_SANG')
        self.assertEqual(f(link('on'), nut(ket_qua='DEN_NOI'), SEP0, None, None, None, now)[0], 'TT_DA_DEN')
        self.assertEqual(f(link('on'), nut(ket_qua='HET_DUONG'), SEP0, None, None, None, now)[0], 'TT_CAN_GIUP')
        self.assertEqual(f(link('on'), nut(ket_qua='DUNG_SEP'), SEP0, None, None, None, now)[0], 'TT_CAN_GIUP')
        n = nut(chuyen_chay=True)
        self.assertEqual(f(link('on'), n, SEP0, None, None, None, now)[0], 'TT_DANG_DI')
        self.assertEqual(f(link('on'), n, SEP0, ('RA_LUI', '', now - 1), None, None, now)[0], 'TT_DANG_THOAT')
        self.assertEqual(f(link('on'), n, SEP0, ('RA_LUI', '', now - 9), None, None, now)[0], 'TT_DANG_DI')   # log cu
        self.assertEqual(f(link('on'), n, SEP0, ('XOAY_P', '', now), None, None, now)[0], 'TT_DANG_XOAY')
        self.assertEqual(f(link('on'), n, SEP0, ('CHO_3', '', now), None, None, now)[0], 'TT_DANG_CHO')
        self.assertEqual(f(link('on'), n, SEP0, ('XANH_VANG', '', now), None, None, now)[0], 'TT_DANG_DI')
        self.assertEqual(f(link('on'), n, SEP0, None, None, 'RA_TIEN', now)[0], 'TT_DANG_THOAT')
        self.assertEqual(f(link('on'), n, SEP0, None, 'CHO', None, now)[0], 'TT_DANG_CHO')
        self.assertEqual(f(link('on'), n, SEP0, None, 'THOAT', None, now)[0], 'TT_DANG_THOAT')
        self.assertEqual(f(link('on'), n, dict(SEP0, tu_the=('VAT_LA_TRUOC', '', '')), None, None, None, now)[0], 'TT_DANG_CHO')
        self.assertEqual(f(link('on'), n, dict(SEP0, dung_va_bao=True), ('RA_LUI', '', now), None, None, now)[0], 'TT_CAN_GIUP')

    def test_doc_log_that(self):
        with open(os.path.join(DL, 'log_phu_lai.txt')) as fh:
            pl = V.phu_lai_cuoi(fh.read().splitlines())
        self.assertEqual(pl[0], 'KHONG_CO_DICH')
        self.assertGreater(pl[2], 1.79e9)
        s = V.sep_cuoi(V.doc_duoi(os.path.join(DL, 'sep_nhat_ky.csv'), 10 ** 8), '9999')
        self.assertTrue(s['tu_the'] and s['lenh'] and s['pose'])
        self.assertFalse(s['dung_va_bao'])
        s2 = V.sep_cuoi(['1,2026-11-05 21:00:00,TU_THE,CHO,,,,,0.1,0.2,30', '2,2026-11-05 21:00:01,LENH,DUNG,,,DUNG_VA_BAO x,,0.1,0.2,30'], '2026-11-05 20:59:59')
        self.assertEqual((s2['tu_the'][0], s2['lenh'][0], s2['pose'][:3], s2['dung_va_bao']), ('CHO', 'DUNG', (0.1, 0.2, 30.0), True))
        self.assertFalse(V.sep_cuoi(s2_dong := ['2,2026-11-05 21:00:01,LENH,DUNG,,,DUNG_VA_BAO x,,,,'], '2026-11-05 21:00:02')['dung_va_bao'])
        self.assertIsNone(V.sep_cuoi(s2_dong, '')['pose'])
        with open(os.path.join(DL, 'log_nut_chuyen.txt')) as fh:
            self.assertIn(V.dong_chuyen_cuoi(fh.read().splitlines()), ('DI', 'CHO', 'THOAT', None))
        self.assertEqual(V.dong_chuyen_cuoi(['-- tuyen 1, lan 1/3', '   lop 2: sep bao RA_TIEN: cho phu lai tien ra, toi da 12 s.']), 'THOAT')
        self.assertEqual(V.dong_chuyen_cuoi(['   lop 2: sep bao RA_LUI (vat sat mui): ...', '   lop 2: RA_XONG sau 2.0 s.']), 'DI')
        self.assertEqual(V.dong_chuyen_cuoi(['   khong den duoc (ABORTED, cach dich 1.45 m).']), 'CHO')
        self.assertEqual(V.thoat_moi('100.00 RA_LUI 3 7', 101.0), 'RA_LUI')
        self.assertIsNone(V.thoat_moi('100.00 RA_LUI 3 7', 105.0))
        self.assertIsNone(V.thoat_moi('', 105.0))

    def test_han_js_tu_trang_nut(self):
        s = V.han_js(C)
        self.assertIn('function han(s){', s)
        self.assertIn('HAN=HAN_V10.concat(HAN);', s)
        self.assertLess(s.index('HAN=HAN_V10.concat'), s.index('var HAN_TU='))
        self.assertEqual(len(V.HAN_V10), len(V.MA) + 2)          # + TT_XEM_TUYEN, TT_LAI_TAY (chi o trang)
        for m in V.MA:
            self.assertIn(m, dict(V.HAN_V10))
        # ma TT_* khong lan vao ma cu: \\b coi '_' la chu, nen 'TT_DANG_CHO' khong khop 'CHO'
        self.assertTrue(all(k.startswith('TT_') for k, _ in V.HAN_V10))


class M1Http(Nen):
    def test_trang_v10(self):
        st, ct, b = http(self.port, 'GET', '/v10')
        self.assertEqual((st, ct.split(';')[0]), (200, 'text/html'))
        t = b.decode()
        self.assertNotIn('/*HAN_JS*/', t)
        self.assertIn('var HAN_V10=', t)
        self.assertNotIn('http://', t.replace('http://www.w3.org', ''))   # khong tai gi ngoai mang
        self.assertNotIn('https://', t)

    def test_trang_thai_theo_viec(self):
        self.assertEqual(self.ma(), 'TT_KHONG_KET_NOI')
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        for k in ('ma', 'ly_do', 'link', 'thieu', 'chi_tiet', 'han_s', 'viec', 'chuyen', 'bat', 'tat', 'dung', 'pin'):
            self.assertIn(k, j)
        LINK['on'] = True
        self.assertEqual(self.ma(), 'TT_SAN_SANG')
        self.assertTrue(C.bat_dau_chuyen('di_B'))
        self.assertEqual(self.ma(), 'TT_DANG_DI')
        with open(os.path.join(TAM, 'log_phu_lai.txt'), 'w') as f:
            f.write(ros(time.time(), 'XANH_VANG -> RA_LUI (vung_sau) | SAU=0.05') + '\n')
        self.assertEqual(self.ma(), 'TT_DANG_THOAT')
        with open(os.path.join(TAM, 'log_phu_lai.txt'), 'a') as f:
            f.write(ros(time.time(), 'RA_LUI -> XOAY_P () | x') + '\n')
        self.assertEqual(self.ma(), 'TT_DANG_XOAY')
        C.dung_chuyen()
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        self.assertEqual(self.ma(), 'TT_SAN_SANG')          # NGAT -> san sang
        os.environ['GIA_CHUYEN_S'] = '0.1'
        self.assertTrue(C.bat_dau_chuyen('di_B'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        self.assertEqual(self.ma(), 'TT_DA_DEN')
        os.environ.update(GIA_KET='HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.', GIA_MA='1')
        self.assertTrue(C.bat_dau_chuyen('ve_A'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertEqual((j['ma'], j['ly_do']), ('TT_CAN_GIUP', 'HET_DUONG'))
        LINK['on'] = False
        self.assertTrue(C.bat_dau_tat())
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['tat']['chay']))
        self.assertEqual(self.ma(), 'TT_DA_NGAT')

    def test_loi_v10_khong_lam_hong_nut(self):
        cu = V.trang_thai
        V.trang_thai = lambda C: 1 / 0
        try:
            st, j = js(self.port, 'GET', '/v10/trang_thai')
            self.assertEqual(st, 500)
            self.assertIn('trang_v10', j['loi'])
            self.assertEqual(http(self.port, 'GET', '/nut')[0], 200)
            self.assertEqual(js(self.port, 'GET', '/api/nut/trang_thai')[0], 200)
        finally:
            V.trang_thai = cu
        self.assertEqual(js(self.port, 'GET', '/v10/khong_co')[0], 404)
        self.assertEqual(js(self.port, 'POST', '/v10/khong_co')[0], 404)



# ---------------- M4 ----------------
class M4Lenh(Nen):
    def test_di_B_ve_A_giong_nut(self):
        st, j = js(self.port, 'POST', '/v10/api/di', {'dich': 'B'})
        self.assertEqual((st, j), (202, {'da_bat_dau': True, 'viec': 'chuyen'}))
        self.assertEqual(C.trang_thai_nut()['chuyen']['huong'], 'di_B')
        with open(C.CFG['log_chuyen']) as f:
            dau = f.readline()
        self.assertIn(': di_B (BANG=day) ==', dau)                 # cung dong dau log nhu /nut (bat_dau_chuyen)
        self.assertEqual(js(self.port, 'POST', '/v10/api/di', {'dich': 'A'}), (409, {'da_bat_dau': False, 'viec': 'chuyen'}))
        self.assertEqual(js(self.port, 'POST', '/api/nut/ve_A'), (409, {'da_bat_dau': False, 'viec': 'chuyen'}))
        self.assertEqual(js(self.port, 'POST', '/v10/api/bat')[0], 409)
        self.assertEqual(js(self.port, 'POST', '/v10/api/tat')[0], 409)
        time.sleep(0.3)
        st, j = js(self.port, 'POST', '/v10/api/dung')
        self.assertEqual((st, j['ket_qua']), (200, 'DA_DUNG'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        self.assertEqual(C.trang_thai_nut()['chuyen']['ket_qua'], 'NGAT')
        st, j = js(self.port, 'POST', '/v10/api/di', {'dich': 'A'})
        self.assertEqual(st, 202)
        self.assertEqual(C.trang_thai_nut()['chuyen']['huong'], 've_A')
        js(self.port, 'POST', '/v10/api/dung')
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))

    def test_dich_sai_va_khong_co_chuyen(self):
        self.assertEqual(js(self.port, 'POST', '/v10/api/di', {'dich': 'C'})[0], 400)
        self.assertEqual(js(self.port, 'POST', '/v10/api/di', {})[0], 400)
        self.assertEqual(js(self.port, 'POST', '/v10/api/di')[0], 400)
        self.assertEqual(js(self.port, 'POST', '/v10/api/dung'), js(self.port, 'POST', '/api/nut/dung'))
        self.assertEqual(js(self.port, 'POST', '/v10/api/khong_co')[0], 404)
        self.assertFalse(C.trang_thai_nut()['chuyen']['chay'])

    def test_bat_tat(self):
        st, j = js(self.port, 'POST', '/v10/api/bat')
        self.assertEqual((st, j['viec']), (202, 'bat'))
        self.assertEqual(self.ma(), 'TT_DANG_KET_NOI')
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['bat']['chay']))
        self.assertTrue(C.trang_thai_nut()['bat']['dong'].startswith('SAN SANG'))
        st, j = js(self.port, 'POST', '/v10/api/tat')
        self.assertEqual((st, j['viec']), (202, 'tat'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['tat']['chay']))
        self.assertEqual(self.ma(), 'TT_DA_NGAT')

    def test_pin(self):
        with open(C.CFG['pin_file'], 'w') as f:
            f.write('2468\n')
        for ten, than in (('di', {'dich': 'B'}), ('dung', None), ('bat', None), ('tat', None)):
            self.assertEqual(js(self.port, 'POST', '/v10/api/' + ten, than)[0], 401, ten)
        self.assertFalse(C.trang_thai_nut()['chuyen']['chay'])
        self.assertEqual(js(self.port, 'POST', '/v10/api/dung', pin='2468')[0], 200)
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertTrue(j['pin'])

    def test_trang_co_nut_va_cau_hoi_nhu_nut(self):
        t = http(self.port, 'GET', '/v10')[2].decode()
        for cau in ('B로 출발? 모터 ON, 손은 스위치 옆에.', 'A로 복귀? 모터 ON, 손은 스위치 옆에.', '차량의 ROS 전체를 종료할까요?',
                    '전체를 껐다가 다시 시작할까요 (B단계)? 차량은 A선, 모터 OFF.'):
            self.assertIn(cau, t)
            self.assertIn(cau, C.TRANG_NUT)
        for x in ("lenh('bat')", "lenh('tat')", "lenh('dung')", "lenh('di')"):
            self.assertIn(x, t)



# ---------------- M2 ----------------
class M2BanDo(Nen):
    def setUp(self):
        super().setUp()
        V._CACHE_BAN.update(khoa=None, kq=None)

    def test_ban_do_tu_tep_cua_xe(self):
        st, j = js(self.port, 'GET', '/v10/ban_do')
        self.assertEqual(st, 200, j)
        self.assertIsNone(j['loi'])
        self.assertEqual((j['w'], j['h'], j['res'], j['ox'], j['oy']), (52, 32, 0.05, -0.566, -0.415))
        self.assertEqual(len(j['vung']), 52 * 32)
        self.assertEqual(j['footprint'], [[0.145, 0.08], [0.145, -0.08], [-0.105, -0.08], [-0.105, 0.08]])
        self.assertEqual((j['inflation_radius'], j['cost_scaling_factor'], j['r_noi_tiep']), (0.2, 3.0, 0.09))
        self.assertTrue(j['nguon']['ban_do'].endswith('du_lieu_0111/map_xuong_v3.yaml'))   # map_xuong_v3_pg -> map_xuong_v3.yaml
        self.assertEqual((j['A'], j['B']), ([0.0, 0.0], [1.6, 0.82]))
        # vung do = dung tap o tinh_tuyen cam (Ban.do) tren costmap dung lai; o vat = vung 0
        TT = V.nap(C, 'tinh_tuyen')
        ban = V._CACHE_BAN['ban']
        for k in range(52 * 32):
            i, jj = k % 52, k // 52
            z = j['vung'][jj * 52 + i]
            if z in '12' or z == '3':
                self.assertEqual(z == '1', ban.do((i, jj)))
        w_, h_, _, _, _, occ, _ = V.doc_ban_do(j['nguon']['ban_do'])
        self.assertEqual(j['vung'].count('0'), sum(1 for v in occ if v != 0))     # vat + chua biet
        za = j['vung'][ban.o(*TT.A)[1] * 52 + ban.o(*TT.A)[0]]
        zb = j['vung'][ban.o(*TT.B)[1] * 52 + ban.o(*TT.B)[0]]
        self.assertNotIn(zb, '01')
        self.assertNotEqual(za, '0')

    def test_doi_tep_tham_so_thi_doi_vung(self):
        _, j1 = js(self.port, 'GET', '/v10/ban_do')
        p = os.path.join(TAM, 'nav2_tam.yaml')
        with open(os.environ['NAV_PARAMS']) as f:
            t = f.read()
        with open(p, 'w') as f:
            f.write(t.replace('inflation_radius: 0.20 ', 'inflation_radius: 0.30 ').replace('footprint: "[[0.145, 0.08], [0.145, -0.08], [-0.105, -0.08], [-0.105, 0.08]]"',
                    'footprint: "[[0.145, 0.10], [0.145, -0.10], [-0.105, -0.10], [-0.105, 0.10]]"'))
        cu = os.environ['NAV_PARAMS']
        os.environ['NAV_PARAMS'] = p
        try:
            _, j2 = js(self.port, 'GET', '/v10/ban_do')
        finally:
            os.environ['NAV_PARAMS'] = cu
        self.assertEqual(j2['inflation_radius'], 0.3)
        self.assertAlmostEqual(j2['r_noi_tiep'], 0.11)
        self.assertGreater(j2['vung'].count('1'), j1['vung'].count('1'))
        self.assertEqual(j2['vung'].count('0'), j1['vung'].count('0'))

    def test_thieu_tep_bao_loi_khong_so_du_phong(self):
        cu = os.environ.pop('NAV_PARAMS')
        try:
            st, j = js(self.port, 'GET', '/v10/ban_do')
            self.assertEqual(st, 503)
            self.assertIn('NAV_PARAMS', j['loi'])
            self.assertNotIn('vung', j)
            os.makedirs(os.path.join(TAM, 'cau_noi'), exist_ok=True)
            with open(os.path.join(TAM, 'cau_noi', 'cau_noi.env'), 'w') as f:
                f.write('NAV_PARAMS=%s\n' % cu)                 # nhu bat_xe.sh: doc dong NAV_PARAMS= trong ~/cau_noi/cau_noi.env
            st, j = js(self.port, 'GET', '/v10/ban_do')
            self.assertEqual((st, j['nguon']['nav2']), (200, cu))
            os.environ['NAV_PARAMS'] = os.path.join(TAM, 'khong_co.yaml')
            V._CACHE_BAN.update(khoa=None)
            st, j = js(self.port, 'GET', '/v10/ban_do')
            self.assertEqual(st, 503)
            self.assertIn('khong thay tep tham so Nav2', j['loi'])
        finally:
            os.environ['NAV_PARAMS'] = cu
            try:
                os.remove(os.path.join(TAM, 'cau_noi', 'cau_noi.env'))
            except OSError:
                pass
        slam = os.path.join(TAM, 'slam_dinh_vi_xuong.yaml')
        with open(slam, 'w') as f:
            f.write(SLAM_TXT.replace('map_xuong_v3_pg', 'map_khong_co_pg'))
        try:
            st, j = js(self.port, 'GET', '/v10/ban_do')
            self.assertEqual(st, 503)
            self.assertIn('map_khong_co_pg', j['loi'])
            os.remove(slam)
            st, j = js(self.port, 'GET', '/v10/ban_do')
            self.assertEqual(st, 503)
            self.assertIn('slam_dinh_vi_xuong.yaml', j['loi'])
        finally:
            with open(slam, 'w') as f:
                f.write(SLAM_TXT)
        self.assertEqual(js(self.port, 'GET', '/v10/ban_do')[0], 200)

    def test_png_thuan_python_khop_pil(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest('khong co PIL')
        f = os.path.join(HERE, 'du_lieu_0111', 'map_xuong_v3.png')
        w, h, hang = V.doc_png(f)
        im = Image.open(f).convert('L')
        self.assertEqual(hang, [[im.getpixel((x, y)) for x in range(w)] for y in range(h)])
        for mode in ('RGB', 'RGBA', 'LA'):
            p = os.path.join(TAM, 'thu_%s.png' % mode)
            im.convert(mode).save(p)
            self.assertEqual(V.doc_png(p)[2], hang, mode)

    def test_vi_tri_tu_sep(self):
        with open(os.path.join(TAM, 'sep_nhat_ky.csv'), 'w') as f:
            f.write('gio_mono,gio,loai,dau_hieu,nguon_nghi,phep_kiem,ket_qua,se_lam,x,y,goc_do\n'
                    '5.00,2026-11-06 03:00:00,TU_THE,DI_THUONG,,,,,0.41,0.20,15\n5.10,2026-11-06 03:00:01,VONG,x,,,,,,,\n')
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertEqual(j['pose'], {'x': 0.41, 'y': 0.2, 'goc_do': 15.0, 'gio': '2026-11-06 03:00:00'})
        os.remove(os.path.join(TAM, 'sep_nhat_ky.csv'))
        self.assertIsNone(js(self.port, 'GET', '/v10/trang_thai')[1]['pose'])



# ---------------- M3 ----------------
def ghi_pose(x, y, g=0):
    with open(os.path.join(TAM, 'sep_nhat_ky.csv'), 'w') as f:
        f.write('gio_mono,gio,loai,dau_hieu,nguon_nghi,phep_kiem,ket_qua,se_lam,x,y,goc_do\n'
                '5.00,2026-11-06 03:00:00,TU_THE,DI_THUONG,,,,,%.2f,%.2f,%d\n' % (x, y, g))


class M3Tuyen(Nen):
    def test_tuyen_B_giong_goi_thang_tinh_tuyen(self):
        ghi_pose(0.01, 0.00)
        st, j = js(self.port, 'GET', '/v10/tuyen?dich=B')
        self.assertEqual(st, 200, j)
        TT = V.nap(C, 'tinh_tuyen')
        GD = V.nap(C, 'gui_duong')
        self.assertEqual(GD.DICH['di_B'], (TT.B, 0.0))
        ref = TT.tinh_duong(V._CACHE_BAN['ban'], (0.01, 0.0), TT.B, 1, yaw_cuoi=0.0)
        self.assertEqual(j['diem'], [[round(x, 3), round(y, 3)] for x, y, _ in ref])
        self.assertEqual((j['dich'], j['huong'], j['tuyen'], j['tu']), ('B', 'di_B', 1, [0.01, 0.0]))
        self.assertAlmostEqual(j['dai_m'], sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** .5 for a, b in zip(ref, ref[1:])), places=2)
        self.assertGreater(j['kc_do_m'], 0.0)
        self.assertLess(j['kc_do_m'], 0.30)
        self.assertEqual(j['diem'][-1], [1.6, 0.82])

    def test_tuyen_A(self):
        ghi_pose(1.60, 0.82)
        st, j = js(self.port, 'GET', '/v10/tuyen?dich=A')
        self.assertEqual((st, j['huong'], j['diem'][-1]), (200, 've_A', [0.0, 0.0]))

    def test_loi(self):
        self.assertEqual(js(self.port, 'GET', '/v10/tuyen?dich=B')[1]['loi'][:21], 'chua co vi tri xe (se')
        ghi_pose(0.01, 0.00)
        st, j = js(self.port, 'GET', '/v10/tuyen?dich=C')
        self.assertEqual(st, 422)
        self.assertIn('CHON_B_TU_DO', j['loi'])
        self.assertEqual(js(self.port, 'GET', '/v10/tuyen')[0], 422)
        ghi_pose(-3.0, -3.0)                                  # ngoai ban do
        st, j = js(self.port, 'GET', '/v10/tuyen?dich=B')
        self.assertEqual(st, 422)
        self.assertIn('ngoai costmap', j['loi'])
        self.assertFalse(V.CHON_B_TU_DO)

    def test_trang_chi_cho_chon_B_A(self):
        t = http(self.port, 'GET', '/v10')[2].decode()
        self.assertIn("fetch('/v10/tuyen?dich='+d", t)
        self.assertIn("var d=Math.hypot(x-BD.B[0],y-BD.B[1])<=R_CHAM?'B':Math.hypot(x-BD.A[0],y-BD.A[1])<=R_CHAM?'A':null;", t)



# ---------------- M5 ----------------
class M5Bang(Nen):
    def test_ham_doc_log(self):
        with open(os.path.join(DL, 'log_phu_lai.txt')) as f:
            v, te = V.vung_phu_lai(f.read().splitlines())
        self.assertEqual(v, {'SAU-TRAI': 0.07, 'TRAI': 0.09, 'TR-TRAI': 0.17, 'TRUOC': 0.27})
        self.assertGreater(te, 1.79e9)
        self.assertEqual(V.lan_thu(['-- tuyen 2, lan 1/3: 30 diem', '   lop 2: RA_XONG', '-- tuyen 2, lan 3/3: 5 diem']), (2, 3, 3))
        self.assertIsNone(V.lan_thu(['x']))
        with open(os.path.join(DL, 'sep_nhat_ky.csv')) as f:
            d = f.read().splitlines()
        n = V.nho_ket(d)
        self.assertTrue(all(len(x) == 3 and isinstance(x[0], float) for x in n))
        self.assertEqual(V.nho_ket(['1,g,L1,BAT_DAU_THOAT,,,,,0.50,0.10,3', '2,g,CHUYEN,BAT_DAU,,di_B_day,,,0,0,0',
                                    '3,g,L1,BAT_DAU_THOAT,,,,,0.90,0.76,40', '4,g,TU_THE,KET_SAT_VAT,,,,,0.95,0.76,41',
                                    '5,g,L1,BAT_DAU_THOAT,,,,,,,']), [[0.9, 0.76, 'BAT_DAU_THOAT'], [0.95, 0.76, 'KET_SAT_VAT']])
        self.assertEqual(V.dong_chuyen_co_nghia(['== cau_noi_xe 03:00:00: di_B (BANG=day) ==', '']), '')
        self.assertEqual(V.dong_chuyen_co_nghia(['== cau_noi_xe x ==', '-- tuyen 1, lan 1/3: 3 diem', '  ']), '-- tuyen 1, lan 1/3: 3 diem')

    def test_uu_tien_tu_tep(self):
        u = V.uu_tien(C)
        self.assertEqual((u['teleop'], u['avoid'], u['nav']), (100, 60, 20))
        cu = os.environ['CAU_NOI_TWIST_MUX']
        os.environ['CAU_NOI_TWIST_MUX'] = os.path.join(TAM, 'khong_co.yaml')
        try:
            u = V.uu_tien(C)
        finally:
            os.environ['CAU_NOI_TWIST_MUX'] = cu
        self.assertIn('loi', u)
        self.assertNotIn('teleop', u)

    def test_tien_trinh_chi_lay_tham_so_biet(self):
        import subprocess
        f = os.path.join(TAM, 'lai_xe_lidar.py')
        with open(f, 'w') as fh:
            fh.write('import time\ntime.sleep(30)\n')
        p = subprocess.Popen([sys.executable if os.path.basename(sys.executable).startswith('python3') else 'python3', f, '--ros-args',
                              '-p', 'phat_lenh:=true', '-p', 'ma_bi_mat:=BIMAT123', '-p', 'sep_ra_thoat:=false'])
        try:
            time.sleep(0.3)
            tt = V.tien_trinh_lai()
            self.assertEqual(tt.get('lai_xe_lidar.py'), {'phat_lenh': 'true', 'sep_ra_thoat': 'false'})
            LINK['on'] = True
            self.assertTrue(C.bat_dau_chuyen('di_B'))
            with open(os.path.join(TAM, 'log_phu_lai.txt'), 'w') as fh:
                fh.write(ros(time.time(), 'XANH_VANG -> RA_LUI (vung_sau) | SAU=0.05 TRUOC=0.30') + '\n')
            st, b = http(self.port, 'GET', '/v10/trang_thai')[0::2]
            self.assertNotIn(b'BIMAT123', b)
            j = json.loads(b)
            self.assertEqual(j['ma'], 'TT_DANG_THOAT')
            self.assertEqual((j['bang']['ai_lai'], j['bang']['ai_uu_tien']), ('lai_xe_lidar', 60))
            self.assertEqual(j['bang']['vung'], {'SAU': 0.05, 'TRUOC': 0.3})
            self.assertEqual(j['bang']['lan'], [1, 1, 3])
            self.assertEqual(j['bang']['phu_lai'], 'RA_LUI')
        finally:
            p.kill()
            p.wait()
            C.dung_chuyen()
            cho(lambda: not C.trang_thai_nut()['chuyen']['chay'])

    def test_bang_nao_xoay(self):
        LINK['on'] = True
        ghi_pose(0.01, 0.00)
        with open(os.path.join(TAM, 'sep_nhat_ky.csv'), 'a') as f:
            f.write('6.00,2026-11-06 03:00:02,LENH,DI,,,,,0.01,0.00,0\n6.10,2026-11-06 03:00:03,CHUYEN,BAT_DAU,,di_B_day,,,0.01,0.00,0\n'
                    '6.20,2026-11-06 03:00:04,L1,BAT_DAU_THOAT,,,,,0.90,0.76,40\n')
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertEqual(j['nao']['tu_the'][0], 'DI_THUONG')
        self.assertEqual(j['nao']['lenh'][0], 'DI')
        self.assertGreater(j['nao']['tuoi_s'], 0)
        self.assertEqual(j['nho'], [[0.9, 0.76, 'BAT_DAU_THOAT']])
        self.assertEqual(j['bang']['nho'], 1)
        L = V.nap(C, 'lai_xe_lidar')
        x = j['bang']['xoay']
        self.assertEqual(x['r_xoay_m'], round(L.R_XOAY, 4))
        self.assertEqual(x['duoc'], x['kc_m'] > L.R_XOAY)
        self.assertEqual(j['bang']['ai_lai'], '')
        t = http(self.port, 'GET', '/v10')[2].decode()
        for ten in ('단계', '운전 담당', '통로 시도', '앞코 → 장애물', '제자리 회전', '두뇌 기억', '실제 코드'):
            self.assertIn('<span>%s</span>' % ten, t)



# ---------------- M7 ----------------
class M7CanGiup(Nen):
    def test_doc_scan_that(self):
        L = V.nap(C, 'lai_xe_lidar')
        for f, n, hop_le, cat in (('du_lieu_0511_toi/scan_ket_5b2.txt', 420, 389, False), ('du_lieu_0511_toi/scan_ket_5b3.txt', 420, 390, False),
                                  ('du_lieu_0511/scan_ket_k2_1933.txt', 128, 121, True)):          # so tia nhu RA_SOAT_120
            s = V.doc_scan(os.path.join(HERE, f))
            self.assertEqual((len(s.ranges), len(L.scan_to_xy(s)), bool(getattr(s, 'bi_cat', False))), (n, hop_le, cat), f)

    def test_anh_lidar_tep_moi_nhat(self):
        st, j = js(self.port, 'GET', '/v10/anh_lidar')
        self.assertEqual((st, j['co']), (200, False))                   # khong co tep -> "khong co anh"
        for i, f in enumerate(('scan_ket_5b2.txt', 'scan_ket_5b3.txt')):
            shutil.copy(os.path.join(DL, f), os.path.join(TAM, f))
            os.utime(os.path.join(TAM, f), (time.time() - 100 + i * 50,) * 2)
        try:
            st, j = js(self.port, 'GET', '/v10/anh_lidar')
            self.assertTrue(j['co'])
            self.assertTrue(j['tep'].endswith('scan_ket_5b3.txt'))
            L = V.nap(C, 'lai_xe_lidar')
            pts = [p for p in L.scan_to_xy(V.doc_scan(os.path.join(DL, 'scan_ket_5b3.txt'))) if (p[0] ** 2 + p[1] ** 2) ** .5 <= V.R_ANH]
            self.assertEqual(j['diem'], [[round(x, 3), round(y, 3)] for x, y in pts])
            self.assertEqual(j['so_tia'], 420)
        finally:
            for f in ('scan_ket_5b2.txt', 'scan_ket_5b3.txt'):
                os.remove(os.path.join(TAM, f))

    def test_thu_lai_qua_duong_nut(self):
        LINK['on'] = True
        self.assertEqual(js(self.port, 'POST', '/v10/api/thu_lai')[0], 409)     # chua co chuyen nao
        os.environ.update(GIA_KET='HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.', GIA_MA='1', GIA_CHUYEN_S='0.1')
        self.assertEqual(js(self.port, 'POST', '/v10/api/di', {'dich': 'A'})[0], 202)
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        self.assertEqual(self.ma(), 'TT_CAN_GIUP')
        os.environ.update(GIA_KET='DEN_NOI: ve_A xong.', GIA_MA='0', GIA_CHUYEN_S='30')
        st, j = js(self.port, 'POST', '/v10/api/thu_lai')
        self.assertEqual((st, j['viec']), (202, 'chuyen'))
        self.assertEqual(C.trang_thai_nut()['chuyen']['huong'], 've_A')
        self.assertEqual(js(self.port, 'POST', '/v10/api/thu_lai')[0], 409)
        js(self.port, 'POST', '/v10/api/dung')
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        with open(C.CFG['pin_file'], 'w') as f:
            f.write('2468\n')
        self.assertEqual(js(self.port, 'POST', '/v10/api/thu_lai')[0], 401)

    def test_trang_co_khung_can_giup(self):
        t = http(self.port, 'GET', '/v10')[2].decode()
        for x in ('id="help"', 'id="foto"', "lenh('thu_lai')", 'navigator.vibrate', "classList.toggle('bao'", "fetch('/v10/anh_lidar'"):
            self.assertIn(x, t)


if __name__ == '__main__':
    unittest.main()
