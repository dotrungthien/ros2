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
if [ -n "$GIA_TRE" ]; then sleep "$GIA_TRE" & wait; fi
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
        os.environ.pop('GIA_TRE', None)

    def ma(self):
        st, j = js(self.port, 'GET', '/v10/trang_thai')
        self.assertEqual(st, 200)
        return j['ma']


# ---------------- M1 ----------------
def link(l='on', chi=''):
    return {'link': l, 'chi_tiet': chi}


def nut(chuyen_chay=False, ket_qua='', bat=False, tat=False, tat_ma=None):
    return {'tat': {'chay': tat, 'ma': tat_ma, 'dong': ''}, 'bat': {'chay': bat},
            'chuyen': {'chay': chuyen_chay, 'ket_qua': ket_qua, 'dong': '', 'giay': 1.0, 'ma': None}}


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
        # gui_duong chet vi RuntimeError khi bi SIGINT (gia lap 06/11): ma 1 nhung dong khong phai HET_DUONG -> LOI_THOAT
        n1 = nut(ket_qua='HET_DUONG')
        n1['chuyen'].update(ma=1, dong='RuntimeError: Unable to convert call argument to Python object')
        ma_, ly_ = f(link('on'), n1, SEP0, None, None, None, now)
        self.assertEqual((ma_, ly_[:12]), ('TT_CAN_GIUP', 'LOI_THOAT ma'))
        n1['chuyen'].update(dong='HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.')
        self.assertEqual(f(link('on'), n1, SEP0, None, None, None, now), ('TT_CAN_GIUP', 'HET_DUONG'))
        n1['chuyen'].update(ma=-9, ket_qua='LOI', dong='')
        self.assertEqual(f(link('on'), n1, SEP0, None, None, None, now)[1][:12], 'LOI_THOAT ma')
        n1['chuyen'].update(ma=130, ket_qua='NGAT', dong='HUY_DICH_KHI_NGAT ok')
        self.assertEqual(f(link('on'), n1, SEP0, None, None, None, now)[0], 'TT_SAN_SANG')
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
        self.assertEqual(f(link('on'), n, dict(SEP0, tu_the=('CHO', 'KHONG_CO_DICH', '')), None, None, None, now)[0], 'TT_DANG_DI')   # sep chi nhin
        self.assertEqual(f(link('on'), n, dict(SEP0, tu_the=('CHO', 'VAT_LA 3s', '')), None, None, None, now)[0], 'TT_DANG_DI')
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
                                    '3,g,L1,BAT_DAU_THOAT,,,,,0.90,0.76,40', '4,g,TRUY_VET,KET_SAT_VAT,n,k,BAT,,1.05,0.76,41',
                                    '5,g,TRUY_VET,KET_SAT_VAT,n,k,HET,,1.30,0.76,41', '6,g,TRUY_VET,VAT_LA_TRUOC,n,k,BAT,,0.93,0.77,41',
                                    '7,g,L1,BAT_DAU_THOAT,,,,,,,']), [[0.9, 0.76, 'BAT_DAU_THOAT'], [1.05, 0.76, 'KET_SAT_VAT']])
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

            def co_dong_tuyen():                                     # chuyen.sh gia ghi dong '-- tuyen' sau khi Popen tra ve
                with open(C.CFG['log_chuyen']) as fl:
                    return '-- tuyen 1' in fl.read()
            self.assertTrue(cho(co_dong_tuyen))
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



# ---------------- M6 ----------------
from http.server import BaseHTTPRequestHandler as _Goc  # noqa: E402
import urllib.parse  # noqa: E402

MA_GIA = 'ma-thu-7531'      # ma gia cho teleop gia (khong phai ma that)


class TeleopGia(_Goc):
    """Giong teleop_web: /login (form code) -> 302 + Set-Cookie teleop_token; /cmd can cookie (neu co ma) -> 200, ghi lenh."""
    ma = MA_GIA
    lenh, dang_nhap = [], []

    def log_message(self, *a):
        pass

    def do_POST(self):
        n = int(self.headers.get('Content-Length') or 0)
        b = self.rfile.read(n).decode()
        if self.path == '/login':
            code = urllib.parse.parse_qs(b).get('code', [''])[0]
            TeleopGia.dang_nhap.append(code == self.ma)
            if code == self.ma:
                self.send_response(302)
                self.send_header('Location', '/')
                self.send_header('Set-Cookie', 'teleop_token=tok123; HttpOnly; Path=/; SameSite=Strict')
            else:
                self.send_response(401)
            self.end_headers()
            return
        if self.path == '/cmd':
            if self.ma and 'teleop_token=tok123' not in (self.headers.get('Cookie') or ''):
                self.send_response(401)
                self.end_headers()
                return
            TeleopGia.lenh.append(json.loads(b))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"ok": true}')


class M6LaiTay(Nen):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.tp = C.ThreadingHTTPServer(('127.0.0.1', 0), TeleopGia)
        os.environ['CAU_NOI_TELEOP_PORT'] = str(cls.tp.server_address[1])
        threading.Thread(target=cls.tp.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.tp.shutdown()
        cls.tp.server_close()
        os.environ.pop('CAU_NOI_TELEOP_PORT', None)
        super().tearDownClass()

    def setUp(self):
        super().setUp()
        TeleopGia.lenh, TeleopGia.dang_nhap, TeleopGia.ma = [], [], MA_GIA
        V._TELEOP.update(cookie=None, t_hong=-1e9)
        os.environ.pop(V.TEN_BIEN_MA, None)

    def tearDown(self):
        os.environ.pop(V.TEN_BIEN_MA, None)

    def test_dang_nhap_bang_ma_trong_moi_truong(self):
        st, b = http(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0::2]
        self.assertEqual(st, 503)                                   # khong co bien moi truong -> bao, khong so du phong
        self.assertEqual(TeleopGia.dang_nhap, [])
        os.environ[V.TEN_BIEN_MA] = MA_GIA
        st, b = http(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0::2]
        self.assertEqual(st, 200)
        self.assertNotIn(MA_GIA.encode(), b)
        self.assertEqual(TeleopGia.dang_nhap, [True])
        for _ in range(3):
            self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 0, 'a': -1})[0], 200)
        self.assertEqual(TeleopGia.dang_nhap, [True])               # cookie dung lai, khong dang nhap moi lan
        self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 0, 'a': 0})[0], 200)
        self.assertEqual(TeleopGia.lenh, [{'linear': 1.0, 'angular': 0.0}] + [{'linear': 0.0, 'angular': -1.0}] * 3 + [{'linear': 0.0, 'angular': 0.0}])

    def test_ma_sai_khong_thu_lien_tuc(self):
        os.environ[V.TEN_BIEN_MA] = 'ma-sai'
        st, b = http(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0::2]
        self.assertEqual(st, 502)
        self.assertNotIn(b'ma-sai', b)
        for _ in range(5):
            self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0], 503)
        self.assertEqual(TeleopGia.dang_nhap, [False])              # 1 lan sai, cho T_CHO_DANG_NHAP (teleop khoa sau 5 lan)
        self.assertEqual(TeleopGia.lenh, [])

    def test_teleop_khong_ma_va_gioi_han(self):
        TeleopGia.ma = ''
        self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 5, 'a': -9})[0], 200)
        self.assertEqual(TeleopGia.lenh, [{'linear': 1.0, 'angular': -1.0}])
        self.assertEqual(TeleopGia.dang_nhap, [])
        self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 'x'})[0], 400)

    def test_pin_va_teleop_tat(self):
        with open(C.CFG['pin_file'], 'w') as f:
            f.write('2468\n')
        self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0], 401)
        self.assertEqual(TeleopGia.lenh, [])
        cu = os.environ['CAU_NOI_TELEOP_PORT']
        os.environ['CAU_NOI_TELEOP_PORT'] = '1'
        try:
            st, j = js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0}, pin='2468')
        finally:
            os.environ['CAU_NOI_TELEOP_PORT'] = cu
        self.assertEqual(st, 502)
        self.assertIn('khong goi duoc teleop_web', j['loi'])

    def test_khoa_khi_co_chuyen(self):            # 07/11 111-Q2 (9)
        TeleopGia.ma = ''
        with C.LOCK:
            C.CHUYEN['chay'] = True
        try:
            st, j = js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})
            self.assertEqual(st, 409)
            self.assertTrue(j.get('khoa'))
            self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 0, 'a': -1})[0], 409)
            self.assertEqual(TeleopGia.lenh, [])                              # teleop khong nhan lenh nao
            self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 0, 'a': 0})[0], 200)   # lenh 0 (tha nut) van qua
            self.assertEqual(TeleopGia.lenh, [{'linear': 0.0, 'angular': 0.0}])
        finally:
            with C.LOCK:
                C.CHUYEN['chay'] = False
        self.assertEqual(js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})[0], 200)       # het chuyen: mo khoa

    def test_trang_co_pad(self):
        t = http(self.port, 'GET', '/v10')[2].decode()
        for x in ('aria-label="전진"', 'aria-label="후진"', 'aria-label="좌회전"', 'aria-label="우회전"', "'pointerup'", "fetch('/v10/api/lai_tay'",
                  'setInterval(function(){gui_pad(l,a);},100)'):
            self.assertIn(x, t)



# ---------------- D(1): ben ghi <-> ben doc (doc nguon ben ghi; tep co dong khong duoc in: chi assertTrue + cau rieng) ----------------
def doc(*p):
    with open(os.path.join(HERE, *p), encoding='utf-8') as f:
        return f.read()


class D1KhopGhep(unittest.TestCase):
    def test_sep_nhat_ky(self):
        S = V.nap(C, 'sep')
        cot = S.COT_NHAT_KY.split(',')
        self.assertEqual((cot[1], cot[2], cot[3], cot[6], cot[8:11]), ('gio', 'loai', 'dau_hieu', 'ket_qua', ['x', 'y', 'goc_do']))
        src = doc('sep.py')
        for s in ("ghi('TU_THE'", "ghi('LENH'", "ghi('CHUYEN', 'BAT_DAU'", "'BAT_DAU_THOAT'", "'BAT_DAU_RA_XOAY'", "'DUNG_VA_BAO'",
                  "os.path.expanduser('~/sep_nhat_ky.csv')"):
            self.assertIn(s, src)
        for ten in tuple(d for _, d in V.NHO_LOAI):
            self.assertIn("'%s'" % ten, src, ten)
        # moi (loai, dau_hieu) cua NHO_LOAI co that trong nhat ky xe 05/11 (khong chi la chuoi trong ma)
        with open(os.path.join(DL, 'sep_nhat_ky.csv')) as f:
            co = {tuple(l.split(',')[2:4]) for l in f}
        self.assertTrue(V.NHO_LOAI - {('L1', 'BAT_DAU_RA_XOAY')} <= co, V.NHO_LOAI - co)
        self.assertIn("ghi('L1', 'BAT_DAU_RA_XOAY'", src)
        # TU_THE that cua sep: trang khong suy trang thai tu TU_THE (chi hien o dong 두뇌)
        with open(os.path.join(DL, 'sep_nhat_ky.csv')) as f:
            tu_the = {l.split(',')[3] for l in f if l.split(',')[2:3] == ['TU_THE']}
        self.assertLessEqual({'DI_THUONG', 'CHO', 'THAN_TRONG', 'DUNG_VA_BAO', 'THOAT'}, tu_the)
        self.assertFalse(hasattr(V, 'SEP_CHO') or hasattr(V, 'SEP_THOAT'))
        self.assertTrue(C.CFG['sep_nhat_ky'].endswith('/sep_nhat_ky.csv'))

    def test_log_phu_lai(self):
        src = doc('lai_xe_lidar.py')
        self.assertIn("'%s -> %s (%s)%s | %s'", src)
        self.assertIn("'vung gan nhat: %s | dich=%s trang_thai=%s'", src)
        self.assertIn("self.get_logger()", src)
        for ten in V.PHU_LAI_THOAT + V.PHU_LAI_XOAY + V.PHU_LAI_CHO:
            self.assertIn("'%s'" % ten, src, ten)
        self.assertIn("'CHO_%d'", src)
        L = V.nap(C, 'lai_xe_lidar')
        self.assertEqual(set(L.TEN_VUNG.values()), set(V.VUNG_TEN))
        bx = doc('bat_xe.sh')
        self.assertTrue('python3 ~/lai_xe_lidar.py' in bx and '> ~/log_phu_lai.txt 2>&1' in bx, 'bat_xe.sh khong ghi ~/log_phu_lai.txt')
        self.assertTrue('python3 ~/sep.py' in bx, 'bat_xe.sh khong chay ~/sep.py')
        self.assertTrue("-p lidar_angle_offset_deg" not in bx, 'bat_xe.sh doi goc lech LiDAR')
        self.assertEqual(L.scan_to_xy.__defaults__, (0.0,))

    def test_log_chuyen_gui_duong(self):
        src = doc('gui_duong.py')
        for s in ("'-- tuyen %d, lan %d/%d:", "lop 2: sep bao %s", "'RA_TIEN', 'tien ra'", "'RA_LUI (vat sat mui)'", "sep bao CHO (%s)",
                  "'   khong den duoc (%s%s).'", "lop 2: RA_XONG sau", "RA_HET_GIO:", "tinh lai duong tu cho xe", "'HUY_DICH_KHI_NGAT %s'",
                  "DICH = {'di_B': (TT.B, 0.0), 've_A': (TT.A, None)}"):
            self.assertIn(s, src, s)
        self.assertIn('exec python3 "$HOME/gui_duong.py" "$HUONG" "$TUYEN_DAU"', doc('chuyen.sh'))
        self.assertIn('TUYEN_DAU="${2:-1}"', doc('chuyen.sh'))           # /nut khong doi so 2 => tuyen 1 truoc (M3 cung thu tu)

    def test_sep_thoat_ghi_doc(self):
        S = V.nap(C, 'sep')
        p = os.path.join(TAM, 'sep_thoat_thu')
        S.ghi_lenh(p, 500.0, 'RA_TIEN', '3 7')
        with open(p) as f:
            self.assertEqual(V.thoat_moi(f.readline(), 501.0), 'RA_TIEN')
        self.assertIn("declare_parameter('tep_thoat', os.path.expanduser('~/sep_thoat'))", doc('sep.py'))
        self.assertIn('time.clock_gettime(time.CLOCK_BOOTTIME)', doc('sep.py'))     # /proc/uptime = CLOCK_BOOTTIME

    def test_ma_thoat(self):
        self.assertEqual(C.MA_CHUYEN, {0: 'DEN_NOI', 1: 'HET_DUONG', 3: 'SAI_VI_TRI', 4: 'DUNG_SEP', 130: 'NGAT'})
        self.assertIn('Ma thoat: 0 den noi, 1 het duong, 2 sai doi so, 3 sai vi tri, 4 sep bao dung, 130 bi ngat (NGAT).', doc('gui_duong.py'))

    def test_teleop_web(self):
        src = doc('..', 'dev_ws', 'src', 'my_bot', 'scripts', 'teleop_web.py')
        for s in ("COOKIE_NAME = 'teleop_token'", "request.form.get('code', '')", "data.get('linear', 0.0)", "data.get('angular', 0.0)",
                  "self.declare_parameter('port', 5000)", "@app.route('/cmd', methods=['POST'])", "@app.route('/login', methods=['GET', 'POST'])",
                  "self.declare_parameter('cmd_topic', '/cmd_vel_teleop')", "MAX_FAILS = 5"):
            self.assertIn(s, src, s)
        tm = doc('..', 'dev_ws', 'src', 'my_bot', 'config', 'twist_mux.yaml')
        self.assertIn('topic: /cmd_vel_teleop', tm)
        self.assertNotIn("'port'", doc('..', 'dev_ws', 'src', 'my_bot', 'launch', 'mapping_my_bot.launch.py'))   # launch khong doi cong 5000

    def test_ten_bien_ma_giong_bat_xe(self):
        import re
        bx = doc('bat_xe.sh')
        dn = {m.group(1) for m in re.finditer(r'^([A-Z_]+)="?\$\{\1:-', bx, re.M)}
        dung = set(re.findall(r'access_code:="?\$\{?([A-Z_]+)', bx))
        self.assertTrue(len(dung) == 1, 'bat_xe.sh: khong thay dung mot bien cho access_code')
        self.assertTrue(V.TEN_BIEN_MA in dung and V.TEN_BIEN_MA in dn, 'ten bien ma cua cau noi khac bat_xe.sh')

    def test_duong_dan_tren_pi(self):
        import re
        bx = doc('bat_xe.sh')
        m = re.search(r'^SLAM="\$\{SLAM:-(\S+?)\}"', bx, re.M)
        self.assertTrue(m is not None and m.group(1) == '/home/project1/slam_dinh_vi_xuong.yaml', 'SLAM mac dinh cua bat_xe.sh khac')
        self.assertTrue("sed -n 's/^NAV_PARAMS=//p' \"$HOME/cau_noi/cau_noi.env\"" in bx, 'bat_xe.sh khong doc NAV_PARAMS tu cau_noi.env')
        cai = doc('cai_cau_noi.sh')
        self.assertIn('for f in trang_v10.py trang_v10.html; do', cai)
        self.assertIn('NAV_PARAMS=/home/$ME/nav2_params_cho.yaml', cai)
        self.assertIn('EnvironmentFile=-$DIR/cau_noi.env', cai)
        self.assertEqual(C.CFG['port'], 5001)
        self.assertEqual(C.CFG['trang_v10'], os.path.join(HERE, 'trang_v10.html'))

    def test_trang_khop_api(self):
        import re
        t = http_tinh()
        s = V.trang_thai(C)
        for k in set(re.findall(r'\bS\.([a-z_]+)', t)) - {'ma', 'loi'}:      # S.loi: chi trang dat khi mat mang
            self.assertIn(k, s, k)
        for k in set(re.findall(r'\bb\.([a-z_]+)', t)) & {'buoc', 'ai_lai', 'ai_uu_tien', 'lan', 'vung', 'vung_tuoi_s', 'xoay', 'nho', 'code'}:
            self.assertIn(k, s['bang'], k)
        ma_trang = set(re.findall(r"'(TT_[A-Z_]+)'", t))
        self.assertEqual(ma_trang, set(V.MA) | {'TT_XEM_TUYEN', 'TT_LAI_TAY'})
        url = set(re.findall(r"fetch\('(/v10/[a-z_/]+)", t))
        self.assertEqual(url, {'/v10/trang_thai', '/v10/ban_do', '/v10/tuyen', '/v10/anh_lidar', '/v10/api/', '/v10/api/lai_tay'})
        lenh = set(re.findall(r"lenh\('([a-z_]+)'", t))
        self.assertEqual(lenh, {'bat', 'tat', 'dung', 'di', 'thu_lai'})
        self.assertEqual(set(V.LENH_NUT) | {'di', 'thu_lai', 'lai_tay'}, lenh | {'lai_tay'})


def http_tinh():
    with open(os.path.join(HERE, 'trang_v10.html'), encoding='utf-8') as f:
        return f.read()


# ---------------- 111-Q2 (07/11): (9) khoa lai tay khi co chuyen, (11) so_voi_pi.sh, (15) ma 5 + HUY_DICH ok ----------------
DONG_LOI_OK = 'LOI_GUI_DUONG RuntimeError: Unable to convert call argument to Python object; HUY_DICH ok'


def nut_ma5(dong):
    n = nut(ket_qua='LOI')
    n['chuyen'].update(ma=5, dong=dong)
    return n


class Q2Ma5(unittest.TestCase):
    def test_ma5_huy_dich_ok_khac_loi_thoat(self):
        f = V.suy_trang_thai
        ma_, ly_ = f(link('on'), nut_ma5(DONG_LOI_OK), SEP0, None, None, None, 1000.0)
        self.assertEqual(ma_, 'TT_CAN_GIUP')
        self.assertTrue(ly_.startswith('LOI_DA_HUY'), ly_)
        dai = 'LOI_GUI_DUONG RuntimeError: ' + 'x' * 160 + '; HUY_DICH ok'      # thong bao dai: ly_do bi cat 120 van nhan dung
        self.assertTrue(f(link('on'), nut_ma5(dai), SEP0, None, None, None, 1000.0)[1].startswith('LOI_DA_HUY'))
        for kq in ('khong_tra_loi', 'khong_co_dich'):                           # khong xac nhan huy -> giu LOI_THOAT
            ly = f(link('on'), nut_ma5(DONG_LOI_OK.replace('HUY_DICH ok', 'HUY_DICH ' + kq)), SEP0, None, None, None, 1000.0)[1]
            self.assertTrue(ly.startswith('LOI_THOAT'), ly)
        n = nut_ma5(DONG_LOI_OK)
        n['chuyen'].update(ma=1)                                                # "ok" chi tinh voi ma 5
        self.assertTrue(f(link('on'), n, SEP0, None, None, None, 1000.0)[1].startswith('LOI_THOAT'))

    def test_trang_cau_ma5(self):
        t = http_tinh()
        self.assertIn('오류로 주행을 멈췄어요. 목표는 취소됐어요', t)
        self.assertIn('Nav2 목표가 아직 살아 있을 수 있어요: 모터 스위치를 끄세요', t)    # LOI_THOAT giu cau cu


class Q2Nut(unittest.TestCase):
    def test_trang_khoa_pad_va_nut_sau_dung(self):
        t = http_tinh()
        chay = [l for l in t.split('\n') if "['TT_DANG_DI','TT_DANG_CHO','TT_DANG_THOAT','TT_DANG_XOAY'].indexOf(m)>=0" in l]
        self.assertEqual(len(chay), 1)
        self.assertNotIn('vao_lai_tay', chay[0])                              # dang chay: chi con 정지
        self.assertIn("['정지',\"lenh('dung')\",DO]", chay[0])
        self.assertIn("function SAU_DUNG(){return nut([['다시 시도',\"lenh('thu_lai')\"],['수동 조종',\"vao_lai_tay()\"],['연결 해제',\"lenh('tat')\"]]);}", t)
        self.assertIn("m==='TT_SAN_SANG'&&(S.chuyen||{}).ket_qua==='NGAT')h=SAU_DUNG()", t)
        self.assertIn("(S.ly_do||'').indexOf('LOI_DA_HUY')===0?SAU_DUNG()", t)
        vao = [l for l in t.split('\n') if l.startswith('function vao_lai_tay(')][0]
        self.assertIn('if((S.chuyen||{}).chay){', vao)                       # dang chay: khong vao pad
        self.assertIn("if(LOCAL==='TT_LAI_TAY'&&(S.chuyen||{}).chay){tha();LOCAL=null;}", t)


class Q2D1(unittest.TestCase):
    def test_huy_dich_gui_duong_khop_trang(self):
        g = doc('gui_duong.py')
        self.assertTrue("io.bao('LOI_GUI_DUONG %s: %s; HUY_DICH %s' % (e.__class__.__name__, str(e)[:160], kq))" in g, 'dong LOI_GUI_DUONG cua gui_duong.py doi')
        self.assertTrue("return 'ok' if fut.done() else 'khong_tra_loi'" in g, 'ket qua huy dich cua gui_duong.py doi')
        self.assertTrue('MA_LOI = 5 ' in g and 'return MA_LOI' in g, 'ma 5 cua gui_duong.py doi')
        dong = 'LOI_GUI_DUONG %s: %s; HUY_DICH %s' % ('RuntimeError', 'y' * 160, 'ok')       # dung dinh dang ben ghi
        self.assertTrue(C.DONG_CHUYEN.match(dong))                             # cau noi chon dong nay lam dong ket thuc chuyen
        self.assertEqual(C.MA_CHUYEN.get(5, 'LOI'), 'LOI')
        self.assertTrue(V.suy_trang_thai(link('on'), nut_ma5(dong), SEP0, None, None, None, 1.0)[1].startswith('LOI_DA_HUY'))

    def test_so_voi_pi_co_trang_v10(self):
        import subprocess
        sv = doc('so_voi_pi.sh')                                                # tep co dong khong duoc in: chi assertTrue
        for d in ('trang_v10.py|~/cau_noi/trang_v10.py', 'trang_v10.html|~/cau_noi/trang_v10.html'):
            self.assertTrue(('\n' + d + '\n') in sv, 'so_voi_pi.sh thieu dong bang %s' % d.split('|')[0])
        cai = doc('cai_cau_noi.sh')
        self.assertTrue('DIR="$HOME/cau_noi"' in cai and 'cp -f "$f" "$DIR/$f"' in cai, 'cai_cau_noi.sh khong chep vao ~/cau_noi')
        r = subprocess.run(['bash', os.path.join(HERE, 'so_voi_pi.sh'), '--nhac'], capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 0)
        self.assertFalse('trang_v10' in r.stdout, 'so_voi_pi.sh --nhac con nhac trang_v10')


# ---------------- 111-Q3 (08/11): (1) 경로 계산 중 (CHUAN_BI), (2) 시간 초과 (han_het) ----------------
# Dong log THAT: git show origin/odom-118:xe_scripts/du_lieu_0711/log_khe_tren_v1_1741.txt (md5 ad846080), xe 07/11 17:41,
# chinh la ~/log_nut_chuyen.txt cua cau noi (dong dau cau_noi_xe.py:261, cac dong sau la stdout gui_duong.py).
LOG_THAT_0711 = [
    '== cau_noi_xe 17:41:08: di_B (BANG=day) ==',
    '   tuyen 1: khong tim duoc duong (11, 8) -> (43, 24) => doi tuyen',
    '== Tuyen 1 hong. ==',
    '   CANH BAO: dinh re (0.66; 0.71) chi cach o do 0.15 m (< 0.24), khong doi duoc',
    '-- tuyen 2, lan 1/3: 50 diem tu (0.01; 0.00) toi (1.60; 0.82)',
    'DEN_NOI: di_B xong theo tuyen 2 (cach dich 0.15 m).',
]


class Q3ChuanBi(unittest.TestCase):
    def test_chuan_bi_khi_log_rong(self):
        f = V.suy_trang_thai
        n = nut(chuyen_chay=True)
        self.assertEqual(f(link('on'), n, SEP0, None, None, None, 1000.0), ('TT_DANG_DI', 'CHUAN_BI'))
        # sep con dong TU_THE cu CHO / KHONG_CO_DICH (= 목표 없음) va phu lai KHONG_CO_DICH: van CHUAN_BI
        self.assertEqual(f(link('on'), n, dict(SEP0, tu_the=('CHO', 'KHONG_CO_DICH', '')), ('KHONG_CO_DICH', '', 1000.0), None, None, 1000.0),
                         ('TT_DANG_DI', 'CHUAN_BI'))
        self.assertEqual(f(link('on'), n, SEP0, None, 'DI', None, 1000.0), ('TT_DANG_DI', ''))       # da co '-- tuyen'
        self.assertEqual(f(link('on'), n, SEP0, None, 'CHO', None, 1000.0)[0], 'TT_DANG_CHO')        # sep bao CHO truoc SAP_GUI
        self.assertEqual(f(link('on'), n, SEP0, None, None, 'RA_LUI', 1000.0)[0], 'TT_DANG_THOAT')   # lop 2 truoc '-- tuyen'
        self.assertEqual(f(link('on'), nut(), SEP0, None, None, None, 1000.0), ('TT_SAN_SANG', ''))  # khong co chuyen: khong CHUAN_BI

    def test_ranh_gioi_chuan_bi_den_dang_di_log_that(self):
        f = V.suy_trang_thai
        n = nut(chuyen_chay=True)
        for k in range(1, len(LOG_THAT_0711) + 1):
            ly = f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(LOG_THAT_0711[:k]), None, 1000.0)
            self.assertEqual(ly, ('TT_DANG_DI', 'CHUAN_BI' if k <= 4 else ''), 'dong %d: %s' % (k, LOG_THAT_0711[k - 1]))
        self.assertEqual(V.dong_chuyen_cuoi(LOG_THAT_0711[:4]), None)
        self.assertEqual(V.dong_chuyen_cuoi(LOG_THAT_0711[:5]), 'DI')
        self.assertEqual(V.lan_thu(LOG_THAT_0711), (2, 1, 3))


class Q3ChuanBiHttp(Nen):
    def test_chuan_bi_co_dung_va_khoa_pad(self):
        LINK['on'] = True
        os.environ['GIA_TRE'] = '30'                       # chuyen.sh gia: chua in '-- tuyen' (gui_duong dang chuan bi)
        self.assertTrue(C.bat_dau_chuyen('di_B'))
        try:
            st, j = js(self.port, 'GET', '/v10/trang_thai')
            self.assertEqual((j['ma'], j['ly_do']), ('TT_DANG_DI', 'CHUAN_BI'))
            st, j = js(self.port, 'POST', '/v10/api/lai_tay', {'l': 1, 'a': 0})
            self.assertEqual((st, j.get('khoa')), (409, True))                          # pad khoa (server)
        finally:
            C.dung_chuyen()
            self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        t = http_tinh()
        self.assertIn("function chuan_bi(){return ma()==='TT_DANG_DI'&&S.ly_do==='CHUAN_BI';}", t)
        self.assertIn("set('tt',chuan_bi()?'경로 계산 중':ten(m))", t)                  # o trang thai
        self.assertIn("set('p0',chuan_bi()?'경로 계산 중':", t)                         # 단계
        self.assertIn("var nt=chuan_bi()?'두뇌: 출발 준비 대기':", t)                    # dong 두뇌 khong in TU_THE cu (목표 없음)
        chay = [l for l in t.split('\n') if "['TT_DANG_DI','TT_DANG_CHO','TT_DANG_THOAT','TT_DANG_XOAY'].indexOf(m)>=0" in l][0]
        self.assertIn("['정지',\"lenh('dung')\",DO]", chay)                             # CHUAN_BI = TT_DANG_DI: chi 정지
        self.assertNotIn('vao_lai_tay', chay)
        self.assertIn('if((S.chuyen||{}).chay){', [l for l in t.split('\n') if l.startswith('function vao_lai_tay(')][0])   # pad khoa (trang)


class Q3HanHet(Nen):
    def test_han_het_o_600_s(self):
        LINK['on'] = True
        try:
            with C.LOCK:
                C.JOB['t0'] = time.monotonic() - 600
            j = V.trang_thai(C)
            self.assertLess(j['han_s'], 0)
            self.assertTrue(j['han_het'])
            with C.LOCK:
                C.JOB['t0'] = time.monotonic() - 10
            j = V.trang_thai(C)
            self.assertGreater(j['han_s'], 0)
            self.assertFalse(j['han_het'])
            with C.LOCK:
                C.JOB['t0'] = None
            self.assertFalse(V.trang_thai(C)['han_het'])                                 # chua co moc con quay
        finally:
            with C.LOCK:
                C.JOB['t0'] = None
        t = http_tinh()
        self.assertIn("(S.han_het?' (시간 초과)':S.han_s!==null&&S.han_s!==undefined?' (남은 시간 '+S.han_s+'초)':'')", t)



# ---------------- 08/11 bo 1e: CHO_VAT / DI_TIEP (gui_duong kieu mat), phu lai chi nhin ----------------
class MatLai(Nen):
    def test_d1_tep_xe_kieu_mat(self):
        import re
        sv = doc('so_voi_pi.sh')                                                         # tep co dong khong duoc in: chi assertTrue
        for d in ('bt_mat.xml|~/bt_mat.xml', 'nav2_params_mat.yaml|~/nav2_params_mat.yaml', 'lop_huong_dan.py|~/lop_huong_dan.py',
                  'slam_dinh_vi_xuong.yaml|~/slam_dinh_vi_xuong.yaml', 'slam_dinh_vi_xuong_mat.yaml|~/slam_dinh_vi_xuong_mat.yaml',
                  'du_lieu_0111/map_xuong_v3.yaml|~/map_xuong_v3.yaml',
                  'du_lieu_0111/map_xuong_v3.png|~/map_xuong_v3.png'):
            self.assertTrue(('\n' + d + '\n') in sv, 'so_voi_pi.sh thieu dong %s' % d.split('|')[0])
        self.assertTrue('\ndu_lieu_0111/slam_dinh_vi_xuong.yaml|' not in sv, 'so_voi_pi.sh con so slam yaml cu')
        self.assertIn('minimum_travel_distance: 0.25', doc('slam_dinh_vi_xuong.yaml'))            # ban cu giu nguyen
        self.assertIn('minimum_travel_distance: 0.10', doc('slam_dinh_vi_xuong_mat.yaml'))
        self.assertIn('minimum_travel_heading: 0.15', doc('slam_dinh_vi_xuong_mat.yaml'))
        self.assertTrue('SLAM="${SLAM:-/home/project1/slam_dinh_vi_xuong.yaml}"' in doc('bat_xe.sh'), 'bat_xe.sh doi SLAM mac dinh')
        # bt_mat.xml = bt_cho.xml, chi doi retries 15 -> 1000; nav2_params_mat tro toi bt_mat (kiem_xe so qua MONG_BT)
        bo_dau = lambda t: t[t.index('-->') + 3:]
        self.assertEqual(bo_dau(doc('bt_mat.xml')), bo_dau(doc('bt_cho.xml')).replace(
            'number_of_retries="15" name="NavigateRecovery"', 'number_of_retries="1000" name="NavigateRecovery"').replace(
            '''          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>''', '''          <Sequence name="LuiKhiKhongTien">
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
            <BackUp backup_dist="0.25" backup_speed="0.10"/>
          </Sequence>
        </RecoveryNode>''', 1))                                           # 2d: chi nhanh phuc hoi cua FollowPath them BackUp
        bt = bo_dau(doc('bt_mat.xml'))
        self.assertEqual(bt.count('<BackUp '), 1)
        self.assertLess(bt.index('name="FollowPath"'), bt.index('<BackUp '))
        self.assertLess(bt.index('<BackUp '), bt.index('name="RecoveryFallback"'))     # khong nam o nhanh ComputePathToPose / RecoveryFallback
        self.assertIn('nav2_back_up_action_bt_node', doc('nav2_params_mat.yaml'))
        self.assertIn('movement_time_allowance: 10.0', doc('nav2_params_mat.yaml'))
        # 2e: nav2_params_mat_rpp.yaml = nav2_params_mat.yaml, chi FollowPath = RPP y het nav2_params_cho.yaml
        import yaml
        ym = lambda t: yaml.safe_load(doc(t))
        r, m, c = ym('nav2_params_mat_rpp.yaml'), ym('nav2_params_mat.yaml'), ym('nav2_params_cho.yaml')
        fp = lambda d: d['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(fp(r), fp(c))
        bo = lambda d: dict(d, controller_server={k: v for k, v in d['controller_server']['ros__parameters'].items() if k != 'FollowPath'})
        self.assertEqual(bo(r), bo(m))
        self.assertTrue('\nnav2_params_mat_rpp.yaml|~/nav2_params_mat_rpp.yaml\n' in doc('so_voi_pi.sh'), 'so_voi_pi.sh thieu mat_rpp')
        pm = doc('nav2_params_mat.yaml')
        self.assertIn('default_nav_to_pose_bt_xml: /home/project1/bt_mat.xml', pm)
        self.assertIn('yaml_filename: /home/project1/map_xuong_v3.yaml', pm)
        bx = doc('bat_xe.sh')
        self.assertTrue("MONG_BT=$(sed -n 's/^ *default_nav_to_pose_bt_xml: *//p' \"$NAV_PARAMS\"" in bx, 'bat_xe.sh MONG_BT doi')
        self.assertTrue('if [ "$MAT_LAI" = "true" ]; then echo mat > ~/goc_dang_chay; else echo cu > ~/goc_dang_chay; fi' in bx,
                        'bat_xe.sh khong ghi de ~/goc_dang_chay moi lan bat')
        self.assertTrue('rm -f ~/goc_dang_chay' in bx, 'bat_xe.sh tat khong xoa ~/goc_dang_chay')
        self.assertTrue('|[m]ap_server|[l]op_huong_dan|' in bx, 'NODES thieu map_server / lop_huong_dan')
        self.assertTrue(bx.count('$NGHE_FP >') == 1 and bx.count('$NGHE_FP $SEP_MAT >') == 1 and 'NGHE_FP="-p nghe_follow_path:=false"' in bx,
                        'nghe_follow_path')
        self.assertTrue('SEP_MAT=""; [ "$MAT_LAI" = "true" ] && SEP_MAT="-p dem_luot_thoat:=false"' in bx, 'bat_xe.sh dem_luot_thoat')
        S_ = V.nap(C, 'sep')
        self.assertIs(S_.P_MAC_DINH['dem_luot_thoat'], True)                              # ben doc: tham so co that, mac dinh nhu cu
        self.assertIn('for k, v in P_MAC_DINH.items():', doc('sep.py'))                    # nut khai bao moi khoa P_MAC_DINH
        for ten in ('sep.py', 'lai_xe_lidar.py'):                                          # ben doc: ten tham so co that
            self.assertIn("declare_parameter('nghe_follow_path', True)", doc(ten))
        kx = doc('kiem_xe.sh')
        self.assertIn('if [ "$(head -1 ~/goc_dang_chay 2>/dev/null)" = "mat" ]; then', kx)
        self.assertIn('python3 ~/lop_huong_dan.py --kiem-costmap 10 || KQ=1', kx)
        G = V.nap(C, 'gui_duong')
        self.assertEqual(os.path.basename(G.TEP_GOC), 'goc_dang_chay')
        launch = doc('..', 'dev_ws', 'src', 'my_bot', 'launch', 'navigation_my_bot.launch.py')
        self.assertIn("cmd=['python3', EnvironmentVariable('LOP_HUONG_DAN_PY', default_value='/home/project1/lop_huong_dan.py')]", launch)
        self.assertIn("remappings=remappings + [('map', 'map_tinh')]", launch)
        self.assertIn("'node_names': ['map_server'] + lifecycle_nodes + ['collision_monitor']", launch)
        self.assertIn('map_topic: /map_tinh', pm)
        self.assertIn('map_topic: /lop_huong_dan', pm)
        # 08/11 buoc 2g: behavior_server (BackUp BT + /backup gui_duong) ra thang /cmd_vel_avoid (twist_mux 60) khi mat_lai:=true,
        # khong qua smoother/collision_monitor; mat_lai:=false giu cmd_vel_nav_raw. D(1): phu lai phai chi nhin khi MAT_LAI=true.
        import re as _re
        bs = launch[launch.index("executable='behavior_server'"):]
        bs = bs[:bs.index('Node(')]
        self.assertIn("remappings=remappings + [('cmd_vel', ra_behavior)]", bs)
        self.assertIn('''ra_behavior = PythonExpression(["'cmd_vel_avoid' if '", mat_lai, "'.lower() in ('true', '1') else 'cmd_vel_nav_raw'"])''', launch)
        self.assertIn('''ra_smoother = PythonExpression(["'cmd_vel_nav_tho' if '", mat_lai, "'.lower() in ('true', '1') else 'cmd_vel_nav'"])''', launch)
        self.assertIn('behavior_plugins: ["spin", "backup", "wait"]', pm)
        tm = doc('..', 'dev_ws', 'src', 'my_bot', 'config', 'twist_mux.yaml')
        self.assertTrue(_re.search(r'avoid:\n +topic: /cmd_vel_avoid\n +timeout: 0\.2\n +priority: 60\n', tm), 'twist_mux avoid 60')
        loi = 'then echo "LOI: MAT_LAI=true thi PHU_LAI_PHAT_LENH phai false (/cmd_vel_avoid chi cho behavior_server)"; exit 1; fi'
        self.assertTrue(('if [ "${MAT_LAI:-false}" = "true" ] && [ "$PHU_LAI_PHAT_LENH" = "true" ]; ' + loi) in doc('bat_xe.sh'))
        self.assertIn('if [ "$MAT_LAI" = "true" ] && [ "$PHU_LAI_PHAT_LENH" = "true" ]; ' + loi, doc('gia_lap', 'bat_gia.sh'))
        self.assertEqual(C.env_muc_b({'MAT_LAI': 'true'})['PHU_LAI_PHAT_LENH'], 'false')
        G = V.nap(C, 'gui_duong')
        self.assertIn("ActionClient(self, BackUp, 'backup')", doc('gui_duong.py'))
        self.assertEqual((G.cau_hinh({})['lui_m'], G.cau_hinh({})['lui_v']), (0.35, 0.10))

    def test_cau_noi_env_muc_b(self):
        self.assertIs(C.env_muc_b({}), C.ENV_MUC_B)                                      # khong MAT_LAI: nhu cu
        self.assertIs(C.env_muc_b({'MAT_LAI': 'false'}), C.ENV_MUC_B)
        self.assertEqual(C.ENV_MUC_B, {'SEP_RA_LENH': 'true', 'SEP_RA_THOAT': 'true', 'PHU_LAI_PHAT_LENH': 'true'})
        self.assertEqual(C.env_muc_b({'MAT_LAI': 'true'}),
                         {'SEP_RA_LENH': 'true', 'SEP_RA_THOAT': 'false', 'PHU_LAI_PHAT_LENH': 'false', 'MAT_LAI': 'true'})
        self.assertIn('dict(os.environ, **env_muc_b())', doc('cau_noi_xe.py'))
        bx = doc('bat_xe.sh')                                                            # ben doc: bat_xe.sh doc dung 4 ten bien
        for k in C.ENV_MUC_B_MAT:
            self.assertTrue(('%s="${%s:-' % (k, k)) in bx, 'bat_xe.sh khong doc bien %s' % k)

    def test_cho_vat_va_di_tiep(self):
        f = V.suy_trang_thai
        n = nut(chuyen_chay=True)
        log = ['== cau_noi_xe 20:00:00: di_B (BANG=day) ==', 'KIEU gui_duong: mat (~/goc_dang_chay)',
               '-- tuyen 1, lan 1/1: navigate_to_pose tu (0.01; 0.00) toi (1.60; 0.82)']
        self.assertEqual(f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(log[:2]), None, 1.0), ('TT_DANG_DI', 'CHUAN_BI'))
        self.assertEqual(f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(log), None, 1.0), ('TT_DANG_DI', ''))
        log.append('CHO_VAT: 5/30 s (cach dich 1.20 m)')
        self.assertEqual(f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(log), None, 1.0), ('TT_DANG_CHO', 'gui_duong cho'))
        log.append('   DI_TIEP: xe tien lai.')
        self.assertEqual(f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(log), None, 1.0), ('TT_DANG_DI', ''))
        log.append('   khong den duoc (CHO_VAT_30s, cach dich 0.95 m).')
        self.assertEqual(f(link('on'), n, SEP0, None, V.dong_chuyen_cuoi(log), None, 1.0)[0], 'TT_DANG_CHO')

    def test_phu_lai_chi_nhin_khong_hien_thoat_xoay(self):
        f = V.suy_trang_thai
        n = nut(chuyen_chay=True)
        for pl, ma_cu in ((('RA_LUI', '', 1.0), 'TT_DANG_THOAT'), (('XOAY_P', '', 1.0), 'TT_DANG_XOAY'), (('CHO_SEP', '', 1.0), 'TT_DANG_CHO')):
            self.assertEqual(f(link('on'), n, SEP0, pl, 'DI', None, 1.0)[0], ma_cu)                       # mac dinh nhu cu
            self.assertEqual(f(link('on'), n, SEP0, pl, 'DI', None, 1.0, False), ('TT_DANG_DI', ''))     # chi nhin
        self.assertEqual(f(link('on'), n, SEP0, ('RA_LUI', '', 1.0), 'DI', 'RA_LUI', 1.0, False)[0], 'TT_DANG_THOAT')   # sep thoat van tinh

    def test_trang_thai_doc_phat_lenh_tu_proc(self):
        import subprocess
        f = os.path.join(TAM, 'lai_xe_lidar.py')
        with open(f, 'w') as fh:
            fh.write('import time\ntime.sleep(30)\n')
        LINK['on'] = True
        p = subprocess.Popen([sys.executable if os.path.basename(sys.executable).startswith('python3') else 'python3', f, '--ros-args',
                              '-p', 'phat_lenh:=false', '-p', 'sep_ra_thoat:=false', '-p', 'nghe_follow_path:=false'])
        try:
            time.sleep(0.3)
            self.assertTrue(C.bat_dau_chuyen('di_B'))

            def co_dong_tuyen():
                with open(C.CFG['log_chuyen']) as fl:
                    return '-- tuyen 1' in fl.read()
            self.assertTrue(cho(co_dong_tuyen))
            with open(os.path.join(TAM, 'log_phu_lai.txt'), 'w') as fh:
                fh.write(ros(time.time(), 'XANH_VANG -> RA_LUI (vung_sau) | SAU=0.05 TRUOC=0.30') + '\n')
            st, j = js(self.port, 'GET', '/v10/trang_thai')
            self.assertEqual(j['ma'], 'TT_DANG_DI')                                    # phu lai chi nhin: khong 탈출
            self.assertEqual(j['bang']['code']['lai_xe_lidar.py'].get('phat_lenh'), 'false')
        finally:
            p.kill()
            p.wait()
            C.dung_chuyen()
            cho(lambda: not C.trang_thai_nut()['chuyen']['chay'])

if __name__ == '__main__':
    unittest.main()
