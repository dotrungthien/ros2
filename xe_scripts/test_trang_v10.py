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
            C.CHUYEN.update(ma=None, ket_qua='', dong='')
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


if __name__ == '__main__':
    unittest.main()
