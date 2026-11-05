#!/usr/bin/env python3
"""test_cau_noi.py (viec 111, 06/11): test hoi quy cau_noi_xe.py: /nut, /api/nut/*, /api/b/* phai tra NHU BAN 579cda5.
Kho truoc viec 111 khong co test nao cho cau noi: tep nay viet va chay DAT tren cau_noi_xe.py @579cda5 truoc khi them /v10.
Chay cau noi that (ThreadingHTTPServer, cong ngau nhien) voi HOME gia, bat_xe.sh / chuyen.sh gia (bash). Khong can ROS.
Chay: python3 -m unittest test_cau_noi   (trong xe_scripts)
"""
import hashlib
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
TAM = tempfile.mkdtemp(prefix='test_cau_noi_')
os.environ['HOME'] = TAM
for k in [k for k in os.environ if k.startswith('CAU_NOI_')]:
    del os.environ[k]
os.environ['CAU_NOI_CHINH'] = 'tien_trinh_khong_ton_tai_1 tien_trinh_khong_ton_tai_2'

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
sleep "${GIA_CHUYEN_S:-30}" &
wait
echo "DEN_NOI: $1 xong theo tuyen 1 (cach dich 0.05 m)."
exit 0
'''
for ten, nd in (('bat_xe.sh', BAT_XE), ('chuyen.sh', CHUYEN)):
    with open(os.path.join(TAM, ten), 'w') as f:
        f.write(nd)
sys.path.insert(0, HERE)
import cau_noi_xe as C  # noqa: E402

# md5 cua TRANG_NUT @579cda5 (trang /nut phai giu nguyen tung byte)
MD5_TRANG_NUT = '9a96075bd5ab866540f71e027608bcf1'


def http(port, method, path, body=None, pin=None):
    data = None if body is None else json.dumps(body).encode()
    rq = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), data=data, method=method)
    if pin is not None:
        rq.add_header('X-Pin', pin)
    try:
        with urllib.request.urlopen(rq, timeout=5) as r:
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


class CauNoiCu(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = C.ThreadingHTTPServer(('127.0.0.1', 0), C.H)
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        shutil.rmtree(TAM, ignore_errors=True)

    def setUp(self):
        cho(lambda: C.trang_thai_nut()['viec'] == 'rong', 15)
        try:
            os.remove(C.CFG['pin_file'])
        except OSError:
            pass

    def test_01_trang_nut_giu_nguyen(self):
        st, ct, b = http(self.port, 'GET', '/nut')
        self.assertEqual(st, 200)
        self.assertTrue(ct.startswith('text/html'))
        self.assertEqual(hashlib.md5(b).hexdigest(), hashlib.md5(C.TRANG_NUT.encode()).hexdigest())
        self.assertEqual(hashlib.md5(C.TRANG_NUT.encode()).hexdigest(), MD5_TRANG_NUT)

    def test_02_trang_thai_nut_khoa(self):
        st, j = js(self.port, 'GET', '/api/nut/trang_thai')
        self.assertEqual(st, 200)
        self.assertEqual(set(j), {'viec', 'tat', 'bat', 'chuyen', 'dung', 'sep'})
        self.assertEqual(j['viec'], 'rong')
        self.assertEqual(set(j['chuyen']), {'chay', 'huong', 'giay', 'ma', 'ket_qua', 'dong'})

    def test_03_di_B_409_dung(self):
        st, j = js(self.port, 'POST', '/api/nut/di_B')
        self.assertEqual((st, j['da_bat_dau'], j['viec']), (202, True, 'chuyen'))
        st, j = js(self.port, 'POST', '/api/nut/ve_A')
        self.assertEqual((st, j['da_bat_dau']), (409, False))
        st, j = js(self.port, 'POST', '/api/nut/bat')
        self.assertEqual(st, 409)
        time.sleep(0.3)
        st, j = js(self.port, 'POST', '/api/nut/dung')
        self.assertEqual(st, 200)
        self.assertEqual(j['ket_qua'], 'DA_DUNG')
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['chuyen']['chay']))
        c = C.trang_thai_nut()['chuyen']
        self.assertEqual((c['huong'], c['ma'], c['ket_qua']), ('di_B', 130, 'NGAT'))
        self.assertTrue(c['dong'].startswith('HUY_DICH_KHI_NGAT'))

    def test_04_dung_khi_khong_co_chuyen(self):
        st, j = js(self.port, 'POST', '/api/nut/dung')
        self.assertEqual((st, j['ket_qua']), (200, 'KHONG_CO_CHUYEN'))

    def test_05_bat_roi_tat(self):
        st, j = js(self.port, 'POST', '/api/nut/bat')
        self.assertEqual((st, j['viec']), (202, 'bat'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['bat']['chay']))
        b = C.trang_thai_nut()['bat']
        self.assertEqual(b['ma'], 0)
        self.assertTrue(b['dong'].startswith('SAN SANG'))
        st, j = js(self.port, 'POST', '/api/nut/tat')
        self.assertEqual((st, j['viec']), (202, 'tat'))
        self.assertTrue(cho(lambda: not C.trang_thai_nut()['tat']['chay']))
        t = C.trang_thai_nut()['tat']
        self.assertEqual((t['ma'], t['dong']), (0, 'DA_DON 0 tep'))

    def test_06_pin(self):
        with open(C.CFG['pin_file'], 'w') as f:
            f.write('2468\n')
        self.assertEqual(js(self.port, 'POST', '/api/nut/dung')[0], 401)
        self.assertEqual(js(self.port, 'POST', '/api/nut/dung', pin='1111')[0], 401)
        self.assertEqual(js(self.port, 'POST', '/api/nut/dung', pin='2468')[0], 200)
        self.assertEqual(js(self.port, 'POST', '/api/b/connect', {})[0], 401)

    def test_07_duong_cu_khac(self):
        st, j = js(self.port, 'GET', '/api/b/status')
        self.assertEqual(st, 200)
        self.assertEqual(j['link'], 'off')
        for k in ('link', 'connect_s', 'nav', 'drv', 'wait_s', 'pose', 'manual', 'photo', 'notified', 'han_s', 'thieu', 'chi_tiet', 'log_cuoi'):
            self.assertIn(k, j)
        self.assertEqual(js(self.port, 'POST', '/api/b/goal', {})[0], 501)
        self.assertEqual(js(self.port, 'GET', '/khong_co')[0], 404)
        self.assertEqual(js(self.port, 'POST', '/api/nut/khong_co')[0], 404)
        self.assertEqual(js(self.port, 'POST', '/khong_co')[0], 404)
        self.assertEqual(http(self.port, 'GET', '/')[0], 404)          # chon_b.html khong co trong HERE cua kho -> 404 nhu cu

    def test_08_doc_sep_tep_that(self):
        C.CFG['sep_nhat_ky'] = os.path.join(HERE, 'du_lieu_0511_toi', 'sep_nhat_ky.csv')
        try:
            s = C.doc_sep()
        finally:
            C.CFG['sep_nhat_ky'] = os.path.join(TAM, 'sep_nhat_ky.csv')
        self.assertTrue(s['tu_the'] and s['lenh'])


if __name__ == '__main__':
    unittest.main()
