#!/usr/bin/env python3
"""trang_v10.py (viec 111, 06/11/2026 dem, CHUA THU TREN XE): phan /v10 cua cau_noi_xe.py (trang dieu khien v10, mot trang).

cau_noi_xe.py chi chuyen moi yeu cau /v10... sang xu_ly(); loi o day tra 500 cho /v10, /nut va duong cu khong bi anh huong.
Khong can ROS. Chi DOC tep cua xe (khong sua tep lai): ~/sep_nhat_ky.csv, ~/log_phu_lai.txt, ~/log_nut_chuyen.txt,
~/sep_thoat, ~/gui_duong_luot, ~/ket_qua_chuyen.csv. Nut bam goi lai DUNG ham cua /nut (bat_dau_job, bat_dau_chuyen,
dung_chuyen, bat_dau_tat). Ma trang thai goc ASCII/tieng Viet; trang dich sang tieng Han luc in (han()).
Manh: M1 khung + trang thai; M4 di / dung / bat / tat (goi lai H._nut cua /nut: cung PIN, cung 409).
"""
import json
import os
import re
import time

TEP_TRANG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'trang_v10.html')
T_PHU_LAI_MOI = 3.0      # s: dong log phu lai cu hon so nay thi khong dung de suy trang thai (log dinh ky ~1 s)
T_THOAT_MOI = 3.0        # s: ~/sep_thoat cu hon so nay coi nhu khong co (sep ghi moi vong khi dang giao; T_RA2_TRA cua sep 3 s)

# ---------- M1: trang thai ----------
# Ma trang thai cua /v10 (chip, tieu de); trang dich bang HAN_V10 (dat truoc bang HAN cua /nut).
MA = ('TT_KHONG_KET_NOI', 'TT_DANG_KET_NOI', 'TT_SAN_SANG', 'TT_DANG_DI', 'TT_DANG_CHO', 'TT_DANG_THOAT', 'TT_DANG_XOAY',
      'TT_DA_DEN', 'TT_CAN_GIUP', 'TT_DANG_NGAT', 'TT_DA_NGAT')
# Ten trang thai phu lai (lai_xe_lidar, doc tu log) -> trang thai trang
PHU_LAI_THOAT = ('RA_TIEN', 'RA_LUI', 'THOAT_TIEN', 'THOAT_LUI', 'THOAT_XOAY', 'THOAT_CUNG')
PHU_LAI_XOAY = ('XOAY_P',)
PHU_LAI_CHO = ('CHO_SEP',)
SEP_CHO = ('CHO', 'VAT_LA_TRUOC')            # TU_THE cua sep -> dang cho
SEP_THOAT = ('KET_SAT_VAT',)
_RE_ROS = re.compile(r'^\[(\w+)\] \[(\d+(?:\.\d+)?)\] \[([^\]]+)\]: (.*)$')
_RE_L1_DOI = re.compile(r'^(\S+) -> (\S+) \(([^)]*)\)')       # giong sep._RE_L1_DOI
_RE_L1_DINH_KY = re.compile(r'trang_thai=(\S+)')             # giong sep._RE_L1_DINH_KY


def doc_duoi(path, n_byte=262144):
    """Cac dong cua phan cuoi tep (n_byte); [] neu khong doc duoc."""
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - n_byte))
            return f.read().decode('utf-8', 'ignore').splitlines()
    except OSError:
        return []


def phu_lai_cuoi(dong):
    """Trang thai phu lai moi nhat trong log ~/log_phu_lai.txt: (ten, ly_do, giay_epoch) hoac None.
    Dong ROS: '[INFO] [<epoch>] [lai_xe_lidar]: A -> B (ly) | ...' hoac '... trang_thai=B'."""
    for l in reversed(dong):
        m = _RE_ROS.match(l)
        if not m or m.group(3) != 'lai_xe_lidar':
            continue
        msg = m.group(4)
        d = _RE_L1_DOI.match(msg)
        if d:
            return d.group(2), d.group(3), float(m.group(2))
        d = _RE_L1_DINH_KY.search(msg)
        if d:
            return d.group(1), '', float(m.group(2))
    return None


def sep_cuoi(dong, tu_gio=''):
    """Tu ~/sep_nhat_ky.csv: TU_THE, LENH cuoi, vi tri cuoi (x, y, goc_do, gio) va co DUNG_VA_BAO tu `tu_gio` (chuoi gio
    'YYYY-mm-dd HH:MM:SS', so sanh chuoi) khong. Cot: gio_mono,gio,loai,dau_hieu,nguon_nghi,phep_kiem,ket_qua,se_lam,x,y,goc_do."""
    out = {'tu_the': None, 'lenh': None, 'pose': None, 'dung_va_bao': False, 'gio': ''}
    for l in dong:
        c = l.split(',')
        if len(c) < 11 or c[0] == 'gio_mono':
            continue
        out['gio'] = c[1]
        if c[2] == 'TU_THE':
            out['tu_the'] = (c[3], c[6], c[1])
        elif c[2] == 'LENH':
            out['lenh'] = (c[3], c[6], c[1])
        if c[8] and c[9]:
            try:
                out['pose'] = (float(c[8]), float(c[9]), float(c[10] or 0), c[1])
            except ValueError:
                pass
        if 'DUNG_VA_BAO' in l and c[1] >= tu_gio:
            out['dung_va_bao'] = True
    return out


def dong_chuyen_cuoi(dong):
    """Dong co nghia cuoi cua ~/log_nut_chuyen.txt (stdout gui_duong.py) -> 'THOAT' | 'CHO' | 'DI' | None."""
    for l in reversed(dong):
        s = l.strip()
        if 'lop 2: sep bao RA_' in s:
            return 'THOAT'
        if 'sep bao CHO' in s or s.startswith('khong den duoc') or 'khong den duoc (' in s:
            return 'CHO'
        if s.startswith('-- tuyen') or 'RA_XONG' in s or 'RA_HET_GIO' in s or 'tinh lai duong' in s:
            return 'DI'
    return None


def thoat_moi(dong, t_boot):
    """~/sep_thoat '<giay BOOTTIME> CHU ...': CHU neu con moi (<= T_THOAT_MOI s), nguoc lai None."""
    o = (dong or '').split()
    if len(o) < 2:
        return None
    try:
        t = float(o[0])
    except ValueError:
        return None
    return o[1] if -1.0 <= t_boot - t <= T_THOAT_MOI else None


def suy_trang_thai(link, nut, sep, phu_lai, chuyen_log, thoat, t_epoch):
    """Ma trang thai /v10 tu du lieu that. link: trang_thai() cua cau noi; nut: trang_thai_nut(); sep: sep_cuoi();
    phu_lai: phu_lai_cuoi(); chuyen_log: dong_chuyen_cuoi(); thoat: thoat_moi(). Tra (ma, ly_do)."""
    if nut['tat']['chay']:
        return 'TT_DANG_NGAT', 'bat_xe.sh tat'
    if nut['bat']['chay'] or link['link'] == 'connecting':
        return 'TT_DANG_KET_NOI', 'bat_xe.sh'
    if link['link'] == 'off':
        if nut['tat']['ma'] == 0:
            return 'TT_DA_NGAT', nut['tat']['dong']
        return 'TT_KHONG_KET_NOI', ''
    if link['link'] != 'on':
        return 'TT_KHONG_KET_NOI', link.get('chi_tiet') or 'fail'
    c = nut['chuyen']
    if c['chay']:
        if sep['dung_va_bao']:
            return 'TT_CAN_GIUP', 'sep DUNG_VA_BAO'
        pl = phu_lai if phu_lai and t_epoch - phu_lai[2] <= T_PHU_LAI_MOI else None
        if (pl and pl[0] in PHU_LAI_THOAT) or thoat in ('THOAT', 'RA_LUI', 'RA_TIEN') or chuyen_log == 'THOAT':
            return 'TT_DANG_THOAT', pl[0] if pl and pl[0] in PHU_LAI_THOAT else (thoat or 'gui_duong lop 2')
        if pl and pl[0] in PHU_LAI_XOAY:
            return 'TT_DANG_XOAY', pl[0]
        tu_the = sep['tu_the'][0] if sep['tu_the'] else ''
        if (pl and (pl[0] in PHU_LAI_CHO or pl[0].startswith('CHO_'))) or tu_the in SEP_CHO or chuyen_log == 'CHO':
            return 'TT_DANG_CHO', pl[0] if pl and (pl[0] in PHU_LAI_CHO or pl[0].startswith('CHO_')) else (tu_the if tu_the in SEP_CHO else 'gui_duong cho')
        if tu_the in SEP_THOAT:
            return 'TT_DANG_THOAT', tu_the
        return 'TT_DANG_DI', ''
    if c['ket_qua'] == 'DEN_NOI':
        return 'TT_DA_DEN', c['dong']
    if c['ket_qua'] in ('HET_DUONG', 'DUNG_SEP'):
        return 'TT_CAN_GIUP', c['ket_qua']
    return 'TT_SAN_SANG', ''


def boot_giay():
    try:
        with open('/proc/uptime') as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return 0.0


def doc_dong_dau(path):
    try:
        with open(path) as f:
            return f.readline()
    except OSError:
        return ''


def tep(C, ten):
    """Duong dan tep cua xe (HOME cau noi). CFG cua cau noi co thi dung, khong thi ~/<ten>."""
    return C.CFG.get('v10_' + ten) or os.path.join(C.HOME, ten)


def trang_thai(C):
    link = C.trang_thai()
    nut = C.trang_thai_nut()
    tu_gio = ''
    if nut['chuyen']['chay'] and nut['chuyen']['giay'] is not None:
        tu_gio = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time() - nut['chuyen']['giay']))
    sep = sep_cuoi(doc_duoi(C.CFG['sep_nhat_ky']), tu_gio or '9999')
    pl = phu_lai_cuoi(doc_duoi(tep(C, 'log_phu_lai.txt'), 65536))
    cl = dong_chuyen_cuoi(doc_duoi(C.CFG['log_chuyen'], 65536)) if nut['chuyen']['chay'] else None
    th = thoat_moi(doc_dong_dau(tep(C, 'sep_thoat')), boot_giay())
    now = time.time()
    ma, ly = suy_trang_thai(link, nut, sep, pl, cl, th, now)
    return {
        'ma': ma, 'ly_do': ly, 'link': link['link'], 'thieu': link['thieu'], 'chi_tiet': link['chi_tiet'],
        'han_s': link['han_s'], 'viec': nut['viec'], 'connect_s': link['connect_s'],
        'chuyen': nut['chuyen'], 'bat': nut['bat'], 'tat': nut['tat'], 'dung': nut['dung'],
        'pin': os.path.exists(C.CFG['pin_file']),
    }


# ---------- HTTP ----------

def han_js(C):
    """Bang HAN + ham han() cua trang /nut (lay nguyen van tu TRANG_NUT, khong chep tay) + bang HAN_V10 dat truoc."""
    t = C.TRANG_NUT
    a = t.index('var HAN=[')
    b = t.index('var HAN_TU=')
    e = t.index('\n}\n', t.index('function han(s){')) + 3
    return 'var HAN_V10=' + json.dumps(HAN_V10, ensure_ascii=False) + ';\n' + t[a:b] + 'HAN=HAN_V10.concat(HAN);\n' + t[b:e]


HAN_V10 = [
    ['TT_KHONG_KET_NOI', '연결 안 됨'], ['TT_DANG_KET_NOI', '연결 중'], ['TT_SAN_SANG', '준비 완료'],
    ['TT_XEM_TUYEN', '경로 확인'], ['TT_DANG_DI', '이동 중'], ['TT_DANG_CHO', '대기 중'], ['TT_DANG_THOAT', '탈출 중'],
    ['TT_DANG_XOAY', '방향 맞추는 중'], ['TT_DA_DEN', '도착'], ['TT_LAI_TAY', '수동 조종'], ['TT_CAN_GIUP', '도움이 필요해요'],
    ['TT_DANG_NGAT', '종료 중'], ['TT_DA_NGAT', '연결 해제됨'],
]


# ---------- M4: di / dung / bat / tat ----------
LENH_NUT = {'bat': 'bat', 'tat': 'tat', 'dung': 'dung'}       # /v10/api/<lenh> -> /api/nut/<lenh> (nguyen ham cua /nut)
DICH_NUT = {'B': 'di_B', 'A': 've_A'}                         # 출발: dich B -> 'B로 출발', A -> 'A로 복귀'


def doc_than(h):
    try:
        n = int(h.headers.get('Content-Length') or 0)
        return json.loads(h.rfile.read(n) or b'{}') if n else {}
    except (ValueError, OSError):
        return {}


def lenh(h, C, ten):
    """POST /v10/api/<ten>: chuyen nguyen sang H._nut (PIN, 409, ma tra ve y nhu /nut)."""
    if ten in LENH_NUT:
        return h._nut(LENH_NUT[ten])
    if ten == 'di':
        d = doc_than(h).get('dich')
        if d not in DICH_NUT:
            return h._gui(400, {'loi': 'dich phai la B hoac A', 'dich': d})
        return h._nut(DICH_NUT[d])
    return h._gui(404, {'loi': 'khong co duong nay'})


def xu_ly(h, C, method, p):
    """Mot yeu cau /v10... (h: BaseHTTPRequestHandler cua cau noi, C: mo dun cau_noi_xe)."""
    if method == 'GET' and p in ('/v10', '/v10/'):
        try:
            with open(C.CFG.get('trang_v10') or TEP_TRANG, encoding='utf-8') as f:
                html = f.read()
        except OSError:
            return h._gui(404, {'loi': 'khong thay tep', 'tep': 'trang_v10.html'})
        return h._gui(200, html.replace('/*HAN_JS*/', han_js(C)), 'text/html; charset=utf-8')
    if method == 'GET' and p == '/v10/trang_thai':
        return h._gui(200, trang_thai(C))
    if method == 'POST' and p.startswith('/v10/api/'):
        return lenh(h, C, p[len('/v10/api/'):])
    return h._gui(404, {'loi': 'khong co duong nay'})
