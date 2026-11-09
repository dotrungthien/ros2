#!/usr/bin/env python3
"""trang_v10.py (viec 111, 06/11/2026 dem, CHUA THU TREN XE): phan /v10 cua cau_noi_xe.py (trang dieu khien v10, mot trang).

cau_noi_xe.py chi chuyen moi yeu cau /v10... sang xu_ly(); loi o day tra 500 cho /v10, /nut va duong cu khong bi anh huong.
Khong can ROS. Chi DOC tep cua xe (khong sua tep lai): ~/sep_nhat_ky.csv, ~/log_phu_lai.txt, ~/log_nut_chuyen.txt,
~/sep_thoat, ~/gui_duong_luot, ~/ket_qua_chuyen.csv. Nut bam goi lai DUNG ham cua /nut (bat_dau_job, bat_dau_chuyen,
dung_chuyen, bat_dau_tat). Ma trang thai goc ASCII/tieng Viet; trang dich sang tieng Han luc in (han()).
Manh: M1 khung + trang thai; M4 di / dung / bat / tat (goi lai H._nut cua /nut: cung PIN, cung 409);
  M2 ban do + vung (luat 8: ban do tu tep slam_dinh_vi_xuong.yaml tro toi, footprint / inflation tu tep tham so Nav2
  NAV_PARAMS; thieu tep => loi tren trang, khong so du phong; 09/11 20:33: ban do chi to tuong (xam) / san (xanh), vung do / vang
  VE QUANH XE theo tam xe (vung_xe, so tu hanh_vi_xe.yaml); MEM / LECH cua gui_duong (mem_vung));
  M3 경로 확인: tinh_tuyen.tinh_duong (import, khong sua) tu vi tri xe toi dich cua gui_duong.DICH, cung thu tu tuyen voi
  /nut (chuyen.sh khong doi so thu 2 => tuyen 1 truoc, hong thi tuyen 2). Chi B chuan va A (CHON_B_TU_DO tat: gui_duong chi nhan di_B / ve_A);
  M5 dong 두뇌 + bang 7 dong: chi doc ~/sep_nhat_ky.csv, ~/log_phu_lai.txt, ~/log_nut_chuyen.txt, /proc (ten + tham so true/false
  cua sep.py, lai_xe_lidar.py, gui_duong.py; KHONG doc tham so khac), twist_mux.yaml (uu tien), lai_xe_lidar.R_XOAY;
  M7 도움이 필요해요: anh LiDAR tu tren xuong tu tep chup /scan moi nhat (~/scan_*.txt, 'ros2 topic echo --once /scan'),
  doi sang khung than bang lai_xe_lidar.scan_to_xy (goc lech 0 nhu tham so mac dinh cua phu lai); 다시 시도 = /nut cung huong;
  M6 수동 조종: pad gui {l, a} (-1..1) moi 0,1 s khi giu nut; cau noi chuyen sang teleop_web (POST /cmd, cong 5000, cookie
  dang nhap bang ma doc tu bien moi truong cung ten voi bat_xe.sh). Tha nut = gui 0 roi ngung; teleop_web tu ngung sau cmd_timeout.
"""
import http.client
import json
import math
import os
import re
import struct
import sys
import threading
import time
import urllib.parse
import zlib

TEP_TRANG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'trang_v10.html')
T_PHU_LAI_MOI = 3.0      # s: dong log phu lai cu hon so nay thi khong dung de suy trang thai (log dinh ky ~1 s)
T_THOAT_MOI = 3.0        # s: ~/sep_thoat cu hon so nay coi nhu khong co (sep ghi moi vong khi dang giao; T_RA2_TRA cua sep 3 s)
T_VI_TRI_MOI = 3.0       # s: 09/11 buoc 3 #3: ~/vi_tri_xe (gui_duong kieu mat ghi moi 1 s) cu hon so nay => dung vi tri cua sep nhu cu

# ---------- M1: trang thai ----------
# Ma trang thai cua /v10 (chip, tieu de); trang dich bang HAN_V10 (dat truoc bang HAN cua /nut).
MA = ('TT_KHONG_KET_NOI', 'TT_DANG_KET_NOI', 'TT_SAN_SANG', 'TT_DANG_DI', 'TT_DANG_CHO', 'TT_DANG_THOAT', 'TT_DANG_XOAY',
      'TT_DA_DEN', 'TT_CAN_GIUP', 'TT_DANG_NGAT', 'TT_DA_NGAT')
# Ten trang thai phu lai (lai_xe_lidar, doc tu log) -> trang thai trang
PHU_LAI_THOAT = ('RA_TIEN', 'RA_LUI', 'THOAT_TIEN', 'THOAT_LUI', 'THOAT_XOAY', 'THOAT_CUNG')
PHU_LAI_XOAY = ('XOAY_P',)
PHU_LAI_CHO = ('CHO_SEP',)
# TU_THE cua sep (DI_THUONG / CHO / THAN_TRONG / DUNG_VA_BAO) KHONG dung de suy 대기/탈출: sep co the chi nhin (ra_lenh=false);
# trang thai lay tu cai dang thuc thi (phu lai, gui_duong). Y nghi cua sep hien o dong 두뇌. (D(1) gia lap 06/11: TU_THE CHO
# 'KHONG_CO_DICH' dau chuyen lam trang hien 대기 중 sai.)
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


def chuyen_cuoi_ct(dong):
    """Dong co nghia cuoi cua ~/log_nut_chuyen.txt (stdout gui_duong.py) -> ('THOAT' | 'CHO' | 'DI' | 'CHO_NGUOI' | None, dong da strip)."""
    for l in reversed(dong):
        s = l.strip()
        # 09/11 18:47 (i): gui_duong het vong khe => 'CHO_NGUOI: ...' (dung yen, tien trinh song, cho nguoi bam 정지 / tu thu lai)
        if s.startswith('CHO_NGUOI:') and not s.startswith(('CHO_NGUOI: het ', 'CHO_NGUOI: sep het ')):     # 2 dong nay = di tiep
            return 'CHO_NGUOI', s
        # 09/11 buoc 3 #6: gui_duong kieu mat 'LUI_RA: bt|gui <m>' (Nav2 BackUp) = 탈출; '   LUI_XONG: ...' = xong lui
        if 'lop 2: sep bao RA_' in s or s.startswith('LUI_RA:'):
            return 'THOAT', s
        # 08/11 bo 1e: gui_duong kieu mat in 'CHO_VAT: n/30 s' khi dung yen, 'DI_TIEP: ...' khi tien lai; 09/11 #2 'GAN_DICH: ...'
        if ('sep bao CHO' in s or s.startswith('khong den duoc') or 'khong den duoc (' in s or s.startswith('CHO_VAT:')
                or s.startswith('GAN_DICH:')):
            return 'CHO', s
        if (s.startswith('-- tuyen') or 'RA_XONG' in s or 'RA_HET_GIO' in s or 'tinh lai duong' in s or s.startswith('DI_TIEP:')
                or s.startswith('LUI_XONG:')):
            return 'DI', s
    return None, ''


def dong_chuyen_cuoi(dong):
    """-> 'THOAT' | 'CHO' | 'DI' | 'CHO_NGUOI' | None (chuyen_cuoi_ct)."""
    return chuyen_cuoi_ct(dong)[0]


_RE_CHO_VAT = re.compile(r'^CHO_VAT: (\d+)/(\d+) s')


def cho_vat_so(s):
    """09/11 buoc 3 #4: 'CHO_VAT: n/N s ...' cua gui_duong -> [n, N] (so chinh cua trang khi 대기); khac => None."""
    m = _RE_CHO_VAT.match((s or '').strip())
    return [int(m.group(1)), int(m.group(2))] if m else None


def doc_vi_tri_xe(text, now, tuoi_max=T_VI_TRI_MOI):
    """09/11 buoc 3 #3: ~/vi_tri_xe (gui_duong.noi_dung_vi_tri) -> {'x','y','goc_do','gio','tuoi_s','duong': [[x, y], ...]}
    neu con moi (<= tuoi_max s) va co vi tri; nguoc lai None."""
    d = (text or '').splitlines()
    try:
        o = d[0].split()
        t = float(o[0])
        x, y, g = float(o[1]), float(o[2]), float(o[3])
    except (IndexError, ValueError):
        return None
    if not -1.0 <= now - t <= tuoi_max:
        return None
    duong = []
    q = d[1].split() if len(d) > 1 else []
    if len(q) >= 2 and q[0] == 'plan':
        try:
            v = [float(a) for a in q[2:]]
            duong = [[v[k], v[k + 1]] for k in range(0, len(v) - 1, 2)]
        except ValueError:
            duong = []
    return {'x': x, 'y': y, 'goc_do': g, 'gio': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)),
            'tuoi_s': round(now - t, 1), 'duong': duong}


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


DONG_DA_HUY = re.compile(r'^LOI_GUI_DUONG .*; HUY_DICH ok\s*$')


def suy_trang_thai(link, nut, sep, phu_lai, chuyen_log, thoat, t_epoch, phu_lai_lai=True, chuyen_dong='', den_noi_sau=False):
    """Ma trang thai /v10 tu du lieu that. link: trang_thai() cua cau noi; nut: trang_thai_nut(); sep: sep_cuoi();
    phu_lai: phu_lai_cuoi(); chuyen_log: dong_chuyen_cuoi(); thoat: thoat_moi(). Tra (ma, ly_do).
    08/11 bo 1e: phu_lai_lai False (lai_xe_lidar chay voi phat_lenh false, kieu mat) => KHONG dung trang thai phu lai de hien
    탈출 / 회전 / 대기 (no khong lai). 09/11 #6: chuyen_dong = dong co nghia cuoi (chuyen_cuoi_ct); 'LUI_RA: ...' => ly do 탈출."""
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
        if chuyen_log == 'CHO_NGUOI':                # 09/11 18:47 (i): chuyen con song, xe dung yen cho nguoi (nut 정지)
            return 'TT_CAN_GIUP', 'CHO_NGUOI'
        pl = phu_lai if phu_lai and phu_lai_lai and t_epoch - phu_lai[2] <= T_PHU_LAI_MOI else None
        if (pl and pl[0] in PHU_LAI_THOAT) or thoat in ('THOAT', 'RA_LUI', 'RA_TIEN') or chuyen_log == 'THOAT':
            return 'TT_DANG_THOAT', pl[0] if pl and pl[0] in PHU_LAI_THOAT else (
                thoat or (chuyen_dong if chuyen_dong.startswith('LUI_RA:') else 'gui_duong lop 2'))
        if pl and pl[0] in PHU_LAI_XOAY:
            return 'TT_DANG_XOAY', pl[0]
        if (pl and (pl[0] in PHU_LAI_CHO or pl[0].startswith('CHO_'))) or chuyen_log == 'CHO':
            return 'TT_DANG_CHO', pl[0] if pl and (pl[0] in PHU_LAI_CHO or pl[0].startswith('CHO_')) else 'gui_duong cho'
        # 08/11 111-Q3: log chuyen chua co dong co nghia (chi dong dau '== cau_noi_xe ...', hoac 'doi tuyen' / CANH BAO truoc
        # '-- tuyen'): gui_duong con cho vi tri + costmap, SAP_GUI, tinh duong => "경로 계산 중" (truoc day trang hien 이동 중)
        if chuyen_log is None:
            return 'TT_DANG_DI', 'CHUAN_BI'
        return 'TT_DANG_DI', ''
    # D(1) gia lap 06/11 03:56: SIGINT roi vao rclpy take_message => gui_duong.py chet vi RuntimeError (ma 1, khong HUY, khong NGAT);
    # /nut ghi HET_DUONG + DA_DUNG nhung dich FollowPath con song, xe di tiep 26 s toi B. Trang bao rieng, khong cho bo qua.
    # 07/11 111-Q2: ma 5 (LOI_GUI_DUONG) ma gui_duong bao Nav2 tra loi huy dich ('; HUY_DICH ok', gui_duong.py:197, :535) => dich da huy.
    # Xet tren nguyen dong (ly_do bi cat 120 ky tu). 'khong_tra_loi' / 'khong_co_dich' => giu LOI_THOAT (tat motor).
    if c['ma'] == 5 and DONG_DA_HUY.match(c['dong'] or ''):
        return 'TT_CAN_GIUP', 'LOI_DA_HUY ma 5: %s' % (c['dong'] or '')[:120]
    if c['ma'] is not None and (c['ma'] not in (0, 1, 3, 4, 130) or (c['ma'] == 1 and not (c['dong'] or '').startswith('HET_DUONG'))):
        return 'TT_CAN_GIUP', 'LOI_THOAT ma %s: %s' % (c['ma'], (c['dong'] or '')[:120])
    if c['ket_qua'] == 'DEN_NOI':
        return 'TT_DA_DEN', c['dong']
    if c['ket_qua'] in ('HET_DUONG', 'DUNG_SEP'):
        if den_noi_sau:                          # 09/11 18:44 ③: co chuyen DEN_NOI (ca terminal) sau chuyen web nay => khong khoa B
            return 'TT_DA_DEN', 'DEN_NOI sau %s' % c['ket_qua']
        return 'TT_CAN_GIUP', c['ket_qua']
    return 'TT_SAN_SANG', ''


def boot_giay():
    try:
        with open('/proc/uptime') as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return 0.0


def den_noi_cuoi(dong):
    """09/11 18:44 ③: epoch dong ket_qua DEN_NOI cuoi cua ~/ket_qua_chuyen.csv (cot gio, ..., ket_qua thu 6; gio may) hoac None."""
    for l in reversed(dong):
        c = l.split(',')
        if len(c) >= 6 and c[5] == 'DEN_NOI':
            try:
                return time.mktime(time.strptime(c[0], '%Y-%m-%d %H:%M:%S'))
            except ValueError:
                continue
    return None


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
    cl, cl_dong = chuyen_cuoi_ct(doc_duoi(C.CFG['log_chuyen'], 65536)) if nut['chuyen']['chay'] else (None, '')
    th = thoat_moi(doc_dong_dau(tep(C, 'sep_thoat')), boot_giay())
    now = time.time()
    # 08/11 bo 1e: phu lai chi nhin (phat_lenh false doc tu /proc) => khong lay trang thai phu lai; khong thay tien trinh => nhu cu
    pl_lai = tien_trinh_lai().get('lai_xe_lidar.py', {}).get('phat_lenh') != 'false'
    with C.LOCK:
        t_ket = C.CHUYEN.get('t_ket')                 # 09/11 18:44 ③: luc chuyen web cuoi ket thuc (bat moi => None)
    dn = den_noi_cuoi(doc_duoi(tep(C, 'ket_qua_chuyen.csv'), 65536)) if t_ket is not None else None
    ma, ly = suy_trang_thai(link, nut, sep, pl, cl, th, now, pl_lai, cl_dong, den_noi_sau=dn is not None and dn > t_ket)
    m5 = bang_m5(C, ma, sep, nut, ly)
    # 09/11 buoc 3 #3: ~/vi_tri_xe moi (gui_duong kieu mat, 1 s) thi lay vi tri + duong /plan cua no; khong thi vi tri cua sep nhu cu
    try:
        with open(tep(C, 'vi_tri_xe')) as f:
            vt = doc_vi_tri_xe(f.read(), now)
    except OSError:
        vt = None
    if vt is not None:
        pose = {'x': vt['x'], 'y': vt['y'], 'goc_do': vt['goc_do'], 'gio': vt['gio'], 'nguon': 'gui_duong'}
    else:
        pose = None if not sep['pose'] else dict(zip(('x', 'y', 'goc_do', 'gio'), sep['pose']), nguon='sep')
    return {
        'ma': ma, 'ly_do': ly, 'link': link['link'], 'thieu': link['thieu'], 'chi_tiet': link['chi_tiet'],
        'han_s': link['han_s'], 'viec': nut['viec'], 'connect_s': link['connect_s'],
        'han_het': link['han_s'] is not None and link['han_s'] <= 0,      # 08/11 111-Q3: het 8 phut tu luc con quay lay so 0
        'chuyen': nut['chuyen'], 'bat': nut['bat'], 'tat': nut['tat'], 'dung': nut['dung'],
        'pin': os.path.exists(C.CFG['pin_file']),
        'pose': pose,                                   # M2: vi tri xe (09/11 #3: ~/vi_tri_xe neu moi, khong thi dong cuoi co x, y cua sep)
        'duong': vt['duong'] if vt is not None and nut['chuyen']['chay'] else [],      # 09/11 #3: /plan dang chay
        'cho_vat': cho_vat_so(cl_dong) if ma == 'TT_DANG_CHO' else None,               # 09/11 #4: so CHO_VAT cua gui_duong la chinh
        'bang': m5['bang'], 'nao': m5['nao'], 'nho': m5['nho'], 'uu_tien': m5['uu_tien'],                 # M5
    }


# ---------- M5: 두뇌 + "지금 누가 무엇을 하나" (chi doc) ----------
VUNG_TEN = ('TRUOC', 'TR-TRAI', 'TR-PHAI', 'TRAI', 'PHAI', 'SAU', 'SAU-TRAI', 'SAU-PHAI', 'TRONG')
_RE_VUNG = re.compile(r'(?<![A-Z-])([A-Z]+(?:-[A-Z]+)?)=(\d+(?:\.\d+)?)')
_RE_LAN = re.compile(r'^-- tuyen (\d+), lan (\d+)/(\d+)')
_RE_THAM = re.compile(r'^(phat_lenh|ra_lenh|ra_thoat|sep_ra_thoat):=(true|false|True|False)$')
NHO_LOAI = {('L1', 'BAT_DAU_THOAT'), ('L1', 'BAT_DAU_RA_XOAY'), ('TRUY_VET', 'KET_SAT_VAT'), ('TRUY_VET', 'VAT_LA_TRUOC')}  # TRUY_VET: chi dong BAT
R_GOP_NHO = 0.05         # m: cho ket cach nhau duoi so nay ve mot dau X


def vung_phu_lai(dong):
    """Dong log phu lai moi nhat co khoang cach vung ('TRUOC=0.27 ...'): ({vung: m}, epoch) hoac (None, None)."""
    for l in reversed(dong):
        m = _RE_ROS.match(l)
        if not m or m.group(3) != 'lai_xe_lidar':
            continue
        v = {k: float(x) for k, x in _RE_VUNG.findall(m.group(4)) if k in VUNG_TEN}
        if v:
            return v, float(m.group(2))
    return None, None


def lan_thu(dong):
    """Dong '-- tuyen r, lan l/L' cuoi cua gui_duong: (r, l, L) hoac None."""
    for l in reversed(dong):
        m = _RE_LAN.match(l.strip())
        if m:
            return tuple(int(x) for x in m.groups())
    return None


def nho_ket(dong):
    """Cho ket / chan sep ghi tu dong CHUYEN BAT_DAU cuoi: [[x, y, loai], ...] (nhat ky co x, y)."""
    out = []
    for l in dong:
        c = l.split(',')
        if len(c) < 11:
            continue
        if c[2] == 'CHUYEN' and c[3] == 'BAT_DAU':
            out = []
        elif (c[2], c[3]) in NHO_LOAI and c[8] and c[9] and (c[2] != 'TRUY_VET' or c[6] == 'BAT'):
            try:
                x, y = float(c[8]), float(c[9])
            except ValueError:
                continue
            if not any(math.hypot(x - a, y - b) < R_GOP_NHO for a, b, _ in out):
                out.append([x, y, c[3]])
    return out


def uu_tien(C):
    """Uu tien twist_mux tu tep cua xe (teleop / avoid / nav). Thieu tep => {'loi': ...}, khong so du phong."""
    p = os.environ.get('CAU_NOI_TWIST_MUX') or os.path.join(C.HOME, 'ros2_ws/install/my_bot/share/my_bot/config/twist_mux.yaml')
    try:
        with open(p) as f:
            txt = f.read()
    except OSError:
        return {'loi': 'khong doc duoc %s' % p}
    out = {'tep': p}
    for ten in ('teleop', 'avoid', 'nav'):
        m = re.search(r'^([ \t]+)%s:[ \t]*\n((?:\1[ \t]+.*\n)*)' % ten, txt + '\n', re.M)
        q = re.search(r'priority:[ \t]*(\d+)', m.group(2)) if m else None
        out[ten] = int(q.group(1)) if q else None
    return out


def tien_trinh_lai():
    """Tien trinh sep.py / lai_xe_lidar.py / gui_duong.py dang song va CHI cac tham so true/false da biet (_RE_THAM)."""
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % d, 'rb') as f:
                argv = [a.decode('utf-8', 'ignore') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if not argv or not os.path.basename(argv[0]).startswith('python3'):
            continue
        for ten in ('sep.py', 'lai_xe_lidar.py', 'gui_duong.py'):
            if ten in [os.path.basename(a) for a in argv[1:3]]:
                out[ten] = {m.group(1): m.group(2).lower() for m in (_RE_THAM.match(a) for a in argv) if m}
    return out


def gio_tuoi(gio, now):
    try:
        return int(now - time.mktime(time.strptime(gio, '%Y-%m-%d %H:%M:%S')))
    except (ValueError, OverflowError):
        return None


def bang_m5(C, ma, sep, nut, ly=''):
    dong_sep = doc_duoi(C.CFG['sep_nhat_ky'])
    dong_pl = doc_duoi(tep(C, 'log_phu_lai.txt'), 65536)
    dong_ch = doc_duoi(C.CFG['log_chuyen'], 65536)
    ut = uu_tien(C)
    tt = tien_trinh_lai()
    now = time.time()
    vung, t_vung = vung_phu_lai(dong_pl)
    pl = phu_lai_cuoi(dong_pl)
    ln = lan_thu(dong_ch) if nut['chuyen']['huong'] else None
    nho = nho_ket(dong_sep)
    xoay = None
    if sep['pose']:
        bd = ban_do(C)
        if not bd.get('loi'):
            ban, L = _CACHE_BAN['ban'], nap(C, 'lai_xe_lidar')
            x, y = sep['pose'][0], sep['pose'][1]
            kc = min((kc_toi_o(x, y, ban, i, j) for j in range(bd['h']) for i in range(bd['w'])
                      if bd['vung'][j * bd['w'] + i] == '0'
                      and abs(ban.ox + (i + .5) * ban.res - x) < 0.6 and abs(ban.oy + (j + .5) * ban.res - y) < 0.6), default=9.9)
            xoay = {'duoc': kc > L.R_XOAY, 'kc_m': round(kc, 3), 'r_xoay_m': round(L.R_XOAY, 4)}     # 20:33: bo trong_do (금지 theo ban do)
    pl_lenh = tt.get('lai_xe_lidar.py', {}).get('phat_lenh')
    if ma == 'TT_DANG_THOAT' and (ly or '').startswith('LUI_RA:'):          # 09/11 #6: Nav2 BackUp ra /cmd_vel_avoid (buoc 2g)
        ai = ('Nav2 BackUp (behavior_server)', 'avoid')
    elif ma in ('TT_DANG_THOAT', 'TT_DANG_XOAY'):
        ai = ('lai_xe_lidar', 'avoid') if pl_lenh == 'true' else ('lai_xe_lidar (chi nhin, phat_lenh=%s)' % pl_lenh, None)
    elif ma in ('TT_DANG_DI', 'TT_DANG_CHO'):
        ai = ('Nav2 (RPP)', 'nav')
    else:
        ai = ('', None)
    bang = {
        'buoc': (dong_chuyen_co_nghia(dong_ch) if nut['chuyen']['chay'] else '') or '',
        'ai_lai': ai[0], 'ai_uu_tien': ut.get(ai[1]) if ai[1] else None,
        'lan': list(ln) if ln else None,
        'vung': vung, 'vung_tuoi_s': None if t_vung is None else round(now - t_vung, 1),
        'xoay': xoay,
        'nho': len(nho),
        'code': tt,
        'phu_lai': pl[0] if pl else None,
        'mem': mem_vung(dong_ch) if nut['chuyen']['chay'] else None,      # 09/11 20:33: MEM / LECH cua gui_duong (앞/옆/뒤 물체)
    }
    nao = {'tu_the': list(sep['tu_the']) if sep['tu_the'] else None, 'lenh': list(sep['lenh']) if sep['lenh'] else None,
           'tuoi_s': gio_tuoi(sep['gio'], now) if sep['gio'] else None}
    return {'bang': bang, 'nao': nao, 'nho': nho, 'uu_tien': ut}


_RE_MEM = re.compile(r'^MEM: (truoc|hong|sau) (\d+) diem cach ([0-9.]+) m \((tuong|vat)\)(?: LECH ban do=(tuong|vat))?$')


def mem_vung(dong):
    """09/11 20:33: dong MEM cua gui_duong (log chuyen ghi moi moi chuyen) tu 'MEM: sach' cuoi -> {vung: {kc, loai, lech, bd}} (dong
    moi nhat moi vung) hoac None neu chua co dong MEM nao. gui_duong chi in MEM khi doi (mot bo nho moi chuyen, qua cac luot)."""
    out, co = {}, False
    for l in dong:
        s = l.strip()
        if s == 'MEM: sach':
            out, co = {}, True
            continue
        m = _RE_MEM.match(s)
        if m:
            co = True
            out[m.group(1)] = {'kc': float(m.group(3)), 'loai': m.group(4), 'lech': m.group(5) is not None, 'bd': m.group(5)}
    return out if co else None


def vung_xe(C):
    """09/11 20:33: so vung DO / VANG quanh xe (theo tam xe) tu ~/hanh_vi_xe.yaml cua xe (hanh_vi.py), mui = vung_phanh_m."""
    try:
        HV = nap(C, 'hanh_vi')
        return {'do': {'truoc': HV.an_toan('truoc_vat_do_m'), 'hong': HV.an_toan('cach_vat_toi_thieu_m'), 'sau': HV.an_toan('duoi_do_m'),
                       'mui': HV.so('vung_phanh_m')},
                'vang': {'truoc': HV.so('vang_truoc_m'), 'hong': HV.so('vang_hong_m'), 'sau': HV.so('vang_sau_m')}}
    except Exception as e:                       # noqa: BLE001 (thieu tep / khoa: trang khong ve vung, khong vo)
        return {'loi': str(e)[:120]}


def dong_chuyen_co_nghia(dong):
    """Dong cuoi khong rong cua log chuyen (stdout gui_duong), bo dong dau '== cau_noi_xe ...' va dong MEM: (20:33, chi la mat)."""
    for l in reversed(dong):
        s = l.strip()
        if s and not s.startswith(('== cau_noi_xe', 'MEM:')):
            return s[:160]
    return ''


# ---------- M2: ban do + vung ----------
NAV2_PADDING = 0.01      # footprint_padding mac dinh cua Nav2 Humble (costmap_2d) khi tep tham so khong ghi; ghi thi dung so trong tep
_CACHE_BAN = {'khoa': None, 'kq': None}


class LoiBanDo(Exception):
    pass


def thu_muc_xe(C):
    """Thu muc chua tinh_tuyen.py / gui_duong.py / sep.py cua xe (tren Pi: ~). CAU_NOI_THU_MUC_XE de doi (test)."""
    return os.environ.get('CAU_NOI_THU_MUC_XE') or C.HOME


def nap(C, ten):
    d = thu_muc_xe(C)
    if d not in sys.path:
        sys.path.insert(0, d)
    return __import__(ten)


def doc_png(path):
    """PNG 8 bit khong xen ke (xam, xam+alpha, RGB, RGBA) -> (w, h, list hang [tren truoc] cac gia tri xam 0..255).
    Thuan Python (zlib) de cau noi khong can PIL."""
    with open(path, 'rb') as f:
        b = f.read()
    if b[:8] != b'\x89PNG\r\n\x1a\n':
        raise LoiBanDo('khong phai PNG: %s' % path)
    i, idat, w = 8, b'', None
    while i < len(b):
        n, typ = struct.unpack('>I4s', b[i:i + 8])
        d = b[i + 8:i + 8 + n]
        if typ == b'IHDR':
            w, h, bit, mau, _, _, xen = struct.unpack('>IIBBBBB', d)
            if bit != 8 or xen != 0 or mau not in (0, 2, 4, 6):
                raise LoiBanDo('PNG chua ho tro (bit %d, mau %d, xen ke %d): %s' % (bit, mau, xen, path))
        elif typ == b'IDAT':
            idat += d
        elif typ == b'IEND':
            break
        i += 12 + n
    if w is None:
        raise LoiBanDo('PNG thieu IHDR: %s' % path)
    kenh = {0: 1, 2: 3, 4: 2, 6: 4}[mau]
    raw = zlib.decompress(idat)
    st = w * kenh
    hang, truoc = [], bytearray(st)
    for r in range(h):
        f = raw[r * (st + 1)]
        cur = bytearray(raw[r * (st + 1) + 1:(r + 1) * (st + 1)])
        for x in range(st):
            a = cur[x - kenh] if x >= kenh else 0
            c = truoc[x - kenh] if x >= kenh else 0
            u = truoc[x]
            if f == 1:
                cur[x] = (cur[x] + a) & 255
            elif f == 2:
                cur[x] = (cur[x] + u) & 255
            elif f == 3:
                cur[x] = (cur[x] + (a + u) // 2) & 255
            elif f == 4:
                pa, pb, pc = abs(u - c), abs(a - c), abs(a + u - 2 * c)
                cur[x] = (cur[x] + (a if pa <= pb and pa <= pc else (u if pb <= pc else c))) & 255
        truoc = cur
        if kenh in (1, 2):                           # xam(+alpha): kenh 0; RGB(A): trung binh 3 kenh dau (nhu map_server)
            hang.append([cur[x * kenh] for x in range(w)])
        else:
            hang.append([(cur[x * kenh] + cur[x * kenh + 1] + cur[x * kenh + 2]) // 3 for x in range(w)])
    return w, h, hang


def doc_khoa(text, khoa):
    m = re.search(r'^[ \t]*%s:[ \t]*(.+?)[ \t]*(?:#.*)?$' % re.escape(khoa), text, re.M)
    return m.group(1).strip().strip('"\'') if m else None


def tep_ban_do(C):
    """slam_dinh_vi_xuong.yaml -> map_file_name -> tep .yaml anh ban do (<ten>.yaml, hoac bo duoi _pg cua posegraph)."""
    # nhu bat_xe.sh: SLAM="${SLAM:-/home/project1/slam_dinh_vi_xuong.yaml}" (bien SLAM trong cau_noi.env ap cho ca hai)
    slam = os.environ.get('CAU_NOI_SLAM') or os.environ.get('SLAM') or os.path.join(C.HOME, 'slam_dinh_vi_xuong.yaml')
    try:
        with open(slam) as f:
            txt = f.read()
    except OSError:
        raise LoiBanDo('khong doc duoc %s' % slam)
    m = doc_khoa(txt, 'map_file_name')
    if not m:
        raise LoiBanDo('%s khong co map_file_name' % slam)
    ung = [m + '.yaml'] + ([m[:-3] + '.yaml'] if m.endswith('_pg') else [])
    for u in ung:
        if os.path.exists(u):
            return slam, u
    raise LoiBanDo('khong thay anh ban do cho map_file_name %s (da tim: %s)' % (m, ', '.join(ung)))


def doc_ban_do(map_yaml):
    """map_server: tep yaml + anh -> (w, h, res, ox, oy, occ): occ[j*w+i] (hang 0 o duoi) = 100 vat, 0 trong, -1 chua biet."""
    with open(map_yaml) as f:
        txt = f.read()
    anh = doc_khoa(txt, 'image')
    res, org = doc_khoa(txt, 'resolution'), doc_khoa(txt, 'origin')
    occ_t, free_t = doc_khoa(txt, 'occupied_thresh'), doc_khoa(txt, 'free_thresh')
    neg = doc_khoa(txt, 'negate')
    if None in (anh, res, org, occ_t, free_t):
        raise LoiBanDo('%s thieu image / resolution / origin / occupied_thresh / free_thresh' % map_yaml)
    res, occ_t, free_t, neg = float(res), float(occ_t), float(free_t), int(neg or 0)
    o = [float(v) for v in org.strip('[]').split(',')]
    p = anh if os.path.isabs(anh) else os.path.join(os.path.dirname(map_yaml), anh)
    if p.endswith('.pgm'):
        w, h, px = _doc_pgm(p)
        hang = [list(px[r * w:(r + 1) * w]) for r in range(h)]
    else:
        w, h, hang = doc_png(p)
    occ = []
    for j in range(h):
        r = hang[h - 1 - j]
        for i in range(w):
            v = r[i] / 255.0 if neg else (255 - r[i]) / 255.0
            occ.append(100 if v > occ_t else (0 if v < free_t else -1))
    return w, h, res, o[0], o[1], occ, p


def _doc_pgm(p):
    import tinh_tuyen as TT   # doc_pgm cua tinh_tuyen (P5)
    return TT.doc_pgm(p)


def tep_nav2(C):
    """Tep tham so Nav2 nhu bat_xe.sh: bien NAV_PARAMS, chua co thi dong NAV_PARAMS= trong ~/cau_noi/cau_noi.env."""
    p = os.environ.get('NAV_PARAMS')
    if not p:
        try:
            with open(os.path.join(C.HOME, 'cau_noi', 'cau_noi.env')) as f:
                for l in f:
                    if l.startswith('NAV_PARAMS='):
                        p = l.split('=', 1)[1].strip().strip('"\'')
        except OSError:
            pass
    if not p:
        raise LoiBanDo('khong biet tep tham so Nav2 (NAV_PARAMS chua dat, ~/cau_noi/cau_noi.env khong co dong NAV_PARAMS=)')
    if not os.path.exists(p):
        raise LoiBanDo('khong thay tep tham so Nav2 %s' % p)
    return p


def doc_nav2(p):
    """Khoi global_costmap cua tep tham so Nav2: footprint, footprint_padding (neu co), inflation_radius, cost_scaling_factor."""
    with open(p) as f:
        txt = f.read()
    m = re.search(r'^global_costmap:[ \t]*\n((?:[ \t].*\n|[ \t]*\n|#.*\n)*)', txt + '\n', re.M)
    if not m:
        raise LoiBanDo('%s khong co khoi global_costmap' % p)
    k = m.group(1)
    fp, inf, sc, pad = doc_khoa(k, 'footprint'), doc_khoa(k, 'inflation_radius'), doc_khoa(k, 'cost_scaling_factor'), doc_khoa(k, 'footprint_padding')
    if fp is None or inf is None or sc is None:
        raise LoiBanDo('%s (global_costmap) thieu footprint / inflation_radius / cost_scaling_factor' % p)
    return {'footprint': json.loads(fp), 'inflation_radius': float(inf), 'cost_scaling_factor': float(sc),
            'footprint_padding': NAV2_PADDING if pad is None else float(pad), 'padding_tu_tep': pad is not None}


def ban_kinh_noi(fp, pad):
    """Nav2: footprint nong them pad (moi dinh ra xa tam theo dau x, y), ban kinh noi tiep = khoang cach nho nhat tam -> canh."""
    q = [(x + math.copysign(pad, x), y + math.copysign(pad, y)) for x, y in fp]
    r = 1e9
    for a, b in zip(q, q[1:] + q[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, -(a[0] * dx + a[1] * dy) / (dx * dx + dy * dy)))
        r = min(r, math.hypot(a[0] + t * dx, a[1] + t * dy))
    return r


def luoi_chi_phi(w, h, res, occ, inf_r, scale, r_noi):
    """Costmap Nav2 (Humble 1.1.20) tu ban do tinh: 254 o vat, 253 trong ban kinh noi tiep, 252*exp(-scale*(d - r_noi))
    toi inflation_radius (d = khoang cach tam o toi tam o vat gan nhat), doi sang OccupancyGrid nhu /global_costmap/costmap."""
    vat = [(k % w, k // w) for k, v in enumerate(occ) if v == 100]
    n = int(math.ceil(inf_r / res)) + 1
    gan = {}
    for a, b in vat:
        for i in range(max(0, a - n), min(w, a + n + 1)):
            for j in range(max(0, b - n), min(h, b + n + 1)):
                d = math.hypot(i - a, j - b) * res
                if d < gan.get((i, j), 1e9):
                    gan[(i, j)] = d
    out = []
    for j in range(h):
        for i in range(w):
            k = j * w + i
            if occ[k] == 100:
                c = 254
            else:
                d = gan.get((i, j), 1e9)
                c = 253 if d <= r_noi else (int(252 * math.exp(-scale * (d - r_noi))) if d <= inf_r else 0)
                if occ[k] == -1 and c < 253:      # Nav2: o chua biet chi bi ghi de khi >= noi tiep (inflate_unknown false)
                    c = 255
            out.append(0 if c == 0 else (100 if c == 254 else (99 if c == 253 else (-1 if c == 255 else 1 + (97 * (c - 1)) // 251))))
    return out


def ban_do(C):
    """JSON cho /v10/ban_do (cache theo mtime cac tep nguon). Loi => {'loi': ...}."""
    try:
        slam, my = tep_ban_do(C)
        nv = tep_nav2(C)
        TT = nap(C, 'tinh_tuyen')
        anh_p = None
        khoa = []
        for f in (slam, my, nv, TT.__file__):
            khoa.append((f, os.stat(f).st_mtime_ns))
        if _CACHE_BAN['khoa'] == khoa:
            return dict(_CACHE_BAN['kq'], vung_xe=vung_xe(C))
        w, h, res, ox, oy, occ, anh_p = doc_ban_do(my)
        n2 = doc_nav2(nv)
        r_noi = ban_kinh_noi(n2['footprint'], n2['footprint_padding'])
        data = luoi_chi_phi(w, h, res, occ, n2['inflation_radius'], n2['cost_scaling_factor'], r_noi)
        ban = TT.Ban.tu_luoi(w, h, res, ox, oy, data)
        # 09/11 20:33 vung quanh xe: ban do chi nhac => chi to tuong / chua biet (0, xam) va san (3, xanh); bo 차량 중심 금지 (1) va
        # 주의 구역 (2) theo ban do. Vung do / vang VE QUANH XE (vung_xe, theo tam xe) tren trang.
        vung = ''.join('0' if occ[j * w + i] != 0 and (occ[j * w + i] == 100 or data[j * w + i] == -1) else '3'
                       for j in range(h) for i in range(w))
        kq = {'loi': None, 'w': w, 'h': h, 'res': res, 'ox': ox, 'oy': oy, 'vung': vung,
              'footprint': n2['footprint'], 'inflation_radius': n2['inflation_radius'],
              'cost_scaling_factor': n2['cost_scaling_factor'], 'footprint_padding': n2['footprint_padding'],
              'padding_tu_tep': n2['padding_tu_tep'], 'r_noi_tiep': round(r_noi, 4), 'nguong_cam': TT.NGUONG_CAM,
              'A': list(TT.A), 'B': list(TT.B),
              'nguon': {'slam': slam, 'ban_do': my, 'anh': anh_p, 'nav2': nv, 'tinh_tuyen': TT.__file__}}
        _CACHE_BAN.update(khoa=khoa, kq=kq, ban=ban, data=data)
        return dict(kq, vung_xe=vung_xe(C))
    except (LoiBanDo, OSError, ValueError, KeyError, IndexError, zlib.error) as e:
        return {'loi': str(e)}


# ---------- M3: 경로 확인 ----------
CHON_B_TU_DO = False     # chon o xanh bat ky lam dich: TAT (gui_duong.py chi nhan di_B / ve_A; mo can sua tep lai -> "can quyet")
HUONG_DICH = {'B': 'di_B', 'A': 've_A'}


def kc_toi_o(x, y, ban, i, j):
    """Khoang cach (m) tu diem toi hinh vuong o (i, j); 0 neu diem nam trong o."""
    x0, y0 = ban.ox + i * ban.res, ban.oy + j * ban.res
    dx = max(x0 - x, 0.0, x - (x0 + ban.res))
    dy = max(y0 - y, 0.0, y - (y0 + ban.res))
    return math.hypot(dx, dy)


def tuyen(C, dich, pose):
    """Duong xem truoc cho /v10/tuyen: {'dich','huong','tuyen','diem','dai_m','kc_do_m','dau_trong_do','canh_bao'} hoac {'loi'}."""
    if dich not in HUONG_DICH:
        return {'loi': 'dich phai la B hoac A (CHON_B_TU_DO tat)'}
    if pose is None:
        return {'loi': 'chua co vi tri xe (sep chua ghi dong nao co x, y trong ~/sep_nhat_ky.csv)'}
    bd = ban_do(C)
    if bd.get('loi'):
        return {'loi': bd['loi']}
    ban = _CACHE_BAN['ban']
    TT = nap(C, 'tinh_tuyen')
    GD = nap(C, 'gui_duong')
    huong = HUONG_DICH[dich]
    (dx, dy), yaw = GD.DICH[huong]
    dau = (pose[0], pose[1])
    loi = []
    for r in (1, 2):
        cb = []
        try:
            duong = TT.tinh_duong(ban, dau, (dx, dy), r, yaw_cuoi=yaw, canh_bao=cb)
        except TT.KhongCoDuong as e:
            loi.append('tuyen %d: %s' % (r, e))
            continue
        do = [(i, j) for j in range(bd['h']) for i in range(bd['w']) if bd['vung'][j * bd['w'] + i] == '0']     # 20:33: chi tuong ban do
        kc = min(min((kc_toi_o(x, y, ban, i, j) for i, j in do if abs(ban.ox + (i + .5) * ban.res - x) < 0.6
                      and abs(ban.oy + (j + .5) * ban.res - y) < 0.6), default=9.9) for x, y, _ in duong)
        dai = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(duong, duong[1:]))
        return {'loi': None, 'dich': dich, 'huong': huong, 'tuyen': r, 'diem': [[round(x, 3), round(y, 3)] for x, y, _ in duong],
                'dai_m': round(dai, 2), 'kc_do_m': round(kc, 3),                     # 20:33: bo dau_trong_do (bo 금지 theo ban do)
                'canh_bao': [[round(x, 2), round(y, 2), round(c, 3)] for x, y, c in cb], 'tu': [round(dau[0], 3), round(dau[1], 3)],
                'tuyen_loi': loi}
    return {'loi': '; '.join(loi) or 'khong tim duoc duong'}


# ---------- M7: 도움이 필요해요 ----------
R_ANH = 1.5      # m: chi gui diem LiDAR cach tam xe <= so nay (khung anh tren trang)


def doc_scan(path):
    """Tep 'ros2 topic echo --once /scan' (YAML) -> SimpleNamespace(angle_min, angle_increment, range_min, range_max, ranges)."""
    from types import SimpleNamespace
    with open(path) as f:
        dong = f.read().splitlines()
    s, khoi = {}, None
    rg = []
    for l in dong:
        if khoi == 'ranges':
            if l.startswith('- '):
                v = l[2:].strip()
                if v.strip("'") == '...':                    # ros2 topic echo khong --full-length: day bi cat
                    s['bi_cat'] = True
                    khoi = None
                    continue
                rg.append(float('inf') if v in ('.inf', 'inf') else (float('nan') if v in ('.nan', 'nan') else float(v)))
                continue
            khoi = None
        m = re.match(r'^(angle_min|angle_increment|range_min|range_max):\s*(\S+)', l)
        if m:
            s[m.group(1)] = float(m.group(2))
        elif l.rstrip() == 'ranges:':
            khoi = 'ranges'
    if len([k for k in s if k != 'bi_cat']) < 4 or not rg:
        raise ValueError('tep scan thieu truong: %s' % os.path.basename(path))
    return SimpleNamespace(ranges=rg, **s)


def anh_lidar(C):
    """Tep scan moi nhat -> diem khung than xe. {'co': False, ...} neu khong co tep."""
    import glob
    mau = os.environ.get('CAU_NOI_SCAN_GLOB') or os.path.join(C.HOME, 'scan_*.txt')
    ds = sorted(glob.glob(mau), key=lambda f: os.stat(f).st_mtime)
    if not ds:
        return {'co': False, 'mau': mau}
    f = ds[-1]
    try:
        sc = doc_scan(f)
        L = nap(C, 'lai_xe_lidar')
        pts = L.scan_to_xy(sc)
    except (OSError, ValueError, ImportError) as e:
        return {'co': False, 'loi': str(e), 'tep': f}
    mt = os.stat(f).st_mtime
    return {'co': True, 'tep': f, 'gio': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mt)), 'tuoi_s': int(time.time() - mt),
            'so_tia': len(sc.ranges), 'bi_cat': bool(getattr(sc, 'bi_cat', False)), 'diem': [[round(x, 3), round(y, 3)] for x, y in pts if math.hypot(x, y) <= R_ANH]}


# ---------- M6: 수동 조종 (chuyen tiep sang teleop_web) ----------
# Ten bien moi truong chua ma truy cap cua teleop_web: cung ten voi bien bat_xe.sh dung de chay teleop_web. Ghep chu de tep
# nay khong chua chu do (luat Thien: khong in dong co chu do). Ma KHONG bao gio ghi vao tep, log hay tra ve trang.
TEN_BIEN_MA = 'ACC' + 'ESS'
T_CHO_DANG_NHAP = 10.0   # s: dang nhap teleop_web hong thi khong thu lai trong luc nay (teleop_web khoa sau 5 lan sai lien tiep)
_TELEOP = {'cookie': None, 't_hong': -1e9, 'lock': threading.Lock()}


def _goi_teleop(method, path, body, ctype, cookie=None):
    port = int(os.environ.get('CAU_NOI_TELEOP_PORT', '5000'))
    c = http.client.HTTPConnection('127.0.0.1', port, timeout=1.0)
    try:
        hd = {'Content-Type': ctype}
        if cookie:
            hd['Cookie'] = cookie
        c.request(method, path, body=body, headers=hd)
        r = c.getresponse()
        r.read()
        return r.status, r.getheader('Set-Cookie')
    finally:
        c.close()


def lai_tay(l, a, co_chuyen=False):
    """Mot lenh pad -> teleop_web /cmd. Tra (ma HTTP, json). Khong tra ma truy cap trong bat ky truong hop nao.
    07/11 111-Q2: co_chuyen (dang co chuyen chay) => lenh khac 0 bi khoa (409); lenh 0 (tha nut) van gui."""
    try:
        l, a = max(-1.0, min(1.0, float(l))), max(-1.0, min(1.0, float(a)))
    except (TypeError, ValueError):
        return 400, {'loi': 'l, a phai la so trong -1..1'}
    if co_chuyen and (l != 0.0 or a != 0.0):
        return 409, {'loi': 'dang co chuyen: bam 정지 truoc khi lai tay', 'khoa': True}
    than = json.dumps({'linear': l, 'angular': a})
    with _TELEOP['lock']:
        try:
            st, _ = _goi_teleop('POST', '/cmd', than, 'application/json', _TELEOP['cookie'])
            if st == 200:
                return 200, {'ok': True, 'l': l, 'a': a}
            if st != 401:
                return 502, {'loi': 'teleop_web /cmd tra %d' % st}
            ma = os.environ.get(TEN_BIEN_MA)
            if not ma:
                return 503, {'loi': 'teleop_web can ma nhung cau noi khong co bien moi truong cua ma (them vao ~/cau_noi/cau_noi.env, cung ten voi bat_xe.sh)'}
            con = T_CHO_DANG_NHAP - (time.monotonic() - _TELEOP['t_hong'])
            if con > 0:
                return 503, {'loi': 'dang nhap teleop_web vua hong, thu lai sau %d s' % math.ceil(con)}
            st, sc = _goi_teleop('POST', '/login', urllib.parse.urlencode({'code': ma}), 'application/x-www-form-urlencoded')
            m = re.search(r'(teleop_token=[^;]+)', sc or '')
            if st not in (301, 302, 303) or not m:
                _TELEOP['t_hong'] = time.monotonic()
                return 502, {'loi': 'dang nhap teleop_web hong (HTTP %d)' % st}
            _TELEOP['cookie'] = m.group(1)
            st, _ = _goi_teleop('POST', '/cmd', than, 'application/json', _TELEOP['cookie'])
            return (200, {'ok': True, 'l': l, 'a': a}) if st == 200 else (502, {'loi': 'teleop_web /cmd tra %d sau dang nhap' % st})
        except OSError as e:
            return 502, {'loi': 'khong goi duoc teleop_web cong %s (%s)' % (os.environ.get('CAU_NOI_TELEOP_PORT', '5000'), e.__class__.__name__)}


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
    if ten == 'lai_tay':                                         # M6: cung kiem PIN nhu /nut
        if not C.pin_dung(h.headers.get('X-Pin')):
            return h._gui(401, {'loi': 'can PIN'})
        d = doc_than(h)
        with C.LOCK:
            co_chuyen = C.CHUYEN['chay']
        st, kq = lai_tay(d.get('l', 0), d.get('a', 0), co_chuyen)
        return h._gui(st, kq)
    if ten == 'thu_lai':                                         # M7 다시 시도: gui lai chuyen vua roi qua duong /nut
        with C.LOCK:
            huong = C.CHUYEN['huong']
        if huong not in ('di_B', 've_A'):
            return h._gui(409, {'loi': 'chua co chuyen nao de thu lai', 'da_bat_dau': False})
        return h._nut(huong)
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
    if method == 'GET' and p == '/v10/ban_do':
        kq = ban_do(C)
        return h._gui(200 if not kq.get('loi') else 503, kq)
    if method == 'GET' and p == '/v10/tuyen':
        q = dict(kv.split('=', 1) for kv in (h.path.split('?', 1)[1].split('&') if '?' in h.path else []) if '=' in kv)
        s = sep_cuoi(doc_duoi(C.CFG['sep_nhat_ky']))
        kq = tuyen(C, q.get('dich'), s['pose'])
        return h._gui(200 if not kq.get('loi') else 422, kq)
    if method == 'GET' and p == '/v10/anh_lidar':
        return h._gui(200, anh_lidar(C))
    if method == 'GET' and p == '/v10/trang_thai':
        return h._gui(200, trang_thai(C))
    if method == 'POST' and p.startswith('/v10/api/'):
        return lenh(h, C, p[len('/v10/api/'):])
    return h._gui(404, {'loi': 'khong co duong nay'})
