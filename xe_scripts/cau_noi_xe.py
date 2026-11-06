#!/usr/bin/env python3
"""cau_noi_xe.py: cau noi giua dien thoai va Pi (viec 92, Core, Thien nang len 31/10).
Chay san tren Pi ngay khi bat nguon (systemd). Khong can ROS, chi dung thu vien Python co san.

Viec no lam (chi 3 viec):
  1) Phuc vu trang giao dien chon B (chon_b.html) va ban do (map_xuong_v2.pgm/.yaml) o cong 5001.
  2) Nut "차량 연결": goi `bash ~/bat_xe.sh tat` roi `bash ~/bat_xe.sh [gio]`, thay cho viec mo terminal.
  3) Bao trang thai ket noi that: doc danh sach tien trinh tren Pi, khong doan.
Viec KHONG lam (chua viet): gui muc tieu, dung xe, vi tri xe (M4/M5). Cac duong do tra 501.
Buoc 4 (03/11, them, duong cu khong doi): trang GET /nut + /api/nut/{trang_thai,bat,di_B,ve_A,dung,tat}: bat muc B,
  chay ~/chuyen.sh BANG=day, dung chuyen (SIGINT gui_duong.py). Moi luc mot viec (409). Dung khan = cong tac motor.
Viec 111 (06/11, them, /nut va duong cu khong doi): trang GET /v10 + /v10/... (trang_v10.py, trang_v10.html cung thu muc).

Trang thai ket noi (link):
  off         khong co tien trinh nao cua he thong
  connecting  bat_xe.sh dang chay
  on          bat_xe.sh ra ma 0 (da tu kiem vi tri) VA du tien trinh chinh dang song
  fail        bat_xe.sh ra ma khac 0 / qua gio, hoac dang chay ma thieu tien trinh chinh

CHUA CHAY TREN Pi THAT (31/10 sang, viet o ngoai duong). Da thu voi bat_xe.sh gia (xem THU_CAU_NOI trong goi).
"""
import hmac
import json
import os
import re
import signal
import sys
import subprocess
import threading
import time   # moi moc thoi gian dung time.monotonic(): dong ho Pi bi chinh giua chung (bat_xe.sh set-time) khong lam hong han 420 s
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))
CFG = {
    "port": int(os.environ.get("CAU_NOI_PORT", "5001")),
    "bat_xe": os.environ.get("CAU_NOI_BAT_XE", HOME + "/bat_xe.sh"),
    "trang": os.environ.get("CAU_NOI_TRANG", HERE + "/chon_b.html"),
    "ban_do": os.environ.get("CAU_NOI_MAP", HOME + "/map_xuong_v2"),  # + .pgm / .yaml
    "pin_file": os.environ.get("CAU_NOI_PIN_FILE", HOME + "/.cau_noi_pin"),
    "log": os.environ.get("CAU_NOI_LOG", HOME + "/log_cau_noi.txt"),
    "nav_log": os.environ.get("CAU_NOI_NAV_LOG", HOME + "/log_bat_xe_nav.txt"),
    "han_s": 480,       # 8 phut ke tu luc con quay lay so 0 (Quy trinh chuan 71, quy tac 2)
    "job_max_s": 420,   # bat_xe.sh toi da 60 + 120 + kiem; qua 420 s coi la hong
    "lech_gio_s": 30,   # Pi lech gio dien thoai > 30 s thi chinh gio khi ket noi
}
# Ten tien trinh chinh (lay tu NODES trong bat_xe.sh). Khop o argv[0] hoac argv[1], khong khop doi so.
CHINH = os.environ.get("CAU_NOI_CHINH", "").split() or ["gyro_mpu6050", "odom_tu_lenh", "slam_toolbox", "dieu_khien", "ydlidar",
         "teleop_web", "bt_navigator", "planner_server", "controller_server",
         "behavior_server", "velocity_smoother", "lifecycle_manager"]
GIO_RE = re.compile(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$")

LOCK = threading.Lock()
JOB = {"running": False, "t_start": 0.0, "ket_qua": None, "chi_tiet": "", "t0": None, "proc": None, "ma": None, "dong": ""}
_CACHE = [0.0, set()]
_NAVC = [None, False]


def quet_tien_trinh():
    """Tra ve tap ten tien trinh chinh dang song. Doc /proc 1 lan moi 1,5 s (khong goi pgrep)."""
    now = time.monotonic()
    if now - _CACHE[0] < 1.5:
        return _CACHE[1]
    co = set()
    me = os.getpid()
    try:
        ds = os.listdir("/proc")
    except OSError:
        ds = []
    for d in ds:
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open("/proc/%s/cmdline" % d, "rb") as f:
                argv = f.read().split(b"\0")[:2]
        except OSError:
            continue
        s = b" ".join(argv).decode("utf-8", "ignore")
        for n in CHINH:
            if n in s:
                co.add(n)
    _CACHE[0], _CACHE[1] = now, co
    return co


def nav_tu_tat():
    """True neu nhat ky Nav2 cua lan bat NAY co 'CRITICAL FAILURE': lifecycle_manager mat nhip tim cua mot may chu
    va tu tat ca cum Nav2 trong khi cac tien trinh van con song (31/10 16:00:57: velocity_smoother, 97 s sau khi bat)."""
    path = CFG["nav_log"]
    try:
        st = os.stat(path)
    except OSError:
        return False
    key = (st.st_mtime_ns, st.st_size)
    if _NAVC[0] == key:
        return _NAVC[1]
    try:
        with open(path, "rb") as f:
            f.seek(max(0, st.st_size - 200000))
            txt = f.read().decode("utf-8", "ignore")
    except OSError:
        return False
    _NAVC[0], _NAVC[1] = key, ("CRITICAL FAILURE" in txt)
    return _NAVC[1]


def doc_cuoi(path, n=6):
    try:
        with open(path, "r", errors="ignore") as f:
            return [l.rstrip() for l in f.readlines()[-n:]]
    except OSError:
        return []


def chay_job(gio, max_s=None, env=None):
    """Chay trong luong rieng: tat sach roi bat. Ghi ket qua vao JOB. /nut: max_s 600, env them SEP_RA_* (muc B)."""
    log = CFG["log"]
    max_s = max_s or CFG["job_max_s"]
    kq, chi, rc = "fail", "", None
    try:
        with open(log, "w") as lf:
            lf.write("== cau_noi_xe %s: tat truoc ==\n" % datetime.now().strftime("%H:%M:%S"))
            lf.flush()
            subprocess.run(["bash", CFG["bat_xe"], "tat"], stdout=lf, stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL, timeout=120)
            lf.write("== bat ==\n")
            lf.flush()
            args = ["bash", CFG["bat_xe"]] + ([gio] if gio else [])
            p = subprocess.Popen(args, stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                 start_new_session=True, env=env)
            with LOCK:
                JOB["proc"] = p
            t_ket_thuc = time.monotonic() + max_s
            vi_tri_doc = 0
            while p.poll() is None:
                if time.monotonic() > t_ket_thuc:
                    os.killpg(p.pid, signal.SIGKILL)
                    chi = "qua %d giay, da dung" % max_s
                    break
                time.sleep(0.5)
                # bat_xe.sh in dong nay khi con quay lay so 0
                try:
                    with open(log, "r", errors="ignore") as rf:
                        rf.seek(vi_tri_doc)
                        moi = rf.read()
                        vi_tri_doc = rf.tell()
                    if "Con quay da lay so 0" in moi:
                        with LOCK:
                            JOB["t0"] = time.monotonic()
                except OSError:
                    pass
            rc = p.poll()
            if rc == 0:
                kq = "ok"
            elif not chi:
                chi = "bat_xe.sh ra ma %s" % rc
    except Exception as e:  # noqa: BLE001
        chi = "loi: %s" % e
    dong = dong_khop(log, DONG_BAT)
    with LOCK:
        JOB["running"] = False
        JOB["ket_qua"] = kq
        JOB["chi_tiet"] = chi
        JOB["proc"] = None
        JOB["ma"] = rc
        JOB["dong"] = dong


def bat_dau_job(gio, max_s=None, env=None):
    with LOCK:
        if JOB["running"] or CHUYEN["chay"] or TAT["chay"]:   # /nut: moi luc mot viec (dang co chuyen thi khong tat/bat)
            return False
        JOB.update(running=True, t_start=time.monotonic(), ket_qua=None, chi_tiet="", t0=None, ma=None, dong="")
    threading.Thread(target=chay_job, args=(gio, max_s, env), daemon=True).start()
    return True


# ---------- Buoc 4 (03/11): trang /nut. Bat du doi (muc B), Di B / Ve A (chuyen.sh BANG=day), Dung chuyen ----------
# Dung khan van la cong tac motor. NAV_PARAMS: systemd nap cau_noi.env vao moi truong (EnvironmentFile); bat_xe.sh
# tu doc ~/cau_noi/cau_noi.env neu thieu. CHUA CHAY TREN PI.
CHUYEN = {"chay": False, "huong": "", "t_start": 0.0, "ma": None, "ket_qua": "", "dong": "", "pgid": None}
DUNG = {"ket_qua": "", "ms": None, "gio": ""}
TAT = {"chay": False, "t_start": 0.0, "ma": None, "dong": ""}
MA_CHUYEN = {0: "DEN_NOI", 1: "HET_DUONG", 3: "SAI_VI_TRI", 4: "DUNG_SEP", 130: "NGAT"}   # "Ma thoat" dau chuyen.sh
DONG_CHUYEN = re.compile(r"^(DEN_NOI|HET_DUONG|DUNG_SEP|DUNG: SAI_VI_TRI|HUY_DICH_KHI_NGAT|LOI|SAI )")
DONG_BAT = re.compile(r"^(SAN SANG|CHUA SAN SANG|QUA 8 PHUT|DA BAT|LOI)")
DONG_TAT = re.compile(r"^(DA_DON|CON_ROS_SONG|CON_CHAY)")   # bat_xe.sh tat: 0 = DA_DON n tep; 1 = CON_CHAY / CON_ROS_SONG
ENV_MUC_B = {"SEP_RA_LENH": "true", "SEP_RA_THOAT": "true", "PHU_LAI_PHAT_LENH": "true"}
CFG.update(chuyen=HOME + "/chuyen.sh", log_chuyen=HOME + "/log_nut_chuyen.txt", log_tat=HOME + "/log_nut_tat.txt",
           sep_nhat_ky=HOME + "/sep_nhat_ky.csv", bat_max_s=600, dung_cho_s=3.0)
CFG.update(trang_v10=os.environ.get("CAU_NOI_TRANG_V10", HERE + "/trang_v10.html"))   # viec 111


def dong_khop(path, mau):
    """Dong cuoi khop mau; khong co thi dong cuoi khac rong."""
    ds = doc_cuoi(path, 400)
    return next((l for l in reversed(ds) if mau.match(l)), next((l for l in reversed(ds) if l.strip()), ""))


def song(pid):
    """Tien trinh con song va khong phai zombie."""
    try:
        with open("/proc/%d/stat" % pid) as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return False


def pid_gui_duong():
    """PID cac `python3 ... gui_duong.py ...` (mau bai 303: argv[0] la python3, mot doi so dung ten gui_duong.py;
    `bash -c "... gui_duong.py"` khong khop)."""
    ds = []
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == os.getpid():
            continue
        try:
            with open("/proc/%s/cmdline" % d, "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if os.path.basename(argv[0]).startswith(b"python3") and b"gui_duong.py" in [os.path.basename(a) for a in argv[1:]]:
            ds.append(int(d))
    return ds


def bat_dau_tat():
    with LOCK:
        if JOB["running"] or CHUYEN["chay"] or TAT["chay"]:
            return False
        TAT.update(chay=True, t_start=time.monotonic(), ma=None, dong="")
    threading.Thread(target=chay_tat, daemon=True).start()
    return True


def chay_tat():
    """bash ~/bat_xe.sh tat (CAU_NOI_BAT_XE), han 120 s nhu buoc tat cua chay_job."""
    rc = None
    try:
        with open(CFG["log_tat"], "w") as lf:
            lf.write("== cau_noi_xe %s: tat ==\n" % datetime.now().strftime("%H:%M:%S"))
            lf.flush()
            rc = subprocess.run(["bash", CFG["bat_xe"], "tat"], stdout=lf, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, timeout=120).returncode
    except subprocess.TimeoutExpired:
        with open(CFG["log_tat"], "a") as lf:
            lf.write("LOI: tat qua 120 giay\n")
    except Exception as e:  # noqa: BLE001
        with open(CFG["log_tat"], "a") as lf:
            lf.write("LOI: %s\n" % e)
    dong = dong_khop(CFG["log_tat"], DONG_TAT)
    with LOCK:
        TAT.update(chay=False, ma=rc, dong=dong)


def bat_dau_chuyen(huong):
    with LOCK:
        if JOB["running"] or CHUYEN["chay"] or TAT["chay"]:
            return False
        lf = open(CFG["log_chuyen"], "w")
        lf.write("== cau_noi_xe %s: %s (BANG=day) ==\n" % (datetime.now().strftime("%H:%M:%S"), huong))
        lf.flush()
        p = subprocess.Popen(["bash", CFG["chuyen"], huong], stdout=lf, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True, cwd=HOME,
                             env=dict(os.environ, BO_QUA_KIEM="1", BANG="day"))
        CHUYEN.update(chay=True, huong=huong, t_start=time.monotonic(), ma=None, ket_qua="", dong="", pgid=p.pid)
    threading.Thread(target=cho_chuyen, args=(p, lf), daemon=True).start()
    return True


def cho_chuyen(p, lf):
    rc = p.wait()
    lf.close()
    dong = dong_khop(CFG["log_chuyen"], DONG_CHUYEN)
    with LOCK:
        CHUYEN.update(chay=False, ma=rc, ket_qua=MA_CHUYEN.get(rc, "LOI"), dong=dong, pgid=None)


def dung_chuyen():
    """SIGINT cho gui_duong.py va nhom chuyen (moi tien trinh mot lan: INT thu hai roi vao luc gui_duong dang huy dich);
    sau dung_cho_s giay con song thi SIGKILL: dich Nav2 co the CHUA huy -> tat motor."""
    t0 = time.monotonic()
    with LOCK:
        pg = CHUYEN["pgid"]
    pids = pid_gui_duong()

    def gui_tin(sig):
        for x in pids:
            try:
                if pg is None or os.getpgid(x) != pg:
                    os.kill(x, sig)
            except ProcessLookupError:
                pass
        if pg is not None:
            try:
                os.killpg(pg, sig)
            except ProcessLookupError:
                pass

    def con_song():
        with LOCK:
            c = pg is not None and CHUYEN["chay"] and CHUYEN["pgid"] == pg
        return c or any(song(x) for x in pids)

    if pg is None and not pids:
        kq = {"ket_qua": "KHONG_CO_CHUYEN", "ms": None}
    else:
        gui_tin(signal.SIGINT)
        while con_song() and time.monotonic() - t0 < CFG["dung_cho_s"]:
            time.sleep(0.05)
        if con_song():
            gui_tin(signal.SIGKILL)
            kq = {"ket_qua": "KHONG CHAC - TAT MOTOR", "ms": None}
        else:
            kq = {"ket_qua": "DA_DUNG", "ms": int((time.monotonic() - t0) * 1000)}
    kq["pid"] = pids
    with LOCK:
        DUNG.update(ket_qua=kq["ket_qua"], ms=kq["ms"], gio=datetime.now().strftime("%H:%M:%S"))
    return kq


def doc_sep():
    """TU_THE va LENH cuoi trong ~/sep_nhat_ky.csv (chi doc 256 kB cuoi). Cot: gio_mono,gio,loai,dau_hieu,...,ket_qua."""
    out = {"tu_the": "", "lenh": ""}
    try:
        with open(CFG["sep_nhat_ky"], "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 262144))
            ds = f.read().decode("utf-8", "ignore").splitlines()
    except OSError:
        return out
    for l in ds:
        c = l.split(",")
        if len(c) >= 7 and c[2] in ("TU_THE", "LENH"):
            out["tu_the" if c[2] == "TU_THE" else "lenh"] = "%s (%s; %s)" % (c[3], c[1][11:], c[6])
    return out


def trang_thai_nut():
    now = time.monotonic()
    with LOCK:
        j, c, d, t = dict(JOB), dict(CHUYEN), dict(DUNG), dict(TAT)
    viec = "bat" if j["running"] else ("chuyen" if c["chay"] else ("tat" if t["chay"] else "rong"))
    return {
        "viec": viec,
        "tat": {"chay": t["chay"], "giay": round(now - t["t_start"], 1) if t["chay"] else None,
                "ma": t["ma"], "dong": t["dong"]},
        "bat": {"chay": j["running"], "giay": round(now - j["t_start"], 1) if j["running"] else None,
                "ma": j.get("ma"), "dong": dong_khop(CFG["log"], DONG_BAT) if j["running"] else j.get("dong", "")},
        "chuyen": {"chay": c["chay"], "huong": c["huong"], "giay": round(now - c["t_start"], 1) if c["chay"] else None,
                   "ma": c["ma"], "ket_qua": c["ket_qua"],
                   "dong": dong_khop(CFG["log_chuyen"], DONG_CHUYEN) if c["chay"] else c["dong"]},
        "dung": d,
        "sep": doc_sep(),
    }


TRANG_NUT = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>차량 버튼</title>
<style>body{font-family:sans-serif;margin:16px;max-width:520px}button{font-size:20px;width:100%;padding:14px;margin:6px 0}
#dung{background:#c00;color:#fff}.o{border:1px solid #888;padding:8px;margin:8px 0;white-space:pre-wrap;font-family:monospace;font-size:14px}
.canh{background:#fe0;padding:8px;font-weight:bold}</style></head><body>
<div class="canh">비상 정지 = 모터 스위치. '주행 정지'는 Nav2 목표만 취소합니다.</div>
<button id="bat">전체 시작 (B단계)</button>
<button id="di">B로 출발</button><button id="ve">A로 복귀</button>
<button id="dung">주행 정지</button>
<button id="tat">차량 종료</button>
<div class="o" id="tt">상태 확인 중...</div>
<div class="o" id="tl"></div>
<script>
var pin=sessionStorage.getItem('pin')||'';
var HUONG={di_B:'A→B',ve_A:'B→A'};
// chi luc in: sau ma goc them (tieng Han), giu ma goc; ma dai truoc ma ngan; \\b: dau _ la chu nen DUNG khong khop trong DUNG_SEP
var HAN=[['HUY_DICH_KHI_NGAT','목표 취소'],['DUNG_YEN_KHONG_RO','원인 불명 정지'],['KHONG_CO_DICH','목표 없음'],
 ['NHAY_VI_TRI','위치 급변 재확인'],['VAT_LA_TRUOC','앞 장애물'],['KET_SAT_VAT','장애물에 막힘'],['TU_BAT_LAI','자동 재시작'],
 ['HET_DUONG','경로 없음'],['KHONG_DAT','실패-재시도'],['DI_THUONG','정상 주행'],['MAT_SCAN','라이다 대기'],
 ['KHONG BAT','이미 실행 중'],['SAN SANG','준비 완료'],['DEN_NOI','도착'],['DA_DUNG','정지 완료'],['VAT_LA','장애물 감지'],
 ['NGAT','중단됨'],['LOI','오류'],['DUNG','정지'],['CHO','대기'],['DI','진행'],['cach dich','목표까지'],['giay','초']];
var HAN_TU={};HAN.forEach(function(x){HAN_TU[x[0]]=x[1];});
var HAN_RE=new RegExp('\\\\b(CHUA )?('+HAN.map(function(x){return x[0];}).join('|')+')\\\\b','g');
function han(s){
  if(!s)return s;
  return String(s).replace(HAN_RE,function(m,chua,ma){
    if(chua&&ma==='SAN SANG')return m;   // CHUA SAN SANG khong phai 준비 완료
    return (chua||'')+ma+' ('+HAN_TU[ma]+')';
  }).replace(/\\bdi_B\\b/g,'A→B').replace(/\\bve_A\\b/g,'B→A');
}
function gui(lenh,hoi){
  if(hoi&&!confirm(hoi))return;
  fetch('/api/nut/'+lenh,{method:'POST',headers:{'X-Pin':pin}}).then(function(r){
    if(r.status==401){pin=prompt('PIN 입력')||'';sessionStorage.setItem('pin',pin);return gui(lenh);}
    return r.json().then(function(j){document.getElementById('tl').textContent=(HUONG[lenh]||lenh)+': HTTP '+r.status+'\\n'+JSON.stringify(j,null,1);});
  }).catch(function(e){document.getElementById('tl').textContent=(HUONG[lenh]||lenh)+': 네트워크 오류 '+e;});
}
document.getElementById('bat').onclick=function(){gui('bat','전체를 껐다가 다시 시작할까요 (B단계)? 차량은 A선, 모터 OFF.');};
document.getElementById('di').onclick=function(){gui('di_B','B로 출발? 모터 ON, 손은 스위치 옆에.');};
document.getElementById('ve').onclick=function(){gui('ve_A','A로 복귀? 모터 ON, 손은 스위치 옆에.');};
document.getElementById('dung').onclick=function(){gui('dung');};
document.getElementById('tat').onclick=function(){gui('tat','차량의 ROS 전체를 종료할까요?');};
function hoi(){
  fetch('/api/nut/trang_thai').then(function(r){return r.json();}).then(function(s){
    var b=s.bat,c=s.chuyen,k=s.tat,t='작업: '+({rong:'대기',bat:'시작 중',chuyen:'주행 중',tat:'종료 중'}[s.viec])+'\\n';
    t+='종료: '+(k.chay?('진행 중 '+k.giay+' s'):('코드 '+(k.ma===null?'-':k.ma)))+'\\n  '+(han(k.dong)||'')+'\\n';
    t+='시작: '+(b.chay?('진행 중 '+b.giay+' s'):('코드 '+(b.ma===null?'-':b.ma)))+'\\n  '+(han(b.dong)||'')+'\\n';
    t+='주행 '+(HUONG[c.huong]||c.huong||'')+': '+(c.chay?('진행 중 '+c.giay+' s'):((han(c.ket_qua)||'-')+' 코드 '+(c.ma===null?'-':c.ma)))+'\\n  '+(han(c.dong)||'')+'\\n';
    t+='정지: '+(han(s.dung.ket_qua)||'-')+(s.dung.ms!==null?(' '+s.dung.ms+' ms'):'')+(s.dung.gio?(' 시각 '+s.dung.gio):'')+'\\n';
    t+='두뇌: TU_THE '+(han(s.sep.tu_the)||'-')+'\\n     LENH '+(han(s.sep.lenh)||'-');
    document.getElementById('tt').textContent=t;
  }).catch(function(){document.getElementById('tt').textContent='Pi 연결 끊김';});
}
hoi();setInterval(hoi,1000);
</script></body></html>"""


def trang_thai():
    now = time.monotonic()
    co = quet_tien_trinh()
    thieu = [n for n in CHINH if n not in co]
    with LOCK:
        j = dict(JOB)
    chi = j["chi_tiet"]
    if j["running"]:
        link = "connecting"
    elif j["ket_qua"] == "fail":
        link = "fail"
    elif not thieu:
        link = "on"
    elif len(thieu) == len(CHINH):
        link = "off"
    else:
        link = "fail"
        chi = "thieu tien trinh: " + ", ".join(thieu)
    if link == "fail" and not chi:
        chi = "thieu: " + ", ".join(thieu)
    if link == "on" and nav_tu_tat():
        link = "fail"
        chi = "Nav2 tu tat (lifecycle_manager mat nhip tim); cac tien trinh van song. Xem ~/log_bat_xe_nav.txt"
    han = None
    if link == "on" and j["t0"]:
        han = int(CFG["han_s"] - (now - j["t0"]))
    return {
        "link": link,
        "connect_s": round(now - j["t_start"], 1) if j["running"] else 0,
        "nav": "idle", "drv": "none", "wait_s": 0,       # M4/M5 chua viet
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},        # chua doc tf; chi dung ngay luc vua ket noi (xe o A)
        "manual": False, "photo": None, "notified": False,
        "han_s": han, "thieu": thieu, "chi_tiet": chi,
        "log_cuoi": doc_cuoi(CFG["log"]) if link in ("connecting", "fail") else [],
    }


def gio_hop_le(s):
    """Tra ve chuoi gio neu Pi lech dien thoai > lech_gio_s, nguoc lai None (khong chinh gio)."""
    if not s or not GIO_RE.match(s):
        return None
    try:
        t = datetime.strptime(s, "%Y-%m-%d %H:%M:%S")
    except ValueError:
        return None
    return s if abs((datetime.now() - t).total_seconds()) > CFG["lech_gio_s"] else None


def pin_dung(header):
    try:
        with open(CFG["pin_file"], "r") as f:
            pin = f.read().strip()
    except OSError:
        return True            # khong co tep PIN thi khong hoi
    return bool(pin) and hmac.compare_digest(pin, (header or "").strip())


class H(BaseHTTPRequestHandler):
    server_version = "cau_noi_xe/1"

    def log_message(self, fmt, *a):
        pass

    def _gui(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False)
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _tep(self, path, ctype):
        try:
            with open(path, "rb") as f:
                self._gui(200, f.read(), ctype)
        except OSError:
            self._gui(404, {"loi": "khong thay tep", "tep": os.path.basename(path)})

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/index.html", "/chon_b.html"):
            self._tep(CFG["trang"], "text/html; charset=utf-8")
        elif p == "/api/b/status":
            self._gui(200, trang_thai())
        elif p == "/map.pgm":
            self._tep(CFG["ban_do"] + ".pgm", "application/octet-stream")
        elif p == "/map.png":
            self._tep(CFG["ban_do"] + ".png", "image/png")
        elif p == "/map.yaml":
            self._tep(CFG["ban_do"] + ".yaml", "text/plain; charset=utf-8")
        elif p == "/nut":
            self._gui(200, TRANG_NUT, "text/html; charset=utf-8")
        elif p == "/api/nut/trang_thai":
            self._gui(200, trang_thai_nut())
        elif p == "/v10" or p.startswith("/v10/"):
            self._v10("GET", p)
        else:
            self._gui(404, {"loi": "khong co duong nay"})

    def do_POST(self):
        p = self.path.split("?")[0]
        if p.startswith("/v10/"):
            return self._v10("POST", p)
        if p.startswith("/api/nut/"):
            return self._nut(p[len("/api/nut/"):])
        if not p.startswith("/api/b/"):
            return self._gui(404, {"loi": "khong co duong nay"})
        if not pin_dung(self.headers.get("X-Pin")):
            return self._gui(401, {"loi": "can PIN"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, OSError):
            body = {}
        cmd = p[len("/api/b/"):]
        if cmd == "connect":
            ok = bat_dau_job(gio_hop_le(str(body.get("time", ""))))
            return self._gui(202 if ok else 409, {"da_bat_dau": ok})
        # goal / stop / retry / manual / auto: chua viet (M4, M5)
        return self._gui(501, {"loi": "chua lam", "lenh": cmd})

    def _v10(self, method, p):
        """Viec 111: /v10 o trang_v10.py. Loi o day chi tra 500 cho /v10; /nut va duong cu khong doi."""
        try:
            import trang_v10
            return trang_v10.xu_ly(self, sys.modules[__name__], method, p)
        except Exception as e:  # noqa: BLE001
            return self._gui(500, {"loi": "trang_v10: %s" % e})

    def _nut(self, cmd):
        if not pin_dung(self.headers.get("X-Pin")):
            return self._gui(401, {"loi": "can PIN"})
        if cmd == "dung":
            return self._gui(200, dung_chuyen())
        if cmd == "bat":
            ok = bat_dau_job(None, CFG["bat_max_s"], dict(os.environ, **ENV_MUC_B))
        elif cmd in ("di_B", "ve_A"):
            ok = bat_dau_chuyen(cmd)
        elif cmd == "tat":
            ok = bat_dau_tat()
        else:
            return self._gui(404, {"loi": "khong co duong nay"})
        return self._gui(202 if ok else 409, {"da_bat_dau": ok, "viec": trang_thai_nut()["viec"]})


def kiem():
    """python3 cau_noi_xe.py --kiem : in tien trinh chinh nao thay / khong thay (chay khi he thong dang bat)."""
    _CACHE[0] = 0.0
    co = quet_tien_trinh()
    for n in CHINH:
        print(("THAY    " if n in co else "KHONG   ") + n)
    thieu = [n for n in CHINH if n not in co]
    for n in thieu:
        print("-- tien trinh co chu '%s' o cho khac trong dong lenh (neu co):" % n)
        for d in os.listdir("/proc"):
            if d.isdigit() and int(d) != os.getpid():
                try:
                    with open("/proc/%s/cmdline" % d, "rb") as f:
                        c = f.read().replace(b"\0", b" ").decode("utf-8", "ignore")
                except OSError:
                    continue
                if n in c:
                    print("   pid %s: %s" % (d, c[:160]))


def main():
    if "--kiem" in sys.argv:
        return kiem()
    srv = ThreadingHTTPServer(("0.0.0.0", CFG["port"]), H)
    print("cau_noi_xe: cong %d, bat_xe=%s, trang=%s" % (CFG["port"], CFG["bat_xe"], CFG["trang"]), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
