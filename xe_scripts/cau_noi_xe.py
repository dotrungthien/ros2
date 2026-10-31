#!/usr/bin/env python3
"""cau_noi_xe.py: cau noi giua dien thoai va Pi (viec 92, Core, Thien nang len 31/10).
Chay san tren Pi ngay khi bat nguon (systemd). Khong can ROS, chi dung thu vien Python co san.

Viec no lam (chi 3 viec):
  1) Phuc vu trang giao dien chon B (chon_b.html) va ban do (map_xuong_v2.pgm/.yaml) o cong 5001.
  2) Nut "차량 연결": goi `bash ~/bat_xe.sh tat` roi `bash ~/bat_xe.sh [gio]`, thay cho viec mo terminal.
  3) Bao trang thai ket noi that: doc danh sach tien trinh tren Pi, khong doan.
Viec KHONG lam (chua viet): gui muc tieu, dung xe, vi tri xe (M4/M5). Cac duong do tra 501.

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
JOB = {"running": False, "t_start": 0.0, "ket_qua": None, "chi_tiet": "", "t0": None, "proc": None}
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


def chay_job(gio):
    """Chay trong luong rieng: tat sach roi bat. Ghi ket qua vao JOB."""
    log = CFG["log"]
    kq, chi = "fail", ""
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
                                 start_new_session=True)
            with LOCK:
                JOB["proc"] = p
            t_ket_thuc = time.monotonic() + CFG["job_max_s"]
            vi_tri_doc = 0
            while p.poll() is None:
                if time.monotonic() > t_ket_thuc:
                    os.killpg(p.pid, signal.SIGKILL)
                    chi = "qua %d giay, da dung" % CFG["job_max_s"]
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
    with LOCK:
        JOB["running"] = False
        JOB["ket_qua"] = kq
        JOB["chi_tiet"] = chi
        JOB["proc"] = None


def bat_dau_job(gio):
    with LOCK:
        if JOB["running"]:
            return False
        JOB.update(running=True, t_start=time.monotonic(), ket_qua=None, chi_tiet="", t0=None)
    threading.Thread(target=chay_job, args=(gio,), daemon=True).start()
    return True


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
        else:
            self._gui(404, {"loi": "khong co duong nay"})

    def do_POST(self):
        p = self.path.split("?")[0]
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
