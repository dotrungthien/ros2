#!/usr/bin/env python3
"""dung_xe (viec 5, soan 03/11/2026, CHUA CHAY TREN PI): dung xe bang tay, chay mot lan, KHONG goi ros2 CLI.

  1. Ngay khi len: phat Twist 0 len /cmd_vel_teleop 20 Hz (twist_mux uu tien 100 > avoid 60 > nav 20, timeout 0,3 s).
  2. Gui SIGINT cho chuyen.sh va gui_duong.py; sau 1 s con song thi SIGKILL. Chi khop tien trinh co argv[0] la bash
     hoac python3 VA mot doi so co ten tep dung rieng la chuyen.sh / gui_duong.py (bai hoc 303: khong khop "bash -c ...").
  3. Huy MOI dich (CancelGoal goal_id 0, stamp 0) cua /follow_path va /navigate_to_pose, han 2 s moi cai.
  4. Giu Twist 0 them 1,5 s sau khi huy, roi thoat.
In mot dong: DUNG_TAY huy_follow=<..> huy_nav=<..> giet=<pid:ten,...|0> t_ms=<ms tu luc chay toi luc in>
  huy_*: tra_loi:<return_code>:<so dich dang huy> | khong_tra_loi | khong_may_chu
Ma thoat: 0 neu ca hai huy co tra loi, 1 neu khong.
Chay:  python3 ~/dung_xe.py
"""
import os
import signal
import sys
import time

T_BAT_DAU = time.monotonic()
TEP_GIET = ('chuyen.sh', 'gui_duong.py')
DONG_TEP = {'bash', 'python3'}
HAN_HUY = 2.0
GIU_SAU = 1.5
CHO_KILL = 1.0


def tim_tien_trinh():
    """[(pid, ten tep)] cua cac tien trinh can giet (doc /proc, khong goi pgrep)."""
    ra, me = [], os.getpid()
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open('/proc/%s/cmdline' % d, 'rb') as f:
                argv = [a.decode('utf-8', 'ignore') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if not argv or os.path.basename(argv[0]) not in DONG_TEP:
            continue
        ten = next((os.path.basename(a) for a in argv[1:] if os.path.basename(a) in TEP_GIET), None)
        if ten:
            ra.append((int(d), ten))
    return ra


def gui_tin_hieu(ds, sig):
    for pid, _ in ds:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def con_song(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from action_msgs.srv import CancelGoal

    rclpy.init()
    node = Node('dung_xe')
    pub = node.create_publisher(Twist, '/cmd_vel_teleop', 10)
    pub.publish(Twist())
    node.create_timer(0.05, lambda: pub.publish(Twist()))

    def ngu(giay):
        het = time.monotonic() + giay
        while time.monotonic() < het:
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, het - time.monotonic())))

    ds = tim_tien_trinh()
    gui_tin_hieu(ds, signal.SIGINT)
    t_int = time.monotonic()
    da_kill = [False]

    def kill_neu_can():
        if not da_kill[0] and time.monotonic() - t_int >= CHO_KILL:
            gui_tin_hieu([(p, t) for p, t in ds if con_song(p)], signal.SIGKILL)
            da_kill[0] = True

    def huy(ten):
        cli = node.create_client(CancelGoal, ten + '/_action/cancel_goal')
        het = time.monotonic() + HAN_HUY
        while not cli.service_is_ready() and time.monotonic() < het:
            ngu(0.02)
            kill_neu_can()
        if not cli.service_is_ready():
            return False, 'khong_may_chu'
        fut = cli.call_async(CancelGoal.Request())          # goal_id 0 + stamp 0 = huy moi dich
        while not fut.done() and time.monotonic() < het:
            ngu(0.02)
            kill_neu_can()
        if not fut.done() or fut.result() is None:
            return False, 'khong_tra_loi'
        r = fut.result()
        return True, 'tra_loi:%d:%d' % (r.return_code, len(r.goals_canceling))

    ok_f, huy_follow = huy('/follow_path')
    ok_n, huy_nav = huy('/navigate_to_pose')
    while not da_kill[0]:
        ngu(0.02)
        kill_neu_can()
    ngu(GIU_SAU)
    giet = ','.join('%d:%s' % (p, t) for p, t in ds) or '0'
    print('DUNG_TAY huy_follow=%s huy_nav=%s giet=%s t_ms=%d'
          % (huy_follow, huy_nav, giet, int((time.monotonic() - T_BAT_DAU) * 1000)), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok_f and ok_n else 1


if __name__ == '__main__':
    sys.exit(main())
