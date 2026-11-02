#!/usr/bin/env python3
"""Doc bag (sqlite) : tre scan so voi tf odom->base, tre header so voi gio ghi, luc map->odom ngung tang. argv: thu muc bag."""
import bisect
import glob
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

d = sys.argv[1]
c = sqlite3.connect(glob.glob(d + '/*.db3')[0])
tp = {i: (n, get_message(t)) for i, n, t in c.execute('select id,name,type from topics')}


def s(st):
    return st.sec + st.nanosec * 1e-9


tf_ob, tf_mo, scan, odom = [], [], [], []   # (gio_ghi, stamp)
for tid, ts, data in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
    n, T = tp[tid]
    r = ts * 1e-9
    if n == '/tf':
        for t in deserialize_message(data, T).transforms:
            k = (t.header.frame_id, t.child_frame_id)
            if k == ('odom', 'base_footprint'):
                tf_ob.append((r, s(t.header.stamp)))
            elif k == ('map', 'odom'):
                tf_mo.append((r, s(t.header.stamp)))
    elif n == '/scan':
        scan.append((r, s(deserialize_message(data, T).header.stamp)))
    elif n == '/odom':
        m = deserialize_message(data, T)
        odom.append((r, s(m.header.stamp), m.twist.twist.angular.z))

t0 = scan[0][0] if scan else 0


def pct(v, ps=(0, 10, 50, 90, 99, 100)):
    v = sorted(v)
    return ' '.join('p%d=%.3f' % (p, v[min(len(v) - 1, int(p / 100 * len(v)))]) for p in ps) if v else '-'


print('== %s  scan=%d tf_odom=%d tf_map=%d odom=%d' % (d.rstrip('/').split('/')[-1], len(scan), len(tf_ob), len(tf_mo), len(odom)))
print('tre ghi-header  odom->base tf :', pct([r - st for r, st in tf_ob]))
print('tre ghi-header  /scan         :', pct([r - st for r, st in scan]))
print('tre ghi-header  /odom         :', pct([r - st for r, st, _ in odom]))
print('tre ghi-header  map->odom tf  :', pct([r - st for r, st in tf_mo]))
# (1) scan stamp - stamp tf odom->base moi nhat da toi luc ghi scan (>0: tf chua kip toi gio scan)
rec = [r for r, _ in tf_ob]
oz_r = [r for r, _, _ in odom]
quay, thang = [], []
for r, st in scan:
    i = bisect.bisect_right(rec, r) - 1
    if i < 0:
        continue
    lag = st - max(x[1] for x in tf_ob[max(0, i - 5):i + 1])
    j = bisect.bisect_right(oz_r, r) - 1
    wz = abs(odom[j][2]) if j >= 0 else 0
    (quay if wz > 0.2 else thang).append(lag)
print('scan - tf_odom moi nhat (thang/dung, n=%d):' % len(thang), pct(thang))
print('scan - tf_odom moi nhat (quay |wz|>0.2, n=%d):' % len(quay), pct(quay))
print('ty le scan co tf_odom chua toi (>0): %.0f%%' % (100.0 * sum(1 for x in thang + quay if x > 0) / max(1, len(thang + quay))))
# (2) map->odom: stamp ngung tang
if tf_mo:
    last, t_last = tf_mo[0][1], tf_mo[0][0]
    dung = []
    for r, st in tf_mo:
        if st > last + 1e-6:
            if r - t_last > 2.0:
                dung.append((t_last - t0, r - t0))
            last, t_last = st, r
    print('map->odom stamp dung tang > 2 s (t ghi tu scan dau):', ['%.0f..%.0f' % x for x in dung],
          '| stamp cuoi tang luc t=%.0f, bag het t=%.0f' % (t_last - t0, tf_mo[-1][0] - t0))
    # stamp map->odom = gio scan + transform_timeout? so sanh voi scan
    print('map->odom stamp - gio ghi :', pct([st - r for r, st in tf_mo[:200]], (10, 50, 90)), '(200 tin dau)')
# tre theo thoi gian: moi 20 s, trung vi (ghi-header) cua tf odom
b = {}
for r, st in tf_ob:
    b.setdefault(int((r - t0) // 20), []).append(r - st)
print('tre tf_odom theo 20 s (trung vi):', ' '.join('%d:%.2f' % (k * 20, sorted(v)[len(v) // 2]) for k, v in sorted(b.items())))
b = {}
for r, st in scan:
    b.setdefault(int((r - t0) // 20), []).append(r - st)
print('tre scan    theo 20 s (trung vi):', ' '.join('%d:%.2f' % (k * 20, sorted(v)[len(v) // 2]) for k, v in sorted(b.items())))
