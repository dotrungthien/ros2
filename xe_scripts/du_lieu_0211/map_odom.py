#!/usr/bin/env python3
"""map->odom trong bag: moi 10 s in so tin, stamp nho nhat/lon nhat (gio), gia tri x y yaw; dem tin co stamp lui. argv: bag."""
import datetime
import glob
import math
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

c = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
tid = c.execute("select id from topics where name='/tf'").fetchone()[0]
g = lambda x: datetime.datetime.fromtimestamp(x).strftime('%H:%M:%S.%f')[:-4]
o = {}
lui, prev = 0, 0
for ts, data in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
    for t in deserialize_message(data, TFMessage).transforms:
        if (t.header.frame_id, t.child_frame_id) != ('map', 'odom'):
            continue
        st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if st < prev - 1e-6:
            lui += 1
        prev = max(prev, st)
        q = t.transform.rotation
        v = (t.transform.translation.x, t.transform.translation.y, math.degrees(2 * math.atan2(q.z, q.w)))
        k = int(ts / 1e10)
        o.setdefault(k, []).append((ts / 1e9, st, v))
for k, L in sorted(o.items()):
    sts = [x[1] for x in L]
    print(g(L[0][0]), 'n=%d' % len(L), 'stamp %s..%s' % (g(min(sts)), g(max(sts))), 'ghi-stamp cuoi=%.2f' % (L[-1][0] - L[-1][1]),
          'x=%.3f y=%.3f yaw=%.1f' % L[-1][2])
print('tin stamp lui:', lui)
