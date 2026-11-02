#!/usr/bin/env python3
"""In /rosout trong bag (bo tin cua teleop), gom theo node + 60 ky tu dau. argv: thu muc bag, [loc chuoi]."""
import datetime
import glob
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

c = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
loc = sys.argv[2] if len(sys.argv) > 2 else None
tid, typ = c.execute("select id,type from topics where name='/rosout'").fetchone()
T = get_message(typ)
dem = {}
for ts, data in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
    m = deserialize_message(data, T)
    if 'teleop' in m.name or 'ACCESS' in m.msg or 'access' in m.msg:
        continue
    gio = datetime.datetime.fromtimestamp(ts / 1e9).strftime('%H:%M:%S')
    if loc:
        if loc in m.msg:
            print(gio, m.name, m.msg[:160])
        continue
    k = (m.name, m.msg[:60])
    if k not in dem:
        dem[k] = [0, gio]
    dem[k][0] += 1
for (n, s), (k, g) in sorted(dem.items(), key=lambda x: x[1][1]):
    print(g, k, n, s)
