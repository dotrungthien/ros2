"""So sanh bang tuyen CU / v3 (dem 30/10) / TIEP TUYEN bang mo phong do choi (KHONG phai Nav2 that)."""
import math, sys
import mo_phong_he_thong as S
d = math.radians
TT = {
    1: [(1.30, 0.30, d(-120.0)), (1.00, 0.10, d(-146.3)), (0.0, 0.0, d(-174.3))],
    2: [(1.30, 0.75, d(-166.9)), (0.45, 0.76, d(179.3)), (0.15, 0.60, d(-151.9)), (0.0, 0.30, d(-116.6)), (0.0, 0.0, d(-90.0))],
}
mc = int(sys.argv[1]) if len(sys.argv) > 1 else 20
for ten, tb, td in [('tuyen 1 CU', S.TUYEN, 1), ('tuyen 1 v3 (1.45;0.03)', S.TUYEN_MOI, 1), ('tuyen 1 TIEP TUYEN', TT, 1),
                    ('tuyen 2 CU', S.TUYEN, 2), ('tuyen 2 v3', S.TUYEN_MOI, 2), ('tuyen 2 TIEP TUYEN', TT, 2)]:
    S.thong_ke(mc, ten, tb, td)
# dem so lan xoay tai cho theo mo hinh do choi: lay tu nhat ky? chi in goc xoay yeu cau
def xoay(tuyen, yaw0=0.0, start=(1.60, 0.82)):
    cur, y, out = start, yaw0, []
    for (x, yv, yaw) in tuyen:
        h = math.atan2(yv - cur[1], x - cur[0])
        e1 = abs(S.wrap(h - y)); e2 = abs(S.wrap(yaw - h)); 
        out.append((round(math.degrees(e1)), round(math.degrees(e2)))); cur, y = (x, yv), yaw
    return out
for ten, tb in [('CU', S.TUYEN), ('v3', S.TUYEN_MOI), ('TT', TT)]:
    print(ten, 'tuyen1 (lech luc xuat phat, lech khi toi):', xoay(tb[1]), '| tuyen2:', xoay(tb[2]))
