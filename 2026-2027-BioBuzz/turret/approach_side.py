import math, numpy as np, approach_check as A, entry_window as E
# sanity: points at the neighbour cells' centres must register as hits; a point in open air must not
for st in ("down","up"):
    pr = A.neighbour(st)[0][0]; ax,up,xc,sec,(a0,a1)=pr
    c = A.PIV + (a0+a1)/2*ax + sec.centroid.y*up + np.array([xc,0,0])
    air = c + np.array([0,0,0.8])
    print(st, "neighbour cell centre", np.round(c,3), "hit:", A.in_prism(c[None],pr,0.04)[0], "| 0.8 m above it hit:", A.in_prism(air[None],pr,0.04)[0])
uc=(E.B[0]+E.P[0])/2; sp=np.arange(4.5,12,0.02)
labels={3:"other HIVE",4:"frame"}
for ball in ("Pollen","Nectar"):
    m,r=E.BALLS[ball]; els=np.arange(A.LAUNCH[ball][0],A.LAUNCH[ball][1]+0.1,2.0)
    for st in ("down","up"):
        for d in (1.0,1.5,2.5):
            row=[]
            for psi in (70,75,80,85):
                ps=math.radians(psi); why=A.shots((d*math.sin(ps),uc+d*math.cos(ps)),els,sp,r,m,st)
                best=max(((A.window(why[i],sp) or (0,0))[1],els[i]) for i in range(len(els)))
                free=max((A.window(np.where(why[i]>=3,0,why[i]),sp) or (0,0))[1] for i in range(len(els)))
                hit=sorted({labels[c] for c in (3,4) if (why==c).any()})
                row.append(f"{psi}°: {best[1]:.0f}° ±{best[0]:.1f}% (free ±{free:.1f}%{' '+'/'.join(hit) if hit else ''})")
            print(ball,st,f"{d}m","  ".join(row))
