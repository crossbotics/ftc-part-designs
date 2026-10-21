import math, numpy as np, approach_check as A, entry_window as E
uc=(E.B[0]+E.P[0])/2; sp=np.arange(3.5,12,0.02)
for ball in ("Pollen","Nectar"):
    m,r=E.BALLS[ball]; els=np.arange(A.LAUNCH[ball][0],A.LAUNCH[ball][1]+0.1,1.0)
    for d in (0.33,0.4,0.45,0.5):
        why=A.shots((0.0,uc+d),els,sp,r,m,"down")
        rows=[(round(float(els[i])),A.window(why[i],sp),A.window(np.where(why[i]>=3,0,why[i]),sp)) for i in range(len(els))]
        ok=[(e,w) for e,w,_ in rows if w]
        best=max(ok,key=lambda t:t[1][1]) if ok else None
        fb=max([(e,w) for e,_,w in rows if w],key=lambda t:t[1][1],default=None)
        print(ball,f"{d:.2f} m from opening centre:", f"best {best[0]}° ±{best[1][1]:.1f}% at {best[1][0]:.2f} m/s" if best else "no angle scores",
              f"| scoring angles {ok[0][0]}-{ok[-1][0]}°" if ok else "", f"| without structure: {fb[0]}° ±{fb[1][1]:.1f}%" if fb else "")
