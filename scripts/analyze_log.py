# scripts/analyze_log.py
import csv, math, sys, pathlib
import matplotlib.pyplot as plt

def read_rows(p):
    with open(p, newline="", encoding="utf-8") as f:
        rdr = csv.reader(f)
        header = next(rdr)
        # auto-detect minimal schema: either [step,x,y,z] or the day2 schema
        if header[:4] == ["step","x","y","z"]:
            idx = {"t":None,"x":1,"y":2,"z":3,"coll":None}
        else:
            # day2 logger schema
            # t_epoch,flight,mode,x,y,z,vx,vy,vz,pitch,roll,yaw,collided,...
            idx = {"t":0,"x":3,"y":4,"z":5,"coll":12}
        rows = []
        for r in rdr:
            t = float(r[idx["t"]]) if idx["t"] is not None else None
            rows.append({
                "t": t,
                "x": float(r[idx["x"]]),
                "y": float(r[idx["y"]]),
                "z": float(r[idx["z"]]),
                "coll": int(r[idx["coll"]]) if idx["coll"] is not None else 0
            })
        return rows

def path_length_xy(rows):
    d=0.0
    for i in range(1,len(rows)):
        dx = rows[i]["x"]-rows[i-1]["x"]
        dy = rows[i]["y"]-rows[i-1]["y"]
        d += math.hypot(dx,dy)
    return d

def avg_speed(rows):
    if rows[0]["t"] is None: return None
    T = rows[-1]["t"]-rows[0]["t"]
    if T<=0: return None
    return path_length_xy(rows)/T

def smoothness(rows):
    # mean absolute heading change (rad) — smaller is smoother
    angs=[]
    prev=None
    for i in range(1,len(rows)):
        dx = rows[i]["x"]-rows[i-1]["x"]
        dy = rows[i]["y"]-rows[i-1]["y"]
        if dx==0 and dy==0: continue
        a = math.atan2(dy,dx)
        if prev is not None:
            da = math.atan2(math.sin(a-prev), math.cos(a-prev))
            angs.append(abs(da))
        prev=a
    return sum(angs)/len(angs) if angs else 0.0

def main():
    p = pathlib.Path(sys.argv[1] if len(sys.argv)>1 else "square_log.csv")
    rows = read_rows(p)
    d = path_length_xy(rows)
    v = avg_speed(rows)
    sm = smoothness(rows)
    coll = sum(r["coll"] for r in rows)

    print(f"File: {p}")
    print(f"Path length (XY): {d:.2f} m")
    if v is not None: print(f"Average speed: {v:.2f} m/s")
    print(f"Smoothness (mean |Δheading|): {sm:.3f} rad")
    print(f"Collisions counted: {coll}")

    xs=[r["x"] for r in rows]; ys=[r["y"] for r in rows]
    plt.figure()
    plt.plot(xs, ys, linewidth=2)
    plt.scatter([xs[0]],[ys[0]], marker="o", label="start")
    plt.scatter([xs[-1]],[ys[-1]], marker="x", label="end")
    plt.axis("equal"); plt.title(p.name); plt.legend()
    out = p.with_suffix(".png")
    plt.savefig(out, dpi=160, bbox_inches="tight")
    print(f"Saved path plot → {out}")

if __name__=="__main__":
    main()
