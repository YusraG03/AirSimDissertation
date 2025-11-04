import csv, heapq, math, time, os
from pathlib import Path
from datetime import datetime
import airsim

ALT_M=3.0
SPEED=3.0
CELL=1.0           # 1 m per grid cell
ORIGIN=(0.0,0.0)   # (row=0,col=0) -> AirSim (x=0,y=0)

# ---------- grid / A* ----------
def load_grid(p):
    g=[]
    with open(p, newline="") as f:
        for r in csv.reader(f):
            g.append([int(x) for x in r])
    return g

def nbrs(r,c,H,W):
    for dr,dc in [(1,0),(-1,0),(0,1),(0,-1)]:
        rr,cc=r+dr,c+dc
        if 0<=rr<H and 0<=cc<W:
            yield rr,cc

def astar(grid, start, goal):
    H,W=len(grid),len(grid[0])
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    openQ=[(0,start)]
    came={start:None}
    gscore={start:0}
    fscore={start:h(start,goal)}
    while openQ:
        _,cur=heapq.heappop(openQ)
        if cur==goal:
            path=[cur]
            while came[cur] is not None:
                cur=came[cur]; path.append(cur)
            return list(reversed(path))
        for nb in nbrs(cur[0],cur[1],H,W):
            if grid[nb[0]][nb[1]]==1: continue
            tent=gscore[cur]+1
            if tent<gscore.get(nb,1e9):
                came[nb]=cur
                gscore[nb]=tent
                fscore[nb]=tent+h(nb,goal)
                heapq.heappush(openQ,(fscore[nb],nb))
    return None

def grid_to_world(cell):
    r,c=cell
    x = ORIGIN[0] + c*CELL
    y = ORIGIN[1] + r*CELL
    return x,y

# ---------- simple CSV logger (compatible with analyzer) ----------
def open_log(run_csv):
    f=open(run_csv, "w", newline="", encoding="utf-8")
    w=csv.writer(f)
    w.writerow(["t_epoch","flight","mode","x","y","z","vx","vy","vz",
                "pitch","roll","yaw","collided",
                "cx","cy","cz","nx","ny","nz","battery_est","waypoint_idx"])
    return f,w

def eul_from_quat(q):
    w,x,y,z=q.w_val,q.x_val,q.y_val,q.z_val
    t0=2*(w*x+y*z); t1=1-2*(x*x+y*y); roll=math.atan2(t0,t1)
    t2=2*(w*y-z*x); t2=max(-1,min(1,t2)); pitch=math.asin(t2)
    t3=2*(w*z+x*y); t4=1-2*(y*y+z*z);   yaw=math.atan2(t3,t4)
    return pitch,roll,yaw

def log_row(w, flight, mode, state, coll, wp_idx):
    p=state.kinematics_estimated.position
    v=state.kinematics_estimated.linear_velocity
    q=state.kinematics_estimated.orientation
    pitch,roll,yaw=eul_from_quat(q)
    w.writerow([time.time(), flight, mode,
                p.x_val,p.y_val,p.z_val,
                v.x_val,v.y_val,v.z_val,
                pitch,roll,yaw,
                int(coll.has_collided),
                coll.impact_point.x_val,coll.impact_point.y_val,coll.impact_point.z_val,
                coll.normal.x_val,coll.normal.y_val,coll.normal.z_val,
                -1, wp_idx])

def main(map_path="data/map1.csv", start=(0,0), goal=(8,11), tag="map1"):
    runs_dir=Path("runs"); runs_dir.mkdir(exist_ok=True)
    run_id=f"astar_{tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_csv=runs_dir/f"{run_id}.csv"

    grid=load_grid(map_path)
    path_cells=astar(grid,start,goal)
    if not path_cells:
        print("[!] No path"); return
    waypoints=[grid_to_world(c) for c in path_cells]

    csv_f,csv_w=open_log(run_csv)

    c=airsim.MultirotorClient(); c.confirmConnection()
    c.enableApiControl(True); c.armDisarm(True)
    c.takeoffAsync().join(); c.moveToZAsync(-ALT_M,SPEED).join()

    # short hover log
    t0=time.time()
    while time.time()-t0<1.0:
        s=c.getMultirotorState(); col=c.simGetCollisionInfo()
        log_row(csv_w, run_id, "hover", s, col, -1); time.sleep(0.1)

    # follow waypoints
    for i,(x,y) in enumerate(waypoints):
        c.moveToPositionAsync(x,y,-ALT_M,SPEED).join()
        s=c.getMultirotorState(); col=c.simGetCollisionInfo()
        log_row(csv_w, run_id, "wp", s, col, i)

    c.landAsync().join()
    s=c.getMultirotorState(); col=c.simGetCollisionInfo()
    log_row(csv_w, run_id, "landed", s, col, len(waypoints)-1)

    c.armDisarm(False); c.enableApiControl(False)
    csv_f.close()
    print(f"[✓] Run saved → {run_csv}")

if __name__=="__main__":
    # change start/goal if your map differs
     main(map_path="data/map1.csv", start=(0,0), goal=(8,11), tag="map1")
