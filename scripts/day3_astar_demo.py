import csv, heapq, math, time
from pathlib import Path
import airsim

ALT_M=3.0
SPEED=3.0
CELL=1.0     # 1 m per grid cell
ORIGIN=(0.0,0.0)  # map (0,0) -> AirSim (x=0,y=0)

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
            path.reverse(); return path
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
    # map row,col to AirSim (x,y) with ORIGIN at top-left row=0,col=0
    r,c=cell
    x = ORIGIN[0] + c*CELL
    y = ORIGIN[1] + r*CELL
    return x,y

def main():
    grid = load_grid(Path("data/map1.csv"))
    start=(0,0)                # row,col
    goal =(8,11)               # adjust to a free cell in your map

    path = astar(grid,start,goal)
    if not path: 
        print("No path found"); return
    waypoints=[grid_to_world(p) for p in path]

    client=airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True); client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-ALT_M, SPEED).join()

    for i,(x,y) in enumerate(waypoints):
        client.moveToPositionAsync(x,y,-ALT_M,SPEED).join()
        # tiny settle
        time.sleep(0.1)

    client.landAsync().join()
    client.armDisarm(False); client.enableApiControl(False)
    print(f"Done. Waypoints: {len(waypoints)}")

if __name__=="__main__":
    main()
