# Random baseline in AirSim (non-destructive; logs CSV for report)
# Run: python scripts/day4_random_baseline.py --secs 120 --seed 42

import csv, time, argparse, random, os
from datetime import datetime
import airsim
import numpy as np

ACTIONS = [
    ("forward",  (3.0, 0.0, 0.0)),   # vx, vy, vz in body frame (m/s)
    ("left",     (2.0, 1.5, 0.0)),
    ("right",    (2.0,-1.5, 0.0)),
    ("ascend",   (0.0, 0.0,-1.0)),   # vz negative = up in NED
    ("descend",  (0.0, 0.0, 1.0)),
    ("hover",    (0.0, 0.0, 0.0)),
]

def get_pose(client):
    s = client.getMultirotorState()
    pos = s.kinematics_estimated.position
    vel = s.kinematics_estimated.linear_velocity
    return (pos.x_val, pos.y_val, pos.z_val, vel.x_val, vel.y_val, vel.z_val)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=int, default=120)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dt", type=float, default=0.5, help="seconds per action")
    args = ap.parse_args()
    random.seed(args.seed)

    # prepare log
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join("data", "runs")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"day4_random_{run_id}.csv")

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    try:
        client.takeoffAsync(timeout_sec=10).join()
        client.moveToZAsync(-3.0, 1.5).join()  # hover about 3m above ground

        start = time.time()
        steps = 0
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t","action","vx","vy","vz","x","y","z","collided"])
            while time.time() - start < args.secs:
                name, (vx,vy,vz) = random.choice(ACTIONS)
                client.moveByVelocityBodyFrameAsync(vx,vy,vz, args.dt, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
                x,y,z, vxm,vym,vzm = get_pose(client)
                col = int(client.simGetCollisionInfo().has_collided)
                w.writerow([round(time.time()-start,2), name, vx,vy,vz, x,y,z, col])
                steps += 1

        # gentle land
        client.moveByVelocityBodyFrameAsync(0,0,0, 1.0).join()
        client.landAsync(timeout_sec=10).join()

        print(f"Logged {steps} steps to {csv_path}")

    finally:
        client.armDisarm(False)
        client.enableApiControl(False)

if __name__ == "__main__":
    main()
