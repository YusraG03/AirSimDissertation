import airsim
import time
import csv

# --- Connect to AirSim ---
client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected to AirSim successfully!")
client.enableApiControl(True)
client.armDisarm(True)

# --- Takeoff ---
print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()  # hover at 3m height

# --- Define waypoints for square path ---
waypoints = [
    (5, 0, -3),   # move forward 5m
    (5, 5, -3),   # move right 5m
    (0, 5, -3),   # move backward 5m
    (0, 0, -3)    # move left 5m (back to start)
]

# --- Open CSV for logging ---
with open("square_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["step", "x", "y", "z"])

    step = 0

    # --- Fly through each waypoint ---
    for (x, y, z) in waypoints:
        print(f"Moving to waypoint: ({x}, {y}, {z})")
        client.moveToPositionAsync(x, y, z, 1).join()
        time.sleep(2)  # wait at each waypoint

        # Log 10 samples at each waypoint
        for i in range(10):
            pos = client.getMultirotorState().kinematics_estimated.position
            writer.writerow([step, pos.x_val, pos.y_val, pos.z_val])
            print(f"Log {step}: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
            step += 1
            time.sleep(0.5)

# --- Land ---
print("Landing...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)

print("Mission completed! Path logged to square_log.csv")
