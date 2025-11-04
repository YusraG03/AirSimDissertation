import airsim
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()

# Fly forward
print("Flying forward...")
client.moveToPositionAsync(5, 0, -3, 2).join()
time.sleep(2)

print("Landing...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)

print("Mission complete!")
