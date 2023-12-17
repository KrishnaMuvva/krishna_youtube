# Importing the library
import airsim
import time

# Connect to the client
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable api control
client.enableApiControl(True, "uav")

print("Disarming the motors")

#Arming the motors
client.armDisarm(False, "uav")

time.sleep(1)


print("Arming the motors")

#Arming the motors
client.armDisarm(True, "uav")

time.sleep(1)

print("About to takeoff")
# Takeoff
uav_takeoff = client.takeoffAsync(vehicle_name = "uav")
uav_takeoff.join()

print("Hovering")

time.sleep(5)

print("About to land")

# Landing

uav_land = client.landAsync(vehicle_name = "uav")
uav_land.join()

# Disarm
client.armDisarm(False, "uav")

time.sleep(1)

print("Done")
