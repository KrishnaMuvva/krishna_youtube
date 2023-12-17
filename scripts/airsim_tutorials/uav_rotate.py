# Importing the library
import airsim
import time
import math

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

time.sleep(2)

# # Angular Displacement
# print("Angular Motion")
# uav_rotation = client.moveByRollPitchYawZAsync(math.radians(10),0,0,-1.5,2,vehicle_name = "uav")
# uav_rotation.join()

# Angular Velocity
print("Angular Velocity")
uav_angular_velocity = client.moveByRollPitchYawrateZAsync(0,0,math.radians(50),-1.5,5,vehicle_name = "uav")
uav_angular_velocity.join()


print("About to land")

# Landing

uav_land = client.landAsync(vehicle_name = "uav")
uav_land.join()

# Disarm
client.armDisarm(False, "uav")

time.sleep(1)

print("Done")
