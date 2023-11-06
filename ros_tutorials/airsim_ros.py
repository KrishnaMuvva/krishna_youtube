import airsim
import math
import time

import numpy as np

import matplotlib.pyplot as plt

import rospy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import Takeoff, Land
from sensor_msgs.msg import Range

class UAS:

	def __init__(self):

		rospy.init_node('airsim_ros_node')

		self.airsim_client = airsim.MultirotorClient()

		self.airsim_client.confirmConnection()

		self.airsim_client.enableApiControl(True, "UAS")

		self.potential_constant = 10

		self.distance_threshold = 15

		self.velocity = VelCmd()

		self.vel_pub = rospy.Publisher('/airsim_node/UAS/vel_cmd_body_frame', VelCmd, queue_size = 1)

		self.range = Range()

		self.range_sub = rospy.Subscriber('/airsim_node/UAS/distance/TOF', Range, self.range_cb)

		self.rate = rospy.Rate(10)

	def range_cb(self, range_data):

		self.range = range_data


	def Arm(self, vehicle_name):

		print("Arm")

		self.airsim_client.armDisarm(True, vehicle_name)

	def DisArm(self, vehicle_name):

		print("Disarm")

		self.airsim_client.armDisarm(False, vehicle_name)

	def Takeoff(self, vehicle_name):

		print("Takeoff")

		uav_takeoff = self.airsim_client.takeoffAsync(vehicle_name = vehicle_name)
		uav_takeoff.join()

		flex_row_alt = self.airsim_client.moveByVelocityAsync(0,0,-1.5,1,vehicle_name = "UAS")
		flex_row_alt.join()



	def Land(self, vehicle_name):

		print("Land")

		uav_land = self.airsim_client.landAsync(vehicle_name = vehicle_name)
		uav_land.join()




	def navigation(self, steps):

		for i in range(steps):

			if self.range.range > self.distance_threshold:

				self.velocity.twist.linear.x = 2

				self.velocity.twist.angular.z = 0

				self.vel_pub.publish(self.velocity)

				print("Moving Forward", self.range.range)

			else:

				self.velocity.twist.linear.x = 0

				self.velocity.twist.angular.z = math.radians(80)

				self.vel_pub.publish(self.velocity)

				print("Turning around yaw", self.range.range)


			self.rate.sleep()

	def main(self):

		self.Takeoff("UAS")

		time.sleep(1)

		self.navigation(150)

		self.Land("UAS")

		time.sleep(1)

uas = UAS()
uas.main()
