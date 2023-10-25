import airsim
import matplotlib.pyplot as plt
import numpy as np
import random


class Control:

	def __init__(self):

		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()

		self.client.enableApiControl(True)

		self.uav_state = self.client.getMultirotorState()

		# High Rise time
		#self.Kp = 0.1

		# No oscillations
		#self.Kp = 0.5

		# Small Oscillations
		self.Kp = 1

		# Unstable System
		#self.Kp = 5

		# PI - Steady state error
		self.Kp, self.Ki = 0.5, 0.1

		self.Kp, self.Kd, self.Ki = 0.5, 3.5, 0.1

		self.reference_alt, self.alt_val = -10, 0

		self.error, self.prev_error, self.error_sum = 0, 0, 0

		self.count = 0

		self.z, self.z_ref, self.time_stamps = [], [], []

		self.wind = airsim.Vector3r(0, 0, 0)
		self.client.simSetWind(self.wind)

		print(self.client.getBarometerData())

		print(self.uav_state.kinematics_estimated.position)

	
	def Sensor_Update(self):

		self.uav_state = self.client.getMultirotorState()

		self.z.append(abs(self.uav_state.kinematics_estimated.position.z_val))
		self.z_ref.append(abs(self.reference_alt))

	
	def Takeoff(self):

		self.client.armDisarm(True)

		self.client.takeoffAsync().join()


	def Open_Loop(self):

		vz = self.reference_alt - self.uav_state.kinematics_estimated.position.z_val

		self.velocity_control = self.client.moveByVelocityAsync(0, 0, vz, 1)

		self.velocity_control.join()


	def PID_Control(self, iterations):

		for i in range(iterations):

			self.Sensor_Update()

			self.error = self.reference_alt - self.uav_state.kinematics_estimated.position.z_val

			if i > 0:

				vz = self.Kp*(self.error) + self.Ki*(self.error_sum) + self.Kd*(self.error - self.prev_error)

			else:

				vz = self.Kp*(self.error)

			self.error_sum += self.error

			self.prev_error = self.error 

			self.velocity_control = self.client.moveByVelocityAsync(0, 0, vz, 1)

			self.velocity_control.join()

			#self.reference_alt -= 1

			self.reference_alt -= (self.count+1)

			self.count += 0.2



	def PD_Control(self, iterations):

		for i in range(iterations):

			self.Sensor_Update()

			self.error = self.reference_alt - self.uav_state.kinematics_estimated.position.z_val

			if i > 0:

				vz = self.Kp*(self.error) + self.Kd*(self.error - self.prev_error)

			else:

				vz = self.Kp*(self.error)

			self.prev_error = self.error 

			self.velocity_control = self.client.moveByVelocityAsync(0, 0, vz, 1)

			self.velocity_control.join()

			self.reference_alt -= self.count

			self.count += 0.2


	def PI_Control(self, iterations):

		for i in range(iterations):

			self.Sensor_Update()

			self.error = self.reference_alt - self.uav_state.kinematics_estimated.position.z_val

			if i > 0:

				vz = self.Kp*(self.error) + self.Ki*(self.error_sum)

			else:

				vz = self.Kp*(self.error)

			self.error_sum += self.error 

			self.velocity_control = self.client.moveByVelocityAsync(0, 0, vz, 1)

			self.velocity_control.join()

			#self.reference_alt -= 1

			self.reference_alt -= (self.count+1)

			self.count += 0.2



	def P_Control(self, iterations):

		for i in range(iterations):

			self.Sensor_Update()

			self.error = self.reference_alt - self.uav_state.kinematics_estimated.position.z_val

			vz = self.Kp*(self.error)

			self.velocity_control = self.client.moveByVelocityAsync(0, 0, vz, 1)

			self.velocity_control.join()

			self.reference_alt -= 1


	def main(self, iterations):

		self.Takeoff()

		self.Sensor_Update()

		#self.Open_Loop()

		#self.P_Control(iterations)

		#self.PI_Control(iterations)

		self.PID_Control(iterations)

		#self.PD_Control(iterations)

		self.Sensor_Update()

		self.time_stamps = np.arange(len(self.z_ref))

		print(self.time_stamps)
		print(self.z_ref)
		print(self.z)

		plt.plot(self.time_stamps, self.z_ref)
		plt.plot(self.time_stamps, self.z)
		plt.show()

		print(np.mean(np.array(self.z_ref) - np.array(self.z)))


control = Control()
control.main(25)