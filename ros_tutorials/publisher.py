import rospy
from std_msgs.msg import Float64
import random


rospy.init_node('publisher_node')

barometer = Float64()

rate = rospy.Rate(1)

def publisher_function(n):

	publisher = rospy.Publisher('/barometer_topic', Float64, queue_size = 1)

	for i in range(n):

		barometer.data = random.uniform(0,100)

		publisher.publish(barometer)

		rate.sleep()


publisher_function(10)