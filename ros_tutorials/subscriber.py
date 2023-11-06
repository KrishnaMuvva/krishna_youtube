import rospy
from std_msgs.msg import Float64

rospy.init_node('subscriber_node')

def cb_function(barometer):

	print("Barometer data ", barometer.data)

def subscriber_function():

	subscriber = rospy.Subscriber('/barometer_topic', Float64, cb_function)

	rospy.spin()


subscriber_function()