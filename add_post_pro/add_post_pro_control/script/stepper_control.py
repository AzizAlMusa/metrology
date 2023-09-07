#!/usr/bin/env python
#  --------------------------------VERSION 1-------------------------------------
# import rospy
# from std_msgs.msg import Int32

# def publisher(data):
# 	# define the actions the publisher will make
# 	pub = rospy.Publisher('stepper_position',Int32, queue_size=10)
# 	# initialize the publishing node
# 	rospy.init_node('stepper_control', anonymous=True)
	
# 	# define how many times per second
# 	# will the data be published
# 	# let's say 10 times/second or 10Hz
# 	rate = rospy.Rate(10)
# 	# to keep publishing as long as the core is running
# 	while not rospy.is_shutdown():

# 		# you could simultaneously display the data
# 		# on the terminal and to the log file
# 		rospy.loginfo(data)
		
# 		# publish the data to the topic using publish()
# 		pub.publish(data)
		
# 		# keep a buffer based on the rate defined earlier
# 		rate.sleep()

# # def main():
# #     rospy.init_node('stepper_control', anonymous=True)
# # 	rospy.spin()

# if __name__ == '__main__':
# 	# it is good practice to maintain
# 	# a 'try'-'except' clause   
# 	try:
# 		# rospy.init_node('stepper_control', anonymous=True)
# 		# rospy.spin()
# 		publisher(100)

# 	except rospy.ROSInterruptException:
# 		pass
# -----------------------------VERSION 2--------------------------------------
import rospy
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('python_publisher')

# Create ROS publisher
pub = rospy.Publisher('toggle_led', String, queue_size=10)

# Loop to continuously publish messages to ROS topic
while not rospy.is_shutdown():
    msg = 'Hello from Python!'
    rospy.loginfo('Publishing message: %s', msg)
    pub.publish(msg)
    rospy.sleep(1)
