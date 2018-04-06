#!/usr/bin/env python

import sys
import rospy
import serial
import time
import intera_interface

from std_msgs.msg import String

def callback(data):
	global val
	global start_time
	#global previous_val
	start_time = time.time()
	print start_time
	if not data_received:
		val = data.data
		if val == "None":
			val = None
	# if val == previous_val:
	# 	val = None
	


	# previous_val = val

def listener():
	rospy.Subscriber("Arduino_value", String, callback)


def talker():
	pub = rospy.Publisher('Pickup_object', String, queue_size=10)
	rospy.init_node('EMG_Interface', anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	head_display = intera_interface.HeadDisplay()
	listener()
	head_display.display_image(home_interface[0])
	idx = 0
	global start_time
	start_time = time.time()
	while not rospy.is_shutdown():
		if time.time() - start_time >= timeout:
			global val
			val = None
			print val
		if val == "Next" or val == "next":
			global data_received
			data_received = True
			idx = (idx+1) % len(home_interface)
			head_display.display_image(home_interface[idx])
			data_received = False
		if val == "select" or val == "Select":
			data_received = True
			head_display.display_image(selected_image[idx])

			data_received = False


	# while not rospy.is_shutdown():
	# 	rospy.loginfo(j)
	# 	pub.publish(j)
	# 	rate.sleep()

val = None
previous_val = None
data_received = False
start_time = time.time()
timeout = 5

image_source = "/home/raj/sawyer_ws/src/Raj/images/"
home_interface =[
				image_source + "Untitled.jpg",
				image_source + "Untitled-0.jpg",
				image_source + "Untitled-1.jpg",
				image_source + "Untitled-2.jpg",
				image_source + "Untitled-3.jpg"
				]

selected_image =[
				image_source + "Untitled.jpg",
				image_source + "Untitled-0.jpg",
				image_source + "Untitled-1.jpg",
				image_source + "Untitled-2.jpg",
				image_source + "Untitled-3.jpg"
				]

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

