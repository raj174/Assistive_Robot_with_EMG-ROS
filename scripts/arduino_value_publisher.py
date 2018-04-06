#!/usr/bin/env python

import sys
import rospy
import serial

from std_msgs.msg import String

ser = serial.Serial('/dev/ttyACM0', 9600)

def talker():
	pub = rospy.Publisher('Arduino_value', String, queue_size=10)
	rospy.init_node('ArduinoSerial', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		j = ser.readline().replace('\r\n','')
		j = j.lower()
		if j == 'next':
			rospy.loginfo(j)
			pub.publish(j)
		elif j =='select':
			rospy.loginfo(j)
			pub.publish(j)
		elif j =='back':
			rospy.loginfo(j)
			pub.publish(j)
		else:
			rospy.loginfo("None")
			pub.publish("None")
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
