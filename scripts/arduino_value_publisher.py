#!/usr/bin/env python

from __future__ import absolute_import, division, unicode_literals, print_function
 
import tty, termios
import time
import rospy
import serial
import sys

if sys.version_info.major < 3:
    import thread as _thread
else:
    import _thread

from std_msgs.msg import String


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
def keypress():
    global char
    char = getch()

try:
	ser = serial.Serial('/dev/ttyACM0', 9600)
except:
	pass

def talker():
	pub = rospy.Publisher('Arduino_value', String, queue_size=10)
	rospy.init_node('ArduinoSerial', anonymous=True)
	global char 
	char = "None"
	#_thread.start_new_thread(keypress, ())

	while not rospy.is_shutdown():
		j = ser.readline().replace('\r\n','')
		j = j.lower()
		if j == 'next':
			rospy.loginfo(j)
			pub.publish(j)
			j = "None"
		elif j =='select':
			rospy.loginfo(j)
			pub.publish(j)
			j = "None"
		
		elif j =='back':
			rospy.loginfo(j)
			pub.publish(j)
			j = "None"
		else:
			pass
			# rospy.loginfo("None")
			# pub.publish("None")
	# while not rospy.is_shutdown():
	# 	try:
	# 		j = ser.readline().replace('\r\n','')
	# 		j = j.lower()
	# 	except:
	# 		j = None
	# 	if j is not None:
	# 		if j == 'next':
	# 			rospy.loginfo(j)
	# 			pub.publish(j)
	# 		elif j =='select':
	# 			rospy.loginfo(j)
	# 			pub.publish(j)
			
	# 		elif j =='back':
	# 			rospy.loginfo(j)
	# 			pub.publish(j)
	# 		else:
	# 			rospy.loginfo("None")
	# 			pub.publish("None")
	# 	else:
	# 		if char.decode('utf-8') == 'n':
	# 			rospy.loginfo("next")
	# 			pub.publish("next")
	# 			char = "None"
	# 			_thread.start_new_thread(keypress, ())
	# 		elif char.decode('utf-8') == 's':
	# 			rospy.loginfo("select")
	# 			pub.publish("select")
	# 			char = "None"
	# 			_thread.start_new_thread(keypress, ())
	# 		elif j =='back' or char.decode('utf-8') == 'b':
	# 			rospy.loginfo("back")
	# 			pub.publish("back")
	# 			char = "None"
	# 			_thread.start_new_thread(keypress, ())
	# 		elif char == 'q' or char == '\x1b':  # x1b is ESC
	# 			rospy.exit()
	# 		else:
	# 			#rospy.loginfo("None")
	# 			#pub.publish("None")
	# 			pass
		rospy.sleep(2)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
