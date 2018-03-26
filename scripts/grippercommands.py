#!/usr/bin/env python

import sys
import rospy
import intera_interface


rospy.init_node('Gripper_commands', anonymous = True)
gripper = intera_interface.Gripper()
question = "Open or Close gripper? "
chosen_command = raw_input(question).lower()


if chosen_command == "o":
	gripper.open()
	rospy.sleep(0.5)
elif chosen_command == "c":
	gripper.close()
	rospy.sleep(0.5)
elif chosen_command == "w":
	print(gripper.get_object_weight())
