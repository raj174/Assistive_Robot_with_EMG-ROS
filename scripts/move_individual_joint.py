#!/usr/bin/env python
import rospy
import intera_interface
import numpy


rospy.init_node('Hello_Sawyer1')
limb = intera_interface.Limb('right')
current_pose = limb.endpoint_pose() 
print (current_pose)
#limb.move_to_neutral()
#rospy.sleep(0.5)
angles = limb.joint_angles()
print(angles)
question = "give a value for right_j5 in radians? "
question2 = "would you like to continue the loop?"

while True:
    if raw_input(question2) == "n":
        break
    angles['right_j5'] = eval(raw_input(question))
    print(angles)
    limb.move_to_joint_positions(angles)
    current_pose = limb.endpoint_pose() 
    print(current_pose)
    print(angles)
    rospy.sleep(0.5)
