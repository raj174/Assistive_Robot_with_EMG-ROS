#!/usr/bin/env python
import rospy
import intera_interface


rospy.init_node('Hello_Sawyer', anonymous = True)
limb_mv = intera_interface.Limb('right')
limb_mv.set_joint_position_speed(0.1)
current_pose = limb_mv.endpoint_pose() 
print (current_pose)

home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': -0.0}

question = "What position do you want to go to n or h ? "
chosen_position = raw_input(question).lower()
print(chosen_position)

if chosen_position == "n":
    limb_mv.move_to_neutral()
    print (limb_mv.joint_angles())
    current_pose = limb_mv.endpoint_pose() # get current endpoint pose
    print current_pose 
    rospy.sleep(0.5)
elif chosen_position == "h":
    limb_mv.move_to_joint_positions(home_position)
    print (limb_mv.joint_angles())
    current_pose = limb_mv.endpoint_pose() # get current endpoint pose
    print current_pose 
    rospy.sleep(0.5)
