#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Intera RSDK Inverse Kinematics Example
"""
import sys
import rospy
import intera_interface
import tf
import struct
import numpy as np
import copy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def constrain(x, min_x, max_x):
    return min(max_x, max(x, min_x))


def move_up_down(limb = "right", tip_name = "right_hand", steps = 400.0, movement = None):
    limb_mv = intera_interface.Limb(limb)
    current_pose = limb_mv.endpoint_pose()
    #print current_pose 
    overhead_orientation = Quaternion(x=-0.00142460053167,y=0.999994209902,z=-0.00177030764765,w=0.00253311793936)
    if not movement:
        return
    [dx,dy,dz] = movement
    dy = constrain(dy,-0.7596394482267009,0.7596394482267009)
    dz = constrain(dz, 0.1, 1)
    
    # up position [0.45,-0.453,0.6] 0.11 for pick up location
    
    #x= 0.5,y = 0.5, z= 0.5,w= 0.5 for enpoint facing forward (orientation)
    #table side parametres x=0.6529605227057904, y= +-0.7596394482267009, z=0.10958623747123714)
    #table straight parametres x=0.99, y=0.0, z=0.1)
    poses = list()
    poses.append(Pose(position=Point(x=dx, y=dy, z=dz),orientation=overhead_orientation))
    pose = poses[0]
    r = rospy.Rate(1/(4.0/steps))
    limb_mv.set_joint_position_speed(0.1)
    ik_delta = Pose()
    ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps

    ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
    ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
    ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
    ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
    ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
    ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
    for d in range(int(steps), -1, -1):
        if rospy.is_shutdown():
            return
        ik_step = Pose()
        ik_step.position.x = d*ik_delta.position.x + pose.position.x
        ik_step.position.y = d*ik_delta.position.y + pose.position.y
        ik_step.position.z = d*ik_delta.position.z + pose.position.z
        ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
        ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
        ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
        ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
        joint_angles = limb_mv.ik_request(ik_step, tip_name)
        if joint_angles:
            limb_mv.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
        current_pose = limb_mv.endpoint_pose()
        r.sleep()
        #print current_pose
    rospy.sleep(1.0) 
    return True


def main():

    rospy.init_node("rsdk_ik_service_client")
    # height 0.24 to pickup location with tip right_hand
    if move_up_down(movement = [0.45,-0.453,0.6]):
        rospy.loginfo("call passed!")
    else:
        rospy.logerr("call FAILED")



if __name__ == '__main__':
    main()
