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

def approach(pose):
    approach = copy.deepcopy(pose)
    # approach with a pose the hover-distance above the requested pose
    approach.position.z = approach.position.z + self._hover_distance
    joint_angles = self._limb.ik_request(approach, self._tip_name)
    self._limb.set_joint_position_speed(0.001)
    self._guarded_move_to_joint_position(joint_angles)
    self._limb.set_joint_position_speed(0.1)

"""def _servo_to_pose(pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
    r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
    current_pose = self._limb.endpoint_pose()
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
        joint_angles = self._limb.ik_request(ik_step, self._tip_name)
        if joint_angles:
            self._limb.set_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
        r.sleep()
    rospy.sleep(1.0)"""

def ik_service_client(limb = "right", tip_name = "right_hand", steps = 400.0):
    limb_mv = intera_interface.Limb(limb)
    gripper = intera_interface.Gripper()
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    # Add desired pose for inverse kinematics
    current_pose = limb_mv.endpoint_pose()
    #print current_pose 
    movement = [0.45,-0.453,-0.21]
    overhead_orientation = Quaternion(x=-0.00142460053167,y=0.999994209902,z=-0.00177030764765,w=0.00253311793936)
    #orientation = [0.0,1,0.0,0.0]
    #gripper.close()
    rospy.sleep(1.0)
    [dx,dy,dz] = movement
    #[ox,oy,oz,ow] = orientation
    dy = constrain(dy,-0.7596394482267009,0.7596394482267009)
    dz = constrain(dz, 0.1, 1)
    
    # up position [0.45,-0.453,0.6] 0.11 for pick up location

    #poses = {'right': PoseStamped(header=hdr,pose=Pose(position=Point(x=0.450628752997,y=0.161615832271,z=0.217447307078,),
    #orientation=Quaternion(x=0.704020578925,y=0.710172716916,z=0.00244101361829,w=0.00194372088834,),),),} neutral pose
    
    #x= 0.5,y = 0.5, z= 0.5,w= 0.5 for enpoint facing forward (orientation)
    #table side parametres x=0.6529605227057904, y= +-0.7596394482267009, z=0.10958623747123714)
    #table straight parametres x=0.99, y=0.0, z=0.1)
    poses = list()
    poses.append(Pose(position=Point(x=dx, y=dy, z=dz),orientation=overhead_orientation))
    pose = poses[0]
    """poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= dx,
                    y= dy,
                    z= dz,
                ),
                orientation=Quaternion(
                    x= ox,
                    y= oy,
                    z= oz,
                    w= ow,
                ),
            ),
        ),
    }"""

    #ikreq.pose_stamp.append(poses[limb])
    
    # Request inverse kinematics from base to "right_hand" link
    #ikreq.tip_names.append(tip_name) # right_hand, right_wrist, right_hand_camera 

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    limb_mv.set_joint_position_speed(0.05)
    ik_delta = Pose()
    ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
    print ik_delta.position.x
    print "Hi"
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
            limb_mv.set_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    # Check if result valid, and type of seed ultimately used to get solution
    # if (resp.result_type[0] > 0):
    #     seed_str = {
    #                 ikreq.SEED_USER: 'User Provided Seed',
    #                 ikreq.SEED_CURRENT: 'Current Joint Angles',
    #                 ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
    #                }.get(resp.result_type[0], 'None')
    #     rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
    #           (seed_str,))
    #     # Format solution into Limb API-compatible dictionary
    #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #    limb_mv.move_to_joint_positions(limb_joints)
        current_pose = limb_mv.endpoint_pose()
        print current_pose 
    #    rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    #    rospy.loginfo("------------------")
    #    rospy.loginfo("Response Message:\n%s", resp)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    return True


def main():
    """RSDK Inverse Kinematics

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, the example will use the default limb
    and call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    rospy.init_node("rsdk_ik_service_client")

    if ik_service_client():
        rospy.loginfo("Simple IK call passed!")
    else:
        rospy.logerr("Simple IK call FAILED")



if __name__ == '__main__':
    main()
