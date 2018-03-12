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



def ik_service_client(limb = "right", use_advanced_options = False):
    limb_mv = intera_interface.Limb('right')
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    #poses = {'right': PoseStamped(header=hdr,pose=Pose(position=Point(x=0.450628752997,y=0.161615832271,z=0.217447307078,),orientation=Quaternion(x=0.704020578925,y=0.710172716916,z=0.00244101361829,w=0.00194372088834,),),),} neutral pose

    # Add desired pose for inverse kinematics
    current_pose = limb_mv.endpoint_pose()
    print current_pose 
    movement = [0.0,0.0,0.0]
    orientation = [0.0,0.0,0.0,0.0]
    [dx,dy,dz] = movement
    [ox,oy,oz,ow] = orientation
    
    #x= 0.45338495447986793,y = 0.5440510565212842, z= 0.535113472086493,w= 0.4605476120238838 for enpoint facing forward 

    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=current_pose['position'].x + dx,
                    y=current_pose['position'].y + dy,
                    z=current_pose['position'].z + dz,
                ),
                orientation=Quaternion(
                    x= 0.45338495447986793, #current_pose['orientation'].x + ox,
                    y= 0.5440510565212842, #current_pose['orientation'].y + oy,
                    z= 0.535113472086493, #current_pose['orientation'].z + oz,
                    w= 0.4605476120238838, #current_pose['orientation'].w + ow,
                ),
            ),
        ),
    }
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb_mv.move_to_joint_positions(limb_joints)
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
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
