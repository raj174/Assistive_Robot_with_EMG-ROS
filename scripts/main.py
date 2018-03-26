#!/usr/bin/env python

import sys
import rospy
import intera_interface
import tf
import struct
import numpy as np

import argparse
import copy
import rospkg

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)

from gazebo_msgs.srv import (
	SpawnModel,
	DeleteModel,
)

from std_msgs.msg import Header
from sensor_msgs.msg import (JointState, Image)

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

class PickAndPlace(object):
	def __init__(self, limb="right", hover_distance = 0.30, tip_name="right_gripper_tip"):
		self._limb_name = limb # string
		self._tip_name = tip_name # string
		self._hover_distance = hover_distance # in meters
		self._limb = intera_interface.Limb(limb)
		self._gripper = intera_interface.Gripper()

		# verify robot is enabled
		print("Getting robot state... ")
		self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()

	def constrain(self, x, min_x, max_x):
		return min(max_x, max(x, min_x))

	def move_to_neutral(self):
		print("Moving the {0} arm to neutral pose...".format(self._limb_name))
		self._limb.move_to_neutral()
		self.gripper_open

	def move_to_home(self):
		home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': 0.0}
		self._guarded_move_to_joint_position(home_position)
		self.gripper_open()

	def move_to_start(self, start_angles = None):
		print("Moving the {0} arm to start pose...".format(self._limb_name))
		if not start_angles:
			start_angles = dict(zip(self._joint_names, [0]*7))
		self._guarded_move_to_joint_position(start_angles)
		self.gripper_open()

	def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

	def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

	def _approach(self, pose):
		approach = copy.deepcopy(pose)
		# approach with a pose the hover-distance above the requested pose
		approach.position.z = approach.position.z + self._hover_distance
		joint_angles = self._limb.ik_request(approach, self._tip_name)
		self._limb.set_joint_position_speed(0.1)
		self._guarded_move_to_joint_position(joint_angles)
		self._limb.set_joint_position_speed(0.1)

	def _retract(self):
		# retrieve current pose from endpoint
		current_pose = self._limb.endpoint_pose()
		ik_pose = Pose()
		ik_pose.position.x = current_pose['position'].x
		ik_pose.position.y = current_pose['position'].y
		ik_pose.position.z = current_pose['position'].z + self._hover_distance
		ik_pose.orientation.x = current_pose['orientation'].x
		ik_pose.orientation.y = current_pose['orientation'].y
		ik_pose.orientation.z = current_pose['orientation'].z
		ik_pose.orientation.w = current_pose['orientation'].w
		self._servo_to_pose(ik_pose)

	def _servo_to_pose(self, pose, time=4.0, steps=400.0):
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
				self._limb.move_to_joint_positions(joint_angles)
			else:
				rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
			r.sleep()
		rospy.sleep(1.0)

	def ik_service_client(limb = "right", tip_name = "right_hand", steps = 100.0):
	# Add desired pose for inverse kinematics
	#print current_pose
		current_pose = self._limb.endpoint_pose()
		movement = [0.45,-0.453,0.60] # height 0.24 to pickup location with tip right_hand
		overhead_orientation = Quaternion(x=-0.00142460053167,y=0.999994209902,z=-0.00177030764765,w=0.00253311793936)

		[dx,dy,dz] = movement

		dy = constrain(dy,-0.7596394482267009,0.7596394482267009)
		dz = constrain(dz, 0.1, 1)

		poses = list()
		poses.append(Pose(position=Point(x=dx, y=dy, z=dz),orientation=overhead_orientation))
		pose = poses[0]

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
			joint_angles = limb_mv.ik_request(ik_step, self._tip_name)
			if joint_angles:
				limb_mv.move_to_joint_positions(joint_angles)
			else:
				rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
			current_pose = limb_mv.endpoint_pose()
			print current_pose 
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			rospy.logerr("Result Error %d", resp.result_type[0])
			return False

		return True



	def pick(self, pose):
		if rospy.is_shutdown():
			return
		# open the gripper
		self.gripper_open()
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		if rospy.is_shutdown():
			return
		# close gripper
		self.gripper_close()
		# retract to clear object
		self._retract()

	def place(self, pose):
		if rospy.is_shutdown():
			return
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		if rospy.is_shutdown():
			return
		# open the gripper
		self.gripper_open()
		# retract to clear object
		self._retract()



# class camera_check(object):
# 	def __init__(self, limb="right", hover_distance = 0.30, tip_name="right_hand_camera"):
#         self._limb_name = limb # string
#         self._tip_name = tip_name # string
#         self._hover_distance = hover_distance # in meters
#         self._limb = intera_interface.Limb(limb)
#         self._camera = intera_interface.RobotParams()
#         # verify robot is enabled
#         print("Getting robot state... ")
#         self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
#         self._init_state = self._rs.state().enabled
#         print("Enabling robot... ")
#         self._rs.enable()

# class pick(object):
# 	def __init__(self, limb="right", hover_distance = 0.3, tip_name="right_gripper_tip"):
#         self._limb_name = limb # string
#         self._tip_name = tip_name # string
#         self._hover_distance = hover_distance # in meters
#         self._limb = intera_interface.Limb(limb)
#         self._gripper = intera_interface.Gripper()
#         # verify robot is enabled
#         print("Getting robot state... ")
#         self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
#         self._init_state = self._rs.state().enabled
#         print("Enabling robot... ")
#         self._rs.enable()

# def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
#                        table_reference_frame="world",
#                        block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
#                        block_reference_frame="world"):
#     # Get Models' Path
#     model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
#     # Load Table SDF
#     table_xml = ''
#     with open (model_path + "cafe_table/model.sdf", "r") as table_file:
#         table_xml=table_file.read().replace('\n', '')
#     # Load Block URDF
#     block_xml = ''
#     with open (model_path + "block/model.urdf", "r") as block_file:
#         block_xml=block_file.read().replace('\n', '')
#     # Spawn Table SDF
#     rospy.wait_for_service('/gazebo/spawn_sdf_model')
#     try:
#         spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
#         resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
#                              table_pose, table_reference_frame)
#     except rospy.ServiceException, e:
#         rospy.logerr("Spawn SDF service call failed: {0}".format(e))
#     # Spawn Block URDF
#     rospy.wait_for_service('/gazebo/spawn_urdf_model')
#     try:
#         spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
#         resp_urdf = spawn_urdf("block", block_xml, "/",
#                                block_pose, block_reference_frame)
#     except rospy.ServiceException, e:
#         rospy.logerr("Spawn URDF service call failed: {0}".format(e))

# def delete_gazebo_models():
#     # This will be called on ROS Exit, deleting Gazebo models
#     # Do not wait for the Gazebo Delete Model service, since
#     # Gazebo should already be running. If the service is not
#     # available since Gazebo has been killed, it is fine to error out
#     try:
#         delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
#         resp_delete = delete_model("cafe_table")
#         resp_delete = delete_model("block")
#     except rospy.ServiceException, e:
#         print("Delete Model service call failed: {0}".format(e))
def main():
	rospy.init_node("pick_and_place", anonymous = True)
	limb = 'right'
	hover_distance = 0.3 # meters
	#load_gazebo_models()
	#rospy.on_shutdown(delete_gazebo_models)

	# Starting Joint angles for right arm
	pnp = PickAndPlace(limb, hover_distance)

	starting_joint_angles = {'right_j0': 0,
							 'right_j1': -1.6,
							 'right_j2': 0,
							 'right_j3': 0,
							 'right_j4': 0,
							 'right_j5': 0,
							 'right_j6': 0}
	
	# An orientation for gripper fingers to be overhead and parallel to the obj
	overhead_orientation = Quaternion(
							 x=-0.00142460053167,
							 y=0.999994209902,
							 z=-0.00177030764765,
							 w=0.00253311793936)
	
	block_poses = list()
	block_poses.append(Pose(
		position=Point(x=0.45, y=-0.453, z=0.20),
		orientation=overhead_orientation))
	# Feel free to add additional desired poses for the object.
	# Each additional pose will get its own pick and place.
	block_poses.append(Pose(
		position=Point(x=0.45, y=0.0, z=0.20),
		orientation=overhead_orientation))

	print("Running. Ctrl-c to quit")
	pnp.move_to_neutral()
	rospy.sleep(1.0)
	#pnp.move_to_home()
	#pnp.move_to_start(starting_joint_angles)
	idx = 0
	while not rospy.is_shutdown():
	    print("\nPicking...")
	    pnp.pick(block_poses[idx])
	    print("\nPlacing...")
	    idx = (idx+1) % len(block_poses)
	    pnp.place(block_poses[idx])
	return 0

if __name__ == '__main__':
	main()