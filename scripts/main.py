#!/usr/bin/env python

import sys
import rospy
import intera_interface
import tf
import struct
import copy

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from check_object_position import CheckObjectPosition

from sensor_msgs.msg import (JointState, Image)
from std_msgs.msg import String

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

class DropLocation(object):
	def __init__(self, name='drop1', x=0.3, y=0.3, z=0.7, hover_offset=0.4):
		self.name = name
		self.orientation = Quaternion(x=-0.00142460053167,y=0.999994209902,	z=-0.00177030764765,w=0.00253311793936)
		self.position = None
		self.pose = None
		self.hover_pose = None
		self.hover_position = None
		self.hover_offset = hover_offset

		self.set_pose(x,y,z)

	def set_pose(self, x, y, z):
		self.position = Point(x=x, y=y, z=z)
		self.pose = Pose(position=self.position, orientation=self.orientation)

		self.hover_position = copy.deepcopy(self.position)
		self.hover_position.z = self.position.z + self.hover_offset
		self.hover_pose = Pose(position=self.hover_position, orientation=self.orientation)

class PickAndPlace(object):
	def __init__(self, limb="right", hover_distance = 0.30, tip_name="right_hand"):
		self._limb_name = limb # string
		self._tip_name = tip_name # string
		self._hover_distance = hover_distance # in meters
		self._limb = intera_interface.Limb(limb)
		self._gripper = intera_interface.Gripper()
		self._rp = intera_interface.RobotParams()
		self.overhead_orientation = Quaternion(x=-0.00142460053167,y=0.999994209902,z=-0.00177030764765,w=0.00253311793936)

		self.drop_locations = {}

		self.drop_locations['drop1']   	= DropLocation('drop1', 0.60, 0.0, 0.225)
		self.drop_locations['drop2'] 	= DropLocation('drop2', 0.60, 0.0, 0.225)
		self.drop_locations['drop3']  	= DropLocation('drop3', 0.60, 0.0, 0.225)
		self.drop_locations['drop4']   	= DropLocation('drop4', 0.60, 0.0, 0.225)


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
		self.gripper_open()

	def move_to_home(self):
		self._limb.set_joint_position_speed(0.2)
		home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': 0.0}
		self._guarded_move_to_joint_position(home_position)
		self.gripper_open()

	def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
			#print (self._limb.endpoint_pose())
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

	def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

	def get_overhead_orientation(self,pose):
		current_pose = copy.deepcopy(pose)
		current_pose.position.z = 0.245
		current_pose.orientation = self.overhead_orientation
		return current_pose

	def _approach(self, pose):
		approach = copy.deepcopy(pose)
		approach.position.z = approach.position.z + self._hover_distance
		joint_angles = self._limb.ik_request(approach, self._tip_name)
		self._limb.set_joint_position_speed(0.3)
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

	def _servo_to_pose(self, pose, time=4.0, steps=500.0):
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

	def pick(self, pose):
		pose = self.get_overhead_orientation(pose)
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

	def place(self, drop_location):
		if rospy.is_shutdown():
			return
		# servo above pose
		self._approach(self.drop_locations[drop_location].pose)
		# servo to pose
		self._servo_to_pose(self.drop_locations[drop_location].pose)
		if rospy.is_shutdown():
			return
		# open the gripper
		self.gripper_open()
		# retract to clear object
		self._retract()


def callback(data):
	global val 
	val = data.data

def main():
	rospy.init_node("pick_and_place", anonymous = True)
	limb = 'right'
	hover_distance = 0.3 # meters
	tip_name = 'right_hand'
	product_name = ["ketchup","mayonaise","barbeque","salad"]
	pnp = PickAndPlace(limb, hover_distance,tip_name)
	cop = CheckObjectPosition(limb,hover_distance)

	cop.move_to_home()
	rospy.sleep(1.0)
	while True:
		if rospy.is_shutdown():
			break
		question = "What do you want to pick? "
		chosen_position = int(raw_input(question))
		if chosen_position <= len(product_name):
			product_to_pick = product_name[chosen_position]
		else:
			break
		product_pose = cop.check_object(product_to_pick)
		if not product_pose:
			product_pose = cop.check_object(product_to_pick,False)
		print("Running. Ctrl-c to quit")
		#pnp.move_to_neutral()
		rospy.sleep(1.0)
		print("\nPicking...")
		pnp.pick(product_pose)
		print("\nPlacing...")
		pnp.place("drop1")
def arduino_listner():
	rospy.Subscriber("Pickup_object", String, callback)

# def main():
# 	rospy.init_node("pick_and_place", anonymous = True)
# 	limb = 'right'
# 	hover_distance = 0.3 # meters
# 	tip_name = 'right_hand'
# 	pnp = PickAndPlace(limb, hover_distance,tip_name)
# 	cop = CheckObjectPosition(limb,hover_distance)
# 	arduino_listner()
# 	cop.move_to_home()
# 	rospy.sleep(1.0)
# 	while True:
# 		if rospy.is_shutdown():
# 			break
# 		product_to_pick = val 
# 		if product_to_pick == "None":
# 			pass
# 		else:
# 			product_pose = cop.check_object(product_to_pick)
# 			if not product_pose:
# 				product_pose = cop.check_object(product_to_pick,False)
# 			if product_pose == False:
# 				pass
# 			else:
# 				print("Running. Ctrl-c to quit")
# 				#pnp.move_to_neutral()
# 				rospy.sleep(1.0)
# 				print("\nPicking...")
# 				pnp.pick(product_pose)
# 				print("\nPlacing...")
# 				pnp.place("drop1")
			


val = None

if __name__ == '__main__':
	main()