#!/usr/bin/env python

import sys
import rospy
import intera_interface
import tf
import struct
import copy
import time

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
import PyKDL
from tf_conversions import posemath
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
	def __init__(self, name='drop1', x=0.3, y=0.3, z=0.7, hover_offset=0.3):
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

		self.drop_locations['ketchup']   	= DropLocation('drop1', 0.60, 0.0, 0.23, self._hover_distance)
		self.drop_locations['mayonaise'] 	= DropLocation('drop1', 0.60, 0.0, 0.23, self._hover_distance)
		self.drop_locations['barbecue']  	= DropLocation('drop1', 0.60, 0.0, 0.23, self._hover_distance)
		self.drop_locations['salad']   		= DropLocation('drop1', 0.60, 0.0, 0.23, self._hover_distance)


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
		self.gripper_open()
		self._limb.set_joint_position_speed(0.2)
		home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.1, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': -0.0}
		f = lambda x : int(x[-1])
		set_home_position = [home_position[i] for i in sorted(home_position.keys(), key=f)]
		self._guarded_move_to_joint_position_two(set_home_position)
		#self._guarded_move_to_joint_position(home_position)
		

	def _guarded_move_to_joint_position(self, joint_angles, timeout=10.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
			#print (self._limb.endpoint_pose())
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def _guarded_move_to_joint_position_two(self, set_joint_angles, speed_ratio = 0.4, accel_ratio = 0.2, timeout = 10.0):
		try:
			traj = MotionTrajectory(limb = self._limb)
			wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,
			                                 max_joint_accel=accel_ratio)
			waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

			joint_angles = self._limb.joint_ordered_angles()

			waypoint.set_joint_angles(joint_angles = joint_angles)
			traj.append_waypoint(waypoint.to_msg())
			if len(set_joint_angles) != len(joint_angles):
			 	rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
			 	return None

			waypoint.set_joint_angles(joint_angles = set_joint_angles)
			traj.append_waypoint(waypoint.to_msg())

			result = traj.send_trajectory(timeout= timeout)
			if result is None:
				rospy.logerr('Trajectory FAILED to send')
				return

			if result.result:
				rospy.loginfo('Motion controller successfully finished the trajectory!')
			else:
				rospy.logerr('Motion controller failed to complete the trajectory with error %s',
				result.errorId)
		except rospy.ROSInterruptException:
			rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')

	def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

	def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

	def get_overhead_orientation(self,pose):
		current_pose = copy.deepcopy(pose)
		current_pose.orientation = self.overhead_orientation
		return current_pose

	def _approach(self, pose, speed = 0.3, timeout = 5.0, speed_ratio = 0.5, accel_ratio = 0.5):
		approach = copy.deepcopy(pose)
		approach.position.z = approach.position.z + self._hover_distance
		set_joint_angles = self._limb.ik_request(approach, self._tip_name)

		f = lambda x : int(x[-1])
		set_joint_angles = [set_joint_angles[i] for i in sorted(set_joint_angles.keys(), key=f)]

		#print set_joint_angles
		self._limb.set_joint_position_speed(speed)
		#rospy.sleep(1.0)
		self._guarded_move_to_joint_position_two(set_joint_angles)
		#self._guarded_move_to_joint_position(set_joint_angles)
		#self._limb.set_joint_position_speed(0.2)

	def _retract(self, distance = 0):
		# retrieve current pose from endpoint
		current_pose = self._limb.endpoint_pose()
		ik_pose = Pose()
		ik_pose.position.x = current_pose['position'].x
		ik_pose.position.y = current_pose['position'].y
		ik_pose.position.z = current_pose['position'].z + self._hover_distance - distance
		ik_pose.orientation.x = current_pose['orientation'].x
		ik_pose.orientation.y = current_pose['orientation'].y
		ik_pose.orientation.z = current_pose['orientation'].z
		ik_pose.orientation.w = current_pose['orientation'].w
		self.linear_movement(ik_pose)

	def _servo_to_pose(self, pose, time=4.0, steps=500.0):
		''' An *incredibly simple* linearly-interpolated Cartesian move '''
		r = rospy.Rate(1/(time/steps)) 
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

	def linear_movement(self,position, linear_speed = 0.2, linear_accel = 0.2, rotational_speed = 0.1, rotational_accel = 0.1):
	   	try:
			traj_options = TrajectoryOptions()
			traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
			traj = MotionTrajectory(trajectory_options = traj_options, limb = self._limb)

			wpt_opts = MotionWaypointOptions(max_linear_speed=linear_speed,
			                                 max_linear_accel=linear_accel,
			                                 max_rotational_speed=rotational_speed,
			                                 max_rotational_accel=rotational_accel,
			                                 max_joint_speed_ratio=0.2)
			waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)
			poseStamped = PoseStamped()
			poseStamped.pose = position
			waypoint.set_cartesian_pose(poseStamped, self._tip_name)

			rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

			traj.append_waypoint(waypoint.to_msg())

			result = traj.send_trajectory(timeout=5.0)
			if result is None:
			    rospy.logerr('Trajectory FAILED to send')
			    return

			if result.result:
			    rospy.loginfo('Motion controller successfully finished the trajectory!')
			else:
			    rospy.logerr('Motion controller failed to complete the trajectory with error %s',
			                 result.errorId)
		except rospy.ROSInterruptException:
			rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')

	def pick(self, pose_to_move):
		pose = self.get_overhead_orientation(pose_to_move)
		if rospy.is_shutdown():
			return
		# open the gripper
		self.gripper_open()
		# servo above pose
		self._approach(pose)
		# servo to pose
		self.linear_movement(pose)
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
		self._approach(self.drop_locations[drop_location].pose, speed = 0.001)
		# servo to pose
		self.linear_movement(self.drop_locations[drop_location].pose)
		if rospy.is_shutdown():
			return
		# open the gripper
		self.gripper_open()
		# retract to clear object
		self._retract(0.10)


def callback(data):
	global val
	global start_time
	start_time = time.time()
	#print start_time
	if data_receive:
		val = data.data



#Code to check if the programme is running without having to listen to any topic
# def main():
# 	rospy.init_node("pick_and_place", anonymous = True)
# 	limb = 'right'
# 	hover_distance = 0.3 # meters
# 	tip_name = 'right_hand'
# 	product_name = ["ketchup","mayonaise","barbeque","salad"]
# 	pnp = PickAndPlace(limb, hover_distance,tip_name)
# 	cop = CheckObjectPosition(limb,hover_distance)
# 	product_pose = 
# 	cop.move_to_home()
# 	rospy.sleep(1.0)
# 	while True:
# 		if rospy.is_shutdown():
# 			break
# 		question = "What do you want to pick? "
# 		chosen_position = int(raw_input(question))
# 		if chosen_position <= len(product_name):
# 			product_to_pick = product_name[chosen_position]
# 		else:
# 			break
# 		product_pose = cop.check_object(product_to_pick)
# 		if not product_pose:
# 			product_pose = cop.check_object(product_to_pick,False)
# 		print("Running. Ctrl-c to quit")
# 		#pnp.move_to_neutral()
# 		rospy.sleep(1.0)
# 		print("\nPicking...")
# 		pnp.pick(product_pose)
# 		print("\nPlacing...")
# 		pnp.place("drop1")

def pickup_listner():
	rospy.Subscriber("Pickup_object", String, callback)

def main():
	global data_receive
	rospy.init_node("pick_and_place", anonymous = True)
	pub = rospy.Publisher('pick_and_place_state', String, queue_size=10)
	limb = 'right'
	hover_distance = 0.3 # meters
	tip_name = 'right_hand'
	pnp = PickAndPlace(limb, hover_distance,tip_name)
	cop = CheckObjectPosition(limb,hover_distance)
	pickup_listner()
	pnp.move_to_home()
	rospy.sleep(1.0)
	while not rospy.is_shutdown():
		global val
		if time.time() - start_time >= timeout:
			val = "none"
		product_to_pick = val 
		if product_to_pick.lower() == "none":
			pass
		else:
			data_receive = False
			product_pose = cop.check_object(product_to_pick)
			if not product_pose:
				product_pose = cop.check_object(product_to_pick,False)
			if product_pose == False:
				pnp.move_to_home()
			else:
				print("Running. Ctrl-c to quit")
				rospy.sleep(1.0)
				pub.publish("picking")
				rospy.loginfo("Picking...")
				pnp.pick(product_pose)
				pub.publish("placing")
				rospy.loginfo("Placing...")
				pnp.place(product_to_pick)
				new_drop_position = cop.product_position(product_to_pick,pnp.drop_locations[product_to_pick].position)
				pnp.drop_locations[product_to_pick].set_pose(new_drop_position.x,new_drop_position.y,new_drop_position.z) 
				pnp.move_to_home()
			data_receive = True
			

val = "none"
previous_val = None
data_receive = True
start_time = time.time()
timeout = 1

if __name__ == '__main__':
	main()
