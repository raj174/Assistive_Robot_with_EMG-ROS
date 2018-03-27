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

from sensor_msgs.msg import (JointState, Image)

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

class PickAndPlace(object):
	def __init__(self, limb="right", hover_distance = 0.30, tip_name="right_hand", camera_name = "right_hand_camera"):
		self._limb_name = limb # string
		self._tip_name = tip_name # string
		self._hover_distance = hover_distance # in meters
		self._camera_name = camera_name
		self._limb = intera_interface.Limb(limb)
		self._gripper = intera_interface.Gripper()
		self._rp = intera_interface.RobotParams()
		self._light = intera_interface.Lights()
		self._cameras = intera_interface.Cameras()


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
			#print (self._limb.endpoint_pose())
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
		#print (approach)
		# approach with a pose the hover-distance above the requested pose
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



class check_object_position(object):
	def __init__(self, limb="right", hover_distance = 0.40, tip_name="right_hand_camera", camera_name = "right_hand_camera"):
		self._limb_name = limb # string
		self._tip_name = tip_name # string
		self._hover_distance = hover_distance # in meters
		self._camera_name = camera_name

		self._limb = intera_interface.Limb(limb)
		self._gripper = intera_interface.Gripper()
		self._rp = intera_interface.RobotParams()
		self._light = intera_interface.Lights()
		self._cameras = intera_interface.Cameras()

	def move(self,pose):
		move = copy.deepcopy(pose)

	def _hover_position(self, pose):
		approach = copy.deepcopy(pose)
		approach.position.z = approach.position.z + self._hover_distance
		joint_angles = self._limb.ik_request(approach, self._tip_name)
		self._limb.set_joint_position_speed(0.2)
		self._guarded_move_to_joint_position(joint_angles)

	def _movedown(self,pose):
		movedown = copy.deepcopy(pose)
		joint_angles = self._limb.ik_request(movedown, self._tip_name)
		self._limb.set_joint_position_speed(0.1)
		self._guarded_move_to_joint_position(joint_angles)

	def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
			#print (self._limb.endpoint_pose())
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def check_pick_up_object(self,product):



	def check_object(self, pose, find = None, hover_position = True):
		if rospy.is_shutdown():
			return

		if find == None :
			rospy.logerr("Object to find is not provided.")
		else:
			if hover_position == True:
				self._hover_position(pose)
			self._movedown(pose)
			if check_pick_up_object(find):
				self._hover_position(pose)
				return True
			return False



	#need to confirm

	def camera_check(self):
	    valid_cameras = self._rp.get_camera_names()
	    if not valid_cameras:
	        self._rp.log_message(("Cannot detect any camera_config"
	            " parameters on this robot. Exiting."), "ERROR")
	        return

	    if not self._cameras.verify_camera_exists(camera_name):
	        rospy.logerr("Could not detect the specified camera, exiting.")
	        return
	    rospy.loginfo("Opening camera '{0}'...".format(camera_name))
	    cameras.start_streaming(camera_name)
	    #cameras.set_cognex_strobe(False)
	    cameras.set_callback(camera_name, self.show_image_callback, rectify_image=False, callback_args=None)

	def show_image_callback(self,img_data):
	    """The callback function to show image by using CvBridge and cv
	    """
	    bridge = CvBridge()
	    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	    parameters =  aruco.DetectorParameters_create()

	    try:
	        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
	    except CvBridgeError, err:
	        rospy.logerr(err)
	        return

	    #lists of ids and the corners beloning to each id
	    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
	    #print(ids)
	 
	    cv_image = aruco.drawDetectedMarkers(cv_image, corners)
	    new_size = cv_image.shape
	    new_size = new_size[1]*3, new_size[0]*3
	    img = cv2.resize(cv_image, new_size)
	    #print img.shape
	    #print(rejectedImgPoints)
	    # Display the resulting frame
	    cv2.imshow('frame', img)
	    cv2.waitKey(3)


def main():
	rospy.init_node("pick_and_place", anonymous = True)
	limb = 'right'
	hover_distance = 0.3 # meters
	tip_name = 'right_hand'
	tip_name_camera = 'right_hand_camera'
	camera_name = 'right_hand_camera'
	products = ["ketchup","mayonaise","barbeque","salad creme"]
	product = products[1]
	cop = check_object_position(limb,hover_distance,tip_name_camera,camera_name)
	pnp = PickAndPlace(limb, hover_distance,tip_name,camera_name)

	
	# An orientation for gripper fingers to be overhead and parallel to the obj
	overhead_orientation = Quaternion(
							 x=-0.00142460053167,
							 y=0.999994209902,
							 z=-0.00177030764765,
							 w=0.00253311793936)
	overhead_orientation_camera = Quaternion(
									x = 0.0,
									y = 1.0,
									z = 0.0,
									w = 0.0)
	
	block_poses = list()
	block_poses.append(Pose(
		position=Point(x=0.45, y=-0.453, z=0.245),
		orientation=overhead_orientation))
	# Feel free to add additional desired poses for the object.
	# Each additional pose will get its own pick and place.
	block_poses.append(Pose(
		position=Point(x=0.60, y=0.0, z=0.225),
		orientation=overhead_orientation))
	camera_pose = []
	camera_pose,append(Pose(
		position=Point(x=0.45, y=-0.453, z=0.3),
		orientation=overhead_orientation_camera))
	camera_pose.append(Pose(
		position=Point(x=0.45, y=-0.553, z=0.3),
		orientation=overhead_orientation_camera))
	camera_pose,append(Pose(
		position=Point(x=0.362, y=-0.453, z=0.3),
		orientation=overhead_orientation_camera))
	camera_pose.append(Pose(
		position=Point(x=0.362, y=-0.453, z=0.3),
		orientation=overhead_orientation_camera))

	print (block_poses)
	print("Running. Ctrl-c to quit")
	pnp.move_to_neutral()
	rospy.sleep(1.0)
	#pnp.move_to_home()
	#pnp.move_to_start(starting_joint_angles)
	idx = 0
	while not cop.check_object(camera_pose[1],product): #choose wanted position
		idx = (idx+1) % len(camera_pose)
		cop.check_object(camera_pose[idx],product, hover_position = False)
	print("\nPicking...")
	pnp.pick(block_poses[0])
	print("\nPlacing...")
	#idx = (idx+1) % len(block_poses)
	pnp.place(block_poses[1])
	return 0

if __name__ == '__main__':
	main()