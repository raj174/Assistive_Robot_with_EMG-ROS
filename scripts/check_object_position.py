#!/usr/bin/env python

import time
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
from std_msgs.msg import String

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

class KitchenObject(object):
	def __init__(self, name='ketchup', product_id=0, x=0.3, y=0.3, z=0.7, hover_offset=0.3):
		self.name = name
		self.orientation = Quaternion(x = 0.0, y = 1.0, z = 0.0, w = 0.0)
		self.orientation_drop = Quaternion(x = 0.7, y = -0.7, z = 0.0, w = 0.0)
		self.position = None
		self.pose = None
		self.product_id = product_id
		self.hover_pose = None
		self.hover_position = None
		self.hover_offset = hover_offset

		self.set_pose(x,y,z)

	def set_pose(self, x, y, z, orientation_get = True):
		if orientation_get:
			orientation = self.orientation
		else:
			orientation = self.orientation_drop
		self.position = Point(x=x, y=y, z=z)
		self.pose = Pose(position=self.position, orientation = orientation)

		self.hover_position = copy.deepcopy(self.position)
		self.hover_position.z = self.position.z + self.hover_offset
		self.hover_pose = Pose(position=self.hover_position, orientation= orientation)

	def get_pose(self):
		return self.pose

	def get_position(self):
		return self.position

	def get_orientation(self):
		return self.orientation

	def get_name(self):
		return self.name

	def get_hover_pose(self):
		return self.hover_pose

	def printf(self):
		rospy.loginfo("hover_pose: {hover_pose}\npose: {pose}".format(hover_pose=self.hover_pose, pose=self.pose))


class CheckObjectPosition(object):
	def __init__(self, limb="right", hover_distance = 0.30, tip_name="right_hand_camera", camera_name = "right_hand_camera"):
		self._limb_name = limb # string
		self._tip_name = tip_name # string
		self._hover_distance = hover_distance # in meters
		self._camera_name = camera_name
		self._limb = intera_interface.Limb(limb)
		self._gripper = intera_interface.Gripper()
		self._rp = intera_interface.RobotParams()
		self._light = intera_interface.Lights()
		self._cameras = intera_interface.Cameras()
		self._camera_orientation = Quaternion(
									x = 0.0,
									y = 1.0,
									z = 0.0,
									w = 0.0)
		## Create Kitchen Items
		self.kitchen_objects = {}

		self.kitchen_objects['ketchup']   = KitchenObject('ketchup'  , 0, 0.450, -0.453, 0.245)
		self.kitchen_objects['mayonaise'] = KitchenObject('mayonaise', 1, 0.450, -0.553, 0.245)
		self.kitchen_objects['barbecue']  = KitchenObject('barbecue' , 2, 0.362, -0.453, 0.245)
		self.kitchen_objects['salad']     = KitchenObject('salad'    , 3, 0.362, -0.553, 0.245)

		self.check_positions =  [
								(0.450, -0.453),
								(0.450, -0.553),
								(0.362, -0.453),
								(0.362, -0.553),
								]


		self.detected_object_data = {
			'frame': None,
			'obj_id': None,
			'obj_pose': None
		}

	def constrain(self, x, min_x, max_x):
		return min(max_x, max(x, min_x))

	def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

	def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

	def move_to_neutral(self):
		print("Moving the {0} arm to neutral pose...".format(self._limb_name))
		self._limb.move_to_neutral()
		self.gripper_open()

	def move_to_home(self):
		self._limb.set_joint_position_speed(0.2)
		home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': 0.0}
		self._guarded_move_to_joint_position(home_position)
		self.gripper_open()

	def move_to_pose(self, pose):
		joint_angles = self._limb.ik_request(pose, self._tip_name)
		#print pose
		self._limb.set_joint_position_speed(0.2)
		self._guarded_move_to_joint_position(joint_angles)

	def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
			#print (self._limb.endpoint_pose())
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def update_product_positions(self):
		current_id = None
		for i in range(0,len(self.kitchen_objects)):
			move_pose = Pose(position= Point(x = self.check_positions[i][0], y= self.check_positions[i][1], z = 0.245), orientation=self._camera_orientation)
			self.move_to_pose(move_pose)
			rospy.sleep(2.0)
			currrent_id = self.detected_object_data['obj_id']
			#print self.detected_object_data['obj_id']
			for j in self.kitchen_objects:
				if self.kitchen_objects[j].product_id == currrent_id:
					self.kitchen_objects[j].set_pose(self.check_positions[i][0],self.check_positions[i][1],0.245)
					current_id = None
				elif current_id == type(None):
					rospy.loginfo("Nothing found at position x: {x} y: {y}".format(x = self.check_positions[i][0],y = self.check_positions[i][1]))

	def product_position(self,product_name, position):
		pick_position = copy.deepcopy(position)
		drop_position = Point(x=self.kitchen_objects[product_name].position.x, y=self.kitchen_objects[product_name].position.y, z=self.kitchen_objects[product_name].position.z)
		if pick_position.y  == 0:  #for now ok as it is a known
			self.kitchen_objects[product_name].set_pose(pick_position.x,pick_position.y,pick_position.z,False)
		else:
			self.kitchen_objects[product_name].set_pose(pick_position.x,pick_position.y,pick_position.z)
		return drop_position

	def get_product_pose(self, product_name=''):
		if not product_name in self.kitchen_objects.keys():
			return None
		else:
			return self.kitchen_objects[product_name].get_pose()

	def get_product(self, product_name=''):
		if not product_name in self.kitchen_objects.keys():
			return None
		else:
			return self.kitchen_objects[product_name]


	def check_object(self, product_name='', hover_position=True, timeout=5):
		if rospy.is_shutdown(): 
			return
		product = self.get_product(product_name)
		if not product:
			rospy.logerr("Could not find object with name {name}.".format(name=product_name))
			return None
		if hover_position == True:
			#print product.get_hover_pose()
			self.move_to_pose(product.get_hover_pose())
		
		self.move_to_pose(product.get_pose())

		self.check_object_with_camera()
		## wait for camera to initialize
		rospy.sleep(1.0)
		#print len(self.kitchen_objects)-1

		start_time = time.time()
		while True:
			if type(self.detected_object_data['obj_id']) != type(None):
				break
			if time.time() - start_time >= timeout:
				break
			if rospy.is_shutdown():
				break
		if type(self.detected_object_data['obj_id']) == type(None):
			rospy.logerr("Did not detect any product at position.".format(product=product_name))
			if hover_position == True:
				self.update_product_positions()
		elif self.detected_object_data['obj_id'] == product.product_id:
			rospy.loginfo("Detctected {product} at position. Product ID {id}".format(product = product_name, id=product.product_id))
			self.move_to_pose(product.get_hover_pose())
			return product.get_pose()
		elif self.detected_object_data['obj_id'] >= 0 and self.detected_object_data['obj_id'] <= (len(self.kitchen_objects)-1):
			if hover_position == True:
				self.update_product_positions()
		return False

	def check_object_with_camera(self):
	    valid_cameras = self._rp.get_camera_names()
	    if not valid_cameras:
	        self._rp.log_message(("Cannot detect any camera_config"
	            " parameters on this robot. Exiting."), "ERROR")
	        return

	    if not self._cameras.verify_camera_exists(self._camera_name):
	        rospy.logerr("Could not detect the specified camera, exiting.")
	        return
	    self._cameras.start_streaming(self._camera_name)
	    self._cameras.set_callback(self._camera_name, self.show_image_callback, rectify_image=False, callback_args=None)

	def show_image_callback(self,img_data):
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
	    # print(ids)
	    ## TODO #2: RAJ look into the section below
	    ## updating detected object data
	    
	    self.detected_object_data['frame'] = None
	    self.detected_object_data['obj_pose'] = None

	    if (type(ids) == type(None)) or (type(ids[0]) == type(None)):
	    	self.detected_object_data['obj_id'] = None
	    else:
	    	self.detected_object_data['obj_id'] = ids[0][0]

def main():
	rospy.init_node("Check_object_position", anonymous = True)
	limb = 'right'
	hover_distance = 0.3 # meters
	tip_name = 'right_hand_camera'
	product_name = ["ketchup","mayonaise","barbeque","salad"]
	cop = check_object_position(limb,hover_distance)	
	cop.move_to_home()
	rospy.sleep(1.0)
	#pnp.move_to_start(starting_joint_angles)
	#idx = 0
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
			product_pose_repeat = cop.check_object(product_to_pick,False)
			if not product_pose_repeat:
				pass
			
			# if not cop.check_object(product[1]): #choose wanted position
			# # idx = (idx+1) % len(camera_pose)
			# # cop.check_object(camera_pose[idx],product, hover_position = False)
			# pass

if __name__ == '__main__':
	main()
