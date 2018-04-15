#!/usr/bin/env python

import sys
import rospy
import serial
import time
import intera_interface

from std_msgs.msg import String

import numpy as np
import glob
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image



def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
    image_msg = []
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    # termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*7,3), np.float32)
	objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	images = glob.glob('*.jpg')
	for fname in images:
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
		# If found, add object points, image points (after refining them)
		if ret == True:
			objpoints.append(objp)
			corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)
			# Draw and display the corners
			cv2.drawChessboardCorners(img, (7,6), corners2, ret)
			cv2.imshow('img', img)
			cv2.waitKey(500)
			cv2.destroyAllWindows()


def main():

    camera_name = "right_hand_camera"

    use_canny_edge = 1
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    
    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(camera_name):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(camera_name))
    cameras.start_streaming(camera_name)
    cameras.set_cognex_strobe(False)
    cameras.set_callback(camera_name, show_image_callback, rectify_image=False, callback_args=None)

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    main()

