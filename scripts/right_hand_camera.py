#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
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

import argparse
import numpy as np

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface
from sensor_msgs.msg import Image


def show_image_callback(img_data, (edge_detection, window_name)):
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
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    cv_win_name = ' '.join([window_name, edge_str])
    cv2.namedWindow(cv_win_name, 0)
    # refresh the image on the screen
    image_msg.append(img_data)
    for one_msg in image_msg:
        image_pub.publish(one_msg)
        rospy.Rate(1).sleep()
    #head_display.display_image("/home/raj/sawyer_ws/src/Raj/images/maxresdefault.jpg")
    #print(frame.shape) #480x640
    light = intera_interface.Lights()
    #print (light.list_all_lights())

    #'head_red_light', 'right_hand_blue_light', 'head_blue_light', 'right_hand_green_light', 'head_green_light', 'right_hand_red_light'



    print (light.get_light_state("right_hand_blue_light"))
    light.set_light_state("head_blue_light",True)
    light.set_light_state("head_green_light",True)
    light.set_light_state("head_red_light",True)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
 
    #print(parameters)
 
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
    print(corners)
 
    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs
 
    cv_image = aruco.drawDetectedMarkers(cv_image, corners)
    new_size = cv_image.shape
    new_size = new_size[1]*3, new_size[0]*3
    img = cv2.resize(cv_image, new_size)
    print img.shape
    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame', img)
    cv2.waitKey(3)

def main():
    """Camera Display Example

    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    """
    camera_name = "right_hand_camera"

    use_canny_edge = 0
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
    cameras.set_callback(camera_name, show_image_callback, rectify_image=False, callback_args=(use_canny_edge, camera_name))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    main()
