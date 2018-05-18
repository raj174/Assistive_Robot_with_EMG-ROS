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

import numpy as np

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface
from sensor_msgs.msg import Image

markerLength = 0.035
camera_matrix = np.asmatrix([[613.299988, 0.0, 354.949005],[0.0, 0.0, 612.106018],[214.380005, 0.0, 1.0]])
dist_coeffs = np.asarray([-0.439, 0.263, 0.001, 0.0, -0.113])

def nothing():
    pass

def move_distance(corners):
    image_centre_point_y = 480/2
    image_centre_point_x = 752/2
    centre_x, centre_y, scale = get_centre_point(corners[0][0])
    distance_x = (image_centre_point_x-centre_x) * scale
    distance_y = (centre_y-image_centre_point_y) * scale
    return [distance_x,distance_y]

def average(start_point_one, length_one, start_point_two, length_two):
    midpoint_one = start_point_one + (length_one / 2)
    midpoint_two = start_point_two + (length_two / 2)
    median = (midpoint_one + midpoint_two) / 2
    return int(round(median))


def get_centre_point(corners):
    length = [0.0,0.0,0.0,0.0]
    total_length = 0
    #length = np.sqrt(np.square(corners[1][0]-corners[0][0]) + np.square(corners[1][1]-corners[0][1]))
    for i in range(0,len(corners)):
        if i<3 :
            length[i] = np.sqrt(np.square(corners[i+1][0]-corners[i][0]) + np.square(corners[i+1][1]-corners[i][1]))
        else:
            length[i] = np.sqrt(np.square(corners[0][0]-corners[i][0]) + np.square(corners[0][1]-corners[i][1]))
    print length
    
    total_length = sum(length)
    average_length = total_length/len(length)
    scale = markerLength /(average_length)
    
    x_centre_point = average(corners[0][0], length[0], corners[3][0], length[2])
    y_centre_point = average(corners[1][1], length[1], corners[0][1], length[3])
    return [x_centre_point,y_centre_point, scale]

def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    #image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
    image_msg = []
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    #cv_image = cv_image[162:,381:]
    #cv2.namedWindow('image')
    #cv2.createTrackbar('threshold','image',0,255,nothing)
    #cv2.createTrackbar('threshold_int','image',0,255,nothing)
    #thresh, cv_image = cv2.threshold(cv_image,cv2.getTrackbarPos('threshold_int','image'),cv2.getTrackbarPos('threshold','image'),cv2.THRESH_BINARY)
    #light = intera_interface.Lights()
    #print (light.list_all_lights())

    #'head_red_light', 'right_hand_blue_light', 'head_blue_light', 'right_hand_green_light', 'head_green_light', 'right_hand_red_light'

    #print (light.get_light_state("right_hand_blue_light"))
    # light.set_light_state("head_blue_light",False)
    # light.set_light_state("head_green_light",False)
    # light.set_light_state("head_red_light",False)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    #print(parameters)
 
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
    #print ids
    #print type(corners[0][0][0][0])  
    #cv2.imwrite(filename = "chess.jpg", img = cv_image)
    if ids != None: # if aruco marker detected
        distance_x, distance_y, = move_distance(corners)
        print distance_x, distance_y   
    #     rvec, tvec, object_ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker
    #     imgWithAruco = aruco.drawDetectedMarkers(cv_image, corners,ids, (0,255,0))
    #     imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 20)
    #     #cv2.imshow('frame2',imgWithAruco)
    #     print rvec
    #     print tvec
    # else:
    #     imgWithAruco = aruco.drawDetectedMarkers(cv_image, corners)
 
    imgWithAruco = aruco.drawDetectedMarkers(cv_image, corners)
    new_size = imgWithAruco.shape
    new_size = new_size[1]*3, new_size[0]*3
    img = cv2.resize(cv_image, new_size)
    #print img.shape
    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame', imgWithAruco)
    #cv2.imshow('frame', img)
    cv2.waitKey(3)

def main():
    """Camera Display Example

    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    """
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

    #print cameras.get_gain(camera_name)
    #print cameras.get_exposure(camera_name)
    #cv2.namedWindow('image')
    # cv2.createTrackbar('threshold_int','image',0,255,nothing)
    # cv2.createTrackbar('threshold','image',1,100,nothing)
    #with camera and light exposure 0.2 and light on
    cameras.set_gain(camera_name, 5)
    cameras.set_exposure(camera_name, 20)


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
