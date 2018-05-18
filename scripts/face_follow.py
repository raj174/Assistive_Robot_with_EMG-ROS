#!/usr/bin/env python

import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface
from sensor_msgs.msg import Image



# cap = cv2.VideoCapture(0)

# while True:
#   ret, img = cap.read()
#   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#   cv2.imshow('img1',gray)
#   faces = face_cascade.detectMultiScale(gray, 2, 5)
#   for (x,y,w,h) in faces:
#       cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)
#       roi_gray = gray[y:y+h, x:x+w]
#       roi_colour =img[y:y+h, x:x+w]
#       eyes = eye_cascade.detectMultiScale(roi_gray)
#       for (ex,ey,ew,eh) in eyes:
#           cv2.rectangle(roi_colour, (ex,ey),(ex+ew,ey+eh),(0,255,0),2)
#   cv2.imshow('img',img)
#   k = cv2.waitKey(30) & 0xff
#   if k == 27:
#       break

# cap.release()
# cv2.destroyAllWindows()
position = 0.0

def constrain(x, min_x, max_x):
    return min(max_x, max(x, min_x))

def update_position(x,w):
    global position
    img_centre_x = 1280/2
    rectangle_centre_x = x+(w/2)

    distance_x = img_centre_x - rectangle_centre_x
    print distance_x
    if distance_x < -40:
        position = constrain(position - 0.05,-1.5,0.3)
    elif distance_x > 40:
        position = constrain(position + 0.05,-1.5,0.3)



def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    global position
    head = intera_interface.Head()
    head.set_pan(position)
    face_cascade = cv2.CascadeClassifier('/home/raj/sawyer_ws/src/Raj/scripts/haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier('/home/raj/sawyer_ws/src/Raj/scripts/haarcascade_eye.xml')
    bridge = CvBridge()
    image_msg = []
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img1',cv_image)
    faces = face_cascade.detectMultiScale(gray, 2, 5)
    for (x,y,w,h) in faces:
        cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,0,0), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_colour =cv_image[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        update_position(x,w)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_colour, (ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    cv2.imshow('img',cv_image)
    k = cv2.waitKey(30) & 0xff

    #'head_red_light', 'right_hand_blue_light', 'head_blue_light', 'right_hand_green_light', 'head_green_light', 'right_hand_red_light'

    #print (light.get_light_state("right_hand_blue_light"))
    # light.set_light_state("head_blue_light",False)
    # light.set_light_state("head_green_light",False)
    # light.set_light_state("head_red_light",False)
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    # parameters =  aruco.DetectorParameters_create()
    #print(parameters)
 

 
    # imgWithAruco = aruco.drawDetectedMarkers(cv_image, corners)
    # new_size = imgWithAruco.shape
    # new_size = new_size[1]*3, new_size[0]*3
    # img = cv2.resize(cv_image, new_size)
    #print img.shape
    # Display the resulting frame
    #cv2.waitKey(3)

def main():
    camera_name = "head_camera"

    use_canny_edge = 1
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    
    print("Initializing node... ")
    rospy.init_node('follow_face', anonymous=True)
    cameras = intera_interface.Cameras()
    cameras.set_gain(camera_name, 5)
    cameras.set_exposure(camera_name, 50)
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
