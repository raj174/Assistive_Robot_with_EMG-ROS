# !/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import intera_interface
import rospy
from sensor_msgs.msg import Image

rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/head_display',Image)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)

sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
rospy.spin()