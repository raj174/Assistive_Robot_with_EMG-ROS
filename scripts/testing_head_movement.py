#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
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
import random

import rospy
import time
import math

import intera_interface

from intera_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = intera_interface.Head()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def __constrain(self, x, min_x, max_x):
        return min(max_x, max(x, min_x))

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self._done:
            self.set_neutral()

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)

    def run(self):
        self.set_neutral()
        # """
        # Performs the wobbling
        # """
        # command_rate = rospy.Rate(1)
        # control_rate = rospy.Rate(100)
        # start = rospy.get_time()
        # while not rospy.is_shutdown() and (rospy.get_time() - start < 10.0):
        #     angle = self.__constrain(-1.9, -2.0, 0.95)
        #     self._head.set_pan(angle, speed=0.3, timeout=0)

        #     print ("position 1")
        #     time.sleep(10);

        #     angle = self.__constrain(0.9, -2.0, 0.95)
        #     self._head.set_pan(angle, speed=0.3, timeout=0)

        #     print ("position 2")
        #     time.sleep(10);

        #     command_rate.sleep()

        self._done = True
        rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Head Example: Wobbler

    Nods the head and pans side-to-side towards random angles.
    Demonstrates the use of the intera_interface.Head class.
    """
    rospy.init_node("rsdk_head_runr")

    my_wobbler = Wobbler()
    rospy.on_shutdown(my_wobbler.clean_shutdown)
    print("Wobbling... ")
    my_wobbler.run()
    print("Done.")

if __name__ == '__main__':
    main()

    #print (dir(intera_interface))



## intera_interface.HEAD_PAN_ANGLE_TOLERANCE