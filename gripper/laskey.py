#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
http://docs.ros.org/api/control_msgs/html/action/GripperCommand.html

GripperCommand command
---
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint
"""

import argparse, cv2, math, os, rospy, sys, threading, time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

import tf
import tf2_ros
import tf2_geometry_msgs
import IPython

import hsrb_interface
from hsrb_interface import geometry
from geometry_msgs.msg import PoseStamped, Point, WrenchStamped



class Gripper(object):

    def __init__(self):
        self.g_cancel   = '/gripper_controller/gripper_action/cancel'
        self.g_feedback = '/gripper_controller/gripper_action/feedback'
        self.g_goal     = '/gripper_controller/gripper_action/goal'
        self.g_result   = '/gripper_controller/gripper_action/result'
        self.g_status   = '/gripper_controller/gripper_action/status'

        self._bridge = CvBridge()
        self._input_image = None
        self._input_state = rospy.Subscriber(topic_name, JointState, self._state_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, WrenchStamped, timeout=5.0)

    def _state_cb(self, data):
        try:
            self._input_state = data
        except:
            rospy.logerr('could not read joint positions')

    def read_data(self):
        return self._input_state



if __name__=='__main__':
    rospy.init_node('main', anonymous=True)
    gripper = Gripper()
    print("\nNow sleeping (you may need to click START in the gazebo simulator) ...")
    rospy.sleep(2.0)

    idx = 0
    while not rospy.is_shutdown():
        print("\n{}".format(gripper.read_data()))
        print(idx)
        idx += 1
