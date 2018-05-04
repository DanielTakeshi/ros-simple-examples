#!/usr/bin/python
# -*- coding: utf-8 -*-
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



class Joint_Positions(object):

    def __init__(self):
        #topic_name = '/hsrb/joint_states'
        topic_name = '/joint_states'
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
    bridge = CvBridge()
    rospy.init_node('main', anonymous=True)
    jp = Joint_Positions()
    print("\nNow sleeping (you may need to click START in the gazebo simulator) ...")
    rospy.sleep(2.0)

    idx = 0
    while not rospy.is_shutdown():
        print("\n{}".format(jp.read_data()))
        print(idx)
        idx += 1
