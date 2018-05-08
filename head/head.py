#!/usr/bin/env python
""" From Justin Huang """
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import math
import rospy

PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'
LOOK_AT_ACTION_NAME = 'head_controller/point_head'
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
PAN_TILT_TIME = 2.5


class Head(object):
    """Head controls the Fetch's head.
    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi / 2
    MAX_PAN = math.pi / 2
    MIN_TILT = -math.pi / 2
    MAX_TILT = math.pi / 4

    def __init__(self):
        self.traj_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.point_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        while not self.traj_client.wait_for_server(timeout=rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.logwarn('Waiting for head trajectory server...')
        while not self.point_client.wait_for_server(timeout=rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.logwarn('Waiting for head pointing server...')

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        Requires a frame ID, such as `base_link`

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z

        goal.min_duration = rospy.Duration(PAN_TILT_TIME)
        self.point_client.send_goal(goal)
        self.point_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

        This is more similar to the torso code which uses a simple action server
        and then a trajectory. Note that here we have two joints to consider.

        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        pan = min(max(pan, Head.MIN_PAN), Head.MAX_PAN)
        tilt = min(max(tilt, Head.MIN_TILT), Head.MAX_TILT)

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        goal.trajectory.points.append(point)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()


def wait_for_time():
    """ Wait for simulated time to begin. """
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == "__main__":
    rospy.init_node('head_demo')
    wait_for_time()
    rospy.sleep(2)
    head = Head()

    # Look at 
    frame_id = 'base_link'
    x, y, z = 10, 0, 0
    head.look_at(frame_id, x, y, z)

    # Pan tilt
    #pan, tilt = 0, 0
    #head.pan_tilt(pan, tilt)
