#!/usr/bin/env python
""" Based on Justin Huang's code. """
import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg

ACTION_SERVER = 'torso_controller/follow_joint_trajectory'
TORSO_JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))

    def set_height(self, height):
        """Sets the torso height.
        This will always take ~5 seconds to execute.

        The `goal.trajectory`, just before we send it to client, has info:

            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs:         0
              frame_id: ''
            joint_names: ['torso_lift_joint']
            points: 
              - 
                positions: [0.4]
                velocities: []
                accelerations: []
                effort: []
                time_from_start: 
                  secs: 5
                  nsecs:         0

        where the `joint_names` was obviously adjusted by `torso_lift_joint` via
        a simple list appending, and the `positions` depends on what height we
        send it to. Here I copied the output when I sent it to height 0.4.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        height = min(height, 0.4)
        height = max(height, 0.0)
        
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(TORSO_JOINT_NAME)
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(height)
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == "__main__":
    # Similar to the gripper code
    rospy.init_node('torso_demo')
    wait_for_time()

    torso = Torso()
    torso.set_height(0.4)
    torso.set_height(0.0)
    torso.set_height(0.4)
