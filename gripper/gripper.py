#! /usr/bin/env python
""" 
Based on code from Justin Huang at UW CSE.
https://github.com/cse481wi18/cse481wi18
"""
import actionlib
import control_msgs.msg
import rospy
import sys, time
import tmc_control_msgs.msg
# HSR uses: tmc_control_msgs.msg.GripperApplyEffortActionGoal (?)

CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
ACTION_SERVER = 'gripper_controller/gripper_action'

# Unfortunately none of these work for the HSR :-( incompatible types
#ACTION_SERVER = '/hsrb/gripper_controller/apply_force'
#ACTION_SERVER = '/hsrb/gripper_controller/follow_joint_trajectory'
#ACTION_SERVER = '/hsrb/grasp_state_request_action'
#ACTION_SERVER = '/hsrb/gripper_controller/grasp'


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35   # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        The `goal` has type:
            <class 'control_msgs.msg._GripperCommandGoal.GripperCommandGoal'>
        with a single attribute, accessed via `goal.command`, which consists of:
            position: 0.0
            max_effort: 0.0
        by default, and is of type:
            <class 'control_msgs.msg._GripperCommand.GripperCommand'>

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))


def wait_for_time():
    """Wait for simulated time to begin.

    A useful method. Note that rviz will display the ROS Time in the bottom left
    corner. For Gazebo, just click the play button if it's paused to start.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == "__main__":
    # Looks like this works for the Fetch :-)
    rospy.init_node('gripper_demo')
    wait_for_time()
    time_delay = 1
    use_delay = True

    print("Now forming the gripper")
    gripper = Gripper()
    gripper.close()
    print("gripper now closed")
    if use_delay:
        time.sleep(time_delay)

    gripper.open()
    print("gripper now open")
    if use_delay:
        time.sleep(time_delay)

    gripper.close(35)
    print("gripper now closed")
    if use_delay:
        time.sleep(time_delay)

    gripper.open()
    print("gripper now open")
    if use_delay:
        time.sleep(time_delay)

    # closes very slowly ...
    gripper.close(1)
    print("gripper now closed")
    if use_delay:
        time.sleep(time_delay)
