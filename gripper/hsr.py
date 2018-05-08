#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher(
    '/hsrb/gripper_controller/command',
    trajectory_msgs.msg.JointTrajectory, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'gripper_controller' and c.state == 'running':
            running = True

# fill ROS message
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["hand_motor_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [1.0]
p.velocities = [0.5]
p.effort = [1.0]
p.time_from_start = rospy.Time(2)
traj.points = [p]

# publish ROS message
pub.publish(traj)
