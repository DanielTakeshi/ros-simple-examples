# Torso

## Fetch

It's similar to the gripper code, and uses an action server (though it differs
in that it needs to follow a trajectory and doesn't use the gripper command).

Run `python torso.py` to see how tall the Fetch can be :).  

This uses `control_msgs/FollowJointTrajectory` Action types, but for the single
torso movement, it's pretty easy as all we need is to specify the height. Other
trajectories will be more complicated. 

The `trajectory_msgs/JointTrajectory` message is characterized by these joint
names and points:

```
Header header
string[] joint_names
JointTrajectoryPoint[] points
```

[which is available here][1]. So, for this code, there is only one joint to
worry about, and it only needs single scalars for the positions. Whew.


Relevant info from Fetch:

http://docs.fetchrobotics.com/api_overview.html

And from the ROS docs more generally:

http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html



## HSR

(I don't think it has a similar functionality.)


[1]:http://docs.ros.org/api/trajectory_msgs/html/index-msg.html
