# Arm


## Fetch

These use an action interface with a `control_msgs/FollowJointTrajectory`
action, like the torso and head. (Well, for the head, using the pan tilt method,
not the "look at an (x,y,z) coordinate" strategy.)

Unlike the torso and head code, the code (for the arm, at least) uses the MoveIt library.

We have these *action interfaces*:

- `arm_controller/follow_joint_trajectory` to control just the seven joints of
  the arm.
- `arm_with_torso_controller/follow_joint_trajectory` to control the seven
  joints of the arm plus the torso.
- `torso_controller/follow_joint_trajectory` to control just the torso.

I did the torso one elsewhere,and I don't think we need to adjust both the arm
and torso together. Let's just focus on the first action interface.


## HSR
