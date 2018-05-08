# ros-simple-examples

Some ROS code examples which hopefully are robot-agnostic. I'll test in
simulation and hopefully it's just a toggle to figure out which robot to use,
plus a list of the appropriate robot's ROS topic names.

Versions:

- Fetch: Ubuntu 14.04, Ros Indigo
- HSR: Ubuntu 14.04, Ros Kinetic (though really we should use 16.04)
- Python 2.7.6, :(

I'm sure there's a better way to manage versions, but I simply use one machine
with 14.04 and switch between Fetch and HSR code by switching their ordering in
my `.bashrc` file manually. Bleh.

Examples:

- `camera`: Processes the robot's camera images, mostly understood.
- `gripper`: Open and close the (Fetch's) gripper, mostly understood.
- `torso`: Adjust Fetch's torso, mostly understood.

There are also topics for the robots listed in `topics`. However, these might be
dependent on what kind of launch file we used for the simulator?


## TODOs and Questions

- Figure out what groupings like these rostopics represent:

  ```
  /torso_controller/follow_joint_trajectory/cancel
  /torso_controller/follow_joint_trajectory/feedback
  /torso_controller/follow_joint_trajectory/goal
  /torso_controller/follow_joint_trajectory/result
  /torso_controller/follow_joint_trajectory/status
  ```

  I think these mean that together these form an "action interface," and in this
  case one that represents the torso movement (for the Fetch). But I'd like to
  know more.


- Figure out how movement of the Fetch's base and arm work.
