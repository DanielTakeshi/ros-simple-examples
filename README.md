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
- `head`: Adjust Fetch's head, mostly understood.
- `torso`: Adjust Fetch's torso, mostly understood.

There are also topics for the robots listed in `topics`. However, these might be
dependent on what kind of launch file we used for the simulator?


## TODOs and Questions

- Figure out why we group ros topics into those with cancel, feedback, goal,
  result, and status topics. I think these form an "action interface," but I'd
  like to know deeply.

  Update: [somewhat answered in the HSR docs][1].

- Figure out how movement of the Fetch's base and arm work.

- Use [HSR code examples][1] in the case of when Fetch's motion requires MoveIt,
  for compatibility with our existing HSR code base.


[1]:https://docs.hsr.io/manual_en/development/ros_interface.html
