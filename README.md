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

- `base`: Adjust Fetch's base.
- `camera`: Processes the HSR and Fetch camera images.
- `gripper`: Adjust Fetch's gripper.
- `head`: Adjust Fetch's head.
- `torso`: Adjust Fetch's torso.

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


## Robots

Here is some additional information about the robots we have.


### Fetch


See the docs for details on the joint angles. For the Fetch, with the Gazebos
simulator running in the background, I get this from echo-ing the
`/joint_states` ros topic:

```
daniel@daniel-ubuntu-mac:~/ros-simple-examples/states$ rostopic echo -n 1 /joint_states 
header: 
  seq: 2502
  stamp: 
    secs: 25
    nsecs:  74000000
  frame_id: ''
name: ['l_wheel_joint', 'r_wheel_joint', 'torso_lift_joint', 'bellows_joint', 'head_pan_joint', 'head_tilt_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
position: [0.0904709937685535, -0.13369708805940572, -2.1922572375654923e-08, 0.00663651940159223, 0.001038524967845511, 0.0023006007164765307, 1.320004990825269, 1.3999840248157627, -0.19984345342595944, 1.71997015957281, 3.2424016920273857e-06, 1.6600004833253479, -1.3478056999360888e-06, 0.04986168604674154, 0.04989492635705342]
velocity: [1.0627606486403384e-05, 0.0013527573505002295, 5.7977575865534834e-05, -4.450639518941297e-05, -0.0013211152971605432, 0.001643271318197307, -0.0003535670947248451, -0.00019485233294148, 0.00013064563488349577, -0.0001666420730967442, -0.0003557676420180161, 0.00014203296699294974, 0.00021448259548277236, -0.008826693605311384, -0.0088349170683755]
effort: [0.0, 0.0, 0.0, -1.6580199308398567, -0.0015118692770609578, -0.07394783544888785, -0.03427439973805562, 11.822129377258655, -0.7697657524232971, 18.351679878283072, 0.2756228700101902, -0.3805430747091004, -0.02469191997828887, 10.033240310311884, 9.966759689688116]
---
```

In easier-to-read form, the joint names are:

```
// Wheel, torso
'l_wheel_joint', 
'r_wheel_joint', 
'torso_lift_joint', 
'bellows_joint',  // actually I can't find this in docs!!

// For the head
'head_pan_joint', 
'head_tilt_joint', 

// These are the 7DOF arm joints.
'shoulder_pan_joint',
'shoulder_lift_joint', 
'upperarm_roll_joint', 
'elbow_flex_joint', 
'forearm_roll_joint', 
'wrist_flex_joint', 
'wrist_roll_joint',

// Finally, the gripper
'l_gripper_finger_joint', 
'r_gripper_finger_joint',
```


### HSR

For the HSR, the joint states are:

```
daniel@daniel-ubuntu-mac:~/ros-simple-examples/states$ rostopic echo -n 1 /hsrb/joint_states 
header: 
  seq: 147
  stamp: 
    secs: 175
    nsecs: 636000000
  frame_id: ''
name: ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'base_l_drive_wheel_joint', 'base_r_drive_wheel_joint', 'base_roll_joint', 'hand_l_spring_proximal_joint', 'hand_motor_joint', 'hand_r_spring_proximal_joint', 'head_pan_joint', 'head_tilt_joint', 'wrist_flex_joint', 'wrist_roll_joint']
position: [-9.120342727531039e-05, 0.01637693306919232, -1.5700004646044103, -5.950544798150048e-05, -1.861907801536944e-05, 9.602650967721615e-07, 0.00015512160125652485, -3.5844492282777196e-05, 0.00016245653104984825, 2.974513302334003e-07, 0.0009638909841056531, -1.5731330858686743, -5.8117104977384315e-05]
velocity: [9.180129606437106e-05, 0.007555368244559862, 0.00031475762415357614, -0.0013761978010660126, 0.0014498472922710932, 0.0006561699685778806, 0.00456957118478556, 5.528347479977406e-05, 6.169668895067484e-05, 4.669170088413178e-05, -9.264681426704654e-05, -0.00013081460253357293, -9.149466419448049e-05]
effort: [0.18621821422218382, 55.08541747733215, 0.0010732245617894253, -0.039412425624447765, 0.04216265206354866, 0.17697903979233284, 0.0, 0.0, 0.0, -0.005823677186356235, -1.156852470296954, 2.8196592257856423, 0.05220894967870038]
---
```

There are some similar joint names, so that's good. Here's a sketch of a
possible grouping.

```
// Arm and wrist
'arm_flex_joint', 
'arm_lift_joint', 
'arm_roll_joint',
'wrist_flex_joint', 
'wrist_roll_joint'

// The base
'base_l_drive_wheel_joint', 
'base_r_drive_wheel_joint', 
'base_roll_joint',

// The hand (i.e., gripper)
'hand_motor_joint',
'hand_l_spring_proximal_joint',
'hand_r_spring_proximal_joint',

// The head
'head_pan_joint',
'head_tilt_joint',
```

If we consider the two hand proximal joints as part of the "arm" then I assume
we can call this 7DOF? Not entirely sure.

See the [HSR core repository][2] and files such as the `sensors.py` for
information on how to deal with sensors. That one has joint states.

[1]:https://docs.hsr.io/manual_en/development/ros_interface.html
[2]:https://github.com/BerkeleyAutomation/HSR_CORE/tree/master/core
