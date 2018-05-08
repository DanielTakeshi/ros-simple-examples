# Gripping

Based on code from Justin Huang at UW CSE. `https://github.com/cse481wi18/cse481wi18`

## Code Usage

Just run `python gripper.py` with a Fetch Gazebo setup already running. If you
switch to the simulator, you will see the gripper closing and opening. :-)

Right now it doesn't work with the HSR, because the topics don't see to match
up. There doesn't seem to be a clear analogue to the
`gripper_controller/gripper_action` that the Fetch has, and examples of error
messages are:

```
[WARN] [WallTime: 1525806234.333098] [169.827000] Could not process inbound
connection: topic types do not match:

[tmc_control_msgs/GripperApplyEffortActionGoal] vs.
[control_msgs/GripperCommandActionGoal]{'topic':
'/hsrb/gripper_controller/grasp/goal', 'tcp_nodelay': '0', 'md5sum':
'43355ef33a9f9ba5d26e50a3cc6e60bd', 'type':
'tmc_control_msgs/GripperApplyEffortActionGoal', 'callerid': '/gazebo'}
```

The `tmc_control_msgs` must be from the HSR's topics, but I can't find
information about that so it must not be as standard as `control_msgs`,
unfortunately. But it seems to be installed with HSR's code, as I see it when
running `rosmsg list`:

```
tmc_control_msgs/ExxxDriveMode
tmc_control_msgs/GripperApplyEffortAction
tmc_control_msgs/GripperApplyEffortActionFeedback
tmc_control_msgs/GripperApplyEffortActionGoal
tmc_control_msgs/GripperApplyEffortActionResult
tmc_control_msgs/GripperApplyEffortFeedback
tmc_control_msgs/GripperApplyEffortGoal
tmc_control_msgs/GripperApplyEffortResult
tmc_control_msgs/JointExxxDriveMode
tmc_control_msgs/ServoParam
tmc_control_msgs/ServoState
```

But there is nothing matching the `control_msgs/GripperCommandActionGoal` type.

I'm sure I could try and get this to work for the HSR by trying out different
things, but it's far easier to use the Python interface and that's what we
should be doing. See `laskey.py` for details.


## Documentation and Notes

See actionlib for additional details: `http://wiki.ros.org/actionlib`

From the Fetch docs:

> `gripper_controller/gripper_action` exposes a `control_msgs/GripperCommand`
> `ActionServer`. The gripper command takes in position and effort as
> parameters.  Generally, the gripper is commanded to a fully closed or fully
> opened position, so effort is used to limit the maximum effort. As the gripper
> never fully reaches the closed position, the grasp strength will be determined
> by the maximum effort.

And the docs show:

```
http://docs.ros.org/api/control_msgs/html/action/GripperCommand.html

GripperCommand command
---
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint
```

Note that there is also a GripperCommand *message*

`http://docs.ros.org/api/control_msgs/html/msg/GripperCommand.html`

which has only `position` and `max_effort`, which differs from the "action"
shown earlier, which is also named "GripperCommand".


## Basic Information

When I have a Gazebo simulator running with the Fetch, here is the relevant
information for the gripper topics:

```
daniel@daniel-ubuntu-mac:~/ros-simple-examples/grasp$ rostopic info /gripper_controller/gripper_action/cancel
Type: actionlib_msgs/GoalID

Publishers: None

Subscribers: 
 * /gazebo (http://10.105.209.175:37055/)


daniel@daniel-ubuntu-mac:~/ros-simple-examples/grasp$ rostopic info /gripper_controller/gripper_action/feedback
Type: control_msgs/GripperCommandActionFeedback

Publishers: 
 * /gazebo (http://10.105.209.175:37055/)

Subscribers: None


daniel@daniel-ubuntu-mac:~/ros-simple-examples/grasp$ rostopic info /gripper_controller/gripper_action/goal
Type: control_msgs/GripperCommandActionGoal

Publishers: None

Subscribers: 
 * /gazebo (http://10.105.209.175:37055/)


daniel@daniel-ubuntu-mac:~/ros-simple-examples/grasp$ rostopic info /gripper_controller/gripper_action/result
Type: control_msgs/GripperCommandActionResult

Publishers: 
 * /gazebo (http://10.105.209.175:37055/)

Subscribers: None


daniel@daniel-ubuntu-mac:~/ros-simple-examples/grasp$ rostopic info /gripper_controller/gripper_action/status
Type: actionlib_msgs/GoalStatusArray

Publishers: 
 * /gazebo (http://10.105.209.175:37055/)

Subscribers: None
```


## HSR Information

Update: actually maybe the HSR is easier than I thought if we just follow [their
docs carefully][1]. However, I haven't gotten it working from a simple python
script.

[1]:https://docs.hsr.io/manual_en/development/ros_controller_interface.html
