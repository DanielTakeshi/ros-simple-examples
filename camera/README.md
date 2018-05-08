# Cameras

This code lets us extract the camera images and related properties from a
robot's sensors.

**See `RESULTS.md` for detailed observations on the code usage.**

## Quick Code Usage

For a simple example of robot-agnostic code, run:

```
python camera_simple.py --robot X
```

where `X` should be replaced with `hsr` or `fetch`, depending on your robot.
This will run some code and save some images from the robot's camera. 

In order for this to do something interesting, you need to start the Gazebo
simulator so that a ROS master exists. For the Fetch, one possibility is to run:
  
```
roslaunch fetch_gazebo playground.launch
```

in one tab, *then* run the python script in a separate tab. For the HSR, one
possibility is:

```
roslaunch hsrb_gazebo_launch hsrb_mock_home_world.launch
```

and then run the camera script.

Important Points:

- The `rospy.sleep(x)` code will hang if the Gazebo simulator is not running. In
  some cases you need to click the 'start' button (lower left corner) in the
  simulator before the robot's cameras see anything. This depends on whether the
  launch file specifies the setup to be paused or not.

- It is helpful to adjust the gazebo simulator so that the robot is seeing
  something interesting. You can move the robot to a more interesting starting
  pose.

- Unfortunately, the Fetch documentation has some outdated topics listed, and
  seems to be shorter. [I filed an older issue about this here][1] which I've
  figured out.

- When I save the depth images as `png` files and then open them, they tend to
  look almost (if not entirely) black, despite how the `imshow()` function
  actually displays some black and white. I'm not sure how to get this to match.


## Launch Files

Here's the launch file I used for the Fetch, accessed via `rosed fetch_gazebo
playground.launch`:

```
<launch>

  <env name="GAZEBO_MODEL_PATH" value="$(find fetch_gazebo)/models:$(optenv GAZEBO_MODEL_PATH)" />

  <arg name="robot" default="fetch"/>
  <arg name="debug" default="false"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>

  <!-- Start Gazebo with a blank world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="world_name" value="$(find fetch_gazebo)/worlds/test_zone.sdf"/>
  </include>

  <!-- Oh, you wanted a robot? -->
  <include file="$(find fetch_gazebo)/launch/include/$(arg robot).launch.xml" />

</launch>
```

And for the HSR, accessed via `roslaunch hsrb_gazebo_launch
hsrb_mock_home_world.launch`, I get:

```
<?xml version="1.0"?>
<launch>
  <arg name="namespace" default="/hsrb"/>
  <arg name="debug" default="false"/>
  <arg name="gui" default="true" />
  <arg name="rviz" default="true"/>
  <arg name="gazebo_visualization" default="false" />
  <arg name="use_manipulation" default="true" />
  <arg name="use_navigation" default="true" />
  <arg name="use_perception" default="true" />
  <arg name="use_task" default="true" />
  <arg name="use_teleop" default="true" />
  <arg name="use_web" default="true" />
  <arg name="use_laser_odom" default="true" />
  <arg name="paused" default="true" />
  <arg name="fast_physics" default="false" />

  <arg unless="$(arg fast_physics)" name="world_suffix" value=""/>
  <arg     if="$(arg fast_physics)" name="world_suffix" value="_fast"/>

  <include file="$(find hsrb_gazebo_launch)/launch/include/hsrb_gazebo_common.xml" pass_all_args="true" >
    <arg name="map" default="$(find tmc_potential_maps)/maps/mock_house/map.yaml" />
    <arg name="world_name" value="$(find tmc_gazebo_worlds)/worlds/mock_home$(arg world_suffix).world"/>
    <arg name="robot_pos" value="-x 0 -y 0 -z 0 -Y 0" />
  </include>

  <group ns="/laser_2d_localizer">
    <param name="init_x" value="-0.4"/>
    <param name="init_y" value="0.6"/>
    <param name="init_theta_deg" value="180.0"/>
  </group>
</launch>
```

[1]:https://github.com/fetchrobotics/fetch_ros/issues/74
