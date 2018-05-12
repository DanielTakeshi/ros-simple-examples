""" Testing the base for how to move to a specific (x,y) point. """
import actionlib
import copy
import geometry_msgs.msg
import math
import nav_msgs.msg
import numpy as np
import rospy
import tf.transformations as tft
import time

def wait_for_time():
    """Wait for simulated time to begin. """
    while rospy.Time().now().to_sec() == 0:
        pass



if __name__ == "__main__":
    rospy.init_node("base_demo")
    base = Base()
    time.sleep(2)

    # TODO: run unit test here, figure out a better way to do this

    # Send the robot to (x,y) at some angle. This is ABSOLUTE motion so targ_x
    # and targ_y should be wrt the map's origin (and not the robot's current
    # origin). Some basic trig, we first rotate, then go forward. At the end, we
    # rotate for the targ_z_angle.
    if False:
        targ_x = 3.0
        targ_y = 2.0
        targ_z_angle = -45.0

        # Get distances and compute angles. All x and y here are absolute values.
        start = copy.deepcopy(base.odom.position)
        dist = np.sqrt( (start.x-targ_x)**2 + (start.y-targ_y)**2 )
        rel_x = targ_x - start.x
        rel_y = targ_y - start.y
        assert -1 <= rel_x / dist <= 1
        assert -1 <= rel_y / dist <= 1
        first_angle_v1 = np.arccos(rel_x / dist)
        first_angle_v2 = np.arcsin(rel_y / dist)
        # After we've gone forward we need to undo the effect of our first turn.
        targ_z = targ_z_angle - (first_angle_v1*(180/math.pi))

        # Note that the output of np.arccos, np.arcsin are in radians.
        print("rel_x, rel_y: {} and {}".format(rel_x, rel_y))
        print("first turn at angle {} (or {})".format(first_angle_v1, first_angle_v2))
        print("in degrees, {} (or {})".format(
                first_angle_v1 * (180/math.pi), first_angle_v2 * (180/math.pi))
        )
        print("targ_z in degrees (including undoing first turn): {}".format(targ_z))

        # Finally, do desired movement.
        base.turn(first_angle_v1)
        base.go_forward(distance=dist, speed=0.2)
        base.turn(targ_z * (math.pi / 180))
