import hsrb_interface
import numpy as np
import time, math
np.set_printoptions(suppress=True)


class RobotInterface(object):

    def __init__(self):
        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')
        # It seems like start_pose is actually a copy and doesn't get updated
        self.start_pose = self.omni_base.pose

    def body_start_pose(self):
        self.whole_body.move_to_go()

    def head_start_pose(self):
        self.whole_body.move_to_joint_positions({'head_pan_joint': 1.5})
        self.whole_body.move_to_joint_positions({'head_tilt_joint':-0.8})

    def position_start_pose(self, offsets=None):
        p = np.copy(self.start_pose)
        if offsets:
            p += np.array(offsets)
        self.omni_base.go(p[0],p[1],p[2],300,relative=False)


def test(x, y, yaw, t):
    print("\ncalling omni_base.go_abs({}, {}, {}, {}):".format(x,y,yaw,t))
    robot.omni_base.go_abs(x, y, yaw, t) 
    time.sleep(2)
    print("updated robot.start_pose: {}".format(robot.start_pose))
    print("updated omni_base.pose: {}".format(robot.omni_base.pose))


if __name__ == "__main__":
    """Note, I ran with:

    roslaunch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch rviz:=False

    and the robot starts at x,y=0,0 and yaw angle 0, but for some other words,
    the robot's `omni_base.pose` differs, so watch out. Also, you can't do this
    on the empty HSR world as that doesn't have a map installed.
    """
    robot = RobotInterface()
    robot.body_start_pose()
    print("\nAt beginning, finished whole_body.move_to_go()")
    print("  starting robot.start_pose: {}".format(robot.start_pose))
    print("  starting omni_base.pose: {}".format(robot.omni_base.pose))

    time.sleep(2)

    # These all work as expected. :-)
    test(0, 0, 90*(math.pi/180), 300)
    test(1, 1, 90*(math.pi/180), 300)
    test(1, 1, -90*(math.pi/180), 300)
