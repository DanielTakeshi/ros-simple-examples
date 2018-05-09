""" Testing out arm code """
from arm import Arm
from arm_joints import ArmJoints
from torso import Torso
import math, rospy, sys
DEGS_TO_RADS = math.pi / 180

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == "__main__":
    rospy.init_node('arm_demo')
    wait_for_time()
    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    torso = Torso()
    # Faster if it's already set up
    #torso.set_height(Torso.MAX_HEIGHT)
    #print("finished setting torso height, now doing poses")

    arm = Arm()
    # Can loop through if we want
    #for vals in DISCO_POSES:
    #    arm.move_to_joints(ArmJoints.from_list(vals))

    # Shoulder pan joint
    # Could be useful to see the individual joints. Note that using
    # `arm.move_to_joints()` doesn't require MoveIt. If we adjust the 0-th
    # indexed joint angle, that will rotate the arm like if we were rotating our
    # arm going across at eye level, with the hand, wrist, elbow, etc., FIXED.
    print("shoulder pan joint")
    pose0 = [0, 0, 0, 0, 0, 0, 0]
    pose0[0] = DEGS_TO_RADS * -92
    arm.move_to_joints(ArmJoints.from_list(pose0))
    pose0[0] = DEGS_TO_RADS * 92
    arm.move_to_joints(ArmJoints.from_list(pose0))


    # Shoulder LIFT joint, now imagine we have a straight arm (actually, it just
    # has to be fixed, not straight...) but we move it up and down. :-)
    print("shoulder LIFT joint")
    pose1 = [0, 0, 0, 0, 0, 0, 0]
    pose1[1] = DEGS_TO_RADS * -70
    arm.move_to_joints(ArmJoints.from_list(pose1))
    pose1[1] = DEGS_TO_RADS * 87 # ha, this is amusing
    arm.move_to_joints(ArmJoints.from_list(pose1))
