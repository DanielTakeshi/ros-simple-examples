""" Testing out arm code

Test out moving specific joint angles, moving to poses (not working so far) and
the joint angle reader.
"""
from arm import Arm
from arm_joints import ArmJoints
from torso import Torso
import math, rospy, sys
from geometry_msgs.msg import PoseStamped
from reader import JointStateReader
DEGS_TO_RADS = math.pi / 180

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def test_shoulders(arm, torso):
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


def test_poses(arm, torso):
    pose1 = PoseStamped()
    pose1.header.frame_id = 'gripper_link'
    pose1.pose.position.x = 1.0
    pose1.pose.position.y = 0.0
    pose1.pose.position.z = 0.0
    pose1.pose.orientation.w = 1

    ##rospy.loginfo("calling IK with pose1:\n{}".format(pose1))
    ##joints = arm.compute_ik(pose_stamped=pose1)
    ##rospy.loginfo("joints are:\n{}".format(joints))
    rospy.loginfo("Now calling move_to_pose...")
    arm.move_to_pose(pose_stamped=pose1)
    rospy.loginfo("Finished moving.")


def test_reader(arm, reader):
    names = ArmJoints.names()
    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{:.4f}'.format(k, v)
    print("")

    # Move and then read the joints again to be clear
    pose = [0, 0, 0, 0, 0, 0, 0]
    pose[1] = DEGS_TO_RADS * -70
    arm.move_to_joints(ArmJoints.from_list(pose))

    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{:.4f}'.format(k, v)
    print("")


if __name__ == "__main__":
    rospy.init_node('arm_demo')
    wait_for_time()

    # Set things up
    torso = Torso()
    torso.set_height(Torso.MAX_HEIGHT)
    arm = Arm()
    reader = JointStateReader()
    rospy.sleep(0.5)
    rospy.loginfo("created torso, arm, and reader")

    #test_shoulders(arm, torso)
    #test_poses(arm, torso)
    test_reader(arm, reader)
