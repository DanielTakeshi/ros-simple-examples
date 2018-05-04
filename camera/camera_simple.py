import argparse, cv2, os, rospy, sys, threading, time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


class SimpleData(object):

    def __init__(self):
        self._msg = None

    def __call__(self, data):
        self._msg = data

    def get_data(self):
        return self._msg


def main(args, topics, data_collectors):
    bridge = CvBridge()
    rospy.init_node('main', anonymous=True)

    for idx,item in enumerate(topics):
        rospy.Subscriber(name=item[0],
                         data_class=item[1],
                         callback=data_collectors[idx], 
                         queue_size=1)
    print("\nNow sleeping (you may need to click START in the gazebo simulator) ...")
    rospy.sleep(2.0)

    while not rospy.is_shutdown():
        data = [data_collectors[i].get_data() for i in range(len(data_collectors))]

        # Adjust the index according to what you want to show/save
        if args.robot == 'fetch':
            imgraw = data[4]
        elif args.robot == 'hsr':
            imgraw = data[6]

        # desired_encoding = {'bgr8','passthrough'} for rgb & depth respectively
        img = bridge.imgmsg_to_cv2(imgraw, desired_encoding="bgr8")
        name = 'test.png'
        cv2.imshow(name, img)
        cv2.imwrite(name, img)
        cv2.waitKey(3)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str)
    parser.add_argument('--imdir', type=str, default='images/')
    args = parser.parse_args()
    args.robot = args.robot.lower()
    assert args.robot in ['fetch','hsr']

    if args.robot == 'fetch':
        # Only the first and last here are not None. :(
        topics = [
            ('head_camera/depth_registered/points', PointCloud2),
            ('head_camera/depth_downsampled/points', PointCloud2),
            ('head_camera/depth/image_raw', Image),
            ('head_camera/depth/image', Image),
            ('head_camera/rgb/image_raw', Image), # saved
        ]
    elif args.robot == 'hsr':
        # All of these are not None, fortunately.
        topics = [
            ('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image), # saved
            ('/hsrb/head_rgbd_sensor/depth_registered/image_raw', Image), # saved
            ('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo),
            ('/hsrb/head_center_camera/image_raw', Image), # saved
            ('/hsrb/head_l_stereo_camera/image_rect_color', Image), # saved
            ('/hsrb/head_r_stereo_camera/image_rect_color', Image), # saved
            ('/hsrb/hand_camera/image_raw', Image),
        ]
    data_collectors = [SimpleData() for _ in range(len(topics))]

    print("\nHere are our topics:")
    for topic in topics:
        print(topic)

    main(args, topics, data_collectors)
