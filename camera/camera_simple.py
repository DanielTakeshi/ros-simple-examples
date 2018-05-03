import argparse, cv2, os, rospy, sys, threading
import numpy as np
from sensor_msgs.msg import Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


class SimpleData(object):

    def __init__(self):
        self._msg = None

    def __call__(self, data):
        self._msg = data

    def get_data(self):
        return self._msg


def main(args):
    bridge = CvBridge()
    rospy.init_node('main', anonymous=True)

    topics = [
        ('head_camera/depth_registered/points', PointCloud2),
        ('head_camera/depth_downsampled/points', PointCloud2), # None :(
        ('head_camera/depth/image_raw', Image), # None :(
        ('head_camera/depth/image', Image), # None :(
        ('head_camera/rgb/image_raw', Image),
    ]
    data_collectors = [SimpleData() for _ in range(len(topics))]

    for idx,item in enumerate(topics):
        rospy.Subscriber(name=item[0],
                         data_class=item[1],
                         callback=data_collectors[idx], 
                         queue_size=1)
    rospy.sleep(2)

    while not rospy.is_shutdown():
        data = [data_collectors[i].get_data() for i in range(len(data_collectors))]
        depth, imgraw = data[0], data[4]
        print("")
        for item in data:
            print(type(item))
        img = bridge.imgmsg_to_cv2(imgraw, "bgr8")
        name = 'test.png'
        cv2.imshow(name, img)
        cv2.imwrite(name, img)
        cv2.waitKey(3)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='fetch')
    parser.add_argument('--imdir', type=str, default='images/')
    args = parser.parse_args()
    main(args)
