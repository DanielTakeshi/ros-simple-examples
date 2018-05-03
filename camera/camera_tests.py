"""
Should process robot camera images. See README for usage instructions.

See: `http://wiki.ros.org/cv_bridge` for CvBridge info. Essentially, it converts
between ROS image messages and OpenCV image messages.

> rospy.init_node(NAME, anonymous=True), is very important as it tells rospy the
> name of your node -- until rospy has this information, it cannot start
> communicating with the ROS Master. The name must be a base name, i.e. it
> cannot contain any slashes "/".  `anonymous=True` ensures that your node has a
> unique name by adding random numbers to the end of NAME.

See: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

> One of the first calls you will likely execute in a rospy program is the call
> to rospy.init_node(), which initializes the ROS node for the process. You can
> only have one node in a rospy process, so you can only call rospy.init_node()
> once.

The `callback` argument for Subscriber: 

> callback (fn(msg, cb_args)) --- function to call (fn(data)) when data is
> received. If callback_args is set, the function must accept the callback_args
> as a second argument, i.e. fn(data, callback_args). NOTE: Additional callbacks
> can be added using add_callback().

Another issue, if you use the '/head_camera/depth/image_raw' (or even just the
'image' instead of 'image_raw') topic, we get:

daniel@daniel-ubuntu-mac:~/ros-simple-examples/camera$ python camera.py 
Traceback (most recent call last):
  File "camera.py", line 98, in <module>
    main(args)
  File "camera.py", line 81, in main
    img = bridge.imgmsg_to_cv2(HIC.get_im(), "bgr8")
  File "/opt/ros/indigo/lib/python2.7/dist-packages/cv_bridge/core.py", line 163, in imgmsg_to_cv2
    dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
AttributeError: 'NoneType' object has no attribute 'encoding'

which is confusing ...
"""
import argparse, cv2, os, rospy, sys, threading
import numpy as np
from sensor_msgs.msg import Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


class HeadImage(object):
    """ Wrapper for convenience. """

    def __init__(self):
        """
        event that will block until the info is received.
        attribute for storing the rx'd message.
        """
        self._event = threading.Event()
        self._msg = None

    def __call__(self, headImage):
        """ Uses __call__ so the object itself acts as the callback. 
        
        This means: do `HIC = HeadImage()` and then HIC(headImage=img) *is* the
        callback function. First argument (not `self`) assumed to be the data
        (from a `msg` thing). Save the data, trigger the event, `.set()` will
        stop the thread. 
        """
        self._msg = headImage
        self._event.set()

    def get_im(self, timeout=None):
        """
        Blocks until the data is rx'd with optional timeout.
        Returns the received message.
        """
        # self._event.wait(timeout)
        return self._msg


# Could use this as callback and ask for IMAGE later, but not recommended
IMAGE = None
def callback(data):
    global IMAGE
    IMAGE = data


# Simpler example
class SimpleData(object):

    def __init__(self):
        self._msg = None

    def __call__(self, data):
        self._msg = data

    def get_data(self):
        return self._msg



def main(args):
    """ Continually queries the robot camera images and saves them. 

    Use possible names like:
        http://docs.fetchrobotics.com/api_overview.html#head-camera-interface

    Note: if you save lots of images without pausing, then a weak computer like
    my laptop will likely crash.
    """
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
    print("now sleeping...")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        data = [data_collectors[i].get_data() for i in range(len(data_collectors))]
        for item in data:
            print(type(item))
        # Not directly usable (I think).
        # type(depth.data), type(imgraw.data), both <'str'>?
        # Both have (height,width) of (480,640)
        depth = data[0]
        imgraw = data[4]

        img = bridge.imgmsg_to_cv2(imgraw, "bgr8")
        name = 'test.png'
        cv2.imshow(name, img)
        cv2.imwrite(name, img)
        cv2.waitKey(3)
        print("now sleeping...\n")
        rospy.sleep(3)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='fetch')
    parser.add_argument('--imdir', type=str, default='images/')
    args = parser.parse_args()
    r_name = args.robot.lower()
    assert r_name in ['fetch', 'hsr']

    main(args)
