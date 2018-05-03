"""
Put some names here.

Fetch:

head_camera/depth_registered/points is a sensor_msgs/PointCloud2 which has both 3d and color data. It is published at VGA resolution (640x480) at 15Hz.

head_camera/depth_downsampled/points is a sensor_msgs/PointCloud2 which has only 3d data. It is published at QQVGA (160x120) resolution at 15Hz and is intended primarily for use in navigation/moveit for obstacle avoidance.

head_camera/depth/image_raw is a sensor_msgs/Image. This is unit16 depth image (2D) in mm . It is published at VGA resolution (640x480) at 15Hz.

head_camera/depth/image is a sensor_msgs/Image. This is float depth image (2D) in m. It is published at VGA resolution (640x480) at 15Hz.

head_camera/rgb/image_raw is a sensor_msgs/Image. This is just the 2d color data. It is published at VGA resolution (640x480) at 15Hz.
"""
