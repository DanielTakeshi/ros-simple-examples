"""
        Fetch:

'head_camera/depth_registered/points'       sensor_msgs/PointCloud2 

    has both 3d and color data. 
    It is published at VGA resolution (640x480) at 15Hz.

'head_camera/depth_downsampled/points'      sensor_msgs/PointCloud2 
    
    has only 3d data. It is published at QQVGA (160x120) resolution at 15Hz and is 
    intended primarily for use in navigation/moveit for obstacle avoidance.

'head_camera/depth/image_raw'               sensor_msgs/Image

    This is unit16 depth image (2D) in mm . 
    It is published at VGA resolution (640x480) at 15Hz.

'head_camera/depth/image'                   sensor_msgs/Image

    This is float depth image (2D) in m. 
    It is published at VGA resolution (640x480) at 15Hz.

'head_camera/rgb/image_raw'                 sensor_msgs/Image

    This is just the 2d color data. 
    It is published at VGA resolution (640x480) at 15Hz.


        HSR:

'/hsrb/head_rgbd_sensor/rgb/image_rect_color',          Image

'/hsrb/head_rgbd_sensor/depth_registered/image_raw',    Image

'/hsrb/head_rgbd_sensor/rgb/camera_info',               CameraInfo

'/hsrb/head_center_camera/image_raw',                   Image

'/hsrb/head_l_stereo_camera/image_rect_color',          Image

'/hsrb/head_r_stereo_camera/image_rect_color',          Image

'/hsrb/hand_camera/image_raw',                          Image

"""
