# Results

I ran `python camera_simple.py --robot fetch` and `python camera_simple.py
--robot hsr` to get some of the images. This is robot-agnostic code with the
exception of the topics to print out and, to some extent, the gazebo world
(though we should be using similar worlds for both robots).

## Fetch

Starting setup:

![.](images/fetch/setup_fetch.png?raw=true)

Here is `head_camera/rgb/image_raw`:

![.](images/fetch/head_camera--rgb--image_raw.png?raw=true)

Unfortunately that is the only image that I have actually been able to save!


## HSR

Starting setup:

![.](images/hsr/setup_hsr.png?raw=true)


Here is `/hsrb/head_center_camera/image_raw`, which has a view of the hand since
it is in the way:

![.](images/hsr/hsrb--head_center_camera--image_raw.png?raw=true)



Here are `/hsrb/head_l_stereo_camera/image_rect_color` and then for the right
camera. These are at higher resolutions than the others, interesting. These
correspond to the cameras in the two "eyes" of the robot. The previous one (the
`head_center_camera`) is located in between these two cameras.

![.](images/hsr/hsrb--head_l_stereo_camera--image_rect_color.png?raw=true)
![.](images/hsr/hsrb--head_r_stereo_camera--image_rect_color.png?raw=true)



Now we go to the `head_rgbd_sensor` for the rgb and depth images. Here is
`/hsrb/head_rgbd_sensor/rgb/image_rect_color` (i.e., the rgb), and then 
`/hsrb/head_rgbd_sensor/depth_registered/image_raw`, it is a bit
hard to see depending on how we save the image, supposed to give us some depth
information. These cameras are located at the top of the robot head, above the
stereo cameras.

![.](images/hsr/hsrb--head_rgbd_sensor--rgb--image_rect_color.png?raw=true)
![.](images/hsr/hsrb--head_rgbd_sensor--depth_registered--image_raw.png?raw=true)

I did not record images but there are *also* topics for:

- `/hsrb/head_rgbd_sensor/rgb/image_raw`
- `/hsrb/head_rgbd_sensor/depth_registered/image`
- `/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw`
- `/hsrb/head_rgbd_sensor/depth_registered/rectified_points`
- Plus `CameraInfo` topics for all the image topics ...

The docs claim the difference between `image_raw` and `image_rect_color` (for
the RGB stuff, not the depth) is "RGB image" for the former and "RGB image with
distortion was rectified" for the latter.




Finally, here is `/hsrb/hand_camera/image_raw`, so for the *hand*, not the head,
so this is also a lot different from the rest. See the documentation for the
precise camera location on the robot, it makes sense based on this image.

![.](images/hsr/hsrb--hand_camera--image_raw.png?raw=true)



See: [the camera topics docs][1] and [HSR design docs][2] for details.


[1]:https://docs.hsr.io/manual_en/reference/ros_interface.html?highlight=head_center_camera#id27
[2]:https://docs.hsr.io/manual_en/overview/parts_names.html
