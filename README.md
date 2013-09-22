ROS OpenCV camera driver
========================
It is very easy to capture video device if we use cv::VideoCapture of OpenCV.

Nodes
================

cv_camera_node
------------------

## Publish ##

* ~image_raw (sensor_msgs/Image)
* ~camera_info (sensor_msgs/CameraInfo)

## Parameters ##

* ~rate (double: default 30.0) publish rate [Hz]
* ~device_id (int: default 0) capture device id

Nodelet
================

This node works as nodelet.

cv_camera/CvCameraNodelet
---------------------------
same as node.

