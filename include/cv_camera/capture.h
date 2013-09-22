#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include "cv_camera/exception.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

/**
 * @brief namespace of this package
 */
namespace cv_camera {

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class Capture {
 public:
  /** 
   * @brief costruct with ros node and topic settings
   * 
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   */
  Capture(ros::NodeHandle& node,
           const std::string& topic_name,
           int32_t buffer_size);
  
  /** 
   * @brief Open capture device with device ID.
   * 
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   * 
   */
  void open(int32_t device_id);
  
  /** 
   * @brief Open default camera device.
   * 
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /** 
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   */
  void capture();

  /** 
   * @brief Publish the image that is already captured by capture().
   * 
   */
  void publish();

  /** 
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   * 
   * @return CameraInfo
   */
  const sensor_msgs::CameraInfo& getInfo() const
  {
    return info_;
  }

  /** 
   * @brief accessor of cv::Mat
   * 
   * you have to call capture() before call this.
   * 
   * @return captured cv::Mat
   */
  const cv::Mat& getCvImage() const
  {
    return bridge_.image;
  }
  
  /** 
   * @brief accessor of ROS Image message.
   * 
   * you have to call capture() before call this.
   * 
   * @return message pointer.
   */
  const sensor_msgs::ImagePtr getImageMsgPtr() const
  {
    return bridge_.toImageMsg();
  }
  
 private:

  /**
   * @brief node handle for advertise.
   */
  ros::NodeHandle node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string topic_name_;

  /**
   * @brief size of publisher buffer
   */
  int32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher pub_;

  /**
   * @brief capture device.
   */
  cv::VideoCapture cap_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage bridge_;

  /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::CameraInfo info_;
};

}  // end namespace

#endif  // end CV_CAMERA_CAPTURE_H
