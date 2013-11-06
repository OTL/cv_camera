#include "cv_camera/capture.h"
#include <sstream>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(ros::NodeHandle& node,
                 const std::string& topic_name,
                 int32_t buffer_size,
                 const std::string& frame_id) :
    node_(node),
    it_(node_),
    topic_name_(topic_name),
    buffer_size_(buffer_size),
    frame_id_(frame_id),
    info_manager_(node_, frame_id)
{
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id);
  if (!cap_.isOpened()) {
    std::stringstream stream;
    stream << "device_id " << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string& file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened()) {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}

bool Capture::capture()
{
  if(cap_.read(bridge_.image)) {
    ros::Time now = ros::Time::now();
    bridge_.encoding = enc::BGR8;
    bridge_.header.stamp = now;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0) {
      info_.height = bridge_.image.rows;
    }
    if (info_.width == 0) {
      info_.width = bridge_.image.cols;
    }
    info_.header.stamp = now;
    info_.header.frame_id = frame_id_;

    return true;
  }
  return false;
}

void Capture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
}

}  // namespace cv_camera
