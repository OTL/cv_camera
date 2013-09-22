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
    frame_id_(frame_id)
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
}

void Capture::open()
{
  open(0);
}

bool Capture::capture()
{
  if(cap_.read(bridge_.image)) {
    bridge_.encoding = enc::BGR8;
    info_.height = bridge_.image.rows;
    info_.width = bridge_.image.cols;
    ros::Time now = ros::Time::now();
    info_.header.stamp = now;
    info_.header.frame_id = frame_id_;
    bridge_.header.stamp = now;
    bridge_.header.frame_id = frame_id_;
    return true;
  }
  return false;
}

void Capture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
}

}  // namespace cv_camera
