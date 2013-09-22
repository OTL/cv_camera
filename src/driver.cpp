#include "cv_camera/driver.h"

namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{

Driver::Driver(ros::NodeHandle& private_node,
               ros::NodeHandle& camera_node) :
    private_node_(private_node),
    camera_(new Capture(camera_node, "image_raw", PUBLISHER_BUFFER_SIZE))
{
}

void Driver::setup() {
  double hz(DEFAULT_RATE);

  int32_t device_id(0);
  
  bool is_opened = false;
  private_node_.getParam("device_id", device_id);
  camera_->open(device_id);

  private_node_.getParam("rate", hz);
  rate_.reset(new ros::Rate(hz));
}

void Driver::proceed() {
  camera_->capture();
  camera_->publish();
  rate_->sleep();
}

Driver::~Driver() {
}

}  // namespace cv_camera
