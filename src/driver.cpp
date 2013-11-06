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
    camera_node_(camera_node)
{
}

void Driver::setup() {
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string frame_id("camera");
  std::string file_path("");
  
  private_node_.getParam("device_id", device_id);
  private_node_.getParam("frame_id", frame_id);
  private_node_.getParam("rate", hz);

  int32_t image_width(640);
  int32_t image_height(480);
  
  camera_.reset(new Capture(camera_node_,
                            "image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id));
  if (private_node_.getParam("file", file_path) &&
      file_path != "") {
    camera_->openFile(file_path);
  } else {
    camera_->open(device_id);
  }
  if (private_node_.getParam("image_width", image_width)) {
    if(!camera_->setWidth(image_width)) {
      ROS_WARN("fail to set image_width");
    }
  }
  if (private_node_.getParam("image_height", image_height)) {
    if(!camera_->setHeight(image_height)) {
      ROS_WARN("fail to set image_height");
    }
  }
  
  rate_.reset(new ros::Rate(hz));
}

void Driver::proceed() {
  if (camera_->capture()) {
    camera_->publish();
  }
  rate_->sleep();
}

Driver::~Driver() {
}

}  // namespace cv_camera
