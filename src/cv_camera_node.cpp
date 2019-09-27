// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto opts = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr private_node = std::make_shared<rclcpp::Node>("cv_camera", opts);
  cv_camera::Driver driver(private_node, private_node);

  try
  {
    driver.setup();
    while (rclcpp::ok())
    {
      driver.proceed();
      rclcpp::spin_some(private_node);
    }
  }
  catch (cv_camera::DeviceError &e)
  {
    RCLCPP_ERROR(private_node->get_logger(),"cv camera open failed: %s", e.what());
    return 1;
  }

  return 0;
}
