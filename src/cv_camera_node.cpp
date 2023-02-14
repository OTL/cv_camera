// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto opts = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto driver = std::make_shared<cv_camera::Driver>(opts);
    exec.add_node(driver);

    exec.spin();
    rclcpp::shutdown();

    return 0;
}
