// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_DRIVER_H
#define CV_CAMERA_DRIVER_H

#include "cv_camera/capture.hpp"

namespace cv_camera {

/**
 * @brief ROS cv camera driver.
 *
 * This wraps getting parameters and publish in specified rate.
 */
class Driver : public rclcpp::Node
{
   public:
    /**
     * @brief construct with ROS node handles.
     *
     * use private_node for getting topics like ~rate or ~device,
     * camera_node for advertise and publishing images.
     */
    explicit Driver(const rclcpp::NodeOptions& options);
    ~Driver();

    rclcpp::TimerBase::SharedPtr m_proceed_tmr; /*!< @sa proceed() */

    /**
     * @brief Setup camera device and ROS parameters.
     *
     * @throw cv_camera::DeviceError device open failed.
     */
    bool setup();
    /**
     * @brief Capture, publish and sleep
     */
    void proceed();

   private:
    /**
     * @brief ROS private timer for publishing images.
     */
    rclcpp::TimerBase::SharedPtr publish_tmr_;
    /**
     * @brief wrapper of cv::VideoCapture.
     */
    std::shared_ptr<Capture> camera_;

    /**
     * @brief publishing rate.
     */
    std::shared_ptr<rclcpp::Rate> rate_;
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
