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
    /**
     * @brief camera port.
     */
    std::string port;
    /**
     * @brief Topic name.
     */
    std::string topic_name;
    /**
     * @brief Camera name.
     */
    std::string name;
    /**
     * @brief Environment variables
     */
    const int VIDEO_STREAM_CAM_RECOVERY_TIME = getEnv("VIDEO_STREAM_CAM_RECOVERY_TIME", 2);
    const int VIDEO_STREAM_CAM_RECOVERY_TRIES = getEnv("VIDEO_STREAM_CAM_RECOVERY_TRIES", 10);
    /**
     * @brief Reconnection attempts to open a camera port
     */
    int m_reconnection_attempts = 0;
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
