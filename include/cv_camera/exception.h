#ifndef CV_CAMERA_EXCEPTION_H
#define CV_CAMERA_EXCEPTION_H

#include <stdexcept>

namespace cv_camera {

/**
 * @brief ROS cv camera device exception.
 *
 */
class DeviceError : public std::runtime_error {
 public:
  DeviceError(const std::string& cause):
      std::runtime_error(cause) {}
};

}  // end namespace cv_camera

#endif  // CV_CAMERA_EXCEPTION_H
