// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(rclcpp::Node::SharedPtr node, const std::string &topic_name,
                 uint32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      frame_id_(frame_id),
      buffer_size_(buffer_size),
      info_manager_(node_.get(), frame_id),
      capture_delay_(rclcpp::Duration(0))
{
    int dur = 0;
    node_->get_parameter_or("capture_delay",dur,dur);
    this->capture_delay_ = rclcpp::Duration(dur);
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_->get_parameter("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid camera info URL %s", url.c_str());
    }
  }

  rescale_camera_info_  = false;
  node_->get_parameter_or("rescale_camera_info", rescale_camera_info_, rescale_camera_info_);

  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_->get_parameter(param_for_code, code) || !node_->get_parameter(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      RCLCPP_ERROR(node_->get_logger(),"Setting with code %s and value %s failed", code, value);
    }
  }
}

void Capture::rescaleCameraInfo(uint width, uint height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.k[0] *= width_coeff;
  info_.k[2] *= width_coeff;
  info_.k[4] *= height_coeff;
  info_.k[5] *= height_coeff;

  info_.p[0] *= width_coeff;
  info_.p[2] *= width_coeff;
  info_.p[5] *= height_coeff;
  info_.p[6] *= height_coeff;
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  // pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
  custom_qos.depth = buffer_size_;
  pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, custom_qos);

  loadCameraInfo();
}

void Capture::open(const std::string &device_path)
{
  std::string device = det_device_path(device_path.c_str());
  
  std::cout << device << std::endl;
  cap_.open(device, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path " + device_path + " cannot be opened");
  }
  // pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
  custom_qos.depth = buffer_size_;
  pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, custom_qos);

  loadCameraInfo();
}

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  // pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
  custom_qos.depth = buffer_size_;  
  pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, custom_qos);

  loadCameraInfo();
}

bool Capture::capture()
{
  if (cap_.retrieve(bridge_.image))
  {
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time stamp = system_clock.now() - capture_delay_;
    bridge_.encoding = enc::BGR8;
    bridge_.header.stamp = stamp;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = bridge_.image.rows;
      info_.width = bridge_.image.cols;
    }
    else if (info_.height != bridge_.image.rows || info_.width != bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(bridge_.image.cols, bridge_.image.rows);
        RCLCPP_INFO_ONCE(node_->get_logger(),"Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, bridge_.image.cols, bridge_.image.rows);
      }
      else
      {
        RCLCPP_WARN_ONCE(node_->get_logger(),"Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, bridge_.image.cols, bridge_.image.rows);
      }
    }
    info_.header.stamp = stamp;
    info_.header.frame_id = frame_id_;

    return true;
  }
  return false;
}

void Capture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_->get_parameter(param_name, value))
    {
      RCLCPP_INFO(node_->get_logger(),"setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

bool Capture::grab()
{
  if (!cap_.isOpened()){
    return false;
  }
  return cap_.grab();
}

std::string Capture::execute_command(const char* command)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
  if (!pipe) {
      throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
  }
  // Debug print
  std::cout << result << std::endl;

  return result;
}
std::string Capture::det_device_path(const char* port)
{
  std::string video_device;
  std::string video_devices = execute_command("ls /dev/video*");
  std::string delimiter = "\n";

  size_t pos = 0;
  std::string token;
  std::string output_command;
  while ((pos = video_devices.find(delimiter)) != std::string::npos) {
      token = video_devices.substr(0, pos);
      std::cout << token << std::endl;
      output_command = "udevadm info --query=path --name="+token;
      std::cout << output_command << std::endl;
      std::string camera_device_info = execute_command(output_command);
      std::cout << camera_device_info << std::endl;
      if (camera_device_info.find(port) != std::string::npos)
      {
        video_device=token;
        return video_device
      }
      

      
      video_devices.erase(0, pos + delimiter.length());
  }
  // std::cout << video_devices << std::endl;
}

} // namespace cv_camera
