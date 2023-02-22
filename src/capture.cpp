// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.hpp"
#include <sstream>
#include <string>

namespace cv_camera {

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(rclcpp::Node::SharedPtr node, const std::string& topic_name, uint32_t buffer_size,
                 const std::string& frame_id)
    : node_(node)
    , it_(node_)
    , topic_name_(topic_name)
    , frame_id_(frame_id)
    , buffer_size_(buffer_size)
    , info_manager_(node_.get(), frame_id)
    , capture_delay_(rclcpp::Duration(0, 0.0))
{
    int dur = 0;
    m_pub_image_ptr = node->create_publisher<sensor_msgs::msg::Image>(topic_name, 1);
    node_->get_parameter_or("capture_delay", dur, dur);
    this->capture_delay_ = rclcpp::Duration(dur, 0.0);
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

    rescale_camera_info_ = false;
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
            RCLCPP_ERROR(node_->get_logger(), "Setting with code %d and value %f failed", code, value);
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

bool Capture::open(const std::string& port)
{
    std::string device;

    if (det_device_path(port.c_str()).compare("-1") != 0)
    {
        device = "/dev/video" + det_device_path(port.c_str());
    }
    else
    {
        std::cout << "Device couldnt be determined in port " << port << std::endl;
    }

    if (device.empty())
    {
        std::cout << "Camera Not found in port " << port << std::endl;
        return false;
    }
    else
    {
        std::cout << "The port " << port << " is located in " << device << std::endl;
    }

    cap_.open(device, cv::CAP_V4L2);

    std::chrono::milliseconds video_recovery_time(VIDEO_STREAM_CAM_RECOVERY_TIME * 1000);  // or whatever

    while (!cap_.isOpened() && m_reconnection_attempts < 10)
    {
        m_reconnection_attempts += 1;
        RCLCPP_ERROR(node_->get_logger(), "Error while opening %s. Retrying %d/10 ", port.c_str(),
                     m_reconnection_attempts);
        cap_.open(device, cv::CAP_V4L2);
        std::this_thread::sleep_for(video_recovery_time);
    }

    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to open device %s.", port.c_str());
        return false;
        // throw DeviceError("device_path " + device_path + " cannot be opened");
    }
    // pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
    rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = buffer_size_;
    pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, custom_qos);

    loadCameraInfo();
    return true;
}

void Capture::open() { open(0); }

void Capture::openFile(const std::string& file_path)
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
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());

        // Pack the OpenCV image into the ROS image.
        set_now(msg->header.stamp);
        msg->header.frame_id = "camera_frame";
        msg->height = bridge_.image.rows;
        msg->width = bridge_.image.cols;
        msg->encoding = mat_type2encoding(bridge_.image.type());
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(bridge_.image.step);
        msg->data.assign(bridge_.image.datastart, bridge_.image.dataend);

        m_pub_image_ptr->publish(std::move(msg));

        return true;
    }
    return false;
}

// void Capture::publish() { pub_.publish(*getImageMsgPtr(), info_); }

bool Capture::setPropertyFromParam(int property_id, const std::string& param_name)
{
    if (cap_.isOpened())
    {
        double value = 0.0;
        if (node_->get_parameter(param_name, value))
        {
            // RCLCPP_INFO(node_->get_logger(), "setting property %s = %lf", param_name.c_str(), value);
            return cap_.set(property_id, value);
        }
    }
    return true;
}

bool Capture::grab()
{
    if (!cap_.isOpened())
    {
        return false;
    }
    return cap_.grab();
}

std::string Capture::execute_command(const char* command)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    return result;
}
std::string Capture::det_device_path(const char* port)
{
    std::string video_device = "-1";
    // TODO: instead of reading the output from shell, iter the directory
    std::string video_devices = execute_command("ls /dev/video*");
    std::string delimiter = "\n";

    size_t pos = 0;
    std::string pre_token;
    std::string token;
    std::string output_command;
    std::vector<int> devices;

    while ((pos = video_devices.find(delimiter)) != std::string::npos)
    {
        // get /dev/videoX substring
        pre_token = video_devices.substr(0, pos);
        // get number of the cam device
        token = pre_token.substr(10, 2);
        devices.push_back(std::stoi(token));

        video_devices.erase(0, pos + delimiter.length());
    }

    // Sort the vector to get devices in order
    std::sort(devices.begin(), devices.end());

    // Iter the devices to identify which port correspond to which videoX
    for (const auto& cam : devices)
    {
        output_command = "udevadm info --query=path --name=/dev/video" + std::to_string(cam);
        std::string camera_device_info = execute_command(output_command.c_str());
        if (camera_device_info.find(port) != std::string::npos)
        {
            video_device = std::to_string(cam);
            return video_device;
        }
    }

    return video_device;
}

}  // namespace cv_camera
