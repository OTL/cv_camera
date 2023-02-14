// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"
#include <string>

namespace {
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}  // namespace

namespace cv_camera {

Driver::Driver(const rclcpp::NodeOptions& options) : Node("cv_camera", options)
{
    auto ptr = std::shared_ptr<Driver>(this, [](Driver*) {});
    this->setup();
}

bool Driver::setup()
{
    double hz_pub(DEFAULT_RATE), hz_read(DEFAULT_RATE);
    int32_t device_id(0);
    std::string port("");
    std::string frame_id("camera");
    std::string file_path("");
    std::string topic_name("");

    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("read_rate", 30.0);
    this->declare_parameter("device_id", 0);
    this->declare_parameter("port", "");
    this->declare_parameter("frame_id", "camera");
    this->declare_parameter("file_path", "");
    this->declare_parameter("topic_name", "/video_mapping/test");
    this->declare_parameter("cv_cap_prop_fourcc", (double)cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    this->get_parameter("publish_rate", hz_pub);
    this->get_parameter("read_rate", hz_read);
    this->get_parameter("device_id", device_id);
    this->get_parameter("frame_id", frame_id);
    this->get_parameter("topic_name", topic_name);

    int32_t image_width(640);
    int32_t image_height(480);

    // Timers
    m_proceed_tmr =
        this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / hz_read)), std::bind(&Driver::proceed, this));

    camera_.reset(new Capture(shared_from_this(), topic_name, PUBLISHER_BUFFER_SIZE, frame_id));

    if (this->get_parameter("file", file_path) && file_path != "")
    {
        camera_->openFile(file_path);
    }
    else if (this->get_parameter("port", port) && port != "")
    {
        camera_->open(port);
    }
    else
    {
        camera_->open(device_id);
    }
    if (this->get_parameter("image_width", image_width))
    {
        if (!camera_->setWidth(image_width))
        {
            RCLCPP_WARN(get_logger(), "fail to set image_width");
            return false;
        }
    }
    if (this->get_parameter("image_height", image_height))
    {
        if (!camera_->setHeight(image_height))
        {
            RCLCPP_WARN(get_logger(), "fail to set image_height");
            return false;
        }
    }

    camera_->setPropertyFromParam(cv::CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
    camera_->setPropertyFromParam(cv::CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
    camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
    camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
    camera_->setPropertyFromParam(cv::CAP_PROP_FPS, "cv_cap_prop_fps");
    camera_->setPropertyFromParam(cv::CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
    camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
    camera_->setPropertyFromParam(cv::CAP_PROP_FORMAT, "cv_cap_prop_format");
    camera_->setPropertyFromParam(cv::CAP_PROP_MODE, "cv_cap_prop_mode");
    camera_->setPropertyFromParam(cv::CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
    camera_->setPropertyFromParam(cv::CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
    camera_->setPropertyFromParam(cv::CAP_PROP_SATURATION, "cv_cap_prop_saturation");
    camera_->setPropertyFromParam(cv::CAP_PROP_HUE, "cv_cap_prop_hue");
    camera_->setPropertyFromParam(cv::CAP_PROP_GAIN, "cv_cap_prop_gain");
    camera_->setPropertyFromParam(cv::CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
    camera_->setPropertyFromParam(cv::CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

    camera_->setPropertyFromParam(cv::CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
    camera_->setPropertyFromParam(cv::CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
    publish_tmr_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / hz_pub)), [&]() {
        if (camera_->capture())
        {
            camera_->publish();
        }
    });
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif  // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif  // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
    camera_->setPropertyFromParam(cv::CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif  // CV_CAP_PROP_BUFFERSIZE

    rate_.reset(new rclcpp::Rate(hz_read));
    return true;
}

void Driver::proceed()
{
    camera_->grab();
    rate_->sleep();
}

Driver::~Driver() {}

}  // namespace cv_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(cv_camera::Driver)
