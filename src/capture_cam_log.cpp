#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <fmt/core.h>

using namespace std::chrono;

namespace crosswalk_signal
{

class CaptureCamLog : public rclcpp::Node
{
public:
  CaptureCamLog() : Node("capture_cam_log"), count_(1)
  {
    initialize();
  }

  CaptureCamLog(const rclcpp::NodeOptions &options) : Node("capture_cam_log", options), count_(1)
  {
    initialize();
  }

private:
  void initialize()
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image", 10,
        std::bind(&CaptureCamLog::image_callback, this, std::placeholders::_1));

    // ディレクトリの作成
    create_log_directory();
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
      return;
    }

    cv::imshow("Camera Feed", cv_image->image);
    int key = cv::waitKey(30); 

    if (key == 'F' || key == 'f') {
      save_image(cv_image->image);
    }
  }

  void save_image(const cv::Mat &image)
  {
    std::string file_name = log_directory_ + "/" + generate_image_name();
    cv::imwrite(file_name, image);
    RCLCPP_INFO(this->get_logger(), "Image saved: %s", file_name.c_str());
  }

  std::string generate_image_name()
  {
    return fmt::format("{:06d}.png", count_++);
  }

  void create_log_directory()
  {
    // 現在の作業ディレクトリを取得し、相対パスを絶対パスに変換
    std::string base_directory = std::filesystem::current_path().string();
    log_directory_ = base_directory + "/signal_logs/camera1/" + generate_timestamp();
    std::filesystem::create_directories(log_directory_);
  }

  std::string generate_timestamp()
  {
    auto now = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", &now_tm);
    return std::string(buffer);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::string log_directory_;
  int count_;
};

} // namespace crosswalk_signal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::CaptureCamLog)
