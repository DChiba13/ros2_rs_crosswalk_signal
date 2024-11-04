#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

namespace crosswalk_signal
{

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(rclcpp::NodeOptions options);
  ~ImagePublisher();

private:
  void cvImage2ROSImage(const cv::Mat &src, sensor_msgs::msg::Image &dst);
  void onTimerElapsed();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture camera_;
};

/*** コンストラクタ ***/
ImagePublisher::ImagePublisher(rclcpp::NodeOptions options) : Node("test_img_pub", options)
{
  /*** パラメータの初期化 ***/
  auto camera_path = this->declare_parameter<std::string>("camera_path", "/dev/video0");
  this->get_parameter("camera_path", camera_path);
  RCLCPP_INFO(this->get_logger(), "Camera path: %s", camera_path.c_str());

  /*** USBカメラの初期化 ***/
  camera_.open(camera_path);
  if (!camera_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s", camera_path.c_str());
    exit(0);
  }

  /*** パブリッシャの初期化 ***/
  pub_img_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image", 10);

  /*** タイマーの初期化 ***/
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::onTimerElapsed, this));
}

/*** デストラクタ ***/
ImagePublisher::~ImagePublisher()
{
}

/*** OpenCVのMat型からROS2のImage型へ変換するための関数 ***/
void ImagePublisher::cvImage2ROSImage(const cv::Mat &src, sensor_msgs::msg::Image &dst)
{
  dst.height = src.rows;
  dst.width = src.cols;
  if (src.type() == CV_8UC1)
  {
    dst.encoding = "mono8";
  }
  else if (src.type() == CV_8UC3)
  {
    dst.encoding = "bgr8";
  }
  dst.step = static_cast<uint32_t>(src.step);
  size_t size = src.step * src.rows;
  dst.data.resize(size);
  memcpy(&dst.data[0], src.data, size);
  dst.header.frame_id = "camera";
  dst.header.stamp = this->now();
}

/*** 一定時間ごとに呼び出されるコールバック関数 ***/
void ImagePublisher::onTimerElapsed()
{
  cv::Mat cv_img;
  auto ros_img = std::make_unique<sensor_msgs::msg::Image>();

  /*** USBカメラから画像を取得 ***/
  camera_ >> cv_img;

  if (cv_img.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Captured image is empty.");
    return;
  }
  cv::resize(cv_img, cv_img, cv::Size(1280, 720));

  /*** OpenCVのMat型 -> ROS2のImage型への変換 ***/
  cvImage2ROSImage(cv_img, *ros_img);

  /*** メッセージのパブリッシュ ***/
  pub_img_->publish(std::move(ros_img));
}

} // namespace crosswalk_signal

/*** ImagePublisherクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::ImagePublisher)
