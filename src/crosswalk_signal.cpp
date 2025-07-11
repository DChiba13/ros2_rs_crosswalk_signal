#include "ros2_rs_crosswalk_signal/crosswalk_signal.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>

#include <traffic_signal_reco.hpp>  // ライブラリヘッダ

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using crosswalk_signal::Recognition;

using namespace std;
using namespace cv;
using namespace signal_reco;

Recognition::Recognition(const rclcpp::NodeOptions & options)
: Node("crosswalk_signal", options)
{
  using std::placeholders::_1;

  // サブスクライバ
  sub_img_ = this->create_subscription<Image>(
    "/camera1/image", 10, std::bind(&Recognition::onImageReceived, this, _1));
  
  sub_pcd_ = this->create_subscription<PointCloud2>(
    "/lidar/points", 10, std::bind(&Recognition::onPointcloudReceived, this, _1));
  
  // パブリッシャ
  pub_result_image_ = this->create_publisher<Image>("/signal_image", 10);
  pub_signal_state_ = this->create_publisher<ros2_rs_interfaces::msg::TrafficSignal>("/light_msg", 10);

  RCLCPP_INFO(this->get_logger(), "Recognition node initialized.");
}

Recognition::~Recognition() {}

void Recognition::onImageReceived(const Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    latest_image_ = cv_ptr->image.clone();
    image_stamp_ = msg->header.stamp;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  processIfReady();
}

void Recognition::onPointcloudReceived(const PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  latest_pcd_ = msg;
  pcd_stamp_ = msg->header.stamp;

  processIfReady();
}

void Recognition::processIfReady()
{
  if (latest_image_.empty() || latest_pcd_ == nullptr) return;

  // ライブラリインスタンス
  SignalReco reco;

  // 画像を設定
  reco.src_camera_img = latest_image_;

  // 点群変換
  reco.src_points.clear();
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*latest_pcd_, pcl_cloud);
  for (const auto& pt : pcl_cloud.points) {
    pr::LidarData p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.reflectivity = pt.intensity;
    p.range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    reco.src_points.push_back(p);
  }

  // メイン処理
  reco.loop_main();

  // 結果画像をパブリッシュ
  auto ros_img = std::make_unique<Image>();
  cv_bridge::CvImage img_bridge;
  img_bridge.encoding = "bgr8";
  img_bridge.image = reco.camera_img;
  img_bridge.header.stamp = image_stamp_;
  img_bridge.toImageMsg(*ros_img);
  ros_img->header.frame_id = "camera";
  pub_result_image_->publish(std::move(ros_img));

  // 結果文字列をパブリッシュ
  auto signal_msg = ros2_rs_interfaces::msg::TrafficSignal();
  signal_msg.state = reco.signal_state;
  signal_msg.header.stamp = image_stamp_;
  signal_msg.header.frame_id = "camera";
  pub_signal_state_->publish(signal_msg);

  // 状態クリア（連続処理を避けるため）
  latest_pcd_ = nullptr;
  latest_image_ = cv::Mat();
}
