#include "ros2_rs_crosswalk_signal/crosswalk_signal.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <traffic_signal_reco.hpp>  // ライブラリヘッダ

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using crosswalk_signal::Recognition;

using namespace std;
using namespace cv;
using namespace signal_reco;

Recognition::Recognition(rclcpp::NodeOptions options) : Node("crosswalk_signal", options)
{
  initTopic();
  RCLCPP_INFO(this->get_logger(), "Recognition node initialized.");
}

Recognition::~Recognition() {}

void Recognition::initTopic()
{
  using std::placeholders::_1;
  // サブスクライバ
  sub_img_ = this->create_subscription<Image>("/camera1/image", 10, std::bind(&Recognition::onImageSubscribed, this, _1));
  sub_pcd_ = this->create_subscription<PointCloud2>("/lidar/points", 10, std::bind(&Recognition::onPointcloudSubscribed, this, _1));
  
  // パブリッシャ
  pub_result_image_ = this->create_publisher<Image>("/signal_image", 10);
  pub_signal_state_ = this->create_publisher<ros2_rs_interfaces::msg::TrafficSignal>("/light_msg", 10);
}

void Recognition::onImageSubscribed(const Image::SharedPtr msg)
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

void Recognition::onPointcloudSubscribed(const PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  latest_pcd_ = msg;
  pcd_stamp_ = msg->header.stamp;

  processIfReady();
}

void Recognition::convertPointCloudToLidarData(const PointCloud2::SharedPtr& pointcloud, std::vector<LidarData>& lidar_data)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*pointcloud, pcl_cloud);

  lidar_data.clear();
  for (const auto& pt : pcl_cloud.points) {
    pr::LidarData p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.reflectivity = pt.intensity;
    p.range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    lidar_data.push_back(p);
  }
}

void Recognition::cvImageToROSImage(const cv::Mat &src, Image &dst)
{
  dst.height = src.rows;
  dst.width = src.cols;
  if(src.type() == CV_8UC1) dst.encoding = "mono8";
  else if(src.type() == CV_8UC3) dst.encoding = "bgr8";
  dst.step = (uint32_t)(src.step);
  size_t size = src.step * src.rows;
  dst.data.resize(size);
  memcpy(&dst.data[0], src.data, size);
  dst.header.frame_id="img";
  dst.header.stamp = this->now();
}

void Recognition::publishResultImage(const cv::Mat &camera_img)
{
  // ROS2 Imageメッセージを作成
  auto ros_img = std::make_unique<Image>();
  // cv::MatをROS2 Imageに変換
  cvImageToROSImage(camera_img, *ros_img);
  // ヘッダー情報を設定
  ros_img->header.frame_id = "camera";
  ros_img->header.stamp = image_stamp_;
  // パブリッシュ
  pub_result_image_->publish(std::move(ros_img));
}

void Recognition::publishSignalState(const string &signal_state)
{
  // TrafficSignalメッセージを作成
  auto signal_msg = ros2_rs_interfaces::msg::TrafficSignal();
  // 判定結果をメッセージに設定
  signal_msg.state = signal_state;
  // パブリッシュ
  pub_signal_state_->publish(signal_msg);
}

void Recognition::processIfReady()
{
  // カメラ画像も点群もどちらも受信して初めて処理を行う
  if (latest_image_.empty() || latest_pcd_ == nullptr) return;
  // ライブラリインスタンス
  SignalReco reco;
  // 画像を設定
  reco.src_camera_img = latest_image_;
  // 点群変換
  convertPointCloudToLidarData(latest_pcd_, reco.src_points);
  // メイン処理
  reco.loop_main();
  // 結果画像をパブリッシュ
  publishResultImage(reco.camera_img);
  // 結果文字列をパブリッシュ
  publishSignalState(reco.signal_state);
  // 状態クリア（連続処理を避けるため）
  latest_pcd_ = nullptr;
  latest_image_ = cv::Mat();
}

/*** Recognitionクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::Recognition)