#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <vector>
#include <string>

namespace crosswalk_signal {

class LogPcdPublisher : public rclcpp::Node
{
public:
  LogPcdPublisher(const rclcpp::NodeOptions & options)
  : Node("log_pcd_pub", options)
  {
    folder_path_ = this->declare_parameter<std::string>("folder_path", "");
    if (folder_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No folder path provided in YAML file.");
      return;
    }

    load_pcd_files();
    if (pcd_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No .pcd files found in the specified folder.");
      return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
    index_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "/log_sync/index", 10,
      std::bind(&LogPcdPublisher::on_index_received, this, std::placeholders::_1));
  }

private:
  void load_pcd_files()
  {
    for (const auto& entry : std::filesystem::directory_iterator(folder_path_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        pcd_files_.push_back(entry.path().string());
      }
    }
    std::sort(pcd_files_.begin(), pcd_files_.end());
  }

  void on_index_received(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int idx = msg->data;
    if (idx < 0 || static_cast<size_t>(idx) >= pcd_files_.size()) {
      RCLCPP_WARN(this->get_logger(), "Invalid index received: %d", idx);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files_[idx], cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load: %s", pcd_files_[idx].c_str());
      return;
    }

    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    ros_msg.header.frame_id = "lidar";
    ros_msg.header.stamp = this->now();

    publisher_->publish(ros_msg);
    RCLCPP_INFO(this->get_logger(), "Published (index %d): %s", idx, pcd_files_[idx].c_str());
  }

  std::string folder_path_;
  std::vector<std::string> pcd_files_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr index_subscriber_;
};

} // namespace crosswalk_signal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::LogPcdPublisher)
