#ifndef ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_
#define ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros2_rs_interfaces/msg/traffic_signal.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// C++ includes
#include <iostream>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <ryusei/common/logger.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/common/math.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <fstream>
#include <cstring>
#include <string.h>

using sensor_msgs::msg::Image;
using ros2_rs_interfaces::msg::TrafficSignal;
using namespace project_ryusei;
using namespace cv;
using namespace std;

#define DEG_TO_RAD (M_PI / 180.0)

namespace crosswalk_signal
{
  class Recognition : public rclcpp::Node
  {
  public:
    Recognition(rclcpp::NodeOptions options);
    ~Recognition();

  private:
    // === Subscribers ===
    rclcpp::Subscription<Image>::SharedPtr sub_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;

    // === Publishers ===
    rclcpp::Publisher<TrafficSignal>::SharedPtr pub_signal_state_;
    rclcpp::Publisher<Image>::SharedPtr pub_result_image_;

    // === Data buffer ===
    cv::Mat latest_image_;
    rclcpp::Time image_stamp_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pcd_;
    rclcpp::Time pcd_stamp_;

    // === Mutex ===
    std::mutex data_mutex_;

    // === Processing ===
    void onImageSubscribed(Image::SharedPtr img);
    void onPointcloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processIfReady();
    void cvImageToROSImage(const cv::Mat &src, Image &dst);
    void SignalImagePublisher(Mat &camera_img);
  };
} // namespace crosswalk_signal

#endif // ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_