#ifndef ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_
#define ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_

// ros2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros2_rs_interfaces/msg/traffic_signal.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// c++ includes
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
    rclcpp::Subscription<Image>::SharedPtr sub_img_;
    TrafficSignal light_msg_;
    rclcpp::Publisher<TrafficSignal>::SharedPtr light_pub_;
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    mutex g_mutex_;

    // 画像の信号認識する領域を限定[%]
    double IMG_LEFT_EDGE_RATIO;
    double IMG_TOP_EDGE_RATIO;
    double IMG_RIGHT_EDGE_RATIO;
    double IMG_BOTTOM_EDGE_RATIO;
    int img_left;
    int img_top;
    int img_width;
    int img_height;

    // 赤青黃のHSV表色系での閾値を設定
    int MIN_H_RED_01, MAX_H_RED_01;
    int MIN_H_RED_02, MAX_H_RED_02;
    int MIN_S_RED, MAX_S_RED;
    int MIN_V_RED, MAX_V_RED;

    int MIN_H_GREEN, MAX_H_GREEN;
    int MIN_S_GREEN, MAX_S_GREEN;
    int MIN_V_GREEN, MAX_V_GREEN;

    int MIN_H_YELLOW, MAX_H_YELLOW;
    int MIN_S_YELLOW, MAX_S_YELLOW;
    int MIN_V_YELLOW, MAX_V_YELLOW;

    // 信号が青なのか赤なのか判断するフラグ
    bool green_light_flag = false;
    bool red_light_flag = false;

    // IMAGE_THRESHフレーム連続で赤、青が認識されると信号とみなす
    int RED_IMAGE_THRESH;
    int GREEN_IMAGE_THRESH;

    // 赤、青信号が何フレーム連続で検出されたか数えるcount
    int red_cnt = 0;
    int green_cnt = 0;

    // 赤or青判定を画像に表示する文字
    std::string light_msg_state;

    // 信号の候補領域のピクセル数の閾値
    int pixel_num = 0;
    int MIN_PIX_NUM;
    int MAX_PIX_NUM;

    // 信号の候補領域のアスペクト比の閾値
    // 横 : 縦 = ASPECT_RATIO : 1
    double aspect_ratio = .0f;
    double MIN_ASPECT_RATIO;
    double MAX_ASPECT_RATIO;

    int YELLOW_PIX_TH; // 候補領域内の黄色画素ピクセル数の閾値
    // 横 : 縦 = ASPECT_RATIO_YELLOW : 1
    double aspect_ratio_yellow;
    double MIN_YELLOW_ASPECT_RATIO;
    double MAX_YELLOW_ASPECT_RATIO;

    vector<Rect> rect_sign;
    vector<Rect> rect_region;

    Mat camera_img, hsv, top_region, extract_red, extract_green,extract_yellow, bin_img_red, bin_img_green, bin;
    void onImageSubscribed(Image::SharedPtr img);
    void init_param();  // パラメータ初期化
    void extractRedSignal(Mat &rgb, Mat &hsv, Mat &extract_red);
    void extractGreenSignal(Mat &rgb, Mat &hsv, Mat &extract_green);
    void binalizeImage(Mat &src, Mat &gray_img);
    Mat red_median, green_median, kernel_erode, kernel_dilate, red_erode, green_erode, red_dilate, green_dilate;
    Mat labeled_red, labeled_green;
    Mat stats_red, stats_green, centroids_red, centroids_green;
    int num_labels_red, num_labels_green;
    string color_type;
    void createCandidateArea(Mat &camera, const Mat &stats, vector<int> &left, vector<int> &top, vector<int> &width, vector<int> &height, int num_labels, string color_type);
    void extractYellowInBlob(Mat &rgb, int num_labels, const vector<int> &widths, const vector<int> &heights, const vector<int> &lefts, const vector<int> &tops, bool isRedSignal, Mat &top_region, int img_left, int img_top);
    void drawOverlay(cv::Mat &image, bool red_light_flag, bool green_light_flag);
    void addTextToImage(cv::Mat &image, const TrafficSignal &light_state);
    void cvImageToROSImage(const cv::Mat &src, Image &dst);
    void SignalImagePublisher(Mat &camera_img);
    
    void run(Mat &camera_img);
  };
} // namespace crosswalk_signal

#endif // ROS2_RS_CROSSWALK_SIGNAL_COMMON_INCLUDES_HPP_