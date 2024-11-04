#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace crosswalk_signal
{

class HSVChecker : public rclcpp::Node
{
public:
  HSVChecker(rclcpp::NodeOptions options);
  ~HSVChecker();
  void extractRed(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_red);
  void extractGreen(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_green);
  void extractYellow(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_yellow);

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // HSVパラメータを宣言
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

  cv::Mat hsv_image;

  // サブスクライバを宣言
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

/*** コンストラクタ ***/
HSVChecker::HSVChecker(rclcpp::NodeOptions options) : Node("check_hsv", options)
{
  // パラメータの宣言と取得
  this->declare_parameter<int>("MIN_H_RED_01", 0);
  this->declare_parameter<int>("MAX_H_RED_01", 180);
  this->declare_parameter<int>("MIN_H_RED_02", 0);
  this->declare_parameter<int>("MAX_H_RED_02", 180);
  this->declare_parameter<int>("MIN_S_RED", 0);
  this->declare_parameter<int>("MAX_S_RED", 255);
  this->declare_parameter<int>("MIN_V_RED", 0);
  this->declare_parameter<int>("MAX_V_RED", 255);

  this->declare_parameter<int>("MIN_H_GREEN", 0);
  this->declare_parameter<int>("MAX_H_GREEN", 180);
  this->declare_parameter<int>("MIN_S_GREEN", 0);
  this->declare_parameter<int>("MAX_S_GREEN", 255);
  this->declare_parameter<int>("MIN_V_GREEN", 0);
  this->declare_parameter<int>("MAX_V_GREEN", 255);

  this->declare_parameter<int>("MIN_H_YELLOW", 0);
  this->declare_parameter<int>("MAX_H_YELLOW", 180);
  this->declare_parameter<int>("MIN_S_YELLOW", 0);
  this->declare_parameter<int>("MAX_S_YELLOW", 255);
  this->declare_parameter<int>("MIN_V_YELLOW", 0);
  this->declare_parameter<int>("MAX_V_YELLOW", 255);
  
  this->get_parameter("MIN_H_RED_01", MIN_H_RED_01);
  this->get_parameter("MAX_H_RED_01", MAX_H_RED_01);
  this->get_parameter("MIN_H_RED_02", MIN_H_RED_02);
  this->get_parameter("MAX_H_RED_02", MAX_H_RED_02);
  this->get_parameter("MIN_S_RED", MIN_S_RED);
  this->get_parameter("MAX_S_RED", MAX_S_RED);
  this->get_parameter("MIN_V_RED", MIN_V_RED);
  this->get_parameter("MAX_V_RED", MAX_V_RED);

  this->get_parameter("MIN_H_GREEN", MIN_H_GREEN);
  this->get_parameter("MAX_H_GREEN", MAX_H_GREEN);
  this->get_parameter("MIN_S_GREEN", MIN_S_GREEN);
  this->get_parameter("MAX_S_GREEN", MAX_S_GREEN);
  this->get_parameter("MIN_V_GREEN", MIN_V_GREEN);
  this->get_parameter("MAX_V_GREEN", MAX_V_GREEN);

  this->get_parameter("MIN_H_YELLOW", MIN_H_YELLOW);
  this->get_parameter("MAX_H_YELLOW", MAX_H_YELLOW);
  this->get_parameter("MIN_S_YELLOW", MIN_S_YELLOW);
  this->get_parameter("MAX_S_YELLOW", MAX_S_YELLOW);
  this->get_parameter("MIN_V_YELLOW", MIN_V_YELLOW);
  this->get_parameter("MAX_V_YELLOW", MAX_V_YELLOW);

  // 画像サブスクライバの初期化
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image", 10, std::bind(&HSVChecker::imageCallback, this, std::placeholders::_1));
}

/*** デストラクタ ***/
HSVChecker::~HSVChecker()
{
}
void HSVChecker::extractRed(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_red)
{
  // 青信号の緑色を抽出
  for(int y = 0; y < src.rows; y++){
    for(int x = 0; x < src.cols; x++){
      cv::Vec3b val = hsv.at<cv::Vec3b>(y, x);
      if(  ((MIN_H_RED_01 <= val[0] && val[0] <= MAX_H_RED_01) || 
           (MIN_H_RED_02 <= val[0] && val[0] <= MAX_H_RED_02))
         && MIN_S_RED <= val[1] && val[1] <= MAX_S_RED
         && MIN_V_RED <= val[2] && val[2] <= MAX_V_RED){
          ex_red.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(y, x);
      }
    }
  }
}
void HSVChecker::extractGreen(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_green)
{
  // 青信号の緑色を抽出
  for(int y = 0; y < src.rows; y++){
    for(int x = 0; x < src.cols; x++){
      cv::Vec3b val = hsv.at<cv::Vec3b>(y, x);
      if(    MIN_H_GREEN <= val[0] && val[0] <= MAX_H_GREEN
          && MIN_S_GREEN <= val[1] && val[1] <= MAX_S_GREEN
          && MIN_V_GREEN <= val[2] && val[2] <= MAX_V_GREEN){
          ex_green.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(y, x);
      }
    }
  }
}
void HSVChecker::extractYellow(cv::Mat &src, cv::Mat &hsv, cv::Mat &ex_yellow)
{
  // 青信号の緑色を抽出
  for(int y = 0; y < src.rows; y++){
    for(int x = 0; x < src.cols; x++){
      cv::Vec3b val = hsv.at<cv::Vec3b>(y, x);
      if(    MIN_H_YELLOW <= val[0] && val[0] <= MAX_H_YELLOW
          && MIN_S_YELLOW <= val[1] && val[1] <= MAX_S_YELLOW
          && MIN_V_YELLOW <= val[2] && val[2] <= MAX_V_YELLOW){
          ex_yellow.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(y, x);
      }
    }
  }
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    // userdata から hsv_image を取得し、適切な型にキャスト
    cv::Mat* hsv_image_ptr = static_cast<cv::Mat*>(userdata);
    
    // std::cout << "hsv_image.width: " << hsv_image_ptr->cols 
    //           << ", hsv_image.height: " << hsv_image_ptr->rows << std::endl;

    // 画像の範囲を確認
    if (x < 0 || y < 0 || x >= hsv_image_ptr->cols || y >= hsv_image_ptr->rows) {
      std::cout << "y : " << y << std::endl;
      std::cout << "x : " << x << std::endl;
      std::cout << "Clicked point is out of image bounds." << std::endl;
      return;
    }

    // HSV ピクセル値を取得
    cv::Vec3b hsv_pixel = hsv_image_ptr->at<cv::Vec3b>(y, x);
    int h = hsv_pixel[0];
    int s = hsv_pixel[1];
    int v = hsv_pixel[2];

    // ターミナルに HSV 値を出力
    std::cout << "Clicked pixel at (" << x << ", " << y << ") - HSV: "
              << "H=" << h << ", S=" << s << ", V=" << v << std::endl;
  }
}

/*** 画像を受け取りHSVフィルタ処理を行うコールバック関数 ***/
void HSVChecker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // ROS Image メッセージを OpenCV の形式に変換
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // BGR から HSV に変換
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
    std::cout << "hsv_image.width: " << hsv_image.cols << ", hsv_image.height: " << hsv_image.rows << std::endl;

    // HSV範囲でフィルタをかける
    cv::Mat ex_red = cv::Mat::zeros(image.size(), image.type());
    cv::Mat ex_green = cv::Mat::zeros(image.size(), image.type());
    cv::Mat ex_yellow = cv::Mat::zeros(image.size(), image.type());
    extractRed(image, hsv_image, ex_red);
    extractGreen(image, hsv_image, ex_green);
    extractYellow(image, hsv_image, ex_yellow);

    // フィルタリング結果を表示（OpenCVウィンドウ）
    cv::imshow("camera image", image);
    cv::imshow("extract red", ex_red);
    cv::imshow("extract green", ex_green);
    cv::imshow("extract yellow", ex_yellow);
    // マウスコールバック関数を設定
    cv::setMouseCallback("camera image", onMouse, &hsv_image);
    cv::setMouseCallback("extract red", onMouse, &hsv_image);
    cv::setMouseCallback("extract green", onMouse, &hsv_image);
    cv::setMouseCallback("extract yellow", onMouse, &hsv_image);

    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
  }
}

}  // namespace crosswalk_signal

/*** HSVChecker クラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::HSVChecker)
