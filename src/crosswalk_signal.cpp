#include "ros2_rs_crosswalk_signal/crosswalk_signal.hpp"

/*** 名前空間を省略して利用できるように宣言 ***/
using sensor_msgs::msg::Image;
using namespace crosswalk_signal;

/*** コンストラクタ ***/
Recognition::Recognition(rclcpp::NodeOptions options) : Node("crosswalk_signal", options)
{
  using std::placeholders::_1;
  Recognition::init_param();
  /*** サブスクライバ,パブリッシャの初期化 ***/
  sub_img_ = this->create_subscription<Image>("/camera1/image", 10, std::bind(&Recognition::onImageSubscribed, this, _1));
  light_pub_ = this->create_publisher<TrafficSignal>("/light_msg",10);
  image_pub_ = this->create_publisher<Image>("/signal_image", 10);
}

/*** デストラクタ ***/
Recognition::~Recognition()
{

}

// パラメータ初期化
void Recognition::init_param()
{
  /*** declare_parameter ***/
  this->declare_parameter<double>("IMG_TOP_EDGE_RATIO", 0);
  this->declare_parameter<double>("IMG_BOTTOM_EDGE_RATIO", 0);
  this->declare_parameter<double>("IMG_LEFT_EDGE_RATIO", 0);
  this->declare_parameter<double>("IMG_RIGHT_EDGE_RATIO", 0);

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

  this->declare_parameter<int>("RED_IMAGE_THRESH", 0);
  this->declare_parameter<int>("GREEN_IMAGE_THRESH", 0);
  this->declare_parameter<int>("MIN_PIX_NUM", 0);
  this->declare_parameter<int>("MAX_PIX_NUM", 0);
  this->declare_parameter<double>("MIN_ASPECT_RATIO", 0);
  this->declare_parameter<double>("MAX_ASPECT_RATIO", 0);
  this->declare_parameter<int>("YELLOW_PIX_TH", 0);
  this->declare_parameter<double>("MIN_YELLOW_ASPECT_RATIO", 0);
  this->declare_parameter<double>("MAX_YELLOW_ASPECT_RATIO", 0);

  /*** get_parameter ***/
  this->get_parameter("IMG_TOP_EDGE_RATIO", IMG_TOP_EDGE_RATIO);
  this->get_parameter("IMG_BOTTOM_EDGE_RATIO", IMG_BOTTOM_EDGE_RATIO);
  this->get_parameter("IMG_LEFT_EDGE_RATIO", IMG_LEFT_EDGE_RATIO);
  this->get_parameter("IMG_RIGHT_EDGE_RATIO", IMG_RIGHT_EDGE_RATIO);

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

  this->get_parameter("RED_IMAGE_THRESH", RED_IMAGE_THRESH);
  this->get_parameter("GREEN_IMAGE_THRESH", GREEN_IMAGE_THRESH);
  this->get_parameter("MIN_PIX_NUM", MIN_PIX_NUM);
  this->get_parameter("MAX_PIX_NUM", MAX_PIX_NUM);
  this->get_parameter("MIN_ASPECT_RATIO", MIN_ASPECT_RATIO);
  this->get_parameter("MAX_ASPECT_RATIO", MAX_ASPECT_RATIO);
  this->get_parameter("YELLOW_PIX_TH", YELLOW_PIX_TH);
  this->get_parameter("MIN_YELLOW_ASPECT_RATIO", MIN_YELLOW_ASPECT_RATIO);
  this->get_parameter("MAX_YELLOW_ASPECT_RATIO", MAX_YELLOW_ASPECT_RATIO);
}

/* カメラ画像から赤色の画素を抽出する関数 */
void Recognition::extractRedSignal(Mat &rgb, Mat &hsv, Mat &extract_red)
{
  // 赤信号の赤色を抽出
  for(int y = 0; y < rgb.rows; y++){
    for(int x = 0; x < rgb.cols; x++){
      cv::Vec3b val = hsv.at<cv::Vec3b>(y, x);
      if(  ((MIN_H_RED_01 <= val[0] && val[0] <= MAX_H_RED_01) || 
          (MIN_H_RED_02 <= val[0] && val[0] <= MAX_H_RED_02))
        && MIN_S_RED <= val[1] && val[1] <= MAX_S_RED
        && MIN_V_RED <= val[2] && val[2] <= MAX_V_RED)
      {
        extract_red.at<cv::Vec3b>(y, x) = rgb.at<cv::Vec3b>(y, x);
      }
    }
  }
}

/***  カメラ画像から緑色の画素を抽出する関数 ***/
void Recognition::extractGreenSignal(cv::Mat &rgb, cv::Mat &hsv, cv::Mat &extract_green)
{
  // 青信号の緑色を抽出
  for(int y = 0; y < rgb.rows; y++){
    for(int x = 0; x < rgb.cols; x++){
      cv::Vec3b val = hsv.at<cv::Vec3b>(y, x);
      if(    MIN_H_GREEN <= val[0] && val[0] <= MAX_H_GREEN
          && MIN_S_GREEN <= val[1] && val[1] <= MAX_S_GREEN
          && MIN_V_GREEN <= val[2] && val[2] <= MAX_V_GREEN){
          extract_green.at<cv::Vec3b>(y, x) = rgb.at<cv::Vec3b>(y, x);
      }
    }
  }
}

/* 抽出した色を白くし、二値化する関数 */
void Recognition::binalizeImage(Mat &src, Mat &gray_img)
{
  for(int y = 0; y<src.rows; y++){
    for(int x = 0; x<src.cols; x++){
      if(src.at<cv::Vec3b>(y, x)!=cv::Vec3b(0, 0, 0)){
        gray_img.at<uchar>(y, x) = 255;
      }
    }
  }
}

void Recognition::createCandidateArea(Mat &camera, const Mat &stats, vector<int> &left, vector<int> &top, vector<int> &width, vector<int> &height, int num_labels, string color_type)
{
  for (int label = 1; label < num_labels; label++)
  {
    top[label] = stats.at<int>(label, cv::CC_STAT_TOP);
    left[label] = stats.at<int>(label, cv::CC_STAT_LEFT);
    width[label] = stats.at<int>(label, cv::CC_STAT_WIDTH);
    height[label] = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    
    // ピンク色の矩形を描く
    // ピクセル数とアスペクト比を見る
    pixel_num = width[label] * height[label];
    aspect_ratio = ((double)width[label])/((double)height[label]);
    if(pixel_num<MIN_PIX_NUM || pixel_num>MAX_PIX_NUM)
    {
      continue;
    }
    if(aspect_ratio<MIN_ASPECT_RATIO || aspect_ratio>MAX_ASPECT_RATIO)
    {
      continue;
    }
    if(color_type == "red"){
      cv::rectangle(camera, cv::Rect(left[label] + img_left, top[label] + img_top, width[label], height[label]), cv::Scalar(255, 0, 255),2);
      cv::rectangle(extract_red, cv::Rect(left[label] + img_left, top[label] + img_top, width[label], height[label]), cv::Scalar(255, 0, 255),2);
    }
    else if(color_type == "green"){
      cv::rectangle(camera, cv::Rect(left[label] + img_left, top[label] + img_top, width[label], height[label]), cv::Scalar(255, 255, 0),2);
      cv::rectangle(extract_green, cv::Rect(left[label] + img_left, top[label] + img_top, width[label], height[label]), cv::Scalar(255, 255, 0),2);
    }
  }
}

void Recognition::extractYellowInBlob(Mat &rgb, int num_labels, const vector<int> &widths, const vector<int> &heights, const vector<int> &lefts, const vector<int> &tops, bool isRedSignal, Mat &top_region
                      , int img_left, int img_top)
{
  Mat top_region_hsv;
  cv::cvtColor(top_region, top_region_hsv, cv::COLOR_BGR2HSV);
  for (int label = 1; label < num_labels; label++)
  {
    int left = lefts[label];
    int top = tops[label];
    int width = widths[label];
    int height = heights[label];

    // ピクセル数とアスペクト比を見る
    pixel_num = height * width;
    aspect_ratio = ((double)width)/((double)height);
    if(pixel_num<MIN_PIX_NUM || pixel_num>MAX_PIX_NUM) continue;
    if(aspect_ratio<MIN_ASPECT_RATIO || aspect_ratio>MAX_ASPECT_RATIO) continue;
    // 黄色画素抽出のループ
    cv::Mat blob_rgb(top_region,cv::Rect(left, top, width, height));
    cv::Mat blob_hsv(top_region_hsv, cv::Rect(left, top, width, height));

    cv::Mat extract_yellow = cv::Mat::zeros(blob_hsv.size(), blob_hsv.type());
    cv::medianBlur(blob_hsv, blob_hsv, 3);
    int yellow_pix_cnt = 0;

    for (int y = 0; y < blob_hsv.rows; y++){
      for (int x = 0; x < blob_hsv.cols; x++){
        cv::Vec3b val = blob_hsv.at<cv::Vec3b>(y, x);
        if (   MIN_H_YELLOW <= val[0] && val[0] <= MAX_H_YELLOW
            && MIN_S_YELLOW <= val[1] && val[1] <= MAX_S_YELLOW
            && MIN_V_YELLOW <= val[2] && val[2] <= MAX_V_YELLOW){
          extract_yellow.at<cv::Vec3b>(y, x) = blob_rgb.at<cv::Vec3b>(y, x);
          yellow_pix_cnt++;
        }
      }
    }

    cv::Mat bin_img_yellow = cv::Mat::zeros(blob_hsv.size(), CV_8UC1);
    binalizeImage(extract_yellow, bin_img_yellow);

    cv::Mat labeled_yellow, stats_yellow, centroids_yellow;
    int num_labels_yellow = cv::connectedComponentsWithStats(bin_img_yellow, labeled_yellow, stats_yellow, centroids_yellow);
    for (int label = 1; label < num_labels_yellow; label++){
      int yellow_width = stats_yellow.at<int>(label, cv::CC_STAT_WIDTH);
      int yellow_height = stats_yellow.at<int>(label, cv::CC_STAT_HEIGHT);
      // int yellow_left = stats_yellow.at<int>(label, cv::CC_STAT_LEFT);
      // int yellow_top = stats_yellow.at<int>(label, cv::CC_STAT_TOP);
      aspect_ratio_yellow = (double)yellow_width / (double)yellow_height;

      // cv::rectangle(bin_img_yellow, cv::Rect(yellow_left, yellow_top, yellow_width, yellow_height), cv::Scalar(256/2), 2);
      // cv::imshow("bin_img_yellow", bin_img_yellow);
      if (isRedSignal){
        // cout << "yellow_pix_cnt[red]["<< label << "]: " << yellow_pix_cnt << endl;
        // cout << "aspect_ratio_yellow[red]["<< label << "]: "<< aspect_ratio_yellow << endl;
        if(num_labels_yellow > 1 && yellow_pix_cnt >= YELLOW_PIX_TH && aspect_ratio_yellow >= MIN_YELLOW_ASPECT_RATIO && aspect_ratio_yellow <= MAX_YELLOW_ASPECT_RATIO){
          cv::rectangle(rgb, cv::Rect(left + img_left, top + img_top, width, height), cv::Scalar(0, 0, 255), 2); // 赤信号は赤い矩形
          // cv::rectangle(top_region, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 2);
          light_msg_.x = left + img_left;
          light_msg_.y = top + img_top;
          red_light_flag = true;
        }
      }
      else{
        // cout << "yellow_pix_cnt[green]["<< label << "]: " << yellow_pix_cnt << endl;
        // cout << "aspect_ratio_yellow[green]["<< label << "]: "<< aspect_ratio_yellow << endl;
        if(num_labels_yellow > 1 && yellow_pix_cnt >= YELLOW_PIX_TH && aspect_ratio_yellow >= MIN_YELLOW_ASPECT_RATIO && aspect_ratio_yellow <= MAX_YELLOW_ASPECT_RATIO)
        {
          cv::rectangle(rgb, cv::Rect(left + img_left, top + img_top, width, height), cv::Scalar(255, 0, 0), 2); // 青信号は青い矩形
          // cv::rectangle(top_region, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 2);
          light_msg_.x = left + img_left;
          light_msg_.y = top + img_top;
          green_light_flag = true;
        }
      }
      // cout << " " << endl;
    }
  }
}

/*** カメラサイズを1280x720 pxの場合 ***/
void Recognition::drawOverlay(cv::Mat &image, bool red_light_flag, bool green_light_flag) 
{
  cv::Scalar color;
  if (red_light_flag) {
    color = cv::Scalar(0, 0, 255); // 赤色
    cv::rectangle(image, cv::Rect(10, 580, 60, 60), color, -1);
  } 
  if(green_light_flag) {
    color = cv::Scalar(255, 0, 0); // 青色
    cv::rectangle(image, cv::Rect(10, 650, 60, 60), color, -1);
  }
}

/*** カメラサイズが1280x720 pxの場合 ***/
void Recognition::addTextToImage(cv::Mat &image, const TrafficSignal &light_state)
{
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 2;
  int thickness = 5;
  cv::Point textOrg(0, 0);
  if(light_state.state=="RedLight"){
    cv::Point textOrg(80, 630); // 文字列を表示する位置
    cv::putText(image, light_state.state, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
  } else if(light_state.state=="GreenLight"){
    cv::Point textOrg(80, 700); // 文字列を表示する位置
    cv::putText(image, light_state.state, textOrg, fontFace, fontScale, cv::Scalar(255, 0, 0), thickness);
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

void Recognition::SignalImagePublisher(Mat &camera_img)
{
  auto ros_img = std::make_unique<Image>();
  cvImageToROSImage(camera_img, *ros_img);
  /*** メッセージのパブリッシュ ***/
  image_pub_->publish(std::move(ros_img));
}

void Recognition::run(Mat &camera_img)
{
  img_left = camera_img.size().width * IMG_LEFT_EDGE_RATIO;
  img_top = camera_img.size().height * IMG_TOP_EDGE_RATIO;
  img_width = camera_img.size().width * (IMG_RIGHT_EDGE_RATIO - IMG_LEFT_EDGE_RATIO);
  img_height = camera_img.size().height * (IMG_BOTTOM_EDGE_RATIO- IMG_TOP_EDGE_RATIO);
  // 入力画像のうち、画像の信号認識を行う範囲領域
  top_region = camera_img(cv::Rect(img_left, img_top, img_width, img_height));
  cv::cvtColor(top_region, hsv, cv::COLOR_BGR2HSV);
  // 赤色、緑色を抽出
  extract_red = Mat::zeros(top_region.size(), top_region.type());
  extract_green = Mat::zeros(top_region.size(), top_region.type());
  extractRedSignal(top_region, hsv, extract_red);
  extractGreenSignal(top_region, hsv, extract_green);
  // 二値化
  bin_img_red = Mat::zeros(top_region.size(), CV_8UC1);
  bin_img_green = Mat::zeros(top_region.size(), CV_8UC1);
  binalizeImage(extract_red, bin_img_red);
  binalizeImage(extract_green, bin_img_green);
  // メディアンフィルター
  red_median = (top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  green_median = (top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  cv::medianBlur(bin_img_red, red_median, 3);
  cv::medianBlur(bin_img_green, green_median, 3);
  // 収縮処理
  kernel_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  red_erode = cv::Mat::zeros(bin_img_red.size(), bin_img_red.type());
  green_erode = cv::Mat::zeros(bin_img_green.size(), bin_img_green.type());
  cv::erode(red_median, red_erode, kernel_erode);
  cv::erode(green_median, green_erode, kernel_erode);
  // 膨張処理
  kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
  red_dilate = cv::Mat::zeros(bin_img_red.size(), bin_img_red.type());
  green_dilate = cv::Mat::zeros(bin_img_green.size(), bin_img_green.type());
  cv::dilate(red_erode, red_dilate, kernel_dilate);
  cv::dilate(green_erode, green_dilate, kernel_dilate);
  // ラベリング
  num_labels_red = cv::connectedComponentsWithStats(red_dilate, labeled_red, stats_red, centroids_red);
  std::vector<int> red_width(num_labels_red), red_height(num_labels_red), red_left(num_labels_red), red_top(num_labels_red);
  color_type = "red";
  createCandidateArea(camera_img, stats_red, red_left, red_top, red_width, red_height, num_labels_red, color_type);
  num_labels_green = cv::connectedComponentsWithStats(green_dilate, labeled_green, stats_green, centroids_green);
  std::vector<int> green_width(num_labels_green), green_height(num_labels_green), green_left(num_labels_green), green_top(num_labels_green);
  color_type = "green";
  createCandidateArea(camera_img, stats_green, green_left, green_top, green_width, green_height, num_labels_green, color_type);
  // 信号の色判定
  extractYellowInBlob(camera_img, num_labels_red, red_width, red_height, red_left, red_top, true, top_region, img_left, img_top);
  extractYellowInBlob(camera_img, num_labels_green, green_width, green_height, green_left, green_top, false, top_region, img_left, img_top);
  if(red_light_flag) red_cnt++;
  else red_cnt = 0;
  if(green_light_flag) green_cnt++;
  else green_cnt = 0;
  if(red_cnt>RED_IMAGE_THRESH){
    drawOverlay(camera_img, red_light_flag, green_light_flag);
    light_msg_.state = "RedLight";
    addTextToImage(camera_img, light_msg_);
    light_pub_->publish(light_msg_);
    red_cnt=0;
  }
  if(green_cnt>GREEN_IMAGE_THRESH){
    drawOverlay(camera_img, red_light_flag, green_light_flag);
    light_msg_.state = "GreenLight";
    addTextToImage(camera_img, light_msg_);
    light_pub_->publish(light_msg_);
    green_cnt = 0;
  }
  red_light_flag = false;
  green_light_flag = false;
  SignalImagePublisher(camera_img);
}

/*** 画像をサブスクライブすると呼び出されるコールバック関数 ***/
void Recognition::onImageSubscribed(Image::SharedPtr img)
{
  g_mutex_.lock();
  const auto &sub_img = img;
  g_mutex_.unlock();
  /*** メッセージのポインタを確認 ***/
  // RCLCPP_INFO(this->get_logger(), "%0lx", reinterpret_cast<std::uintptr_t>(sub_img.get()));
  /*** ROS2のImage型 -> OpenCVのMat型への変換 ***/
  auto cv_img = cv_bridge::toCvShare(sub_img, sub_img->encoding);
  /*** Mat型が空の場合コールバック関数を抜ける ***/
  if(cv_img->image.empty()) return;
  camera_img = cv_img->image;
  /*** 信号認識処理 ***/
  run(camera_img);
  // cv::imshow("camera_img", camera_img);
  // cv::imshow("bin red", bin_img_red);
  // cv::imshow("bin_green", bin_img_green);
  // cv::imshow("red_median", red_median);
  // cv::imshow("green median", green_median);
  // cv::imshow("red_erode", red_erode);
  // cv::imshow("green erode", green_erode);
  // cv::imshow("red dilate", red_dilate);
  // cv::imshow("green_dilate", green_dilate);
  // cv::waitKey(1);
}

/*** Recognitionクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::Recognition)