#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <filesystem> // C++17の機能を使う

namespace crosswalk_signal {

class LogImgPublisher : public rclcpp::Node
{
public:
  LogImgPublisher(const rclcpp::NodeOptions & options) : Node("log_img_pub", options), current_image_idx_(0)
  {
    // パラメータの宣言とフォルダパスの取得
    folder_path_ = this->declare_parameter<std::string>("folder_path", "");
    if (folder_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No folder path provided in YAML file.");
      return;
    }

    load_image_files();
    if (image_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No image files found in the specified folder.");
      return;
    }

    // イメージをパブリッシュするためのパブリッシャー
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image", 10);

    // 初回起動時に最初の画像を表示・パブリッシュ
    publish_image();

    // キーボード入力を監視するためのタイマー
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LogImgPublisher::timer_callback, this));
  }

private:
  void load_image_files()
  {
    // フォルダ内のすべての画像ファイルを取得
    for (const auto& entry : std::filesystem::directory_iterator(folder_path_)) {
      if (entry.is_regular_file()) {
        std::string file_path = entry.path().string();
        if (is_image_file(file_path)) {
          image_files_.push_back(file_path);
        }
      }
    }

    if (image_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No image files found in the specified folder.");
    }
  }

  bool is_image_file(const std::string& file_path)
  {
    // 拡張子をチェックして、画像ファイルかどうかを確認
    std::vector<std::string> valid_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"};
    std::string extension = file_path.substr(file_path.find_last_of("."));
    for (const auto& ext : valid_extensions) {
      if (extension == ext) {
        return true;
      }
    }
    return false;
  }

  // OpenCVのMat型からROS2のImage型へ変換するための関数
  void cvImage2ROSImage(const cv::Mat &src, sensor_msgs::msg::Image &dst)
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

  void publish_image()
  {
    if (current_image_idx_ >= image_files_.size()) {
      RCLCPP_WARN(this->get_logger(), "No more images to publish.");
      return;
    }

    // 画像を読み込む
    cv::Mat image = cv::imread(image_files_[current_image_idx_]);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_files_[current_image_idx_].c_str());
      return;
    }

    // OpenCVのウィンドウで画像を表示
    cv::imshow("Image Display", image);
    cv::waitKey(30); // 30msの待機

    // OpenCVのMat型からROS2のImage型に変換
    sensor_msgs::msg::Image img_msg;
    cvImage2ROSImage(image, img_msg);

    // 画像をパブリッシュ
    publisher_->publish(img_msg);

    RCLCPP_INFO(this->get_logger(), "Published image: %s", image_files_[current_image_idx_].c_str());
  }

  void next_image()
  {
    current_image_idx_ = (current_image_idx_ + 1) % image_files_.size();
    publish_image();
  }

  void previous_image()
  {
    current_image_idx_ = (current_image_idx_ == 0) ? image_files_.size() - 1 : current_image_idx_ - 1;
    publish_image();
  }

  void timer_callback()
  {
    // キーボード入力をチェックし、次/前の画像を切り替える
    int key = cv::waitKey(30);
    if (key == 'd') {
      next_image();
    } else if (key == 'a') {
      previous_image();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string folder_path_;
  std::vector<std::string> image_files_;
  size_t current_image_idx_;
};

} // namespace crosswalk_signal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crosswalk_signal::LogImgPublisher)
