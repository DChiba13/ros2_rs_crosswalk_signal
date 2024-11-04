#include "rclcpp/rclcpp.hpp"
#include <ros2_rs_interfaces/msg/traffic_signal.hpp> // TrafficSignalメッセージのヘッダーをインクルード
#include "std_msgs/msg/string.hpp"

using ros2_rs_interfaces::msg::TrafficSignal;

class LightSubscriber : public rclcpp::Node
{
public:
    // コンストラクタ
    LightSubscriber() : Node("test_light_sub")
    {
        // サブスクライバの初期化
        light_sub_ = this->create_subscription<TrafficSignal>(
            "/light_msg", 10,  // トピック名とキューサイズ
            std::bind(&LightSubscriber::onLightSignalReceived, this, std::placeholders::_1)  // コールバック関数
        );
    }

private:
    // コールバック関数
    void onLightSignalReceived(const TrafficSignal::SharedPtr msg)
    {
        std::cout << msg->state.c_str() << std::endl;
        // 受け取った信号を判定
        if (msg->state == "RedLight") {
            RCLCPP_INFO(this->get_logger(), "%s",msg->state.c_str());
            RCLCPP_INFO(this->get_logger(), "%d",msg->x);
            RCLCPP_INFO(this->get_logger(), "%d",msg->y);
        }
        if (msg->state == "GreenLight") {
            RCLCPP_INFO(this->get_logger(), "%s",msg->state.c_str());
            RCLCPP_INFO(this->get_logger(), "%d",msg->x);
            RCLCPP_INFO(this->get_logger(), "%d",msg->y);
        }
    }

    // サブスクライバ
    rclcpp::Subscription<TrafficSignal>::SharedPtr light_sub_;
};

int main(int argc, char *argv[])
{
    // ROS 2ノードの初期化
    rclcpp::init(argc, argv);

    // ノードの実行
    rclcpp::spin(std::make_shared<LightSubscriber>());

    // 終了処理
    rclcpp::shutdown();
    return 0;
}
