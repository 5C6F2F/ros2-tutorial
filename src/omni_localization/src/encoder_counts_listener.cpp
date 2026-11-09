// encoder_countsを受信して表示する、最小のSubscriberノード

#include <memory>

#include "omni_msgs/msg/encoder_counts.hpp"
#include "rclcpp/rclcpp.hpp"

class EncoderCountsListener : public rclcpp::Node {
 public:
  EncoderCountsListener() : Node("encoder_counts_listener") {
    counts_sub_ = create_subscription<omni_msgs::msg::EncoderCounts>(
        "encoder_counts", 10,
        [this](const omni_msgs::msg::EncoderCounts::ConstSharedPtr msg) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                               "front=%d rear_left=%d rear_right=%d",
                               msg->front, msg->rear_left, msg->rear_right);
        });
  }

 private:
  rclcpp::Subscription<omni_msgs::msg::EncoderCounts>::SharedPtr counts_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderCountsListener>());
  rclcpp::shutdown();
  return 0;
}
