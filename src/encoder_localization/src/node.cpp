#include <units.h>

#include <array>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tutorial_interfaces/msg/encoder_counts.hpp>

#include "odometry.hpp"
#include "wheel_config.hpp"

using namespace units::literals;
using EncoderCounts = tutorial_interfaces::msg::EncoderCounts;

// 測定輪の個数
constexpr int N = 3;
constexpr int CONFIG_SIZE_PER_WHEEL = 4;

namespace NodeName {
constexpr char ENCODER_LOCALIZATION[] = "encoder_localization";
}

namespace TopicName {
constexpr char ENCODER_POSE[] = "/encoder_pose";
constexpr char ENCODER_COUNTS[] = "/encoder_counts";
}  // namespace TopicName

namespace Parameters {
constexpr char ENCODER_RESOLUTION[] = "encoder_resolution";
constexpr char WHEEL_CONFIGS[] = "wheel_configs";
constexpr char FREQUENCY[] = "frequency";
}  // namespace Parameters

class EncoderLocalizationNode : public rclcpp::Node {
 public:
  EncoderLocalizationNode() : Node(NodeName::ENCODER_LOCALIZATION) {
    last_time_ = this->get_clock()->now();

    this->declare_parameter(Parameters::ENCODER_RESOLUTION, 4096);
    int encoder_resolution =
        this->get_parameter(Parameters::ENCODER_RESOLUTION).as_int();

    this->declare_parameter(Parameters::WHEEL_CONFIGS, std::vector<double>());
    std::vector<double> config_vector =
        this->get_parameter(Parameters::WHEEL_CONFIGS).as_double_array();

    // パラメータの要素数が合わない場合は強制終了(必要な要素数は4N)
    if (config_vector.size() != N * CONFIG_SIZE_PER_WHEEL) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid wheel config size! expected: %d, actual: %zu",
                   N * CONFIG_SIZE_PER_WHEEL, config_vector.size());
      exit(1);
    }

    std::array<WheelConfig, N> wheel_configs;
    for (int i = 0; i < N; i++) {
      int base_i = i * CONFIG_SIZE_PER_WHEEL;
      wheel_configs[i].x = millimeter_t(config_vector[base_i + 0]);
      wheel_configs[i].y = millimeter_t(config_vector[base_i + 1]);
      wheel_configs[i].theta = radian_t(config_vector[base_i + 2]);
      wheel_configs[i].radius = millimeter_t(config_vector[base_i + 3]);
    }

    odometry_ =
        std::make_unique<Odometry<N>>(wheel_configs, encoder_resolution);

    this->declare_parameter(Parameters::FREQUENCY, 30);
    const int frequency = this->get_parameter(Parameters::FREQUENCY).as_int();
    if (frequency <= 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid frequency parameter! expected > 0, actual: %d",
                   frequency);
      exit(1);
    }
    min_dt_ = 1.0 / static_cast<double>(frequency);

    encoder_counts_sub_ = this->create_subscription<EncoderCounts>(
        TopicName::ENCODER_COUNTS, 10,
        [this](const EncoderCounts::ConstSharedPtr msg) {
          this->callback(msg);
        });

    encoder_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        TopicName::ENCODER_POSE, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Odometry Node has started.");
  }

 private:
  rclcpp::Time last_time_;
  double min_dt_;

  std::unique_ptr<Odometry<N>> odometry_;
  rclcpp::Subscription<EncoderCounts>::SharedPtr encoder_counts_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoder_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void callback(const EncoderCounts::ConstSharedPtr msg) {
    std::array<int, N> encoder_counts = {
        msg->front_counts, msg->rear_left_counts, msg->rear_right_counts};

    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Duration dt = current_time - last_time_;

    if (dt.seconds() < min_dt_) {
      return;
    }

    auto [pose, twist] = odometry_->update(encoder_counts, dt);

    RCLCPP_INFO(this->get_logger(), " pose: %lf, %lf, %lf",
                pose.pose.position.x, pose.pose.position.y,
                tf2::getYaw(pose.pose.orientation));
    RCLCPP_INFO(this->get_logger(), "twist: %lf, %lf, %lf, dt: %f",
                twist.twist.linear.x, twist.twist.linear.y,
                twist.twist.angular.z, dt.seconds());

    publish(pose, twist, current_time);
    last_time_ = current_time;
  }

  void publish(PoseWithCovariance pose, TwistWithCovariance twist,
               rclcpp::Time current_time) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose = pose;
    odom_msg.twist = twist;

    encoder_pose_pub_->publish(odom_msg);

    tf2::Transform odom_tf;
    odom_tf.setOrigin(
        tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));
    tf2::Quaternion q_odom(pose.pose.orientation.x, pose.pose.orientation.y,
                           pose.pose.orientation.z, pose.pose.orientation.w);
    odom_tf.setRotation(q_odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom_tf.getOrigin().x();
    tf.transform.translation.y = odom_tf.getOrigin().y();
    tf.transform.translation.z = odom_tf.getOrigin().z();
    tf.transform.rotation = tf2::toMsg(odom_tf.getRotation());

    tf_broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
