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
constexpr int ENCODER_RESOLUTION = 4096;

// 車輪のパラメーター
// 単位はmm
constexpr double SQRT_3 = 1.73205080757;
constexpr millimeter_t TREAD_RADIUS = 186.5_mm;
constexpr millimeter_t WHEEL_RADIUS = 30_mm;

// Front
constexpr millimeter_t front_x = TREAD_RADIUS;
constexpr millimeter_t front_y = 0_mm;
constexpr radian_t front_theta = 90_deg;
constexpr millimeter_t front_radius = WHEEL_RADIUS;

// Rear Left
constexpr millimeter_t rear_left_x = -TREAD_RADIUS / 2.0;
constexpr millimeter_t rear_left_y = TREAD_RADIUS * SQRT_3 / 2.0;
constexpr radian_t rear_left_theta = 90_deg + 120_deg;
constexpr millimeter_t rear_left_radius = WHEEL_RADIUS;
;

// Rear Right
constexpr millimeter_t rear_right_x = -TREAD_RADIUS / 2.0;
constexpr millimeter_t rear_right_y = -TREAD_RADIUS * SQRT_3 / 2.0;
constexpr radian_t rear_right_theta = 90_deg + 240_deg;
constexpr millimeter_t rear_right_radius = WHEEL_RADIUS;

constexpr std::array<WheelConfig, N> WHEEL_CONFIGS = {
    WheelConfig{front_x, front_y, front_theta, front_radius},
    WheelConfig{rear_left_x, rear_left_y, rear_left_theta, rear_left_radius},
    WheelConfig{rear_right_x, rear_right_y, rear_right_theta,
                rear_right_radius}};

constexpr int FREQUENCY = 30;

namespace NodeName {
constexpr char ENCODER_LOCALIZATION[] = "encoder_localization";
}

namespace TopicName {
constexpr char ENCODER_POSE[] = "/encoder_pose";
constexpr char ENCODER_COUNTS[] = "/encoder_counts";
}  // namespace TopicName

class EncoderLocalizationNode : public rclcpp::Node {
 public:
  EncoderLocalizationNode() : Node(NodeName::ENCODER_LOCALIZATION) {
    last_time_ = this->get_clock()->now();

    odometry_ =
        std::make_unique<Odometry<N>>(WHEEL_CONFIGS, ENCODER_RESOLUTION);

    if (FREQUENCY <= 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid frequency parameter! expected > 0, actual: %d",
                   FREQUENCY);
      exit(1);
    }
    min_dt_ = 1.0 / static_cast<double>(FREQUENCY);

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
