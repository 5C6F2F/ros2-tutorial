// 測定輪の累積カウントからオドメトリを計算するノード
//
// 受信周期ではなくメッセージの時刻を使うので、bag再生でも同じ結果になる。
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omni_kinematics/omni_kinematics.hpp"
#include "omni_msgs/msg/encoder_counts.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class EncoderOdometry : public rclcpp::Node
{
public:
  EncoderOdometry()
  : Node("encoder_odometry")
  {
    declare_parameter("wheel_radius", 0.025);
    declare_parameter("base_radius", 0.150);
    declare_parameter("counts_per_rev", 8192);
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);
    // 速度の分散 [vx, vy, vyaw]。EKFが他のセンサーとの重みを決めるのに使う
    declare_parameter("twist_covariance_diagonal", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-2});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // QoSはYAMLの qos_overrides で起動時に変更できる
    rclcpp::SubscriptionOptions counts_options;
    counts_options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    counts_sub_ = create_subscription<omni_msgs::msg::EncoderCounts>(
      "encoder_counts", rclcpp::QoS(10),
      [this](const omni_msgs::msg::EncoderCounts::ConstSharedPtr msg) {
        counts_callback(*msg);
      },
      counts_options);
  }

private:
  void counts_callback(const omni_msgs::msg::EncoderCounts & msg)
  {
    // 途中から起動しても位置が飛ばないよう、最初のカウントは基準として保存する
    if (!last_counts_) {
      last_counts_ = msg;
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    const double dt = (stamp - rclcpp::Time(last_counts_->header.stamp)).seconds();
    const omni_kinematics::BodyMotion delta = body_delta(msg);
    last_counts_ = msg;

    pose_ = omni_kinematics::integrate_midpoint(pose_, delta);
    publish_odometry(stamp, delta, dt);
    if (get_parameter("publish_tf").as_bool()) {
      publish_transform(stamp);
    }
  }

  // カウントの差分 → 測定輪の変位 → 機体の変位
  omni_kinematics::BodyMotion body_delta(const omni_msgs::msg::EncoderCounts & msg) const
  {
    const double wheel_radius = get_parameter("wheel_radius").as_double();
    const double base_radius = get_parameter("base_radius").as_double();
    const int counts_per_rev = get_parameter("counts_per_rev").as_int();
    const double meters_per_count = 2.0 * M_PI * wheel_radius / counts_per_rev;

    const omni_kinematics::WheelMotion wheels{
      (msg.front - last_counts_->front) * meters_per_count,
      (msg.rear_left - last_counts_->rear_left) * meters_per_count,
      (msg.rear_right - last_counts_->rear_right) * meters_per_count};
    return omni_kinematics::wheels_to_body(wheels, base_radius);
  }

  void publish_odometry(
    const rclcpp::Time & stamp, const omni_kinematics::BodyMotion & delta, const double dt)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = get_parameter("odom_frame").as_string();
    odom.child_frame_id = get_parameter("base_frame").as_string();
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.orientation.z = std::sin(pose_.yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(pose_.yaw / 2.0);
    if (dt > 0.0) {
      odom.twist.twist.linear.x = delta.x / dt;
      odom.twist.twist.linear.y = delta.y / dt;
      odom.twist.twist.angular.z = delta.yaw / dt;
    }
    const auto variance = get_parameter("twist_covariance_diagonal").as_double_array();
    odom.twist.covariance[0] = variance.at(0);    // vx
    odom.twist.covariance[7] = variance.at(1);    // vy
    odom.twist.covariance[35] = variance.at(2);   // vyaw
    odom_pub_->publish(odom);
  }

  void publish_transform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = get_parameter("odom_frame").as_string();
    transform.child_frame_id = get_parameter("base_frame").as_string();
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.rotation.z = std::sin(pose_.yaw / 2.0);
    transform.transform.rotation.w = std::cos(pose_.yaw / 2.0);
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<omni_msgs::msg::EncoderCounts>::SharedPtr counts_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<omni_msgs::msg::EncoderCounts> last_counts_;
  omni_kinematics::Pose2D pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdometry>());
  rclcpp::shutdown();
  return 0;
}
