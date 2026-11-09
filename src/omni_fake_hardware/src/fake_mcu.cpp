// 実機のマイコン、モーター、測定輪の代役となるノード
//
// cmd_velに従って機体の真の姿勢を動かし、その動きから測定輪の累積カウントを作る。
// 有効にすると、滑りの影響を受けないジャイロの角速度も出力する。
// STM32のタイマー割り込みに相当する処理は timer_callback() にある。

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "omni_kinematics/omni_kinematics.hpp"
#include "omni_msgs/msg/encoder_counts.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class FakeMcu : public rclcpp::Node {
 public:
  FakeMcu() : Node("fake_mcu"), random_engine_(std::random_device{}()) {
    declare_parameter("wheel_radius", 0.025);
    declare_parameter("base_radius", 0.150);
    declare_parameter("counts_per_rev", 8192);
    declare_parameter("publish_rate", 100.0);
    declare_parameter("cmd_vel_timeout", 1.0);
    declare_parameter("slip_ratio", 0.0);
    declare_parameter("slip_wheel_bias", std::vector<double>{1.0, 0.6, 1.4});
    declare_parameter("slip_noise_std", 0.0);
    declare_parameter("publish_base_link_true", false);
    declare_parameter("base_link_true_parent_frame", "odom");
    declare_parameter("drop_ratio", 0.0);
    declare_parameter("enable_imu", false);
    declare_parameter("imu_frame", "base_link");
    declare_parameter("imu_gyro_noise_std", 0.01);

    // QoSはYAMLの qos_overrides で起動時に変更できる
    rclcpp::PublisherOptions counts_options;
    counts_options.qos_overriding_options = rclcpp::QosOverridingOptions{
        rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    counts_pub_ = create_publisher<omni_msgs::msg::EncoderCounts>(
        "encoder_counts", rclcpp::QoS(10), counts_options);
    if (get_parameter("enable_imu").as_bool()) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    }
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          cmd_vel_ = *msg;
          last_cmd_vel_time_ = now();
        });
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_step_time_ = now();
    last_cmd_vel_time_ = last_step_time_;
    const auto period = std::chrono::duration<double>(
        1.0 / get_parameter("publish_rate").as_double());
    timer_ = create_timer(period, [this]() { timer_callback(); });
  }

 private:
  void timer_callback() {
    const rclcpp::Time stamp = now();
    const double dt = (stamp - last_step_time_).seconds();
    last_step_time_ = stamp;

    const omni_kinematics::BodyMotion delta = body_delta(stamp, dt);
    true_pose_ = omni_kinematics::integrate_midpoint(true_pose_, delta);
    add_wheel_travel(delta);

    publish_counts(stamp);
    if (imu_pub_ && dt > 0.0) {
      publish_imu(stamp, delta.yaw / dt);
    }
    if (get_parameter("publish_base_link_true").as_bool()) {
      publish_true_pose(stamp);
    }
  }

  // 指令が途絶えたら、実機の安全停止と同じく機体を止める
  omni_kinematics::BodyMotion body_delta(const rclcpp::Time& stamp,
                                         const double dt) const {
    const double timeout = get_parameter("cmd_vel_timeout").as_double();
    if ((stamp - last_cmd_vel_time_).seconds() > timeout) {
      return omni_kinematics::BodyMotion{};
    }
    return omni_kinematics::BodyMotion{cmd_vel_.linear.x * dt,
                                       cmd_vel_.linear.y * dt,
                                       cmd_vel_.angular.z * dt};
  }

  void add_wheel_travel(const omni_kinematics::BodyMotion& delta) {
    const double base_radius = get_parameter("base_radius").as_double();
    const auto wheels = omni_kinematics::body_to_wheels(delta, base_radius);
    const auto bias = get_parameter("slip_wheel_bias").as_double_array();

    wheel_travel_.front += wheels.front * slip_scale(bias.at(0));
    wheel_travel_.rear_left += wheels.rear_left * slip_scale(bias.at(1));
    wheel_travel_.rear_right += wheels.rear_right * slip_scale(bias.at(2));
  }

  // 測定輪が地面に対して滑り、実際より短い距離しか測れない状況を表す
  double slip_scale(const double wheel_bias) {
    const double slip_ratio = get_parameter("slip_ratio").as_double();
    const double noise_std = get_parameter("slip_noise_std").as_double();
    const double noise = noise_std * standard_normal_(random_engine_);
    return (1.0 - slip_ratio * wheel_bias) * (1.0 + noise);
  }

  void publish_counts(const rclcpp::Time& stamp) {
    // 通信の欠落を再現する。測定輪の変位は wheel_travel_ に積算され続ける
    if (uniform_(random_engine_) < get_parameter("drop_ratio").as_double()) {
      return;
    }

    const double wheel_radius = get_parameter("wheel_radius").as_double();
    const int counts_per_rev = get_parameter("counts_per_rev").as_int();
    const double counts_per_meter =
        counts_per_rev / (2.0 * M_PI * wheel_radius);

    omni_msgs::msg::EncoderCounts msg;
    msg.header.stamp = stamp;
    msg.front = static_cast<int32_t>(
        std::llround(wheel_travel_.front * counts_per_meter));
    msg.rear_left = static_cast<int32_t>(
        std::llround(wheel_travel_.rear_left * counts_per_meter));
    msg.rear_right = static_cast<int32_t>(
        std::llround(wheel_travel_.rear_right * counts_per_meter));
    counts_pub_->publish(msg);
  }

  // ジャイロは機体の回転を直接測るので、測定輪の滑りの影響を受けない
  void publish_imu(const rclcpp::Time& stamp, const double true_yaw_rate) {
    const double noise_std = get_parameter("imu_gyro_noise_std").as_double();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = get_parameter("imu_frame").as_string();
    msg.angular_velocity.z =
        true_yaw_rate + noise_std * standard_normal_(random_engine_);
    msg.angular_velocity_covariance[8] = noise_std * noise_std;
    // 姿勢と加速度は出力しない。共分散の先頭を-1にして、値がないことを示す
    msg.orientation_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(msg);
  }

  // 真の姿勢は、走行を始めた位置を原点とする base_link_true_parent_frame
  // 上で表す
  void publish_true_pose(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id =
        get_parameter("base_link_true_parent_frame").as_string();
    transform.child_frame_id = "base_link_true";
    transform.transform.translation.x = true_pose_.x;
    transform.transform.translation.y = true_pose_.y;
    transform.transform.rotation.z = std::sin(true_pose_.yaw / 2.0);
    transform.transform.rotation.w = std::cos(true_pose_.yaw / 2.0);
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Publisher<omni_msgs::msg::EncoderCounts>::SharedPtr counts_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_step_time_;
  omni_kinematics::Pose2D true_pose_;
  omni_kinematics::WheelMotion wheel_travel_;  // 倍精度の累積変位 [m]

  std::mt19937 random_engine_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeMcu>());
  rclcpp::shutdown();
  return 0;
}
