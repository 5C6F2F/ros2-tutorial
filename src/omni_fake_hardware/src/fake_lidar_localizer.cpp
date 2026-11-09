// LiDARによる自己位置推定の代役となるノード
//
// 真の姿勢にノイズを加え、map上の絶対位置として低い周期で出力する。
// 実機のスキャンマッチングと同じく、結果は計測した時刻より遅れて届く。

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_kinematics/pose_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class FakeLidarLocalizer : public rclcpp::Node {
 public:
  FakeLidarLocalizer()
      : Node("fake_lidar_localizer"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_),
        random_engine_(std::random_device{}()) {
    declare_parameter("map_frame", "map");
    declare_parameter("lidar_rate", 5.0);
    declare_parameter("lidar_latency", 0.10);
    declare_parameter("lidar_position_noise_std", 0.02);
    declare_parameter("lidar_yaw_noise_std", 0.02);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "lidar_pose", 10);
    const auto period = std::chrono::duration<double>(
        1.0 / get_parameter("lidar_rate").as_double());
    timer_ = create_timer(period, [this]() { timer_callback(); });
  }

 private:
  void timer_callback() {
    const std::string map_frame = get_parameter("map_frame").as_string();
    const rclcpp::Time measured_time =
        now() - rclcpp::Duration::from_seconds(
                    get_parameter("lidar_latency").as_double());

    geometry_msgs::msg::TransformStamped truth;
    try {
      truth = tf_buffer_.lookupTransform(map_frame, "base_link_true",
                                         measured_time);
    } catch (const tf2::TransformException& exception) {
      // 起動直後は真の姿勢がまだ届いていないので、しばらく出続けることがある
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "真の姿勢を取得できません: %s", exception.what());
      return;
    }

    const double position_std =
        get_parameter("lidar_position_noise_std").as_double();
    const double yaw_std = get_parameter("lidar_yaw_noise_std").as_double();
    omni_kinematics::Pose2D pose = omni_kinematics::to_pose2d(truth);
    pose.x += position_std * standard_normal_(random_engine_);
    pose.y += position_std * standard_normal_(random_engine_);
    pose.yaw = std::remainder(
        pose.yaw + yaw_std * standard_normal_(random_engine_), 2.0 * M_PI);

    // 時刻は、結果を送った時刻ではなく計測した時刻にする
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = measured_time;
    msg.header.frame_id = map_frame;
    msg.pose.pose = omni_kinematics::to_pose_msg(pose);
    msg.pose.covariance[0] = position_std * position_std;  // x
    msg.pose.covariance[7] = position_std * position_std;  // y
    msg.pose.covariance[35] = yaw_std * yaw_std;           // yaw
    pose_pub_->publish(msg);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 random_engine_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLidarLocalizer>());
  rclcpp::shutdown();
  return 0;
}
