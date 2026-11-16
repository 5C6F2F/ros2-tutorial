// LiDARの絶対位置からオドメトリのずれを求め、map → odom として発行するノード
//
// odom → base_link はなめらかなまま残し、ずれの補正だけを map → odom
// が受け持つ。
// LiDARの結果は遅れて届くので、計測した時刻のオドメトリと比べてずれを求める。

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

using geometry_msgs::msg::PoseWithCovarianceStamped;

class MapOdomCorrector : public rclcpp::Node {
 public:
  MapOdomCorrector()
      : Node("map_odom_corrector"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    // 周期は起動時にだけ読む。0にできないので範囲も指定する
    rcl_interfaces::msg::ParameterDescriptor rate_descriptor;
    rate_descriptor.read_only = true;
    rate_descriptor.floating_point_range.resize(1);
    rate_descriptor.floating_point_range[0].from_value = 1.0;
    rate_descriptor.floating_point_range[0].to_value = 1000.0;

    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_rate", 50.0, rate_descriptor);
    declare_parameter("correction_gain", 0.3);
    declare_parameter("max_measurement_age", 0.5);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    lidar_sub_ = create_subscription<PoseWithCovarianceStamped>(
        "lidar_pose", 10,
        [this](const PoseWithCovarianceStamped& msg) { pending_ = msg; });

    const auto period = std::chrono::duration<double>(
        1.0 / get_parameter("publish_rate").as_double());
    timer_ = create_timer(period, [this]() { timer_callback(); });
  }

 private:
  void timer_callback() {
    if (pending_) {
      apply_pending_measurement();
    }
    publish_map_to_odom();
  }

  // 計測時刻のオドメトリが届くまで待つ。待つ間もコールバックは止めない
  void apply_pending_measurement() {
    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string odom_frame = get_parameter("odom_frame").as_string();
    const std::string base_frame = get_parameter("base_frame").as_string();
    const rclcpp::Time stamp(pending_->header.stamp);

    if (pending_->header.frame_id != map_frame) {
      RCLCPP_WARN(get_logger(), "lidar_poseのフレームが %s ではありません: %s",
                  map_frame.c_str(), pending_->header.frame_id.c_str());
      pending_.reset();
      return;
    }
    if (!tf_buffer_.canTransform(odom_frame, base_frame, stamp)) {
      if ((now() - stamp).seconds() >
          get_parameter("max_measurement_age").as_double()) {
        RCLCPP_WARN(get_logger(),
                    "計測時刻のオドメトリがないため、lidar_poseを捨てました");
        pending_.reset();
      }
      return;
    }

    tf2::Transform odom_to_base;
    tf2::fromMsg(
        tf_buffer_.lookupTransform(odom_frame, base_frame, stamp).transform,
        odom_to_base);
    tf2::Transform map_to_base;
    tf2::fromMsg(pending_->pose.pose, map_to_base);
    pending_.reset();

    // map → base_link = (map → odom) * (odom → base_link) を map → odom
    // について解く
    const tf2::Transform measured = map_to_base * odom_to_base.inverse();
    // ノイズで機体が跳ねないよう、測定値へ少しずつ近づける
    const double gain = get_parameter("correction_gain").as_double();
    map_to_odom_.setOrigin(
        map_to_odom_.getOrigin().lerp(measured.getOrigin(), gain));
    map_to_odom_.setRotation(
        map_to_odom_.getRotation().slerp(measured.getRotation(), gain));
  }

  void publish_map_to_odom() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = get_parameter("map_frame").as_string();
    transform.child_frame_id = get_parameter("odom_frame").as_string();
    transform.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_->sendTransform(transform);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<PoseWithCovarianceStamped> pending_;
  // 起動時はmapとodomが一致する。tf2::Transformは既定では初期化されない
  tf2::Transform map_to_odom_{tf2::Transform::getIdentity()};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomCorrector>());
  rclcpp::shutdown();
  return 0;
}
