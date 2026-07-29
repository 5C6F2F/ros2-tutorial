#pragma once

#include <Eigen/Dense>
#include <rclcpp/duration.hpp>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "units.hpp"

using PoseWithCovariance = geometry_msgs::msg::PoseWithCovariance;
using TwistWithCovariance = geometry_msgs::msg::TwistWithCovariance;

template <int N>
class Odometry {
  static_assert(N > 2, "N must be greater than 2.");

 public:
  Odometry(std::array<WheelConfig, N>& wheel_configs, int encoder_resolution)
      : encoder_resolution_(encoder_resolution) {
    wheel_matrix_inv_ = get_wheel_matrix_inv(wheel_configs);
    last_encoder_counts_.fill(0);
    position_ << 0.0, 0.0, 0.0;
  }

  std::pair<PoseWithCovariance, TwistWithCovariance> update(
      const std::array<int, N>& encoder_counts, rclcpp::Duration dt) {
    Eigen::Vector3d delta_local = calc_local_delta(encoder_counts);
    update_world_position(delta_local);

    PoseWithCovariance pose_msg;
    pose_msg.pose.position.x = position_.x();
    pose_msg.pose.position.y = position_.y();
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, position_.z());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    TwistWithCovariance twist_msg;
    double dt_sec = dt.seconds();
    if (dt_sec > 0) {
      twist_msg.twist.linear.x = delta_local.x() / dt_sec;
      twist_msg.twist.linear.y = delta_local.y() / dt_sec;
      twist_msg.twist.angular.z = delta_local.z() / dt_sec;
    }

    return {pose_msg, twist_msg};
  }

  void reset_pose() { position_ << 0.0, 0.0, 0.0; }

 private:
  Eigen::Vector3d position_;
  int encoder_resolution_;
  std::array<int, N> last_encoder_counts_;
  Eigen::Matrix<double, 3, N> wheel_matrix_inv_;

  Eigen::Matrix<double, 3, N> get_wheel_matrix_inv(
      const std::array<WheelConfig, N>& wheel_configs) {
    Eigen::Matrix<double, N, 3> wheel_matrix = getWheelMatrix(wheel_configs);

    // 逆行列 (N>=4のときMoore-Penroseの疑似逆行列)
    if constexpr (N == 3) {
      return wheel_matrix.inverse();
    } else {
      return (wheel_matrix.transpose() * wheel_matrix).inverse() *
             wheel_matrix.transpose();
    }
  }

  Eigen::Matrix<double, N, 3> getWheelMatrix(
      const std::array<WheelConfig, N>& wheel_configs) {
    Eigen::Matrix<double, N, 3> matrix;

    for (int i = 0; i < N; i++) {
      auto [x, y, theta, radius] = wheel_configs[i];

      matrix(i, 0) = cos(theta.value());
      matrix(i, 1) = sin(theta.value());
      matrix(i, 2) = (x.value() * matrix(i, 1) - y.value() * matrix(i, 0));

      double wheel_circumference = 2 * M_PI * radius.value();
      matrix.row(i) /= wheel_circumference;
    }

    return matrix;
  }

  Eigen::Vector3d calc_local_delta(const std::array<int, N>& encoder_counts) {
    Eigen::Vector<double, N> wheel_delta_vec;

    for (int i = 0; i < N; i++) {
      int count_diff = encoder_counts[i] - last_encoder_counts_[i];

      wheel_delta_vec(i) = static_cast<double>(count_diff) /
                           static_cast<double>(encoder_resolution_);

      last_encoder_counts_[i] = encoder_counts[i];
    }

    return wheel_matrix_inv_ * wheel_delta_vec;
  }

  void update_world_position(const Eigen::Vector3d& delta_local) {
    double delta_x = delta_local.x();
    double delta_y = delta_local.y();
    double d_theta = delta_local.z();

    // ロボット座標系からフィールド座標系へ座標変換(中点法を使用)
    position_.x() = delta_x * cos(position_.z() + d_theta / 2) -
                    delta_y * sin(position_.z() + d_theta / 2);
    position_.y() = delta_x * sin(position_.z() + d_theta / 2) +
                    delta_y * cos(position_.z() + d_theta / 2);
    position_.z() += d_theta;

    position_.z() = normalize_angle(position_.z());
  }

  double normalize_angle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }
};
