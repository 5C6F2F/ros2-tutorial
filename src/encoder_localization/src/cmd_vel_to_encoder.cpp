#include <units.h>

#include <Eigen/Dense>
#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tutorial_interfaces/msg/encoder_counts.hpp>

#include "wheel_config.hpp"

using namespace units::literals;
using EncoderCounts = tutorial_interfaces::msg::EncoderCounts;
using Twist = geometry_msgs::msg::Twist;

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
constexpr char CMD_VEL_TO_ENCODER[] = "cmd_vel_to_encoder";
}

namespace TopicName {
constexpr char CMD_VEL[] = "/cmd_vel";
constexpr char ENCODER_COUNTS[] = "/encoder_counts";
}  // namespace TopicName

class CmdVelToEncoderNode : public rclcpp::Node {
 public:
  CmdVelToEncoderNode()
      : Node(NodeName::CMD_VEL_TO_ENCODER), vx_(0.0), vy_(0.0), omega_(0.0) {
    encoder_resolution_ = ENCODER_RESOLUTION;

    for (int i = 0; i < N; i++) {
      double x = WHEEL_CONFIGS[i].x.value();
      double y = WHEEL_CONFIGS[i].y.value();
      double theta = WHEEL_CONFIGS[i].theta.value();
      double radius = WHEEL_CONFIGS[i].radius.value();

      wheel_matrix_(i, 0) = std::cos(theta);
      wheel_matrix_(i, 1) = std::sin(theta);
      wheel_matrix_(i, 2) = x * wheel_matrix_(i, 1) - y * wheel_matrix_(i, 0);

      double wheel_circumference = 2.0 * M_PI * radius;
      wheel_matrix_.row(i) /= wheel_circumference;
    }

    dt_ = 1.0 / FREQUENCY;

    // エンコーダカウントを初期化
    encoder_counts_.fill(0);

    // 受け取ったcmd_velをメンバ変数に保存するだけ
    // 実際の計算は定周期に実行
    cmd_vel_sub_ = this->create_subscription<Twist>(
        TopicName::CMD_VEL, 10, [this](const Twist::ConstSharedPtr msg) {
          vx_ = msg->linear.x;
          vy_ = msg->linear.y;
          omega_ = msg->angular.z;
        });

    encoder_pub_ =
        this->create_publisher<EncoderCounts>(TopicName::ENCODER_COUNTS, 10);

    // create_wall_timerで指定した周期ごとにコールバックが呼ばれる。
    // chrono::duration<double>でdt_秒を指定。
    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                     [this]() { this->timer_callback(); });

    RCLCPP_INFO(this->get_logger(), "cmd_vel_to_encoder node has started.");
  }

 private:
  Eigen::Matrix<double, N, 3> wheel_matrix_;
  int encoder_resolution_;
  double dt_;
  double vx_, vy_, omega_;             // 最新の速度指令
  std::array<int, N> encoder_counts_;  // エンコーダカウント

  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<EncoderCounts>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // タイマーコールバック
  //
  // 1. 速度指令 → 各車輪のRPS(順運動学)
  // 2. RPS × dt → 1ステップあたりの回転数
  // 3. 回転数 × encoder_resolution → カウント増分
  // 4. 累積カウントに加算してpublish
  void timer_callback() {
    Eigen::Vector3d robot_vel(vx_, vy_, omega_);

    // 順運動学: wheel_rps = W * [vx, vy, omega]^T  [RPS]
    Eigen::Vector<double, N> wheel_rps = wheel_matrix_ * robot_vel;

    // 1ステップあたりの回転数 [rot] → エンコーダカウント増分
    for (int i = 0; i < N; i++) {
      double delta_rot = wheel_rps(i) * dt_;  // [rot/s] × [s] = [rot]
      int delta_count =
          static_cast<int>(std::round(delta_rot * encoder_resolution_));
      encoder_counts_[i] += delta_count;
    }

    EncoderCounts msg;
    msg.front_counts = encoder_counts_[0];
    msg.rear_left_counts = encoder_counts_[1];
    msg.rear_right_counts = encoder_counts_[2];
    encoder_pub_->publish(msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToEncoderNode>());
  rclcpp::shutdown();
  return 0;
}
