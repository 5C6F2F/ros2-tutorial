#include <units.h>

#include <Eigen/Dense>
#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tutorial_interfaces/msg/encoder_counts.hpp>

using namespace units::length;
using EncoderCounts = tutorial_interfaces::msg::EncoderCounts;
using Twist = geometry_msgs::msg::Twist;

// 測定輪の個数
constexpr int N = 3;
constexpr int CONFIG_SIZE_PER_WHEEL = 4;

namespace NodeName {
constexpr char CMD_VEL_TO_ENCODER[] = "cmd_vel_to_encoder";
}

namespace TopicName {
constexpr char CMD_VEL[] = "/cmd_vel";
constexpr char ENCODER_COUNTS[] = "/encoder_counts";
}  // namespace TopicName

namespace Parameters {
constexpr char ENCODER_RESOLUTION[] = "encoder_resolution";
constexpr char WHEEL_CONFIGS[] = "wheel_configs";
constexpr char FREQUENCY[] = "frequency";
}  // namespace Parameters

class CmdVelToEncoderNode : public rclcpp::Node {
 public:
  CmdVelToEncoderNode()
      : Node(NodeName::CMD_VEL_TO_ENCODER), vx_(0.0), vy_(0.0), omega_(0.0) {
    this->declare_parameter(Parameters::ENCODER_RESOLUTION, 4096);
    encoder_resolution_ =
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

    for (int i = 0; i < N; i++) {
      int base_i = i * CONFIG_SIZE_PER_WHEEL;
      meter_t x = millimeter_t(config_vector[base_i + 0]);
      meter_t y = millimeter_t(config_vector[base_i + 1]);
      double theta = config_vector[base_i + 2];
      meter_t radius = millimeter_t(config_vector[base_i + 3]);

      wheel_matrix_(i, 0) = std::cos(theta);
      wheel_matrix_(i, 1) = std::sin(theta);
      wheel_matrix_(i, 2) =
          x.value() * wheel_matrix_(i, 1) - y.value() * wheel_matrix_(i, 0);

      double wheel_circumference = 2.0 * M_PI * radius.value();
      wheel_matrix_.row(i) /= wheel_circumference;
    }

    this->declare_parameter(Parameters::FREQUENCY, 30);
    const int frequency = this->get_parameter(Parameters::FREQUENCY).as_int();
    if (frequency <= 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid frequency parameter! expected > 0, actual: %d",
                   frequency);
      exit(1);
    }
    dt_ = 1.0 / frequency;

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
