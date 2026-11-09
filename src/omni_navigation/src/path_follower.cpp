// 経路に追従し、進捗を報告するActionサーバー
//
// コールバックの中で走行の完了を待つと、他のコールバックが止まる。
// そこでGoalは受け取るだけにして、走行はタイマーで少しずつ進める。

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "omni_kinematics/pose_conversions.hpp"
#include "omni_msgs/action/follow_path.hpp"
#include "omni_navigation/pure_pursuit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

using omni_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

class PathFollower : public rclcpp::Node {
 public:
  PathFollower()
      : Node("path_follower"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_),
        last_pose_time_(now()),
        last_feedback_time_(now()) {
    declare_parameter("global_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("control_rate", 50.0);
    declare_parameter("feedback_rate", 10.0);
    declare_parameter("lookahead_distance", 0.30);
    declare_parameter("max_linear_speed", 0.50);
    declare_parameter("max_angular_speed", 1.00);
    declare_parameter("position_gain", 1.5);
    declare_parameter("yaw_gain", 2.0);
    declare_parameter("position_tolerance", 0.02);
    declare_parameter("yaw_tolerance", 0.035);
    declare_parameter("tf_timeout", 0.5);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<FollowPath>(
        this, "follow_path",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const FollowPath::Goal> goal) {
          return handle_goal(*goal);
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle) {
          handle_accepted(goal_handle);
        });

    const auto period = std::chrono::duration<double>(
        1.0 / get_parameter("control_rate").as_double());
    control_timer_ = create_timer(period, [this]() { control_step(); });
  }

 private:
  rclcpp_action::GoalResponse handle_goal(const FollowPath::Goal& goal) {
    if (goal.path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "空の経路は受け付けません");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const std::string global_frame = get_parameter("global_frame").as_string();
    if (goal.path.header.frame_id != global_frame) {
      RCLCPP_WARN(get_logger(), "経路のフレームが %s ではありません: %s",
                  global_frame.c_str(), goal.path.header.frame_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal_handle_) {
      RCLCPP_WARN(get_logger(),
                  "走行中のGoalがあるため、新しいGoalを受け付けません");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    path_.clear();
    for (const auto& pose : goal_handle->get_goal()->path.poses) {
      path_.push_back(omni_kinematics::to_pose2d(pose.pose));
    }
    goal_handle_ = goal_handle;
    nearest_index_ = 0;
    pose_.reset();
    last_pose_time_ = now();
    last_feedback_time_ = now();
  }

  void control_step() {
    if (!goal_handle_) {
      return;
    }
    if (goal_handle_->is_canceling()) {
      finish(
          [this](const FollowPath::Result::SharedPtr result) {
            goal_handle_->canceled(result);
          },
          false);
      RCLCPP_INFO(get_logger(), "キャンセルしました");
      return;
    }

    update_pose();
    if ((now() - last_pose_time_).seconds() >
        get_parameter("tf_timeout").as_double()) {
      finish(
          [this](const FollowPath::Result::SharedPtr result) {
            goal_handle_->abort(result);
          },
          false);
      RCLCPP_ERROR(get_logger(), "自己位置を取得できないため中断しました");
      return;
    }
    if (!pose_) {
      publish_velocity(omni_kinematics::BodyMotion{});
      return;
    }

    const omni_kinematics::Pose2D& goal = path_.back();
    const double distance_to_goal =
        omni_navigation::distance_between(*pose_, goal);
    if (reached(goal, distance_to_goal)) {
      finish(
          [this](const FollowPath::Result::SharedPtr result) {
            goal_handle_->succeed(result);
          },
          true);
      RCLCPP_INFO(get_logger(), "ゴールに到達しました");
      return;
    }

    publish_velocity(next_velocity());
    publish_feedback(distance_to_goal);
  }

  // TFから最新の姿勢を取得する。取得できなければ前回の値を残す
  void update_pose() {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          get_parameter("global_frame").as_string(),
          get_parameter("base_frame").as_string(), tf2::TimePointZero);
      // TFの発行が止まると、バッファには古い値が残り続ける。時刻で鮮度を判断する
      const rclcpp::Time stamp(transform.header.stamp,
                               get_clock()->get_clock_type());
      if (stamp > last_pose_time_) {
        last_pose_time_ = stamp;
      }
      pose_ = omni_kinematics::to_pose2d(transform);
    } catch (const tf2::TransformException& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                           exception.what());
    }
  }

  bool reached(const omni_kinematics::Pose2D& goal,
               const double distance_to_goal) const {
    const double yaw_error = std::remainder(goal.yaw - pose_->yaw, 2.0 * M_PI);
    return distance_to_goal <=
               get_parameter("position_tolerance").as_double() &&
           std::abs(yaw_error) <= get_parameter("yaw_tolerance").as_double();
  }

  omni_kinematics::BodyMotion next_velocity() {
    omni_navigation::PurePursuitConfig config;
    config.lookahead_distance = get_parameter("lookahead_distance").as_double();
    config.max_linear_speed = get_parameter("max_linear_speed").as_double();
    config.max_angular_speed = get_parameter("max_angular_speed").as_double();
    config.position_gain = get_parameter("position_gain").as_double();
    config.yaw_gain = get_parameter("yaw_gain").as_double();

    nearest_index_ =
        omni_navigation::find_nearest_index(path_, *pose_, nearest_index_);
    const std::size_t target_index = omni_navigation::find_target_index(
        path_, *pose_, nearest_index_, config.lookahead_distance);
    return omni_navigation::compute_velocity(*pose_, path_[target_index],
                                             path_.back(), config);
  }

  void publish_velocity(const omni_kinematics::BodyMotion& velocity) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.x;
    twist.linear.y = velocity.y;
    twist.angular.z = velocity.yaw;
    cmd_vel_pub_->publish(twist);
  }

  void publish_feedback(const double distance_to_goal) {
    const double period = 1.0 / get_parameter("feedback_rate").as_double();
    if ((now() - last_feedback_time_).seconds() < period) {
      return;
    }
    last_feedback_time_ = now();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->distance_to_goal = static_cast<float>(distance_to_goal);
    goal_handle_->publish_feedback(feedback);
  }

  // 機体を止めてからGoalを終了状態へ移す
  template <typename EndGoal>
  void finish(EndGoal end_goal, const bool success) {
    publish_velocity(omni_kinematics::BodyMotion{});
    auto result = std::make_shared<FollowPath::Result>();
    result->success = success;
    end_goal(result);
    goal_handle_.reset();
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<GoalHandle> goal_handle_;
  std::vector<omni_kinematics::Pose2D> path_;
  std::size_t nearest_index_{0};
  std::optional<omni_kinematics::Pose2D> pose_;
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_feedback_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
