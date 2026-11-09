// ゴールの受信から追従の完了までを統括する状態機械
//
// ServiceとActionの応答はコールバックで受け取り、そこで次の状態へ進める。
// 応答を待ってブロックしないので、走行中でも新しいゴールを受信できる。
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "omni_msgs/action/follow_path.hpp"
#include "omni_msgs/srv/generate_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using geometry_msgs::msg::PoseStamped;
using omni_msgs::action::FollowPath;
using omni_msgs::srv::GeneratePath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

enum class State {IDLE, PLANNING, FOLLOWING, CANCELING};

const char * to_string(const State state)
{
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::PLANNING: return "PLANNING";
    case State::FOLLOWING: return "FOLLOWING";
    case State::CANCELING: return "CANCELING";
  }
  return "UNKNOWN";
}

class NavigationManager : public rclcpp::Node
{
public:
  NavigationManager()
  : Node("navigation_manager"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    planning_start_time_(now())
  {
    declare_parameter("global_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("planning_timeout", 1.0);

    // QoSはYAMLの qos_overrides で起動時に変更できる
    rclcpp::PublisherOptions path_options;
    path_options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", rclcpp::QoS(10), path_options);
    goal_sub_ = create_subscription<PoseStamped>(
      "goal_pose", 10,
      [this](const PoseStamped::ConstSharedPtr msg) {on_goal_pose(*msg);});
    path_client_ = create_client<GeneratePath>("generate_path");
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    using namespace std::chrono_literals;
    watchdog_timer_ = create_timer(100ms, [this]() {watch_responses();});
  }

private:
  void set_state(const State state)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s", to_string(state_), to_string(state));
    state_ = state;
  }

  void on_goal_pose(const PoseStamped & goal)
  {
    const std::string global_frame = get_parameter("global_frame").as_string();
    if (goal.header.frame_id != global_frame) {
      RCLCPP_WARN(
        get_logger(), "ゴールのフレームが %s ではありません: %s",
        global_frame.c_str(), goal.header.frame_id.c_str());
      return;
    }

    // 処理中のゴールが片付いたら、最後に受けたゴールへ向かう
    latest_goal_ = goal;
    switch (state_) {
      case State::IDLE:
        request_path();
        break;
      case State::FOLLOWING:
        cancel_following();
        break;
      case State::PLANNING:
      case State::CANCELING:
        break;
    }
  }

  void request_path()
  {
    const PoseStamped goal = *latest_goal_;
    latest_goal_.reset();

    const auto start = current_pose();
    if (!start) {
      set_state(State::IDLE);
      return;
    }
    if (!path_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "generate_pathを呼び出せません");
      set_state(State::IDLE);
      return;
    }

    auto request = std::make_shared<GeneratePath::Request>();
    request->start = *start;
    request->goal = goal;
    set_state(State::PLANNING);
    planning_start_time_ = now();
    path_client_->async_send_request(
      request,
      [this](rclcpp::Client<GeneratePath>::SharedFuture future) {on_path(*future.get());});
  }

  std::optional<PoseStamped> current_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        get_parameter("global_frame").as_string(),
        get_parameter("base_frame").as_string(),
        tf2::TimePointZero);
      PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_ERROR(get_logger(), "現在の姿勢を取得できません: %s", exception.what());
      return std::nullopt;
    }
  }

  void on_path(const GeneratePath::Response & response)
  {
    if (latest_goal_) {
      request_path();
      return;
    }
    if (response.path.poses.empty()) {
      RCLCPP_ERROR(get_logger(), "経路を生成できませんでした");
      set_state(State::IDLE);
      return;
    }
    if (!follow_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "follow_pathを呼び出せません");
      set_state(State::IDLE);
      return;
    }

    path_pub_->publish(response.path);

    FollowPath::Goal follow_goal;
    follow_goal.path = response.path;
    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr goal_handle) {on_goal_response(goal_handle);};
    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000, "ゴールまで %.2f m", feedback->distance_to_goal);
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {on_follow_result(result);};
    set_state(State::FOLLOWING);
    follow_client_->async_send_goal(follow_goal, options);
  }

  void on_goal_response(const GoalHandle::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "経路追従のGoalが拒否されました");
      resume_or_idle();
      return;
    }
    goal_handle_ = goal_handle;
    // Goalの受理を待つ間に新しいゴールが届いていた場合
    if (latest_goal_) {
      cancel_following();
    }
  }

  void cancel_following()
  {
    // Goalの受理前はキャンセルできない。on_goal_responseで改めて呼ばれる
    if (!goal_handle_) {
      return;
    }
    set_state(State::CANCELING);
    follow_client_->async_cancel_goal(goal_handle_);
  }

  void on_follow_result(const GoalHandle::WrappedResult & result)
  {
    goal_handle_.reset();
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "ゴールに到達しました");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "経路追従をキャンセルしました");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "経路追従が中断されました");
        break;
    }
    resume_or_idle();
  }

  // 相手のノードが落ちると、応答のコールバックは呼ばれない。状態が進まなくなるのを防ぐ
  void watch_responses()
  {
    const double timeout = get_parameter("planning_timeout").as_double();
    if (state_ == State::PLANNING && (now() - planning_start_time_).seconds() > timeout) {
      RCLCPP_ERROR(get_logger(), "generate_pathの応答がありません");
      path_client_->prune_pending_requests();
      resume_or_idle();
    } else if (
      (state_ == State::FOLLOWING || state_ == State::CANCELING) &&
      !follow_client_->action_server_is_ready())
    {
      RCLCPP_ERROR(get_logger(), "follow_pathのサーバーが停止しました");
      goal_handle_.reset();
      resume_or_idle();
    }
  }

  void resume_or_idle()
  {
    if (latest_goal_) {
      request_path();
    } else {
      set_state(State::IDLE);
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Client<GeneratePath>::SharedPtr path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  State state_{State::IDLE};
  rclcpp::Time planning_start_time_;
  std::optional<PoseStamped> latest_goal_;
  GoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationManager>());
  rclcpp::shutdown();
  return 0;
}
