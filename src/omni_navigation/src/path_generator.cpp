// 開始姿勢とゴールから直線経路を返すServiceサーバー
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "omni_msgs/srv/generate_path.hpp"
#include "omni_kinematics/pose_conversions.hpp"
#include "omni_navigation/straight_path.hpp"
#include "rclcpp/rclcpp.hpp"

using omni_msgs::srv::GeneratePath;

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator()
  : Node("path_generator")
  {
    declare_parameter("waypoint_spacing", 0.05);
    declare_parameter("angular_spacing", 0.10);

    service_ = create_service<GeneratePath>(
      "generate_path",
      [this](
        const GeneratePath::Request::SharedPtr request,
        GeneratePath::Response::SharedPtr response) {
        generate_path(*request, *response);
      });
  }

private:
  // 失敗したときは空の経路を返す
  void generate_path(const GeneratePath::Request & request, GeneratePath::Response & response)
  {
    const double waypoint_spacing = get_parameter("waypoint_spacing").as_double();
    const double angular_spacing = get_parameter("angular_spacing").as_double();
    if (waypoint_spacing <= 0.0 || angular_spacing <= 0.0) {
      RCLCPP_ERROR(get_logger(), "経路点の間隔は正の値にしてください");
      return;
    }
    if (request.start.header.frame_id != request.goal.header.frame_id) {
      RCLCPP_ERROR(
        get_logger(), "startとgoalのフレームが異なります: %s, %s",
        request.start.header.frame_id.c_str(), request.goal.header.frame_id.c_str());
      return;
    }

    const auto waypoints = omni_navigation::make_straight_path(
      omni_kinematics::to_pose2d(request.start.pose),
      omni_kinematics::to_pose2d(request.goal.pose),
      waypoint_spacing, angular_spacing);

    response.path.header.stamp = now();
    response.path.header.frame_id = request.goal.header.frame_id;
    for (const auto & waypoint : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = response.path.header;
      pose.pose = omni_kinematics::to_pose_msg(waypoint);
      response.path.poses.push_back(pose);
    }
    RCLCPP_INFO(get_logger(), "%zu点の経路を生成しました", response.path.poses.size());
  }

  rclcpp::Service<GeneratePath>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGenerator>());
  rclcpp::shutdown();
  return 0;
}
