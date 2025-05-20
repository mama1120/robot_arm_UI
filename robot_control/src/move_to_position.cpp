#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// Include your service message header
#include <rviz_services/srv/move_to_pose.hpp>

using moveit::planning_interface::MoveGroupInterface;

class MoveToPoseNode : public rclcpp::Node {
public:
  MoveToPoseNode() : Node("move_to_pose_server") {
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() { this->initialize(); });
  }

private:
  void initialize() {
    move_group_ = std::make_shared<MoveGroupInterface>(
      shared_from_this(),
      "robot_arm"
    );
    
    // Configure position-only planning
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("link4_1");
    move_group_->setPlannerId("RRTConnectConfigDefault");
    move_group_->setPlanningTime(15.0);  // Increased planning time
    move_group_->setNumPlanningAttempts(20);  // More attempts
    move_group_->setGoalPositionTolerance(0.01);  // 1cm position tolerance

    service_ = this->create_service<rviz_services::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&MoveToPoseNode::move_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Position-only service initialized");
    init_timer_->cancel();
  }

  void move_to_pose_callback(
    const std::shared_ptr<rviz_services::srv::MoveToPose::Request> request,
    const std::shared_ptr<rviz_services::srv::MoveToPose::Response> response) {
    
    // Set position target only (ignore orientation)
    geometry_msgs::msg::Point target_position;
    target_position.x = request->x;
    target_position.y = request->y;
    target_position.z = request->z;

    // Clear any previous pose targets
    move_group_->clearPoseTargets();
    
    // Set position target with end effector link
    move_group_->setPositionTarget(target_position.x, target_position.y, target_position.z, move_group_->getEndEffectorLink());
    
    // Execute movement
    auto const result = move_group_->move();
    
    response->success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Position movement succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Position movement failed");
    }
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  rclcpp::Service<rviz_services::srv::MoveToPose>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}