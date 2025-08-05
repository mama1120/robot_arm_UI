/**
 * 
 * This program provides a simple ROS2 node, which listens for service requests to move the robot arm to a specified position.
 * It uses MoveIt's MoveGroupInterface to plan and execute the movement, allowing for precise control
 * of the robot arm's end-effector position in 3D space. Example service call:
 * ros2 service call /move_to_pose rviz_services/srv/MoveToPose "{0.3, y: 0.0, z: 0.4}"
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>

// Include your service message header
#include <rviz_services/srv/move_to_pose.hpp>

using moveit::planning_interface::MoveGroupInterface;

class MoveToPoseNode : public rclcpp::Node {
public:
  MoveToPoseNode() : Node("move_to_pose") {
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() { this->initialize(); });
  }
  
  ~MoveToPoseNode() {
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
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
    move_group_->setPlanningTime(15.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setGoalPositionTolerance(0.001);

    // Create joint state subscriber to get current joint values
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;
      });

    // Create the service
    service_ = this->create_service<rviz_services::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&MoveToPoseNode::move_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Position and joint4 rotation service initialized");
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
    
    // Execute position movement
    RCLCPP_INFO(this->get_logger(), "Moving to position (%.3f, %.3f, %.3f)", 
                request->x, request->y, request->z);
                
    auto position_result = move_group_->move();
    
    if (position_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Position movement failed");
      response->success = false;
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Position movement succeeded");
    response->success = true;
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  rclcpp::Service<rviz_services::srv::MoveToPose>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::thread spin_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
