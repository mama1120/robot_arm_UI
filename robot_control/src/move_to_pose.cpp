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
    move_group_->setGoalPositionTolerance(0.01);

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
    
    // If theta rotation is requested, adjust joint4
    if (request->apply_theta) {
      // Wait to ensure we have latest joint states after position movement
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      // Check if we have joint state information
      if (!latest_joint_state_ || latest_joint_state_->name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No joint state information available for joint4 adjustment");
        response->success = true;  // Position movement succeeded
        return;
      }
      
      // Find joint4 in the joint state message
      int joint4_index = -1;
      for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
        if (latest_joint_state_->name[i] == "joint4") {
          joint4_index = static_cast<int>(i);
          break;
        }
      }
      
      if (joint4_index == -1) {
        RCLCPP_ERROR(this->get_logger(), "joint4 not found in joint states");
        response->success = true;  // Position movement succeeded
        return;
      }
      
      // Get target theta value (absolute joint angle)
      double target_joint4_value = request->theta * M_PI / 180.0;  // Convert to radians
      
      RCLCPP_INFO(this->get_logger(), "Setting joint4 to: %.3f degrees (%.3f radians)",
                  request->theta, target_joint4_value);
      
      // Set joint value target for joint4
      move_group_->setJointValueTarget("joint4", target_joint4_value);
      
      // Execute joint movement
      auto joint_result = move_group_->move();
      
      if (joint_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Joint4 adjustment succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Joint4 adjustment failed");
        // Still return success since position movement succeeded
      }
    }
    
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
//ros2 service call /move_to_pose rviz_services/srv/MoveToPose "{x: 0.3, y: 0.0, z: 0.4, theta: 55.0, apply_theta: true}"