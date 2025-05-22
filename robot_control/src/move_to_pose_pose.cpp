#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// Include your actual service message header
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
    // Initialize MoveGroupInterface with base_link frame
    move_group_ = std::make_shared<MoveGroupInterface>(
      shared_from_this(),
      "robot_arm"
    );
    
    // Configure frame settings correctly
    move_group_->setPoseReferenceFrame("base_link");  // Set reference frame for poses
    move_group_->setEndEffectorLink("link4_1");       // Set your end effector link
    move_group_->setPlannerId("RRTConnectConfigDefault");
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);

    service_ = this->create_service<rviz_services::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&MoveToPoseNode::move_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Service initialized in base_link frame");
    init_timer_->cancel();
  }

  void move_to_pose_callback(
    const std::shared_ptr<rviz_services::srv::MoveToPose::Request> request,
    const std::shared_ptr<rviz_services::srv::MoveToPose::Response> response) {
    
    // Create pose stamped message with base_link frame
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";  // Set the frame_id to base_link
    target_pose.pose.position.x = request->x;
    target_pose.pose.position.y = request->y;
    target_pose.pose.position.z = request->z;

    target_pose.pose.orientation.w = request->w;
    target_pose.pose.orientation.x = request->w_x;
    target_pose.pose.orientation.y = request->w_y;
    target_pose.pose.orientation.z = request->w_z;

    move_group_->setPoseTarget(target_pose);
    RCLCPP_INFO(this->get_logger(), "Moving to target pose in base_link frame: x=%.2f, y=%.2f, z=%.2f, w=%.2f, w_x=%.2f, w_y=%.2f, w_z=%.2f",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.w,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z);
   
    auto const result = move_group_->move();
    
    response->success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Movement in base_link frame succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Movement in base_link frame failed");
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

//Example of the node being called by a service: 
//ros2 service call /move_to_pose rviz_services/srv/MoveToPose "{x: -0.3391999304294586, y: 0.3847830891609192, z: 0.14593285977840424, 
//w: 0.2997700870037079, w_x: -0.634311318397522, w_y: 0.29266050457954407, w_z: 0.6497206091880798}