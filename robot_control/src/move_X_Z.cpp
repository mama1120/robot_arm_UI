#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rviz_services/srv/move_linear.hpp"  // Custom service type

    // Function for moving to a target pose
    bool move_to_position(moveit::planning_interface::MoveGroupInterface& move_group, 
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& logger_name) {
    move_group.setPlannerId("RRTConnectConfigDefault");
    move_group.setNumPlanningAttempts(3);
    move_group.setPoseTarget(target_pose);

    bool success = static_cast<bool>(move_group.move());
    if (success) {
    RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement successful");
    } else {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement failed");
    }
    return success;
    }


class RobotArmService : public rclcpp::Node {
public:
    RobotArmService() : Node("robot_arm_service"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Delay MoveGroupInterface initialization
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&RobotArmService::initialize_move_group, this));

        // Create service for MoveLinear
        service_ = this->create_service<rviz_services::srv::MoveLinear>(
            "move_linear",
            std::bind(&RobotArmService::handle_move_linear, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void initialize_move_group() {
        move_group_robot_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "robot_arm");
        move_group_robot_arm_->setPlannerId("RRTConnectConfigDefault");
        move_group_robot_arm_->setPlanningTime(5.0);
        move_group_robot_arm_->setNumPlanningAttempts(3);

        init_timer_->cancel(); // Stop the timer after initialization
    }

    // Service callback for MoveLinear
    void handle_move_linear(
        const std::shared_ptr<rviz_services::srv::MoveLinear::Request> request,
        std::shared_ptr<rviz_services::srv::MoveLinear::Response> response) {
        
        if (!move_group_robot_arm_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized");
            response->success = false;
            return;
        }

        // Convert distance from mm to meters
        double distance_m = request->distance_mm / 1000.0;

        // Validate direction input
        if (request->direction != "X" && request->direction != "Z") {
            RCLCPP_ERROR(this->get_logger(), "Invalid direction: %s. Use 'X' or 'Z'", request->direction.c_str());
            response->success = false;
            return;
        }

        // Get the transform from link4_1 to base_link
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("base_link", "link4_1", tf2::TimePointZero, tf2::durationFromSec(1.0));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            response->success = false;
            return;
        }

        // Create a pose in link4_1 frame with the relative movement
        geometry_msgs::msg::PoseStamped pose_link4_1;
        pose_link4_1.header.frame_id = "link4_1";
        pose_link4_1.pose.position.x = (request->direction == "X") ? distance_m : 0.0;
        pose_link4_1.pose.position.y = 0.0;
        pose_link4_1.pose.position.z = (request->direction == "Z") ? distance_m : 0.0;
        pose_link4_1.pose.orientation.w = 1.0;

        // Transform the pose to the base_link frame
        geometry_msgs::msg::PoseStamped pose_base_link;
        try {
            pose_base_link = tf_buffer_.transform(pose_link4_1, "base_link", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF transformation failed: %s", ex.what());
            response->success = false;
            return;
        }

    // Move the robot arm to the new pose
    bool movement_success = move_to_position(*move_group_robot_arm_, pose_base_link.pose, "pose_setter");
    response->success = movement_success;


    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_robot_arm_;
    rclcpp::Service<rviz_services::srv::MoveLinear>::SharedPtr service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
