#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/set_bool.hpp>  // Standard SetBool service

class MoveToHomeService : public rclcpp::Node {
public:
    MoveToHomeService() : Node("move_to_home_service") {
        // Service initialization
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "move_to_home",
            std::bind(&MoveToHomeService::moveToHomeCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "MoveToHomeService is ready. Call /move_to_home to move the robot.");
    }

private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    void moveToHomeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        (void)request; // Unused parameter

        RCLCPP_INFO(this->get_logger(), "Received service call. Moving to home position...");

        // Initialize MoveGroupInterface AFTER the node is fully created
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "robot_arm");

        move_group.setNamedTarget("home");
        bool success = static_cast<bool>(move_group.move());

        // Send response
        response->success = success;
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Robot successfully moved to home position.");
            response->message = "Movement successful";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to home position.");
            response->message = "Movement failed";
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // FIX: Create a shared pointer of the node correctly
    auto node = std::make_shared<MoveToHomeService>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
