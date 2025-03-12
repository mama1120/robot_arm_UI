#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"  // Service for boolean request
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;
using std::placeholders::_2;

class GripperControlClient : public rclcpp::Node
{
public:
    GripperControlClient() : Node("gripper_control_client")
    {
        // Create a client to the control_gripper service
        client_ = this->create_client<example_interfaces::srv::SetBool>("control_gripper");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        RCLCPP_INFO(this->get_logger(), "Service available. Sending request...");

        // Create request and send the command to open the gripper
        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = true;  // True means open the gripper

        // Send the request asynchronously
        auto future = client_->async_send_request(request);

        // Wait for response from the server
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Gripper command response: %s", response->message.c_str());

            // If the gripper should be opened, move the robot's hand to the "open" pose
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Moving gripper to 'open' pose.");
                move_gripper_to_pose("open");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive gripper command response.");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;

    void move_gripper_to_pose(const std::string &pose_name)
    {
        // Get the MoveGroup interface for controlling the hand
        auto move_group_hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "hand");

        // Set the target pose to 'open' pose
        move_group_hand->setNamedTarget(pose_name);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_hand->plan(plan);

        // Execute the planned motion
        move_group_hand->execute(plan);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 context

    // Create and spin the client node
    rclcpp::spin(std::make_shared<GripperControlClient>());

    rclcpp::shutdown();  // Shutdown ROS 2 when done
    return 0;
}
