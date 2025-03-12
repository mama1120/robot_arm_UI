#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"  // Standard service for a boolean request

using std::placeholders::_1;
using std::placeholders::_2;

class GripperServiceNode : public rclcpp::Node
{
public:
    GripperServiceNode() : Node("gripper_service_server")
    {
        service_ = this->create_service<example_interfaces::srv::SetBool>(
            "control_gripper",
            std::bind(&GripperServiceNode::handle_gripper_request, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Gripper Service Server is ready.");
    }

private:
    void handle_gripper_request(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Received request: OPEN gripper");
            response->success = true;
            response->message = "Gripper opened.";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received request: CLOSE gripper");
            response->success = true;
            response->message = "Gripper closed.";
        }
    }

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperServiceNode>());
    rclcpp::shutdown();
    return 0;
}
