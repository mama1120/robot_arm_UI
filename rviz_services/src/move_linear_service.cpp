#include "rclcpp/rclcpp.hpp"
#include "rviz_services/srv/move_linear.hpp"  // Generated header

void move_linear_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rviz_services::srv::MoveLinear::Request> request,
  std::shared_ptr<rviz_services::srv::MoveLinear::Response> response)
{
  (void) request_header; // Unused

  // Log the received values
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Moving linear in direction: '%s', distance: %.2f mm",
              request->direction.c_str(), request->distance_mm);  


  // Assume movement is successful
  response->success = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_linear_service");

  auto service = node->create_service<rviz_services::srv::MoveLinear>(
    "move_linear", move_linear_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MoveLinear service is ready.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
