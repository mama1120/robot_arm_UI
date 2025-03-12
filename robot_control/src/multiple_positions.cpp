#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <cmath>

// Initialise constants
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double TOLERANCE = 1.0 * DEG2RAD; // 1 degree tolerance in radians
constexpr double approach_value = 0.1;

//Initialise positions
std::vector<double> picking_position = {-0.3391999304294586, 0.3847830891609192, 0.14593285977840424, 0.2997700870037079, -0.634311318397522, 0.29266050457954407, 0.6497206091880798};

std::vector<double> placing_position = {0.4687330424785614, -0.0017549340846017003, 0.1464482843875885, 0.7049350738525391, 0.0027630655094981194, 0.7092611789703369, -0.0027462122961878777};

// was macht diese Funktion?
bool within_tolerance(const std::vector<double>& current, const std::vector<double>& target, double tolerance) {
    for (size_t i = 0; i < current.size(); ++i) {
        if (std::abs(current[i] - target[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

// function for moving to home position
void move_to_home_position(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& logger_name) {
    move_group.setNamedTarget("home");
    bool success_home_position = static_cast<bool>(move_group.move());
    
    if (success_home_position) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement to home position successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement to home position failed");
    }
}

// function for moving to any other position
void move_to_position(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const std::vector<double>& position,
                      const double& approach, 
                      const std::string& logger_name) {
                      
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = position[0];
    target_pose.position.y = position[1];
    target_pose.position.z = position[2] + approach;
    target_pose.orientation.w = position[3];
    target_pose.orientation.x = position[4];
    target_pose.orientation.y = position[5];
    target_pose.orientation.z = position[6];
    
    move_group.setPlannerId("RRTConnectConfigDefault");//LBKPIECEkConfigDefault
    //move_group.setPlanningTime(100.0);
    move_group.setNumPlanningAttempts(5);
  
    move_group.setPoseTarget(target_pose);
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement failed");
    }
}

// function for operating gripper
void gripper_operation(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& operation, const std::string& logger_name) {
    move_group.setNamedTarget(operation);
    bool success_gripper = static_cast<bool>(move_group.move());
    //moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    //bool success_gripper = (move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_gripper) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Gripper operation successful");
        //move_group.move();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Gripper operation failed");
    }
}

void spawn_objects() {
    
    // Mauer erzeugen
    // Erstelle eine Schnittstelle zur Planungsumgebung
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Erstelle ein Kollisionsobjekt
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box1";

    // Definiere die Form und Größe des Objekts
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.3;  // Länge
    primitive.dimensions[1] = 0.02;  // Breite
    primitive.dimensions[2] = 0.3;//0.25;  // Höhe

    // Definiere die Pose des Objekts
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.15;

    // Füge die Primitive und Pose zum Kollisionsobjekt hinzu
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Füge das Kollisionsobjekt zur PlanningScene hinzu
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    
    
    // Ball erzeugen
    
    

    //RCLCPP_INFO(rclcpp::get_logger("collision_object"), "Kollisionsobjekt wurde hinzugefügt.");
}

int main(int argc, char* argv[]) {
    
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "pose_setter", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
        
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_robot_arm = std::make_shared<MoveGroupInterface>(node, "robot_arm");
    move_group_robot_arm->setPlannerId("RRTConnectConfigDefault");
    move_group_robot_arm->setPlanningTime(100.0);
    move_group_robot_arm->setNumPlanningAttempts(10);
    
    auto move_group_hand = std::make_shared<MoveGroupInterface>(node, "hand");
    //moveit::planning_interface::MoveGroupInterface gripper_group(node, "hand");
    //move_group_hand->setPlannerId("RRTConnectConfigDefault");
    //move_group_hand->setPlanningTime(100.0);
        
    spawn_objects();
    
    /*moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Erstelle ein Kollisionsobjekt (z.B. einen Ball)
    moveit_msgs::msg::CollisionObject ball;
    ball.header.frame_id = "world";  // Setze den Frame des Balls
    ball.id = "ball";  // ID des Balls

    // Definiere die Form und Größe des Balls (Kugel)
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[0] = 0.01;  // Radius des Balls (10 cm)

    // Definiere die Pose des Balls
    //ball.header.frame_id = move_group_robot_arm->getEndEffectorLink();
    geometry_msgs::msg::Pose ball_pose;
    ball_pose.orientation.w = 1.0;
    ball_pose.position.x = -0.3391999304294586;  // Setze die Position des Balls
    ball_pose.position.y = 0.3847830891609192;
    ball_pose.position.z = 0.03;  // Setze den Ball 50 cm über dem Boden

    // Füge die Primitive und Pose zum Kollisionsobjekt hinzu
    ball.primitives.push_back(sphere);
    ball.primitive_poses.push_back(ball_pose);
    ball.operation = ball.ADD;    
    planning_scene_interface.applyCollisionObject(ball);*/
    

    // Move to "home" position
    RCLCPP_INFO(node->get_logger(), "Moving to 'home' position...");
    move_to_home_position(*move_group_robot_arm, "pose_setter");
    

    // Wait for a short period to ensure the move completes
    //rclcpp::sleep_for(std::chrono::seconds(2));


    // Move to the approach point of picking position
    RCLCPP_INFO(node->get_logger(), "Moving to approach point of picking position...");
    move_to_position(*move_group_robot_arm, picking_position, approach_value, "pose_setter");
    
    // Open gripper
    RCLCPP_INFO(node->get_logger(), "Open gripper...");
    gripper_operation(*move_group_hand, "open", "pose_setter");
    
    //gripper_group.setNamedTarget("open");
    /*moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success_gripper = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_gripper) {
        RCLCPP_INFO(rclcpp::get_logger("pose_setter"), "Gripper operation successful");
        gripper_group.move();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("pose_setter"), "Gripper operation failed");
    }*/
    
    // Wait for a short period to ensure the move completes
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    
    // Move to the picking position    
    RCLCPP_INFO(node->get_logger(), "Moving to picking position...");
    move_to_position(*move_group_robot_arm, picking_position, 0.0, "pose_setter");
    
    
    // Füge das Kollisionsobjekt zur Planungsszene hinzu
    /*std::vector<std::string> touch_links;
    touch_links.push_back("link4");
    touch_links.push_back("finger1");
    touch_links.push_back("finger2");
    touch_links.push_back("finger3");
    touch_links.push_back("plate");
    
    move_group_robot_arm->attachObject(ball.id, move_group_robot_arm->getEndEffectorLink(), touch_links);*/
    
    
    // Close gripper
    RCLCPP_INFO(node->get_logger(), "Close gripper...");
    gripper_operation(*move_group_hand, "close", "pose_setter");
    
    // Wait for a short period to ensure the move completes
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    
    // Move to the approach point of picking position
    RCLCPP_INFO(node->get_logger(), "Moving to approach point of picking position...");
    move_to_position(*move_group_robot_arm, picking_position, approach_value, "pose_setter");
    
    
    // Move to the approach point of placing position
    RCLCPP_INFO(node->get_logger(), "Moving to approach point of placing position...");
    move_to_position(*move_group_robot_arm, placing_position, approach_value, "pose_setter");
    
    
    // Move to the approach point of placing position
    RCLCPP_INFO(node->get_logger(), "Moving to placing position...");
    move_to_position(*move_group_robot_arm, placing_position, 0.0, "pose_setter");
    
    // Open gripper
    RCLCPP_INFO(node->get_logger(), "Open gripper...");
    gripper_operation(*move_group_hand, "open", "pose_setter");
    
    // Wait for a short period to ensure the move completes
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    
    // Move to the approach point of placing position
    RCLCPP_INFO(node->get_logger(), "Moving to approach point of placing position...");
    move_to_position(*move_group_robot_arm, placing_position, approach_value, "pose_setter");
    
    
    // Close gripper
    RCLCPP_INFO(node->get_logger(), "Close gripper...");
    gripper_operation(*move_group_hand, "close", "pose_setter");
    
    // Wait for a short period to ensure the move completes
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    
    // Move to "home" position
    RCLCPP_INFO(node->get_logger(), "Moving to 'home' position...");
    move_to_home_position(*move_group_robot_arm, "pose_setter");
    

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
