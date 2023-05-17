#include <memory>
#include<iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

int main(int argc, char * argv[])
{

  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

//move_group_interface.setGoalJointTolerance(0.005);
//move_group_interface.setGoalPositionTolerance(0.005);
//move_group_interface.setGoalOrientationTolerance(0.05);
move_group_interface.setPlanningTime(5);
move_group_interface.setPlannerId("ESTkConfigDefault");
RCLCPP_INFO(logger, move_group_interface.getPlannerId());

// Set a target Pose
//Posicion y orientacion esta entre base_linkinertia y tool0(en ROS2) y en la tablet es entre base y tool0
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.972;
  float Ry=2.455;
  float Rz=-0.078;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.321;  //BARRA ROJA
  msg.position.y = -0.151;  //BARRA VERDE
  msg.position.z = 0.313;  //arriba, abajo   BARRA-AZUL
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);
//move_group_interface.setOrientationTarget(0.865,-0.500,-0.037,-0.027);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);  
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
  move_group_interface.execute(plan);
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
