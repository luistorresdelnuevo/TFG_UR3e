#include <memory>
#include<iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <ur_msgs/srv/set_io.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

/*
using namespace std::chrono_literals;
using namespace std;

int main(int argc, char * argv[])
{
rclcpp::init(argc,argv);
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cartesian_path_planner");  // CHANGE



auto const target_pose1 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.739;
  float Ry=2.650;
  float Rz=0.055;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.242;  //BARRA ROJA
  msg.position.y = -0.128;  //BARRA VERDE
  msg.position.z = 0.410;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

//Bajar pieza 2
auto const target_pose2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.739;
  float Ry=2.650;
  float Rz=0.055;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.242;  //BARRA ROJA
  msg.position.y = -0.128;  //BARRA VERDE
  msg.position.z = 0.310;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

//Bajar pieza 3
auto const target_pose3 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.739;
  float Ry=2.650;
  float Rz=0.055;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.242;  //BARRA ROJA
  msg.position.y = -0.128;  //BARRA VERDE
  msg.position.z = 0.210;  //arriba, abajo   BARRA-AZUL
  return msg;
}();




//CREANDO UN CLIENTE PARA CARTESIAN PATH


rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr client_ =node->create_client<moveit_msgs::srv::GetCartesianPath>("/compute_cartesian_path");          // CHANGE


while (!client_ ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }



auto request = std::make_shared<moveit_msgs::srv::GetCartesianPath>();
request->header.frame_id = "base";
request->waypoints.push_back(geometry_msgs::msg::Pose()); // La pose inicial se agrega automáticamente
std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose1,target_pose2,target_pose3}; // Agregue aquí los puntos de la trayectoria cartesiana
for (const auto& waypoint : waypoints) {
  request->waypoints.push_back(waypoint);
}
request->max_step = 0.01;
request->jump_threshold = 0;
request->avoid_collisions = true;

auto future = client_->async_send_request(request);
rclcpp::spin_until_future_complete(node, future);
auto response = future.get();
auto trajectory = response->solution;

if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de Camino cartesiano funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}*/
