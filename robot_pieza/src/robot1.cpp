#include <memory>
#include<iostream>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <string>
#include <ur_msgs/srv/set_io.hpp>
#include <functional>

using namespace std;

auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();

void AbrirGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node)
{

 request->fun= 1; 
 request->pin= 16; //16 abrir y 17 cerrar
 request->state= 0; //Funciona por flanco
  while (!client_ ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result = client_ ->async_send_request(request);

 if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

 request->fun= 1; 
 request->pin= 17; //16 abrir y 17 cerrar
 request->state= 0; //Funciona por flanco

 while (!client_ ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result = client_ ->async_send_request(request);

 if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

 request->fun= 1; 
 request->pin= 16; //16 abrir y 17 cerrar
 request->state= 1; //Funciona por flanco

 while (!client_ ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result = client_ ->async_send_request(request);

 if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

}




void callback (auto const node, moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm,const geometry_msgs::msg::Pose& pose, auto *move_group_interface,rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_){
 move_group_interface->setPoseTarget(pose);
 bool success = (move_group_interface->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 move_group_interface->move();

 AbrirGripper(client_,node);

 
 auto message = std_msgs::msg::String();
 message.data ="pieza";
 publisher_->publish(message);

}





int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot1",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );


//Posicion 1
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
  msg.position.z = 0.201;  //arriba, abajo   BARRA-AZUL
  return msg;
}();



//Creacion cliente (servicio)
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_ =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");// CHANGE
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
//Creación del plan
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

//Crear subscriptor


publisher_= node->create_publisher<std_msgs::msg::String>("pieza", 10); //pieza es el nombre del topic y 10 es el tamano de cola para limitar mensajes

//subscription_ = rclcpp::create_subscription<geometry_msgs::msg::Pose>("pieza", 10, callback(node,my_plan_arm,target_pose1, &move_group_interface,client_,publisher_));

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
