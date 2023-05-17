#include <memory>
#include<iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <ur_msgs/srv/set_io.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <semaphore.h>
//#include "ur_msgs/SetIORequest.h"
//#include "ur_msgs/SetIOResponse.h"

pthread_mutex_t mutex;

using namespace std::chrono_literals;

//CREANDO UN CLIENTE PARA GRIPPER



auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
 //Crea un cliente que llama al servicio 





void CerrarGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node)
{

  pthread_mutex_lock(&mutex);
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
request->pin= 17; //16 abrir y 17 cerrar
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

  pthread_mutex_unlock(&mutex);
}


void AbrirGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node)
{
   pthread_mutex_lock(&mutex);


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
  pthread_mutex_unlock(&mutex);

}







int main(int argc, char * argv[])
{


  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  pthread_mutex_init(&mutex, NULL);

auto const node = std::make_shared<rclcpp::Node>(
  "hello_moveit",
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);

rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_ =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE


//POSICIONES
//pos1
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

//Bajar pieza 2
auto const target_pose2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.568;
  float Ry=2.375;
  float Rz=-0.629;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.033;  //BARRA ROJA
  msg.position.y = -0.3;  //BARRA VERDE
  msg.position.z = 0.471;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

//Bajar pieza 3
auto const target_pose3 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=3.584;
  float Ry=0.566;
  float Rz=0.082;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.024;  //BARRA ROJA
  msg.position.y = -0.435;  //BARRA VERDE
  msg.position.z = 0.184;  //arriba, abajo   BARRA-AZUL
  return msg;
}();





// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");


// Create a ROS logger
auto const logger = rclcpp::get_logger("hello_moveit");

//MOVEMOS POSICION INICIAL
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

//Abrimos Gripper
AbrirGripper(client_,node);

// Movemos a coger pieza
move_group_interface.setPoseTarget(target_pose1);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

CerrarGripper(client_,node);

//movimiento intermedio
move_group_interface.setPoseTarget(target_pose2);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

//Lugar a dejar la pieza
move_group_interface.setPoseTarget(target_pose3);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

AbrirGripper(client_,node);

//MOVEMOS POSICION INICIAL
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();



 //auto const logger = rclcpp::get_logger("hello_moveit");



  // Shutdown ROS
  pthread_mutex_destroy(&mutex);
  rclcpp::shutdown();
  return 0;
}