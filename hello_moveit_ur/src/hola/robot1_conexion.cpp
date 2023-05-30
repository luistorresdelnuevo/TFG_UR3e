#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include<ur_msgs/msg/io_states.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <ur_msgs/srv/set_io.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


int pin0=0;//output
int pin1=0;//input


//THIS IS ROBOT1

void callback(const ur_msgs::msg::IOStates::SharedPtr msg)
{
  // Aquí puedes procesar los datos recibidos
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_in_states[1].state);//pIN QUE SE ACTIVA AL TERMINAR ROBOT2 Y PARA EMPEZAR
  pin1=int(msg->digital_in_states[1].state);
  //mIENTRAS ROBOT1 SE MUEVE MANDAMOS UN PONEMOS D0OUT PARA QUE SE RESETEE.
  //RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_out_states[0].state);//CUANDO ACABA EL ROBOT1 SU TAREA MANDAMOS UN 1 A ROBOT2
  pin0=msg->digital_out_states[0].state;
}


void func1(int argc,char *argv[])
{
  // Aquí puedes procesar los datos recibidos
  rclcpp::init(argc,argv);
  auto const node1 = std::make_shared<rclcpp::Node>(
    "PADRIKE",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  ur_msgs::msg::IOStates::SharedPtr msg;
  auto subscription = node1->create_subscription<ur_msgs::msg::IOStates>(
    "/io_and_status_controller/io_states",  // Nombre del topic
    10,callback  // Tamaño de la cola de mensajes// Función de callback
  );
  rclcpp::spin(node1);
}






int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit_ur",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );



  std::thread nodo1Thread(func1);
  


//vARIABLE




//CLIENTE PARA MANDAR UN 1 A ROBOT2
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1 =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
auto request_1 = std::make_shared<ur_msgs::srv::SetIO::Request>();


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
  msg.position.z = 0.410;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");

  rclcpp::sleep_for(std::chrono::seconds(2));
  std::cout<< std::endl <<pin1 << std::endl;
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();



if(pin1==1 && pin0==0)
{


  //CARGAMOS EN EL PLAN LA POSICION INICIAL
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


//PONER A 0 D0OUT
 request_1->fun= 1; 
 request_1->pin= 0; //16 abrir y 17 cerrar
 request_1->state= 0; //Funciona por flanco

 while (!client_1->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result_1 = client_1 ->async_send_request(request_1);

 if (rclcpp::spin_until_future_complete(node, result_1) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }


//Mover el ROBOT
move_group_interface.move();


//PONER A 1 EL ROBOT2, MANDAR UN D0OUT=1
 request_1->fun= 1; 
 request_1->pin= 0; //16 abrir y 17 cerrar
 request_1->state= 1; //Funciona por flanco

 while (!client_1 ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result_1 = client_1 ->async_send_request(request_1);

 if (rclcpp::spin_until_future_complete(node, result_1) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }
}




nodo1Thread.join();
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
