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

pthread_mutex_t mutex;

bool pin0=false;//output
bool pin1=false;//input

int npaso=0;


//THIS IS ROBOT1

void callback(const ur_msgs::msg::IOStates::SharedPtr msg)
{
  // Aquí puedes procesar los datos recibidos
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_in_states[1].state);//pIN QUE SE ACTIVA AL TERMINAR ROBOT2 Y PARA EMPEZAR
  pin1=msg->digital_in_states[1].state;
  //mIENTRAS ROBOT1 SE MUEVE MANDAMOS UN PONEMOS D0OUT PARA QUE SE RESETEE.
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_out_states[0].state);//CUANDO ACABA EL ROBOT1 SU TAREA MANDAMOS UN 1 A ROBOT2
  pin0=msg->digital_out_states[0].state;
}


///Señal para funcionar robot2
void SignalR2(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node){

 request->fun= 1; 
 request->pin= 0;
 request->state= 1; ///Se pone a 1 el pin 0 para que R1 FUNCIONE
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
}


void NoSignalR2(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node){

 request->fun= 1; 
 request->pin= 0;
 request->state= 0; ///Se pone a 1 el pin 0 para que R1 FUNCIONE
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
}






int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit_ur",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto subscription = node->create_subscription<ur_msgs::msg::IOStates>(
    "/io_and_status_controller/io_states",  // Nombre del topic
    10,callback  // Tamaño de la cola de mensajes// Función de callback
  );

pthread_mutex_init(&mutex, NULL);

//vARIABLE
//CLIENTE PARA MANDAR UN 1 A ROBOT2
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_ =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE
auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();


// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


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



SignalR2(client_,node);
//LOGICA DE MOVMIENTO
if(npaso==0 && pin1==true && pin0==true)
{
  //Movimiento de R1 a HOME
NoSignalR2(client_,node);//pone pin0 a 0
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();



npaso++;
}


if(npaso==1 && pin1==true && pin0==false)
{
  //Movimiento a pieza de R1 A PIEZA y abrir pinza
move_group_interface.setPoseTarget(target_pose1);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


SignalR2(client_,node);//Pone pin0 a true
npaso++;
NoSignalR2(client_,node);//pone pin0 a 0
};

if(npaso==2 && pin1==true && pin0==false)
{
//Movimiento a intercambio


SignalR2(client_,node);
npaso++;
NoSignalR2(client_,node);//pone pin0 a 0
};


if(npaso==3 && pin1==true && pin0==false)
{

//Movimiento a intercambio


SignalR2(client_,node);
npaso++;
NoSignalR2(client_,node);//pone pin0 a 0
};

if(npaso==4 && pin1==true && pin0==false)
{
//Movimiento de Abrir Pinza


SignalR2(client_,node);
npaso++;
NoSignalR2(client_,node);
};

if(npaso==5 && pin1==true && pin0==false)
{
//Movimiento de a home


SignalR2(client_,node);
npaso++;
NoSignalR2(client_,node);
};


  pthread_mutex_destroy(&mutex);
  // Shutdown ROS
  rclcpp::spin(node);//Posible candidato a quitarlo.
  rclcpp::shutdown();
  return 0;
}
