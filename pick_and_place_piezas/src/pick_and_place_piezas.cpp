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


/*
auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
 //Crea un cliente que llama al servicio 





void CerrarGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node)
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
//spin_until_future_complete bloquea la ejecución del programa hasta que el servicio termine
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


}


void AbrirGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_, auto const node){
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
*/


void CambiarZPiezas(geometry_msgs::msg::Pose& pose){
  pose.position.z-=0.021;
}



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  

auto const node = std::make_shared<rclcpp::Node>(//Funcion que crea un objeto y devuelve puntero a ese objeto.
  "pick_and_place_piezas", //Nompre del nodo
  //Los parámetros del nodo se declaran a partir de las sobreescrituras de las siguientes lienas
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);


pthread_mutex_init(&mutex, NULL);


rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_ =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");


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

//pos2
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

//Pos3
auto target_pose3 = []{
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


//Pos4
auto const target_pose4 = []{
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




//posfin1
auto const posfin1 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.739;
  float Ry=2.167;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.165;  //BARRA ROJA
  msg.position.y = -0.379;  //BARRA VERDE
  msg.position.z = 0.152;  //arriba, abajo   BARRA-AZUL
  return msg;
}();




//Posfin2
auto const posfin2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=0.549;
  float Ry=-3.093;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.142;  //BARRA ROJA
  msg.position.y = -0.429;  //BARRA VERDE
  msg.position.z = 0.152;  //arriba, abajo   BARRA-AZUL
  return msg;
}();



//Posfin3
auto const posfin3 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=2.635;
  float Ry=-1.711;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.090;  //BARRA ROJA
  msg.position.y = -0.407;  //BARRA VERDE
  msg.position.z = 0.152;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


//Posfin4
auto const posfin4 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=0.549;
  float Ry=-3.093;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.142;  //BARRA ROJA
  msg.position.y = -0.35;  //BARRA VERDE
  msg.position.z = 0.152;  //arriba, abajo   BARRA-AZUL
  return msg;
}();



// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
//Se crea instancia de un objeto ur_manipulator
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
//Se declara una variable my_plan_arm, que se utiliza para almacenar el plan de movimiento
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
//Establece como marco de referencia el link base para la planificacion del objetivo
move_group_interface.setPoseReferenceFrame("base");



const double eef_step = 0.01;  // Tolerancia de error para los movimientos cartesianos
const double jump_threshold = 0.0;  // Umbral de salto permitido
std::vector<geometry_msgs::msg::Pose> waypoints;
double fraction;



// Create a ROS logger
auto const logger = rclcpp::get_logger("pick_and_place_piezas");



//MOVEMOS POSICION INICIAL
move_group_interface.setPlannerId("PRMkConfigDefault");
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();




///PROGRAMA REAL PARA HACER BUCLE.
for(int i=0;i<4;i++)
{

  if(i==0)
  {
    waypoints.clear();
    //AbrirGripper(client_,node);
  }
  else if(i==1)
  {
    CambiarZPiezas(target_pose3);
    waypoints.clear();
  }
  else if(i==2)
  {
    CambiarZPiezas(target_pose3);
    waypoints.clear();
  }
  else if(i==3)
  {
    CambiarZPiezas(target_pose3);
    waypoints.clear();
  }





//Abrimos Gripper
//AbrirGripper(client_,node);

// Movemos a pos1 (justo encima de coger las piezas)
move_group_interface.setPlannerId("RRTstarkConfigDefault"); //Cambiamos el planificador
move_group_interface.setPoseTarget(target_pose1);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


//Bajamos a por pieza
move_group_interface.setPlannerId("PRMstarkConfigDefault"); //Cambiamos el planificador
move_group_interface.setPoseTarget(target_pose3);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


//Cerramos pinza
//CerrarGripper(client_,node);


//Subimos rectos para no dar en los bordes
waypoints.push_back(target_pose3);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose1);
moveit_msgs::msg::RobotTrajectory trajectory; //Almacena trayectoria
my_plan_arm.trajectory_=trajectory;


double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
my_plan_arm.trajectory_=trajectory;

    if (fraction == 1.0)
    {
        // Éxito: la trayectoria se calculó correctamente
        // Hacer algo con la trayectoria resultante
        // Por ejemplo:
        std::cout << "Trayectoria calculada exitosamente." << std::endl;
        std::cout << "Número de puntos en la trayectoria: " << trajectory.joint_trajectory.points.size() << std::endl;
        success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        move_group_interface.execute(trajectory);
    }
    else
    {
        // Falla: no se pudo calcular la trayectoria
        std::cout << "No se pudo calcular la trayectoria." << std::endl;
        std::cout << "Número de puntos en la trayectoria: " << trajectory.joint_trajectory.points.size() << std::endl;
        std::cout <<  "Valor de fraction; "<< fraction<< std::endl;

    }


//Colocar robot justo encima de donde va a suceder a dejarse la pieza
move_group_interface.setPlannerId("RRTstarkConfigDefault"); //Cambiamos el planificador
move_group_interface.setPoseTarget(target_pose4);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


if(i==0)
  {
    waypoints.clear();
    move_group_interface.setPlannerId("PRMstarkConfigDefault"); //Cambiamos el planificador
    move_group_interface.setPoseTarget(posfin1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    //AbrirGripper(client_,node);
  }
  else if(i==1)
  {
    waypoints.clear();
    move_group_interface.setPlannerId("PRMstarkConfigDefault"); //Cambiamos el planificador
    move_group_interface.setPoseTarget(posfin1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    //AbrirGripper(client_,node);
  }
  else if(i==2)
  {
    waypoints.clear();
    move_group_interface.setPlannerId("PRMstarkConfigDefault"); //Cambiamos el planificador
    move_group_interface.setPoseTarget(posfin1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    //AbrirGripper(client_,node);
  }
  else if(i==3)
  {
    waypoints.clear();
    move_group_interface.setPlannerId("PRMstarkConfigDefault"); //Cambiamos el planificador
    move_group_interface.setPoseTarget(posfin1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    //AbrirGripper(client_,node);
  }





  if(i<4)
  {
    move_group_interface.setPlannerId("RRTstarkConfigDefault"); //Cambiamos el planificador
    move_group_interface.setPoseTarget(target_pose1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
  }
  else{
      //MOVEMOS POSICION INICIAL
    move_group_interface.setPlannerId("PRMkConfigDefault");
    move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
    bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
  }


}
  //MOVEMOS POSICION INICIAL
  move_group_interface.setPlannerId("PRMkConfigDefault");
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.move();



  // Shutdown ROS
  pthread_mutex_destroy(&mutex);
  rclcpp::shutdown();
  return 0;
}