#include <memory>
#include<iostream>
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

using namespace std;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  //Abrimos el fichero en modo escritura.
  ofstream archivo1("PLANIFICADOR_CAMBIO_CUADRANTE_1_a_2.txt");//1 es la posicion que toma tras el home
  if(!archivo1.is_open()){
    std::cout <<"Error al abrir el archivo"<<std::endl;
    return 1;
  }

  ofstream archivo2("PLANIFICADOR_CAMBIO_CUADRANTE_2_a_1.txt");//2 es la posición que toma tras el ir a 1 primero
  if(!archivo2.is_open()){
    std::cout <<"Error al abrir el archivo"<<std::endl;
    return 1;
  }
  archivo1<<"MOVIMIENTOS DE POSICION 1 A 2 "<<std::endl;
  archivo2<<"MOVIMIENTOS DE POSICIÓN 2 A 1 "<<std::endl;





  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");



//CREAR EL PLAN
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


//Lo movemos a la posicion inicial.
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));

bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();
move_group_interface.setPoseReferenceFrame("base");


//Posicion cuadrante 1
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

  msg.position.x = -0.142;  //BARRA ROJA
  msg.position.y = -0.128;  //BARRA VERDE
  msg.position.z = 0.310;  //arriba, abajo   BARRA-AZUL
  return msg;
}();
//std::this_thread::sleep_for(std::chrono::seconds(2));
move_group_interface.setPoseTarget(target_pose1);
std::cout<< "Desde Home a la posicion inicial" << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();





//Posicion cuadrante 2
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

  msg.position.x = 0.142;  //BARRA ROJA
  msg.position.y = -0.328;  //BARRA VERDE
  msg.position.z = 0.310;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


for(int j=0;j<4;j++){

if(j==0)
{
//Características del ESTkConfigDefault
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo1<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
archivo2<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
}

else if(j==1)
{
//Características del RRTConnectkConfigDefault
move_group_interface.setPlannerId("RRTConnectkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo1<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
archivo2<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
}

else if(j==2)
{
//Características del TRRTkConfigDefault
move_group_interface.setPlannerId("TRRTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo1<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
archivo2<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
}

else if(j==3)
{
  //Características del PRMkConfigDefault
move_group_interface.setPlannerId("PRMkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo1<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
archivo2<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;
}
else{ std::cout<<"ERROR"<<std::endl;}



//MOVIMIENTOS
for(int i=0;i<1000;i++)
{

//posicion 1 a 2
move_group_interface.setPoseTarget(target_pose2);
std::cout<< "Desde Posición1 a Posición2" << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo1<< my_plan_arm.planning_time_ <<std::endl;
move_group_interface.move();


//posicion 2 a 1
move_group_interface.setPoseTarget(target_pose1);
std::cout<< "Desde Home a la posicion inicial" << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo2<< my_plan_arm.planning_time_ <<std::endl;
move_group_interface.move();

}

archivo1<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
archivo2<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
}

archivo1.close();
archivo2.close();
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
