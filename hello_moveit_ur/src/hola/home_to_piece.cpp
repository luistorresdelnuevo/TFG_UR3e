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
  ofstream archivo("PLANIFICADORES_HOME_PIEZA.txt");
  if(!archivo.is_open()){
    std::cout <<"Error al abrir el archivo"<<std::endl;
    return 1;
  }




  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");



//CREAR EL PLAN
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;







//Características del ESTkConfigDefault
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;

for(int i=0;i<1000;i++)
{

//Lo movemos a la posicion inicial.
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));

std::cout<< "Comeme el pie." << std::endl;
std::cout<<"El tiempo de planeamiento es: "<<move_group_interface.getPlanningTime()<<std::endl;
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

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
  msg.position.z = 0.210;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

move_group_interface.setPoseTarget(target_pose1);

std::cout<< "Desde Home a la posición indicada." << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo<< my_plan_arm.planning_time_ <<std::endl; //Imprime el tiempo de planeamiento
move_group_interface.move();
}
archivo<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;








//Características del RRTConnectkConfigDefault
move_group_interface.setPlannerId("RRTConnectkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;

for(int i=0;i<1000;i++)
{

//Lo movemos a la posicion inicial.
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));

std::cout<< "Comeme el pie." << std::endl;
std::cout<<"El tiempo de planeamiento es: "<<move_group_interface.getPlanningTime()<<std::endl;
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

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
  msg.position.z = 0.210;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

move_group_interface.setPoseTarget(target_pose1);

std::cout<< "Desde Home a la posición indicada." << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo<< my_plan_arm.planning_time_ <<std::endl; //Imprime el tiempo de planeamiento
move_group_interface.move();
}
archivo<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;








//Características del TRRTkConfigDefault
move_group_interface.setPlannerId("TRRTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;

for(int i=0;i<1000;i++)
{

//Lo movemos a la posicion inicial.
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));

std::cout<< "Comeme el pie." << std::endl;
std::cout<<"El tiempo de planeamiento es: "<<move_group_interface.getPlanningTime()<<std::endl;
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

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
  msg.position.z = 0.210;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

move_group_interface.setPoseTarget(target_pose1);

std::cout<< "Desde Home a la posición indicada." << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo<< my_plan_arm.planning_time_ <<std::endl; //Imprime el tiempo de planeamiento
move_group_interface.move();
}
archivo<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;







//Características del PRMkConfigDefault
move_group_interface.setPlannerId("PRMkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");
archivo<<"El planificador: "<<move_group_interface.getPlannerId()<<std::endl;

for(int i=0;i<1000;i++)
{

//Lo movemos a la posicion inicial.
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));

std::cout<< "Comeme el pie." << std::endl;
std::cout<<"El tiempo de planeamiento es: "<<move_group_interface.getPlanningTime()<<std::endl;
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

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
  msg.position.z = 0.210;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

move_group_interface.setPoseTarget(target_pose1);

std::cout<< "Desde Home a la posición indicada." << std::endl;
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
archivo<< my_plan_arm.planning_time_ <<std::endl; //Imprime el tiempo de planeamiento
move_group_interface.move();
}
archivo<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;



  // Shutdown ROSsss
  rclcpp::shutdown();
  return 0;
}
