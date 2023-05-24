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
#include <vector>


using namespace std::chrono_literals;
using namespace std;

int main(int argc, char * argv[])
{
rclcpp::init(argc,argv);
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hello_moveit_ur");  // CHANGE



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


// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
move_group_interface.setPlannerId("ESTkConfigDefault");
move_group_interface.setPoseReferenceFrame("base");

//Pos home:
move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

//Pos1
move_group_interface.setPoseTarget(target_pose1);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();

//CREANDO UN CLIENTE PARA CARTESIAN PATH

const double eef_step = 0.01;  // Tolerancia de error para los movimientos cartesianos
const double jump_threshold = 0.0;  // Umbral de salto permitido

std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);


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
        move_group_interface.move();
    }
    else
    {
        // Falla: no se pudo calcular la trayectoria
        std::cout << "No se pudo calcular la trayectoria." << std::endl;
    }

success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.execute(trajectory);

move_group_interface.setPoseTarget(target_pose1);
success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.execute(trajectory);


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
