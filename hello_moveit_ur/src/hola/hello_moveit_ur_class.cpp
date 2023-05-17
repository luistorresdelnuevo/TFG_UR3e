#include <memory>
#include<iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

using moveit::planning_interface::MoveGroupInterface;


class PickAndPlace : public rclcpp::Node
{
private:
  moveit::planning_interface::MoveGroupInterface move_group_interface;
public:
    PickAndPlace(): Node(std::make_shared<rclcpp::Node>("hello_moveit")),move_group_interface("ur_manipulator"){
        // Inicializar el grupo de movimientos
        move_group_interface.setPlanningTime(10.0);
        move_group_interface.allowReplanning(true);
        move_group_interface.setPlannerId("PRMstarkConfigDefault");
        move_group_interface.setPoseReferenceFrame("base");
    }

    // Metodo para realizar un movimiento de pick and place
    void Movement()
    {
      // Create a ROS logger

      //MOVIMIENTO DE INICIO
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
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
        return msg;}();

        move_group_interface.setPoseTarget(target_pose1);

        success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        move_group_interface.move();

        //RCLCPP_INFO(logger, move_group_interface.getPlannerId());
    }

};




int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  // Create a ROS logger
  auto node = std::make_shared<rclcpp::Node>("hello_moveit");

  //Create an object
  PickAndPlace pick_and_place(node);

  pick_and_place.Movement();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
