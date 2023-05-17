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
//#include "ur_msgs/SetIORequest.h"
//#include "ur_msgs/SetIOResponse.h"

/*
void abrirgripper(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}


void cerrargripper(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}


using namespace std::chrono_literals;


class IOClient : public rclcpp::Node
{

  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<ur_msgs::srv::set_io>::SharedPtr cli_;
  size_t count_;
  const std::shared_ptr<ur_msgs::srv::set_io::Request> request
  //auto request = std::make_shared<ur_msgs::srv::set_io::Request>();


  public:
  IOClient(): Node("client"), count_(0)
  {
    cli_ = this->create_client()<ur_msgs::srv::set_io>(SetIO, 'set_io');
    timer_ = this->create_wall_timer( 500ms, std::bind(&IOClient::timer_callback, this));
  }

  void send_request()
  {
    request.fun = SetIO.FUN_SET_DIGITAL_OUT;
    request.pin = SetIO.PIN_TOOL_DOUT1;
    request.state = SetIO.STATE_ON;
  }
}
*/


using namespace std::chrono_literals;

int main(int argc, char * argv[])
{

  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  /*
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
*/

 



//CREANDO UN CLIENTE PARA GRIPPER

std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hello_moveit_ur");  // CHANGE
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_ =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE

 //auto const logger = rclcpp::get_logger("hello_moveit");

auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
request->fun= 1; 
request->pin= 17; //16 abrir y 17 cerrar
request->state= 1; //Funciona por flanco

while (!client_ ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
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




  // Create a ROS logger


/*
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");


move_group_interface.setPlanningTime(5);
move_group_interface.setPlannerId("ESTkConfigDefault");
RCLCPP_INFO(logger, move_group_interface.getPlannerId());

// Set a target Pose
//Posicion y orientacion esta entre base_linkinertia y tool0(en ROS2) y en la tablet es entre base y tool0
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.972;
  float Ry=2.455;
  float Rz=-0.078;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.321;  //BARRA ROJA
  msg.position.y = -0.151;  //BARRA VERDE
  msg.position.z = 0.313;  //arriba, abajo   BARRA-AZUL
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);
//move_group_interface.setOrientationTarget(0.865,-0.500,-0.037,-0.027);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);  
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
  move_group_interface.execute(plan);
}
*/

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
