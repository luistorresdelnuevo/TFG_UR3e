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
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

auto request_1 = std::make_shared<ur_msgs::srv::SetIO::Request>();



pthread_mutex_t mutex;



int npaso=0;


//THIS IS ROBOT2
//Interrupción que lee el valor del pin0IN y pin1OUT constantemente del topic /io_status_controller
class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber(std::string odom_topic_name) : Node("odom_subscriber") {

    subscription_ = this->create_subscription<ur_msgs::msg::IOStates>(
        odom_topic_name, 10,
        std::bind(&OdomSubscriber::callback, this,
                  std::placeholders::_1));
  }
  bool getPin0()
  {
    return pin0;
  }

  bool getPin1()
  {
    return pin1;
  }


private:
  void callback(const ur_msgs::msg::IOStates::SharedPtr msg){
  // Aquí puedes procesar los datos recibidos
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_in_states[0].state);//pIN QUE SE ACTIVA AL TERMINAR ROBOT2 Y PARA EMPEZAR
  pin0=int(msg->digital_in_states[0].state);
  //mIENTRAS ROBOT1 SE MUEVE MANDAMOS UN PONEMOS D0OUT PARA QUE SE RESETEE.
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_out_states[1].state);//CUANDO ACABA EL ROBOT1 SU TAREA MANDAMOS UN 1 A ROBOT2
  pin1=(msg->digital_out_states[1].state);

  std::cout<<std::endl<<npaso<<std::endl;
  std::cout<<pin0<<std::endl;
  std::cout<<pin1<<std::endl;

}
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr subscription_;
  bool pin0=false;//output
  bool pin1=false;//input
};


/*
void callback(const ur_msgs::msg::IOStates::SharedPtr msg){
  // Aquí puedes procesar los datos recibidos
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_in_states[0].state);//pIN QUE SE ACTIVA AL TERMINAR ROBOT2 Y PARA EMPEZAR
  pin1=int(msg->digital_in_states[0].state);
  //mIENTRAS ROBOT1 SE MUEVE MANDAMOS UN PONEMOS D0OUT PARA QUE SE RESETEE.
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_out_states[1].state);//CUANDO ACABA EL ROBOT1 SU TAREA MANDAMOS UN 1 A ROBOT2
  pin0=(msg->digital_out_states[1].state);

  std::cout<<std::endl<<npaso<<std::endl;

}
*/

///Señal para funcionar robot2
void SignalR1(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1, auto const node){

 request_1->fun= 1; 
 request_1->pin= 1;
 request_1->state= 1; ///Se pone a 1 el pin 0 para que R1 FUNCIONE
  while (!client_1 ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result = client_1 ->async_send_request(request_1);

 if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }
}

//señal para que PARE DE FUNCIONAR no funcione R1
void NoSignalR1(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1, auto const node){

 request_1->fun= 1; 
 request_1->pin= 1;
 request_1->state= 0; ///Se pone a 1 el pin 0 para que R1 FUNCIONE
  while (!client_1 ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result = client_1 ->async_send_request(request_1);

 if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }
}



//Funcion que cambia el movimiento de piezas progresivamente
void CambiarZPiezas(geometry_msgs::msg::Pose& pose){
  pose.position.z-=0.01;
}




int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit_ur",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

// Instantiate the odometry subscriber node
  std::shared_ptr<OdomSubscriber> odom_subs_node =
      std::make_shared<OdomSubscriber>("/io_and_status_controller/io_states");

/*
  auto subscription = node1->create_subscription<ur_msgs::msg::IOStates>(
    "/io_and_status_controller/io_states",  // Nombre del topic
    10,callback  // Tamaño de la cola de mensajes// Función de callback
  );
*/



///REALIZAR HILOS DE EJECUCIÓN
  //rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  //executor.add_node(node);
  executor.add_node(odom_subs_node);
  //executor.add_node(node);

std::thread thread1([&executor]() {
  executor.spin();
});
//pthread_mutex_init(&mutex, NULL);

//vARIABLE
//CLIENTE PARA MANDAR UN 1 A ROBOT2
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1 =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE



// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


//Caminos Cartesianos
//CREANDO UN CLIENTE PARA CARTESIAN PATH
const double eef_step = 0.01;  // Tolerancia de error para los movimientos cartesianos
const double jump_threshold = 0.0;  // Umbral de salto permitido
std::vector<geometry_msgs::msg::Pose> waypoints;
std::vector<geometry_msgs::msg::Pose> waypoints2;
double fraction,fraction1;
waypoints.clear();//usar para limpiar el vector

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


//POSICIONES
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
  msg.position.z = 0.410;  //arriba, abajo   BARRA-AZUL
  return msg;
}();



//POSICIONES
//pos3
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
  msg.position.z = 0.410;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


//POSICIONES
//pos4
auto const target_pose4 = []{
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



//POSICIONES
//pos5
auto const target_pose5 = []{
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



//POSICIONES
//pos6
auto const target_pose6 = []{
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


//POSICIONES
//pos7
auto const target_pose7 = []{
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



if(npaso==0)
  {
      SignalR1(client_1,node);
  }

//executor2.spin();
//LOGICA DE MOVMIENTO
while(1)
{
  //std::cout<<"Perro"<<std::endl;
  if(npaso==0 && odom_subs_node->getPin1()==true && odom_subs_node->getPin0()==true)
  {
  //NoSignalR1(client_1,node);//pone pin1 a 0
  move_group_interface.setPlannerId("ESTkConfigDefault");
  move_group_interface.setPoseReferenceFrame("base");
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.move();

  //Señal a R1
  
  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;
  }
  else if(npaso==1 && odom_subs_node->getPin0()==true)
  {
  NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento de pieza a intercambio y abrir pinza
move_group_interface.setPoseTarget(target_pose1);
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;
  }
  else if(npaso==5 && odom_subs_node->getPin0()==true)
  {
  NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento de pieza a intercambio y abrir pinza
move_group_interface.setPoseTarget(target_pose1);
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;
  }
  else if(npaso==4 && odom_subs_node->getPin0()==true)
  {
//Cerrar Gripper 
   NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento de pieza a intercambio y abrir pinza
  move_group_interface.setPlannerId("ESTkConfigDefault");
  move_group_interface.setPoseReferenceFrame("base");
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.move();


  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;
  }
  else if(npaso==3 && odom_subs_node->getPin0()==true)
  {
  NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento de pieza a intercambio y abrir pinza
move_group_interface.setPoseTarget(target_pose1);
bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group_interface.move();


  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;
  }
  else if(npaso==2 && odom_subs_node->getPin0()==true)
  {
  NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento de pieza a intercambio y abrir pinza
  move_group_interface.setPlannerId("ESTkConfigDefault");
  move_group_interface.setPoseReferenceFrame("base");
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.move();


  SignalR1(client_1,node);
  NoSignalR1(client_1,node);
  npaso++;

  }
}



  thread1.join();
  //rclcpp::spin(node);
  //pthread_mutex_destroy(&mutex);
  // Shutdown ROS
  //rclcpp::spin(node);//Posible candidato a quitarlo.
  rclcpp::shutdown();
  return 0;
}
