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

auto request_gripper = std::make_shared<ur_msgs::srv::SetIO::Request>();


pthread_mutex_t mutex;


int npaso=0;
int npieza=0;
bool FinWhile=false;


//THIS IS ROBOT1
//Interrupción lee el valor del PIN0OUT Y DE PIN1IN, constantemente del topic /io_status_controller
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
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_out_states[0].state);//pIN QUE SE ACTIVA AL TERMINAR ROBOT2 Y PARA EMPEZAR
  pin0=int(msg->digital_out_states[0].state);
  //mIENTRAS ROBOT1 SE MUEVE MANDAMOS UN PONEMOS D0OUT PARA QUE SE RESETEE.
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Mensaje recibido: %d", msg->digital_in_states[1].state);//CUANDO ACABA EL ROBOT1 SU TAREA MANDAMOS UN 1 A ROBOT2
  pin1=(msg->digital_in_states[1].state);

  std::cout<<"PASO "<<npaso<<std::endl;
  std::cout<<"PIN0 "<<pin0<<std::endl;
  std::cout<<"PIN1 "<<pin1<<std::endl;

}
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr subscription_;
  bool pin0=false;//output
  bool pin1=false;//input
};




//Funcion cerrar gripper
void CerrarGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_gripper, auto const node)
{

  pthread_mutex_lock(&mutex);
request_gripper->fun= 1; 
request_gripper->pin= 16; //16 abrir y 17 cerrar
request_gripper->state= 0; //Funciona por flanco

while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

request_gripper->fun= 1; 
request_gripper->pin= 17; //16 abrir y 17 cerrar
request_gripper->state= 0; //Funciona por flanco

while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

request_gripper->fun= 1; 
request_gripper->pin= 17; //16 abrir y 17 cerrar
request_gripper->state= 1; //Funciona por flanco

while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

  pthread_mutex_unlock(&mutex);
}



void AbrirGripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_gripper, auto const node)
{
   pthread_mutex_lock(&mutex);


request_gripper->fun= 1; 
request_gripper->pin= 16; //16 abrir y 17 cerrar
request_gripper->state= 0; //Funciona por flanco
  while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 auto result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

request_gripper->fun= 1; 
request_gripper->pin= 17; //16 abrir y 17 cerrar
request_gripper->state= 0; //Funciona por flanco

while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }

request_gripper->fun= 1; 
request_gripper->pin= 16; //16 abrir y 17 cerrar
request_gripper->state= 1; //Funciona por flanco

while (!client_gripper ->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 result_gripper = client_gripper ->async_send_request(request_gripper);

if (rclcpp::spin_until_future_complete(node, result_gripper) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio de gripper funcionando");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call serviece");
  }
  pthread_mutex_unlock(&mutex);

}




///Señal para funcionar robot2
void SignalR2(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1, auto const node){

 request_1->fun= 1; 
 request_1->pin= 0;
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




void NoSignalR2(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1, auto const node){

 request_1->fun= 1; 
 request_1->pin= 0;
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
  pose.position.z-=0.015;
}




int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot1_dualrobots",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Instantiate the odometry subscriber node
  std::shared_ptr<OdomSubscriber> odom_subs_node =
      std::make_shared<OdomSubscriber>("/io_and_status_controller/io_states");

///REALIZAR HILOS DE EJECUCIÓN
  //rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  //executor.add_node(node);
  executor.add_node(odom_subs_node);
  //executor.add_node(node);

std::thread thread1([&executor]() {
  executor.spin();
});



//vARIABLE
//CLIENTE PARA MANDAR UN 1 A ROBOT2
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1 =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE
//CLIENTE PARA ABRIR Y CERRAR GRIPPER
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_gripper =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE




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
double fraction,fraction2;
waypoints.clear();//usar para limpiar el vector
waypoints2.clear();
const double eef_step2 = 0.01;  // Tolerancia de error para los movimientos cartesianos
const double jump_threshold2 = 0.0;  // Umbral de salto permitido


NoSignalR2(client_1,node);//pone pin0 a 0

//POSICIONES
//pos1
auto const target_pose1 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.742;
  float Ry=2.614;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.245;  //BARRA ROJA
  msg.position.y = -0.135;  //BARRA VERDE
  msg.position.z = 0.400;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


//POSICIONES
//pieza 1, siguiente pieza se va decrementando z 0,021
auto target_pose2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.742;
  float Ry=2.614;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.245;  //BARRA ROJA
  msg.position.y = -0.135;  //BARRA VERDE
  msg.position.z = 0.211;  //arriba, abajo   BARRA-AZUL
  return msg;
}();

//home

//POSICION INTERCAMBIO
//pos3
auto const target_pose3 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=0.692;
  float Ry=-3.064;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.028;  //BARRA ROJA
  msg.position.y = -0.511;  //BARRA VERDE
  msg.position.z = 0.230;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


//POSICION PARA REALIZAR CARTESIAN PATH DE RETIRADA
//pos4
auto const target_pose4 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=0.692;
  float Ry=-3.064;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = 0.028;  //BARRA ROJA
  msg.position.y = -0.511;  //BARRA VERDE
  msg.position.z = 0.150;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


auto const target_pose5 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=0.692;
  float Ry=-3.064;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.218;  //BARRA ROJA
  msg.position.y = -0.337;  //BARRA VERDE
  msg.position.z = 0.186;  //arriba, abajo   BARRA-AZUL
  return msg;
}();


if(npaso==0)
  {
      SignalR2(client_1,node);
  }


//LOGICA DE MOVMIENTO

while(FinWhile!=true)
{
 if(npaso==0 && odom_subs_node->getPin1()==true && odom_subs_node->getPin0()==true)
 {
  //Movimiento de R1 a HOME
  std::this_thread::sleep_for(std::chrono::seconds(1));
  NoSignalR2(client_1,node);//pone pin0 a 0
   move_group_interface.setPlannerId("PRMkConfigDefault");
   move_group_interface.setPoseReferenceFrame("base");
   move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();
  //Abrimos gripper
  AbrirGripper(client_gripper,node);
  std::this_thread::sleep_for(std::chrono::seconds(2));//espera durante dos segundos

  npaso++;
 }


 else if(npaso==1 && odom_subs_node->getPin1()==true)
 {
  //Movimiento a pieza de R1 A PIEZA
   NoSignalR2(client_1,node);//pone pin0 a 0
   move_group_interface.setPlannerId("PRMstarkConfigDefault");

   move_group_interface.setPoseTarget(target_pose1);
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   move_group_interface.setPlannerId("PRMkConfigDefault");
   move_group_interface.setPoseTarget(target_pose2);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   //Cerrar Gripper
   CerrarGripper(client_gripper,node);

   //Contador de PIEZA
   npieza++;
  

  waypoints.push_back(target_pose2);
  waypoints.push_back(target_pose1);



  moveit_msgs::msg::RobotTrajectory trajectory; //Almacena trayectoria
  my_plan_arm.trajectory_=trajectory;


  fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
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
        std::cout << "No se pudo calcular la trayectoria y se acaba el programa." << std::endl;
        FinWhile=true;

  }

   move_group_interface.setPoseTarget(target_pose3);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   move_group_interface.setPlannerId("PRMstarkConfigDefault");

   move_group_interface.setPoseTarget(target_pose4);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

  
  AbrirGripper(client_gripper,node);

  waypoints2.push_back(target_pose4);
  waypoints2.push_back(target_pose3);

  //Cambiar Z piezas

  CambiarZPiezas(target_pose2);


  moveit_msgs::msg::RobotTrajectory trajectory2; //Almacena trayectoria
  my_plan_arm.trajectory_=trajectory2;


  fraction2 = move_group_interface.computeCartesianPath(waypoints2, eef_step2, jump_threshold2, trajectory2);
  my_plan_arm.trajectory_=trajectory2;

    if (fraction2 == 1.0)
    {
        // Éxito: la trayectoria se calculó correctamente
        // Hacer algo con la trayectoria resultante
        // Por ejemplo:
        std::cout << "Trayectoria calculada exitosamente." << std::endl;
        std::cout << "Número de puntos en la trayectoria: " << trajectory2.joint_trajectory.points.size() << std::endl;
        bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        move_group_interface.execute(trajectory2);
    }
    else
    {
        // Falla: no se pudo calcular la trayectoria
        std::cout << "No se pudo calcular la trayectoria." << std::endl;
        std::cout << "Número de puntos en la trayectoria: " << trajectory2.joint_trajectory.points.size() << std::endl;
        std::cout <<  "Valor de fraction; "<< fraction2<< std::endl;
        std::cout << "No se pudo calcular la trayectoria y se acaba el programa." << std::endl;
        FinWhile=true;
    }

   move_group_interface.setPlannerId("PRMstarkConfigDefault");


    move_group_interface.setPoseTarget(target_pose5);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   move_group_interface.setPoseTarget(target_pose1);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();




  //mover un poco para poder realizar un nuevo camino cartesiano.
   do{
    SignalR2(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin1()!=false);


   npaso++;
 }



 else if(npaso==2 && odom_subs_node->getPin1()==true)
 {
  NoSignalR2(client_1,node);//pone pin0 a 0
  

   move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   std::this_thread::sleep_for(std::chrono::seconds(2));//espera durante dos segundos
   do{
    SignalR2(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin1()!=false);

   std::this_thread::sleep_for(std::chrono::seconds(1));//espera durante dos segundos

   npaso++;
 }



 else if(npaso==3 && odom_subs_node->getPin1()==true)
 {
  NoSignalR2(client_1,node);//pone pin0 a 0
  std::this_thread::sleep_for(std::chrono::seconds(3));



  do{
    SignalR2(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }while(odom_subs_node->getPin1()!=false);

  std::this_thread::sleep_for(std::chrono::seconds(2));//espera durante dos segundos

   npaso++;
 }



 else if(npaso==4 && odom_subs_node->getPin1()==true)
 {
   NoSignalR2(client_1,node);//pone pin0 a 0

   if (npieza<5)
   {
    //CambiarZPiezas(target_pose2);
    waypoints.clear();
    waypoints2.clear();
    
      
     SignalR2(client_1,node);
     std::this_thread::sleep_for(std::chrono::seconds(2));
     npaso=0;
   }
   else
   {
      FinWhile=true;
   }


  //Resetear valores



  std::this_thread::sleep_for(std::chrono::seconds(2));//espera durante dos segundos
 }

 else if(npaso==-1 && odom_subs_node->getPin0()==true)
 {
    SignalR2(client_1,node);
      do{
        std::this_thread::sleep_for(std::chrono::seconds(2));
     }while(odom_subs_node->getPin1()!=true);
    npaso++;
 }
}



  thread1.join();
  //pthread_mutex_destroy(&mutex);
  // Shutdown ROS
  //rclcpp::spin(node);//Posible candidato a quitarlo.
  rclcpp::shutdown();
  return 0;
}
