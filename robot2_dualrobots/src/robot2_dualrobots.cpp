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
#include<thread>
#include<chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

auto request_1 = std::make_shared<ur_msgs::srv::SetIO::Request>();

auto request_gripper = std::make_shared<ur_msgs::srv::SetIO::Request>();


pthread_mutex_t mutex;


int npaso=0;
int npieza=0;
bool FinWhile=false;







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
    pin0=(msg->digital_in_states[0].state);
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




///Señal para funcionar robot1
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
  pose.position.z+=0,015;
}




int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot2_dualrobots",
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
//pthread_mutex_init(&mutex, NULL);

//vARIABLE
//CLIENTE PARA MANDAR UN 1 A ROBOT2
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_1 =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE
//CLIENTE PARA ABRIR Y CERRAR GRIPPER
rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_gripper =node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");          // CHANGE



 // Create the MoveIt MoveGroup Interface
 using moveit::planning_interface::MoveGroupInterface;
 //Se crea instancia de un objeto ur_manipulator
 auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
 //Se declara una variable my_plan_arm, que se utiliza para almacenar el plan de movimiento
 moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
 //Establece como marco de referencia el link base para la planificacion del objetivo
 move_group_interface.setPoseReferenceFrame("base");
 move_group_interface.setPlannerId("PRMkConfigDefault");


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

NoSignalR1(client_1,node);

//POSICIONES
 //pos1

  auto const target_pose0 = []{
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




 auto const target_pose1 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.740;
  float Ry=2.618;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.419;  //BARRA ROJA
  msg.position.y = -0.180;  //BARRA VERDE
  msg.position.z = 0.300;  //arriba, abajo   BARRA-AZUL
  return msg;
 }();


 //POSICIONES
 //posicion intercambio ESTO SE REALIZA CON CARTESIAN PATH ENTRE 1 Y 2
 auto const target_pose2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.740;
  float Ry=2.618;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.419;  //BARRA ROJA
  msg.position.y = -0.180;  //BARRA VERDE
  msg.position.z = 0.140;  //arriba, abajo   BARRA-AZUL
  return msg;
 }();


 //COLOCACION PIEZA EN EL AIRE
 //pos3
 auto target_pose3 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.740;
  float Ry=2.618;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.245;  //BARRA ROJA
  msg.position.y = -0.025;  //BARRA VERDE
  msg.position.z = 0.390;  //arriba, abajo   BARRA-AZUL
  return msg;
 }();


 //POSICION PARA REALIZAR CARTESIAN PATH DE RETIRADA
 //pos4
 auto target_pose4 = []{
  geometry_msgs::msg::Pose msg;


  float Rx=1.740;
  float Ry=2.618;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.245;  //BARRA ROJA
  msg.position.y = -0.025;  //BARRA VERDE
  msg.position.z = 0.156;  //arriba, abajo   BARRA-AZUL
  return msg;
 }();

  auto target_pose1_2 = []{
  geometry_msgs::msg::Pose msg;

  float Rx=1.740;
  float Ry=2.618;
  float Rz=0.000;
  float m = sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
  msg.orientation.x =  (Rx/m) *sin(m/2);  //0.924
  msg.orientation.y = (Ry/m) * sin(m/2);  //0.924
  msg.orientation.z = (Rz/m) * sin(m/2);  //0.924
  msg.orientation.w = cos(m/2);  //0.383

  msg.position.x = -0.252;  //BARRA ROJA
  msg.position.y = -0.256;  //BARRA VERDE
  msg.position.z = 0.208;  //arriba, abajo   BARRA-AZUL
  return msg;
 }();



if(npaso==0)
  {
    do{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SignalR1(client_1,node);
   }while(odom_subs_node->getPin0()!=true);
  }

//executor2.spin();
//LOGICA DE MOVMIENTO
while(FinWhile!=true)
{
  //std::cout<<"Perro"<<std::endl;
  if(npaso==0 && odom_subs_node->getPin1()==true && odom_subs_node->getPin0()==true)
  {
  std::this_thread::sleep_for(std::chrono::seconds(1));
   NoSignalR1(client_1,node);//pone pin1 a 0
   move_group_interface.setPlannerId("PRMkConfigDefault");
   move_group_interface.setPoseReferenceFrame("base");
   move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   //Abrimos gripper
   AbrirGripper(client_gripper,node);


   
   //Senal a R1 , para que empiece su movimiento
   do{
    SignalR1(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin0()==false);

   npaso++;
  }



  else if(npaso==1 && odom_subs_node->getPin0()==true)
  {
   NoSignalR1(client_1,node);//pone pin1 a 0
    //Movimiento a intercambio y realizar cartesian path
    move_group_interface.setPlannerId("PRMstarkConfigDefault");
    move_group_interface.setPoseTarget(target_pose0);
    bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();

    move_group_interface.setPoseTarget(target_pose1);
    success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();


    //CREANDO UN CLIENTE PARA CARTESIAN PATH
    waypoints.push_back(target_pose1);
    waypoints.push_back(target_pose2);



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

    CerrarGripper(client_gripper,node);

    npieza++;


   do{
    SignalR1(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin0()!=false);

   npaso++;
  }




  else if(npaso==2 && odom_subs_node->getPin0()==true)
  {
   NoSignalR1(client_1,node);//pone pin1 a 0

   move_group_interface.setPlannerId("PRMkConfigDefault");

   move_group_interface.setPoseTarget(target_pose3);
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();
 

   waypoints2.push_back(target_pose3);
   waypoints2.push_back(target_pose4);



   moveit_msgs::msg::RobotTrajectory trajectory2; //Almacena trayectoria
   my_plan_arm.trajectory_=trajectory2;


   fraction2 = move_group_interface.computeCartesianPath(waypoints2, eef_step2, jump_threshold2, trajectory2);
   my_plan_arm.trajectory_=trajectory2;
   move_group_interface.setPlannerId("PRMkConfigDefault");

    if (fraction2 == 1.0)
    {
        // Éxito: la trayectoria se calculó correctamente
        // Hacer algo con la trayectoria resultante
        // Por ejemplo:
        std::cout << "Trayectoria calculada exitosamente." << std::endl;
        std::cout << "Número de puntos en la trayectoria: " << trajectory2.joint_trajectory.points.size() << std::endl;
        success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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

     AbrirGripper(client_gripper,node);

     CambiarZPiezas(target_pose4);

   do{
    SignalR1(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin0()!=false);

   npaso++;
  }


  else if(npaso==3 && odom_subs_node->getPin0()==true)
  {
   NoSignalR1(client_1,node);//pone pin1 a 0

   move_group_interface.setPoseTarget(target_pose3);
   bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   move_group_interface.setPoseTarget(target_pose1_2);
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();

   move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
   success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   move_group_interface.move();


   do{
    SignalR1(client_1,node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
   }while(odom_subs_node->getPin0()!=false);

   if (npieza<5)
        {
          //CambiarXPiezas(target_pose3);
          //CambiarXPiezas(target_pose4);
          waypoints.clear();
          waypoints2.clear();
          npaso=0;
        }
        else
        {
          FinWhile=true;
        }

   do{
   }while(odom_subs_node->getPin0()==false);
  }





  else if(npaso==4 && odom_subs_node->getPin0()==true)
  {
    NoSignalR1(client_1,node);//pone pin1 a 0
    do{
        if (npieza<5)
        {
          //CambiarXPiezas(target_pose3);
          //CambiarXPiezas(target_pose4);
          waypoints.clear();
          waypoints2.clear();
          npaso=-1;
        }
        else
        {
          FinWhile=true;
        }


    }while(odom_subs_node->getPin0()!=true);
    
    SignalR1(client_1,node);//pone pin1 a 0

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
