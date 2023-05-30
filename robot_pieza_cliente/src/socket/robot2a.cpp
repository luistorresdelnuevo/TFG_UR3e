

#include <memory>
#include<iostream>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
#include <string>
#include <ur_msgs/srv/set_io.hpp>
#include <functional>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


using namespace std;


int main(int argc, char * argv[])
{


  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot2",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );



  // Create the MoveIt MoveGroup Interface
  //using moveit::planning_interface::MoveGroupInterface;
  //auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  


  int sockfd;
  struct sockaddr_in local{};
  struct sockaddr_in serv{};
  //struct hostent *servidor;


  //Creacción del socket:
  sockfd=socket(AF_INET, SOCK_STREAM, 0);

  if(sockfd<0){
    perror("creando socket:");
    exit(1);
  }


  // Configurar la dirección del vliente
  local.sin_family = AF_INET;
  local.sin_port = htons(8080);
  local.sin_addr.s_addr = inet_addr("127.0.0.1");//IP del ordenador Cliente
  if (bind(sockfd, (struct sockaddr *) &local, sizeof(local))<0) 
  {
    perror("asignando direccion:");
    exit(2);
  }

  serv.sin_family = AF_INET;
  serv.sin_port = htons(8080);
  serv.sin_addr.s_addr = inet_addr("127.0.0.11");//IP del ordenador Servidor



//Crear el Socket cliente
  sockfd = socket(AF_INET, SOCK_STREAM, 0);

//conectar con el cliente

  if (connect(sockfd, (struct sockaddr*)&serv, sizeof(serv)) == -1) {
        std::cerr << "Conectando: " << std::endl;
        close(sockfd);
        return -1;
  }

  std::string startSignal = "ready";

  if (send(sockfd, startSignal.c_str(), startSignal.size(), 0) <0) 
  {
    std::cerr << "Error al enviar la señal al robot1" << std::endl;
  } 
  else 
  {
    std::cout << "Señal enviada al robot1" << std::endl;
  }

  // Recibir señal de finalización desde robot1
    char buffer[1024];
    recv(sockfd,buffer,1024,0);
    std::string signal = buffer;

  if (signal == "done") {
        std::cout << "Señal recibida desde robot1: done" << std::endl;
        //Realizar la tarea del pick'n place del robot2

        std::cout << "Realizar la tarea robot2" << std::endl;
        // Realizar las tareas de pick'n place de robot1
        //move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
        //bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //move_group_interface.move();
  } else {
        std::cout << "Señal incorrecta recibida desde robot1" << std::endl;
  }



//Cerrar el socket cliente
  close(sockfd);



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
