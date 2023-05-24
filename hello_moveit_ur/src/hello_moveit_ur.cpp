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
    "hello_moveit_ur",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  


  int serverSocket,newSocket;
  struct sockaddr_in serverAddr{};
  struct sockaddr_storage serverStorage{};
  socklen_t addr_size;


//Crear el Socket servidor
  serverSocket = socket(AF_INET, SOCK_STREAM, 0);


//Configurar la dirección del servidor
  serverAddr.sin_family=AF_INET;
  serverAddr.sin_port = htons(8080);
  serverAddr.sin_addr.s_addr = inet_addr("192.168.10.118");

// Enlazar el socket a la dirección del servidor
  bind(serverSocket, (struct sockaddr*) &serverAddr, sizeof(serverAddr));

// Escuchar por conexiones entrantes
  listen(serverSocket, 1);

// Aceptar la conexión entrante
    addr_size = sizeof(serverStorage);
    newSocket = accept(serverSocket, (struct sockaddr*) &serverStorage, &addr_size);
    std::cout << "Conexión establecida desde robot2" << std::endl;

    switch(fork()) 
    {
      case -1:
      perror("echo server");
      return 1;
      
      case 0:
      char buffer[1024];
      recv(newSocket, buffer, 1024, 0);
      std::string signal = buffer;

      if (signal == "ready") 
      {
        std::cout << "Señal recibida de robot2: ready" << std::endl;

        // Realizar las tareas de pick'n place de robot1
          move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
          bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          move_group_interface.move();

        // Enviar señal de finalización a robot2
        std::string finishSignal = "done";
        send(newSocket, finishSignal.c_str(), finishSignal.length(), 0);
        std::cout << "Señal enviada a robot2: done" << std::endl;
      }
    } /* switch */

    close(newSocket);
    close(serverSocket);
    std::cout << "Fin del programa en robot1" << std::endl;

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

