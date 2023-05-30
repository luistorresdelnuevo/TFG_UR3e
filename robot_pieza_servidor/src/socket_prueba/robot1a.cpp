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
    "robot1",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

 

  // Create the MoveIt MoveGroup Interface
  //using moveit::planning_interface::MoveGroupInterface;
  //auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  




  int serverSocket;//Socket de conexion
  int newSocket; //Socket de datos
  struct sockaddr_in local{}; //Dirección local
  struct sockaddr_in cliente{};//Dirección del cliente
  unsigned int l_cliente; //tamano de la estructura anterior
  unsigned int l_cliente_size;
  char buffer[81]; //Recibe datos
  int leidos;//Numero de datos recibidos.


//Crear el Socket servidor
  serverSocket = socket(AF_INET, SOCK_STREAM, 0);//AF O PF

  if(serverSocket<0){
    perror("creando socket:");
    exit(1);
  }


//DIRECCOPM AL SOCKET
  local.sin_family=AF_INET;
  local.sin_port = htons(8080);
  local.sin_addr.s_addr = inet_addr("127.0.0.1"); //IP DEL ORDENADOR SERVIDOR
  if (bind(serverSocket, (struct sockaddr *) &local, sizeof(local))<0) 
  {
    perror("asignando direccion:");
    exit(2);
  }

// Escuchar por conexiones entrantes
  std::cout << "Escuchando conexiones entrantes" << std::endl;
  listen(serverSocket, 1);

//Esperando nueva conexion
    l_cliente_size = sizeof(cliente);
    newSocket= accept(serverSocket, (struct sockaddr *) &cliente, &l_cliente_size);
    if(newSocket<0)
    {
      perror("aceptando:");
    }
    else{
      memset(buffer, 0, sizeof(buffer));//inicializar buffer a cero
      std::string signal = buffer;
      //Leemos datos, si cliente es cero es por que ha cerrado la conexión
      if (signal == "ready") 
      {
        std::cout << "Señal recibida de robot2: ready" << std::endl;

        // Realizar las tareas de pick'n place de robot1
          //move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
          //bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          //move_group_interface.move();

        // Enviar señal de finalización a robot2
        std::string finishSignal = "done";
        send(newSocket, finishSignal.c_str(), finishSignal.length(), 0);
        std::cout << "Señal enviada a robot2: done" << std::endl;
        //Volver a poner el nuffer a ceros
        memset(buffer, 0, sizeof(buffer));
      }
    }

    close(newSocket);
    close(serverSocket);
    std::cout << "Fin del programa en robot1" << std::endl;

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

