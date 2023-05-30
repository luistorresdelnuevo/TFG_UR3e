#include <memory>
#include<iostream>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>5
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

//Sockets
#include <netdb.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <arpa/inet.h>

/* strings / errors*/
#include <errno.h>
#include <stdio.h> 
#include <string.h> 


//server parameters:
#define SERV_PORT 8080
#define SERV_HOST_ADDR "192.168.10.118"
#define BUF_SIZE 100
#define BACKLOG 5


using namespace std;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot1",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );


//Server pa
 

  // Create the MoveIt MoveGroup Interface
  //using moveit::planning_interface::MoveGroupInterface;
  //auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  


    int sockfd, connfd ;  /* listening socket and connection socket file descriptors */
    unsigned int len;     /* length of client address */
    struct sockaddr_in servaddr, client; 
    
    int  len_rx, len_tx = 0;                     /* received and sent length, in bytes */
    char buff_tx[BUF_SIZE] = "done";
    char buff_rx[BUF_SIZE];   /* buffers for reception  */
    
     
    /* socket creation */
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) 
    { 
        fprintf(stderr, "[SERVER-error]: socket creation failed. %d: %s \n", errno, strerror( errno ));
        return -1;
    } 
    else
    {
        printf("[SERVER]: Socket successfully created..\n"); 
    }
    
    /* clear structure */
    memset(&servaddr, 0, sizeof(servaddr));
  
    /* assign IP, SERV_PORT, IPV4 */
    servaddr.sin_family      = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr(SERV_HOST_ADDR); 
    servaddr.sin_port        = htons(SERV_PORT); 
    
    
    /* Bind socket */
    if ((bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) != 0) 
    { 
        fprintf(stderr, "[SERVER-error]: socket bind failed. %d: %s \n", errno, strerror( errno ));
        return -1;
    } 
    else
    {
        printf("[SERVER]: Socket successfully binded \n");
    }




 /* Listen */
    if ((listen(sockfd, BACKLOG)) != 0) 
    { 
        fprintf(stderr, "[SERVER-error]: socket listen failed. %d: %s \n", errno, strerror( errno ));
        return -1;
    } 
    else
    {
        printf("[SERVER]: Listening on SERV_PORT %d \n\n", ntohs(servaddr.sin_port) ); 
    }
    
    len = sizeof(client); 


 /* Accept the data from incoming sockets in a iterative way */
      while(1)
      {
        connfd = accept(sockfd, (struct sockaddr *)&client, &len); 
        if (connfd < 0) 
        { 
            fprintf(stderr, "[SERVER-error]: connection not accepted. %d: %s \n", errno, strerror( errno ));
            return -1;
        } 
        else
        {              
            while(1) /* read data from a client socket till it is closed */ 
            {  
                /* read client message, copy it into buffer */
                len_rx = read(connfd, buff_rx, sizeof(buff_rx));  
                
                if(len_rx == -1)
                {
                    fprintf(stderr, "[SERVER-error]: connfd cannot be read. %d: %s \n", errno, strerror( errno ));
                }
                else if(len_rx == 0) /* if length is 0 client socket closed, then exit */
                {
                    printf("[SERVER]: client socket closed \n\n");
                    close(connfd);
                    break; 
                }
                else
                {
                    std::cout << "Señal recibida de robot2: ready" << std::endl;
                    if(buff_rx=="ready")
                    {
                      //move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
                     //bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                     //move_group_interface.move();
                    }
                    
                    write(connfd, buff_tx, strlen(buff_tx));
                    std::cout << "Señal enviada a robot2: done" << std::endl;
                    printf("[SERVER]: %s \n", buff_rx);
                }            
            }  
        }                      
    }    




// Escuchar por conexiones entrantes

  std::cout << "Escuchando conexiones entrantes" << std::endl;
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
          //move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
          //bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          //move_group_interface.move();

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

