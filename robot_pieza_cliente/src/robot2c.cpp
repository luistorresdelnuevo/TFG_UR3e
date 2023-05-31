#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h> 
#include <unistd.h>

#define SERVER_ADDRESS  "192.168.10.118"     /* server IP */
#define PORT            8080 

/* Test sequences */
char buf_tx[] = "ready";      
char buf_rx[100];

int main() 
{ 
    int sockfd; 
    struct sockaddr_in servaddr; 
    
    /* Socket creation */
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) 
    { 
        printf("CLIENT: socket creation failed...\n"); 
        return -1;  
    } 
    else
    {
        printf("CLIENT: Socket successfully created..\n"); 
    }
    
    
    memset(&servaddr, 0, sizeof(servaddr));

    /* assign IP, PORT */
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr( SERVER_ADDRESS ); 
    servaddr.sin_port = htons(PORT); 
  
    /* try to connect the client socket to server socket */
    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) 
    { 
        printf("connection with the server failed...\n");  
        return -1;
    } 
    
    printf("connected to the server..\n"); 
  
    /* send test sequences*/
    write(sockfd, buf_tx, sizeof(buf_tx));//Escribe en servidor
    read(sockfd, buf_rx, sizeof(buf_rx));
    if (signal == "done") {
        std::cout << "Señal recibida desde robot1: done" << std::endl;
        //Realizar la tarea del pick'n place del robot2

        std::cout << "Realizar la tarea robot2" << std::endl;
        // Realizar las tareas de pick'n place de robot1
        //move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("home"));
        //bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //move_group_interface.move();
    }


    printf("CLIENT:Received: %s \n", buf_rx);
   
       
    /* close the socket */
    close(sockfd); 
} 