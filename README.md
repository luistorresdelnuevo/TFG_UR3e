# TFG_UR3e
En este repositorio voy a subir mi fase final para el TFG


********************RETO 1********************
(Configurar en el polyscope y en el PC con las IP adecuadas)
Paquete 1 TFG pick_and_place_piezas
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.20.35 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
EXTERNAL CONTROL PLAY
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.20.35 launch_rviz:=true
ros2 launch pick_and_place_piezas pick_and_place_piezas.launch.py






********************RETO 2********************
PAQUETE 1 robot1_dualrobots 
Este paquete se ejecuta en el ordenador 1 (Configurar en el polyscope y en el PC con las IP adecuadas)
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.58.34 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
EXTERNAL CONTROL PLAY
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.58.34 launch_rviz:=true
ros2 launch robot1_dualrobots robot1_dualrobots.launch.py



PAQUETE 2 robot2_dualrobots
Este paquete se ejecuta en el ordendador 2 (Configurar en el polyscope y en el PC con las IP adecuadas)
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.20.35 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
EXTERNAL CONTROL PLAY
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.20.35 launch_rviz:=true
ros2 launch robot2_dualrobots robot2_dualrobots.launch.py


