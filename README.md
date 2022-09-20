# Inventory Simulation: Vargi Bots(e-YRC 2020-21)
This is a inventory simulation, we created in e-Yantra Robotics Competition. This project consists of simulation of two robotic arms UR5_1, UR5_2 along with conveyor belt in warehouse environment. We simulate working of robotic arms according to predefined set of rules. 

### <p align='center'> TeamID: VB#2109 </p>

#### Environment configurations were provided by e-YRC Team. Github: 
##### https://github.com/e-yantra-r3/vb_simulation_pkgs 

#### Out task: 
1) To understand and work on the simulation in Gazebo.
2) Create and use trajectory files to perform reqired movement of the robotics arm.
3) Use problem solving skills to prioritise orders for maximum score.
4) Reduce overall simulation time. 



#### We have used:
1) Concepts of ROS 
2) Concepts of IoT - Network Protocols - sending and recieving data via HTTP and MQTT with python
3) MoveIt motion planning (which is an open source motion planning framework for Robotic Simulation).
4) TF (for converting co-ordinates from one frame of reference to another - at later stages did this manually too)
5) Google App Scripting
6) Javascript, HTML, CSS for making web interface for placing orders and displaying dashboard




### YouTube Playlist of submissions:
https://www.youtube.com/playlist?list=PLb4kh-jYhf9Fkk-73hKoJjhy-sX4AsMBx 


### To run the simulation in your local environment follow these steps: 

1) install requirements as listed in Task0/readme.md

2) open a terminal and cd to src folder of catkin workspace
    (path of current directory should be like: home/catkin_ws/src)
    
3) clone this repository at this location by using following command:
    
    git clone https://github.com/hi-18-K/warehouse_simulation.git

4) run following commands:
  - catkin build
  - source devel/setup.bash
  - roslaunch pkg_task5 task5_solution.launch
    
    (where pkg_task5 can be replaced by name of any package which contains launch-file to run and task5_solution.launch can be replaced by name of launch file required to be executed)
