# warehouse_simulation
This is WareHouse simulation, we created in e-Yantra Robotics Competition. This project consists of simulation of two robotic arms UR5_1 and UR5_2 in warehouse environment. 

### <p align='center'> TeamID: VB#2109 </p>
### <p align='center'> Participants: </p>
<p align='center'> Kunal Sagar </p>
<p align='center'> Khushi Agarwal </p>
<p align='center'> Hitesh Bhadauria</p>
<p align='center'> Unnati Gupta </p>

##### Environment configurations were provided by e-Yantra Team (IIT Bombay) themselves. We were to understand those to fit our motion planning and trajectory files appropriately. 
Their GitHub Repo for the same is:

<p align='center'>  https://github.com/e-yantra-r3/vb_simulation_pkgs </p>



#### <p align='center'>  We have used: </p>
1) Concepts of ROS 
2) Concepts of IoT - Network Protocols - sending and recieving data via HTTP and MQTT with python
3) MoveIt motion planning (which is an open source motion planning framework for Robotic Simulation).
4) TF (for converting co-ordinates from one frame of reference to another - at later stages did this manually too)
5) Google App Scripting
6) Javascript, HTML, CSS for making web interface for placing orders and displaying dashboard




### <p align='center'> YouTube Playlist of submissions: </p>
<p align='center'> https://www.youtube.com/playlist?list=PLb4kh-jYhf9Fkk-73hKoJjhy-sX4AsMBx </p>


## To run the simulation in your local environment follow these steps:

1) install requirements as listed in Task0/readme.md

2) open a terminal and cd to src folder of catkin workspace
    (path of current directory should be like: home/catkin_ws/src)
    
3) clone this repository at this location by using following command:
    git clone https://github.com/hi-18-K/warehouse_simulation.git

4) run following commands:
  i) catkin build
  ii) source devel/setup.bash
  iii) roslaunch pkg_task4 task4_solution.launch
       (where pkg_task4 can be replaced by name of any package which contains launch-file to run and task4_solution.launch can be replaced by name of launch file required to be executed)
