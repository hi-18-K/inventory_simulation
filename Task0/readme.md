# <p align='center'> Task0 </p>

This is task0 submission(0th out of 0-6) of tasks to be submitted for e-YRC 2020-21 - VargiBots.

### In this task we had to:
#### 1) install required softwares: 
- Ubuntu 18.04
- ROS Melodic
- Gazebo
- e-Yantra App (Android)
#### 2) Set-up piazza account for further queries, doubts and other communication
#### 3) Get started with learning ROS
#### 4) Move turtle inside the turtlesim window in a circle and stop at its initial location - using concepts of ROS Node.
#### 5) Screen record the output(how turtle is moving in turtlesim), record bagfiles and submit it.

--------------


## <p align='center'> To run Upcoming tasks in your environment: </p>

#### 1) Install ubuntu (18.04) - or latest Ubuntu version
#### 2) Install ROS Melodic by following given steps:

open your terminal and paste- 

    - sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    - sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    - sudo apt update
    - sudo apt install ros-melodic-desktop-full
  
 Now configuration steps -
 
    - echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    - source ~/.bashrc
    - sudo apt install python-rosdep
    - sudo rosdep init
    - rosdep update
   
Install additional tools -

    - sudo apt-get install ros-melodic-catkin python-catkin-tools
    - sudo apt install ros-melodic-std-msgs
    - sudo apt-get install ros-melodic-ros-tutorials

#### 3) Configure catkin_ws
    - cd ~/
    - mkdir --parents catkin_ws/src
    - cd catkin_ws
    - catkin init
    - cd ~/catkin_ws
    - catkin build
    - source ~/catkin_ws/devel/setup.bash
    - gedit ~/.bashrc
    - Add to the end: source ~/catkin_ws/devel/setup.bash
    - Save and close the editor.
    


#### 4) Run 'git pull https://github.com/e-yantra-r3/vb_simulation_pkgs.git' in src folder of your catkin_ws
#### 5) Create new folder 'vb_tasks' inside src folder of catkin_ws
#### 6) Run 'git clone https://github.com/hi-18-K/warehouse_simulation.git' in vb_tasks folder
#### 7) extract files
#### 8) Run 'catkin build' (make sure you are in catkin_ws in terminal when giving this command)
#### 9) Run 'source devel/setup.bash' (make sure your path is <path_to_catkin_ws>/catkin_ws/~)
#### 10) Run 'roslaunch <pkg_name> <launch_file_name>'

----------------

Having doubt? Wanna know more? you can contact me: 

<p align='center'>  
<a href="https://www.linkedin.com/in/khushiagarwal/" target="_blank"><img height="30" src="https://raw.githubusercontent.com/peterthehan/peterthehan/master/assets/linkedin.svg?raw=true"></a>&nbsp;&nbsp;
<a href="https://www.instagram.com/khushiagarwal846/" target="_blank"><img height="30" src="https://media.giphy.com/media/SwyH7oWi2vhkOjCwiJ/giphy.gif?raw=true"></a>&nbsp;&nbsp;
<a href="https://www.facebook.com/profile.php?id=100055184105814" target="_blank"><img height="30" src="https://raw.githubusercontent.com/peterthehan/peterthehan/master/assets/facebook.svg?raw=true"></a>&nbsp;&nbsp;
</p>

