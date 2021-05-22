# Reflex Stack

[![Build Status](https://travis-ci.com/axkoenig/reflex_stack.svg?token=KeJradpJgXCJqZfQ8pwB&branch=main)](https://travis-ci.com/axkoenig/reflex_stack)

<img src="docs/screenshot.png"/>

## System Architecture 
This repository contains a simulator for Reflex TakkTile robotic hand and the Reflex Interface, an accompanying C++ library. 

**Reflex Simulator**: The idea behind the Reflex simulator is that you swap out the simulator for the real hand without any hassle. To achieve this, the simulator uses the same ROS topics and message definitions as the real hand does. We model some features of the Reflex, such as the tactile sensors and the underactuated distal flexure. 

**Reflex Interface (RI)**: The RI consists of two parts. The *State* module stores the most up to date state information of the Reflex and calculates various useful metrics for grasp analysis (e.g. grasp wrench space, epsilon, grasp matrix, slip prediction, ...). The *Command* module offers high-level control of the robotic hand via ROS services. Further, the RI offers a Keyboard Teleoperation Node for manual control of the robotic hand and the wrist (useful for debugging). The RI will also work on the real hand, although with a reduced feature set (since less information is available).

<img src="docs/system_design.png"/>

## Installation

### Using Docker

**Setup Option 1**: If you simply want to try this software out you can download the pre-built image from Dockerhub and get going directly. 
```bash 
```

**Setup Option 2**: If you want to modify or extend this software you should build it yourself with the included Dockerfile (this may take a while ...). 

```bash 
git clone --recursive https://github.com/axkoenig/reflex_stack.git
cd reflex_stack
docker build -t reflex_stack .
```

**Running the Container** 

Check if everything works by starting the running the container and shelling into it. 
```bash
docker run --name sim -it reflex_stack      # start simulation container
docker exec -it sim bash -l                 # shell into container from a new terminal
rostopic echo /reflex_interface/hand_state  # check if everything works
```

If you want to run multiple simulations on one computer just make sure that you have no ports overlap. You can specifiy the ports like this.  
```bash
# first simulation 
docker run --name sim_1 -it reflex_stack --env ROS_MASTER_URI=http://localhost:11311 --env GAZEBO_MASTER_URI=http://localhost:11321 
# second simulation 
docker run --name sim_2 -it reflex_stack --env ROS_MASTER_URI=http://localhost:11312 --env GAZEBO_MASTER_URI=http://localhost:11322 
```

### Using plain Ubuntu
0. Disclaimer: the below steps assume you have a fresh installation of Ubuntu 20.04. You may want to run all this in a Docker container to avoid version conflicts. 
1. Install ROS Noetic by following [these](http://wiki.ros.org/noetic/Installation/Ubuntu) steps.
2. Clone this repository into a new catkin workspace.

```bash 
# Init new catkin workspace
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
# Clone this repository with its submodules
git clone --recursive https://github.com/axkoenig/reflex_stack.git
```

3. The Reflex Stack was built and tested using Gazebo 11 and DART 6. To run Gazebo with the DART physics engine, you must build Gazebo from source. Running the shell script does this for you. 
```bash 
cd ~/catkin_ws/src/reflex_stack/shell
sudo ./install_gazebo_dart.sh
```
4. Now that you have all the required dependencies you can install the Reflex Stack. 
```bash 
# Build Reflex Stack 
cd ~/catkin_ws
catkin_make
# Source workspace and add to your bashrc
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
5. Check if everything works by firing up the simulation in a new terminal.
```bash 
roslaunch description reflex.launch run_keyboard_teleop_nodes:=true
```

## Acknowledgements

- The robot description package was initially based on the ```ll4ma_robots_description``` package by the [Utah Learning Lab for Manipulation Autonomy](https://bitbucket.org/robot-learning/ll4ma_robots_description/src/main/).
- The keyboard teleoperation node contains some code for non-blocking keyboard input from [teleop_twist_keyboard.cpp](https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp). 