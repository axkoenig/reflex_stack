# Reflex Stack

[![Build Status](https://travis-ci.com/axkoenig/reflex_stack.svg?token=KeJradpJgXCJqZfQ8pwB&branch=main)](https://travis-ci.com/axkoenig/reflex_stack)

<img src="docs/screenshot.png"/>

## System Architecture 
This repository contains a simulator for Reflex TakkTile robotic hand and the Reflex Interface, an accompanying C++ library. 

**Reflex Simulator**: The idea behind the Reflex simulator is that you swap out the simulator for the real hand without any hassle. To achieve this, the simulator uses the same ROS topics and message definitions as the real hand does. We model some features of the Reflex, such as the tactile sensors and the underactuated distal flexure. 

**Reflex Interface (RI)**: The RI consists of two parts. The *State* module stores the most up to date state information of the Reflex and calculates various useful metrics for grasp analysis (e.g. grasp wrench space, epsilon, grasp matrix, slip prediction, ...). The *Command* module offers high-level control of the robotic hand via ROS services. Further, the RI offers a Keyboard Teleoperation Node for manual control of the robotic hand and the wrist (useful for debugging). The RI will also work on the real hand, although with a reduced feature set (since less information is available).

<img src="docs/system_design.png"/>

## Installation

### Option 1: Using Docker

**Build the Container**: If you simply want to try this software out you can download the pre-built image from Dockerhub and skip this step. If you want to modify or extend this software you must build it yourself with the included Dockerfile (this may take a while ...). 

```bash 
git clone --recursive https://github.com/axkoenig/reflex_stack.git
cd reflex_stack
docker build -t axkoenig/reflex_stack .
```

**Running the Container** 

Check if everything works by running the container and shelling into it. 
```bash
docker run --name sim -it --rm axkoenig/reflex_stack    # start simulation container
docker exec -it sim bash -l                             # shell into container from a new terminal
rostopic echo /reflex_interface/hand_state              # check if everything works
```

If you want to run multiple simulations on one computer just make sure that no ports overlap. You can specifiy the ports like this.  
```bash
# first simulation 
docker run --env ROS_MASTER_URI=http://localhost:11311 --env GAZEBO_MASTER_URI=http://localhost:11321 --name sim_1 -it --rm axkoenig/reflex_stack
# second simulation 
docker run --env ROS_MASTER_URI=http://localhost:11312 --env GAZEBO_MASTER_URI=http://localhost:11322 --name sim_2 -it --rm axkoenig/reflex_stack
```

**Visualizing Simulation**
If you want to work with the Gazebo GUI follow these steps and you can view it in your browser.
```bash
docker-compose up
localhost:8080/vnc.html                 # type this in your browser 
```
<img src="docs/docker.png"/>

Note that if you have gzclient installed locally you can follow [this](https://registry.hub.docker.com/_/gazebo) tutorial and check [this](https://www.youtube.com/watch?v=P__phnA57LM) video (which will probably give you a smoother rendering). If you are running the container on Ubuntu check out [this](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) tutorial for hardware acceleration. 

**Interacting with Simulation**
If you want to teleoperate the robotic hand you can fire up the keyboard teleoperation node in a separate terminal. 

```bash
docker exec -it sim bash -l
rosrun reflex_interface finger_teleop_node
```

### Option 2: Using plain Ubuntu
0. Disclaimer: the below steps assume you have a fresh installation of Ubuntu 20.04.
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

## Keyboard Teleoperation

Here are all commands you can do using your keyboard. You can change the key bindings in the [keyboard_teleop.cpp](reflex_interface/src/reflex_interface/keyboard_teleop.cpp) file. You can also use they keyboard teleoperation node on the real hand. Some commands (such as spherical close) are disabled on the real hand as fingers would crash into each other.

| Key | Effect              |
|-----|---------------------|
| u/U | wrist x +/-         |
| i/I | wrist y +/-         |
| o/O | wrist z +/-         |
| j/J | wrist roll +/-      |
| k/K | wrist pitch +/-     |
| l/L | wrist yaw +/-       |
| m   | reset wrist pose    |
| q/Q | finger 1 +/-        |
| w/W | finger 2 +/-        |
| e/E | finger 3 +/-        |
| r/R | preshape +/-        |
| y   | pinch               |
| x   | open                |
| c   | close               |
| v   | spherical open      |
| b   | spherical close     |
| n   | close until contact |
| t   | tighten grasp       |

## Acknowledgements

- The robot description package was initially based on the ```ll4ma_robots_description``` package by the [Utah Learning Lab for Manipulation Autonomy](https://bitbucket.org/robot-learning/ll4ma_robots_description/src/main/).
- The keyboard teleoperation node contains some code for non-blocking keyboard input from [teleop_twist_keyboard.cpp](https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp). 