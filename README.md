## STILL WORK IN PROGRESS
# ROB9 semester project: Robot-to-human affordance handover
##### Albert Christensen, Daniel Lehotsky and Marius Jørgensen

This repository contains the implementation of the ROB9 semester project robot-to-human affordance handover's implementation at Aalborg University


## Installation instructions

### Pre-requisites:

0. OS: Ubuntu 18.04

1. Nvidia driver: 470.57.02
	 CUDA Version:  11.4

2. Install docker and nvidia-docker, good guide here: https://medium.com/@linhlinhle997/how-to-install-docker-and-nvidia-docker-2-0-on-ubuntu-18-04-da3eac6ec494

3. Install ros-melodic (from here: http://wiki.ros.org/melodic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

4. Additional dependencies:
```
sudo apt install gcc-6 g++-6
```

### Installation

1. Pull docker image(s)
```
docker pull huchiewuchie/affordancenet-ros
```

2. Setup catkin workspace
```
cd
mkdir rob9_ws && mkdir rob9_ws/src
cd rob9_ws/src
git clone https://bitbucket.org/masterrob/lh7-handover/src/main/
git clone https://github.com/daniellehot/ROB9.git
cd ..
catkin_make -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4 -DCMAKE_C_COMPILER=gcc-6 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### Usage

To do.

### Additional information
Relevant IP addresses:

- Robot - 192.168.1.106
- Gripper - 192.168.1.140
- PTU Tilt-pan - 192.168.1.110


If you get nvcc fatal error, follow this guide - https://github.com/leggedrobotics/darknet_ros#building

Connection guides
UR5
Robotiq - http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%203-Finger%20Gripper%20using%20the%20Modbus%20TCP%20Protocol
PTU Tilt-pan


roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.106 kinematics_config:=${HOME}/ur10_example_calibration.yaml

### Acknowledgements

Dimitris Chrysostomou - Supervisor
Jan Kjær Jørgensen - Previous group
Rune Grønhøj - Previous group
