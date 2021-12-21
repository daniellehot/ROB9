# ROB9 semester project at Aalborg University: Task-oriented handover using task-agnostic grasping and affordance segmentation
##### Group members: Albert Christensen, Daniel Lehotsky and Marius Jørgensen
##### Supervisor: Dimitris Chrysostomou

This repository contains the implementation of the ROB9 semester project rTask-oriented handover using task-agnostic grasping and affordance segmentation's implementation at Aalborg University.

## Installation instructions

### Pre-requisites:

0. OS: Ubuntu 18.04

1. Nvidia driver: 470.57.02
	 CUDA Version:  11.4

2. Install docker and nvidia-docker, good guide here: https://medium.com/@linhlinhle997/how-to-install-docker-and-nvidia-docker-2-0-on-ubuntu-18-04-da3eac6ec494

3. Install ros-melodic (from here: http://wiki.ros.org/melodic/Installation/Ubuntu)

4. Additional dependencies:
```
sudo apt install gcc-6 g++-6
```

For a full explanation of the system please see the project report.


### Installation

1. Pull docker image(s)
```
docker pull huchiewuchie/affordancenet-rob9
docker pull huchiewuchie/graspnet-rob9
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

In order to use this system, several ros nodes must be started. One complete launch file is on the feature list.

#### 1. Basics
```
roslaunch rob9 arm_bringup.launch
rosrun grasp_aff_association main.py
rosrun realsense_service server.py
rosrun speech recognition.py
rosrun rob9 tf2_service.py
rosrun rob9 moveit_service.py
rosrun rob9 webcam_publisher.py
rosrun moveit_scripts trajectory_service_ik.py
```

#### 2. Launch affordance analyzer in docker, see docker/Readme.md for more information

##### Start docker container:
```
docker run --name affordancenet -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/affordancenet-rob9 /bin/bash
```

##### Run ros node:
```
rosrun rob9 affordance_analyzer server.py
```


#### 3. Launch grasp_generator in docker, see docker/Readme.md for more information

##### Start docker container:
```
docker run --name graspnet-ros -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/graspnet-rob9 /bin/bash
```

##### Run ros node:
```
rosrun grasp_generator server.py
```

#### 4. Run experiment or main execution module:

##### a. Affordance object detection experiment test
```
rosrun rob9 test_affordance.py
```

##### b. Task-agnostic grasp experiment test
```
rosrun rob9 test_grasp.py
```

##### c. Task-oriented grasp experiment test
```
rosrun rob9 test_grasp_affordance.py
```

##### d. Main execution module including speech initiation
```
rosrun moveit_scripts execute_viz.py
```


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
